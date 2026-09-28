"""Detect all circular RGB targets in the calibrated holder plane."""

from dataclasses import dataclass
from typing import Dict, List, Tuple

import cv2
import numpy as np

from denso_vision.holder_geometry import transform_pixel


@dataclass(frozen=True)
class Target:
    color: str
    pixel_x: float
    pixel_y: float
    radius_px: float
    holder_x: float
    holder_y: float


DEFAULT_HSV = {
    'red': ((0, 100, 50), (10, 255, 255), (170, 100, 50), (179, 255, 255)),
    'green': ((45, 80, 50), (85, 255, 255)),
    'blue': ((100, 100, 50), (130, 255, 255)),
}


def detect_targets(
    image: np.ndarray,
    homography: np.ndarray,
    roi: Tuple[float, float, float, float],
    min_radius: float,
    max_radius: float,
    hsv_ranges: Dict[str, tuple] = DEFAULT_HSV,
    min_circularity: float = 0.78,
) -> List[Target]:
    """Return circles fully inside the plane ROI; image is BGR."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    x_min, x_max, y_min, y_max = roi
    found = []
    for color, bounds in hsv_ranges.items():
        mask = cv2.inRange(hsv, np.array(bounds[0]), np.array(bounds[1]))
        if len(bounds) == 4:
            mask |= cv2.inRange(hsv, np.array(bounds[2]), np.array(bounds[3]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if not min_radius <= radius <= max_radius:
                continue
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0 or 4 * np.pi * area / perimeter ** 2 < min_circularity:
                continue
            # Test the perimeter, not just the centre: partial targets are unsafe.
            rim = np.array([
                [x + radius * np.cos(angle), y + radius * np.sin(angle)]
                for angle in np.linspace(0, 2 * np.pi, 16, endpoint=False)
            ], dtype=np.float32)
            if np.any(rim[:, 0] < 0) or np.any(rim[:, 0] >= image.shape[1]) or \
                    np.any(rim[:, 1] < 0) or np.any(rim[:, 1] >= image.shape[0]):
                continue
            plane_rim = cv2.perspectiveTransform(rim.reshape(1, -1, 2), homography).reshape(-1, 2)
            if np.any(plane_rim[:, 0] < x_min) or np.any(plane_rim[:, 0] > x_max) or \
                    np.any(plane_rim[:, 1] < y_min) or np.any(plane_rim[:, 1] > y_max):
                continue
            plane = transform_pixel(homography, np.array([x, y]))
            found.append(Target(color, x, y, radius, float(plane[0]), float(plane[1])))
    return sorted(found, key=lambda target: (target.pixel_y, target.pixel_x))


def validate_targets(targets: List[Target]) -> None:
    """Reject missing, overlapping, or imbalanced RGB target sets."""
    if not targets or len(targets) % 3:
        raise ValueError(f'RGB targets must form complete trios; detected {len(targets)} circles')
    trios = len(targets) // 3
    for color in ('red', 'green', 'blue'):
        count = sum(target.color == color for target in targets)
        if count != trios:
            raise ValueError(f'Expected {trios} {color} circles, detected {count}')
    for index, first in enumerate(targets):
        for second in targets[index + 1:]:
            if np.hypot(first.pixel_x - second.pixel_x, first.pixel_y - second.pixel_y) < \
                    first.radius_px + second.radius_px:
                raise ValueError('Detected circles overlap')


class StableTargets:
    def __init__(self, frames: int, tolerance_px: float):
        if frames < 1 or tolerance_px < 0:
            raise ValueError('Stable frames must be positive and tolerance nonnegative')
        self.frames = frames
        self.tolerance_px = tolerance_px
        self.previous = []
        self.count = 0

    def update(self, targets: List[Target]) -> bool:
        same = len(targets) == len(self.previous) and all(
            a.color == b.color and
            np.hypot(a.pixel_x - b.pixel_x, a.pixel_y - b.pixel_y) <= self.tolerance_px and
            abs(a.radius_px - b.radius_px) <= self.tolerance_px
            for a, b in zip(targets, self.previous)
        )
        self.count = self.count + 1 if same and targets else 1 if targets else 0
        self.previous = targets
        return self.count >= self.frames
