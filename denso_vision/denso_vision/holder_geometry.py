"""Measured geometry and homography helpers for the cellphone holder."""

import math
from dataclasses import dataclass
from typing import Dict

import numpy as np


@dataclass(frozen=True)
class HolderGeometry:
    """Physical layout of the four coplanar tags, expressed in metres."""

    tag_size: float = 0.0215
    top_spacing: float = 0.0952
    bottom_spacing: float = 0.0998
    side_spacing: float = 0.2075

    @property
    def height(self) -> float:
        """Return trapezoid height implied by the four measured sides."""
        half_width_difference = (
            self.bottom_spacing - self.top_spacing
        ) / 2.0
        squared_height = self.side_spacing ** 2 - half_width_difference ** 2
        if squared_height <= 0.0:
            raise ValueError(
                'side_spacing is too short for the measured widths'
            )
        return math.sqrt(squared_height)

    def tag_centres(self) -> Dict[int, np.ndarray]:
        """Return tag centres in the holder plane frame.

        The origin is the layout centre, +X points right, and +Y points from
        the lower row (IDs 4/3) to the upper row (IDs 1/2). This ID layout
        matches the physical holder: IDs 1/2 are top-left/top-right and IDs
        4/3 are bottom-left/bottom-right.
        """
        half_height = self.height / 2.0
        return {
            1: np.array([-self.top_spacing / 2.0, half_height]),
            2: np.array([self.top_spacing / 2.0, half_height]),
            3: np.array([self.bottom_spacing / 2.0, -half_height]),
            4: np.array([-self.bottom_spacing / 2.0, -half_height]),
        }

    def tag_corners(self) -> Dict[int, np.ndarray]:
        """Return corners TL, TR, BR, BL for each identically oriented tag."""
        half_size = self.tag_size / 2.0
        offsets = np.array([
            [-half_size, half_size],
            [half_size, half_size],
            [half_size, -half_size],
            [-half_size, -half_size],
        ])
        return {
            tag_id: centre + offsets
            for tag_id, centre in self.tag_centres().items()
        }

    def all_corners(self) -> np.ndarray:
        """Return all 16 plane points, ordered by ID then TL, TR, BR, BL."""
        corners = self.tag_corners()
        return np.concatenate([corners[tag_id] for tag_id in (1, 2, 3, 4)])

    def detector_corners(self) -> Dict[int, np.ndarray]:
        """Return physical corners in OpenCV's detected-corner order.

        The physical labels have different rotations on the holder. OpenCV
        preserves each marker's local corner order, so the corresponding
        physical corners must be reordered before fitting one homography to
        all sixteen points. The mapping was validated against the recorded
        D405 footage with all four tags visible.
        """
        corners = self.tag_corners()
        orders = {
            1: [3, 0, 1, 2],
            2: [1, 2, 3, 0],
            3: [3, 0, 1, 2],
            4: [3, 0, 1, 2],
        }
        return {
            tag_id: corners[tag_id][orders[tag_id]]
            for tag_id in (1, 2, 3, 4)
        }


def transform_pixel(homography: np.ndarray, pixel: np.ndarray) -> np.ndarray:
    """Transform one image pixel into an XY coordinate on the holder plane."""
    pixel_homogeneous = np.array([pixel[0], pixel[1], 1.0], dtype=float)
    plane_homogeneous = np.asarray(homography) @ pixel_homogeneous
    scale = plane_homogeneous[2]
    if abs(scale) < 1e-12:
        raise ValueError('pixel maps to infinity under this homography')
    return plane_homogeneous[:2] / scale
