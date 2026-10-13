"""Locate unchecked Google test checkboxes; reuse the holder homography."""

import cv2
import numpy as np

from denso_vision.colored_targets import Target
from denso_vision.holder_geometry import transform_pixel


class CaptchaDetector:
    # shortcut: Google's regular English widget layout; update the screenshot
    # template and offsets if Google changes it. No image challenges are parsed.
    CENTER = np.array([40.5, 39.5])
    SIDE = 73.5
    PIXELS_PER_METRE = 5000.0

    def __init__(self, template_path, min_side_mm=3.0, max_side_mm=9.0,
                 match_threshold=0.72):
        if not (np.isfinite([min_side_mm, max_side_mm, match_threshold]).all() and
                2 <= min_side_mm < max_side_mm <= 15 and
                0.6 <= match_threshold <= 1):
            raise ValueError('Invalid CAPTCHA size bounds or match threshold')
        template = cv2.imread(str(template_path), cv2.IMREAD_GRAYSCALE)
        if template is None:
            raise ValueError(f'Cannot load CAPTCHA template: {template_path}')
        self.threshold = match_threshold
        self.templates = []
        scale = self.PIXELS_PER_METRE / 1000
        for side in range(int(np.ceil(min_side_mm * scale)),
                          int(np.floor(max_side_mm * scale)) + 1):
            factor = side / self.SIDE
            resized = cv2.resize(template, None, fx=factor, fy=factor,
                                 interpolation=cv2.INTER_AREA)
            resized = cv2.GaussianBlur(resized, (7, 7), 1.2)
            self.templates.append((resized, self.CENTER * factor, side))

    def detect(self, image, homography, roi):
        x_min, x_max, y_min, y_max = roi
        if not (np.isfinite(roi).all() and 0 < x_max - x_min <= 0.30 and
                0 < y_max - y_min <= 0.40):
            raise ValueError('CAPTCHA ROI must fit a 300 x 400 mm holder area')
        scale = self.PIXELS_PER_METRE
        plane_to_rectified = np.array([
            [scale, 0, -x_min * scale],
            [0, -scale, y_max * scale],
            [0, 0, 1],
        ])
        image_to_rectified = plane_to_rectified @ homography
        rectified_to_image = np.linalg.inv(image_to_rectified)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        rectified = cv2.warpPerspective(
            gray, image_to_rectified,
            (int(np.ceil((x_max - x_min) * scale)),
             int(np.ceil((y_max - y_min) * scale))),
            borderValue=255,
        )
        # Match the low-resolution camera rendering, not screenshot antialiasing.
        rectified = cv2.GaussianBlur(rectified, (7, 7), 1.2)
        candidates = []
        for template, center_offset, side in self.templates:
            height, width = template.shape
            if height > rectified.shape[0] or width > rectified.shape[1]:
                continue
            scores = cv2.matchTemplate(rectified, template, cv2.TM_CCOEFF_NORMED)
            # Extract isolated peaks, not one target for every matching pixel/scale.
            peaks = (scores >= self.threshold) & (scores == cv2.dilate(scores, np.ones((3, 3), np.uint8)))
            for y, x in zip(*np.where(peaks)):
                center = np.array([x, y]) + center_offset
                half = side / 2
                corners = center + np.array([[-half, -half], [half, -half],
                                             [half, half], [-half, half]])
                rim = cv2.perspectiveTransform(corners.astype(np.float32)[None],
                                              rectified_to_image)[0]
                if (np.any(rim < 0) or np.any(rim[:, 0] >= image.shape[1]) or
                        np.any(rim[:, 1] >= image.shape[0])):
                    continue
                # Checked boxes must not become new touch targets.
                cx, cy = np.rint(center).astype(int)
                inner = rectified[cy - side // 4:cy + side // 4,
                                  cx - side // 4:cx + side // 4]
                if inner.size == 0 or inner.std() > 15:
                    continue
                pixel = transform_pixel(rectified_to_image, center)
                holder = transform_pixel(homography, pixel)
                radius = float(np.linalg.norm(rim - pixel, axis=1).mean() / np.sqrt(2))
                candidates.append((float(scores[y, x]), side / scale,
                                   Target('captcha', float(pixel[0]), float(pixel[1]),
                                          radius, float(holder[0]), float(holder[1]))))
        accepted = []
        for score, side_m, target in sorted(candidates, key=lambda item: item[0], reverse=True):
            if any(np.hypot(target.holder_x - other.holder_x,
                            target.holder_y - other.holder_y) < max(side_m, other_side)
                   for other_side, other in accepted):
                continue
            accepted.append((side_m, target))
        return sorted((target for _, target in accepted),
                      key=lambda item: (item.pixel_y, item.pixel_x))
