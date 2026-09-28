"""Checks for dynamic RGB detection before any robot movement."""

import cv2
import numpy as np
import pytest
from dataclasses import replace

from denso_vision.colored_targets import StableTargets, detect_targets, validate_targets


COLORS = ((0, 0, 255), (0, 255, 0), (255, 0, 0))
ROI = (0.0, 800.0, 0.0, 600.0)


def frame(trios, radii=None, random_order=False):
    image = np.full((600, 800, 3), 245, np.uint8)
    positions = [(120 + 175 * column, 130 + 140 * row)
                 for row in range(3) for column in range(4)][:trios * 3]
    if random_order:
        positions = list(reversed(positions))
    for index, (x, y) in enumerate(positions):
        cv2.circle(image, (x, y), radii[index] if radii else 18, COLORS[index % 3], -1)
    return image


@pytest.mark.parametrize('trios', (1, 2, 3, 4))
def test_detects_all_rgb_targets(trios):
    image = frame(trios, random_order=True)
    # A coloured rectangular UI control must not be mistaken for a circle.
    cv2.rectangle(image, (700, 10), (790, 45), (0, 0, 255), -1)
    targets = detect_targets(image, np.eye(3), ROI, 6, 60)
    validate_targets(targets)
    assert len(targets) == trios * 3
    assert targets == sorted(targets, key=lambda item: (item.pixel_y, item.pixel_x))


def test_mixed_radii_and_stable_layout():
    targets = detect_targets(frame(3, [10, 15, 20, 25, 12, 18, 16, 22, 14]),
                             np.eye(3), ROI, 6, 60)
    validate_targets(targets)
    assert max(t.radius_px for t in targets) - min(t.radius_px for t in targets) > 12
    stable = StableTargets(3, 2)
    assert not stable.update(targets)
    assert not stable.update(targets)
    assert stable.update(targets)
    assert not stable.update(targets[:-1])


def test_missing_and_partial_targets_are_rejected():
    empty = np.full((600, 800, 3), 245, np.uint8)
    assert detect_targets(empty, np.eye(3), ROI, 6, 60) == []
    with pytest.raises(ValueError, match='detected 0 circles'):
        validate_targets([])
    image = frame(1)
    cv2.circle(image, (3, 330), 18, (0, 0, 255), -1)
    targets = detect_targets(image, np.eye(3), ROI, 6, 60)
    assert len(targets) == 3
    validate_targets(targets)


def test_removed_target_is_detected():
    image = frame(1)
    cv2.circle(image, (120, 130), 20, (245, 245, 245), -1)
    targets = detect_targets(image, np.eye(3), ROI, 6, 60)
    with pytest.raises(ValueError, match='detected 2 circles'):
        validate_targets(targets)


def test_missing_color_and_overlap_are_rejected():
    targets = detect_targets(frame(1), np.eye(3), ROI, 6, 60)
    with pytest.raises(ValueError, match='red'):
        validate_targets([targets[0], targets[0], targets[1]])
    with pytest.raises(ValueError, match='overlap'):
        validate_targets([targets[0], replace(targets[1], pixel_x=targets[0].pixel_x), targets[2]])
