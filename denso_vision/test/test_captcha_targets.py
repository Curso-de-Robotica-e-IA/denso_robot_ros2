"""Unchecked checkboxes, not RGB trios or arbitrary square UI elements."""

from pathlib import Path

import cv2
import numpy as np
import pytest

from denso_vision.captcha_targets import CaptchaDetector
from denso_vision.colored_targets import StableTargets, validate_targets


ROOT = Path(__file__).resolve().parents[2]
SCREEN = ROOT / 'denso_robot_descriptions/tools/virtual_phone/meshes/phone_captcha_texture.png'
TEMPLATE = ROOT / 'denso_vision/assets/captcha_checkbox.png'
ROI = (-0.05, 0.05, -0.10, 0.085)
H = np.array([[0.0658 / 1080, 0, -0.0329], [0, -0.141 / 2400, 0.0705], [0, 0, 1]])


def test_original_app_screenshot_and_stability():
    detector = CaptchaDetector(TEMPLATE)
    targets = detector.detect(cv2.imread(str(SCREEN)), H, ROI)
    validate_targets(targets, 'captcha')
    assert len(targets) == 1
    assert targets[0].color == 'captcha'
    assert targets[0].pixel_x == pytest.approx(211.5, abs=4)
    assert targets[0].pixel_y == pytest.approx(1225.5, abs=4)
    stable = StableTargets(3, 3)
    assert not stable.update(targets)
    assert not stable.update(targets)
    assert stable.update(targets)


@pytest.mark.parametrize('rotation', (0, 90, 180, 270))
def test_perspective_and_phone_rotation(rotation):
    source = cv2.imread(str(SCREEN))
    corners = np.float32([[0, 0], [1080, 0], [1080, 2400], [0, 2400]])
    destination = np.float32([[220, 80], [380, 95], [398, 405], [207, 430]])
    warp = cv2.getPerspectiveTransform(corners, destination)
    if rotation:
        rotation_matrix = np.vstack([cv2.getRotationMatrix2D((300, 250), rotation, 1), [0, 0, 1]])
        warp = rotation_matrix @ warp
    frame = cv2.warpPerspective(source, warp, (600, 500), borderValue=(230, 230, 230))
    targets = CaptchaDetector(TEMPLATE).detect(frame, H @ np.linalg.inv(warp), ROI)
    assert len(targets) == 1
    assert targets[0].holder_x == pytest.approx(-0.0200, abs=0.0008)
    assert targets[0].holder_y == pytest.approx(-0.0015, abs=0.0008)


def test_multiple_checkboxes_and_no_fixed_target_count():
    source = cv2.imread(str(SCREEN))
    card = source[1128:1326, 141:937].copy()
    source[:] = 245
    for y in (400, 1050, 1700):
        source[y:y + card.shape[0], 141:141 + card.shape[1]] = card
    targets = CaptchaDetector(TEMPLATE).detect(source, H, ROI)
    validate_targets(targets, 'captcha')
    assert len(targets) == 3


def test_rejects_blank_rgb_and_unrelated_square():
    detector = CaptchaDetector(TEMPLATE)
    blank = np.full((2400, 1080, 3), 245, np.uint8)
    cv2.rectangle(blank, (175, 1190), (248, 1263), (75, 75, 75), 4)
    assert detector.detect(blank, H, ROI) == []
    rgb = cv2.imread(str(SCREEN.with_name('phone_screen_texture.png')))
    assert detector.detect(rgb, H, ROI) == []
    with pytest.raises(ValueError):
        validate_targets([], 'captcha')


def test_checked_partial_and_outside_roi_are_rejected():
    source = cv2.imread(str(SCREEN))
    checked = source.copy()
    cv2.line(checked, (188, 1223), (210, 1244), (40, 150, 40), 10)
    cv2.line(checked, (210, 1244), (239, 1203), (40, 150, 40), 10)
    detector = CaptchaDetector(TEMPLATE)
    assert detector.detect(checked, H, ROI) == []
    assert detector.detect(source[1210:], H @ np.array([[1, 0, 0], [0, 1, 1210], [0, 0, 1]]), ROI) == []
    assert detector.detect(source, H, (0, 0.05, -0.10, 0.085)) == []


@pytest.mark.parametrize('settings', ({'min_side_mm': float('nan')}, {'max_side_mm': 2},
                                    {'match_threshold': 0.1}))
def test_invalid_parameters(settings):
    with pytest.raises(ValueError):
        CaptchaDetector(TEMPLATE, **settings)


def test_invalid_roi_and_missing_template():
    with pytest.raises(ValueError, match='Cannot load'):
        CaptchaDetector(TEMPLATE.with_name('missing.png'))
    source = cv2.imread(str(SCREEN))
    for roi in ((0, 0, 0, 0.1), (0, 100, 0, 0.1), (0, float('nan'), 0, 0.1)):
        with pytest.raises(ValueError, match='ROI'):
            CaptchaDetector(TEMPLATE).detect(source, H, roi)
