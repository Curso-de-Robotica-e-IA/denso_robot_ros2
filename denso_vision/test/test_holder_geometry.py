"""Tests for the measured cellphone-holder geometry."""

import numpy as np
import pytest

from denso_vision.holder_geometry import (
    HolderGeometry, image_tag_order, transform_pixel,
)


def test_measured_trapezoid_geometry():
    """The reconstructed centres preserve all four measurements."""
    geometry = HolderGeometry()
    centres = geometry.tag_centres()

    assert geometry.height == pytest.approx(0.2074872534)
    assert np.linalg.norm(centres[2] - centres[1]) == pytest.approx(0.0952)
    assert np.linalg.norm(centres[4] - centres[3]) == pytest.approx(0.0998)
    assert np.linalg.norm(centres[4] - centres[1]) == pytest.approx(0.2075)
    assert np.linalg.norm(centres[3] - centres[2]) == pytest.approx(0.2075)


def test_corner_order_and_marker_size():
    """All marker corners have the requested side length and ordering."""
    geometry = HolderGeometry()
    corners = geometry.tag_corners()[1]

    assert np.linalg.norm(corners[1] - corners[0]) == pytest.approx(0.0215)
    assert np.linalg.norm(corners[2] - corners[1]) == pytest.approx(0.0215)
    assert geometry.all_corners().shape == (16, 2)


def test_physical_tag_layout_and_detected_corner_order():
    """The physical lower-row IDs and label rotations match the holder."""
    geometry = HolderGeometry()
    centres = geometry.tag_centres()
    raw_corners = geometry.tag_corners()
    detector_corners = geometry.detector_corners()

    assert centres[4][0] < 0.0
    assert centres[3][0] > 0.0
    np.testing.assert_allclose(detector_corners[1][0], raw_corners[1][3])
    np.testing.assert_allclose(detector_corners[2][0], raw_corners[2][1])


def test_transform_pixel():
    """A homogeneous pixel transform returns Cartesian plane coordinates."""
    homography = np.array([
        [0.002, 0.0, -0.5],
        [0.0, -0.002, 0.4],
        [0.0, 0.0, 1.0],
    ])

    assert transform_pixel(homography, np.array([250.0, 200.0])) == \
        pytest.approx([0.0, 0.0])


def test_impossible_layout_is_rejected():
    """Contradictory side lengths are rejected explicitly."""
    geometry = HolderGeometry(
        top_spacing=0.01,
        bottom_spacing=1.0,
        side_spacing=0.1,
    )
    with pytest.raises(ValueError):
        _ = geometry.height


def test_portrait_image_tag_order():
    """The physical portrait holder reads 1 2 / 4 3 in the image."""
    centres = {
        1: np.array([10.0, 10.0]),
        2: np.array([90.0, 12.0]),
        4: np.array([12.0, 190.0]),
        3: np.array([88.0, 188.0]),
    }
    assert image_tag_order(centres) == (1, 2, 4, 3)


def test_counterclockwise_horizontal_image_tag_order():
    """A visual +90 degree rotation reads 2 3 / 1 4 in the image."""
    portrait = {
        1: np.array([-1.0, -2.0]),
        2: np.array([1.0, -2.0]),
        4: np.array([-1.0, 2.0]),
        3: np.array([1.0, 2.0]),
    }
    horizontal = {
        tag_id: np.array([point[1], -point[0]])
        for tag_id, point in portrait.items()
    }
    assert image_tag_order(horizontal) == (2, 3, 1, 4)
