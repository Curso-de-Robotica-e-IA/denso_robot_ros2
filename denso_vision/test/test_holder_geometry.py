"""Tests for the measured cellphone-holder geometry."""

import numpy as np
import pytest

from denso_vision.holder_geometry import HolderGeometry, transform_pixel


def test_measured_trapezoid_geometry():
    """The reconstructed centres preserve all four measurements."""
    geometry = HolderGeometry()
    centres = geometry.tag_centres()

    assert geometry.height == pytest.approx(0.2074872534)
    assert np.linalg.norm(centres[2] - centres[1]) == pytest.approx(0.0952)
    assert np.linalg.norm(centres[4] - centres[3]) == pytest.approx(0.0998)
    assert np.linalg.norm(centres[3] - centres[1]) == pytest.approx(0.2075)
    assert np.linalg.norm(centres[4] - centres[2]) == pytest.approx(0.2075)


def test_corner_order_and_marker_size():
    """All marker corners have the requested side length and ordering."""
    geometry = HolderGeometry()
    corners = geometry.tag_corners()[1]

    assert np.linalg.norm(corners[1] - corners[0]) == pytest.approx(0.0215)
    assert np.linalg.norm(corners[2] - corners[1]) == pytest.approx(0.0215)
    assert geometry.all_corners().shape == (16, 2)


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
