"""Algılamanın kataloğu (forklift_perception.core.catalog) ile simülasyonun yerleşim modülü
(forklift_gazebo/scripts/station_layout.py) aynı geometriyi veriyor mu.

İki paket bilerek birbirine bağımlı değil (gerçek robota simülasyon paketi kurulmaz, plan R8);
formüller iki yerde durduğu için bu test onları eşit tutar (DECISIONS D-63)."""

import os
import sys

import numpy as np
import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory

sys.path.insert(0, os.path.join(get_package_prefix('forklift_gazebo'), 'lib', 'forklift_gazebo'))
import station_layout as sl  # noqa: E402

from forklift_perception.core import catalog  # noqa: E402

PARAMS = os.path.join(get_package_share_directory('forklift_gazebo'), 'config', 'station_params.yaml')


@pytest.fixture(scope='module')
def both():
    return catalog.load(PARAMS), sl.load_yaml(PARAMS)


def test_alignment_targets(both):
    cat, P = both
    assert cat.target_offset == pytest.approx(sl.align_offset(P), abs=1e-12)
    assert cat.target_face_x == pytest.approx(sl.align_face_x(P), abs=1e-12)
    pts = sl.align_points(P, 3, 1)
    assert cat.target_z == pytest.approx(pts['left'][2], abs=1e-12)
    assert cat.target_baseline == pytest.approx(pts['left'][1] - pts['right'][1], abs=1e-12)


def test_bay_line(both):
    cat, P = both
    a, b = sl.bay_line(P, 3, 0)
    assert cat.bay_line_x == pytest.approx((a[0], b[0]), abs=1e-12)
    assert cat.plate_top_z == pytest.approx(a[2], abs=1e-12)


@pytest.mark.parametrize('n_bays', [3, 5])
def test_sign_full_width(both, n_bays):
    cat, P = both
    x, y0, y1, z0, z1 = sl.sign_rect(P, n_bays)
    assert cat.sign_face_x == pytest.approx(x, abs=1e-12)
    assert cat.sign_width == pytest.approx(y1 - y0, abs=1e-12)
    assert cat.sign_height == pytest.approx(z1 - z0, abs=1e-12)
    assert cat.sign_top_z == pytest.approx(z1, abs=1e-12)


def test_parcel_and_pitch(both):
    cat, P = both
    assert cat.bay_pitch == pytest.approx(sl.bay_y(P, 3, 1) - sl.bay_y(P, 3, 0), abs=1e-12)
    cx, _, cz = sl.parcel_center_in_bay(P)
    assert cat.parcel_front_gap == pytest.approx(cx - cat.parcel_size[0] / 2, abs=1e-12)
    assert cat.ledge_height + cat.parcel_size[2] / 2 == pytest.approx(cz, abs=1e-12)
    assert np.isclose(cat.seal_size, P['parcel']['seal']['size']['y'])
