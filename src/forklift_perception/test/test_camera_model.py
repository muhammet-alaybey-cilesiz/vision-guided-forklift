"""CameraModel birim testleri: izdüşüm ↔ ters izdüşüm tutarlılığı ve bilinen geometri."""

import math

import numpy as np
import pytest

from forklift_perception.core.camera_model import CameraModel

# Şerit kamerasına benzer: 0.54 m yükseklik, 45° aşağı, HFOV 80°, 640 × 480
W, H, HFOV = 640, 480, math.radians(80)
F = (W / 2) / math.tan(HFOV / 2)
K = [[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]]


def R_base_opt(pitch):
    """Aşağı pitch açısıyla ileri bakan kameranın optik eksenleri (sütunlar) base'te."""
    c, s = math.cos(pitch), math.sin(pitch)
    z = np.array([c, 0.0, -s])            # ileri ve aşağı
    x = np.array([0.0, -1.0, 0.0])        # görüntüde sağ = robotun sağı
    y = np.cross(z, x)                    # görüntüde aşağı
    return np.column_stack([x, y, z])


def model(D=None):
    return CameraModel(K, D, W, H, R_base_opt(math.radians(45)), [0.10, 0.0, 0.54])


def test_center_pixel_hits_ground_at_camera_height():
    # 45° aşağı bakan kameranın merkez ışını zemine kameradan yükseklik kadar ileride değer
    pts, ok = model().pixels_to_plane([[W / 2, H / 2]], 0.0)
    assert ok[0]
    assert pts[0, 0] == pytest.approx(0.10 + 0.54, abs=1e-9)
    assert pts[0, 1] == pytest.approx(0.0, abs=1e-9)


def test_project_and_back_project_roundtrip():
    rng = np.random.default_rng(1)
    g = np.column_stack([rng.uniform(0.3, 2.0, 200), rng.uniform(-0.5, 0.5, 200), np.zeros(200)])
    m = model()
    uv, front, inside = m.project(g)
    assert front.all()
    back, ok = m.pixels_to_plane(uv[inside], 0.0)
    assert ok.all()
    assert np.abs(back - g[inside]).max() < 1e-9


def test_distortion_roundtrip():
    m = model(D=[-0.12, 0.03, 0.0005, -0.0003, 0.0])
    g = np.array([[0.6, 0.2, 0.0], [1.5, -0.4, 0.0], [0.4, 0.0, 0.025]])
    uv, _, inside = m.project(g)
    assert inside.all()
    back = np.vstack([m.pixels_to_plane(uv[i:i + 1], g[i, 2])[0] for i in range(3)])
    assert np.abs(back - g).max() < 1e-5


def test_point_behind_camera_is_not_projected():
    uv, front, inside = model().project([[-1.0, 0.0, 0.0]])
    assert not front[0] and not inside[0] and np.isnan(uv[0]).all()


def test_ground_range_matches_hand_calculation():
    near, far = model().ground_range(0.0)
    half_v = math.atan((H / 2) / F)
    assert near == pytest.approx(0.10 + 0.54 / math.tan(math.radians(45) + half_v), abs=2e-3)
    assert far == pytest.approx(0.10 + 0.54 / math.tan(math.radians(45) - half_v), abs=2e-2)


def test_sign_pose_fit_survives_nan_jacobian():
    """F4 A2 bulgusu: artık sonluyken sonlu farkta bir köşe kameranın arkasına düşünce Jacobian NaN oluyor,
    lstsq istisna atıp ön algı düğümünü kapatıyordu. Uydurma artık 'başarısız' döner."""
    from forklift_perception.core.signs import SignDetector
    det = SignDetector.__new__(SignDetector)
    det.block_st = np.zeros((4, 3))
    q = np.array([[300.0, 200.0], [340.0, 200.0], [340.0, 240.0], [300.0, 240.0]])
    calls = []

    def project(cam, params, pts):                   # ilk çağrı (artık) sonlu, sonlu farklar NaN
        calls.append(1)
        return q + 1.0 if len(calls) == 1 else np.full_like(q, np.nan)
    det._project = project
    params, err = det._gauss_newton(None, q, np.zeros(3))
    assert params is None and err == np.inf and len(calls) == 4
