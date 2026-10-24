"""Eğim kestiricisi ve eğik kamera modeli: işaretler ve tutarlılık."""

import math

import numpy as np
import pytest

from forklift_perception.core.attitude import AttitudeEstimator
from forklift_perception.core.camera_model import CameraModel

G = 9.81


def accel(pitch, roll):
    """Duran gövdede ivmeölçerin ölçtüğü özgül kuvvet (yukarı yön, gövde çerçevesinde)."""
    return (G * math.sin(pitch), G * math.cos(pitch) * math.sin(roll), G * math.cos(pitch) * math.cos(roll))


def test_level_calibration_removes_bias_and_reads_pitch_roll():
    est = AttitudeEstimator(tau=0.2, level_samples=50)
    bias = (0.01, -0.02, 0.0)
    for k in range(50):                        # düz zeminde açılış
        est.update(k * 0.01, np.add(accel(0, 0), bias), (0, 0, 0))
    p, r = math.radians(0.41), math.radians(0.34)
    for k in range(50, 400):
        est.update(k * 0.01, np.add(accel(p, r), bias), (0, 0, 0))
    pe, re = est.value()
    assert pe == pytest.approx(p, abs=math.radians(0.01))
    assert re == pytest.approx(r, abs=math.radians(0.01))


def test_gyro_sign_matches_accel_sign():
    est = AttitudeEstimator(tau=100.0, level_samples=1)   # neredeyse yalnız jiroskop
    est.update(0.0, accel(0, 0), (0, 0, 0))
    est.update(0.01, accel(0, 0), (0, 0, 0))
    # +y etrafında negatif açısal hız = burun yukarı
    for k in range(2, 102):
        est.update(k * 0.01, accel(0, 0), (0.0, -0.01, 0.0))
    assert est.value()[0] > 0


def test_tilted_equals_pitched_for_pure_pitch():
    R = np.column_stack([[0, -1, 0], [-0.7071, 0, -0.7071], [0.7071, 0, -0.7071]])
    cam = CameraModel(np.diag([381.0, 381.0, 1.0]) + [[0, 0, 320], [0, 0, 240], [0, 0, 0]], None, 640, 480,
                      R, [0.1, 0.0, 0.54])
    a = cam.pitched(math.radians(0.41), -0.28)
    b = cam.tilted(math.radians(0.41), 0.0, (-0.28, 0.0, 0.0))
    assert np.allclose(a.R, b.R) and np.allclose(a.t, b.t)
    # burun yukarı: kamera yükselir (pivot arkada)
    assert b.t[2] > cam.t[2]


def test_motion_gate_ignores_acceleration_while_driving():
    est = AttitudeEstimator(tau=0.3, level_samples=50, acc_tol=0.05, gyro_tol=0.02)
    for k in range(50):
        est.update(k * 0.01, accel(0, 0), (0, 0, 0))
    for k in range(50, 150):                   # 1 s boyunca 1 m/s² ileri hızlanma: sahte ~6°
        a = np.add(accel(0, 0), (1.0, 0.0, 0.0))
        est.update(k * 0.01, a, (0, 0, 0))
    assert abs(est.value()[0]) < math.radians(0.05)


def test_level_calibration_ignores_dropout_samples():
    # Gazebo IMU'su arada sıfır ivme yayınlıyor; ortalamayla g küçük çıkar ve hareket kapısı hiç açılmaz
    est = AttitudeEstimator(tau=0.2, level_samples=200, acc_tol=0.05, gyro_tol=0.02)
    for k in range(200):
        a = (0.0, 0.0, 0.05) if k % 60 == 5 else accel(0, 0)
        est.update(k * 0.01, a, (0, 0, 0.001))
    assert est.g == pytest.approx(G, abs=1e-6)
    p = math.radians(0.41)
    for k in range(200, 800):                  # duruşta eğim: kapı açık olmalı, eğim okunmalı
        est.update(k * 0.01, accel(p, 0), (0, 0.0005, 0))
    assert est.value()[0] == pytest.approx(p, abs=math.radians(0.05))
