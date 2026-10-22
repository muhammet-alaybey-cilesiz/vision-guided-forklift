"""
calibration.py — kamera kalibrasyonu (dama tahtası) ve ROS kalibrasyon dosyası

Gerçek kamerada ve simde aynı yöntem (DECISIONS D-59): dama tahtasının çok
sayıda görüntüsünden köşeler bulunur, OpenCV calibrateCamera K ve D'yi kestirir.
Dosya biçimi camera_calibration_parsers'ın YAML'ı (ROS camera_calibration aracının
yazdığıyla aynı), böylece gerçek robotta o araçla üretilen dosya doğrudan okunur.
"""

import cv2
import numpy as np
import yaml


def board_object_points(cols, rows, square):
    """İç köşe ızgarası (cols × rows) için düzlemsel 3B noktalar [m] (z = 0)."""
    g = np.zeros((cols * rows, 3), np.float32)
    g[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return g


def find_corners(gray, cols, rows):
    """Dama tahtası iç köşeleri (alt-piksel) → N×2 ya da None."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, c = cv2.findChessboardCorners(gray, (cols, rows), flags=flags)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-4)
    c = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), crit)
    return c.reshape(-1, 2)


def calibrate(image_points, object_points, size, fix_k3=True):
    """calibrateCamera → (rms, K, D, görüş başına hata [px]). size = (genişlik, yükseklik)."""
    flags = cv2.CALIB_FIX_K3 if fix_k3 else 0
    obj = [object_points.astype(np.float32)] * len(image_points)
    img = [p.reshape(-1, 1, 2).astype(np.float32) for p in image_points]
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj, img, size, None, None, flags=flags)
    per_view = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, K, D)
        per_view.append(float(np.sqrt(np.mean(np.sum((proj - i) ** 2, axis=2)))))
    return float(rms), K, D.ravel(), per_view


def save_yaml(path, camera_name, size, K, D, comment=''):
    """ROS kalibrasyon YAML'ı (plumb_bob; R birim, P = [K | 0])."""
    K = np.asarray(K, float)
    D = np.asarray(D, float).ravel()
    P = np.hstack([K, np.zeros((3, 1))])
    data = {
        'image_width': int(size[0]), 'image_height': int(size[1]), 'camera_name': camera_name,
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': [float(x) for x in K.ravel()]},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'rows': 1, 'cols': len(D), 'data': [float(x) for x in D]},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': [float(x) for x in np.eye(3).ravel()]},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': [float(x) for x in P.ravel()]},
    }
    with open(path, 'w') as f:
        if comment:
            f.write(''.join(f'# {line}\n' for line in comment.splitlines()))
        yaml.safe_dump(data, f, sort_keys=False)


def load_yaml(path):
    """→ (K 3×3, D, (genişlik, yükseklik), kamera adı)."""
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], float).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], float)
    return K, D, (int(d['image_width']), int(d['image_height'])), d.get('camera_name', '')
