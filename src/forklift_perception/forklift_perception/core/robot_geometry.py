"""
robot_geometry.py — robotun kameralarda görünen parçaları (çatallar, taşıyıcı, taşınan koli)

forklift_params.yaml'dan (robotun kendi ölçüleri; gerçek robotta da bilinir, R1).
Görüntüde bu bölgeler maskelenir: şerit uydurmada kullanılmaz (F1: çatallar
şeridin kenarlarını kapatıyor) ve yük denetiminde koli bölgesi olarak bakılır.
"""

import cv2
import numpy as np
import yaml


def _box(xr, yr, zr):
    return np.array([[x, y, z] for x in xr for y in yr for z in zr], float)


class RobotGeometry:
    def __init__(self, params, parcel_size):
        fk, fc = params['fork'], params['fork_carriage']
        self.fork = fk
        self.carriage = fc
        self.carriage_front_x = fc['x'] + fc['size']['x'] / 2.0
        self.fork_top0 = fk['ground_z_bottom'] + fk['size']['z']
        self.parcel_size = parcel_size        # (x, y, z), katalogdan
        self.caster_x = params['caster']['x']  # sarhoş teker temas noktası (gövde önden kalkınca pivot)

    @classmethod
    def load(cls, robot_params_path, parcel_size):
        with open(robot_params_path) as f:
            return cls(yaml.safe_load(f), parcel_size)

    def fork_boxes(self, q):
        """İki çatal ve taşıyıcı plakası (base_footprint, her biri 8 köşe)."""
        fk, fc = self.fork, self.carriage
        out = []
        for s in (-1, 1):
            out.append(_box([fk['x_min'], fk['x_min'] + fk['size']['x']],
                            [s * fk['y'] - fk['size']['y'] / 2, s * fk['y'] + fk['size']['y'] / 2],
                            [fk['ground_z_bottom'] + q, fk['ground_z_bottom'] + fk['size']['z'] + q]))
        out.append(_box([fc['x'] - fc['size']['x'] / 2, fc['x'] + fc['size']['x'] / 2],
                        [-fc['size']['y'] / 2, fc['size']['y'] / 2],
                        [fc['ground_z_bottom'] + q, fc['ground_z_bottom'] + fc['size']['z'] + q]))
        return out

    def carried_parcel_box(self, q, gap=0.002):
        """Çataldaki kolinin nominal kutusu (taşıyıcıya dayalı, çatalların üstünde)."""
        px, py, pz = self.parcel_size
        x0 = self.carriage_front_x + gap
        z0 = self.fork_top0 + q
        return _box([x0, x0 + px], [-py / 2, py / 2], [z0, z0 + pz])


def hull_mask(cam, corners_list, dilate=0):
    """Kutuların izdüşüm dış kabukları → bool maske (kameranın arkasındakiler atlanır)."""
    m = np.zeros((cam.height, cam.width), np.uint8)
    for c in corners_list:
        uv, front, _ = cam.project(c)
        if not front.all() or not np.all(np.isfinite(uv)) or np.abs(uv).max() > 1e5:
            continue
        cv2.fillConvexPoly(m, cv2.convexHull(np.round(uv).astype(np.int32)), 255)
    if dilate:
        m = cv2.dilate(m, np.ones((2 * dilate + 1, 2 * dilate + 1), np.uint8))
    return m > 0
