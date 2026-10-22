"""
bay_labels.py — arka plakadaki bölme etiketleri: son yanaşmada mesafe ve yanal konum (D-62)

Etiket: 45 mm sarı kare, bölme ekseninde, z = label_center_z, arka plakanın ön yüzünde
(station x = label_face_x). Numara okunmaz; yalnız geometri kullanılır.
Adımlar
 1. Etiket sarısı maskesi (koli kartonundan 2–3 ton birimi ayrı; ayrıca şekil denetimi).
 2. Dış kontur (rakamlar delik olarak dolar) → dörtgen; merkez = köşegenlerin kesişimi
    (perspektifte doğru merkez; ağırlık merkezi değil).
 3. Işın ile z = label_center_z düzleminin kesişimi → base'te nokta; o noktadaki
    beklenen kare alanıyla boyut denetimi.
 4. Bölme girişi = etiket noktası − label_face_x · (istasyon yönü). Yön: en az iki etiket
    adım (bay_pitch) aralıkla görünüyorsa sıralarının doğrultusundan; tek etiketse
    robotun yönü varsayılır (yaw_valid = False).
 Robotun eğimi (tekerlekler önlük plakasında, D-46) çağıran tarafta düzeltilir: eğimli
 kamera modeliyle yeniden çağrılır (pipelines.LanePipeline). Köşelerle PnP denendi; 45 mm'lik
 etiket ~40 px olduğundan köşe hatası mesafeye ±5–18 mm olarak yansıdı (düzlem kesişimi ±1 mm).
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class LabelPoint:
    uv: np.ndarray
    p: np.ndarray               # base'te etiket merkezi (düzlem kesişimi, robot düz varsayımı)
    area_ratio: float
    quad: np.ndarray            # görüntüde dörtgen (4×2)


@dataclass
class LabelEntry:
    label: LabelPoint
    entry_xy: np.ndarray
    theta: float
    yaw_valid: bool
    distance: float
    confidence: float


class BayLabelDetector:
    def __init__(self, catalog, colors, cfg):
        self.cat, self.colors, self.cfg = catalog, colors, cfg

    def labels(self, rgb, cam, valid_mask=None):
        c = self.cfg
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        m = self.colors.mask(hsv, 'label_yellow')
        if valid_mask is not None:
            m[~valid_mask] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < c['min_area_px']:
                continue
            quad = _quad(cnt)
            if quad is None:
                continue
            uv = _diag_center(quad)
            if uv is None:
                continue
            p, ok = cam.pixels_to_plane([uv], self.cat.label_center_z)
            if not ok[0]:
                continue
            exp = _expected_area(cam, p[0], self.cat.label_size)
            ratio = area / exp if exp > 0 else 0.0
            if not c['area_ratio'][0] <= ratio <= c['area_ratio'][1]:
                continue
            if _shape_error(cam, p[0], self.cat.label_size, quad) > c['shape_tol']:
                continue
            if not self._on_back_plate(hsv, quad):
                continue
            out.append(LabelPoint(uv, p[0], float(ratio), quad))
        out.sort(key=lambda lp: -lp.p[1])                     # soldan sağa
        return out

    def _on_back_plate(self, hsv, quad):
        """Bağlam: etiket alüminyum arka plakanın üstünde (çevresi düşük doygunluklu ve koyu değil).
        Zemindeki sarı-siyah tehlike bandının parçaları siyah şeritlerle çevrili olduğu için elenir."""
        c = self.cfg
        h, w = hsv.shape[:2]
        inner = np.zeros((h, w), np.uint8)
        cv2.fillConvexPoly(inner, np.round(quad).astype(np.int32), 255)
        side = max(np.linalg.norm(quad[0] - quad[1]), np.linalg.norm(quad[1] - quad[2]))
        k = max(3, int(round(side * c['ring_frac'])))
        outer = cv2.dilate(inner, np.ones((2 * k + 1, 2 * k + 1), np.uint8))
        ring = (outer > 0) & (inner == 0)
        if ring.sum() < 10:
            return False
        sat, val = hsv[..., 1][ring], hsv[..., 2][ring]
        low_s = (sat <= c['ring_s_max']).mean() >= c['ring_low_s_min']
        return low_s and (val < c['ring_dark_v']).mean() <= c['ring_dark_max']

    def detect(self, rgb, cam, valid_mask=None):
        pts = self.labels(rgb, cam, valid_mask)
        theta, yaw_valid = 0.0, False
        if len(pts) >= 2:
            # komşu etiketler bay_pitch aralıklı: doğrultuları ön çizgiye paralel
            ths = []
            for a, b in zip(pts, pts[1:]):
                d = a.p[:2] - b.p[:2]                         # sağdan sola (+y)
                L = float(np.linalg.norm(d))
                k = round(L / self.cat.bay_pitch)
                if k >= 1 and abs(L - k * self.cat.bay_pitch) < self.cfg['pitch_tol']:
                    ths.append(math.atan2(-d[0], d[1]))      # d'ye dik, istasyona doğru
            if ths:
                theta, yaw_valid = float(np.mean(ths)), True
        n = np.array([math.cos(theta), math.sin(theta)])
        out = []
        for lp in pts:
            entry = lp.p[:2] - n * self.cat.label_face_x
            conf = float(np.clip(1.0 - abs(1.0 - lp.area_ratio), 0.0, 1.0))
            out.append(LabelEntry(lp, entry, theta, yaw_valid, float(np.dot(entry, n)), conf))
        return out


def _quad(cnt):
    hull = cv2.convexHull(cnt)
    peri = cv2.arcLength(hull, True)
    for eps in (0.03, 0.05, 0.08):
        ap = cv2.approxPolyDP(hull, eps * peri, True)
        if len(ap) == 4:
            return ap.reshape(4, 2).astype(float)
    return None


def _diag_center(q):
    """Dörtgenin köşegenlerinin kesişimi (köşeler çevre sırasında)."""
    a, b, c, d = q
    r, s = c - a, d - b
    den = r[0] * s[1] - r[1] * s[0]
    if abs(den) < 1e-9:
        return None
    t = ((b - a)[0] * s[1] - (b - a)[1] * s[0]) / den
    return a + t * r


def _expected_area(cam, p, size):
    a = size / 2.0
    n = np.array([p[0], p[1], 0.0])
    n /= max(np.linalg.norm(n), 1e-9)
    t = np.array([-n[1], n[0], 0.0])
    corners = np.array([p + a * t + [0, 0, a], p - a * t + [0, 0, a], p - a * t - [0, 0, a], p + a * t - [0, 0, a]])
    uv, front, _ = cam.project(corners)
    return float(cv2.contourArea(uv.astype(np.float32))) if front.all() else 0.0


def _shape_error(cam, p, size, quad):
    """Ölçülen dörtgen ile p'de duran dik karenin beklenen izdüşümü arasındaki en büyük köşe
    farkı / kenar (merkezler çakıştırılarak; köşe sırası en iyi eşleşmeye göre)."""
    a = size / 2.0
    n = np.array([p[0], p[1], 0.0])
    n /= max(np.linalg.norm(n), 1e-9)
    t = np.array([-n[1], n[0], 0.0])
    corners = np.array([p + a * t + [0, 0, a], p - a * t + [0, 0, a], p - a * t - [0, 0, a], p + a * t - [0, 0, a]])
    uv, front, _ = cam.project(corners)
    if not front.all():
        return np.inf
    e = uv - uv.mean(axis=0)
    m = np.asarray(quad, float) - np.asarray(quad, float).mean(axis=0)
    side = max(np.linalg.norm(e[0] - e[1]), np.linalg.norm(e[1] - e[2]), 1e-9)
    best = np.inf
    for order in (m, m[::-1]):
        for k in range(4):
            best = min(best, float(np.max(np.linalg.norm(np.roll(order, k, axis=0) - e, axis=1))))
    return best / side
