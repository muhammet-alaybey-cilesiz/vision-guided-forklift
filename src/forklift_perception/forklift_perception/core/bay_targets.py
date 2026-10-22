"""
bay_targets.py — daire hedef çiftleri (D-55): bölme girişinin konumu ve yaw'ı

Adımlar
 1. Aday plaka: düşük doygunluklu ve parlak (beyaz) bölgelerin dış konturu; delikler
    (siyah halka ve nokta) dolu sayılır.
 2. Boyut denetimi: kontur merkezi z = target_z düzlemine izdüşürülür; o noktada
    36 mm'lik plakanın beklenen piksel alanıyla karşılaştırılır.
 3. Desen denetimi: plaka homografiyle kare bir görüntüye düzeltilir; merkezdeki
    nokta ve halka koyu, aradaki boşluk ve kenar açık olmalı.
 4. Nokta: plakanın İÇİNDE kalan maskeyle koyuluk ağırlık merkezi (D-61: kenar
    pikselleri dışarıda).
 5. Işın ile z = target_z düzleminin kesişimi → base_footprint'te nokta.
 6. Eşleştirme: iki nokta arası katalog tabanı (0.216 m) ± tolerans; çiftin doğrultusu
    robotun yanal eksenine yakın (aynı ön çizgi).
Çıktı: soldan sağa bölmeler; her biri için giriş pozu (ön çizgi x = 0, kanal ekseni),
yaw ve mesafe.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class TargetPoint:
    uv: np.ndarray              # görüntüde nokta [px]
    p: np.ndarray               # base_footprint'te nokta (z = target_z)
    area_ratio: float           # ölçülen / beklenen plaka alanı
    pattern_score: float        # desen karşıtlığı (0–1)


@dataclass
class BayPair:
    left: TargetPoint
    right: TargetPoint
    entry_xy: np.ndarray        # ön çizgi üzerindeki giriş noktası (x = 0 düzleminde), base
    theta: float                # bölme ekseni yönü (+x istasyonun içine), base
    distance: float             # base orijininden ön çizgiye, eksen boyunca
    baseline: float
    confidence: float


class BayTargetDetector:
    def __init__(self, catalog, cfg):
        self.cat = catalog
        self.cfg = cfg

    # --------------------------------------------------------------- adaylar
    def white_mask(self, rgb, gray=None):
        c = self.cfg
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY) if gray is None else gray
        # beyaz plaka: düşük doygunluk + çevresinden belirgin parlak (yerel karşıtlık: gölgedeki
        # plakalar da bulunur; geniş açık zemin kendi çevresinden parlak olmadığı için plakayla
        # birleşmez) + mutlak alt sınır. Açma yok: 4 mm'lik kenar 1 m'de 1–2 piksel.
        local = cv2.blur(gray, (c['local_block'], c['local_block'])).astype(np.int16)
        bright = (gray.astype(np.int16) > local + c['local_delta']) & (gray >= c['white_v_min'])
        return ((hsv[..., 1] <= c['white_s_max']) & bright).astype(np.uint8) * 255

    def candidates(self, rgb, cam, valid_mask=None):
        c = self.cfg
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        white = self.white_mask(rgb, gray)
        if valid_mask is not None:
            white[~valid_mask] = 0
        contours, hier = cv2.findContours(white, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        if hier is None:
            return out
        hier = hier[0]
        ring_ratio = (self.cat.target_ring[0] / self.cat.target_size) ** 2
        for i, cnt in enumerate(contours):
            if hier[i][3] != -1:                               # delikler aşağıda, ebeveynleriyle
                continue
            area = cv2.contourArea(cnt)
            if area < c['min_area_px']:
                continue
            m = cv2.moments(cnt)
            if m['m00'] <= 0:
                continue
            uc = np.array([m['m10'] / m['m00'], m['m01'] / m['m00']])
            p, ok = cam.pixels_to_plane([uc], self.cat.target_z)
            if not ok[0]:
                continue
            exp = self._expected_area(cam, p[0])
            ratio = area / exp if exp > 0 else 0.0
            if c['area_ratio'][0] <= ratio <= c['area_ratio'][1]:
                quad = self._quad(cnt)
                if quad is not None:
                    self._accept(out, gray, cam, quad, ratio)
            elif ratio > c['area_ratio'][1]:
                # plaka açık renkli bir yüzeyle birleşmiş (ör. kaynak istasyonun beyaz önlük bandı):
                # hedefin koyu halkası bileşenin deliği olarak durur; halkadan plaka kurulur
                k = hier[i][2]
                while k != -1:
                    self._from_hole(out, gray, cam, contours[k], ring_ratio)
                    k = hier[k][0]
        return out

    def _accept(self, out, gray, cam, quad, ratio):
        c = self.cfg
        score = self._pattern(gray, quad)
        if score < c['pattern_min']:
            return
        uv = self._centroid(gray, quad)
        if uv is None:
            return
        pt, ok = cam.pixels_to_plane([uv], self.cat.target_z)
        if ok[0]:
            out.append(TargetPoint(uv, pt[0], float(ratio), float(score)))

    def _from_hole(self, out, gray, cam, hole, ring_ratio):
        c = self.cfg
        area = cv2.contourArea(hole)
        if area < c['min_area_px'] * ring_ratio:
            return
        m = cv2.moments(hole)
        if m['m00'] <= 0:
            return
        uc = np.array([m['m10'] / m['m00'], m['m01'] / m['m00']])
        p, ok = cam.pixels_to_plane([uc], self.cat.target_z)
        if not ok[0]:
            return
        exp = self._expected_area(cam, p[0]) * ring_ratio
        ratio = area / exp if exp > 0 else 0.0
        if not c['area_ratio'][0] <= ratio <= c['area_ratio'][1]:
            return
        (cx, cy), (w, h), ang = cv2.minAreaRect(hole)
        k = self.cat.target_size / self.cat.target_ring[0]   # halka → plaka
        quad = cv2.boxPoints(((cx, cy), (w * k, h * k), ang)).astype(np.float32)
        self._accept(out, gray, cam, quad, ratio)

    def _expected_area(self, cam, p):
        """p'de duran, robota bakan 36 mm'lik dik karenin beklenen piksel alanı."""
        a = self.cat.target_size / 2.0
        n = np.array([p[0], p[1], 0.0])
        n /= max(np.linalg.norm(n), 1e-9)
        t = np.array([-n[1], n[0], 0.0])                  # yatay, yüze teğet
        corners = np.array([p + a * t + [0, 0, a], p - a * t + [0, 0, a], p - a * t - [0, 0, a],
                            p + a * t - [0, 0, a]])
        uv, front, _ = cam.project(corners)
        if not front.all():
            return 0.0
        return float(cv2.contourArea(uv.astype(np.float32)))

    @staticmethod
    def _quad(cnt):
        hull = cv2.convexHull(cnt)
        peri = cv2.arcLength(hull, True)
        for eps in (0.04, 0.06, 0.08, 0.1):
            ap = cv2.approxPolyDP(hull, eps * peri, True)
            if len(ap) == 4:
                return ap.reshape(4, 2).astype(np.float32)
        r = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(r)
        if cv2.contourArea(cnt) / max(r[1][0] * r[1][1], 1e-9) < 0.75:
            return None
        return box.astype(np.float32)

    def _pattern(self, gray, quad, n=48):
        """Düzeltilmiş plakada radyal profil: nokta ve halka koyu, boşluk ve kenar açık → 0–1 puan."""
        q = _order(quad)
        H = cv2.getPerspectiveTransform(q, np.float32([[0, 0], [n - 1, 0], [n - 1, n - 1], [0, n - 1]]))
        w = cv2.warpPerspective(gray, H, (n, n), flags=cv2.INTER_LINEAR).astype(float)
        yy, xx = np.mgrid[0:n, 0:n]
        r = np.hypot(xx - (n - 1) / 2, yy - (n - 1) / 2) / (n / 2) * (self.cat.target_size / 2)
        ro, ri, rd = (d / 2 for d in self.cat.target_ring)
        zones = {'dot': r < rd * 0.8, 'gap': (r > rd * 1.15) & (r < ri * 0.9),
                 'ring': (r > ri * 1.1) & (r < ro * 0.9), 'margin': (r > ro * 1.15) & (r < ro * 1.25)}
        if any(z.sum() == 0 for z in zones.values()):
            return 0.0
        mean = {k: w[z].mean() for k, z in zones.items()}
        light = min(mean['gap'], mean['margin'])
        dark = max(mean['dot'], mean['ring'])
        span = max(w.max() - w.min(), 1.0)
        return float(np.clip((light - dark) / span, 0.0, 1.0))

    def _centroid(self, gray, quad):
        m = np.zeros(gray.shape, np.uint8)
        cv2.fillConvexPoly(m, np.round(quad).astype(np.int32), 255)
        m = cv2.erode(m, np.ones((3, 3), np.uint8)) > 0          # D-61: maske plakanın içinde
        if m.sum() < 12:
            return None
        g = gray.astype(float)
        vals = g[m]
        white, black = np.percentile(vals, 90), np.percentile(vals, 10)
        if white - black < self.cfg['min_contrast']:
            return None
        wgt = np.clip((white - g) / (white - black), 0.0, 1.0) * m
        s = wgt.sum()
        if s <= 0:
            return None
        ys, xs = np.nonzero(m)
        ws = wgt[ys, xs]
        return np.array([(ws * xs).sum() / s, (ws * ys).sum() / s])

    # --------------------------------------------------------------- çiftler
    def detect(self, rgb, cam, valid_mask=None):
        pts = self.candidates(rgb, cam, valid_mask)
        return self.pair(pts), pts

    def pair(self, pts):
        c = self.cfg
        base = self.cat.target_baseline
        cand = []
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                a, b = pts[i], pts[j]
                d = b.p[:2] - a.p[:2]
                L = float(np.linalg.norm(d))
                if abs(L - base) > c['baseline_tol']:
                    continue
                # ön çizgiye dik yön: robottan istasyona (orta noktaya doğru)
                n = np.array([d[1], -d[0]]) / L
                mid = (a.p[:2] + b.p[:2]) / 2
                if np.dot(n, mid) < 0:
                    n = -n
                # aynı ön çizgi: çiftin doğrultusu robotun y eksenine yakın olmalı (arka plakadaki
                # bir aday 0.3 m geride kalır ve çifti ~54° döndürür; D-55/D-56)
                if abs(math.degrees(math.atan2(abs(d[0]), abs(d[1])))) > c['max_pair_yaw_deg']:
                    continue
                cand.append((abs(L - base), i, j, n, mid, L))
        cand.sort(key=lambda t: t[0])
        used, pairs = set(), []
        for err, i, j, n, mid, L in cand:
            if i in used or j in used:
                continue
            used.update((i, j))
            a, b = pts[i], pts[j]
            left, right = (a, b) if np.cross(n, a.p[:2] - mid) > 0 else (b, a)
            # hedef yüzü ön çizginin target_face_x kadar önünde: girişi x = 0'a taşı
            entry = mid - n * self.cat.target_face_x
            theta = math.atan2(n[1], n[0])
            conf = float(np.clip(1.0 - err / c['baseline_tol'], 0, 1) * min(a.pattern_score, b.pattern_score) * 2)
            pairs.append(BayPair(left, right, entry, theta, float(np.dot(entry, n)), L, min(conf, 1.0)))
        pairs.sort(key=lambda p: -float(p.entry_xy[1]))         # soldan sağa (base +y sol)
        return pairs


def _order(q):
    """Dörtgen köşeleri: sol üst, sağ üst, sağ alt, sol alt (görüntüde)."""
    q = np.asarray(q, np.float32)
    s = q.sum(axis=1)
    d = np.diff(q, axis=1).ravel()
    return np.float32([q[np.argmin(s)], q[np.argmin(d)], q[np.argmax(s)], q[np.argmax(d)]])
