"""
parcels.py — koli, mühür rengi (ön kamera) ve çatalda yük var mı (şerit kamerası)

Koli: kraft karton kutu; mühür −x yüzünde ortalı, katalog kenarı seal_size.
Adımlar (koli)
 1. Mühür adayları: doygun kırmızı / yeşil / mavi bölgeler, kareye yakın, yeterli alan.
 2. Bağlam: mührün çevresi koli kartonu (kraft) olmalı → kırmızı tabela, renkli şerit ve
    istasyon bandı elenir (mühür bir koli yüzündedir).
 3. Konum: mühür merkezinin ışını ile mühür yüksekliğinin düzlemi (çıkıntıdaki koli:
    ledge_height + parcel_z / 2) kesişir → base'te koli ön yüzünün ortası. Boyut denetimi:
    o noktada beklenen mühür alanı.
 4. Kutu: mührü içeren kraft bileşeninin sınırı (komşu koliler birleşmişse mühürler
    arasındaki orta çizgilerden bölünür).
Yük: taşınan kolinin şerit kamerasındaki silüeti içinde kraft oranı.
"""

from dataclasses import dataclass

import cv2
import numpy as np

SEAL_COLORS = ('red', 'green', 'blue')


@dataclass
class ParcelDet:
    seal_color: str
    seal_uv: np.ndarray
    position: np.ndarray          # base'te koli ön yüzünün ortası (mühür merkezi)
    bbox: tuple                   # (u0, v0, u1, v1)
    confidence: float
    seal_confidence: float


class ParcelDetector:
    def __init__(self, catalog, colors, cfg):
        self.cat, self.colors, self.cfg = catalog, colors, cfg
        self.seal_z = catalog.ledge_height + catalog.parcel_size[2] / 2.0

    def detect(self, rgb, cam):
        c = self.cfg
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        kraft = self.colors.mask(hsv, 'kraft')
        kraft_c = cv2.morphologyEx(kraft, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        seals = []
        for color in SEAL_COLORS:
            m = self.colors.mask(hsv, color)
            m = cv2.bitwise_and(m, cv2.inRange(hsv, (0, c['seal_s_min'], 0), (179, 255, 255)))
            contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < c['min_seal_area_px']:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                if not 1 / c['max_seal_aspect'] <= w / max(h, 1) <= c['max_seal_aspect']:
                    continue
                if area / max(w * h, 1) < c['min_seal_fill']:
                    continue
                mo = cv2.moments(cnt)
                uv = np.array([mo['m10'] / mo['m00'], mo['m01'] / mo['m00']])
                p, ok = cam.pixels_to_plane([uv], self.seal_z)
                if not ok[0]:
                    continue
                exp = self._expected_seal_area(cam, p[0])
                ratio = area / exp if exp > 0 else 0.0
                if not c['seal_area_ratio'][0] <= ratio <= c['seal_area_ratio'][1]:
                    continue
                kf = _ring_fraction(kraft_c, x, y, w, h, c['ring_frac'])
                if kf < c['min_kraft_ring']:
                    continue
                seals.append((color, uv, p[0], (x, y, w, h), float(np.clip(1 - abs(1 - ratio), 0, 1)), kf))
        return self._boxes(kraft_c, seals)

    def _expected_seal_area(self, cam, p):
        a = self.cat.seal_size / 2.0
        n = np.array([p[0], p[1], 0.0])
        n /= max(np.linalg.norm(n), 1e-9)
        t = np.array([-n[1], n[0], 0.0])
        pts = np.array([p + a * t + [0, 0, a], p - a * t + [0, 0, a], p - a * t - [0, 0, a], p + a * t - [0, 0, a]])
        uv, front, _ = cam.project(pts)
        return float(cv2.contourArea(uv.astype(np.float32))) if front.all() else 0.0

    def _boxes(self, kraft, seals):
        n, lab, stats, _ = cv2.connectedComponentsWithStats(kraft, connectivity=8)
        out = []
        for color, uv, p, (x, y, w, h), sconf, kf in seals:
            u, v = int(round(uv[0])), int(round(uv[1]))
            # mührün hemen altındaki kraft bileşeni (mühür bileşenin deliğidir)
            k = 0
            for dv in range(h // 2 + 1, h // 2 + 12):
                vv = min(v + dv, lab.shape[0] - 1)
                if lab[vv, u] > 0:
                    k = lab[vv, u]
                    break
            if k:
                bx, by, bw, bh = stats[k, :4]
                box = [bx, by, bx + bw, by + bh]
            else:
                box = [x - w // 2, y - h // 2, x + w + w // 2, y + h + h]
            # aynı bileşende başka mühür varsa orta çizgiden böl
            for other in seals:
                if other[1] is uv:
                    continue
                ou = other[1][0]
                if box[0] < ou < box[2] and abs(other[1][1] - uv[1]) < h * 2:
                    mid = (ou + uv[0]) / 2
                    if ou > uv[0]:
                        box[2] = min(box[2], int(mid))
                    else:
                        box[0] = max(box[0], int(mid))
            out.append(ParcelDet(color, uv, p, tuple(int(b) for b in box), float(sconf * kf), sconf))
        return out


class LoadDetector:
    """Çatalda koli var mı: şerit kamerasında taşınan kolinin silüeti içinde kraft oranı."""

    def __init__(self, colors, robot_geometry, cfg):
        self.colors, self.rg, self.cfg = colors, robot_geometry, cfg

    def detect(self, rgb, cam, q):
        from forklift_perception.core.robot_geometry import hull_mask
        roi = hull_mask(cam, [self.rg.carried_parcel_box(q)])
        roi = cv2.erode(roi.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
        if roi.sum() < 50:
            return False, 0.0
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        frac = float((self.colors.mask(hsv, 'kraft')[roi] > 0).mean())
        return frac >= self.cfg['min_kraft_fraction'], frac


def _ring_fraction(mask, x, y, w, h, k):
    """Kutunun çevresindeki halkada maskenin oranı (halka kalınlığı k · kenar)."""
    H, W = mask.shape
    d = max(2, int(round(k * max(w, h))))
    x0, y0, x1, y1 = max(0, x - d), max(0, y - d), min(W, x + w + d), min(H, y + h + d)
    region = mask[y0:y1, x0:x1] > 0
    inner = np.zeros_like(region)
    inner[(y - y0):(y - y0 + h), (x - x0):(x - x0 + w)] = True
    ring = ~inner
    return float(region[ring].mean()) if ring.any() else 0.0
