"""
colors.py — HSV renk sınıfları (eşikler perception.yaml'da, plan R4/R7)

Eşikler kayıtlı kümede GT bölgelerinden ölçülen dağılımdan seçildi
(forklift_evaluation color_stats; REPORT_CV §F2). Gerçek kamerada aynı araçla
etiketli karelerden yeniden kalibre edilir. Kırmızı tonu çemberin iki ucundadır;
bu yüzden her sınıf birden çok H aralığı alabilir.
"""

import cv2
import numpy as np


class ColorClasses:
    """cfg: {ad: {h: [[lo, hi], ...], s: [lo, hi], v: [lo, hi]}} (OpenCV ölçeği: H 0–179)."""

    def __init__(self, cfg):
        self.cfg = cfg

    def names(self):
        return list(self.cfg)

    def mask(self, hsv, name):
        c = self.cfg[name]
        s_lo, s_hi = c['s']
        v_lo, v_hi = c['v']
        m = None
        for h_lo, h_hi in c['h']:
            part = cv2.inRange(hsv, (int(h_lo), int(s_lo), int(v_lo)), (int(h_hi), int(s_hi), int(v_hi)))
            m = part if m is None else cv2.bitwise_or(m, part)
        return m

    def classify_pixels(self, hsv_pixels, names):
        """N×3 HSV piksel → her ad için eşleşen oran (sözlük)."""
        px = np.asarray(hsv_pixels).reshape(-1, 1, 3).astype(np.uint8)
        out = {}
        for n in names:
            out[n] = float((self.mask(px, n) > 0).mean()) if len(px) else 0.0
        return out


def to_hsv(rgb):
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
