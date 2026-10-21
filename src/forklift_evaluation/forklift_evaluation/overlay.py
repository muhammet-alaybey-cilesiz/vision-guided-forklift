"""overlay.py — GT geometrisini kamera görüntüsüne çizme ve basit görüntü ölçümleri."""

import cv2
import numpy as np

# RGB
C_TARGET = (255, 0, 255)
C_BAY = (0, 255, 255)
C_LANE = (255, 255, 0)
C_STOP = (255, 128, 0)
C_SIGN = (0, 255, 0)
C_BLOCK = (255, 0, 0)
C_PARCEL = (255, 160, 0)
C_SEAL = (255, 255, 255)
C_FORK = (0, 160, 255)
BOX_EDGES = [(i, j) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count('1') == 1]


def to_base(bf, pts_world):
    """Dünya noktaları → base_footprint (bf: base_footprint'in dünya pozu, sim_tools.Pose)."""
    inv = bf.inv()
    return np.asarray(pts_world, float) @ inv.R.T + inv.p


def _pix(cam, pts_base):
    uv, front, _ = cam.project(pts_base)
    return None if not front.all() else uv


def poly(img, cam, pts_base, color, closed=True, thick=1):
    uv = _pix(cam, pts_base)
    if uv is None:
        return False
    cv2.polylines(img, [np.round(uv).astype(np.int32)], closed, color, thick, cv2.LINE_AA)
    return True


def box(img, cam, corners_base, color, thick=1):
    uv = _pix(cam, corners_base)
    if uv is None:
        return False
    p = np.round(uv).astype(np.int32)
    for i, j in BOX_EDGES:
        cv2.line(img, tuple(p[i]), tuple(p[j]), color, thick, cv2.LINE_AA)
    return True


def cross(img, uv, color, size=4):
    u, v = int(round(uv[0])), int(round(uv[1]))
    cv2.line(img, (u - size, v), (u + size, v), color, 1)
    cv2.line(img, (u, v - size), (u, v + size), color, 1)


def label(img, text, org, color=(255, 255, 255)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)


def poly_mask(shape, uv, erode=0):
    m = np.zeros(shape[:2], np.uint8)
    cv2.fillPoly(m, [np.round(uv).astype(np.int32)], 255)
    if erode:
        m = cv2.erode(m, np.ones((2 * erode + 1, 2 * erode + 1), np.uint8))
    return m > 0


def dark_centroid(rgb, uv_plate, min_contrast=40.0, erode=0):
    """Plaka dörtgeni içinde koyuluk ağırlık merkezi (D-55 yöntemi) → (u, v) ya da None.

    Ağırlık = (beyaz − gri) / (beyaz − siyah), [0, 1]'e kırpılır; beyaz ve siyah
    düzeyleri plakanın kendi 90. ve 10. yüzdeliğidir (ışıktan bağımsız).
    """
    m = poly_mask(rgb.shape, uv_plate, erode)
    if m.sum() < 30:
        return None
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(float)
    vals = gray[m]
    white, black = np.percentile(vals, 90), np.percentile(vals, 10)
    if white - black < min_contrast:
        return None
    w = np.clip((white - gray) / (white - black), 0.0, 1.0) * m
    s = w.sum()
    if s <= 0:
        return None
    vv, uu = np.mgrid[0:rgb.shape[0], 0:rgb.shape[1]]
    return np.array([(w * uu).sum() / s, (w * vv).sum() / s])


def saturated_fraction(rgb, uv_poly, s_min=80, erode=1):
    """Poligon içindeki doygun (renkli) piksel oranı; şerit izdüşümünün renkli banda düşüp düşmediği."""
    m = poly_mask(rgb.shape, uv_poly, erode)
    if m.sum() < 10:
        return None
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    return float((hsv[..., 1][m] >= s_min).mean())


def save_rgb(rgb, path):
    cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def stripe_row_offset(rgb, uv_poly, s_min=80):
    """Şerit dörtgeninin (a0, b0, b1, a1 köşeleri) satırlarında: renkli piksellerin yatay ağırlık
    merkezi − izdüşümdeki merkez çizgisi [px]; satırların medyanı (tanı: izdüşüm yanal kayması)."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    sat = hsv[..., 1].astype(float)
    a = (uv_poly[0] + uv_poly[3]) / 2.0          # başlangıç ucunun ortası
    b = (uv_poly[1] + uv_poly[2]) / 2.0          # bitiş ucunun ortası
    half_w = max(np.linalg.norm(uv_poly[0] - uv_poly[3]), np.linalg.norm(uv_poly[1] - uv_poly[2])) / 2.0
    v0, v1 = int(np.ceil(min(a[1], b[1]))) + 2, int(np.floor(max(a[1], b[1]))) - 2
    if v1 - v0 < 3 or abs(b[1] - a[1]) < 1e-6:
        return None
    offs = []
    for v in range(max(v0, 0), min(v1, rgb.shape[0] - 1) + 1):
        uc = a[0] + (v - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
        u0, u1 = int(np.floor(uc - 2.5 * half_w)), int(np.ceil(uc + 2.5 * half_w))
        if u0 < 0 or u1 >= rgb.shape[1]:
            continue
        w = (sat[v, u0:u1 + 1] >= s_min).astype(float)
        if w.sum() < 2:
            continue
        offs.append((w * np.arange(u0, u1 + 1)).sum() / w.sum() - uc)
    return float(np.median(offs)) if offs else None
