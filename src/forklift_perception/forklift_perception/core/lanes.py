"""
lanes.py — renkli şeritler (zemin kılavuz şeridi ve bölme şeridi), kuş bakışı (IPM) yöntemi

Adımlar
 1. IPM: görüntü, zemin düzlemine (z = plane_z) homografiyle çevrilir; her IPM pikseli
    zeminde sabit bir alan (res × res) olur. Robotun kendi parçaları ve görüş dışı
    bölgeler geçersiz maskelenir.
 2. Renk maskeleri (HSV) IPM'de çıkarılır.
 3. Yönlü açma: önce yaklaşma yönüne dik (60–120°) uzun yapılar bulunup çıkarılır
    (istasyonun ön bandı); sonra robot yönüne ±45° içindeki çizgi çekirdekleriyle açma.
 4. Bağlantılı bileşenler: temel bileşen analiziyle (PCA) uzunluk, genişlik, doğrultu;
    uzun ve şerit genişliğinde olanlar şerittir.
 5. Merkez çizgisi: doğrultu boyunca 2 cm'lik dilimlerin ortası (çoklu çizgi; viraja hazır).
    Yanal ofset ve yön, doğru uydurmasından (robotun x = 0'ındaki y ve açı).
 6. Uç ve durma çizgisi: uzak uç görüş içinde ve ötesi boşsa uç görünür; uçta beyaz,
    şeride dik bir bant varsa durma çizgisi.
Bölme şeridi / zemin şeridi ayrımı lane_perception'da hedef çiftleriyle yapılır.
"""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class LaneSeg:
    color: str
    points: np.ndarray            # merkez çizgisi (N×2, base, yakından uzağa)
    direction: np.ndarray         # birim doğrultu (robottan uzağa)
    centroid: np.ndarray
    length: float
    width: float
    lateral_offset: float         # x = 0'da y
    heading: float
    near: float                   # doğrultu boyunca en yakın / en uzak nokta (base orijinden)
    far: float
    end_visible: bool
    stop_bar_distance: float = float('nan')
    kind: int = 0                 # 0 zemin, 1 bölme (lane_perception atar)
    confidence: float = 0.0
    extra: dict = field(default_factory=dict)


class IPM:
    """Kameradan z = plane_z düzlemine kuş bakışı dönüşüm (kamera modeli ve sınırlar sabitse önbellekli)."""

    def __init__(self, cam, x_range, y_range, res, plane_z):
        self.cam, self.res, self.z = cam, res, plane_z
        self.x0, self.x1 = x_range
        self.y0, self.y1 = y_range
        self.W = int(round((self.y1 - self.y0) / res))
        self.H = int(round((self.x1 - self.x0) / res))
        Rt = cam.R.T
        G = cam.K @ np.column_stack([Rt[:, 0], Rt[:, 1], Rt @ (np.array([0, 0, plane_z]) - cam.t)])
        # IPM pikseli (sütun, satır) → zemin (x, y): üst = uzak, sol = +y
        A = np.array([[0.0, -res, self.x1 - res / 2], [-res, 0.0, self.y1 - res / 2], [0.0, 0.0, 1.0]])
        self.H_img_from_ipm = G @ A
        ones = np.full((cam.height, cam.width), 255, np.uint8)
        self.valid = self.warp(ones, cv2.INTER_NEAREST) > 0
        # kameranın arkasına düşen IPM pikselleri (homografi işaret değiştirir) geçersiz
        cols, rows = np.meshgrid(np.arange(self.W), np.arange(self.H))
        pts = self.to_ground(np.column_stack([cols.ravel(), rows.ravel()]))
        _, front, inside = cam.project(np.column_stack([pts, np.full(len(pts), plane_z)]))
        self.valid &= (front & inside).reshape(self.H, self.W)

    def warp(self, img, interp=cv2.INTER_LINEAR):
        return cv2.warpPerspective(img, self.H_img_from_ipm, (self.W, self.H),
                                   flags=interp | cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_CONSTANT)

    def to_ground(self, colrow):
        cr = np.asarray(colrow, float)
        return np.column_stack([self.x1 - (cr[:, 1] + 0.5) * self.res, self.y1 - (cr[:, 0] + 0.5) * self.res])

    def to_ipm(self, xy):
        xy = np.asarray(xy, float)
        return np.column_stack([(self.y1 - xy[:, 1]) / self.res - 0.5, (self.x1 - xy[:, 0]) / self.res - 0.5])


class LaneDetector:
    def __init__(self, colors, cfg):
        self.colors, self.cfg = colors, cfg
        self._ipm_key, self._ipm = None, None
        L = max(3, int(round(cfg['open_length'] / cfg['res'])))
        self.kernels = [_line_kernel(L, a) for a in cfg['open_angles_deg']]
        self.perp_kernels = [_line_kernel(L, a) for a in cfg['perp_angles_deg']]

    def ipm(self, cam):
        key = (cam.K.tobytes(), cam.R.tobytes(), cam.t.tobytes())
        if key != self._ipm_key:
            c = self.cfg
            self._ipm = IPM(cam, c['x_range'], c['y_range'], c['res'], c['plane_z'])
            self._ipm_key = key
        return self._ipm

    def detect(self, rgb, cam, self_mask=None):
        c = self.cfg
        ipm = self.ipm(cam)
        top = ipm.warp(rgb)
        valid = ipm.valid.copy()
        if self_mask is not None:
            sm = ipm.warp(self_mask.astype(np.uint8) * 255, cv2.INTER_NEAREST) > 0
            sm = cv2.dilate(sm.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
            valid &= ~sm
        hsv = cv2.cvtColor(top, cv2.COLOR_RGB2HSV)
        white = (self.colors.mask(hsv, 'white') > 0) & valid
        out = []
        for color in c['lane_colors']:
            m = self.colors.mask(hsv, color)
            m[~valid] = 0
            # yaklaşma yönüne dik yapılar (istasyonun ön bandı) önce çıkarılır; yoksa şeritler
            # bandın köşeleri üzerinden yan bantlarla birleşip tek bir U bileşeni olur
            perp = np.zeros_like(m)
            for k in self.perp_kernels:
                perp = cv2.bitwise_or(perp, cv2.morphologyEx(m, cv2.MORPH_OPEN, k))
            m[cv2.dilate(perp, np.ones((3, 3), np.uint8)) > 0] = 0
            opened = np.zeros_like(m)
            for k in self.kernels:
                opened = cv2.bitwise_or(opened, cv2.morphologyEx(m, cv2.MORPH_OPEN, k))
            n, lab, stats, _ = cv2.connectedComponentsWithStats(opened, connectivity=8)
            for i in range(1, n):
                if stats[i, cv2.CC_STAT_AREA] * c['res'] ** 2 < c['min_area_m2']:
                    continue
                rr, cc = np.nonzero(lab == i)
                seg = self._segment(ipm, valid, white, color, np.column_stack([cc, rr]))
                if seg is not None:
                    out.append(seg)
        return out, ipm

    def _segment(self, ipm, valid, white, color, colrow):
        c = self.cfg
        g = ipm.to_ground(colrow)
        cen = g.mean(axis=0)
        cov = np.cov((g - cen).T)
        w, v = np.linalg.eigh(cov)
        u = v[:, 1]
        if u[0] < 0:
            u = -u
        if abs(math.degrees(math.atan2(u[1], u[0]))) > c['max_heading_deg']:
            return None
        s = (g - cen) @ u
        t = (g - cen) @ np.array([-u[1], u[0]])
        length = float(np.percentile(s, 99) - np.percentile(s, 1))
        width = float(np.percentile(t, 98) - np.percentile(t, 2))
        if length < c['min_length'] or not c['width_range'][0] <= width <= c['width_range'][1]:
            return None
        if length / max(width, 1e-6) < c['min_aspect']:
            return None
        # merkez çizgisi: doğrultu boyunca dilimler. Bir kenarı geçersiz bölgeye (robotun kendi
        # parçaları, görüş dışı) değen dilimde merkez, görünen kenardan nominal yarı genişlik içeride
        # alınır (F1/F2: çatallar şeridin kenarını örtünce merkez dışa kayıyordu); iki kenarı da
        # değen dilim atlanır.
        step, nrm = c['centerline_step'], np.array([-u[1], u[0]])
        half = c['nominal_width'] / 2.0
        bins = np.floor((s - s.min()) / step).astype(int)
        pts = []
        for b in np.unique(bins):
            sel = bins == b
            if sel.sum() < 3:
                continue
            sm, t0, t1 = s[sel].mean(), np.percentile(t[sel], 2), np.percentile(t[sel], 98)
            base = cen + u * sm
            free0 = _valid_at(ipm, valid, base + nrm * (t0 - 1.5 * ipm.res))
            free1 = _valid_at(ipm, valid, base + nrm * (t1 + 1.5 * ipm.res))
            if free0 and free1:
                tc = (t0 + t1) / 2.0
            elif free0:
                tc = t0 + half
            elif free1:
                tc = t1 - half
            else:
                continue
            pts.append(base + nrm * tc)
        if len(pts) < 2:
            return None
        pts = np.array(pts)
        # doğru uydurma merkez noktalarına (yanal ofset ve yön robotun x = 0'ında)
        pc = pts.mean(axis=0)
        w2, v2 = np.linalg.eigh(np.cov((pts - pc).T))
        u = v2[:, 1] if v2[0, 1] >= 0 else -v2[:, 1]
        cen = pc
        s = (g - cen) @ u
        heading = math.atan2(u[1], u[0])
        lateral = float(cen[1] - cen[0] * u[1] / u[0]) if abs(u[0]) > 1e-6 else float('nan')
        o = np.dot(cen, u)
        near, far = float(o + s.min()), float(o + s.max())
        end_xy = cen + u * s.max()
        beyond = ipm.to_ipm([end_xy + u * c['end_probe']])[0]
        bc, br = int(round(beyond[0])), int(round(beyond[1]))
        end_visible = 0 <= br < ipm.H and 0 <= bc < ipm.W and bool(valid[br, bc])
        seg = LaneSeg(color, pts, u, cen, length, width, lateral, heading, near, far, end_visible)
        seg.confidence = float(np.clip(length / c['confident_length'], 0, 1))
        if end_visible:
            seg.stop_bar_distance = self._stop_bar(ipm, white, end_xy, u)
        return seg

    def _stop_bar(self, ipm, white, end_xy, u):
        """Şerit ucunda, şeride dik beyaz bant → doğrultu boyunca ortasının base'ten uzaklığı (yoksa NaN)."""
        c = self.cfg
        n = np.array([-u[1], u[0]])
        rr, cc = np.nonzero(white)
        if len(rr) == 0:
            return float('nan')
        g = ipm.to_ground(np.column_stack([cc, rr]))
        a = (g - end_xy) @ u
        b = (g - end_xy) @ n
        sel = (a > -c['stop_bar_search'][0]) & (a < c['stop_bar_search'][1]) & (np.abs(b) < c['stop_bar_half_len'])
        if sel.sum() * c['res'] ** 2 < c['stop_bar_min_area_m2']:
            return float('nan')
        across = np.percentile(b[sel], 95) - np.percentile(b[sel], 5)
        along = np.percentile(a[sel], 95) - np.percentile(a[sel], 5)
        if across < 2 * along:                       # dik bant değil
            return float('nan')
        return float(np.dot(end_xy, u) + a[sel].mean())


def _line_kernel(L, angle_deg):
    """Doğrultusu robot ekseninden angle_deg (IPM'de dikey = robot ekseni) olan çizgi çekirdeği."""
    k = np.zeros((L, L), np.uint8)
    c = (L - 1) / 2
    a = math.radians(angle_deg)
    dx, dy = -math.sin(a), -math.cos(a)
    p0 = (int(round(c - dx * c)), int(round(c - dy * c)))
    p1 = (int(round(c + dx * c)), int(round(c + dy * c)))
    cv2.line(k, p0, p1, 1, 1)
    return k


def _valid_at(ipm, valid, xy):
    ij = ipm.to_ipm([xy])[0]
    c, r = int(round(ij[0])), int(round(ij[1]))
    return 0 <= r < ipm.H and 0 <= c < ipm.W and bool(valid[r, c])
