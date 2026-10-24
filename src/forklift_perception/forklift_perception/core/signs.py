"""
signs.py — istasyon tabelası: kimlik (renk bloğu) ve istasyonun robota göre pozu (ön kamera)

Tabela: robota bakan dikey levha (katalog: genişlik sign_width, yükseklik sign_height,
üst kenar sign_top_z, yüz station x = sign_face_x). Solunda (robotun bakışıyla +y)
kenarı tabela yüksekliği kadar kare renk bloğu: kırmızı / yeşil / mavi; kaynakta koyu nötr.
İki yol (sonuçlar birleştirilir, aynı tabela bir kez):
 A. Beyaz panel çapası: panel ≈ 3:1 dikdörtgen; sola uzatılarak blok bulunur, renk oradan
    (kaynak tabelasının koyu bloğu koyu arka planla birleştiği için gerekli).
 B. Renk bloğu:
 1. Blok adayları: renk maskelerinin dış konturları (içindeki beyaz koli simgesi dolar) → dörtgen.
 2. Poz: tabela dikey ve yüksekliği bilindiği için yalnız (x, y, yaw) bilinmiyor; blok
    köşeleri ile Gauss-Newton (3 serbestlik). Serbest PnP'ye göre yaw daha kararlı.
    Sonra tabelanın sağ kenarı (yazısız iki satırda beyaz panelin bittiği yer) ölçüme
    eklenir: taban 15 cm'den 60 cm'ye çıkar.
 3. Denetim: blok merkezinin yüksekliği ve yeniden izdüşüm hatası; bloğun sağında
    (tabela düzleminde) beyaz zemin.
Çıktı: renk, istasyon çerçevesinin base_footprint'teki pozu (x, y, yaw), tabela köşeleri.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

BLOCK_COLORS = ('red', 'green', 'blue', 'neutral')


@dataclass
class SignDet:
    color: str
    station_xy: np.ndarray
    station_yaw: float
    corners_px: np.ndarray        # tabelanın tamamı (izdüşüm): sol üst, sağ üst, sağ alt, sol alt
    block_px: np.ndarray
    reproj_px: float
    white_fraction: float
    confidence: float


class SignDetector:
    def __init__(self, catalog, colors, cfg):
        self.cat, self.colors, self.cfg = catalog, colors, cfg
        h, w = catalog.sign_height, catalog.sign_width
        x, top = catalog.sign_face_x, catalog.sign_top_z
        # station çerçevesinde blok köşeleri: sol üst, sağ üst, sağ alt, sol alt (robotun bakışıyla)
        yl = w / 2.0
        self.block_st = np.array([[x, yl, top], [x, yl - h, top], [x, yl - h, top - h], [x, yl, top - h]])
        self.sign_st = np.array([[x, yl, top], [x, -yl, top], [x, -yl, top - h], [x, yl, top - h]])
        self.panel_st = np.array([[x, yl - h * 1.15, top - h * 0.1], [x, -yl + h * 0.15, top - h * 0.1],
                                  [x, -yl + h * 0.15, top - h * 0.9], [x, yl - h * 1.15, top - h * 0.9]])

    def detect(self, rgb, cam):
        c = self.cfg
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        white = self.colors.mask(hsv, 'white_sign') > 0
        # panel çapası iki maskeyle: geniş beyaz ve orta parlaklık bandı (panelden parlak bir arka plan,
        # ör. station_test'in açık gri gökyüzü, panelle birleşmesin)
        out = self._detect_panel(hsv, cam, white)
        out += self._detect_panel(hsv, cam, self.colors.mask(hsv, 'white_panel_mid') > 0, white)
        for color in BLOCK_COLORS:
            m = self.colors.mask(hsv, 'dark_neutral' if color == 'neutral' else color)
            if color == 'neutral':
                # koyu blok, tabelanın ince koyu kenar çizgisi ve koyu arka planla birleşmesin
                m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) < c['min_block_area_px']:
                    continue
                quad = _quad(cnt)
                if quad is None:
                    continue
                det = self._fit(cam, quad, color, white)
                if det is not None:
                    out.append(det)
        return _dedupe(out)

    def _detect_panel(self, hsv, cam, white, white_check=None):
        """Beyaz panel çapası (özellikle kaynak tabelası: koyu nötr blok koyu arka planla birleşir).
        Panel ≈ 3h × h dikdörtgen; üst ve alt kenarları sola panel genişliğinin 1/3'ü kadar uzatılınca
        bloğun köşeleri bulunur; poz dört tabela köşesinden, renk sol karenin piksellerinden."""
        c = self.cfg
        # kapama yok: tabelanın 1 px'lik koyu kenar çizgisi paneli arka plandan ayırır; yazının
        # delikleri dış konturda zaten dolar
        contours, _ = cv2.findContours(white.astype(np.uint8) * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        zc = self.cat.sign_top_z - self.cat.sign_height / 2.0
        k = (self.cat.sign_width - self.cat.sign_height) / self.cat.sign_height    # panel en/boy (3)
        out = []
        for cnt in contours:
            if cv2.contourArea(cnt) < c['min_panel_area_px']:
                continue
            q = _quad(cnt)
            if q is None:
                continue
            q = _order_tl(q)
            wpx = np.linalg.norm(q[1] - q[0]) + np.linalg.norm(q[2] - q[3])
            hpx = np.linalg.norm(q[3] - q[0]) + np.linalg.norm(q[2] - q[1])
            if not c['panel_aspect'][0] <= wpx / max(hpx, 1e-6) <= c['panel_aspect'][1]:
                continue
            tl = q[0] - (q[1] - q[0]) / k
            bl = q[3] - (q[2] - q[3]) / k
            sq = np.array([tl, q[1], q[2], bl])
            p, ok = cam.pixels_to_plane([sq.mean(axis=0)], zc)
            if not ok[0]:
                continue
            yaw0 = math.atan2(p[0][1], p[0][0])
            x0 = p[0][:2] - _rot(yaw0) @ self.sign_st.mean(axis=0)[:2]
            params, err = self._gauss_newton(cam, sq, np.array([x0[0], x0[1], yaw0]), corners_st=self.sign_st)
            if params is None or err > c['max_reproj_px'] * 2:
                continue
            color, frac = self._block_color(hsv, self._project(cam, params, self.block_st))
            if color is None:
                continue
            wf = _fraction(white if white_check is None else white_check, self._project(cam, params, self.panel_st))
            if wf < c['min_white_fraction']:
                continue
            conf = float(np.clip(wf, 0, 1) * frac * 0.8)
            out.append(SignDet(color, params[:2].copy(), float(params[2]), sq,
                               self._project(cam, params, self.block_st), float(err), float(wf), conf))
        return out

    def _block_color(self, hsv, block_uv):
        """Sol karenin rengi: beyaz koli simgesi dışındaki piksellerin çoğunluğu → (renk, oran)."""
        m = np.zeros(hsv.shape[:2], np.uint8)
        if not np.all(np.isfinite(block_uv)):
            return None, 0.0
        cv2.fillConvexPoly(m, np.round(block_uv).astype(np.int32), 255)
        m = cv2.erode(m, np.ones((3, 3), np.uint8)) > 0
        px = hsv[m]
        if len(px) < 10:
            return None, 0.0
        px = px[(px[:, 1] > 60) | (px[:, 2] < 120)]             # simgenin beyazı hariç
        if len(px) < 10:
            return None, 0.0
        names = {'red': 'red', 'green': 'green', 'blue': 'blue', 'dark_neutral': 'neutral'}
        fr = self.colors.classify_pixels(px, list(names))
        best = max(fr, key=fr.get)
        if fr[best] < self.cfg['min_block_fraction']:
            return None, 0.0
        return names[best], fr[best]

    def _fit(self, cam, quad, color, white):
        c = self.cfg
        q = _order_tl(quad)
        # ilk tahmin: blok merkezinin ışını ile blok yüksekliği düzleminin kesişimi
        zc = self.cat.sign_top_z - self.cat.sign_height / 2.0
        p, ok = cam.pixels_to_plane([q.mean(axis=0)], zc)
        if not ok[0]:
            return None
        bc = self.block_st.mean(axis=0)
        yaw0 = math.atan2(p[0][1], p[0][0])             # istasyon kabaca robota bakıyor
        x0 = p[0][:2] - _rot(yaw0) @ bc[:2]
        params = np.array([x0[0], x0[1], yaw0])
        params, err = self._gauss_newton(cam, q, params)
        if params is None or err > c['max_reproj_px']:
            return None
        # sağ kenar: yazının olmadığı iki satırda (yüksekliğin %9 ve %91'i) beyaz panelin bittiği yer;
        # taban 15 cm'den 60 cm'ye çıkar, yaw kestirimi iyileşir
        edges = self._right_edge(cam, params, white)
        if len(edges) == 2:
            p2, err2 = self._gauss_newton(cam, q, params, edges)
            if p2 is not None and err2 <= c['max_reproj_px']:
                params, err = p2, err2
        # blok merkezinin yüksekliği zaten sabit; yön kameraya bakmalı (yüz normali robota doğru)
        R = _rot(params[2])
        normal = -R[:, 0]                                # yüz −x yönüne bakar
        to_cam = cam.t[:2] - (R @ bc[:2] + params[:2])
        if np.dot(normal, to_cam) <= 0:
            return None
        sign_uv = self._project(cam, params, self.sign_st)
        panel_uv = self._project(cam, params, self.panel_st)
        wf = _fraction(white, panel_uv)
        if wf < c['min_white_fraction']:
            return None
        conf = float(np.clip(wf, 0, 1) * np.clip(1 - err / c['max_reproj_px'], 0, 1))
        return SignDet(color, params[:2].copy(), float(params[2]), sign_uv, q, float(err), float(wf), conf)

    def _right_edge(self, cam, params, white):
        """[(station noktası, ölçülen u)]: tabela sağ kenarının iki satırdaki görüntü konumu."""
        h, w = self.cat.sign_height, self.cat.sign_width
        x, top = self.cat.sign_face_x, self.cat.sign_top_z
        out = []
        for f in self.cfg['edge_rows']:
            z = top - f * h
            pts = np.array([[x, -w / 2 + h * 0.6, z], [x, -w / 2, z]])
            uv = self._project(cam, params, pts)
            if not np.all(np.isfinite(uv)):
                continue
            v = int(round(uv[1, 1]))
            u_in, u_pred = uv[0, 0], uv[1, 0]
            span = abs(u_pred - u_in) / 0.6                         # bir tabela yüksekliği [px]
            u0, u1 = int(math.floor(u_in)), int(math.ceil(u_pred + 0.8 * span))
            if not (0 <= v < white.shape[0]) or u0 < 0 or u1 >= white.shape[1] or u1 - u0 < 4:
                continue
            row = white[v, u0:u1 + 1].astype(float)
            run = np.nonzero(row < 0.5)[0]                         # panelin içinden sağa: ilk beyaz olmayan
            if len(run) == 0 or run[0] < 2:
                continue
            u_edge = u0 + run[0] - 0.5                              # son beyaz pikselin sağ kenarı
            if abs(u_edge - u_pred) > 0.5 * span:
                continue
            out.append((pts[1], u_edge))
        return out

    def _project(self, cam, params, pts_st):
        R = _rot(params[2])
        pts = np.column_stack([pts_st[:, :2] @ R.T + params[:2], pts_st[:, 2]])
        return cam.project(pts)[0]

    def _gauss_newton(self, cam, q, params, edges=(), iters=15, corners_st=None):
        """Dört köşe (u, v; varsayılan blok köşeleri) ve isteğe bağlı kenar noktaları (yalnız u) üzerinden
        (x, y, yaw)."""
        ep = np.array([e[0] for e in edges]).reshape(-1, 3)
        eu = np.array([e[1] for e in edges])
        cst = self.block_st if corners_st is None else corners_st

        def resid(pr):
            uv = self._project(cam, pr, cst)
            r = (uv - q).ravel()
            if len(eu):
                r = np.concatenate([r, self._project(cam, pr, ep)[:, 0] - eu])
            return r
        for _ in range(iters):
            r = resid(params)
            if not np.all(np.isfinite(r)):
                return None, np.inf
            J = np.zeros((len(r), 3))
            for k, h in enumerate((1e-4, 1e-4, 1e-4)):
                d = np.zeros(3)
                d[k] = h
                J[:, k] = (resid(params + d) - r) / h
            # sonlu farkta bir nokta kameranın arkasına düşerse J NaN olur; lstsq istisna atar ve
            # (F4 A2'de) ön algı düğümünü kapatıyordu
            if not np.all(np.isfinite(J)):
                return None, np.inf
            try:
                step, *_ = np.linalg.lstsq(J, -r, rcond=None)
            except np.linalg.LinAlgError:
                return None, np.inf
            params = params + step
            if np.linalg.norm(step) < 1e-7:
                break
        r = resid(params)
        if not np.all(np.isfinite(r)):
            return None, np.inf
        return params, float(np.sqrt(np.mean(r[:8].reshape(4, 2) ** 2) * 2))


def _rot(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def _quad(cnt):
    hull = cv2.convexHull(cnt)
    peri = cv2.arcLength(hull, True)
    for eps in (0.02, 0.04, 0.06):
        ap = cv2.approxPolyDP(hull, eps * peri, True)
        if len(ap) == 4:
            return ap.reshape(4, 2).astype(float)
    return None


def _order_tl(q):
    """Köşeler: sol üst, sağ üst, sağ alt, sol alt (görüntüde)."""
    s = q.sum(axis=1)
    d = q[:, 0] - q[:, 1]
    tl, br = q[np.argmin(s)], q[np.argmax(s)]
    tr, bl = q[np.argmax(d)], q[np.argmin(d)]
    return np.array([tl, tr, br, bl])


def _fraction(mask, uv):
    if not np.all(np.isfinite(uv)):
        return 0.0
    m = np.zeros(mask.shape, np.uint8)
    cv2.fillConvexPoly(m, np.round(uv).astype(np.int32), 255)
    inside = m > 0
    return float(mask[inside].mean()) if inside.any() else 0.0


def _dedupe(dets):
    """Aynı tabela iki kez bulunursa (ör. renk maskesi bölünmüş) güveni yüksek olan kalır."""
    out = []
    for d in sorted(dets, key=lambda d: -d.confidence):
        if all(np.linalg.norm(d.station_xy - o.station_xy) > 0.2 for o in out):
            out.append(d)
    return out
