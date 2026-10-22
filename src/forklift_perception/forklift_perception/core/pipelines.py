"""
pipelines.py — kamera başına algılama boru hattı (ROS'tan bağımsız; düğümler ve çevrimdışı
değerlendirme aynı kodu çalıştırır)

FrontPipeline (ön kamera): tabelalar, koliler ve mühürler.
LanePipeline (şerit kamerası): hedef çiftleri, bölme etiketleri, şeritler, çatalda yük.
  Şerit türü hedef çiftlerine göre: çiftin ekseninde ve ucu ön çizgide biten şerit bölme
  şerididir; istasyonun içine uzanan renkli bant (yan bantlar) atılır; geri kalanı zemin şerididir.
"""

import math

import numpy as np
import yaml

from forklift_perception.core import catalog as catalog_mod
from forklift_perception.core.bay_labels import BayLabelDetector
from forklift_perception.core.bay_targets import BayTargetDetector
from forklift_perception.core.colors import ColorClasses
from forklift_perception.core.lanes import LaneDetector
from forklift_perception.core.parcels import LoadDetector, ParcelDetector
from forklift_perception.core.robot_geometry import RobotGeometry, hull_mask
from forklift_perception.core.signs import SignDetector

KIND_FLOOR, KIND_BAY = 0, 1


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


class FrontPipeline:
    def __init__(self, catalog, cfg):
        self.colors = ColorClasses(cfg['colors'])
        self.signs = SignDetector(catalog, self.colors, cfg['signs'])
        self.parcels = ParcelDetector(catalog, self.colors, cfg['parcels'])

    def process(self, rgb, cam):
        return {'signs': self.signs.detect(rgb, cam), 'parcels': self.parcels.detect(rgb, cam)}


class LanePipeline:
    def __init__(self, catalog, robot_geometry, cfg):
        self.cat, self.rg, self.cfg = catalog, robot_geometry, cfg
        self.colors = ColorClasses(cfg['colors'])
        self.targets = BayTargetDetector(catalog, cfg['bay_targets'])
        self.labels = BayLabelDetector(catalog, self.colors, cfg['bay_labels'])
        self.lanes = LaneDetector(self.colors, cfg['lanes'])
        self.load = LoadDetector(self.colors, robot_geometry, cfg['load'])

    def process(self, rgb, cam, q):
        loaded, kraft = self.load.detect(rgb, cam, q)
        parts = self.rg.fork_boxes(q) + ([self.rg.carried_parcel_box(q)] if loaded else [])
        self_mask = hull_mask(cam, parts, dilate=2)
        pairs, _ = self.targets.detect(rgb, cam, ~self_mask)
        labels = self.labels.detect(rgb, cam, ~self_mask)
        # tekerlekler istasyonun önlük plakasındaysa gövde sarhoş teker etrafında burun yukarı döner
        # (D-46; S6'da −0.41° ölçüldü). Ön çizgiye CV mesafesi önlük derinliğinden kısaysa ölçüm
        # eğimli kamera modeliyle yinelenir.
        pitch = self.apron_pitch(pairs, labels)
        if pitch:
            camp = cam.pitched(pitch, self.rg.caster_x)
            pairs, _ = self.targets.detect(rgb, camp, ~self_mask)
            labels = self.labels.detect(rgb, camp, ~self_mask)
            # düz zemin çerçevesinden gövde çerçevesine (çatallar ve odometri gövdeyle döner)
            to_body = _level_to_body(pitch, self.rg.caster_x)
            for p in pairs:
                p.left.p, p.right.p = to_body(p.left.p), to_body(p.right.p)
                p.entry_xy = to_body(np.array([*p.entry_xy, 0.0]))[:2]
            for lb in labels:
                lb.label.p = to_body(lb.label.p)
                lb.entry_xy = to_body(np.array([*lb.entry_xy, 0.0]))[:2]
        segs, _ = self.lanes.detect(rgb, cam, self_mask)
        segs = self.classify(segs, pairs, cam)
        return {'pairs': pairs, 'labels': labels, 'lanes': segs, 'loaded': loaded, 'kraft_fraction': kraft,
                'pitch': pitch}

    def apron_pitch(self, pairs, labels):
        """Önlükteyse beklenen gövde eğimi [rad], değilse 0.

        Ön çizgiye mesafe hedef çiftinden, yoksa etiketten. Plaka istasyonun genişliği kadardır:
        robotun önündeki bölmenin (eksenine en yakın) solunda / sağında komşu bölme varsa o taraftaki
        tahrik tekerleği plakadadır. İki teker plakada: tam eğim atan(kalınlık / |sarhoş teker x|)
        (S6: −0.41°); bir teker: yarısı (yan bölmeler, F2'de ölçüldü)."""
        c = self.cfg['apron']
        items = [(p.distance, p.entry_xy) for p in pairs] + [(lb.distance, lb.entry_xy) for lb in labels]
        if not items:
            return 0.0
        dist = min(d for d, _ in items)
        if not 0.0 < dist < self.cat.apron_depth - c['edge_margin']:
            return 0.0
        ys = [e[1] for _, e in items]
        y0 = min(ys, key=abs)                                   # önümüzdeki bölmenin ekseni
        pitch = self.cat.bay_pitch
        left = any(abs(y - y0 - pitch) < c['neighbour_tol'] for y in ys)
        right = any(abs(y - y0 + pitch) < c['neighbour_tol'] for y in ys)
        wheels = int(left) + int(right)
        return math.atan2(self.cat.plate_thickness, -self.rg.caster_x) * wheels / 2.0

    def classify(self, segs, pairs, cam):
        c = self.cfg['lane_kind']
        out = []
        for s in segs:
            kind, drop = KIND_FLOOR, False
            for p in pairs:
                n = np.array([math.cos(p.theta), math.sin(p.theta)])
                t = np.array([-n[1], n[0]])
                lat = float(np.dot(s.centroid - p.entry_xy, t))
                far = float(np.dot(s.points[-1] - p.entry_xy, n)) if len(s.points) else -1.0
                dh = abs(math.degrees((s.heading - p.theta + math.pi) % (2 * math.pi) - math.pi))
                # bölme eksenleri bulunan çiftten bay_pitch aralıklı (çifti görünmeyen komşu bölmeler de)
                k = round(lat / self.cat.bay_pitch)
                lat_axis = lat - k * self.cat.bay_pitch
                if abs(k) <= c['max_bays_side'] and abs(lat_axis) < c['bay_lateral_tol'] \
                        and dh < c['bay_heading_tol_deg'] and c['bay_far_range'][0] <= far <= c['bay_far_range'][1]:
                    kind = KIND_BAY
                    break
                # istasyonun renkli bandı: içine uzanan ya da (uç çıkıntının altında kaldığı için)
                # ön çizgide biten, ama bölme ekseninde olmayan şerit. Zemin şeritleri önlükten
                # önce (ön çizgiye ~0.4 m) biter.
                if abs(lat) < c['station_half_width'] and far > c['band_far_min']:
                    drop = True
            if drop and kind != KIND_BAY:
                continue
            s.kind = kind
            if kind == KIND_BAY:
                # bölme şeridi plaka üstünde (plane_z yerine plate_top_z): noktaları ışın boyunca düzelt
                s.points = _replane(cam, s.points, self.cfg['lanes']['plane_z'], self.cat.plate_top_z)
            out.append(s)
        return out


def _level_to_body(angle, pivot_x):
    """Düz zemin çerçevesindeki nokta → pivot_x etrafında angle (burun yukarı) dönmüş gövde çerçevesi."""
    c, s_ = math.cos(-angle), math.sin(-angle)
    Rp = np.array([[c, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c]])
    P = np.array([pivot_x, 0.0, 0.0])
    return lambda p: P + Rp.T @ (np.asarray(p, float) - P)


def _replane(cam, pts_xy, z0, z1):
    """z0 düzlemindeki noktaları kamera ışını boyunca z1 düzlemine taşır."""
    if len(pts_xy) == 0:
        return pts_xy
    P = np.column_stack([pts_xy, np.full(len(pts_xy), z0)])
    k = (z1 - cam.t[2]) / (z0 - cam.t[2])
    return (cam.t + (P - cam.t) * k)[:, :2]


def build(catalog_file, robot_params_file, detectors_file):
    """Dosya yollarından iki boru hattı (düğümler ve değerlendirme için)."""
    cat = catalog_mod.load(catalog_file)
    cfg = load_config(detectors_file)
    rg = RobotGeometry.load(robot_params_file, cat.parcel_size)
    return cat, FrontPipeline(cat, cfg), LanePipeline(cat, rg, cfg)
