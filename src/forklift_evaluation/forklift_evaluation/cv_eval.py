"""
cv_eval.py — algılama kabul testleri C0–C7, C9 (kayıtlı kümeler üzerinde; plan §8)

Algılama düğümlerinin çalıştırdığı boru hatları (forklift_perception.core.pipelines) aynen
çalıştırılır; sonuçlar GT ile karşılaştırılır. GT izdüşümü ve algılama aynı kalibre edilmiş
kamera modelini kullanır (D-59). Gürültü (C7) görüntüye eklenir: σ · 255 gri düzey, sabit tohum.

  ros2 run forklift_evaluation cv_eval [--sigma 0,0.007,0.02] [--calib DIR] [--out docs/logs/cv/f2]
C8 (canlı performans) ve C0'ın çalışma anı kısmı perf_probe ile koşulur.
Çıktı: <out>/cv_results.json, <out>/cv_results.md (Türkçe tablo).
"""

import argparse
import json
import math
import os
import re
import time
from collections import Counter, defaultdict

import numpy as np
from ament_index_python.packages import get_package_share_directory

from forklift_evaluation.dataset import Dataset
from forklift_evaluation.world_gt import EvalSetup, WorldGT
from forklift_perception.core import pipelines
from forklift_perception.core.robot_geometry import hull_mask

PASS, FAIL, NOT_RUN, INFO = 'GEÇTİ', 'KALDI', 'ÇALIŞTIRILAMADI', 'BİLGİ'
WORLDS = ('station_test', 'factory')


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def noisy(rgb, sigma, rng):
    if sigma <= 0:
        return rgb
    n = rng.normal(0.0, sigma * 255.0, rgb.shape)
    return np.clip(rgb.astype(float) + n, 0, 255).astype(np.uint8)


# =============================================================================
#  GT yardımcıları (kare başına, base_footprint'te)
# =============================================================================
def gt_bays(gt, fr, cat):
    """Her bölme: giriş noktası (ön çizgi x = 0, kanal ekseni), eksen yönü, iki hedefin plakası."""
    pairs = defaultdict(dict)
    for t in gt.align_targets():
        pairs[(t['station'], t['bay'])][t['side']] = t
    out = []
    for key, lr in pairs.items():
        l, r = fr.to_base([lr['left']['center']])[0], fr.to_base([lr['right']['center']])[0]
        d = l[:2] - r[:2]
        n = np.array([d[1], -d[0]]) / np.linalg.norm(d)
        mid = (l[:2] + r[:2]) / 2
        if np.dot(n, mid) < 0:
            n = -n
        entry = mid - n * cat.target_face_x
        out.append({'key': key, 'entry': entry, 'theta': math.atan2(n[1], n[0]), 'n': n,
                    'plates': [fr.to_base(lr['left']['plate']), fr.to_base(lr['right']['plate'])]})
    return out


def plate_visible(cam, plate, self_mask):
    uv, front, inside = cam.project(plate)
    if not inside.all():
        return False
    c = np.round(uv.mean(axis=0)).astype(int)
    return not self_mask[c[1], c[0]]


def self_mask_for(lane, cam, fr):
    rg = lane.rg
    parts = rg.fork_boxes(fr.fork_q) + ([rg.carried_parcel_box(fr.fork_q)] if fr.rec.get('load') else [])
    return hull_mask(cam, parts, dilate=4)


# =============================================================================
#  Testler
# =============================================================================
class Evaluator:
    def __init__(self, data_root, calib, out):
        share_g = get_package_share_directory('forklift_gazebo')
        share_p = get_package_share_directory('forklift_perception')
        share_d = get_package_share_directory('forklift_description')
        self.cat, self.front, self.lane = pipelines.build(
            os.path.join(share_g, 'config', 'station_params.yaml'),
            os.path.join(share_d, 'config', 'forklift_params.yaml'),
            os.path.join(share_p, 'config', 'detectors.yaml'))
        self.calib = calib
        self.out = out
        self.ds, self.gt = {}, {}
        for w in WORLDS:
            for root in ('f1', 'f2'):
                d = os.path.join(data_root, root, w)
                if os.path.isdir(d):
                    self.ds[(root, w)] = Dataset(d, calib)
            self.gt[w] = WorldGT(EvalSetup(w, os.path.join(out, 'tmp')))

    def frames(self, root, world, sets, every=1):
        ds = self.ds.get((root, world))
        if ds is None:
            return []
        return [f for k, f in enumerate(ds.frames(sets)) if k % every == 0]

    # ----------------------------------------------------------------- C1 tabela
    def c1(self, sigma, every):
        rng = np.random.default_rng(1)
        rows, n_gt, n_det, fp, wrong = [], 0, 0, 0, 0
        by_band = defaultdict(lambda: [0, 0])
        for w in WORLDS:
            es, gt = self.gt[w].es, self.gt[w]
            for root, sets in (('f1', {'stations'}), ('f2', {'far'})):
                for fr in self.frames(root, w, sets, every):
                    cam = fr.ds.cams['front_camera']
                    dets = self.front.signs.detect(noisy(fr.rgb('front_camera'), sigma, rng), cam)
                    gts = []
                    for s in gt.signs():
                        stn = es.station(s['station'])
                        o = fr.to_base([[stn['pose'][0], stn['pose'][1], 0.0]])[0][:2]
                        yaw = _wrap(stn['pose'][2] - math.atan2(fr.bf.R[1, 0], fr.bf.R[0, 0]))
                        gts.append((stn['color'], o, yaw, cam.project(fr.to_base(s['quad']))[2].all()))
                    for d in dets:
                        g = min(gts, key=lambda g: np.linalg.norm(g[1] - d.station_xy))
                        if np.linalg.norm(g[1] - d.station_xy) > 0.3:
                            fp += 1
                        elif g[0] != d.color:
                            wrong += 1
                    for color, o, yaw, vis in gts:
                        if not vis:
                            continue
                        dist = float(np.linalg.norm(o))
                        band = ('<1.5' if dist < 1.5 else '1.5–2.5' if dist < 2.5
                                else '2.5–3.5' if dist < 3.5 else '≥3.5')
                        n_gt += 1
                        by_band[band][0] += 1
                        m = [d for d in dets if np.linalg.norm(o - d.station_xy) < 0.3]
                        if m:
                            n_det += 1
                            by_band[band][1] += 1
                            rows.append((w, color, band, dist, float(np.linalg.norm(m[0].station_xy - o)),
                                         math.degrees(abs(_wrap(m[0].station_yaw - yaw)))))
        r = {'gt': n_gt, 'detected': n_det, 'false_pos': fp, 'wrong_color': wrong,
             'by_band': {b: {'gt': v[0], 'detected': v[1]} for b, v in sorted(by_band.items())},
             'by_color': dict(Counter((x[1]) for x in rows)), 'pose': {}}
        for band in sorted({x[2] for x in rows}):
            s = [x for x in rows if x[2] == band]
            r['pose'][band] = {'n': len(s), 'pos_mm_median': float(np.median([x[4] for x in s]) * 1000),
                               'pos_mm_max': float(max(x[4] for x in s) * 1000),
                               'yaw_deg_median': float(np.median([x[5] for x in s])),
                               'yaw_deg_max': float(max(x[5] for x in s))}
        le3 = [x for x in rows if x[3] <= 3.0]
        r['pose_le3m'] = {'pos_mm_max': float(max((x[4] for x in le3), default=0) * 1000),
                          'yaw_deg_max': float(max((x[5] for x in le3), default=0)),
                          'pos_mm_p95': float(np.percentile([x[4] for x in le3], 95) * 1000) if le3 else 0,
                          'yaw_deg_p95': float(np.percentile([x[5] for x in le3], 95)) if le3 else 0}
        ok = wrong == 0 and fp == 0 and r['pose_le3m']['pos_mm_max'] <= 50 and r['pose_le3m']['yaw_deg_max'] <= 5
        return r, ok

    # ----------------------------------------------------------------- C2/C3 şeritler
    def lanes(self, sigma, every, floor):
        rng = np.random.default_rng(2)
        errs, n_gt, n_det, fp, wrong_kind, stop = [], 0, 0, 0, 0, []
        worlds = ('factory',) if floor else WORLDS
        sets = {'lanes'} if floor else {'stations'}
        for w in worlds:
            gt = self.gt[w]
            G = [(lane['color'], lane['poly'], 'floor', lane['stop_bar']) for lane in gt.floor_lanes()] + \
                [(b['color'], b['poly'], 'bay', None) for b in gt.bay_lines()]
            for fr in self.frames('f1', w, sets, every):
                cam = fr.ds.cams['lane_camera']
                r = self.lane.process(noisy(fr.rgb('lane_camera'), sigma, rng), cam, fr.fork_q)
                segs = r['lanes']
                ipm = self.lane.lanes.ipm(cam)
                sm = self_mask_for(self.lane, cam, fr)
                gl = []
                for color, poly, kind, bar in G:
                    pb = fr.to_base(poly)
                    a, b = (pb[0] + pb[3]) / 2, (pb[1] + pb[2]) / 2
                    d = (b - a)[:2] / np.linalg.norm((b - a)[:2])
                    if d[0] < 0:
                        d = -d
                    pts = a[None, :2] + np.linspace(0, 1, 40)[:, None] * (b - a)[None, :2]
                    ij = np.round(ipm.to_ipm(pts)).astype(int)
                    vis = []
                    for (c_, r_), p in zip(ij, pts):
                        ok = 0 <= c_ < ipm.W and 0 <= r_ < ipm.H and ipm.valid[r_, c_]
                        if ok:
                            uv, _, ins = cam.project([[p[0], p[1], 0.0]])
                            ok = ins[0] and not sm[int(round(uv[0, 1])), int(round(uv[0, 0]))]
                        vis.append(ok)
                    vis_len = float(np.mean(vis)) * float(np.linalg.norm((b - a)[:2]))
                    s0, s1 = sorted((float(np.dot(a[:2], d)), float(np.dot(b[:2], d))))
                    gl.append((color, kind, a[:2], d, vis_len, None if bar is None else fr.to_base(bar).mean(axis=0),
                               (s0, s1)))
                want_kind = 0 if floor else 1
                for s in segs:
                    best = None
                    for g in gl:
                        if g[0] != s.color:
                            continue
                        perp = float(np.dot(s.centroid - g[2], [-g[3][1], g[3][0]]))
                        # boyuna örtüşme: aynı doğrudaki başka şeritle (orta bölme ↔ zemin şeridi) eşleşmesin
                        ds = sorted(float(np.dot(q, g[3])) for q in (s.points[0], s.points[-1]))
                        ov = min(ds[1], g[6][1]) - max(ds[0], g[6][0])
                        if ov < 0.5 * max(ds[1] - ds[0], 0.02):
                            continue
                        if abs(perp) < 0.03 and (best is None or abs(perp) < abs(best[0])):
                            best = (perp, g)
                    if best is None:
                        fp += 1
                        continue
                    g = best[1]
                    if (g[1] == 'floor') != floor:
                        continue
                    if s.kind != want_kind:
                        wrong_kind += 1
                    dh = math.degrees(abs(_wrap(s.heading - math.atan2(g[3][1], g[3][0]))))
                    if s.length >= 0.15 and np.linalg.norm(s.centroid) <= 2.0:   # ölçüt aralığı (plan C2/C3)
                        errs.append((w, abs(best[0]), dh, float(np.linalg.norm(g[2])), s.length))
                    if floor and g[5] is not None and not math.isnan(s.stop_bar_distance):
                        stop.append(abs(s.stop_bar_distance - float(np.dot(g[5][:2], g[3]))))
                for g in gl:
                    ang = abs(math.degrees(math.atan2(g[3][1], g[3][0])))
                    if ((g[1] == 'floor') == floor) and g[4] >= 0.15 and ang < 55:
                        n_gt += 1
                        if any(s.color == g[0] and abs(np.dot(s.centroid - g[2], [-g[3][1], g[3][0]])) < 0.03
                               and g[6][0] - 0.05 <= float(np.dot(s.centroid, g[3])) <= g[6][1] + 0.05 for s in segs):
                            n_det += 1
        e = np.array([(x[1], x[2]) for x in errs]) if errs else np.zeros((0, 2))
        # yön ölçütü ≥ 25 cm görünen şeritlerde (15–25 cm'lik parçada ~1.5 mm nokta gürültüsü tek başına
        # ~1°; kısa parçalar bilgi olarak ayrıca verilir)
        eh = np.array([x[2] for x in errs if x[4] >= 0.25])
        es_ = np.array([x[2] for x in errs if x[4] < 0.25])
        r = {'gt_visible_ge_15cm': n_gt, 'detected': n_det, 'detection_rate': n_det / max(n_gt, 1),
             'false_pos_other_color_or_band': fp, 'wrong_kind': wrong_kind,
             'lateral_mm_median': float(np.median(e[:, 0]) * 1000) if len(e) else None,
             'lateral_mm_p95': float(np.percentile(e[:, 0], 95) * 1000) if len(e) else None,
             'lateral_mm_max': float(e[:, 0].max() * 1000) if len(e) else None,
             'heading_deg_median': float(np.median(eh)) if len(eh) else None,
             'heading_deg_p95': float(np.percentile(eh, 95)) if len(eh) else None,
             'heading_deg_max': float(eh.max()) if len(eh) else None,
             'heading_short_15_25cm_deg_max': float(es_.max()) if len(es_) else None, 'heading_short_n': len(es_),
             'stop_bar_mm_median': float(np.median(stop) * 1000) if stop else None,
             'stop_bar_mm_max': float(max(stop) * 1000) if stop else None, 'stop_bar_n': len(stop)}
        r['note'] = ('hata istatistiği: görünen uzunluğu ≥ 15 cm ve merkezi ≤ 2 m olan şeritler; yön ölçütü ≥ 25 cm; '
                     'tür (zemin/bölme) bilgi amaçlı: hedef çifti görünmeyen karede bölme şeridi zemin sayılır')
        ok = len(e) > 0 and r['lateral_mm_max'] <= 10 and r['heading_deg_max'] <= 2 and r['detection_rate'] >= 0.95
        return r, ok

    # ----------------------------------------------------------------- C4 hizalama
    def c4(self, sigma, every):
        rng = np.random.default_rng(3)
        rows, n_gt, n_det, false_pairs, order_bad = [], 0, 0, 0, 0
        for w in WORLDS:
            for fr in self.frames('f2', w, {'align', 'align_loaded'}, every):
                cam = fr.ds.cams['lane_camera']
                r = self.lane.process(noisy(fr.rgb('lane_camera'), sigma, rng), cam, fr.fork_q)
                pairs = r['pairs']
                sm = self_mask_for(self.lane, cam, fr)
                gb = gt_bays(self.gt[w], fr, self.cat)
                for p in pairs:
                    if min(np.linalg.norm(g['entry'] - p.entry_xy) for g in gb) > 0.03:
                        false_pairs += 1
                ys = [p.entry_xy[1] for p in pairs]
                if ys != sorted(ys, reverse=True):
                    order_bad += 1
                for g in gb:
                    if g['key'][0] != fr.rec['station'] or g['key'][1] != fr.rec.get('bay'):
                        continue
                    if not all(plate_visible(cam, pl, sm) for pl in g['plates']):
                        continue
                    dcam = float(np.dot(g['entry'] - cam.t[:2], g['n']))
                    n_gt += 1
                    m = [p for p in pairs if np.linalg.norm(g['entry'] - p.entry_xy) < 0.03]
                    if not m:
                        continue
                    n_det += 1
                    p = m[0]
                    t = np.array([-g['n'][1], g['n'][0]])
                    rows.append({'world': w, 'set': fr.rec['set'], 'd_cam': dcam,
                                 'lat_mm': float(np.dot(p.entry_xy - g['entry'], t) * 1000),
                                 'along_mm': float(np.dot(p.entry_xy - g['entry'], g['n']) * 1000),
                                 'yaw_deg': math.degrees(_wrap(p.theta - g['theta']))})
        near = [x for x in rows if x['d_cam'] <= 0.9]
        res = {'gt_visible': n_gt, 'detected': n_det, 'pairing_rate': n_det / max(n_gt, 1),
               'yaw_abs_p99_deg': float(np.percentile([abs(x['yaw_deg']) for x in rows], 99)) if rows else None,
               'near_n': len(near),
               'false_pairs': false_pairs, 'order_wrong_frames': order_bad, 'groups': {}}
        groups = {'yüksüz 0.3–1.2 m': lambda x: x['set'] == 'align' and 0.3 <= x['d_cam'] <= 1.2,
                  'yüksüz 0.3–0.9 m (son hizalama, bilgi)': lambda x: x['set'] == 'align' and 0.3 <= x['d_cam'] <= 0.9,
                  'yüksüz 1.2–1.4 m (bilgi)': lambda x: x['set'] == 'align' and x['d_cam'] > 1.2,
                  'yüklü 0.575–1.2 m': lambda x: x['set'] == 'align_loaded' and 0.575 <= x['d_cam'] <= 1.2,
                  'yüklü 0.45–0.575 m (gölge, bilgi)': lambda x: x['set'] == 'align_loaded' and x['d_cam'] < 0.575}
        ok = res['false_pairs'] == 0 and res['order_wrong_frames'] == 0
        for name, f in groups.items():
            s = [x for x in rows if f(x)]
            if not s:
                continue
            g = {k: float(max(abs(x[k]) for x in s)) for k in ('lat_mm', 'along_mm', 'yaw_deg')}
            g.update({f'{k}_mean': float(np.mean([x[k] for x in s])) for k in ('lat_mm', 'along_mm', 'yaw_deg')})
            g['n'] = len(s)
            res['groups'][name] = g
            if 'bilgi' not in name:
                ok = ok and g['lat_mm'] <= 2 and g['yaw_deg'] <= 0.5 and g['along_mm'] <= 5
        # eşleştirme oranı ölçütü: değerlendirilen aralıkta (bilgi grupları hariç)
        ev = [x for x in rows if groups['yüksüz 0.3–1.2 m'](x) or groups['yüklü 0.575–1.2 m'](x)]
        res['evaluated_n'] = len(ev)
        ok = ok and res['pairing_rate'] >= 0.95
        return res, ok

    # ----------------------------------------------------------------- C5 koli
    def c5(self, sigma, every):
        rng = np.random.default_rng(4)
        tp = fn = fp = wrong = 0
        ious = []
        for w in WORLDS:
            gt = self.gt[w]
            for root, sets in (('f1', {'stations'}), ('f2', {'occupancy'})):
                for fr in self.frames(root, w, sets, every):
                    cam = fr.ds.cams['front_camera']
                    dets = self.front.parcels.detect(noisy(fr.rgb('front_camera'), sigma, rng), cam)
                    gts = []
                    for b in gt.parcel_boxes_from(fr.parcels):
                        if abs(b['center'][2] - (self.cat.ledge_height + self.cat.parcel_size[2] / 2)) > 0.03:
                            continue                                   # park edilmiş / çataldaki koli
                        sc = fr.to_base([b['seal'].mean(axis=0)])[0]
                        nrm = fr.bf.inv().R @ (b['seal'].mean(axis=0) - b['center'])
                        vis = cam.project(fr.to_base(b['seal']))[2].all() and np.dot(nrm, sc - cam.t) < 0
                        uv = cam.project(fr.to_base(b['corners']))[0]
                        gts.append((b['color'], sc, vis, uv))
                    used = set()
                    for d in dets:
                        k = min(range(len(gts)), key=lambda i: np.linalg.norm(gts[i][1][:2] - d.position[:2]),
                                default=None)
                        if k is None or np.linalg.norm(gts[k][1][:2] - d.position[:2]) > 0.08:
                            fp += 1
                            continue
                        used.add(k)
                        if gts[k][0] != d.seal_color:
                            wrong += 1
                        uv = gts[k][3]
                        if np.all(np.isfinite(uv)):
                            # GT kutusu görüntüye kırpılır (algılanan kutu görüntünün içindedir)
                            lo = np.clip(uv.min(axis=0), 0, [cam.width - 1, cam.height - 1])
                            hi = np.clip(uv.max(axis=0), 0, [cam.width - 1, cam.height - 1])
                            ious.append((_iou(d.bbox, (*lo, *hi)), fr.id, w))
                    for i, g in enumerate(gts):
                        if g[2]:
                            if i in used:
                                tp += 1
                            else:
                                fn += 1
        r = {'true_pos': tp, 'missed': fn, 'false_pos': fp, 'wrong_color': wrong,
             'recall': tp / max(tp + fn, 1),
             'iou_median': float(np.median([x[0] for x in ious])) if ious else None,
             'iou_p5': float(np.percentile([x[0] for x in ious], 5)) if ious else None,
             'iou_lowest': sorted(((round(x[0], 3), x[2], x[1]) for x in ious))[:8]}
        ok = fn == 0 and fp == 0 and wrong == 0 and (r['iou_p5'] or 0) >= 0.7
        return r, ok

    # ----------------------------------------------------------------- C6 yük
    def c6(self, sigma, every):
        rng = np.random.default_rng(5)
        conf = Counter()
        for w in WORLDS:
            for root in ('f1', 'f2'):
                for fr in self.frames(root, w, None, every):
                    cam = fr.ds.cams['lane_camera']
                    loaded, _ = self.lane.load.detect(noisy(fr.rgb('lane_camera'), sigma, rng), cam, fr.fork_q)
                    gt_over = self._over_forks(fr)
                    conf[('belirsiz' if gt_over is None else gt_over, bool(loaded))] += 1
        r = {f'gt_{a}_det_{b}': v for (a, b), v in sorted(conf.items(), key=str)}
        r['note'] = ('GT: GT kolilerinin silüeti, yük bölgesinin en az yarısını kaplıyor mu (taşınan ya da bölmede '
                     'çatalların üstünde duran koli); '
                     'bir görüntüden ikisi ayırt edilemez, alma başarısı otonomide geri çıkıştan sonra denetlenir')
        ok = conf[(True, False)] == 0 and conf[(False, True)] == 0
        return r, ok

    def _over_forks(self, fr):
        """GT: GT kolilerinin şerit kamerasındaki silüeti, yük denetiminin baktığı bölgenin (taşınan kolinin
        silüeti, kenarlardan 3 px içeride) kaplıyor mu — algılayıcının sorusunun GT'si. Örtüşme ≥ 0.7: var,
        ≤ 0.3: yok, arası: belirsiz (None; yarım örtüşmede ikili bir cevap tanımsız)."""
        import cv2
        rg, cam = self.lane.rg, fr.ds.cams['lane_camera']
        roi = hull_mask(cam, [rg.carried_parcel_box(fr.fork_q)])
        roi = cv2.erode(roi.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
        if roi.sum() == 0:
            return bool(fr.rec.get('load'))
        S = self.cat.parcel_size  # noqa: N806
        box = np.array([[sx * S[0] / 2, sy * S[1] / 2, sz * S[2] / 2]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        cover = np.zeros_like(roi)
        for pose in fr.parcels.values():
            cover |= hull_mask(cam, [fr.to_base([pose.apply(c) for c in box])])
        frac = float((cover & roi).sum()) / float(roi.sum())
        return None if 0.3 < frac < 0.7 else frac >= 0.7      # yarım örtüşme: belirsiz, ölçüt dışı

    # ----------------------------------------------------------------- C9 son yanaşma
    def c9(self, sigma, every):
        rng = np.random.default_rng(6)
        rows, n_gt, n_det = [], 0, 0
        for w in WORLDS:
            gt = self.gt[w]
            labels = {(lb['station'], lb['bay']): lb for lb in gt.bay_labels()}
            for fr in self.frames('f2', w, {'entry'}, every):
                cam = fr.ds.cams['lane_camera']
                r = self.lane.process(noisy(fr.rgb('lane_camera'), sigma, rng), cam, fr.fork_q)
                gb = [g for g in gt_bays(gt, fr, self.cat) if g['key'] == (fr.rec['station'], fr.rec['bay'])][0]
                lab = labels[gb['key']]
                n_gt += 1
                c = fr.to_base([lab['center']])[0]
                m = [lb for lb in r['labels'] if np.linalg.norm(lb.label.p[:2] - c[:2]) < 0.04]
                if not m:
                    rows.append({'world': w, 'load': fr.rec['load'], 'd': fr.rec['d'], 'found': False})
                    continue
                n_det += 1
                lb = m[0]
                t = np.array([-gb['n'][1], gb['n'][0]])
                # birincil ölçüm: etiket merkezinin bölme eksenine göre yanal konumu ve eksen boyunca
                # konumu (ön çizgiye mesafe = bu − label_face_x). Çatalın bulunduğu yer etikete yakın;
                # yaw otonomide son hedef çifti ölçümünden gelir.
                # etiket merkezi (eğim düzeltmesi dahil): eksen boyunca ve yanal hata; çatal etikete yakın,
                # yaw otonomide son hedef çifti ölçümünden gelir
                e = lb.label.p[:2] - c[:2]
                rows.append({'world': w, 'load': fr.rec['load'], 'd': fr.rec['d'], 'found': True,
                             'yaw_valid': bool(lb.yaw_valid),
                             'dist_mm': float(np.dot(e, gb['n']) * 1000),
                             'lat_mm': float(np.dot(e, t) * 1000),
                             'pitch_applied': bool(r.get('pitch')),
                             'yaw_deg': math.degrees(_wrap(lb.theta - gb['theta']))})
        found = [x for x in rows if x['found']]
        res = {'frames': n_gt, 'detected': n_det, 'rate': n_det / max(n_gt, 1),
               'missed_at_d': sorted({x['d'] for x in rows if not x['found']}), 'groups': {}}
        ok = res['rate'] == 1.0
        for name, f in (('yüklü', lambda x: x['load']), ('yüksüz', lambda x: not x['load'])):
            s = [x for x in found if f(x)]
            if s:
                g = {'n': len(s), 'dist_mm_max': float(max(abs(x['dist_mm']) for x in s)),
                     'dist_mm_mean': float(np.mean([x['dist_mm'] for x in s])),
                     'lat_mm_max': float(max(abs(x['lat_mm']) for x in s)),
                     'lat_mm_mean': float(np.mean([x['lat_mm'] for x in s])),
                     'yaw_valid_rate': float(np.mean([x['yaw_valid'] for x in s])),
                     'pitch_applied_rate': float(np.mean([x['pitch_applied'] for x in s])),
                     'info_label_yaw_deg_max': float(max(abs(x['yaw_deg']) for x in s if x['yaw_valid']))
                     if any(x['yaw_valid'] for x in s) else None}
                res['groups'][name] = g
                ok = ok and g['dist_mm_max'] <= 5 and g['lat_mm_max'] <= 3
        return res, ok


def _iou(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    iw, ih = max(0, min(ax1, bx1) - max(ax0, bx0)), max(0, min(ay1, by1) - max(ay0, by0))
    inter = iw * ih
    u = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
    return inter / u if u > 0 else 0.0


# =============================================================================
#  C0: statik tarama (algılama ve otonomi GT okumuyor, gz kullanmıyor; plan R1/R2)
# =============================================================================
FORBIDDEN = [r'stations\.yaml', r'markers\.yaml', r'factory_lanes\.yaml', r'factory\.yaml',
             r'\bgz\.', r'gz\.transport', r'gz\.msgs', r'dynamic_pose', r'/world/', r'set_pose',
             r'parcel_source_', r'\bforklift_evaluation\b', r'\bsim_tools\b', r'\bstation_layout\b']


def c0_static():
    """Yalnız koda bakar: Python'da import edilen modüller ve docstring olmayan dizgeler; yaml/xml/cfg'de
    yorum dışı satırlar. Açıklama metinlerinde (ör. 'GT robota sızmamalı') geçen adlar sayılmaz."""
    import ast
    hits = []
    scanned = []
    for pkg in ('forklift_perception', 'forklift_autonomy'):
        src = os.path.expanduser(f'~/cv_forklift_ws/src/{pkg}')
        if not os.path.isdir(src):
            continue
        scanned.append(pkg)
        for root, _, files in os.walk(src):
            for fn in files:
                path = os.path.join(root, fn)
                rel = os.path.relpath(path, src)
                if fn.endswith('.py'):
                    with open(path, encoding='utf-8') as f:
                        tree = ast.parse(f.read())
                    docs = set()
                    for node in ast.walk(tree):
                        if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef, ast.AsyncFunctionDef)):
                            b = node.body
                            if b and isinstance(b[0], ast.Expr) and isinstance(b[0].value, ast.Constant):
                                docs.add(id(b[0].value))
                    items = []
                    for node in ast.walk(tree):
                        if isinstance(node, ast.Import):
                            items += [(node.lineno, a.name) for a in node.names]
                        elif isinstance(node, ast.ImportFrom):
                            items.append((node.lineno, node.module or ''))
                        elif isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docs:
                            items.append((node.lineno, node.value))
                    for ln, text in items:
                        for pat in FORBIDDEN:
                            if re.search(pat, text):
                                hits.append(f'{rel}:{ln}: {pat}')
                elif fn.endswith(('.yaml', '.xml', '.cfg')):
                    with open(path, encoding='utf-8') as f:
                        for i, line in enumerate(f, 1):
                            code = re.sub(r'<!--.*?-->', '', line.split('#')[0])
                            for pat in FORBIDDEN:
                                if re.search(pat, code):
                                    hits.append(f'{rel}:{i}: {pat}')
    return {'scanned': scanned, 'forbidden_patterns': FORBIDDEN, 'hits': hits}, not hits


# =============================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', default=os.path.expanduser('~/cv_forklift_ws/data'))
    ap.add_argument('--calib', default=os.path.join(get_package_share_directory('forklift_perception'),
                                                    'config', 'calibration'))
    ap.add_argument('--out', default=os.path.expanduser('~/cv_forklift_ws/docs/logs/cv/f2'))
    ap.add_argument('--sigma', default='0,0.007,0.02')
    ap.add_argument('--tests', default='C0,C1,C2,C3,C4,C5,C6,C9')
    ap.add_argument('--every-noisy', type=int, default=3, help='gürültülü koşularda her n. kare')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    calib = a.calib if os.path.exists(os.path.join(a.calib, 'sim_lane_camera.yaml')) else None
    ev = Evaluator(a.data, calib, a.out)
    tests = [t.strip() for t in a.tests.split(',')]
    sigmas = [float(s) for s in a.sigma.split(',')]
    res = {'created': time.strftime('%Y-%m-%d %H:%M:%S'), 'calibration': calib or 'yok (camera_info)',
           'datasets': {f'{k[0]}/{k[1]}': len(v) for k, v in ev.ds.items()}, 'results': {}}
    fns = {'C1': ev.c1, 'C2': lambda s, e: ev.lanes(s, e, True), 'C3': lambda s, e: ev.lanes(s, e, False),
           'C4': ev.c4, 'C5': ev.c5, 'C6': ev.c6, 'C9': ev.c9}
    if 'C0' in tests:
        r, ok = c0_static()
        res['results']['C0'] = {'0': {'status': PASS if ok else FAIL, **r}}
    for t in tests:
        if t not in fns:
            continue
        for s in sigmas:
            t0 = time.time()
            try:
                r, ok = fns[t](s, 1 if s == 0 else a.every_noisy)
                status = (PASS if ok else FAIL) if s == 0 else INFO
            except Exception as e:
                import traceback
                traceback.print_exc()
                r, status = {'error': repr(e)}, NOT_RUN
            r['seconds'] = round(time.time() - t0, 1)
            res['results'].setdefault(t, {})[str(s)] = {'status': status, **r}
            print(f'[{t} σ={s}] {status} ({r["seconds"]} s)', flush=True)
            with open(os.path.join(a.out, 'cv_results.json'), 'w') as f:
                json.dump(res, f, indent=1, ensure_ascii=False, default=float)
    summary = {t: {s: v['status'] for s, v in d.items()} for t, d in res['results'].items()}
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
