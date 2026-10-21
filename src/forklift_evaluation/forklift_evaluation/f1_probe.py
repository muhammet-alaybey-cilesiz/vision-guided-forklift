"""
f1_probe.py — Faz 1 ölçümleri (docs/PLAN_CV_AUTONOMY.md §9, F1)

Kullanım
  ros2 run forklift_evaluation f1_probe --world station_test --tasks overlay,occlusion,sign,odom,record
  ros2 run forklift_evaluation f1_probe --world factory --tasks overlay,record
  ros2 run forklift_evaluation f1_probe --cpu --data data/f1          (simülasyonsuz)
Görevler
  overlay   : GT izdüşümü (hedefler, şeritler, tabelalar, koliler, çatallar) iki kamerada;
              hedeflerde koyuluk ağırlık merkezi ile izdüşüm farkı, şeritlerde renkli piksel oranı
  occlusion : taşınan kolinin şerit kamerasında kapattığı bölge ve hedeflerin görünürlüğü (G3, G4)
  sign      : tabelanın ön kamerada tam göründüğü en yakın mesafe (G5); görüş menzili (G1)
  odom      : /odom tekerlek eklemlerinden mi hesaplanıyor (ışınlama ve sürüş denemesi)
  record    : çevrimdışı değerlendirme kümesi (görüntü + GT + kamera modeli)
Simülasyonu kendisi başlatır (station_sim.launch.py, control:=false, ekransız).
Sonuçlar: <out>/f1_<world>.json, <out>/images/*.jpg; küme: <data>/<world>/.
"""

import argparse
import json
import math
import os
import time
import traceback

import cv2
import numpy as np

from forklift_evaluation import overlay as ov
from forklift_evaluation.ros_cams import RosCams
from forklift_evaluation.world_gt import EvalSetup, Pose, WorldGT, pose_in_front, simt, sl

CAMS = ('front_camera', 'lane_camera')
SHORT = {'front_camera': 'front', 'lane_camera': 'lane'}
CARRY_Q = 0.06                 # taşıma yüksekliği (D-43; autonomy.yaml'a taşınacak)
PARCEL_GAP = 0.002             # koli ön yüzü ↔ taşıyıcı ön yüzü (S6 ile aynı yerleştirme)
PARK = (0.6, -1.2)             # kullanılmayan koli için boş zemin (station_test ve factory'de serbest)
SETTLE = 0.5                   # ışınlamadan sonra beklenen sim süresi [s]


# =============================================================================
#  Oturum: gz (GT, yerleştirme) + ROS (görüntü, camera_info, TF, odom)
# =============================================================================
class Probe:
    def __init__(self, es, gz, ros):
        self.es, self.gz, self.ros = es, gz, ros
        self.gt = WorldGT(es)
        self.rg = simt.RobotGeom(es)
        self.cams = {c: ros.camera(c) for c in CAMS}
        self.parcels = sl.parcels(es.params, es.layout)

    def place(self, p_xy, yaw):
        simt.place_robot(self.gz, p_xy, yaw, settle=SETTLE)

    def capture(self):
        """Yerleştirmeden sonra yakalanmış iki kare + o anki GT (robot pozu, çatal)."""
        t = self.gz.now()
        out = {}
        for c in CAMS:
            r = self.ros.image(c, newer_than=t)
            if r is None:
                raise TimeoutError(f'{c}: yeni kare gelmedi')
            out[c] = r
        return out, simt.robot_bf(self.gz), self.gz.fork()

    def parcel_on_fork(self, name, q):
        bf = simt.robot_bf(self.gz)
        P = self.es.params['parcel']['size']
        local = [self.rg.carriage_front_x + PARCEL_GAP + P['x'] / 2.0, 0.0,
                 self.rg.fork_top0 + q + P['z'] / 2.0 + 0.002]
        pose = bf * Pose(local, np.eye(3))
        self.gz.set_pose(name, pose.p, pose.R)
        self.gz.wait_sim(SETTLE)

    def park(self, name):
        self.gz.set_pose(name, [PARK[0], PARK[1], 0.105], np.eye(3))
        self.gz.wait_sim(0.3)

    def restore_parcels(self):
        for name, _, (x, y, z, yaw) in self.parcels:
            self.gz.set_pose(name, [x, y, z], simt.Rz(yaw))
        self.gz.wait_sim(SETTLE)


# =============================================================================
#  overlay
# =============================================================================
def overlay_poses(es):
    poses = []
    for stn in es.layout['stations']:
        for d in (0.6, 1.2, 2.0):
            poses.append((f'{stn["name"]}_d{d:.1f}', *pose_in_front(stn, d)))
        poses.append((f'{stn["name"]}_d0.9_l+0.3', *pose_in_front(stn, 0.9, 0.3)))
    for ln in es.lanes:
        s, e = np.array(ln['start']), np.array(ln['end'])
        p = s + 0.3 * (e - s)
        poses.append((f'lane_{ln["station"]}', (p[0], p[1]), ln['yaw']))
    return poses


def draw_gt(pr, rgb, cam, bf, q, stats, is_lane):
    gt = pr.gt
    img = rgb.copy()
    for t in gt.align_targets():
        pts = ov.to_base(bf, t['plate'])
        uv, front, inside = cam.project(pts)
        if not front.all():
            continue
        ov.poly(img, cam, pts, ov.C_TARGET)
        c_uv, _, c_in = cam.project(ov.to_base(bf, [t['center']]))
        if not (inside.all() and c_in[0]):
            continue
        ov.cross(img, c_uv[0], ov.C_TARGET)
        area = cv2.contourArea(uv.astype(np.float32))
        cen = ov.dark_centroid(rgb, uv) if area >= 64 else None
        if cen is not None:
            depth = cam.to_optical(ov.to_base(bf, [t['center']]))[0, 2]
            err = cen - c_uv[0]
            # tanı: maske 1 px daraltılınca (kenar pikselleri dışarıda) fark değişiyor mu
            cen_e = ov.dark_centroid(rgb, uv, erode=1)
            err_e = (cen_e - c_uv[0]) if cen_e is not None else (np.nan, np.nan)
            stats['targets'].append({'cam': SHORT[cam_name(pr, cam)], 'station': t['station'], 'bay': t['bay'],
                                     'side': t['side'], 'area_px': float(area), 'depth_m': float(depth),
                                     'u': float(c_uv[0][0]), 'v': float(c_uv[0][1]),
                                     'du': float(err[0]), 'dv': float(err[1]),
                                     'du_eroded': float(err_e[0]), 'dv_eroded': float(err_e[1]),
                                     'err_mm': float(np.linalg.norm(err) * depth / cam.fx * 1000)})
    for b in gt.bay_lines():
        pts = ov.to_base(bf, b['poly'])
        if ov.poly(img, cam, pts, ov.C_BAY):
            _lane_fill(cam, rgb, pts, stats, 'bay_line', b['color'])
    for ln in gt.floor_lanes():
        pts = ov.to_base(bf, ln['poly'])
        if ov.poly(img, cam, pts, ov.C_LANE):
            _lane_fill(cam, rgb, pts, stats, 'floor_lane', ln['color'])
        ov.poly(img, cam, ov.to_base(bf, ln['stop_bar']), ov.C_STOP)
    for s in gt.signs():
        ov.poly(img, cam, ov.to_base(bf, s['quad']), ov.C_SIGN, thick=2)
        ov.poly(img, cam, ov.to_base(bf, s['block']), ov.C_BLOCK)
    for p in gt.parcel_boxes(pr.gz):
        ov.box(img, cam, ov.to_base(bf, p['corners']), ov.C_PARCEL)
        ov.poly(img, cam, ov.to_base(bf, p['seal']), ov.C_SEAL)
    if is_lane:
        for b in gt.fork_boxes(q or 0.0):
            ov.box(img, cam, b, ov.C_FORK)
    return img


def cam_name(pr, cam):
    return next(n for n, c in pr.cams.items() if c is cam)


def _lane_fill(cam, rgb, pts_base, stats, kind, color):
    uv, front, inside = cam.project(pts_base)
    if not inside.all() or color == 'white':
        return
    f = ov.saturated_fraction(rgb, uv)
    if f is not None:
        stats['lanes'].append({'kind': kind, 'color': color, 'sat_fraction': f,
                               'area_px': float(cv2.contourArea(uv.astype(np.float32))),
                               'row_offset_px': ov.stripe_row_offset(rgb, uv)})


def task_overlay(pr, res):
    stats = {'targets': [], 'lanes': [], 'views': []}
    for name, p_xy, yaw in overlay_poses(pr.es):
        if not pr.gt.robot_footprint_ok(p_xy, yaw):
            stats['views'].append({'view': name, 'skipped': 'istasyonla çakışıyor'})
            continue
        pr.place(p_xy, yaw)
        frames, bf, q = pr.capture()
        for c in CAMS:
            img = draw_gt(pr, frames[c][1], pr.cams[c], bf, q, stats, c == 'lane_camera')
            ov.label(img, f'{pr.es.world} {name} {SHORT[c]}', (6, 16))
            ov.save_rgb(img, os.path.join(pr.es.images, f'overlay_{pr.es.world}_{name}_{SHORT[c]}.jpg'))
        stats['views'].append({'view': name, 'xy': list(p_xy), 'yaw': yaw})
    tg = stats['targets']
    summ = {}
    for cam in ('front', 'lane'):
        e = [t for t in tg if t['cam'] == cam]
        if e:
            px = np.array([math.hypot(t['du'], t['dv']) for t in e])
            summ[cam] = {'n': len(e), 'median_px': float(np.median(px)), 'max_px': float(px.max()),
                         'mean_du': float(np.mean([t['du'] for t in e])),
                         'mean_dv': float(np.mean([t['dv'] for t in e])),
                         'max_mm': float(max(t['err_mm'] for t in e))}
    lanes = {}
    for kind in ('bay_line', 'floor_lane'):
        f = [x['sat_fraction'] for x in stats['lanes'] if x['kind'] == kind]
        if f:
            lanes[kind] = {'n': len(f), 'min': float(min(f)), 'median': float(np.median(f))}
    res['overlay'] = {'targets_summary': summ, 'lanes_summary': lanes, **stats}


# =============================================================================
#  occlusion (G3, G4): taşınan koli şerit kamerasında neyi kapatıyor
# =============================================================================
def silhouette(cam, corners_base, shape):
    uv, front, _ = cam.project(corners_base)
    if not front.all():
        return None
    m = np.zeros(shape[:2], np.uint8)
    cv2.fillConvexPoly(m, cv2.convexHull(np.round(uv).astype(np.int32)), 255)
    return m > 0


def task_occlusion(pr, res):
    es, gz = pr.es, pr.gz
    stn = es.station('dest_red')
    name = 'parcel_source_1'
    cam = pr.cams['lane_camera']
    ok, q = simt.fork_to(gz, pr.rg, CARRY_Q)
    out = {'station': stn['name'], 'bay': 1, 'fork_q': q, 'rows': []}
    targets = [t for t in pr.gt.align_targets() if t['station'] == stn['name']]
    S = es.params['parcel']['size']
    hx, hy, hz = S['x'] / 2, S['y'] / 2, S['z'] / 2
    box_local = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    for d in np.arange(0.45, 1.01, 0.025):
        p_xy, yaw = pose_in_front(stn, float(d))
        pr.park(name)
        pr.place(p_xy, yaw)
        a, bf, _ = pr.capture()
        pr.parcel_on_fork(name, q)
        b, bf, _ = pr.capture()
        A, B = a['lane_camera'][1], b['lane_camera'][1]
        diff = np.abs(A.astype(int) - B.astype(int)).max(axis=2) > 12
        ppose = gz.model(name)
        sil = silhouette(cam, ov.to_base(bf, np.array([ppose.apply(c) for c in box_local])), A.shape)
        cam_x = cam.t[0]
        row = {'d_base': float(d), 'd_cam': float(d - cam_x), 'targets': []}
        if sil is not None:
            col = sil[:, int(cam.cx)]
            top = int(np.argmax(col)) if col.any() else None
            row['sil_top_row_center'] = top
            if top is not None:
                g, okg = cam.pixels_to_plane([[cam.cx, top - 0.5]], 0.0)
                row['ground_visible_from_x'] = float(g[0, 0]) if okg[0] else None
                cols = np.where(sil[min(top + 5, A.shape[0] - 1)])[0]
                row['sil_cols_at_top'] = [int(cols.min()), int(cols.max())] if cols.size else None
            dcol = diff[:, int(cam.cx)]
            row['diff_top_row_center'] = int(np.argmax(dcol)) if dcol.any() else None
        for t in targets:
            pts = ov.to_base(bf, t['plate'])
            uv, front, inside = cam.project(pts)
            if not inside.all():
                row['targets'].append({'bay': t['bay'], 'side': t['side'], 'in_image': False})
                continue
            m = ov.poly_mask(A.shape, uv)
            ch = float(diff[m].mean()) if m.any() else None
            hid = float(sil[m].mean()) if (sil is not None and m.any()) else None
            row['targets'].append({'bay': t['bay'], 'side': t['side'], 'in_image': True,
                                   'changed_frac': ch, 'silhouette_frac': hid})
        out['rows'].append(row)
        if abs(d - 0.45) < 1e-6 or abs(d - 0.6) < 1e-6 or abs(d - 0.8) < 1e-6:
            img = B.copy()
            if sil is not None:
                cnt, _ = cv2.findContours(sil.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img, cnt, -1, (255, 0, 0), 1)
            for t in targets:
                ov.poly(img, cam, ov.to_base(bf, t['plate']), ov.C_TARGET)
            ov.label(img, f'yuklu, ledge on cizgisine kamera {d - cam_x:.2f} m', (6, 16))
            ov.save_rgb(img, os.path.join(es.images, f'occlusion_{es.world}_d{d:.2f}.jpg'))
            ov.save_rgb(A, os.path.join(es.images, f'occlusion_{es.world}_d{d:.2f}_noload.jpg'))
    pr.restore_parcels()
    simt.fork_to(gz, pr.rg, 0.0)
    # bölme 1 (orta) hedeflerinin ikisi de kolinin silüetinden çıktığı en yakın kamera mesafesi
    vis = [r['d_cam'] for r in out['rows']
           if all(t.get('in_image') and (t.get('silhouette_frac') or 0) < 0.05 for t in r['targets'] if t['bay'] == 1)]
    out['bay1_targets_visible_min_d_cam'] = min(vis) if vis else None
    res['occlusion'] = out


# =============================================================================
#  sign (G5) ve görüş menzili (G1) — kamera modeliyle, örnek görüntülerle doğrulanır
# =============================================================================
def task_sign(pr, res):
    front = pr.cams['front_camera']
    stn = pr.es.station('source')
    x, y0, y1, z0, z1 = sl.sign_rect(pr.es.params, stn['bays'])
    quad = np.array([[x, y1, z1], [x, y0, z1], [x, y0, z0], [x, y1, z0]])
    h = z1 - z0
    block = np.array([[x, y1, z1], [x, y1 - h, z1], [x, y1 - h, z0], [x, y1, z0]])
    rows, full, blk = [], None, None
    for d in np.arange(0.30, 2.0, 0.01):
        rel = quad + [d, 0, 0]
        _, _, ins = front.project(rel)
        _, _, insb = front.project(block + [d, 0, 0])
        if ins.all() and full is None:
            full = float(d)
        if insb.all() and blk is None:
            blk = float(d)
        rows.append({'d': float(d), 'sign_inside': bool(ins.all()), 'block_inside': bool(insb.all())})
    res['sign'] = {'min_d_full_sign': full, 'min_d_block': blk,
                   'note': 'd = base_footprint → istasyon ön kenarı, robot istasyona dik ve ortada'}
    res['ground_range'] = {c: dict(zip(('near_x', 'far_x'), pr.cams[c].ground_range(0.0))) for c in CAMS}
    res['camera_models'] = {c: {'K': pr.cams[c].K.tolist(), 'D': pr.cams[c].D.tolist(),
                                'R_base_opt': pr.cams[c].R.tolist(), 't_base_opt': pr.cams[c].t.tolist(),
                                'size': [pr.cams[c].width, pr.cams[c].height]} for c in CAMS}
    for d in (full, (full or 0.6) - 0.05, 1.0):
        if d is None:
            continue
        p_xy, yaw = pose_in_front(stn, d)
        pr.place(p_xy, yaw)
        frames, bf, q = pr.capture()
        img = frames['front_camera'][1].copy()
        ov.poly(img, front, ov.to_base(bf, pr.gt.signs()[0]['quad']), ov.C_SIGN, thick=2)
        ov.label(img, f'on kamera, istasyon on kenarina {d:.2f} m', (6, 16))
        ov.save_rgb(img, os.path.join(pr.es.images, f'sign_{pr.es.world}_d{d:.2f}.jpg'))


# =============================================================================
#  odom: tekerlek eklemlerinden mi?
# =============================================================================
def task_odom(pr, res):
    gz, ros = pr.gz, pr.ros
    out = {}
    pr.place((0.0, 0.0), 0.0)
    gz.wait_sim(1.0)
    o1, g1 = ros.odom(), simt.robot_bf(gz)
    pr.place((0.5, 0.0), 0.0)
    gz.wait_sim(1.0)
    o2, g2 = ros.odom(), simt.robot_bf(gz)
    out['teleport'] = {'gt_dx': float(g2.p[0] - g1.p[0]), 'odom_dx': float(o2[1] - o1[1]) if o1 and o2 else None}
    o3, g3 = ros.odom(), simt.robot_bf(gz)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        gz.cmd(0.1)
        time.sleep(0.05)
    gz.cmd(0.0)
    gz.wait_sim(1.5)
    gz.cmd(0.0)
    o4, g4 = ros.odom(), simt.robot_bf(gz)
    out['drive'] = {'gt_dist': float(np.linalg.norm(g4.p[:2] - g3.p[:2])),
                    'odom_dist': float(math.hypot(o4[1] - o3[1], o4[2] - o3[2])) if o3 and o4 else None}
    out['conclusion'] = ('tekerlek eklemlerinden (ışınlama odometriyi değiştirmedi)'
                         if out['teleport']['odom_dx'] is not None and abs(out['teleport']['odom_dx']) < 0.01
                         else 'model pozundan olabilir: ışınlama odometriye yansıdı')
    pr.place((0.0, 0.0), 0.0)
    res['odom'] = out


# =============================================================================
#  record: çevrimdışı küme
# =============================================================================
def record_views(es, gt):
    views = []
    for stn in es.layout['stations']:
        for d in (0.45, 0.6, 0.8, 1.0, 1.3, 1.7, 2.2, 2.8):
            for lat in (-0.3, 0.0, 0.3):
                for yd in (-10, 0, 10):
                    views.append({'set': 'stations', 'station': stn['name'], 'd': d, 'lateral': lat,
                                  'yaw_off_deg': yd, 'load': False})
        if stn['name'] != 'source':
            for d in (0.6, 0.8, 1.0, 1.3):
                for lat in (-0.3, 0.0, 0.3):
                    views.append({'set': 'loaded', 'station': stn['name'], 'd': d, 'lateral': lat,
                                  'yaw_off_deg': 0, 'load': True})
    for ln in es.lanes:
        for f in (0.25, 0.5, 0.75):
            for lat in (-0.08, 0.0, 0.08):
                for yd in (-10, 0, 10):
                    views.append({'set': 'lanes', 'lane': ln['station'], 'frac': f, 'lateral': lat,
                                  'yaw_off_deg': yd, 'load': False})
    return views


def view_pose(es, v):
    if v['set'] == 'lanes':
        ln = next(x for x in es.lanes if x['station'] == v['lane'])
        s, e = np.array(ln['start']), np.array(ln['end'])
        p = s + v['frac'] * (e - s)
        n = np.array([-math.sin(ln['yaw']), math.cos(ln['yaw'])])
        p = p + v['lateral'] * n
        return (float(p[0]), float(p[1])), ln['yaw'] + math.radians(v['yaw_off_deg'])
    stn = es.station(v['station'])
    return pose_in_front(stn, v['d'], v['lateral'], math.radians(v['yaw_off_deg']))


def task_record(pr, res, data_dir):
    es, gz = pr.es, pr.gz
    root = os.path.join(data_dir, es.world)
    os.makedirs(root, exist_ok=True)
    meta = {'world': es.world, 'created': time.strftime('%Y-%m-%d %H:%M:%S'),
            'cameras': {c: {'K': pr.cams[c].K.tolist(), 'D': pr.cams[c].D.tolist(),
                            'R_base_opt': pr.cams[c].R.tolist(), 't_base_opt': pr.cams[c].t.tolist(),
                            'size': [pr.cams[c].width, pr.cams[c].height]} for c in CAMS},
            'note': 'GT yalnız değerlendirme içindir (plan R2); görüntüler gürültüsüz render (σ = 0)'}
    with open(os.path.join(root, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    views = record_views(es, pr.gt)
    load_name = 'parcel_source_1'
    loaded_state = False
    n_ok = n_skip = 0
    t_start = time.time()
    with open(os.path.join(root, 'index.jsonl'), 'w') as idx:
        for i, v in enumerate(views):
            p_xy, yaw = view_pose(es, v)
            if not pr.gt.robot_footprint_ok(p_xy, yaw):
                n_skip += 1
                continue
            if v['load'] != loaded_state:
                if v['load']:
                    simt.fork_to(gz, pr.rg, CARRY_Q)
                else:
                    pr.restore_parcels()
                    simt.fork_to(gz, pr.rg, 0.0)
                loaded_state = v['load']
            if v['load']:
                pr.park(load_name)
            pr.place(p_xy, yaw)
            if v['load']:
                pr.parcel_on_fork(load_name, gz.fork())
            frames, bf, q = pr.capture()
            rec = dict(v, id=f'{i:05d}', robot_bf={'p': bf.p.tolist(), 'R': bf.R.tolist()}, fork_q=q,
                       parcels={n: {'p': gz.model(n).p.tolist(), 'R': gz.model(n).R.tolist()}
                                for n, _, _ in pr.parcels if gz.model(n) is not None},
                       stamps={SHORT[c]: frames[c][0] for c in CAMS})
            for c in CAMS:
                cv2.imwrite(os.path.join(root, f'{rec["id"]}_{SHORT[c]}.png'),
                            cv2.cvtColor(frames[c][1], cv2.COLOR_RGB2BGR))
            idx.write(json.dumps(rec) + '\n')
            n_ok += 1
            if n_ok % 25 == 0:
                print(f'[record {es.world}] {n_ok} görüş ({time.time() - t_start:.0f} s)', flush=True)
    if loaded_state:
        pr.restore_parcels()
        simt.fork_to(gz, pr.rg, 0.0)
    res['record'] = {'dir': root, 'views': n_ok, 'skipped_collision': n_skip,
                     'by_set': {s: sum(1 for v in views if v['set'] == s) for s in ('stations', 'loaded', 'lanes')},
                     'seconds': round(time.time() - t_start, 1)}


# =============================================================================
#  cpu: Python'da temel işlem bütçesi (kaydedilmiş kareler üzerinde)
# =============================================================================
def task_cpu(data_dir):
    files = []
    for w in ('station_test', 'factory'):
        d = os.path.join(data_dir, w)
        if os.path.isdir(d):
            files += sorted(os.path.join(d, f) for f in os.listdir(d) if f.endswith('.png'))[:200]
    if not files:
        return {'error': 'kayıtlı kare yok'}
    imgs = [cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2RGB) for f in files[:120]]
    kernel = np.ones((3, 3), np.uint8)
    ranges = [((0, 120, 60), (8, 255, 255)), ((170, 120, 60), (179, 255, 255)), ((20, 120, 80), (35, 255, 255)),
              ((45, 120, 50), (85, 255, 255)), ((100, 120, 50), (125, 255, 255))]
    t = []
    for img in imgs:
        t0 = time.perf_counter()
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        for lo, hi in ranges:
            m = cv2.inRange(hsv, lo, hi)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
            cv2.connectedComponentsWithStats(m)
        t.append(time.perf_counter() - t0)
    t = np.array(t) * 1000
    return {'frames': len(imgs), 'op': 'RGB→HSV + 5 renk maskesi + açma + bağlantılı bileşenler',
            'mean_ms': float(t.mean()), 'p95_ms': float(np.percentile(t, 95)),
            'budget_ms_at_30hz': 33.3, 'cv2_threads': cv2.getNumThreads()}


# =============================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='station_test')
    ap.add_argument('--tasks', default='overlay,occlusion,sign,odom')
    ap.add_argument('--out', default=os.path.expanduser('~/cv_forklift_ws/docs/logs/cv/f1'))
    ap.add_argument('--data', default=os.path.expanduser('~/cv_forklift_ws/data/f1'))
    ap.add_argument('--cpu', action='store_true', help='yalnız CPU bütçesi (simülasyonsuz)')
    ap.add_argument('--gui', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.cpu:
        r = task_cpu(a.data)
        print(json.dumps(r, indent=1))
        with open(os.path.join(a.out, 'f1_cpu.json'), 'w') as f:
            json.dump(r, f, indent=1)
        return
    es = EvalSetup(a.world, a.out, gui=a.gui)
    tasks = [t.strip() for t in a.tasks.split(',') if t.strip()]
    res = {'world': a.world, 'tasks': tasks, 'started': time.strftime('%Y-%m-%d %H:%M:%S'), 'errors': {}}
    path = os.path.join(a.out, f'f1_{a.world}.json')
    if os.path.exists(path):
        with open(path) as f:
            old = json.load(f)
        old.update({k: v for k, v in res.items() if k != 'errors'})
        res = old
        res.setdefault('errors', {})
    with simt.SimSession(es, f'f1_{a.world}', control=False) as sess:
        gz = ros = None
        try:
            gz = simt.Gz(es)
            simt.wait_world_ready(es, gz, sess, sl.parcels(es.params, es.layout))
            gz.record(False)
            ros = RosCams({c: (es.robot['cameras'][c]['topic'], es.robot['cameras'][c]['camera_info_topic'])
                           for c in CAMS})
            pr = Probe(es, gz, ros)
            res['spike_rclpy_gz_same_process'] = 'çalıştı: rclpy (görüntü, camera_info, TF) + gz-transport aynı süreçte'
            fns = {'overlay': task_overlay, 'occlusion': task_occlusion, 'sign': task_sign, 'odom': task_odom,
                   'record': lambda p, r: task_record(p, r, a.data)}
            for t in tasks:
                print(f'[f1 {a.world}] {t} başlıyor', flush=True)
                try:
                    fns[t](pr, res)
                    res['errors'].pop(t, None)
                except Exception as e:
                    traceback.print_exc()
                    res['errors'][t] = repr(e)
                with open(path, 'w') as f:
                    json.dump(res, f, indent=1, default=float)
        finally:
            if ros is not None:
                ros.close()
            if gz is not None:
                gz.close()
    print(f'[f1 {a.world}] bitti → {path}', flush=True)


if __name__ == '__main__':
    main()
