#!/usr/bin/env python3
"""
station_acceptance.py — yük istasyonu ve koli kabul testleri (S1–S8)

Kullanım
  ros2 run forklift_gazebo station_acceptance.py                # tüm testler
  ros2 run forklift_gazebo station_acceptance.py --tests S1,S2  # seçili testler
Seçenekler
  --docs DIR   çıktı dizini (varsayılan: <ws>/docs)
  --ws DIR     workspace kökü (varsayılan: ~/cv_forklift_ws)
  --gui        simülasyonu Gazebo penceresiyle aç (varsayılan: ekransız EGL)

S1 ve S2 simülasyon gerektirmez; kurulu paketteki xacro ve parametre dosyalarını
kullanır (önce colcon build). S3–S8 için betik simülasyonu kendisi başlatır
(station_sim.launch.py, control:=false: /cmd_vel'e doğrudan yayın yapılır,
DECISIONS D-42) ve tam dünyayı spawn anından itibaren kullanır; S3 bu yüzden
oturumun ilk testidir.

Konumlandırma ve ölçüm yalnızca gz-transport ile yapılır (odometri yok):
  gerçek poz : /world/<w>/dynamic_pose/info  (D-13)
  yerleştirme: /world/<w>/set_pose, test modeli ekleme/silme: /world/<w>/create, /remove
  sim zamanı : /world/<w>/stats
  komutlar   : /cmd_vel (gz.msgs.Twist), /fork/cmd_height (gz.msgs.Double)
  kameralar  : robotun gz görüntü topic'leri
Çıktı: her test için ölçülen değer, kriter ve GEÇTİ / KALDI / ÇALIŞTIRILAMADI;
docs/station_test_results.{json,md}, docs/logs/stations/*, docs/images/stations/*.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import station_layout as sl  # noqa: E402

PASS, FAIL, NOT_RUN = 'GEÇTİ', 'KALDI', 'ÇALIŞTIRILAMADI'
ALL_TESTS = ['S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7', 'S8']
TEST_NAMES = {
    'S1': 'xacro → SDF, gz sdf -k',
    'S2': 'Geometri denetimi',
    'S3': 'Statik kararlılık',
    'S4': 'Üst işaret tespiti',
    'S5': 'Alt işaret tespiti',
    'S6': 'Bırakma toleransı',
    'S7': 'Alma',
    'S8': 'Performans',
}
GEOM_TOL = 0.0005          # S2: ±0.5 mm


# =============================================================================
#  Ortam ve sonuç kaydı
# =============================================================================
class Setup:
    def __init__(self, ws, docs):
        from ament_index_python.packages import get_package_share_directory
        self.ws, self.docs = ws, docs
        self.share = get_package_share_directory('forklift_gazebo')
        self.params_file = os.path.join(self.share, 'config', 'station_params.yaml')
        self.stations_file = os.path.join(self.share, 'config', 'stations.yaml')
        self.markers_file = os.path.join(self.share, 'config', 'markers.yaml')
        self.station_xacro = os.path.join(self.share, 'xacro', 'station.sdf.xacro')
        self.parcel_xacro = os.path.join(self.share, 'xacro', 'parcel.sdf.xacro')
        self.texture_dir = os.path.join(self.share, 'models', 'aruco_markers', 'materials', 'textures')
        self.params = sl.load_yaml(self.params_file)
        self.layout = sl.load_yaml(self.stations_file)
        self.robot = sl.load_yaml(os.path.join(get_package_share_directory('forklift_description'),
                                               'config', 'forklift_params.yaml'))
        self.world = 'station_test'
        self.gui = False
        self.raw_dir = None
        self.logs = os.path.join(docs, 'logs', 'stations')
        self.images = os.path.join(docs, 'images', 'stations')
        os.makedirs(self.logs, exist_ok=True)
        os.makedirs(self.images, exist_ok=True)


@dataclass
class Result:
    id: str
    name: str
    status: str
    measured: str
    criterion: str
    details: dict = field(default_factory=dict)


class Report:
    """Sonuçlar dosyada birikir: seçili testler önceki koşuların sonuçlarını ezmez."""

    def __init__(self, docs):
        self.json_path = os.path.join(docs, 'station_test_results.json')
        self.md_path = os.path.join(docs, 'station_test_results.md')
        self.results = {}
        if os.path.exists(self.json_path):
            with open(self.json_path) as f:
                for r in json.load(f):
                    self.results[r['id']] = Result(**r)

    def add(self, r):
        r.details.setdefault('run_at', time.strftime('%Y-%m-%d %H:%M:%S'))
        self.results[r.id] = r
        print(f'[{r.id}] {r.status:16s} ölçülen: {r.measured} | kriter: {r.criterion}', flush=True)

    def write(self):
        order = [t for t in ALL_TESTS if t in self.results]
        with open(self.json_path, 'w') as f:
            json.dump([asdict(self.results[t]) for t in order], f, indent=2, ensure_ascii=False, default=float)
        lines = ['# İstasyon kabul testi sonuçları', '',
                 f'Üretim zamanı: {time.strftime("%Y-%m-%d %H:%M:%S")}', '',
                 '| # | Test | Ölçülen | Kriter | Sonuç | Koşu |', '|---|---|---|---|---|---|']
        for t in order:
            r = self.results[t]
            lines.append(f'| {r.id} | {r.name} | {r.measured} | {r.criterion} | **{r.status}** | '
                         f'{r.details.get("run_at", "")} |')
        with open(self.md_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')


def run(cmd, timeout=120):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return p.returncode, p.stdout, p.stderr


def xacro_to_file(xacro_file, args, path):
    """xacro çalıştırır, çıktıyı dosyaya yazar; (rc, stderr)."""
    rc, out, err = run(['xacro', xacro_file] + args)
    with open(path, 'w') as f:
        f.write(out)
    return rc, err


# =============================================================================
#  SDF ayrıştırma (S2): eksenlere hizalı kutular, link çerçevesinde
# =============================================================================
def _floats(text):
    return [float(v) for v in text.split()]


class Box:
    def __init__(self, el):
        self.name = el.get('name')
        pose = el.find('pose')
        self.c = np.array(_floats(pose.text)[:3]) if pose is not None else np.zeros(3)
        rpy = _floats(pose.text)[3:] if pose is not None else [0, 0, 0]
        if any(abs(a) > 1e-12 for a in rpy):
            raise ValueError(f'{self.name}: döndürülmüş kutu beklenmiyor')
        self.size = np.array(_floats(el.find('geometry/box/size').text))
        self.lo, self.hi = self.c - self.size / 2, self.c + self.size / 2
        self.el = el


def parse_model(sdf_text):
    root = ET.fromstring(sdf_text)
    model = root.find('model')
    link = model.find('link')
    cols = {Box(e).name: Box(e) for e in link.findall('collision')}
    vis = {Box(e).name: Box(e) for e in link.findall('visual')}
    frames = {f.get('name'): np.array(_floats(f.find('pose').text)[:3]) for f in model.findall('frame')}
    return model, link, cols, vis, frames


class Checker:
    """Ölçülen ↔ beklenen karşılaştırmaları; en büyük hata ve başarısızlıklar."""

    def __init__(self, tol):
        self.tol, self.n, self.max_err, self.fails = tol, 0, 0.0, []

    def eq(self, label, measured, expected):
        err = abs(float(measured) - float(expected))
        self.n += 1
        self.max_err = max(self.max_err, err)
        if err > self.tol:
            self.fails.append(f'{label}: {measured:.5f} ≠ {expected:.5f}')

    def true(self, label, cond):
        self.n += 1
        if not cond:
            self.fails.append(label)


# =============================================================================
#  S1 — xacro → SDF, gz sdf -k
# =============================================================================
def s1(st, rep):
    cases = []
    for n in (1, 3, 5):
        for color in sl.STATION_COLORS:
            cases.append(('station', f'N{n}_{color}', st.station_xacro,
                          [f'name:=station_N{n}_{color}', f'bays:={n}', f'color:={color}',
                           f'params_file:={st.params_file}']))
    for color in sl.SEAL_COLORS:
        cases.append(('parcel', color, st.parcel_xacro,
                      sl.parcel_xacro_args(f'parcel_{color}', color, st.params_file)))
    rows, bad = [], []
    for kind, tag, xf, args in cases:
        path = os.path.join(st.logs, f'S1_{kind}_{tag}.sdf')
        rc_x, err_x = xacro_to_file(xf, args, path)
        rc_k, out_k, err_k = run(['gz', 'sdf', '-k', path])
        text = (err_x + out_k + err_k)
        warn = any(w in text.lower() for w in ('warning', 'error'))
        ok = rc_x == 0 and rc_k == 0 and 'Valid.' in out_k and not warn
        rows.append({'case': f'{kind} {tag}', 'xacro_rc': rc_x, 'gz_rc': rc_k,
                     'gz_out': out_k.strip(), 'stderr': (err_x + err_k).strip()})
        if not ok:
            bad.append(f'{kind} {tag}')
    with open(os.path.join(st.logs, 'S1_summary.json'), 'w') as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)
    measured = f'{len(cases) - len(bad)}/{len(cases)} dosya "Valid.", uyarı/hata yok'
    if bad:
        measured += f'; kalan: {", ".join(bad)}'
    rep.add(Result('S1', TEST_NAMES['S1'], PASS if not bad else FAIL, measured,
                   'N = 1, 3, 5 × 4 istasyon rengi + 3 koli rengi; hata ve uyarı yok',
                   {'cases': rows}))


# =============================================================================
#  S2 — geometri denetimi (üretilen SDF ↔ station_params.yaml, ±0.5 mm)
# =============================================================================
def check_station_sdf(st, ck, sdf_text, n, ids, tag):
    P = st.params
    S, M = P['station'], P['markers']
    model, link, cols, vis, frames = parse_model(sdf_text)
    W = n * S['bay_pitch']
    ck.true(f'{tag}: static', model.findtext('static') == 'true')

    # zemin plakası (collision'lı, D-46) ve arka plaka
    for name in ('base_plate', 'back_plate'):
        ck.true(f'{tag}: {name} collision var', f'{name}_collision' in cols)
    bp = cols['base_plate_collision']
    for lab, m, e in (('x_min', bp.lo[0], -S['apron_depth']),
                      ('x_max', bp.hi[0], S['ledge_depth'] + S['back_plate_thickness']),
                      ('y_min', bp.lo[1], -W / 2), ('y_max', bp.hi[1], W / 2),
                      ('z_min', bp.lo[2], 0), ('z_max', bp.hi[2], S['base_plate_thickness'])):
        ck.eq(f'{tag}: zemin plakası {lab}', m, e)
    bk = cols['back_plate_collision']
    for lab, m, e in (('x_min', bk.lo[0], S['ledge_depth']),
                      ('x_max', bk.hi[0], S['ledge_depth'] + S['back_plate_thickness']),
                      ('y_min', bk.lo[1], -W / 2), ('y_max', bk.hi[1], W / 2),
                      ('z_min', bk.lo[2], 0), ('z_max', bk.hi[2], S['back_plate_height'])):
        ck.eq(f'{tag}: arka plaka {lab}', m, e)

    # çıkıntılar: N + 1 kutu, y'ye göre sıralı; aralarındaki boşluklar = kanallar
    ledges = sorted([b for k, b in cols.items() if k.startswith('ledge_')], key=lambda b: b.c[1])
    ck.true(f'{tag}: çıkıntı sayısı {len(ledges)} = N + 1', len(ledges) == n + 1)
    for b in ledges:
        ck.eq(f'{tag}: {b.name} üst z', b.hi[2], S['ledge_height'])
        ck.eq(f'{tag}: {b.name} alt z', b.lo[2], 0)
        ck.eq(f'{tag}: {b.name} x_min', b.lo[0], 0)
        ck.eq(f'{tag}: {b.name} x_max', b.hi[0], S['ledge_depth'])
        mu = b.el.findtext('surface/friction/ode/mu')
        ck.true(f'{tag}: {b.name} μ = {S["ledge_mu"]}', mu is not None and abs(float(mu) - S['ledge_mu']) < 1e-9)
    ck.eq(f'{tag}: sol uç y', ledges[-1].hi[1], W / 2)
    ck.eq(f'{tag}: sağ uç y', ledges[0].lo[1], -W / 2)
    centers = []
    for i in range(n):
        lo, hi = ledges[i].hi[1], ledges[i + 1].lo[1]
        ck.eq(f'{tag}: kanal {i} genişliği', hi - lo, S['channel_width'])
        ck.eq(f'{tag}: kanal {i} ekseni', (lo + hi) / 2, sl.bay_y(P, n, i))
        centers.append((lo + hi) / 2)
        f = frames.get(f'bay_{i}')
        ck.true(f'{tag}: bay_{i} çerçevesi var', f is not None)
        if f is not None:
            ck.eq(f'{tag}: bay_{i} çerçevesi y', f[1], (lo + hi) / 2)
    for i in range(n - 1):
        ck.eq(f'{tag}: adım {i}→{i + 1}', centers[i + 1] - centers[i], S['bay_pitch'])

    # işaretler: her bölmede üst (ID k) ve alt (ID k + offset); yalnız visual
    face_x = sl.marker_face_x(P)
    ck.true(f'{tag}: işaretlerin collision\'ı yok', not any('marker' in k for k in cols))
    found = {}
    for i, k in enumerate(ids):
        for kind, mid in sl.marker_ids(P, k).items():
            v = vis.get(f'marker_{kind}_{mid}')
            ck.true(f'{tag}: marker_{kind}_{mid} var', v is not None)
            if v is None:
                continue
            ck.eq(f'{tag}: {kind} {mid} ön yüz x', v.lo[0], face_x)
            ck.eq(f'{tag}: {kind} {mid} kalınlık', v.size[0], M['thickness'])
            ck.eq(f'{tag}: {kind} {mid} arka plakaya boşluk', S['ledge_depth'] - v.hi[0], M['standoff'])
            ck.eq(f'{tag}: {kind} {mid} y', v.c[1], sl.bay_y(P, n, i))
            ck.eq(f'{tag}: {kind} {mid} merkez z', v.c[2], M[kind]['center_z'])
            # siyah kare = doku kenarı · 6 / (6 + 2 · beyaz kenar)
            black = v.size[1] * 6 / (6 + 2 * M['quiet_zone_cells'])
            ck.eq(f'{tag}: {kind} {mid} siyah kare', black, M[kind]['size'])
            ck.eq(f'{tag}: {kind} {mid} kare (y = z)', v.size[1], v.size[2])
            amap = v.el.findtext('material/pbr/metal/albedo_map') or ''
            ck.true(f'{tag}: {kind} {mid} albedo_map {amap}',
                    amap.endswith('/' + sl.texture_name(P, mid)) and amap.startswith('model://'))
            found[mid] = v
    return found


def check_texture(st, ck, mid):
    """Kurulu doku: var, ≥ 600 px, kare ve kendi ID'sini veriyor."""
    import cv2
    path = os.path.join(st.texture_dir, sl.texture_name(st.params, mid))
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    ck.true(f'doku {mid} mevcut', img is not None)
    if img is None:
        return
    ck.true(f'doku {mid} ≥ 600 px kare', img.shape[0] == img.shape[1] >= 600)
    d = cv2.aruco.Dictionary_get(getattr(cv2.aruco, st.params['markers']['dictionary']))
    _, ids, _ = cv2.aruco.detectMarkers(img, d, parameters=cv2.aruco.DetectorParameters_create())
    ck.true(f'doku {mid} ID okunuyor', ids is not None and ids.ravel().tolist() == [mid])
    # beyaz kenar genişliği: quiet_zone_cells / (6 + 2 · quiet_zone_cells) · kenar
    q = st.params['markers']['quiet_zone_cells']
    edge_px = round(img.shape[0] * q / (6 + 2 * q))
    ck.true(f'doku {mid} beyaz kenar {edge_px} px',
            int(img[:edge_px].min()) == 255 and int(img[edge_px + 1].min()) == 0)


def check_parcel_sdf(st, ck, sdf_text, color):
    P = st.params
    B = P['parcel']
    model, link, cols, vis, _ = parse_model(sdf_text)
    tag = f'koli {color}'
    body = cols['body_collision']
    for ax, i in (('x', 0), ('y', 1), ('z', 2)):
        ck.eq(f'{tag}: boyut {ax}', body.size[i], B['size'][ax])
        ck.eq(f'{tag}: merkez {ax}', body.c[i], 0)
    m = float(link.findtext('inertial/mass'))
    ck.eq(f'{tag}: kütle', m, B['mass'])
    sx, sy, sz = B['size']['x'], B['size']['y'], B['size']['z']
    for lab, e in (('ixx', m * (sy**2 + sz**2) / 12), ('iyy', m * (sx**2 + sz**2) / 12),
                   ('izz', m * (sx**2 + sy**2) / 12)):
        ck.eq(f'{tag}: {lab}', float(link.findtext(f'inertial/inertia/{lab}')), e)
    mu = body.el.findtext('surface/friction/ode/mu')
    ck.true(f'{tag}: μ = {B["mu"]}', mu is not None and abs(float(mu) - B['mu']) < 1e-9)
    ck.true(f'{tag}: tek collision', list(cols) == ['body_collision'])
    seal = vis['seal_visual']
    ck.eq(f'{tag}: mühür iç yüzü = koli -x yüzü', seal.hi[0], -sx / 2)
    ck.eq(f'{tag}: mühür kalınlık', seal.size[0], B['seal']['thickness'])
    ck.eq(f'{tag}: mühür y', seal.size[1], B['seal']['size']['y'])
    ck.eq(f'{tag}: mühür z', seal.size[2], B['seal']['size']['z'])
    ck.eq(f'{tag}: mühür ortalı y', seal.c[1], 0)
    ck.eq(f'{tag}: mühür ortalı z', seal.c[2], 0)
    diffuse = _floats(seal.el.findtext('material/diffuse'))
    ck.true(f'{tag}: mühür rengi', np.allclose(diffuse, P['colors'][color]))


def s2(st, rep):
    import yaml
    ck = Checker(GEOM_TOL)
    # parametrik kapsama: N = 1, 3, 5 (N = 5 için şemanın uç ID'leri)
    variants = [(1, [0]), (3, [3, 4, 5]), (5, [20, 21, 22, 23, 24])]
    for n, ids in variants:
        path = os.path.join(st.logs, f'S2_station_N{n}.sdf')
        rc, err = xacro_to_file(st.station_xacro,
                                [f'name:=s2_N{n}', f'bays:={n}', 'color:=neutral',
                                 'bay_ids:=' + ','.join(map(str, ids)), f'params_file:={st.params_file}'], path)
        ck.true(f'N{n}: xacro rc 0', rc == 0)
        with open(path) as f:
            check_station_sdf(st, ck, f.read(), n, ids, f'N{n}')
    # stations.yaml'daki gerçek istasyonlar ↔ kurulu markers.yaml
    with open(st.markers_file) as f:
        mk = yaml.safe_load(f)
    by_id = {m['id']: m for m in mk['markers']}
    n_markers = 0
    for stn in st.layout['stations']:
        path = os.path.join(st.logs, f'S2_{stn["name"]}.sdf')
        xacro_to_file(st.station_xacro, sl.station_xacro_args(stn, st.params_file), path)
        with open(path) as f:
            found = check_station_sdf(st, ck, f.read(), stn['bays'], stn['bay_ids'], stn['name'])
        for mid, v in found.items():
            m = by_id.get(mid)
            ck.true(f'markers.yaml ID {mid} var', m is not None)
            if m is None:
                continue
            n_markers += 1
            ck.true(f'markers.yaml ID {mid} istasyon', m['station'] == stn['name'])
            ck.eq(f'markers.yaml ID {mid} boyut', m['size'], st.params['markers'][m['type']]['size'])
            for j, e in enumerate((v.lo[0], v.c[1], v.c[2])):
                ck.eq(f'markers.yaml ID {mid} poz[{j}]', m['pose_station']['xyz'][j], e)
    ck.true(f'markers.yaml işaret sayısı {len(by_id)} = {n_markers}', len(by_id) == n_markers)
    for mid in sl.all_marker_ids(st.params):
        check_texture(st, ck, mid)
    for color in sl.SEAL_COLORS:
        path = os.path.join(st.logs, f'S2_parcel_{color}.sdf')
        xacro_to_file(st.parcel_xacro, sl.parcel_xacro_args(f'parcel_{color}', color, st.params_file), path)
        with open(path) as f:
            check_parcel_sdf(st, ck, f.read(), color)
    with open(os.path.join(st.logs, 'S2_failures.txt'), 'w') as f:
        f.write('\n'.join(ck.fails) + '\n' if ck.fails else 'hata yok\n')
    rep.add(Result('S2', TEST_NAMES['S2'], PASS if not ck.fails else FAIL,
                   f'{ck.n} denetim, {len(ck.fails)} hata; en büyük ölçü sapması {ck.max_err * 1000:.4f} mm; '
                   f'{n_markers} işaret markers.yaml ile, {len(sl.all_marker_ids(st.params))} doku ID ile uyumlu',
                   'Kanal, adım, çıkıntı, plakalar, işaret konum/boyut, koli: YAML ile ±0.5 mm',
                   {'failures': ck.fails[:50], 'checks': ck.n, 'max_err_m': ck.max_err}))


# =============================================================================
#  Poz yardımcıları
# =============================================================================
def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([w, x, y, z])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def tilt_deg(R):
    """Gövde z ekseninin düşeyle açısı [°]."""
    return math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))


def yaw_of_R(R):
    return math.atan2(R[1, 0], R[0, 0])


class Pose:
    def __init__(self, p, R):
        self.p, self.R = np.asarray(p, float), np.asarray(R, float)

    @classmethod
    def from_pq(cls, p, q):
        return cls(p, quat_to_R(q))

    def __mul__(self, o):
        return Pose(self.p + self.R @ o.p, self.R @ o.R)

    def inv(self):
        return Pose(-self.R.T @ self.p, self.R.T)

    def apply(self, v):
        return self.p + self.R @ np.asarray(v, float)


def station_pose(st_entry):
    x, y, yaw = st_entry['pose']
    return Pose([x, y, 0.0], Rz(yaw))


def bay_pose(st, st_entry, i):
    """bay_i çerçevesinin dünya pozu."""
    return station_pose(st_entry) * Pose([0, sl.bay_y(st.params, st_entry['bays'], i), 0], np.eye(3))


# =============================================================================
#  Simülasyon oturumu: ayrı süreç grubu; kapanışta grubun tamamına sinyal
#  (aksi halde gz sim sunucusu geride kalabiliyor)
# =============================================================================
class SimSession:
    def __init__(self, st, tag):
        self.st, self.tag, self.proc = st, tag, None
        self.log_path = os.path.join(st.logs, f'sim_session_{tag}.log')

    def __enter__(self):
        log = open(self.log_path, 'w')
        cmd = ['ros2', 'launch', 'forklift_gazebo', 'station_sim.launch.py',
               f'gui:={"true" if self.st.gui else "false"}',
               f'headless:={"false" if self.st.gui else "true"}',
               'control:=false']
        self.proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print(f'[oturum {self.tag}] başlatıldı (pid {self.proc.pid})', flush=True)
        return self

    def spawned(self):
        with open(self.log_path) as f:
            return 'stations.launch:' in f.read()

    def __exit__(self, *exc):
        pgid = os.getpgid(self.proc.pid)
        for sig, wait_s in ((signal.SIGINT, 25), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            t0 = time.time()
            while time.time() - t0 < wait_s and _group_alive(pgid):
                time.sleep(0.2)
            if not _group_alive(pgid):
                break
        self.proc.wait(timeout=5)
        print(f'[oturum {self.tag}] kapandı', flush=True)
        return False


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


# =============================================================================
#  gz-transport arayüzü: gerçek poz, sim zamanı, kameralar, komutlar, servisler
# =============================================================================
class Gz:
    MODEL = 'forklift'

    def __init__(self, st):
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.model_pb2 import Model
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.twist_pb2 import Twist
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.transport13 import Node as GzNode
        self.st, self.w = st, st.world
        self.Twist, self.Double = Twist, Double
        self.lock = threading.Lock()
        self.node = GzNode()
        self.poses, self.pose_t = {}, None
        self.sim_t = None
        self.fork_q = None
        self.images, self.img_count = {}, {}
        self._rec = None
        itf = st.robot['gazebo']['interface']
        self.cam_topics = {n: c['topic'] for n, c in st.robot['cameras'].items()}
        self.extra_cams = {}
        subs = [(Pose_V, f'/world/{self.w}/dynamic_pose/info', self._on_pose),
                (WorldStatistics, f'/world/{self.w}/stats', self._on_stats),
                (Model, itf['joint_states_topic'], self._on_js)]
        for n, t in self.cam_topics.items():
            subs.append((Image, t, self._img_cb(n)))
        for typ, topic, cb in subs:
            if not self.node.subscribe(typ, topic, cb):
                raise RuntimeError(f'gz aboneliği başarısız: {topic}')
        self.Image = Image
        self.cmd_pub = self.node.advertise(itf['cmd_vel_topic'], Twist)
        self.fork_pub = self.node.advertise(itf['fork_cmd_topic'], Double)

    # ------------------------------------------------------------ abonelikler
    def _on_pose(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        d = {p.name: (np.array([p.position.x, p.position.y, p.position.z]),
                      np.array([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]))
             for p in msg.pose}
        with self.lock:
            self.poses, self.pose_t = d, t
            if self._rec is not None:
                self._rec.append((t, d))

    def _on_stats(self, msg):
        with self.lock:
            self.sim_t = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def _on_js(self, msg):
        for j in msg.joint:
            if j.name.endswith('fork_lift_joint'):
                with self.lock:
                    self.fork_q = j.axis1.position

    def _img_cb(self, name):
        def cb(msg):
            arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3].copy()
            t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            with self.lock:
                self.images[name] = (t, arr)
                self.img_count[name] = self.img_count.get(name, 0) + 1
        return cb

    def subscribe_camera(self, name, topic):
        self.node.subscribe(self.Image, topic, self._img_cb(name))

    # ------------------------------------------------------------- okumalar
    def now(self):
        with self.lock:
            return self.sim_t

    def model(self, name):
        """Modelin dünya pozu (dynamic_pose: yalnız dinamik modeller)."""
        with self.lock:
            v = self.poses.get(name)
        return None if v is None else Pose.from_pq(*v)

    def link(self, model, link):
        with self.lock:
            m, lk = self.poses.get(model), self.poses.get(link)
        return None if m is None or lk is None else Pose.from_pq(*m) * Pose.from_pq(*lk)

    def carriage(self):
        return self.link(self.MODEL, 'fork_carriage_link')

    def fork(self):
        with self.lock:
            return self.fork_q

    def image(self, name, newer_than=None, timeout=5.0):
        """Sim zamanı newer_than'dan sonra yakalanmış kare (RGB)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                v = self.images.get(name)
            if v is not None and (newer_than is None or v[0] > newer_than):
                return v[1]
            time.sleep(0.02)
        return None

    def record(self, on):
        with self.lock:
            if on:
                self._rec = []
                return None
            rec, self._rec = self._rec or [], None
            return rec

    # -------------------------------------------------------------- zaman
    def wait_sim(self, dt, timeout=None):
        w0 = time.time()
        limit = timeout if timeout is not None else 10 * dt + 10
        while self.now() is None:
            if time.time() - w0 > limit:
                raise TimeoutError('sim zamanı gelmiyor (/stats)')
            time.sleep(0.05)
        t0 = self.now()
        while self.now() is None or self.now() - t0 < dt:
            if time.time() - w0 > limit:
                raise TimeoutError(f'sim zamanı {dt} s ilerlemedi')
            time.sleep(0.01)

    def wait_until(self, cond, timeout, what):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if cond():
                return
            time.sleep(0.05)
        raise TimeoutError(what)

    # ----------------------------------------------------------- komutlar
    def cmd(self, v, w=0.0):
        m = self.Twist()
        m.linear.x, m.angular.z = v, w
        self.cmd_pub.publish(m)

    def set_fork(self, q):
        m = self.Double()
        m.data = q
        self.fork_pub.publish(m)

    def _req(self, service, req, req_t, rep_t, timeout_ms=5000, attempts=3):
        """Servis isteği AYRI SÜREÇTEN (gz service CLI) gönderilir (D-49).

        Bu süreçteki sürekli abonelikler (poz 60 Hz, eklem 50 Hz, iki kamera 30 Hz)
        varken gz-transport Python Node.request() yanıtları zaman aşımına düşüyor
        (create/blocking hiç dönmedi, set_pose ara sıra 5 s bekledi); ayrı süreçte
        aynı istekler ~0.3 s'de dönüyor.
        """
        from google.protobuf import text_format
        text = text_format.MessageToString(req, as_one_line=True)
        cmd = ['gz', 'service', '-s', service, '--reqtype', f'gz.msgs.{req_t.__name__}',
               '--reptype', f'gz.msgs.{rep_t.__name__}', '--timeout', str(timeout_ms), '--req', text]
        tried = []
        for _ in range(attempts):
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 20)
            if 'data: true' in p.stdout:
                return True
            tried.append((p.stdout + p.stderr).strip()[:120] or 'yanıt yok')
            time.sleep(0.5)
        raise RuntimeError(f'{service} başarısız: {tried}')

    def set_pose(self, name, p, R):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose as PoseMsg
        m = PoseMsg()
        m.name = name
        m.position.x, m.position.y, m.position.z = (float(v) for v in p)
        q = R_to_quat(np.asarray(R))
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = (float(v) for v in q)
        self._req(f'/world/{self.w}/set_pose', m, PoseMsg, Boolean)

    def spawn(self, name, sdf, p, R, timeout_ms=60000):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        m = EntityFactory()
        m.sdf, m.name, m.allow_renaming = sdf, name, False
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        q = R_to_quat(np.asarray(R))
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = \
            (float(v) for v in q)
        # blocking: model gerçekten eklenince döner; ad çakışmasında data=false
        # (async /create her durumda true döner, Faz 4'te ölçüldü)
        # tek deneme, uzun zaman aşımı: tekrar denemek ad çakışması (data=false) verir
        self._req(f'/world/{self.w}/create/blocking', m, EntityFactory, Boolean, timeout_ms, attempts=1)

    def remove(self, name):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_pb2 import Entity
        m = Entity()
        m.name, m.type = name, Entity.MODEL
        self._req(f'/world/{self.w}/remove/blocking', m, Entity, Boolean, attempts=1)

    def close(self):
        with self.lock:
            self._rec = None
        for n in list(self.cam_topics.values()):
            self.node.unsubscribe(n)


def save_rgb(arr, path):
    import cv2
    cv2.imwrite(path, cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))


def wait_world_ready(st, gz, sess, parcels, timeout=120):
    """Robot ve tüm koliler dünyada, stations.launch zincirini bitirmiş."""
    names = [gz.MODEL] + [p[0] for p in parcels]

    def ready():
        with gz.lock:
            have = set(gz.poses)
        return gz.now() is not None and all(n in have for n in names) and sess.spawned()
    gz.wait_until(ready, timeout, f'dünya hazır değil ({names})')
    # Başlangıçta sunucu sensör/render başlatmasıyla meşgul olabilir: sim zamanı
    # gerçekten akana kadar bekle. Kayıt şimdiden başlar; S3 onu sürdürür, böylece
    # S3 penceresi spawn bitiminden başlar.
    gz.record(True)
    gz.wait_sim(1.0, timeout=timeout)


# =============================================================================
#  S3 — statik kararlılık: tam dünya, 10 s, kolilerin spawn pozundan sapması
# =============================================================================
S3_DURATION = 10.0
S3_TOL = 0.001


def s3(st, rep, gz, parcels):
    nominal = {n: np.array([x, y, z]) for n, _, (x, y, z, yaw) in parcels}
    nominal_yaw = {n: yaw for n, _, (x, y, z, yaw) in parcels}
    # kayıt wait_world_ready'de başladı; ilk örnekten itibaren S3_DURATION
    with gz.lock:
        t_first = gz._rec[0][0] if gz._rec else gz.pose_t
    gz.wait_until(lambda: gz.pose_t is not None and gz.pose_t - t_first >= S3_DURATION,
                  10 * S3_DURATION + 30, 'S3 kaydı tamamlanmadı')
    t0 = t_first
    rec = gz.record(False)
    rows, worst = {}, 0.0
    for n in nominal:
        samples = [(t, d[n]) for t, d in rec if n in d]
        dp = [np.linalg.norm(p - nominal[n]) for _, (p, q) in samples]
        rot = [math.degrees(abs(math.atan2(math.sin(yaw_of_R(quat_to_R(q)) - nominal_yaw[n]),
                                           math.cos(yaw_of_R(quat_to_R(q)) - nominal_yaw[n]))))
               for _, (p, q) in samples]
        tilt = [tilt_deg(quat_to_R(q)) for _, (p, q) in samples]
        drift = np.linalg.norm(samples[-1][1][0] - samples[0][1][0])
        rows[n] = {'samples': len(samples), 'span_s': samples[-1][0] - samples[0][0],
                   'max_disp_from_spawn_mm': max(dp) * 1000, 'drift_first_last_mm': drift * 1000,
                   'max_yaw_change_deg': max(rot), 'max_tilt_deg': max(tilt),
                   'final_xyz': samples[-1][1][0].round(5).tolist()}
        worst = max(worst, max(dp))
    with open(os.path.join(st.logs, 'S3_parcels.json'), 'w') as f:
        json.dump({'t0_sim': t0, 'parcels': rows}, f, indent=2)
    meas = '; '.join(f'{n}: en çok {r["max_disp_from_spawn_mm"]:.3g} mm, eğim {r["max_tilt_deg"]:.3g}°'
                     for n, r in rows.items())
    rep.add(Result('S3', TEST_NAMES['S3'], PASS if worst < S3_TOL else FAIL,
                   f'{meas} ({S3_DURATION:.0f} s sim, spawn pozuna göre)',
                   'Tam dünya, 10 s: hiçbir koli 1 mm\'den fazla hareket etmiyor', {'parcels': rows}))


# =============================================================================
#  S8 — performans: tam dünya, robotun iki kamerası abone, RTF
# =============================================================================
S8_DURATION = 20.0


def s8(st, rep, gz):
    c0 = dict(gz.img_count)
    w0, s0 = time.monotonic(), gz.now()
    time.sleep(S8_DURATION)
    w1, s1 = time.monotonic(), gz.now()
    rtf = (s1 - s0) / (w1 - w0)
    hz = {n: (gz.img_count.get(n, 0) - c0.get(n, 0)) / (w1 - w0) for n in gz.cam_topics}
    cams = ', '.join(f'{n} {v:.1f} Hz' for n, v in hz.items())
    ok = rtf >= 0.9 and all(v > 0 for v in hz.values())
    rep.add(Result('S8', TEST_NAMES['S8'], PASS if ok else FAIL,
                   f'RTF {rtf:.3f} ({S8_DURATION:.0f} s duvar; {"GUI" if st.gui else "ekransız EGL"}; '
                   f'4 istasyon, 3 koli; kameralar: {cams})',
                   'RTF ≥ 0.9, iki kamera açık', {'rtf': rtf, 'camera_hz': hz}))


def run_sim_tests(st, rep, tests):
    """S3–S8: tek oturum. S3 spawn anından ölçmek için ilk, S8 dünya bozulmadan ikinci sırada."""
    parcels = sl.parcels(st.params, st.layout)
    order = [t for t in ('S3', 'S8', 'S4', 'S5', 'S6', 'S7') if t in tests]
    done = set()
    with SimSession(st, 'stations') as sess:
        gz = None
        try:
            gz = Gz(st)
            wait_world_ready(st, gz, sess, parcels)
            if 'S3' not in order:
                gz.record(False)
            for t in order:
                done.add(t)
                try:
                    if t == 'S3':
                        s3(st, rep, gz, parcels)
                    elif t == 'S8':
                        s8(st, rep, gz)
                    elif t in SIM_TESTS:
                        SIM_TESTS[t](st, rep, gz, parcels)
                    else:
                        rep.add(Result(t, TEST_NAMES[t], NOT_RUN, 'bu sürümde uygulanmadı', '—'))
                except Exception as e:
                    traceback.print_exc()
                    rep.add(Result(t, TEST_NAMES[t], NOT_RUN, f'altyapı hatası: {e}', '—'))
        except Exception as e:
            traceback.print_exc()
            for t in order:
                if t not in done:
                    rep.add(Result(t, TEST_NAMES[t], NOT_RUN, f'oturum hatası: {e}', '—'))
        finally:
            if gz is not None:
                gz.close()


# =============================================================================
#  ArUco yardımcıları (OpenCV 4.6 API, D-47)
# =============================================================================
# Kamera bağlantı çerçevesinden (x ileri, y sol, z yukarı) OpenCV optik
# çerçeveye (x sağ, y aşağı, z ileri): sütunlar link çerçevesinde.
R_LINK_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def rpy_to_R(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class Aruco:
    def __init__(self, st):
        import cv2
        self.cv2 = cv2
        self.dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, st.params['markers']['dictionary']))
        det = st.params['markers']['detector']
        self.prm = cv2.aruco.DetectorParameters_create()
        self.prm.cornerRefinementMethod = getattr(cv2.aruco, det['corner_refinement'])
        self.prm.perspectiveRemoveIgnoredMarginPerCell = det['perspective_remove_ignored_margin_per_cell']
        self.markers = {m['id']: m for m in sl.markers(st.params, st.layout)}

    def detect(self, rgb):
        """{id: köşeler (4×2, TL TR BR BL)}"""
        gray = self.cv2.cvtColor(rgb, self.cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.cv2.aruco.detectMarkers(gray, self.dict, parameters=self.prm)
        if ids is None:
            return {}
        return {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}

    def pose(self, corners, size, K):
        """IPPE_SQUARE; işaret pozu optik çerçevede (R, t)."""
        h = size / 2.0
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
        ok, rvec, tvec = self.cv2.solvePnP(obj, corners.astype(np.float64), K, None,
                                           flags=self.cv2.SOLVEPNP_IPPE_SQUARE)
        R, _ = self.cv2.Rodrigues(rvec)
        return R, tvec.ravel()

    def marker_world(self, mid):
        m = self.markers[mid]
        return Pose(m['pose_world']['xyz'], rpy_to_R(m['pose_world']['rpy']))

    def annotate(self, rgb, det, label):
        cv2 = self.cv2
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
        if det:
            ids = np.array(list(det.keys())).reshape(-1, 1)
            cv2.aruco.drawDetectedMarkers(img, [c.reshape(1, 4, 2).astype(np.float32) for c in det.values()], ids)
        cv2.rectangle(img, (0, 0), (img.shape[1], 26), (0, 0, 0), -1)
        # Hershey yazı tipi yalnız ASCII çizer: Türkçe harfler sadeleştirilir
        label = label.translate(str.maketrans('çğıöşüÇĞİÖŞÜ°', 'cgiosuCGIOSU ')).replace('—', '-')
        cv2.putText(img, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return img


def K_from(hfov, w, h):
    f = (w / 2.0) / math.tan(hfov / 2.0)
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1]])


def montage(tiles, cols, path, scale=0.5):
    import cv2
    small = [cv2.resize(t, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) for t in tiles]
    h, w = small[0].shape[:2]
    rows = (len(small) + cols - 1) // cols
    canvas = np.full((rows * h, cols * w, 3), 255, np.uint8)
    for k, t in enumerate(small):
        r, c = divmod(k, cols)
        canvas[r * h:(r + 1) * h, c * w:(c + 1) * w] = t
    cv2.imwrite(path, canvas)


def station_entry(st, name):
    return next(s for s in st.layout['stations'] if s['name'] == name)


def yaw_err_deg(R_est, R_true):
    """İşaretin optik y (≈ düşey) ekseni etrafındaki yönelim farkı."""
    a = math.atan2(R_est[0, 2], R_est[2, 2])
    b = math.atan2(R_true[0, 2], R_true[2, 2])
    return math.degrees(math.atan2(math.sin(a - b), math.cos(a - b)))


# Park yerleri: dünya çerçevesinde, istasyon ayak izleri ve robotun test
# yollarından uzakta (stations.yaml yerleşimine göre; testin kendi seçimi).
PARK_PARCEL = (0.6, -1.2)
PARK_ROBOT = (-0.9, -1.0)


class RobotGeom:
    """Robot ölçüleri forklift_params.yaml'dan (base_footprint çerçevesinde)."""

    def __init__(self, st):
        r = st.robot
        fc, fk, cam = r['fork_carriage'], r['fork'], r['cameras']['front_camera']
        self.carriage_front_x = fc['x'] + fc['size']['x'] / 2.0
        self.fork_top0 = fk['ground_z_bottom'] + fk['size']['z']
        self.fork_len = fk['size']['x']
        self.cam = cam
        self.fork_v = r['fork_lift_joint']['velocity']

    def front_cam_opt(self, bf):
        """Ön kamera optik çerçevesinin dünya pozu (bf: base_footprint dünya pozu)."""
        c = self.cam
        return bf * Pose([c['x'], c['y'], c['ground_z']], Ry(c['pitch']) @ R_LINK_OPT)


def robot_bf(gz):
    bf = gz.link(gz.MODEL, 'base_footprint')
    return bf if bf is not None else gz.model(gz.MODEL)


def place_robot(gz, p_xy, yaw, z=0.003, settle=0.5):
    gz.cmd(0.0)
    gz.set_pose(gz.MODEL, [p_xy[0], p_xy[1], z], Rz(yaw))
    gz.wait_sim(settle)


def park_parcel(gz, name):
    gz.set_pose(name, [PARK_PARCEL[0], PARK_PARCEL[1], 0.105], np.eye(3))
    gz.wait_sim(0.3)


def fork_to(gz, rg, q, timeout=None):
    """Çatal hedefi; |q - hedef| < 1 mm olunca döner. (ulaştı mı, son q)"""
    gz.set_fork(q)
    t_lim = timeout if timeout is not None else 2.0 + 2.0 * 0.15 / rg.fork_v
    t0 = gz.now()
    while gz.now() - t0 < t_lim:
        f = gz.fork()
        if f is not None and abs(f - q) < 0.001:
            gz.wait_sim(0.3)
            return True, gz.fork()
        time.sleep(0.02)
    return False, gz.fork()


# =============================================================================
#  S4 — üst işaret tespiti (robotun ön kamerası)
# =============================================================================
S4_DIST = (0.6, 1.0, 1.5)
S4_ANG = (-15.0, 0.0, 15.0)
S4_FRAMES = 5
S4_MIN_HITS = 4


def view_pose_for(gz, rg, marker_pose, yaw_s, D, ang):
    """Kameranın işarete D (yatay) uzaklıkta, işaret normalinden ang açıda durduğu robot pozu."""
    psi = yaw_s + math.radians(ang)          # robotun bakış yönü (işarete doğru)
    m = marker_pose.p
    cam_xy = m[:2] - D * np.array([math.cos(psi), math.sin(psi)])
    bf_xy = cam_xy - rg.cam['x'] * np.array([math.cos(psi), math.sin(psi)])
    return bf_xy, psi


def s4_view(st, gz, ar, rg, target_id, yaw_s, D, ang):
    mp = ar.marker_world(target_id)
    bf_xy, psi = view_pose_for(gz, rg, mp, yaw_s, D, ang)
    place_robot(gz, bf_xy, psi, settle=0.6)
    t_set = gz.now()
    hits, seen, d_err, frame, det_last = 0, set(), [], None, {}
    K = K_from(rg.cam['horizontal_fov'], rg.cam['image']['width'], rg.cam['image']['height'])
    for _ in range(S4_FRAMES):
        img = gz.image('front_camera', newer_than=t_set)
        if img is None:
            raise RuntimeError('ön kamera görüntüsü gelmiyor')
        t_set = gz.now()
        det = ar.detect(img)
        seen |= set(det)
        if target_id in det:
            hits += 1
            _, tvec = ar.pose(det[target_id], ar.markers[target_id]['size'], K)
            cam = rg.front_cam_opt(robot_bf(gz))
            true_t = (cam.inv() * mp).p
            d_err.append(np.linalg.norm(tvec) - np.linalg.norm(true_t))
        frame, det_last = img, det
    if st.raw_dir:
        save_rgb(frame, os.path.join(st.raw_dir, f's4_ID{target_id}_{D:.1f}_{ang:+.0f}.png'))
    cam = rg.front_cam_opt(robot_bf(gz))
    true_D = abs(np.dot(mp.p - cam.p, -mp.R[:, 2]))   # işaret düzlemine yatay/normal uzaklık
    return {'D': D, 'ang': ang, 'hits': hits, 'seen': sorted(seen), 'true_plane_dist': true_D,
            'dist_err_mm': (float(np.mean(d_err)) * 1000 if d_err else None)}, frame, det_last


def s4(st, rep, gz, parcels):
    ar, rg = Aruco(st), RobotGeom(st)
    known = set(ar.markers)
    groups = []
    # (etiket, istasyon, bölme, beklenen tespit, gizlenecek koli)
    dest, src = station_entry(st, 'dest_red'), station_entry(st, 'source')
    groups.append(('boş bölme', dest, 1, True, None))
    groups.append(('dolu bölme', src, 1, False, None))
    green = next(n for n, c, _ in parcels if n.endswith('_1') and n.startswith('parcel_source'))
    groups.append(('boş bölme, komşular dolu (bilgi)', src, 1, True, green))
    rows, tiles, fails = [], [], []
    for label, stn, i, expect, hide in groups:
        target = sl.marker_ids(st.params, stn['bay_ids'][i])['upper']
        nominal = None
        if hide:
            nominal = gz.model(hide)
            park_parcel(gz, hide)
        for D in S4_DIST:
            for ang in S4_ANG:
                r, img, det = s4_view(st, gz, ar, rg, target, stn['pose'][2], D, ang)
                r.update({'group': label, 'station': stn['name'], 'bay': i, 'target': target, 'expect': expect})
                unknown = sorted(set(r['seen']) - known)
                full_bays = {sl.marker_ids(st.params, s['bay_ids'][j])['upper']
                             for s in st.layout['stations'] for j, c in enumerate(s['occupancy']) if c}
                if hide:
                    full_bays.discard(target)
                hidden_seen = sorted(set(r['seen']) & full_bays)
                ok = (r['hits'] >= S4_MIN_HITS) if expect else (r['hits'] == 0)
                ok = ok and not unknown and not hidden_seen
                r.update({'ok': ok, 'unknown_ids': unknown, 'full_bay_ids_seen': hidden_seen})
                rows.append(r)
                if not ok and 'bilgi' not in label:
                    fails.append(f'{label} D={D} {ang:+.0f}°: {r["hits"]}/{S4_FRAMES}')
                tiles.append(ar.annotate(img, det, f'{label} | ID {target} | D {D} m {ang:+.0f} deg | '
                                                   f'{r["hits"]}/{S4_FRAMES}'))
        if hide:
            gz.set_pose(hide, nominal.p, nominal.R)
            gz.wait_sim(0.5)
    import cv2
    # karo indeksi = grup·9 + mesafe·3 + açı (S4_DIST × S4_ANG sırası)
    for k, name in ((1, 's4_empty_0p6m_0deg.png'), (8, 's4_empty_1p5m_p15deg.png'),
                    (13, 's4_full_1p0m_0deg.png'), (24, 's4_neighbours_full_1p5m_-15deg.png')):
        cv2.imwrite(os.path.join(st.images, name), tiles[k])
    for g in range(3):
        montage(tiles[g * 9:(g + 1) * 9], 3, os.path.join(st.images, f's4_montage_{g}.png'))
    with open(os.path.join(st.logs, 'S4_views.json'), 'w') as f:
        json.dump(rows, f, indent=2, default=float)
    main = [r for r in rows if 'bilgi' not in r['group']]
    emp = [r for r in main if r['expect']]
    ful = [r for r in main if not r['expect']]
    info = [r for r in rows if 'bilgi' in r['group']]
    derr = [abs(r['dist_err_mm']) for r in emp if r['dist_err_mm'] is not None]
    meas = (f'boş bölme (ID {emp[0]["target"]}): {sum(r["ok"] for r in emp)}/{len(emp)} görüşte '
            f'≥{S4_MIN_HITS}/{S4_FRAMES} karede tespit, uzaklık hatası ≤ {max(derr):.1f} mm; '
            f'dolu bölme (ID {ful[0]["target"]}): {sum(r["ok"] for r in ful)}/{len(ful)} görüşte hiç tespit yok; '
            f'bilgi — komşular dolu, bölme boş: {sum(r["ok"] for r in info)}/{len(info)} görüş')
    rep.add(Result('S4', TEST_NAMES['S4'], PASS if not fails else FAIL, meas,
                   'Ön kamera, 0.6/1.0/1.5 m × 0°/±15°: boş bölmede doğru ID; dolu bölmenin üst işareti '
                   'tespit edilmez; bilinmeyen ID yok', {'fails': fails, 'views': rows}))


# =============================================================================
#  S5 — alt işaret tespiti (teste özel statik kamera; araçta değişiklik yok, D-48)
# =============================================================================
S5_NAME = 's5_test_camera'
S5_TOPIC = 's5_test_camera/image'
S5_HFOV = math.radians(80.0)
S5_W, S5_H = 640, 480
S5_PITCH = math.radians(5.0)          # aşağı bakış
S5_Z = (0.02, 0.08)
S5_DIST = (0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8)
S5_FRAMES = 5
S5_POSE_FRAMES = 30
S5_POSE_D = 0.5
S5_LAT_TOL = 0.005
S5_YAW_TOL = 1.0


def s5_camera_sdf(st):
    # gürültü robot kameralarıyla aynı (forklift_params.yaml cameras.*.noise)
    n = st.robot['cameras']['front_camera']['noise']
    # Dinamik ama yerçekimsiz ve collision'sız: hiçbir kuvvet almaz, set_pose ile
    # taşınır ve gerçek pozu dynamic_pose'ta görünür (statik modelde ikisi de yok).
    return f'''<?xml version="1.0"?>
<sdf version="1.11"><model name="{S5_NAME}"><static>false</static><link name="link">
  <gravity>false</gravity>
  <inertial><mass>0.01</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia></inertial>
  <sensor name="camera" type="camera">
    <topic>{S5_TOPIC}</topic><update_rate>30</update_rate><always_on>1</always_on>
    <camera>
      <horizontal_fov>{S5_HFOV}</horizontal_fov>
      <image><width>{S5_W}</width><height>{S5_H}</height><format>R8G8B8</format></image>
      <clip><near>0.005</near><far>10</far></clip>
      <noise><type>{n['type']}</type><mean>{n['mean']}</mean><stddev>{n['stddev']}</stddev></noise>
    </camera>
  </sensor>
</link></model></sdf>'''


S5_SWEEP_YAW = (-5.0, -2.0, 2.0, 5.0)        # [°] bilgi: yaw duyarlılığı
S5_SWEEP_LAT = (-0.02, -0.01, 0.01, 0.02)    # [m] bilgi: yanal ofset duyarlılığı


def s5_view(st, gz, ar, sdf, stn, i, z, D, frames, lateral=0.0, yaw_deg=0.0):
    """Kamerayı bölme ekseninde (± lateral), işaret yüzünden D uzakta, z yüksekliğinde kurar."""
    bay = bay_pose(st, stn, i)
    cam_link = bay * Pose([sl.marker_face_x(st.params) - D, lateral, z], Rz(math.radians(yaw_deg)) @ Ry(S5_PITCH))
    if not getattr(gz, 's5_cam', False):
        gz.spawn(S5_NAME, sdf, cam_link.p, cam_link.R)
        gz.s5_cam = True
    else:
        gz.set_pose(S5_NAME, cam_link.p, cam_link.R)
    gz.wait_sim(0.3)
    actual = gz.model(S5_NAME)
    if actual is None or np.linalg.norm(actual.p - cam_link.p) > 1e-4:
        raise RuntimeError(f'test kamerası pozu tutmadı: {None if actual is None else actual.p}')
    t = gz.now()
    for _ in range(3):                      # taşımadan önceki/sıradaki kareleri at
        if gz.image('s5', newer_than=t, timeout=10) is None:
            raise RuntimeError('test kamerası görüntüsü gelmiyor')
        t = gz.now()
    imgs = []
    for _ in range(frames):
        img = gz.image('s5', newer_than=t, timeout=5)
        t = gz.now()
        imgs.append(img)
    return cam_link * Pose([0, 0, 0], R_LINK_OPT), imgs


def _inside_parcel(st, p, parcel_pose):
    P = st.params['parcel']['size']
    local = parcel_pose.inv().apply(p)
    return all(abs(local[k]) < P[a] / 2.0 for k, a in enumerate('xyz'))


def s5(st, rep, gz, parcels):
    ar = Aruco(st)
    K = K_from(S5_HFOV, S5_W, S5_H)
    sdf = s5_camera_sdf(st)
    gz.subscribe_camera('s5', S5_TOPIC)
    src = station_entry(st, 'source')
    i = 1
    target = sl.marker_ids(st.params, src['bay_ids'][i])['lower']
    green = f'parcel_source_{i}'
    nominal = gz.model(green)
    place_robot(gz, PARK_ROBOT, 0.0)
    size = ar.markers[target]['size']
    mp = ar.marker_world(target)
    cases = [('z0.02_dolu', 0.02, True, True), ('z0.02_bos', 0.02, False, True),
             ('z0.08_bos', 0.08, False, True), ('z0.08_dolu', 0.08, True, False)]
    rows, tiles, fails, pose_rows = [], [], [], []
    import cv2
    for tag, z, full, judged in cases:
        if full:
            gz.set_pose(green, nominal.p, nominal.R)
        else:
            park_parcel(gz, green)
        gz.wait_sim(0.5)
        for D in S5_DIST:
            n = S5_POSE_FRAMES if (abs(D - S5_POSE_D) < 1e-9 and judged) else S5_FRAMES
            cam_opt, imgs = s5_view(st, gz, ar, sdf, src, i, z, D, n)
            if st.raw_dir:
                save_rgb(imgs[0], os.path.join(st.raw_dir, f's5_{tag}_{D:.1f}.png'))
            dets = [ar.detect(im) for im in imgs]
            hits = sum(target in d for d in dets)
            # kamera koli gövdesinin içinde mi (render arka yüzleri çizmez → geçersiz görüş)
            inside = full and _inside_parcel(st, cam_opt.p, gz.model(green))
            seen = sorted(set().union(*[set(d) for d in dets]))
            r = {'case': tag, 'z': z, 'D': D, 'parcel': full, 'hits': hits, 'frames': len(imgs), 'seen': seen,
                 'judged': judged, 'camera_inside_parcel': bool(inside)}
            if hits and n == S5_POSE_FRAMES:
                true = cam_opt.inv() * mp
                lat, dep, yaw = [], [], []
                for d in dets:
                    if target not in d:
                        continue
                    R, t = ar.pose(d[target], size, K)
                    lat.append(t[0] - true.p[0])
                    dep.append(t[2] - true.p[2])
                    yaw.append(yaw_err_deg(R, true.R))
                pr = {'case': tag, 'n': len(lat),
                      'lat_mean_mm': float(np.mean(lat)) * 1000, 'lat_std_mm': float(np.std(lat)) * 1000,
                      'depth_mean_mm': float(np.mean(dep)) * 1000, 'depth_std_mm': float(np.std(dep)) * 1000,
                      'yaw_mean_deg': float(np.mean(yaw)), 'yaw_std_deg': float(np.std(yaw)),
                      'yaw_max_abs_deg': float(np.max(np.abs(yaw)))}
                pr['ok'] = abs(pr['lat_mean_mm']) < S5_LAT_TOL * 1000 and abs(pr['yaw_mean_deg']) < S5_YAW_TOL
                pose_rows.append(pr)
                r['pose'] = pr
                if not pr['ok']:
                    fails.append(f'{tag} poz: yanal {pr["lat_mean_mm"]:.2f} mm, yaw {pr["yaw_mean_deg"]:.2f}°')
            ok = (hits >= len(imgs) - 1) if judged else True
            if inside:
                r['note'] = 'kamera koli gövdesinin içinde: arka yüzler çizilmez, görüş geçersiz'
            r['ok'] = ok
            if judged and not ok:
                fails.append(f'{tag} D={D}: {hits}/{len(imgs)}')
            if any(m not in ar.markers for m in seen):
                fails.append(f'{tag} D={D}: bilinmeyen ID {seen}')
            rows.append(r)
            tiles.append(ar.annotate(imgs[-1], dets[-1], f'{tag} | ID {target} | D {D} m | {hits}/{len(imgs)}'))
            if D in (0.2, 0.5, 0.8):
                cv2.imwrite(os.path.join(st.images, f's5_{tag.replace(".", "p")}_{str(D).replace(".", "p")}m.png'),
                            tiles[-1])
    # Bilgi: bilinen yaw / yanal ofsetlerde kestirim (D = 0.5 m, z = 0.02, koli yok)
    park_parcel(gz, green)
    sweep = []
    for lat, yd in [(0.0, y) for y in S5_SWEEP_YAW] + [(lo, 0.0) for lo in S5_SWEEP_LAT]:
        cam_opt, imgs = s5_view(st, gz, ar, sdf, src, i, 0.02, S5_POSE_D, 3, lateral=lat, yaw_deg=yd)
        d = ar.detect(imgs[0])
        row = {'lateral_m': lat, 'yaw_deg': yd, 'detected': target in d}
        if target in d:
            true = cam_opt.inv() * mp
            R, t = ar.pose(d[target], size, K)
            row.update({'true_x_mm': true.p[0] * 1000, 'est_x_mm': t[0] * 1000,
                        'lat_err_mm': (t[0] - true.p[0]) * 1000, 'yaw_err_deg': yaw_err_deg(R, true.R)})
        sweep.append(row)
    gz.remove(S5_NAME)
    gz.s5_cam = False
    gz.set_pose(green, nominal.p, nominal.R)
    gz.wait_sim(0.5)
    for k, (tag, *_rest) in enumerate(cases):
        montage(tiles[k * len(S5_DIST):(k + 1) * len(S5_DIST)], 4,
                os.path.join(st.images, f's5_montage_{tag.replace(".", "p")}.png'))
    with open(os.path.join(st.logs, 'S5_views.json'), 'w') as f:
        json.dump({'camera': {'hfov_deg': 80, 'size': [S5_W, S5_H], 'pitch_down_deg': 5,
                              'noise': st.robot['cameras']['front_camera']['noise']},
                   'views': rows, 'pose': pose_rows, 'sweep': sweep}, f, indent=2, default=float)
    det_summary = '; '.join(
        f'{tag}: {sum(r["hits"] >= r["frames"] - 1 for r in rows if r["case"] == tag)}/{len(S5_DIST)} mesafe'
        for tag, *_ in cases)
    pose_summary = '; '.join(
        f'{p["case"]} @0.5 m: yanal {p["lat_mean_mm"]:+.2f}±{p["lat_std_mm"]:.2f} mm, '
        f'yaw {p["yaw_mean_deg"]:+.2f}±{p["yaw_std_deg"]:.2f}° (n={p["n"]})' for p in pose_rows)
    info = [r for r in rows if r['case'] == 'z0.08_dolu']
    info_txt = ', '.join(f'{r["D"]}: {"kolinin içi" if r["camera_inside_parcel"] else r["hits"]}' for r in info)
    same = all(p['lat_std_mm'] < 1e-6 and p['yaw_std_deg'] < 1e-6 for p in pose_rows)
    sw = [r for r in sweep if r['detected']]
    sweep_txt = ', '.join(
        (f'yaw {r["yaw_deg"]:+.0f}° → hata {r["yaw_err_deg"]:+.1f}°' if r['yaw_deg'] else
         f'yanal {r["lateral_m"] * 100:+.0f} cm → hata {r["lat_err_mm"]:+.2f} mm') for r in sw)
    rep.add(Result('S5', TEST_NAMES['S5'], PASS if not fails else FAIL,
                   f'ID {target} tespiti (≥ kare-1): {det_summary}. Bilgi z0.08_dolu (beklenen: yok): {info_txt}. '
                   f'Poz: {pose_summary}{" — 30 kare birebir aynı (kamera gürültüsü uygulanmıyor)" if same else ""}'
                   f'. Bilgi taraması @0.5 m: {sweep_txt}',
                   '0.2–0.8 m, z = 0.02 koli var/yok ve z = 0.08 koli yok (D-48): doğru ID; 0.5 m\'de '
                   '30 kare ortalaması yanal < 5 mm, yaw < 1°',
                   {'fails': fails, 'views': rows, 'pose': pose_rows, 'sweep': sweep}))


# =============================================================================
#  S6 / S7 — bırakma ve alma (kaynak istasyonun orta bölmesi, komşular dolu)
# =============================================================================
S6_OFFSETS = (0.0, 0.01, -0.01, 0.02, -0.02, 0.025, -0.025, 0.03, -0.03)   # ±2.5 cm: bilgi (sınır)
S6_CARRY_Q = 0.06
S6_REVERSE = 0.35
S7_REVERSE = 0.5
REVERSE_V = 0.15
S6_PARCEL_GAP = 0.002          # koli ön yüzü ↔ taşıyıcı ön yüzü (yerleştirme)
DOC_CAM = 'doc_camera'


def doc_camera_sdf():
    return f'''<?xml version="1.0"?>
<sdf version="1.11"><model name="{DOC_CAM}"><static>true</static><link name="link">
  <sensor name="camera" type="camera"><topic>{DOC_CAM}/image</topic><update_rate>10</update_rate>
    <always_on>1</always_on>
    <camera><horizontal_fov>1.0</horizontal_fov>
      <image><width>960</width><height>720</height><format>R8G8B8</format></image>
      <clip><near>0.02</near><far>20</far></clip></camera>
  </sensor></link></model></sdf>'''


def ensure_doc_camera(st, gz, stn, i):
    """Belgeleme kamerası (statik, bölmeye önden-yandan-yukarıdan bakar); yoksa ekler."""
    if getattr(gz, 'doc_cam', False):
        return
    gz.subscribe_camera('doc', f'{DOC_CAM}/image')
    # önden-soldan-yukarıdan, bölme merkezine (0.1, 0, 0.1) bakış
    doc = bay_pose(st, stn, i) * Pose([-0.8, 0.8, 0.8], Rz(math.radians(-41.6)) @ Ry(math.radians(30)))
    gz.spawn(DOC_CAM, doc_camera_sdf(), doc.p, doc.R)
    gz.doc_cam = True
    gz.wait_sim(0.5)


def doc_shot(st, gz, name):
    t = gz.now()
    img = gz.image('doc', newer_than=t, timeout=5)
    if img is not None:
        save_rgb(img, os.path.join(st.images, name))


def entry_pose(st, rg, stn, i, offset):
    """Tam giriş: taşıyıcı ön yüzü bölme x = 0'da, yanal ofsetli; base_footprint dünya xy ve yaw."""
    bay = bay_pose(st, stn, i)
    p = bay.apply([-rg.carriage_front_x, offset, 0.0])
    return p[:2], stn['pose'][2]


def reverse(gz, dist):
    """Gerçek poza göre dist kadar geri; (gerçek mesafe, kayıt)."""
    bf0 = robot_bf(gz)
    fwd = bf0.R[:, 0]
    stop_at = dist - REVERSE_V ** 2 / (2 * 1.0)        # DiffDrive yavaşlama 1 m/s² (forklift_params)
    gz.record(True)
    t0 = time.time()
    while np.dot(robot_bf(gz).p - bf0.p, -fwd) < stop_at:
        gz.cmd(-REVERSE_V)
        if time.time() - t0 > 30:
            break
        time.sleep(0.02)
    gz.cmd(0.0)
    gz.wait_sim(1.0)
    gz.cmd(0.0)
    rec = gz.record(False)
    return float(np.dot(robot_bf(gz).p - bf0.p, -fwd)), rec


def parcel_on_fork_pose(st, rg, gz, q):
    bf = robot_bf(gz)
    P = st.params['parcel']['size']
    local = [rg.carriage_front_x + S6_PARCEL_GAP + P['x'] / 2.0, 0.0,
             rg.fork_top0 + q + P['z'] / 2.0 + 0.002]
    return bf * Pose(local, np.eye(3))


def parcel_state(st, gz, name, stn, i):
    pp = gz.model(name)
    bay = bay_pose(st, stn, i)
    rel = bay.inv() * pp
    P = st.params['parcel']['size']
    S = st.params['station']
    bottom = rel.p[2] - P['z'] / 2.0
    overlap = P['y'] / 2.0 - abs(rel.p[1]) - S['channel_width'] / 2.0
    return {'x': rel.p[0], 'y': rel.p[1], 'bottom_z': bottom, 'tilt_deg': tilt_deg(pp.R),
            'min_overlap_m': overlap}


def s6(st, rep, gz, parcels):
    rg = RobotGeom(st)
    src = station_entry(st, 'source')
    i = 1
    name = f'parcel_source_{i}'
    nominal = gz.model(name)
    S = st.params['station']
    ensure_doc_camera(st, gz, src, i)
    bay = bay_pose(st, src, i)
    stage_xy = bay.apply([-1.0, 0.0, 0.0])[:2]
    # komşu bölmelerdeki koliler: S6 boyunca yer değiştirmemeli (D-43, robot gövdesi/temas)
    neighbours = [n for n, _, _ in parcels if n.startswith('parcel_source_') and n != name]
    nb0 = {n: gz.model(n).p.copy() for n in neighbours}
    rows = []
    for off in S6_OFFSETS:
        place_robot(gz, stage_xy, src['pose'][2])
        fork_to(gz, rg, S6_CARRY_Q)
        park_parcel(gz, name)
        xy, yaw = entry_pose(st, rg, src, i, off)
        place_robot(gz, xy, yaw, settle=0.8)
        bf = robot_bf(gz)
        pitch = math.degrees(math.asin(-bf.R[2, 0]))
        pp = parcel_on_fork_pose(st, rg, gz, gz.fork())
        gz.set_pose(name, pp.p, pp.R)
        gz.wait_sim(1.0)
        carried = parcel_state(st, gz, name, src, i)
        reached, q_end = fork_to(gz, rg, 0.0, timeout=6.0)
        gz.wait_sim(1.0)
        q_end = gz.fork()
        placed = parcel_state(st, gz, name, src, i)
        bf_low = robot_bf(gz)
        pitch_low = math.degrees(math.asin(-bf_low.R[2, 0]))
        roll_low = math.degrees(math.atan2(bf_low.R[2, 1], bf_low.R[2, 2]))
        if off in (0.0, 0.02, 0.03):
            doc_shot(st, gz, f's6_lowered_offset_{int(round(off * 100))}cm.png')
        before = gz.model(name).p
        dist, rec = reverse(gz, S6_REVERSE)
        drag = max(np.linalg.norm(d[name][0][:2] - before[:2]) for _, d in rec if name in d)
        final = parcel_state(st, gz, name, src, i)
        if off == 0.0:
            doc_shot(st, gz, 's6_after_reverse_offset_0cm.png')
        crit = {
            'parcel_bottom': abs(final['bottom_z'] - S['ledge_height']) <= 0.002,
            'tilt': final['tilt_deg'] < 2.0,
            'fork_reached_0': q_end is not None and abs(q_end) < 0.002,
            'no_drag': drag < 0.005,
            'overlap': final['min_overlap_m'] >= 0.01,
        }
        row = {'offset_cm': off * 100, 'robot_pitch_deg': pitch, 'carried_bottom_z': carried['bottom_z'],
               'robot_pitch_after_lower_deg': pitch_low, 'robot_roll_after_lower_deg': roll_low,
               'fork_q_end_mm': (q_end or 0) * 1000, 'placed': placed, 'final': final, 'drag_mm': drag * 1000,
               'reverse_m': dist, 'criteria': crit, 'success': all(crit.values())}
        rows.append(row)
        print(f'  S6 ofset {off * 100:+.0f} cm: başarı={row["success"]} {crit} q={row["fork_q_end_mm"]:.1f} mm '
              f'taban={final["bottom_z"]:.4f} eğim={final["tilt_deg"]:.2f}° sürükleme={drag * 1000:.1f} mm', flush=True)
    nb_disp = {n: float(np.linalg.norm(gz.model(n).p - nb0[n])) * 1000 for n in neighbours}
    place_robot(gz, stage_xy, src['pose'][2])
    gz.set_pose(name, nominal.p, nominal.R)
    gz.wait_sim(1.0)
    with open(os.path.join(st.logs, 'S6_trials.json'), 'w') as f:
        json.dump({'trials': rows, 'neighbour_displacement_mm': nb_disp}, f, indent=2, default=float)
    ok_small = all(r['success'] for r in rows if abs(r['offset_cm']) <= 2.0 + 1e-9)
    # ±2.5 cm yalnız bilgi: çatal dış kenarı tam kanal kenarında (0.085)
    fail_3 = all(not r['success'] for r in rows if abs(r['offset_cm']) > 2.5)
    limit = {}
    for sgn in (1, -1):
        good = [abs(r['offset_cm']) for r in rows if r['success'] and r['offset_cm'] * sgn >= 0]
        limit['+' if sgn > 0 else '-'] = max(good) if good else None
    meas = '; '.join(f'{r["offset_cm"]:+.1f} cm{" (bilgi)" if abs(abs(r["offset_cm"]) - 2.5) < 1e-6 else ""}: '
                     f'{"başarılı" if r["success"] else "başarısız"} '
                     f'(q {r["fork_q_end_mm"]:.1f} mm, taban {r["final"]["bottom_z"] * 1000:.1f} mm, '
                     f'eğim {r["final"]["tilt_deg"]:.2f}°, sürükleme {r["drag_mm"]:.1f} mm, '
                     f'geri {r["reverse_m"]:.2f} m, robot yunuslama {r["robot_pitch_after_lower_deg"]:+.2f}°)'
                     for r in rows)
    meas += (f'. Gözlenen sınır: +{limit["+"]} / -{limit["-"]} cm. Tam girişte robot yunuslama '
             f'{rows[0]["robot_pitch_deg"]:.2f}° (D-46). Komşu koliler tüm denemeler sonunda: '
             f'{", ".join(f"{n} {d:.3f} mm" for n, d in nb_disp.items())}')
    nb_ok = all(d < 1.0 for d in nb_disp.values())
    rep.add(Result('S6', TEST_NAMES['S6'], PASS if ok_small and fail_3 and nb_ok else FAIL, meas,
                   '0–±2 cm: koli bölmede (taban 0.05 ± 2 mm, örtüşme ≥ 1 cm), eğim < 2°, çatal 0\'a ulaşıyor '
                   '(< 2 mm), sürükleme < 5 mm; ±3 cm: başarısızlık bekleniyor; komşu koliler < 1 mm',
                   {'trials': rows, 'limit_cm': limit, 'ok_within_2cm': ok_small, 'fails_at_3cm': fail_3,
                    'neighbour_displacement_mm': nb_disp}))


def s7(st, rep, gz, parcels):
    rg = RobotGeom(st)
    src = station_entry(st, 'source')
    i = 1
    name = f'parcel_source_{i}'
    bay = bay_pose(st, src, i)
    stage_xy = bay.apply([-1.0, 0.0, 0.0])[:2]
    ensure_doc_camera(st, gz, src, i)
    place_robot(gz, stage_xy, src['pose'][2])
    fork_to(gz, rg, 0.0)
    before = parcel_state(st, gz, name, src, i)
    p_before = gz.model(name).p
    xy, yaw = entry_pose(st, rg, src, i, 0.0)
    place_robot(gz, xy, yaw, settle=0.8)
    disturbed = np.linalg.norm(gz.model(name).p - p_before)
    reached, q = fork_to(gz, rg, S6_CARRY_Q)
    gz.wait_sim(1.0)
    rel0 = (gz.carriage().inv() * gz.model(name)).p
    lifted = parcel_state(st, gz, name, src, i)
    dist, _ = reverse(gz, S7_REVERSE)
    rel1 = (gz.carriage().inv() * gz.model(name)).p
    after = parcel_state(st, gz, name, src, i)
    slip = float(np.linalg.norm(rel1 - rel0))
    doc_shot(st, gz, 's7_after_pickup.png')
    fork_top = rg.fork_top0 + (gz.fork() or 0.0)
    on_fork = after['x'] < 0 and abs(after['bottom_z'] - fork_top) < 0.01
    ok = slip < 0.01 and on_fork and reached
    rows = {'before': before, 'disturbed_on_entry_mm': disturbed * 1000, 'fork_reached': reached,
            'lifted': lifted, 'after': after, 'rel0': rel0.tolist(), 'rel1': rel1.tolist(),
            'slip_mm': slip * 1000, 'reverse_m': dist}
    with open(os.path.join(st.logs, 'S7_pick.json'), 'w') as f:
        json.dump(rows, f, indent=2, default=float)
    try:
        gz.remove(DOC_CAM)
        gz.doc_cam = False
    except RuntimeError:
        pass
    rep.add(Result('S7', TEST_NAMES['S7'], PASS if ok else FAIL,
                   f'girişte koli {disturbed * 1000:.2f} mm oynadı; '
                   f'kaldırınca taban {lifted["bottom_z"] * 1000:.1f} mm '
                   f'(çıkıntı {st.params["station"]["ledge_height"] * 1000:.0f} mm); {dist:.3f} m geri çekilme sonrası '
                   f'koli çatalda (bölme x {after["x"]:.3f} m), bağıl kayma {slip * 1000:.2f} mm',
                   'Eklem 0\'da tam giriş → 0.06 → 0.5 m geri: koli çatalda, bağıl kayma < 1 cm', rows))


SIM_TESTS = {'S4': s4, 'S5': s5, 'S6': s6, 'S7': s7}


# =============================================================================
TESTS = {'S1': s1, 'S2': s2}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tests', default=','.join(ALL_TESTS))
    ap.add_argument('--ws', default=os.path.expanduser('~/cv_forklift_ws'))
    ap.add_argument('--docs', default=None)
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--raw-dir', default=None, help='S4/S5 ham karelerini bu dizine kaydet (detektör ayarı için)')
    a = ap.parse_args()
    docs = a.docs or os.path.join(a.ws, 'docs')
    st = Setup(a.ws, docs)
    st.gui = a.gui
    st.raw_dir = a.raw_dir
    if st.raw_dir:
        os.makedirs(st.raw_dir, exist_ok=True)
    rep = Report(docs)
    tests = [t.strip() for t in a.tests.split(',') if t.strip()]
    for t in tests:
        if t in TESTS:
            try:
                TESTS[t](st, rep)
            except Exception as e:  # test altyapısı hatası: ÇALIŞTIRILAMADI
                traceback.print_exc()
                rep.add(Result(t, TEST_NAMES[t], NOT_RUN, f'altyapı hatası: {e}', '—'))
        elif t not in ALL_TESTS:
            print(f'bilinmeyen test: {t}', file=sys.stderr)
    sim = [t for t in tests if t in ALL_TESTS and t not in TESTS]
    if sim:
        run_sim_tests(st, rep, sim)
    rep.write()


if __name__ == '__main__':
    main()
