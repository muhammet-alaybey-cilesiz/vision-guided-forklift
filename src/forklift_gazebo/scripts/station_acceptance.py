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
        t0 = self.now()
        w0 = time.time()
        limit = timeout if timeout is not None else 10 * dt + 10
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

    def _req(self, service, req, req_t, rep_t, timeout_ms=3000):
        ok, rep = self.node.request(service, req, req_t, rep_t, timeout_ms)
        if not ok or (hasattr(rep, 'data') and not rep.data):
            raise RuntimeError(f'{service} başarısız')
        return rep

    def set_pose(self, name, p, R):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose as PoseMsg
        m = PoseMsg()
        m.name = name
        m.position.x, m.position.y, m.position.z = (float(v) for v in p)
        q = R_to_quat(np.asarray(R))
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = (float(v) for v in q)
        self._req(f'/world/{self.w}/set_pose', m, PoseMsg, Boolean)

    def spawn(self, name, sdf, p, R):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        m = EntityFactory()
        m.sdf, m.name, m.allow_renaming = sdf, name, False
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        q = R_to_quat(np.asarray(R))
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = \
            (float(v) for v in q)
        self._req(f'/world/{self.w}/create', m, EntityFactory, Boolean)

    def remove(self, name):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_pb2 import Entity
        m = Entity()
        m.name, m.type = name, Entity.MODEL
        self._req(f'/world/{self.w}/remove', m, Entity, Boolean)

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


# =============================================================================
#  S3 — statik kararlılık: tam dünya, 10 s, kolilerin spawn pozundan sapması
# =============================================================================
S3_DURATION = 10.0
S3_TOL = 0.001


def s3(st, rep, gz, parcels):
    nominal = {n: np.array([x, y, z]) for n, _, (x, y, z, yaw) in parcels}
    nominal_yaw = {n: yaw for n, _, (x, y, z, yaw) in parcels}
    gz.record(True)
    t0 = gz.now()
    gz.wait_sim(S3_DURATION)
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
    """S3–S8: tek oturum; S3 spawn anından ölçmek için ilk sırada."""
    parcels = sl.parcels(st.params, st.layout)
    order = [t for t in ('S3', 'S4', 'S5', 'S6', 'S7', 'S8') if t in tests]
    done = set()
    with SimSession(st, 'stations') as sess:
        gz = None
        try:
            gz = Gz(st)
            wait_world_ready(st, gz, sess, parcels)
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


SIM_TESTS = {}


# =============================================================================
TESTS = {'S1': s1, 'S2': s2}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tests', default=','.join(ALL_TESTS))
    ap.add_argument('--ws', default=os.path.expanduser('~/cv_forklift_ws'))
    ap.add_argument('--docs', default=None)
    ap.add_argument('--gui', action='store_true')
    a = ap.parse_args()
    docs = a.docs or os.path.join(a.ws, 'docs')
    st = Setup(a.ws, docs)
    st.gui = a.gui
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
