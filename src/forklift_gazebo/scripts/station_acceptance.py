#!/usr/bin/env python3
"""
station_acceptance.py — yük istasyonu ve koli kabul testleri (S1–S8)

Kullanım
  ros2 run forklift_gazebo station_acceptance.py                # tüm testler
  ros2 run forklift_gazebo station_acceptance.py --tests S1,S2  # seçili testler
Seçenekler
  --docs DIR   çıktı dizini (varsayılan: <ws>/docs)
  --ws DIR     workspace kökü (varsayılan: ~/cv_forklift_ws)

S1 ve S2 simülasyon gerektirmez; kurulu paketteki xacro ve parametre dosyalarını
kullanır (önce colcon build). Çıktı: her test için ölçülen değer, kriter ve
GEÇTİ / KALDI / ÇALIŞTIRILAMADI; docs/station_test_results.{json,md},
docs/logs/stations/*.
"""

import argparse
import json
import os
import subprocess
import sys
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
TESTS = {'S1': s1, 'S2': s2}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tests', default=','.join(ALL_TESTS))
    ap.add_argument('--ws', default=os.path.expanduser('~/cv_forklift_ws'))
    ap.add_argument('--docs', default=None)
    a = ap.parse_args()
    docs = a.docs or os.path.join(a.ws, 'docs')
    st = Setup(a.ws, docs)
    rep = Report(docs)
    for t in [t.strip() for t in a.tests.split(',') if t.strip()]:
        fn = TESTS.get(t)
        if fn is None:
            rep.add(Result(t, TEST_NAMES.get(t, t), NOT_RUN, '—', '—', {'reason': 'bu sürümde uygulanmadı'}))
            continue
        try:
            fn(st, rep)
        except Exception as e:  # test altyapısı hatası: ÇALIŞTIRILAMADI
            traceback.print_exc()
            rep.add(Result(t, TEST_NAMES[t], NOT_RUN, f'altyapı hatası: {e}', '—'))
    rep.write()


if __name__ == '__main__':
    main()
