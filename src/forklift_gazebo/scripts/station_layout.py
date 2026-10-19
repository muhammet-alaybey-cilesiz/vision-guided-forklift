"""station_layout.py — istasyon yerleşimi için ortak hesaplar (tek uygulama).

generate_aruco_textures.py (build), stations.launch.py (spawn) ve
station_acceptance.py (test) bu modülü içe aktarır; bölme konumu, ID şeması,
işaret ve koli pozları yalnızca burada hesaplanır. Ölçüler
config/station_params.yaml'dan, yerleşim config/stations.yaml'dan gelir.

Çerçeveler (DECISIONS D-43):
  station : orijin zeminde, ön kenar ortası; +x istasyonun içine, +y sol, +z yukarı
  bay_i   : orijin zeminde, bölme i kanal ekseninde, ön kenarda; eksenler station ile aynı
  işaret  : OpenCV ArUco işaret çerçevesi; orijin siyah karenin merkezi, x sağa,
            y yukarı (−x yönünden bakan kameraya göre), z işaretten dışarı (−x_station)
"""

import math

import yaml

SEAL_COLORS = ('red', 'green', 'blue')
STATION_COLORS = ('red', 'green', 'blue', 'neutral')
MARKER_TYPES = ('upper', 'lower')

# İşaret çerçevesinin station çerçevesindeki yönelimi (sütunlar: x_m, y_m, z_m):
# x_m = -y_s, y_m = +z_s, z_m = -x_s. Konvansiyon sabiti, ölçü değildir.
MARKER_RPY = (math.pi / 2, 0.0, -math.pi / 2)


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def dict_tag(dictionary):
    """'DICT_4X4_50' → '4x4_50' (doku dosya adı için)."""
    return dictionary.lower().replace('dict_', '')


def texture_name(params, marker_id):
    return f'aruco_{dict_tag(params["markers"]["dictionary"])}_id{marker_id:02d}.png'


def marker_ids(params, k):
    """Bölme numarası k → {'upper': k, 'lower': k + offset} (DECISIONS D-45)."""
    return {'upper': k, 'lower': k + params['markers']['lower_id_offset']}


def all_marker_ids(params):
    m = params['markers']
    return list(range(m['max_bay_id'] + 1)) + \
        list(range(m['lower_id_offset'], m['lower_id_offset'] + m['max_bay_id'] + 1))


def bay_y(params, n_bays, i):
    """y_i = (i - (N-1)/2) · adım; bölme 0 en -y tarafta."""
    return (i - (n_bays - 1) / 2.0) * params['station']['bay_pitch']


def marker_face_x(params):
    """İşaret kutusunun ön (dokulu, -x) yüzünün station x'i."""
    s, m = params['station'], params['markers']
    return s['ledge_depth'] - m['standoff'] - m['thickness']


def marker_texture_size(params, kind):
    """Dokunun kenar uzunluğu (siyah kare + beyaz kenar) [m]."""
    m = params['markers']
    return m[kind]['size'] * (6 + 2 * m['quiet_zone_cells']) / 6.0


def parcel_center_in_bay(params):
    """Bölmede nominal koli merkezi (bay çerçevesi, y = 0)."""
    s, p = params['station'], params['parcel']['size']
    return (s['parcel_front_gap'] + p['x'] / 2.0, 0.0, s['ledge_height'] + p['z'] / 2.0)


def compose(pose, x, y, z=0.0):
    """Station pozu [x, y, yaw] ∘ yerel nokta → dünya (x, y, z)."""
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (px + c * x - s * y, py + s * x + c * y, z)


def station_xacro_args(st, params_file):
    """stations.yaml kaydı → station.sdf.xacro argümanları."""
    return [f'name:={st["name"]}', f'bays:={st["bays"]}', f'color:={st["color"]}',
            'bay_ids:=' + ','.join(str(k) for k in st['bay_ids']), f'params_file:={params_file}']


def parcel_xacro_args(name, color, params_file):
    return [f'name:={name}', f'color:={color}', f'params_file:={params_file}']


def validate_stations(params, layout):
    """stations.yaml tutarlılık denetimi; hata varsa ValueError."""
    errors = []
    max_k = params['markers']['max_bay_id']
    seen_k, seen_names = {}, set()
    for st in layout.get('stations', []):
        name = st.get('name')
        if not name or name in seen_names:
            errors.append(f'istasyon adı boş veya tekrar: {name!r}')
        seen_names.add(name)
        if st.get('color') not in STATION_COLORS:
            errors.append(f'{name}: renk {st.get("color")!r} ∉ {STATION_COLORS}')
        if len(st.get('pose', [])) != 3:
            errors.append(f'{name}: pose [x, y, yaw] olmalı')
        n = st.get('bays')
        if not isinstance(n, int) or n < 1:
            errors.append(f'{name}: bays pozitif tamsayı olmalı')
            continue
        ids, occ = st.get('bay_ids', []), st.get('occupancy', [None] * n)
        if len(ids) != n or len(occ) != n:
            errors.append(f'{name}: bay_ids ({len(ids)}) ve occupancy ({len(occ)}) uzunluğu {n} olmalı')
        for k in ids:
            if not isinstance(k, int) or not 0 <= k <= max_k:
                errors.append(f'{name}: bölme numarası {k!r} ∉ [0, {max_k}]')
            elif k in seen_k:
                errors.append(f'{name}: bölme numarası {k} {seen_k[k]} istasyonunda da var')
            else:
                seen_k[k] = name
        for c in occ:
            if c is not None and c not in SEAL_COLORS:
                errors.append(f'{name}: koli rengi {c!r} ∉ {SEAL_COLORS} veya null')
    if errors:
        raise ValueError('stations.yaml geçersiz:\n  ' + '\n  '.join(errors))


def _r(v):
    """Kayan nokta artıklarını (0.30000000000000004) atar; 1 µm'nin çok altında."""
    return [round(c, 9) for c in v]


def markers(params, layout):
    """Her işaret için kayıt: id, istasyon, bölme, tip, boyut, station ve dünya pozu."""
    out = []
    face_x = marker_face_x(params)
    for st in layout['stations']:
        for i, k in enumerate(st['bay_ids']):
            y = bay_y(params, st['bays'], i)
            for kind, mid in marker_ids(params, k).items():
                z = params['markers'][kind]['center_z']
                wx, wy, wz = compose(st['pose'], face_x, y, z)
                out.append({
                    'id': mid, 'station': st['name'], 'bay': i, 'bay_id': k, 'type': kind,
                    'size': params['markers'][kind]['size'],
                    'pose_station': {'xyz': _r([face_x, y, z]), 'rpy': _r(MARKER_RPY)},
                    'pose_world': {'xyz': _r([wx, wy, wz]),
                                   'rpy': _r([MARKER_RPY[0], MARKER_RPY[1],
                                              MARKER_RPY[2] + st['pose'][2]])},
                })
    return sorted(out, key=lambda m: m['id'])


def parcels(params, layout):
    """Başlangıç kolileri: (model adı, renk, dünya pozu [x, y, z, yaw])."""
    cx, _, cz = parcel_center_in_bay(params)
    out = []
    for st in layout['stations']:
        for i, color in enumerate(st.get('occupancy') or []):
            if color is None:
                continue
            x, y, _ = compose(st['pose'], cx, bay_y(params, st['bays'], i))
            out.append((f'parcel_{st["name"]}_{i}', color, [x, y, cz, st['pose'][2]]))
    return out
