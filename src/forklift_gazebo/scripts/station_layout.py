"""station_layout.py — istasyon yerleşimi için ortak hesaplar (tek uygulama).

generate_station_textures.py (build), stations.launch.py (spawn),
generate_factory_world.py ve station_acceptance.py (test) bu modülü içe
aktarır; bölme konumu, hizalama hedefi, bölme şeridi ve koli pozları yalnızca
burada hesaplanır. Ölçüler
config/station_params.yaml'dan, yerleşim config/stations.yaml'dan gelir.

Çerçeveler (DECISIONS D-43):
  station : orijin zeminde, ön kenar ortası; +x istasyonun içine, +y sol, +z yukarı
  bay_i   : orijin zeminde, bölme i kanal ekseninde, ön kenarda; eksenler station ile aynı
  hizalama hedefi : bölme kanalının iki yanında, çıkıntı ön yüzünde (D-55); ölçülen
            nokta desen merkezi (bullseye: koyuluk ağırlık merkezi, checker: X-köşe)
  bölme şeridi : zemin plakası üstünde bölme ekseni boyunca (D-56)
ArUco işaretleri D-56 ile kaldırıldı.
"""

import math

import yaml

SEAL_COLORS = ('red', 'green', 'blue')
STATION_COLORS = ('red', 'green', 'blue', 'neutral')
ALIGN_SIDES = ('right', 'left')          # right: kanalın -y yanı, left: +y yanı
ALIGN_PATTERNS = ('checker', 'bullseye')


def align_texture(params, pattern=None):
    """Hizalama hedefi doku adı (aruco_markers/materials/textures altında)."""
    return f'align_{pattern or params["markers"]["align"]["pattern"]}.png'



def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)






def bay_y(params, n_bays, i):
    """y_i = (i - (N-1)/2) · adım; bölme 0 en -y tarafta."""
    return (i - (n_bays - 1) / 2.0) * params['station']['bay_pitch']




def align_face_x(params):
    """Hizalama hedefi kutusunun ön (dokulu, -x) yüzünün station x'i (çıkıntı ön yüzü x = 0)."""
    m = params['markers']
    return -(m['standoff'] + m['thickness'])


def align_offset(params):
    """Hedef merkezinin kanal eksenine uzaklığı: kanal yarı genişliği + pay + plaka yarısı."""
    s, a = params['station'], params['markers']['align']
    return s['channel_width'] / 2.0 + a['edge_gap'] + a['size'] / 2.0


def align_points(params, n_bays, i):
    """Bölme i'nin iki hedefinin X-köşe noktaları (station çerçevesi): {'right': xyz, 'left': xyz}."""
    y, d = bay_y(params, n_bays, i), align_offset(params)
    x, z = align_face_x(params), params['station']['ledge_height'] / 2.0
    return {'right': (x, y - d, z), 'left': (x, y + d, z)}


def validate_align(params):
    """Hedefler çıkıntılara sığıyor mu (ön yüz yüksekliği, uç ve ortak çıkıntı genişliği)."""
    s, a = params['station'], params['markers']['align']
    gap = s['bay_pitch'] - s['channel_width']             # iki kanal arası çıkıntı genişliği
    need = a['edge_gap'] + a['size']                      # kanal kenarından plakanın dış kenarına
    errors = []
    if a['size'] > s['ledge_height']:
        errors.append(f'hedef {a["size"]} m > çıkıntı yüksekliği {s["ledge_height"]} m')
    if 2 * need > gap:
        errors.append(f'ortak çıkıntıya iki hedef sığmıyor: {2 * need:.3f} > {gap:.3f} m')
    if need > gap / 2.0:
        errors.append(f'uç çıkıntıya hedef sığmıyor: {need:.3f} > {gap / 2.0:.3f} m')
    if a['pattern'] not in ALIGN_PATTERNS:
        errors.append(f'pattern {a["pattern"]!r} ∉ {ALIGN_PATTERNS}')
    if not 0 < a['checker'] < a['size']:
        errors.append('dama deseni plakadan küçük ve pozitif olmalı')
    b = a['bullseye']
    if not 0 < b['dot'] < b['ring_inner'] < b['ring_outer'] < a['size']:
        errors.append('bullseye: 0 < dot < ring_inner < ring_outer < size olmalı')
    if errors:
        raise ValueError('markers.align geçersiz:\n  ' + '\n  '.join(errors))


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
    return [round(c, 9) + 0.0 for c in v]



def bay_line(params, n_bays, i):
    """Bölme i şeridinin başlangıç ve bitişi (station çerçevesi, zemin plakası üst yüzü):
    önlük bandının iç kenarından kanal girişine (x = 0), bölme ekseninde."""
    s, v = params['station'], params['appearance']
    y = bay_y(params, n_bays, i)
    z = s['base_plate_thickness'] + v['band_height']
    return (-s['apron_depth'] + v['band_width'], y, z), (0.0, y, z)


def bay_lines(params, layout):
    """Her bölme şeridi: istasyon, bölme, başlangıç/bitiş (station ve dünya), genişlik, renk adı."""
    out = []
    for st in layout['stations']:
        for i, k in enumerate(st['bay_ids']):
            a, b = bay_line(params, st['bays'], i)
            out.append({'station': st['name'], 'bay': i, 'bay_id': k,
                        'color': 'yellow' if st['color'] == 'neutral' else st['color'],
                        'start_station': _r(a), 'end_station': _r(b),
                        'start_world': _r(compose(st['pose'], *a)), 'end_world': _r(compose(st['pose'], *b)),
                        'width': params['appearance']['bay_line']['width']})
    return out


def align_targets(params, layout):
    """Her hizalama hedefi: istasyon, bölme, taraf, X-köşenin station ve dünya konumu."""
    out = []
    for st in layout['stations']:
        for i, k in enumerate(st['bay_ids']):
            for side, p in align_points(params, st['bays'], i).items():
                wx, wy, wz = compose(st['pose'], *p)
                out.append({'station': st['name'], 'bay': i, 'bay_id': k, 'side': side,
                            'point_station': _r(p), 'point_world': _r([wx, wy, wz]),
                            'normal_yaw_world': round(st['pose'][2] + math.pi, 9)})
    return out


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
