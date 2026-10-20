#!/usr/bin/env python3
"""generate_aruco_textures.py — ArUco doku PNG'leri ve markers.yaml üretir (build adımı).

CMake her build'de çalıştırır; çıktılar build dizinine yazılır ve paketin
share dizinine kurulur. Kaynak ağaca PNG yazılmaz (.gitignore).

  --params   config/station_params.yaml
  --stations config/stations.yaml
  --textures <dizin>  → <dizin>/aruco_<sözlük>_idNN.png  (şemadaki tüm ID'ler)
  --markers  <dosya>  → markers.yaml (stations.yaml'daki her işaret)

Doku: siyah 4×4 işaret (1 hücrelik siyah çerçeve dahil 6 hücre) + quiet_zone_cells
hücrelik beyaz kenar; hücre başına texture_px_per_cell piksel, en yakın komşu
(keskin kenar). OpenCV 4.6 API'si: Dictionary_get / drawMarker (DECISIONS D-47).
Her PNG üretildikten sonra detectMarkers ile geri okunur; ID tutmazsa build durur.

Hizalama hedefi (D-55): iki desen de üretilir, station xacro markers.align.pattern'i kullanır.
  align_checker.png  — beyaz kare, ortasında 2×2 dama (sol üst ve sağ alt siyah);
                       ölçülen nokta merkez köşe, öz denetim cornerSubPix ile
  align_bullseye.png — beyaz kare, siyah halka + ortada siyah dolu daire; ölçülen
                       nokta elips merkezi, öz denetim fitEllipse ile
Öz denetim: nokta doku merkezinin ±0.5 px içinde olmalı.
markers.yaml'a her hedefin X-köşe konumu 'align_targets' olarak yazılır.
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import station_layout as sl  # noqa: E402


def make_texture(dictionary, marker_id, px_cell, quiet_cells):
    # drawMarker: siyah çerçeve dahil (4 + 2) hücre; beyaz kenarı biz ekliyoruz
    img = cv2.aruco.drawMarker(dictionary, marker_id, 6 * px_cell, borderBits=1)
    q = quiet_cells * px_cell
    return cv2.copyMakeBorder(img, q, q, q, q, cv2.BORDER_CONSTANT, value=255)


def self_check(dictionary, img, marker_id):
    corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary,
                                              parameters=cv2.aruco.DetectorParameters_create())
    return ids is not None and ids.ravel().tolist() == [marker_id]


def make_align_texture(a):
    """Beyaz kare + ortada 2×2 dama deseni; kenar texture_px, desen oranı checker / size."""
    n = a['texture_px']
    c = int(round(n * a['checker'] / a['size'] / 2.0)) * 2      # desen kenarı (çift piksel)
    img = np.full((n, n), 255, np.uint8)
    o, h = (n - c) // 2, c // 2
    img[o:o + h, o:o + h] = 0                 # sol üst
    img[o + h:o + c, o + h:o + c] = 0         # sağ alt
    return img


def make_bullseye_texture(a):
    """Beyaz kare + siyah halka + siyah dolu daire (çaplar size'a oranla); kenar texture_px."""
    n, b = a['texture_px'], a['bullseye']
    k = n / a['size']
    yy, xx = np.mgrid[0:n, 0:n]
    r = np.hypot(xx - (n / 2.0 - 0.5), yy - (n / 2.0 - 0.5))
    img = np.full((n, n), 255, np.uint8)
    img[(r <= b['ring_outer'] * k / 2) & (r >= b['ring_inner'] * k / 2)] = 0
    img[r <= b['dot'] * k / 2] = 0
    return img


def bullseye_self_check(img):
    """Ortadaki dolu dairenin elips merkezi doku merkezinde mi (±0.5 px)."""
    n = img.shape[0]
    _, inv = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    cnts, _ = cv2.findContours(inv, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    dots = [c for c in cnts if cv2.contourArea(c) > 0 and
            np.hypot(*(np.mean(c.reshape(-1, 2), 0) - (n / 2.0 - 0.5))) < 3 and
            cv2.contourArea(c) < 0.25 * n * n]
    if not dots:
        return False
    (cx, cy), _, _ = cv2.fitEllipse(min(dots, key=cv2.contourArea))
    return max(abs(cx - (n / 2.0 - 0.5)), abs(cy - (n / 2.0 - 0.5))) < 0.5


def align_self_check(img):
    """Merkez köşe cornerSubPix ile doku merkezinde mi (±0.5 px)."""
    n = img.shape[0]
    blur = cv2.GaussianBlur(img, (5, 5), 1.0)
    p = np.array([[[n / 2.0 + 2.0, n / 2.0 - 2.0]]], np.float32)    # kasıtlı başlangıç sapması
    cv2.cornerSubPix(blur, p, (7, 7), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3))
    # piksel merkezleri kuralı: köşe (n/2 - 0.5) konumunda
    return float(np.max(np.abs(p.ravel() - (n / 2.0 - 0.5)))) < 0.5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--params', required=True)
    ap.add_argument('--stations', required=True)
    ap.add_argument('--textures', required=True)
    ap.add_argument('--markers', required=True)
    a = ap.parse_args()

    params, layout = sl.load_yaml(a.params), sl.load_yaml(a.stations)
    sl.validate_stations(params, layout)
    m = params['markers']
    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, m['dictionary']))
    # sözlükte yeterli ID var mı (DICT_4X4_50 → 50)
    n_dict = dictionary.bytesList.shape[0]
    ids = sl.all_marker_ids(params)
    if max(ids) >= n_dict:
        sys.exit(f'ID şeması {max(ids)} gerektiriyor, {m["dictionary"]} yalnız {n_dict} ID içeriyor')

    os.makedirs(a.textures, exist_ok=True)
    for mid in ids:
        img = make_texture(dictionary, mid, m['texture_px_per_cell'], m['quiet_zone_cells'])
        if not self_check(dictionary, img, mid):
            sys.exit(f'doku öz denetimi başarısız: ID {mid}')
        cv2.imwrite(os.path.join(a.textures, sl.texture_name(params, mid)), img)

    sl.validate_align(params)
    for pattern, make, check in (('checker', make_align_texture, align_self_check),
                                 ('bullseye', make_bullseye_texture, bullseye_self_check)):
        img = make(m['align'])
        if not check(img):
            sys.exit(f'hizalama hedefi öz denetimi başarısız ({pattern}): nokta doku merkezinde değil')
        cv2.imwrite(os.path.join(a.textures, sl.align_texture(params, pattern)), img)

    doc = {
        'generated': time.strftime('%Y-%m-%d %H:%M:%S'),
        'source': ['config/station_params.yaml', 'config/stations.yaml'],
        'dictionary': m['dictionary'],
        'frames': {
            'station': 'origin on the floor at the middle of the front edge; +x into the station, +y left, +z up',
            'marker': 'OpenCV ArUco marker frame: origin at the black square centre, x right, y up '
                      '(seen from -x_station), z out of the marker (-x_station)',
        },
        'stations': {st['name']: {'pose': st['pose'], 'color': st['color'], 'bays': st['bays'],
                                  'bay_ids': st['bay_ids']} for st in layout['stations']},
        'markers': sl.markers(params, layout),
        'align_frame': 'point = centre corner of the 2x2 checker on the ledge front face (station '
                       'x = face); side right = -y of the channel, left = +y (DECISIONS D-55)',
        'align_targets': sl.align_targets(params, layout),
    }
    os.makedirs(os.path.dirname(os.path.abspath(a.markers)), exist_ok=True)
    with open(a.markers, 'w') as f:
        f.write('# ÜRETİLDİ (generate_aruco_textures.py) — elle düzenlemeyin.\n'
                '# Kaynak: station_params.yaml + stations.yaml; stations.yaml değişince yeniden build edin.\n')
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=None)
    print(f'{len(ids) + 2} doku → {a.textures}; {len(doc["markers"])} işaret, '
          f'{len(doc["align_targets"])} hizalama hedefi → {a.markers}')


if __name__ == '__main__':
    main()
