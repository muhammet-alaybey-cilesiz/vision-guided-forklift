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
"""

import argparse
import os
import sys
import time

import cv2
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
    }
    os.makedirs(os.path.dirname(os.path.abspath(a.markers)), exist_ok=True)
    with open(a.markers, 'w') as f:
        f.write('# ÜRETİLDİ (generate_aruco_textures.py) — elle düzenlemeyin.\n'
                '# Kaynak: station_params.yaml + stations.yaml; stations.yaml değişince yeniden build edin.\n')
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=None)
    print(f'{len(ids)} doku → {a.textures}; {len(doc["markers"])} işaret → {a.markers}')


if __name__ == '__main__':
    main()
