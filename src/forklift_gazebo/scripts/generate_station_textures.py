#!/usr/bin/env python3
"""generate_station_textures.py — hizalama hedefi dokuları ve markers.yaml üretir (build adımı).

CMake her build'de çalıştırır; çıktılar build dizinine yazılır ve paketin
share dizinine kurulur (model://station_targets/materials/textures). Kaynak
ağaca PNG yazılmaz (D-47).
  --params   config/station_params.yaml
  --stations config/stations.yaml
  --textures <dizin>  → align_bullseye.png, align_checker.png
  --markers  <dosya>  → markers.yaml: istasyonlar, hizalama hedefleri (D-55) ve
                        bölme şeritleri (D-56); ground truth, robota verilmez

Hizalama hedefi: iki desen de üretilir, station xacro markers.align.pattern'i kullanır.
  align_bullseye.png — beyaz kare, siyah halka + ortada siyah dolu daire; ölçülen
                       nokta desen merkezi, öz denetim fitEllipse ile
  align_checker.png  — beyaz kare, ortasında 2×2 dama (sol üst ve sağ alt siyah);
                       ölçülen nokta merkez köşe, öz denetim cornerSubPix ile
Öz denetim: nokta doku merkezinin ±0.5 px içinde olmalı; tutmazsa build durur.
ArUco işaretleri D-56 ile kaldırıldı.
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
    sl.validate_align(params)
    m = params['markers']
    os.makedirs(a.textures, exist_ok=True)
    for pattern, make, check in (('checker', make_align_texture, align_self_check),
                                 ('bullseye', make_bullseye_texture, bullseye_self_check)):
        img = make(m['align'])
        if not check(img):
            sys.exit(f'hizalama hedefi öz denetimi başarısız ({pattern}): nokta doku merkezinde değil')
        cv2.imwrite(os.path.join(a.textures, sl.align_texture(params, pattern)), img)

    doc = {
        'generated': time.strftime('%Y-%m-%d %H:%M:%S'),
        'source': ['config/station_params.yaml', 'config/stations.yaml'],
        'note': 'ground truth for evaluation only; the robot must not read it (D-56)',
        'frames': {
            'station': 'origin on the floor at the middle of the front edge; +x into the station, +y left, +z up',
        },
        'stations': {st['name']: {'pose': st['pose'], 'color': st['color'], 'bays': st['bays'],
                                  'bay_ids': st['bay_ids']} for st in layout['stations']},
        'align_pattern': m['align']['pattern'],
        'align_frame': 'point = pattern centre on the ledge front face (station x = face); '
                       'side right = -y of the channel, left = +y (DECISIONS D-55)',
        'align_targets': sl.align_targets(params, layout),
        'bay_lines': sl.bay_lines(params, layout),
    }
    os.makedirs(os.path.dirname(os.path.abspath(a.markers)), exist_ok=True)
    with open(a.markers, 'w') as f:
        f.write('# ÜRETİLDİ (generate_station_textures.py) — elle düzenlemeyin.\n'
                '# Kaynak: station_params.yaml + stations.yaml; stations.yaml değişince yeniden build edin.\n'
                '# Yalnız değerlendirme (ground truth) içindir; robot bu dosyayı okumaz (D-56).\n')
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=None)
    print(f'2 hizalama dokusu → {a.textures}; {len(doc["align_targets"])} hedef, '
          f'{len(doc["bay_lines"])} bölme şeridi → {a.markers}')


if __name__ == '__main__':
    main()
