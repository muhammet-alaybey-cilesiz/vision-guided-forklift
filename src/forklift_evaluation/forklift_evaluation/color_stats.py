"""
color_stats.py — sınıf başına HSV dağılımı (renk eşiklerinin veriden kalibrasyonu, plan R7)

Kayıtlı kümede GT bölgeleri (mühür, koli yüzü, tabela bloğu ve zemini, bölme ve
zemin şeritleri, durma çizgisi, bölme etiketi, hedef plakası) kamera modeliyle
görüntüye izdüşürülür, kenarlardan 2 px içeride kalan piksellerin H, S, V
yüzdelikleri çıkarılır. Gerçek kamerada aynı iş elle etiketlenmiş karelerle yapılır.

  ros2 run forklift_evaluation color_stats --data data/f1 --calib <dizin> --out docs/logs/cv/f2/color_stats.json
Kırmızı tonlar çemberin iki ucunda olduğu için H, (H + 90) mod 180 olarak da verilir.
"""

import argparse
import json
import os
from collections import defaultdict

import cv2
import numpy as np

from forklift_evaluation import overlay as ov
from forklift_evaluation.dataset import Dataset
from forklift_evaluation.world_gt import EvalSetup, WorldGT

PCT = (1, 5, 25, 50, 75, 95, 99)
MAX_PX = 20000                       # sınıf başına kare başına en çok piksel (bellek)


def regions(gt, fr):
    """(sınıf, kamera listesi, dünya poligonu, dışarıda bırakılacak poligonlar)."""
    out = []
    for p in gt.parcel_boxes_from(fr.parcels):
        out.append((f'seal_{p["color"]}', ('front_camera', 'lane_camera'), p['seal'], []))
        out.append(('parcel_kraft', ('front_camera', 'lane_camera'), p['face'], [_grow(p['seal'], 1.4)]))
    for s in gt.signs():
        out.append((f'sign_block_{s["color"]}', ('front_camera',), s['block'], []))
        out.append(('sign_white', ('front_camera',), s['quad'], [s['block']]))
    for b in gt.bay_lines():
        out.append((f'bay_line_{b["color"]}', ('front_camera', 'lane_camera'), b['poly'], []))
    for ln in gt.floor_lanes():
        out.append((f'floor_lane_{ln["color"]}', ('front_camera', 'lane_camera'), ln['poly'], []))
        out.append(('stop_bar_white', ('front_camera', 'lane_camera'), ln['stop_bar'], []))
    for lb in gt.bay_labels():
        out.append(('bay_label_yellow', ('front_camera', 'lane_camera'), lb['quad'], []))
    for t in gt.align_targets():
        out.append(('target_plate', ('front_camera', 'lane_camera'), t['plate'], []))
    return out


def _grow(poly, k):
    c = poly.mean(axis=0)
    return c + (poly - c) * k


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', default=os.path.expanduser('~/cv_forklift_ws/data/f1'))
    ap.add_argument('--calib', default='')
    ap.add_argument('--out', default=os.path.expanduser('~/cv_forklift_ws/docs/logs/cv/f2/color_stats.json'))
    ap.add_argument('--every', type=int, default=3, help='her n. kare')
    a = ap.parse_args()
    result = {}
    for world in ('station_test', 'factory'):
        root = os.path.join(a.data, world)
        if not os.path.isdir(root):
            continue
        ds = Dataset(root, a.calib or None)
        gt = WorldGT(EvalSetup(world, os.path.join(os.path.dirname(a.out), 'tmp')))
        acc = defaultdict(list)
        for k, fr in enumerate(ds.frames()):
            if k % a.every:
                continue
            imgs = {}
            for cls, cams, poly, excl in regions(gt, fr):
                for c in cams:
                    cam = ds.cams[c]
                    uv, front, inside = cam.project(fr.to_base(poly))
                    if not inside.all():
                        continue
                    m = ov.poly_mask((cam.height, cam.width), uv, erode=2)
                    for e in excl:
                        ue, fe, _ = cam.project(fr.to_base(e))
                        if fe.all():
                            m &= ~ov.poly_mask((cam.height, cam.width), ue)
                    if m.sum() < 12:
                        continue
                    if c not in imgs:
                        imgs[c] = cv2.cvtColor(fr.rgb(c), cv2.COLOR_RGB2HSV)
                    px = imgs[c][m]
                    if len(px) > MAX_PX:
                        px = px[np.random.default_rng(0).choice(len(px), MAX_PX, replace=False)]
                    acc[(cls, c)].append(px)
        for (cls, c), lst in sorted(acc.items()):
            px = np.vstack(lst).astype(float)
            h = px[:, 0]
            result.setdefault(world, {}).setdefault(cls, {})[c] = {
                'pixels': int(len(px)), 'regions': len(lst),
                'H': np.percentile(h, PCT).round(1).tolist(),
                'H_shift90': np.percentile((h + 90) % 180, PCT).round(1).tolist(),
                'S': np.percentile(px[:, 1], PCT).round(1).tolist(),
                'V': np.percentile(px[:, 2], PCT).round(1).tolist(),
            }
        print(f'[color_stats] {world}: {len(acc)} sınıf/kamera', flush=True)
    result['percentiles'] = list(PCT)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(result, f, indent=1)
    for world in ('station_test', 'factory'):
        for cls, d in sorted(result.get(world, {}).items()):
            for c, v in d.items():
                print(f'{world:12s} {cls:22s} {c[:5]} n={v["pixels"]:7d}  H5-95 {v["H"][1]:5.1f}-{v["H"][5]:5.1f} '
                      f'(+90: {v["H_shift90"][1]:5.1f}-{v["H_shift90"][5]:5.1f})  '
                      f'S5-95 {v["S"][1]:5.1f}-{v["S"][5]:5.1f}  '
                      f'V5-95 {v["V"][1]:5.1f}-{v["V"][5]:5.1f}')


if __name__ == '__main__':
    main()
