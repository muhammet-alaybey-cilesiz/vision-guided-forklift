#!/usr/bin/env python3
"""Robot gövdesi için dokulu çıkartma (decal) mesh'lerini üretir (build sırasında).

Her çıkartma tek bir dörtgendir (OBJ + MTL + PNG). Yerel çerçeve: dörtgen y-z
düzleminde, merkez orijinde, normal +x; +x'ten bakan izleyici için u ekseni +y
(sağ), v ekseni +z (yukarı). URDF'te rpy ile yerleştirilir (rpy yaw π → normal
-x, yaw ±π/2 → normal ±y). Tek yüzlüdür: yalnız normal tarafından görünür.
Ölçüler forklift_params.yaml → appearance.decals'tan; yazılar sabit.

Çıktı (--out): rear_hazard.{obj,mtl,png}, side_label.{obj,mtl,png}

Kullanım: generate_decals.py --params forklift_params.yaml --out DIR
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

FONTS = ['/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
         '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf']
YELLOW = (242, 178, 12)
BLACK = (22, 22, 24)
GRAPHITE = (46, 48, 54)
WHITE = (245, 245, 242)
AMBER = (255, 160, 20)


def font(size):
    for p in FONTS:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default(size)


def write_quad(out, name, w, h):
    """Merkezde, y-z düzleminde, normal +x olan w × h dörtgen (iki üçgen, CCW)."""
    with open(os.path.join(out, name + '.mtl'), 'w') as f:
        f.write(f'newmtl {name}\nKa 1 1 1\nKd 1 1 1\nKs 0.05 0.05 0.05\nNs 10\nd 1\nillum 2\nmap_Kd {name}.png\n')
    a, b = w / 2.0, h / 2.0
    with open(os.path.join(out, name + '.obj'), 'w') as f:
        f.write(f'# {name}: üretilmiş (generate_decals.py)\nmtllib {name}.mtl\no {name}\n')
        for y, z in ((-a, -b), (a, -b), (a, b), (-a, b)):
            f.write(f'v 0 {y:.6f} {z:.6f}\n')
        f.write('vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 1 0 0\n')
        f.write(f'usemtl {name}\ns off\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n')


def hazard(w_px, h_px):
    period = h_px                               # 45° şerit; genişlik periyodun yarısı
    y, x = np.mgrid[0:h_px, 0:w_px]
    stripe = ((x + y) % period) < period // 2
    arr = np.where(stripe[..., None], np.array(YELLOW, np.uint8), np.array(BLACK, np.uint8))
    return Image.fromarray(arr.astype(np.uint8))


def side_label(w_px, h_px, code, text):
    im = Image.new('RGB', (w_px, h_px), GRAPHITE)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, w_px * 0.035, h_px], fill=AMBER)          # sol kenarda turuncu şerit
    d.text((w_px * 0.08, h_px * 0.06), code, font=font(int(h_px * 0.52)), fill=WHITE)
    d.text((w_px * 0.085, h_px * 0.66), text, font=font(int(h_px * 0.2)), fill=(200, 200, 196))
    return im


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--params', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    D = yaml.safe_load(open(a.params))['appearance']['decals']
    os.makedirs(a.out, exist_ok=True)

    w, h = D['rear']['size']
    write_quad(a.out, 'rear_hazard', w, h)
    hazard(512, int(round(512 * h / w))).save(os.path.join(a.out, 'rear_hazard.png'))

    w, h = D['side']['size']
    write_quad(a.out, 'side_label', w, h)
    side_label(1024, int(round(1024 * h / w)), D['side']['code'], D['side']['text']).save(
        os.path.join(a.out, 'side_label.png'))
    print(f'çıkartmalar → {a.out}')


if __name__ == '__main__':
    main()
