#!/usr/bin/env python3
"""Fabrika görünümü için tabela, etiket ve zemin işareti dokularını üretir (build sırasında).

Çıktı (--out, model://factory_signs/materials/textures):
  sign_<renk>.png      istasyon tabelası, renk ∈ red | green | blue | neutral (1024 × 256)
  bay_<kk>.png         bölme numarası etiketi, kk = 00 … max_bay_id (128 × 128)
  parcel_label.png     koli gönderi etiketi (512 × 384)
  hazard_tape.png      sarı-siyah 45° şerit, yatayda kesintisiz tekrarlanır (512 × 64)
  hazard_panel.png     kolon koruyucu: sarı-siyah 45° şerit, hafif aşınmış (512 × 1024)
  sign_amr_zone.png    "otonom araç çalışma alanı" uyarı levhası (1024 × 640)
  sign_speed.png       hız sınırı levhası (512 × 512)
  floor_walkway.png    yaya yolu zemin yazısı (1024 × 256)

Yazılar Türkçe ve sabit; renkler station_params.yaml → colors'tan. Bölme
etiketlerinde kare çerçeve yoktur: siyah kare kontur ArUco için aday üretir.
Rastgelelik (barkod) sabit tohumla, çıktı her build'de aynıdır.
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

FONT_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
    '/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf',
]
FONT_REG_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf',
]

YELLOW = (242, 190, 0)
BLACK = (24, 24, 24)
WHITE = (250, 250, 248)
DARK = (52, 56, 62)

SIGN_TEXT = {
    'neutral': ('KAYNAK', 'KOLİ ALMA NOKTASI'),
    'red':     ('HEDEF · KIRMIZI', 'KIRMIZI MÜHÜRLÜ KOLİLER'),
    'green':   ('HEDEF · YEŞİL', 'YEŞİL MÜHÜRLÜ KOLİLER'),
    'blue':    ('HEDEF · MAVİ', 'MAVİ MÜHÜRLÜ KOLİLER'),
}


def font(size, bold=True):
    for path in (FONT_CANDIDATES if bold else FONT_REG_CANDIDATES):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size)


def rgb(c):
    return tuple(int(round(255 * v)) for v in c[:3])


def fit_font(draw, text, max_w, size, bold=True):
    """max_w genişliğine sığan en büyük font (size'dan aşağı)."""
    while size > 8:
        f = font(size, bold)
        if draw.textlength(text, font=f) <= max_w:
            return f
        size -= 2
    return font(size, bold)


def box_icon(draw, cx, cy, s, color):
    """Basit koli simgesi: kare + üstte bant çizgisi (çizgi kalınlığı s/10)."""
    w = max(2, s // 10)
    draw.rectangle([cx - s // 2, cy - s // 2 + s // 6, cx + s // 2, cy + s // 2], outline=color, width=w)
    draw.polygon([(cx - s // 2, cy - s // 2 + s // 6), (cx - s // 3, cy - s // 2),
                  (cx + s // 3 + s // 6, cy - s // 2), (cx + s // 2, cy - s // 2 + s // 6)], outline=color, width=w)
    draw.line([(cx, cy - s // 2 + s // 6), (cx, cy - s // 6)], fill=color, width=w)


def station_sign(colors, color):
    W, H = 1024, 256
    im = Image.new('RGB', (W, H), WHITE)
    d = ImageDraw.Draw(im)
    block = DARK if color == 'neutral' else rgb(colors[color])
    d.rectangle([0, 0, H, H], fill=block)
    box_icon(d, H // 2, H // 2 + 6, 120, WHITE)
    d.rectangle([0, 0, W - 1, H - 1], outline=DARK, width=6)
    title, sub = SIGN_TEXT[color]
    d.text((H + 36, 40), title, font=fit_font(d, title, W - H - 72, 92), fill=BLACK)
    d.text((H + 38, 166), sub, font=fit_font(d, sub, W - H - 72, 40, bold=False), fill=DARK)
    return im


def bay_label(k):
    im = Image.new('RGB', (128, 128), YELLOW)
    d = ImageDraw.Draw(im)
    t = '%02d' % k
    f = font(76)
    w = d.textlength(t, font=f)
    d.text(((128 - w) / 2, 18), t, font=f, fill=BLACK)
    return im


def parcel_label():
    W, H = 512, 384
    im = Image.new('RGB', (W, H), WHITE)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, W, 64], fill=BLACK)
    d.text((18, 12), 'SEVK ETİKETİ', font=font(38), fill=WHITE)
    d.text((18, 80), 'GÖNDEREN: HAT 3 · AMR HÜCRESİ', font=font(24, False), fill=BLACK)
    d.text((18, 112), 'ALICI: DEPO B-12', font=font(24, False), fill=BLACK)
    rng = np.random.default_rng(7)
    x = 24
    while x < W - 24:                       # barkod
        bw = int(rng.choice([3, 3, 6, 9]))
        if rng.random() < 0.55:
            d.rectangle([x, 160, x + bw - 1, 300], fill=BLACK)
        x += bw
    d.text((24, 312), '4 006381 333931', font=font(32, False), fill=BLACK)
    d.rectangle([0, 0, W - 1, H - 1], outline=(180, 180, 180), width=3)
    return im


def hazard_tape():
    W, H, period = 512, 64, 64             # 512 / 64 = 8 tam periyot → kesintisiz
    y, x = np.mgrid[0:H, 0:W]
    stripe = ((x + y) % period) < period // 2
    arr = np.where(stripe[..., None], np.array(YELLOW, np.uint8), np.array(BLACK, np.uint8))
    return Image.fromarray(arr.astype(np.uint8))


def hazard_panel():
    W, H, period = 512, 1024, 160          # 0.62 × 1.2 m'lik yüz: ~19 cm'lik şerit çifti
    y, x = np.mgrid[0:H, 0:W]
    stripe = ((x + y) % period) < period // 2
    arr = np.where(stripe[..., None], np.array(YELLOW, np.float32), np.array(BLACK, np.float32))
    rng = np.random.default_rng(3)          # hafif boya aşınması ve kir (sabit tohum)
    noise = rng.normal(0, 6, (H // 8, W // 8)).repeat(8, 0).repeat(8, 1)[..., None]
    arr = np.clip(arr + noise - 4 * (y[..., None] > H * 0.9), 0, 255)
    return Image.fromarray(arr.astype(np.uint8))


def amr_zone_sign():
    W, H = 1024, 640
    im = Image.new('RGB', (W, H), YELLOW)
    d = ImageDraw.Draw(im)
    d.rectangle([12, 12, W - 13, H - 13], outline=BLACK, width=14)
    tri = [(200, 70), (340, 320), (60, 320)]             # uyarı üçgeni
    d.polygon(tri, fill=YELLOW, outline=BLACK, width=18)
    d.text((182, 130), '!', font=font(170), fill=BLACK)
    d.text((390, 80), 'DİKKAT', font=font(120), fill=BLACK)
    d.text((390, 220), 'OTONOM ARAÇ', font=fit_font(d, 'OTONOM ARAÇ', 590, 84), fill=BLACK)
    d.rectangle([40, 370, W - 40, 372], fill=BLACK)
    for i, t in enumerate(('ÇALIŞMA ALANI', 'YETKİSİZ GİRİŞ YASAKTIR')):
        d.text((60, 400 + i * 110), t, font=fit_font(d, t, W - 120, 92), fill=BLACK)
    return im


def speed_sign():
    S = 512
    im = Image.new('RGB', (S, S), WHITE)
    d = ImageDraw.Draw(im)
    d.ellipse([16, 16, S - 16, S - 16], fill=WHITE, outline=(200, 20, 20), width=56)
    f = font(190)
    w = d.textlength('1', font=f)
    d.text(((S - w) / 2, 110), '1', font=f, fill=BLACK)
    d.text((176, 318), 'm/s', font=font(56), fill=BLACK)
    return im


def floor_walkway():
    W, H = 1024, 256
    im = Image.new('RGB', (W, H), (40, 120, 60))
    d = ImageDraw.Draw(im)
    t = 'YAYA YOLU'
    f = font(150)
    w = d.textlength(t, font=f)
    d.text(((W - w) / 2, 40), t, font=f, fill=WHITE)
    return im


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--params', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    P = yaml.safe_load(open(a.params))
    os.makedirs(a.out, exist_ok=True)
    out = {f'sign_{c}.png': station_sign(P['colors'], c) for c in SIGN_TEXT}
    out.update({f'bay_{k:02d}.png': bay_label(k) for k in range(P['markers']['max_bay_id'] + 1)})
    out.update({
        'parcel_label.png': parcel_label(),
        'hazard_tape.png': hazard_tape(),
        'hazard_panel.png': hazard_panel(),
        'sign_amr_zone.png': amr_zone_sign(),
        'sign_speed.png': speed_sign(),
        'floor_walkway.png': floor_walkway(),
    })
    for name, im in out.items():
        im.save(os.path.join(a.out, name))
    print(f'{len(out)} doku → {a.out}')


if __name__ == '__main__':
    main()
