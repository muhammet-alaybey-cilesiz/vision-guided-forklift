#!/usr/bin/env python3
"""ambientCG'den (CC0 1.0) fabrika dünyası dokularını indirir ve depoya hazırlar.

Çıktı: models/factory_materials/materials/textures/<ad>_{albedo,normal,roughness}.jpg
Dokular depoya eklenir (build ağ gerektirmez); bu script yalnızca kaynağı
belgelemek ve dokuları yeniden üretmek içindir. Kaynak listesi TEXTURES'ta,
lisans ve kaynak bağlantıları models/factory_materials/README.md'de.

Kullanım:
  fetch_factory_textures.py [--out DIR] [--cache DIR]
  --cache: indirilen zip'lerin tutulduğu dizin; zip oradaysa yeniden indirilmez
"""
import argparse
import io
import os
import sys
import time
import urllib.request
import zipfile

from PIL import Image

# ad → (ambientCG asset id, çıktı kenarı [px], haritalar)
# Haritalar: Color → albedo, NormalGL → normal (ogre2 OpenGL kuralı, +Y yukarı),
# Roughness → roughness. Metalness haritaları kullanılmaz: ogre2'de ortam haritası
# olmadan metal yüzey kararıyor; metalness station_params.yaml → finishes'ta sabit.
TEXTURES = {
    'tread_plate': ('DiamondPlate001', 1024, ('Color', 'NormalGL', 'Roughness')),
    'aluminium':   ('Metal009', 512, ('Color', 'NormalGL', 'Roughness')),
    'cardboard':   ('Cardboard004', 512, ('Color', 'NormalGL', 'Roughness')),
    'rubber':      ('Rubber004', 512, ('Color', 'NormalGL', 'Roughness')),
}
SUFFIX = {'Color': 'albedo', 'NormalGL': 'normal', 'Roughness': 'roughness'}
URL = 'https://ambientcg.com/get?file={id}_1K-JPG.zip'


def fetch_zip(asset, cache):
    path = os.path.join(cache, f'{asset}_1K-JPG.zip') if cache else None
    if path and os.path.exists(path):
        return open(path, 'rb').read()
    for attempt in range(3):
        try:
            req = urllib.request.Request(URL.format(id=asset), headers={'User-Agent': 'forklift_gazebo'})
            data = urllib.request.urlopen(req, timeout=600).read()
            break
        except OSError as e:
            print(f'  {asset}: indirme hatası ({e}), yeniden deneniyor', file=sys.stderr)
            time.sleep(5)
    else:
        raise SystemExit(f'{asset} indirilemedi')
    if path:
        os.makedirs(cache, exist_ok=True)
        open(path, 'wb').write(data)
    return data


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(here, '..', 'models', 'factory_materials', 'materials', 'textures'))
    ap.add_argument('--cache', default='')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for name, (asset, edge, maps) in TEXTURES.items():
        z = zipfile.ZipFile(io.BytesIO(fetch_zip(asset, a.cache)))
        for m in maps:
            src = f'{asset}_1K-JPG_{m}.jpg'
            img = Image.open(io.BytesIO(z.read(src)))
            img = img.convert('L' if m == 'Roughness' else 'RGB').resize((edge, edge), Image.LANCZOS)
            dst = os.path.join(a.out, f'{name}_{SUFFIX[m]}.jpg')
            img.save(dst, quality=90)
            print(f'{dst}  ← {src}')


if __name__ == '__main__':
    main()
