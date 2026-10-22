"""
catalog.py — istasyon ve koli "kataloğu": ölçü, şekil, renk (konum ve doluluk YOK)

Gerçek dünyadaki karşılığı istasyonun tasarım çizimidir (plan R7). Değerler
station_params.yaml'dan okunur; türetilen her büyüklük burada hesaplanır.
forklift_gazebo/scripts/station_layout.py ile aynı formüller; tutarlılığı
forklift_evaluation'daki test denetler (iki paket birbirine bağımlı değil:
gerçek robota simülasyon paketi kurulmaz).

Çerçeveler (DECISIONS D-43): station = istasyon ön kenarının ortası, zeminde;
+x istasyonun içine, +y sol. bay_i: kanal ekseni, aynı eksenler.
"""

from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class Catalog:
    bay_pitch: float            # bölme adımı
    channel_width: float        # çatal kanalı
    ledge_height: float         # çıkıntı üst yüzü z
    ledge_depth: float
    plate_top_z: float          # istasyon zemin plakası üstü (bölme şeridinin yüzeyi)
    plate_thickness: float      # önlük (zemin plakası) kalınlığı: tekerlekler üstündeyken gövde eğimi
    apron_depth: float          # önlüğün ön kenardan dışarı taşması
    # hizalama hedefi (D-55)
    target_size: float          # beyaz plaka kenarı
    target_face_x: float        # hedef yüzü (station x)
    target_z: float             # hedef merkezi z
    target_offset: float        # hedef merkezinin kanal eksenine uzaklığı
    target_ring: tuple          # (dış, iç, nokta) çaplar
    # bölme şeridi (D-56)
    bay_line_width: float
    bay_line_x: tuple           # (başlangıç, bitiş) station x
    # bölme etiketi (D-62: son yanaşmada mesafe)
    label_size: float
    label_face_x: float
    label_center_z: float
    # tabela (N ≥ 3 bölmeli istasyonlar: tam genişlik)
    sign_width: float
    sign_height: float
    sign_face_x: float
    sign_top_z: float
    # koli
    parcel_size: tuple          # (x, y, z)
    seal_size: float            # mühür kenarı (−x yüzünde ortalı)
    parcel_front_gap: float     # bölmede koli ön yüzü x

    @property
    def target_baseline(self):
        """Bir bölmenin iki hedefi arası (çift tabanı)."""
        return 2.0 * self.target_offset


def from_params(P):
    s, m, v, p = P['station'], P['markers'], P['appearance'], P['parcel']
    a = m['align']
    w = v['sign']['width']
    return Catalog(
        bay_pitch=s['bay_pitch'],
        channel_width=s['channel_width'],
        ledge_height=s['ledge_height'],
        ledge_depth=s['ledge_depth'],
        plate_top_z=s['base_plate_thickness'] + v['band_height'],
        plate_thickness=s['base_plate_thickness'],
        apron_depth=s['apron_depth'],
        target_size=a['size'],
        target_face_x=-(m['standoff'] + m['thickness']),
        target_z=s['ledge_height'] / 2.0,
        target_offset=s['channel_width'] / 2.0 + a['edge_gap'] + a['size'] / 2.0,
        target_ring=(a['bullseye']['ring_outer'], a['bullseye']['ring_inner'], a['bullseye']['dot']),
        bay_line_width=v['bay_line']['width'],
        bay_line_x=(-s['apron_depth'] + v['band_width'], 0.0),
        label_size=v['bay_label']['size'],
        label_face_x=s['ledge_depth'] + v['back_plate_recess'] / 2.0 - v['label_thickness'] / 2.0,
        label_center_z=v['bay_label']['center_z'],
        sign_width=w,
        sign_height=w / v['sign']['aspect'],
        sign_face_x=s['ledge_depth'] + s['back_plate_thickness'] + v['back_plate_recess'] - v['sign']['thickness'],
        sign_top_z=v['sign']['top_z'],
        parcel_size=(p['size']['x'], p['size']['y'], p['size']['z']),
        seal_size=p['seal']['size']['y'],
        parcel_front_gap=s['parcel_front_gap'],
    )


def load(path):
    with open(path) as f:
        return from_params(yaml.safe_load(f))
