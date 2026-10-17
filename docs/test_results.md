# Kabul testi sonuçları

Üretim zamanı: 2026-10-14 20:21:15

| # | Test | Ölçülen | Kriter | Sonuç |
|---|---|---|---|---|
| T1 | Xacro → URDF, check_urdf, gz sdf -p | use_sim=false: çıkış kodları 0/0/0, uyarı/hata 0; use_sim=true: çıkış kodları 0/0/0, uyarı/hata 0 | Hata ve uyarı yok (çıkış 0, stderr boş) | **GEÇTİ** |
| T2 | colcon build --symlink-install | çıkış 0, 2 paket, uyarı satırı 0, stderr bölümü 0 | Hata ve uyarı yok | **GEÇTİ** |
| T3 | Bekleme kararlılığı (spawn sonrası 10 s komutsuz) | x/y kayma maks 0.0011 mm, z tepe-tepe 0.0000 mm (591 örnek, base_link z0 = 0.05000 m) | x/y < 1 mm, z salınımı < 1 mm | **GEÇTİ** |
| T4 | TF ağacı (view_frames) | 15 frame (beklenen 15), kök ['odom'], optical ['front_camera_optical_frame', 'lane_camera_optical_frame'], eksik [], fazla [] | Tüm link'ler + 2 optical frame, tek kök; PDF: docs/frames_sim.pdf | **GEÇTİ** |
| T5 | Düz sürüş 0.2 m/s, 5 s | odometri 1.0004 m, gerçek 1.0004 m, fark %0.00, yanal sapma 0.0 mm | Odometri 1.00 m ± %5; gerçek pozla fark < %5 | **GEÇTİ** |
| T6 | Dönüş 0.5 rad/s, 2π/0.5 s | odometri 359.90°, gerçek 359.89° | 360° ± 5° (odometri ve gerçek) | **GEÇTİ** |
| T7 | Çatal yüksüz 0 → 0.05 → 0.10 → 0 m | [PID modu, hız sınırı 0.05 m/s] -0.000→0.05 m: ±5 mm'ye 0.90 s, aşım 0.63 mm, son hata 0.34 mm; 0.050→0.10 m: ±5 mm'ye 0.90 s, aşım 0.63 mm, son hata 0.34 mm; 0.100→0.00 m: ±5 mm'ye 1.92 s, aşım 0.00 mm, son hata 0.00 mm | Her hedefe 3 s içinde ± 5 mm, aşım < 10 mm | **GEÇTİ** |
| T8 | Yük alma senaryosu (D-16 seçenek A) | yaklaşma 1.261 m → base_link x=1.2615 m, y=-0.0 mm (hedef x=1.260); dönüşte x=1.2644, y=-3.2 mm; taşıma 26.2 s, maks bağıl kayma 0.18 mm, maks koli eğimi 0.00°; düşme yok; bırakınca koli eğimi 0.00°, taban z=0.0500 m (ray üstü 0.050), konum x=1.5031 y=-3.3 mm | Taşıma boyunca bağıl kayma < 1 cm; koli düşmüyor; indirince raylarda dik | **GEÇTİ** |
| T9 | Yüklü çatal tutması (0.06 m, 10 s) | eklem sarkması 0.000 mm, taşıyıcı (gerçek poz) sarkması 0.078 mm; koli tabanı z = 0.0851 m | Sarkma < 5 mm | **GEÇTİ** |
| T10 | Ani fren 0.3 m/s → 0, yüklü | fren anında v=0.300 m/s; caster kalkması maks 0.000 mm (mutlak en alt nokta z min/maks -0.000/-0.000 mm); |Δpitch| maks 0.000°; koli kayması maks 0.09 mm | Devrilme yok, arka teker temas kesmiyor (caster kalkması < 1 mm, gerçek poz; D-18) | **GEÇTİ** |
| T11 | Kamera yayını | front_camera: 28.3 Hz duvar / 28.3 Hz sim, image frame_id=front_camera_optical_frame, camera_info frame_id=front_camera_optical_frame, fx=554.4 (beklenen 554.4), 640×480 rgb8; lane_camera: 26.7 Hz duvar / 26.7 Hz sim, image frame_id=lane_camera_optical_frame, camera_info frame_id=lane_camera_optical_frame, fx=381.5 (beklenen 381.5), 640×480 rgb8 | Her kamera ≥ 25 Hz (duvar saati), frame_id optical frame | **GEÇTİ** |
| T12 | Görüş kontrolü (4 kare) | front unloaded_1p5m: mühür 1720 px, kutu (299, 200, 340, 240); front loaded: mühür 0 px, kutu None; lane unloaded: merkez 21×21 zemin %100, orta sütunda koli üst kenarı satırı yok; lane loaded: merkez 21×21 zemin %100, orta sütunda koli üst kenarı satırı 253 | front koliyi 1.5 m'den görüyor (mühür görünür); lane yüksüz/yüklü merkez bölge zemin | **GEÇTİ** |
| T13 | Ağırlık merkezi analizi (hesap + sim) | yüksüz: hesap x=-0.0669 m, zemin z=0.1090 m (M=11.20 kg); sim x=-0.0669 m, zemin z=0.1090 m (M=11.20 kg); x-marjı ön 6.69 cm / arka 21.31 cm; destek üçgeni marjı 6.69 cm; yüklü: hesap x=-0.0538 m, zemin z=0.1169 m (M=11.70 kg); sim x=-0.0539 m, zemin z=0.1169 m (M=11.70 kg); x-marjı ön 5.38 cm / arka 22.61 cm; destek üçgeni marjı 5.38 cm | Destek poligonuna (x ∈ [-0.28, 0]) marj ≥ 3 cm | **GEÇTİ** |
| T14 | Performans (RTF, iki kamera açık) | RTF 1.000 (20 s; ekransız sunucu, iki kamera abone; /clock / duvar saati) | RTF ≥ 0.9 (iki kamera açık) | **GEÇTİ** |
