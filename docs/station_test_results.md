# İstasyon kabul testi sonuçları

Üretim zamanı: 2026-10-19 19:12:13

| # | Test | Ölçülen | Kriter | Sonuç | Koşu |
|---|---|---|---|---|---|
| S1 | xacro → SDF, gz sdf -k | 15/15 dosya "Valid.", uyarı/hata yok | N = 1, 3, 5 × 4 istasyon rengi + 3 koli rengi; hata ve uyarı yok | **GEÇTİ** | 2026-10-19 19:02:27 |
| S2 | Geometri denetimi | 1154 denetim, 0 hata; en büyük ölçü sapması 0.0000 mm; 24 işaret markers.yaml ile, 50 doku ID ile uyumlu | Kanal, adım, çıkıntı, plakalar, işaret konum/boyut, koli: YAML ile ±0.5 mm | **GEÇTİ** | 2026-10-19 19:02:28 |
| S3 | Statik kararlılık | parcel_source_0: en çok 0.000000 mm, eğim 0.0000°; parcel_source_1: en çok 0.000000 mm, eğim 0.0000°; parcel_source_2: en çok 0.000000 mm, eğim 0.0000° (10 s sim, spawn pozuna göre) | Tam dünya, 10 s: hiçbir koli 1 mm'den fazla hareket etmiyor | **GEÇTİ** | 2026-10-19 19:11:13 |
| S8 | Performans | RTF 1.001 (20 s duvar; ekransız EGL; 4 istasyon, 3 koli; kameralar: front_camera 30.3 Hz, lane_camera 30.3 Hz) | RTF ≥ 0.9, iki kamera açık | **GEÇTİ** | 2026-10-19 19:11:33 |
