# İlk CV + Otonomi Dilimi — Rapor

- **Plan:** `docs/PLAN_CV_AUTONOMY.md`
- **Kararlar:** DECISIONS D-57 ve sonrası
- **Dal:** `feature/cv-autonomy`

| Faz | Durum |
|---|---|
| F1 Altyapı ve ölçüm | **Tamamlandı (2026-10-21), K-1 onayı bekleniyor** |
| F2 Algılama | Başlamadı |
| F3 Alma + GUI modu | Başlamadı |
| F4 Bırakma + transit | Başlamadı |
| F5 Görev + sürüm | Başlamadı |

---

## F1 — Altyapı ve ölçüm

### F1.1 Özet

- **Kamera modeli doğrulandı.** Ground truth (GT) iki kamerada, iki dünyada, 36 görüşte görüntüye çizildi. Tabela, koli, mühür, daire hedef, bölme ve zemin şeridi ile çatal çizimleri görüntüdeki nesnelerin üstüne oturuyor.
- **Sayısal fark:** Hedeflerde yarım piksel düzeltmesiyle kalan fark medyan 0.2 px, en çok 0.84 px.
- **Yarım piksel bulgusu (D-59, karar sende):** gz görüntüyü merkez (319.5, 239.5)'e göre çiziyor, `camera_info` ise (320, 240) bildiriyor. Fark, iki bağımsız yöntemle de ≈ −0.5 px çıktı.
- **Plan geometrisi ölçüldü** (G1–G5):
  - Hesaplar doğru çıktı, bir istisnayla: yüklüyken hedeflerin **temiz** göründüğü mesafe, kolinin gölgesi yüzünden hesaptan 7.5 cm daha uzak (§F1.4).
- **Odometri** tekerlek eklemlerinden hesaplanıyor (D-60).
- **rclpy + gz-transport** aynı süreçte takılmadan çalışıyor.
- **Python bütçesi:** temel renk işlemleri kare başına 2.0 ms (30 Hz bütçesi 33 ms).
- **Çevrimdışı küme:** 756 görüş (station_test 324, factory 432), toplam 1512 kare + GT. Konum: `data/f1/` (310 MB, git'e girmez).
- **Gerileme:** S1–S3 ve S6–S8 GEÇTİ, değerler öncekiyle birebir aynı.

### F1.2 Yapılanlar

| İş | Sonuç |
|---|---|
| `forklift_interfaces` | 10 mesaj + `SetMode` servisi; derleniyor |
| `forklift_perception` (çekirdek) | `CameraModel`: izdüşüm, ışın, düzlem kesişimi, mercek bozulması. 5 birim testi **GEÇTİ** |
| `forklift_evaluation` | `world_gt` (GT geometrisi), `ros_cams` (ROS görüntü/TF/odom), `overlay`, `f1_probe` (ölçüm görevleri) |
| `forklift_gazebo` | gz yardımcıları `scripts/sim_tools.py`'ye taşındı; `SimSession`'a dünya ve `control` seçimi; `station_layout.sign_rect()` (D-58) |
| Gerileme S1–S3, S6–S8 | 2026-10-21 tam koşu: hepsi **GEÇTİ**, değerler birebir aynı. Taşımadan sonra S1/S2 ayrıca yeniden koşuldu: **GEÇTİ** (1231 denetim, 0 hata). Repodaki kanıtların üzerine yazılmasın diye çıktılar depo dışına yazıldı |

### F1.3 GT izdüşümü (kamera modeli doğrulaması)

**Yöntem:** Robot bilinen pozlara ışınlanır (her istasyonun önünde d = 0.6 / 1.2 / 2.0 m ve 0.9 m'de +0.3 m yanal; factory'de her şeridin üstünde). Görüntü, `camera_info` ve TF robotun algısıyla aynı yoldan (ROS) okunur. GT geometrisi `CameraModel` ile çizilir.

**Sayısal denetim:**
- Daire hedef merkezi: görüntüden koyuluk ağırlık merkezi ile ölçülür, izdüşümle karşılaştırılır.
- Şeritler: poligon içindeki renkli piksel oranı ve satır satır merkez farkı ölçülür.

| Ölçü | station_test | factory |
|---|---|---|
| Hedef ölçümü (ön / şerit kamerası) | 64 / 96 | 88 / 120 |
| Ham fark, ortalama (u, v) [px], şerit kamerası | +0.56, +1.00 | +0.53, +0.97 |
| Maske 1 px daraltılınca (u, v) [px], şerit kamerası | −0.37, −0.38 | −0.35, −0.28 |
| Maske 1 px daraltılınca (u, v) [px], ön kamera | −0.36, −0.45 | −0.37, −0.47 |
| −0.5 px düzeltmesiyle kalan fark, medyan / en çok [px] | 0.22 / 0.56 | 0.22 / 0.84 |
| Bölme şeridi: renkli piksel oranı (en düşük) | 0.998 (72 şerit) | 0.977 (96) |
| Bölme şeridi merkez farkı, medyan [px] | −0.50 | −0.50 |
| Zemin şeridi merkez farkı [px] | — | −0.50 (görüntüye tam sığan 1 şerit) |

**Yorum:**

1. Ham farkın (+0.6 / +1.0 px) bir kısmı **ölçüm yönteminden** geliyor. Köşeleri yuvarlanmış maske, plakanın sağında ve altında koyu çıkıntı piksellerini de içine alıyor. Maske 1 px daraltılınca bu etki kayboluyor. Ders: F2'deki hedef algılayıcısı ağırlık maskesini kesinlikle plakanın içinde kurmalı (D-61).
2. Daraltılmış maskede kalan **≈ −0.4 px**, iki kamerada da aynı ve görüntü konumundan bağımsız. Hedeften bağımsız ölçülen şerit merkezi de **−0.50 px**. İkisi birlikte, `camera_info`'nun merkezi yarım piksel yanlış bildirdiğini gösteriyor (D-59). Şerit kamerasında 1 m'de ≈ 1.5 mm etmesi demek.
3. Düzeltmeden sonra kalan fark medyan 0.2 px. Bu, hedefin 45° eğik bakışta küçük görünmesi ve render filtresi düzeyinde.

Görseller: `docs/logs/cv/f1/images/overlay_<dünya>_<görüş>_<kamera>.jpg`. Örneğin:
- `overlay_station_test_source_d1.2_front.jpg`: tabela, renk bloğu, koliler, mühürler, hedefler, bölme şeritleri
- `overlay_station_test_source_d1.2_lane.jpg`: aynı sahne ve çatallar
- `overlay_factory_lane_dest_red_lane.jpg`: zemin şeridi ve durma çizgisi

### F1.4 Görüş ve örtme sınırları (plan G1–G5)

| # | Ölçü | Plan tahmini | Ölçülen | Yöntem |
|---|---|---|---|---|
| G1 | Şerit kamerası zemin aralığı (base_link önü) | 0.22–2.47 m | **0.223–2.483 m** | TF + `camera_info` ile model (izdüşümü §F1.3 doğruladı) |
| G1 | Ön kamera zemin aralığı | — | ≥ **0.668 m** (ufuk görüntüde) | aynı |
| G2 | Çatallar şerit görüntüsünde | Alt ≈ 127 satır | Overlay'de çatal kutuları çatalların üstüne oturuyor, yaklaşık 350. satırdan aşağısı. **Ek bulgu:** robot şeridin üstündeyken 5 cm'lik şerit iki çatalın arasından geçiyor (çatallar arası 4 cm); yakın bölgede çatallar şeridin kenarlarını kapatıyor (`overlay_factory_lane_dest_red_lane.jpg`) | Görsel; maske F2'de modelden türetilir, şerit uydurmada çatal bölgesi kullanılmaz |
| G3 | Taşınan koli: kapatmanın başladığı satır (orta sütun) | ~260 | **258** (silüet) / **257** (görüntü farkı) | Yüklü ve yüksüz kare farkı + kutunun izdüşüm silüeti |
| G3 | Yüklüyken zeminin görüldüğü en yakın mesafe | 0.59 m | **0.593 m** | aynı |
| G3 | Kapatılan sütunlar (üst kenarda) | ±133 px | **185–455** (orta 320 ± 135) | aynı |
| G4 | Yüklüyken hedeflerin silüetten tamamen çıktığı kamera mesafesi | 0.47 m | **0.50 m** (0.475'te %40 kapalı) | 2.5 cm adımlı tarama, dest_red orta bölme |
| G4 | Yüklüyken hedeflerin **gölgesiz** göründüğü kamera mesafesi | — | **0.575 m** | Görüntü farkı 0.50–0.55 m'de de var: kolinin gölgesi sol hedefi ≈ 20 gri düzey karartıyor (sağdakini 3) |
| G5 | Tabelanın (ve renk bloğunun) ön kamerada tam göründüğü en yakın mesafe (base → ön kenar) | 0.59 m | **0.59 m** | Model + 3 doğrulama görüntüsü (`sign_*_d0.54/0.59/1.00.jpg`) |

**Plana etkisi (G4):** Tam girişte kamera ön çizgiye 0.03 m uzakta. Yüklü yanaşmada hedeflerin gölgesiz son ölçümü 0.575 m'de alınırsa, robot **≈ 0.55 m'yi görmeden** gider (plan 0.44 m varsaymıştı).

- **Bütçe:** 0.5° yaw hatası 0.55 m'de 4.8 mm, 1° ise 9.6 mm yanal sapma eder. Plandaki "kayıp anında yaw ≤ 0.5°" hedefi bu yüzden yeterli; A2 bunu ölçecek.
- **İkinci seçenek:** Gölge altında ölçmeye devam etmek. Plaka içi normalizasyon, düzgün kararmayı zaten tolere ediyor; ama kısmi gölge ağırlık merkezini kaydırır. F2'de C4 bunu "yüklü, 0.50–0.575 m" durumu olarak ayrıca ölçecek.

Görseller: `occlusion_station_test_d0.45/0.60/0.80.jpg` (yüklü; kırmızı kontur = koli silüeti, mor = hedefler) ve `_noload` eşleri.

### F1.5 Odometrinin kaynağı (D-60)

| Deneme | GT | `/odom` |
|---|---|---|
| Tekerlek dönmeden 0.5 m ışınlama | +0.500 m | **0.000 m** |
| 0.1 m/s ile 3 s sürüş | 0.3009 m | 0.3009 m |

**Sonuç:** gz DiffDrive odometriyi tekerlek eklemlerinden hesaplıyor; gerçek robottaki enkoderin karşılığı (R1). Zemin kaymadığı için simdeki odometri gerçekte olacağından daha iyi.

### F1.6 Mimari denemeler

| Konu | Sonuç |
|---|---|
| rclpy (2 kamera + `camera_info` + TF + odom) ve gz-transport (poz 60 Hz, istatistik, eklem, 2 kamera) aynı süreçte | **Çalıştı**: iki dünyada toplam ≈ 30 dk, takılma yok. Servisler `gz service` CLI ile (D-49) |
| Python CPU bütçesi (120 kaydedilmiş kare) | RGB→HSV + 5 renk maskesi + morfolojik açma + bağlantılı bileşenler: ortalama **2.01 ms**, p95 2.12 ms (OpenCV 20 iş parçacığı). 30 Hz bütçesinin %6'sı |

### F1.7 Çevrimdışı değerlendirme kümesi

| Küme | station_test | factory | İçerik |
|---|---|---|---|
| `stations` | 288 | 288 | 4 istasyon × d {0.45, 0.6, 0.8, 1.0, 1.3, 1.7, 2.2, 2.8} m × yanal {−0.3, 0, 0.3} × yaw {−10°, 0, 10°} |
| `loaded` | 36 | 36 | 3 hedef istasyon × d {0.6–1.3} × yanal × koli çatalda (q 0.06) |
| `lanes` | — | 108 | 4 şerit × 3 konum × yanal {±8 cm, 0} × yaw {±10°, 0} |

- **Her görüş için kaydedilenler:** iki PNG (kayıpsız) ve şu GT bilgileri: robot pozu, çatal yüksekliği, kolilerin pozları, görüntü damgaları. `meta.json` kamera modellerini tutar.
- **Kayıt süresi:** 409 s ve 554 s.
- **Kapsam dışı kalan:** Robot ve istasyon çakışması nedeniyle atlanan görüş yok.
- Görüntüler gürültüsüz render. F2'deki gürültü taraması (C7) gürültüyü bu kümenin üstüne ekleyecek.

### F1.8 Çalıştırılmayan testler ve açık noktalar

| Konu | Durum |
|---|---|
| T1–T14 (lift_test) | **ÇALIŞTIRILMADI**: robot, köprü ve bringup değişmedi; yalnız test yardımcıları taşındı |
| GUI U/I testleri | **ÇALIŞTIRILMADI**: GUI değişmedi |
| Çatal maskesinin satır sınırları (G2) | Yalnız görsel olarak kontrol edildi; F2'de modelden türetilip birim testiyle ölçülecek |
| `camera_info` yarım piksel farkı | Karar bekliyor (D-59) |
| Kolinin hedeflere düşen gölgesi | F2'de C4'ün yüklü yakın durumu olarak ölçülecek |

### F1.9 Nasıl çalıştırılır

```bash
cd ~/cv_forklift_ws
colcon build --packages-select forklift_interfaces forklift_gazebo forklift_perception forklift_evaluation
source install/setup.bash
python3 -m pytest -q src/forklift_perception/test                       # kamera modeli birim testleri
ros2 run forklift_evaluation f1_probe --world station_test --tasks overlay,occlusion,sign,odom
ros2 run forklift_evaluation f1_probe --world factory --tasks overlay,sign
ros2 run forklift_evaluation f1_probe --world station_test --tasks record   # küme → data/f1/
ros2 run forklift_evaluation f1_probe --cpu                                 # simülasyonsuz
```

Her koşu kendi ekransız simülasyonunu açar. Çalışan başka bir simülasyonla aynı `GZ_PARTITION`'da çakışır.
