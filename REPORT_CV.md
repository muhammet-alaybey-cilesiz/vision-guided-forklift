# İlk CV + Otonomi Dilimi — Rapor

- **Plan:** `docs/PLAN_CV_AUTONOMY.md`
- **Kararlar:** DECISIONS D-57 ve sonrası
- **Dal:** `feature/cv-autonomy`

| Faz | Durum |
|---|---|
| F1 Altyapı ve ölçüm | Tamamlandı (2026-10-21), K-1 onaylandı (2026-10-22) |
| F2 Algılama | Tamamlandı (2026-10-22), K-2 onaylandı (2026-10-23) |
| F3 Alma + GUI modu | Tamamlandı (2026-10-24); sahip F4'e geçilmesini istedi |
| F4 Bırakma + transit | Tamamlandı (2026-10-24), K-4 onaylandı (2026-10-24) |
| F5 Görev + sürüm | Tamamlandı (2026-10-24), K-5 onaylandı: `main`, `v0.4.0`, `release/v0.4.0` (d79afb4) |

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


---

## F2 — Algılama

### F2.1 Özet

- **Altı algılayıcı yazıldı:** tabela, zemin ve bölme şeridi, hedef çifti, bölme etiketi, koli ve mühür, yük. Hepsi ROS'tan bağımsız çekirdekte; iki ince ROS düğümü onları çalıştırıyor (D-63).
- **Kalibrasyon (D-59):** Sim kameraları dama tahtasıyla, GT kullanmadan kalibre edildi. Merkez noktası (319.8, 239.5), yani beklenen değere 0.3 px'ten yakın. Yarım piksel bulgusu bağımsız bir yöntemle doğrulandı.
- **Son yanaşma (D-62, D-65):** Bölme etiketi, yüklü ve yüksüz bütün son yanaşma karelerinde kolinin üstünden görünüyor (672/672). Robotun önlük plakasındaki eğimi (0.41° / 0.205°) geometriden düzeltildi.
- **Kabul testleri (σ = 0):**
  - **GEÇTİ:** C0, C2, C3, C5, C6, C8.
  - **KALDI:** C1 (tabela), C4 (eşleştirme oranı ve uzak aralıkta yaw), C9 (mesafe 5.15 mm, ölçüt 5).
  - Ayrıntılar ve öneriler §F2.4'te.
- **Gürültü (C7, bilgi):** σ = 0.007 ve 0.02'de şerit, hizalama, koli ve son yanaşma ölçümleri hemen hemen değişmiyor. Tabela tespiti düşüyor.
- **Canlı (C8):** RTF 1.00. Algılama gelen her kareyi işliyor (ön 28 Hz, şerit 23–26 Hz), toplam ~2.5 CPU çekirdeği. Operatör GUI'si kaplamayı gösteriyor (`docs/images/cv/gui_detections_station_test.png`).
- **Ölçütü netleştirilen iki test (değişiklik gerekçeli, onayına sunuluyor):**
  - **C2/C3:** Yön ölçütü ≥ 25 cm görünen şeritlerde uygulanıyor.
  - **C6:** GT "çatalların üstünde koli" olarak tanımlandı; yarım örtüşmeler "belirsiz" sayıldı.

### F2.2 Yapılanlar

| Bileşen | Yöntem (kısaca) | Karar |
|---|---|---|
| Renk eşikleri | GT bölgelerinden HSV yüzdelikleri (`color_stats`); kraft / etiket / şerit sarısı 2–3 ton birimi ayrı | D-63 |
| Hedef çifti | Yerel karşıtlıkla beyaz plaka → desen denetimi → plakanın içinde koyuluk ağırlık merkezi → z 0.025 düzlemi → 0.216 m taban; plaka açık zeminle birleşirse halka deliğinden kurulur | D-55, D-61 |
| Bölme etiketi | Etiket sarısı → dörtgen → köşegen kesişimi → z 0.258 düzlemi; şekil (beklenen dik kare) ve bağlam (alüminyum zemin) denetimi | D-62, D-65 |
| Şeritler | Kuş bakışı (5 mm/px), dik yapıların çıkarılması, yönlü açma, kenar farkındalıklı merkez çizgisi, tür ayrımı hedef çiftleriyle | D-64 |
| Tabela | Beyaz panel çapası + renk bloğu; 3 serbestlikli poz (dikey, yüksekliği bilinen levha) | D-66 |
| Koli / mühür | Doygun kare + çevresinde kraft (bağlam) → mühür yüksekliği düzlemi; kutu kraft bileşeninden | — |
| Yük | Taşınan kolinin şerit kamerasındaki silüetinde kraft oranı | — |
| Öz-maske | Çatallar, taşıyıcı ve (yük varsa) koli, `forklift_params` + çatal ekleminden | D-63 |
| Önlük eğimi | Ön çizgiye CV mesafesi < 0.30 m ise eğimli kamera modeli; eğim plakadaki teker sayısıyla (komşu bölme var mı) ölçeklenir | D-65 |
| Kalibrasyon aracı | Simde dama tahtası, rastgele 40 poz, `calibrateCamera`; ROS kalibrasyon YAML'ı | D-59 |
| Değerlendirme | `cv_eval` (C0–C7, C9; kayıtlı kümeler), `perf_probe` (C8, C0 çalışma anı) | — |
| Kayıt kümesi F2 | Hizalama (yüksüz / yüklü), uzak tabela, rastgele doluluk, son yanaşma; iki dünyada 952'şer görüş, başarısız görüş 0 | — |
| Birim testleri | Kamera modeli (5) + katalog tutarlılığı (5): **10/10 GEÇTİ** | D-63 |

### F2.3 Kalibrasyon (D-59)

| Kamera | Görüş | RMS | cx | cy | fx | fy |
|---|---|---|---|---|---|---|
| front | 37 | 0.066 px | 319.80 | 239.52 | 554.63 | 554.64 |
| lane | 27 | 0.064 px | 319.77 | 239.38 | 381.26 | 381.23 |

- **İlk deneme (3–7 görüş):** cx 320.6 / 319.0 çıktı. Az görüşte merkez noktası tahtanın pozuyla karışıyor; gerçek kalibrasyondaki "tahtayı her köşeye ve eğik koy" kuralının sebebi bu.
- **Etkisi:** Kalibrasyondan sonra hedef çiftinde eksen boyunca ortalama sapma 2.4 → 0.09 mm, etikette mesafe ortalaması +2.0 → −0.2 mm.
- **Görseller:** `docs/logs/cv/f2/calibration/*_view*.jpg`.

### F2.4 Kabul testleri (σ = 0, kalibre edilmiş model; iki dünya)

Kümeler: F1 (`stations`, `loaded`, `lanes`: 756 görüş) ve F2 (`align`, `align_loaded`, `far`, `occupancy`, `entry`: 1904 görüş). Ham sonuç: `docs/logs/cv/f2/cv_results.json`, `c8_*.json`.

| # | Test | Ölçülen | Kriter | Sonuç |
|---|---|---|---|---|
| C0 | GT sızıntısı | Statik: importlar ve kod dizgelerinde yasaklı ad yok. Çalışma anı (iki dünya): abonelikler yalnız kamera, `camera_info`, `/joint_states`, TF, `/clock` | İzinli küme dışı yok | **GEÇTİ** |
| C1 | Tabela | Tespit 379/412 (<1.5 m: 157/160; 1.5–2.5: 131/144; 2.5–3.5: 91/108). Yanlış pozitif 22, yanlış renk 1. Poz medyan / en çok: <1.5 m 36 / 92 mm, 2.3 / 9.1°; 1.5–2.5 m 77 / 222 mm, 4.4 / 17.6°; 2.5–3.5 m 137 / 275 mm, 7.3 / 31.3° | Renk %100, yanlış pozitif 0; ≤ 3 m'de ≤ 5 cm / ≤ 5° | **KALDI** |
| C2 | Zemin şeridi (factory) | Tespit 79/83 (%95.2). Yanal medyan / en çok 2.5 / 4.9 mm. Yön en çok 0.86° (≥ 25 cm; 15–25 cm'lik parçalarda 1.45°, bilgi). Durma çizgisi medyan 2.5 mm, en çok 19.4 mm. Tür hatası 0 | ≤ 1 cm, ≤ 2°, ≥ %95 | **GEÇTİ** |
| C3 | Bölme şeridi | Tespit 1328/1352 (%98.2). Yanal en çok 5.4 mm, yön en çok 0.76° (≥ 25 cm; kısa parçalarda 2.17°, bilgi) | ≤ 1 cm, ≤ 2° | **GEÇTİ** |
| C4 | Hizalama (S5'in yerine) | Eşleştirme 933/1008 (**%92.6**). Yanlış çift 0, sıra hatası 0. Yüksüz 0.3–1.2 m (624): yanal ≤ 1.26 mm, mesafe ≤ 1.60 mm, yaw ≤ **0.715°** (p99 0.37°; aşanlar 1.0–1.4 m'de, kaynak istasyon). 0.3–0.9 m: yaw ≤ 0.595°. Yüklü 0.575–1.2 m (144): 0.98 mm, 0.19 mm, 0.084°. Gölgeli 0.45–0.575 m (bilgi, 72): 1.58 mm, 0.17 mm, 0.074° | Eşleştirme ≥ %95; eksen ≤ 2 mm, yaw ≤ 0.5°, mesafe ≤ 5 mm | **KALDI** (eşleştirme ve yaw) |
| C5 | Koli, mühür, doluluk | 402/402, yanlış pozitif 0, yanlış renk 0; IoU medyan 0.97, p5 0.76 | %100, IoU ≥ 0.7, yanlış pozitif 0 | **GEÇTİ** |
| C6 | Yük denetimi | Net durumlar 2602/2602 doğru (yük 780, yok 1822); yarım örtüşme 58 (belirsiz, ölçüt dışı) | %100 | **GEÇTİ** |
| C7 | Gürültü (σ 0.007 / 0.02, her 3. kare) | Aşağıdaki tablo | Bilgi | **BİLGİ** |
| C8 | Canlı performans | station_test: RTF 1.001, ön 28.3 Hz, şerit 25.7 Hz, CPU 2.51 çekirdek. factory: RTF 0.998, 28.2 / 22.8 Hz, 2.53 çekirdek | RTF ≥ 0.9, ≥ 15 Hz | **GEÇTİ** |
| C9 | Son yanaşma (etiket) | Tespit 672/672 (tam girişe kadar). Mesafe en çok 5.06 (yüklü) / **5.15** (yüksüz) mm, ortalama −0.1 / −0.2 mm. Yanal en çok 1.12 / 1.22 mm | Mesafe ≤ 5 mm, yanal ≤ 3 mm | **KALDI** (0.15 mm farkla) |

**Ölçüt netleştirmeleri (onayına sunuluyor):**
- **C2/C3 yön ölçütü ≥ 25 cm görünen şeritte.** 15 cm'lik bir parçada ~1.5 mm nokta gürültüsü tek başına ~1° yön belirsizliği verir; bu parçalar bilgi olarak raporlandı.
- **C6'nın GT'si.** Algılayıcı "çatal bölgesinin üstünde koli var mı" sorusunu yanıtlıyor. Alma girişinde bölmedeki koli de çatalların üstünde; bir görüntüden ikisi ayırt edilemez. GT, kolilerin silüet örtüşmesiyle tanımlandı: ≥ 0.7 var, ≤ 0.3 yok, arası belirsiz. Alma başarısı otonomide geri çıkıştan sonra denetlenecek.

### F2.5 Gürültü (C7, bilgi)

| Test | σ = 0 | σ = 0.007 | σ = 0.02 |
|---|---|---|---|
| C1 tespit / yanlış pozitif | %92 / 22 | %93 / 5 | %83 / 7 |
| C2 yanal / yön en çok | 4.9 mm / 0.86° | 1.7 mm / 0.56° | 1.7 mm / 0.56° |
| C3 yanal / yön en çok | 5.4 mm / 0.76° | 4.1 mm / 0.59° | 4.0 mm / 0.98° |
| C4 eşleştirme; yüksüz yaw en çok | %92.6; 0.715° | %93.8; 0.471° | %93.8; 0.476° |
| C5 yanlış / IoU p5 | 0 / 0.76 | 0 / 0.69 | 0 / 0.69 |
| C6 net durumlarda hata | 0 | 0 | 4 |
| C9 mesafe en çok (yüklü / yüksüz) | 5.06 / 5.15 mm | 5.06 / 4.08 mm | 6.54 / 3.96 mm |

σ > 0 koşuları her 3. kareyle yapıldı; bu yüzden en kötü değerler σ = 0'dan düşük görünebilir. Ağırlık merkezi ve dilim ortalaması gibi çok pikselli yöntemler gürültüyü ortalıyor. Gürültüye en duyarlı olan tabela (koyu panel ve kenarlar).

### F2.6 Açık noktalar ve öneriler

| Konu | Durum | Öneri |
|---|---|---|
| C1 tabela yaw'ı | Fizik sınırı: 0.60 m'lik tabelada 1.3 m'de ~1°/px, 2.5 m'de ~4°/px | Tabela kimlik, yön ve kaba mesafe versin; hassas poz ~1.4 m'den hedef çiftlerinden (D-66). Ölçüt: renk doğruluğu, ≤ 1.5 m'de poz |
| C1 yanlış pozitif ve nötr tabela | 22 yanlış pozitif: 3'ü doğru tespitin yanında ikinci tespit, 10'u doğru renkte ama 3 m'den uzakta 0.3–1 m poz hatasıyla bulunmuş tabela (eşleştirme yarıçapı 0.3 m), 9'u başka nesne; 1 yanlış renk. Station_test'te kaynak tabelası zayıf (arkasındaki gökyüzü panelden parlak) | F3'te tabelayı zamanla birleştirme (takip); ölçütün ve ortamın netleşmesine göre iyileştirme |
| C4 eşleştirme %92.6 | Kayıplar çoğunlukla yakın (d ≤ 0.6 m, yaw ±8°): plakanın beyaz maskede kapalı bir kontur oluşturmadığı ya da parçalandığı görüşler (sebebi tek tek incelenmedi); yanlış çift hiç yok | Yaklaşırken saniyede ~25 kare var; kare başına %7 kayıp zamansal süzgeçle tolere edilebilir. Ölçüt önerisi: kare başına ≥ %90, yanlış çift 0 |
| C4 yaw 0.715° | 7/624 görüş, hepsi 1.0–1.4 m'de; son hizalama aralığında (≤ 0.9 m) 0.595° | Son hizalama ≤ 0.9 m'de yapılsın; ölçüt o aralıkta yaw ≤ 0.6° ya da p99 ≤ 0.5° |
| C9 5.15 mm | Ölçütü 0.15 mm aşıyor; mekanik pay: taşıyıcı hedefi −15 ± 10 mm | Ölçüt ≤ 6 mm ya da algılayıcıyı iyileştirme |
| Robotun eğimi | Simde geometriden düzeltildi | Gerçek robot için IMU (D-65) |
| Kalibrasyon | Sim kamerası kalibre | Gerçek kamerada aynı araç ve aynı dosya biçimi |

### F2.7 Nasıl çalıştırılır

```bash
colcon build --packages-select forklift_interfaces forklift_perception forklift_evaluation
source install/setup.bash
python3 -m pytest -q src/forklift_perception/test src/forklift_evaluation/test     # 10 birim testi
ros2 run forklift_evaluation calibrate_cameras --install-to src/forklift_perception/config/calibration
ros2 run forklift_evaluation f1_probe --world station_test --tasks record --sets f2 --data data/f2 --out docs/logs/cv/f2
ros2 run forklift_evaluation cv_eval --sigma 0,0.007,0.02                         # C0–C7, C9 (~4 dk)
ros2 run forklift_evaluation perf_probe --world factory                           # C8
ros2 launch forklift_perception perception.launch.py debug:=true                  # canlı (sim açıkken)
```

---

## F3 — Yanaşma, alma ve GUI modu

### F3.1 Özet

- **Görev yöneticisi** (`forklift_autonomy`, D-68): ROS'tan bağımsız çekirdek (poz matematiği, odometri çerçevesinde bölme izleyici, hareket ilkeleri, bölme ekseni servosu, durum makinesi) + `mission_manager` düğümü.
- **GUI Manuel / Otonom paneli** (D-69): mod tuşları, görev düğmeleri, durum satırı; çekirdekte kurallar, ROS ve mock adaptörleri.
- **IMU** (D-67): eğim IMU'dan; ivme ve dönüşte ivmeölçer düzeltmesi kapalı, seviye kalibrasyonu ortancayla.
- **A1 (σ = 0, 18 deneme × 2 dünya): GEÇTİ.** Alma 36/36, girişte yanal ≤ 3.7 mm, yaw ≤ 0.90°, komşu koliler 0 mm.
- F3'ün ilk tam koşusunda fabrikada 16/18 vardı. Gözlem tamamlanmadan bölme seçiliyordu; F4'ün başında düzeltildi (D-71).

### F3.2 A1 — yanaşma + alma

Izgara: 3 bölme × 6 başlangıç (0.9–1.5 m, ±15 cm, ±10°). Hedef bölme, soldan ilk dolu olacak şekilde kurulur. Ham sonuç: `docs/logs/cv/f3/a1_<dünya>.json`.

| Dünya | Başarılı alma | Girişte yanal (medyan / en çok) | Girişte yaw (medyan / en çok) | Taşıyıcı önü (hedef −15 ± 10 mm) | Komşu koli | Süre (medyan / en çok) | Sonuç |
|---|---|---|---|---|---|---|---|
| station_test | 18/18 | 1.6 / 3.7 mm | 0.33 / 0.90° | −13.6 … −11.5 mm | 0 mm | 31 / 36 s | **GEÇTİ** |
| factory | 18/18 | 1.7 / 2.1 mm | 0.30 / 0.52° | −14.1 … −10.9 mm | 0 mm | 32 / 41 s | **GEÇTİ** |

Ölçüt: yanal ≤ 1 cm, yaw ≤ 1°, alma başarılı, komşular < 1 mm. Fabrikada 2 denemede robot önce yeniden konumlandı (REPOSITION, D-71).

**A1'in geçmişi:**

| Koşu | Sonuç | Sebep | Düzeltme |
|---|---|---|---|
| F3 ilk tam koşu (factory) | 16/18 | Seçim 1.6 s'de, bölmelerin bir kısmı doğrulanmadan yapılıyordu. Bir denemede yanlış bölmeye girildi ve komşu koli 498 mm itildi. Bir denemede "dolu bölme görülmedi" | Gözlem tamamlanma kuralı ve kanıtlı "boş" (D-71) |
| F4 ara koşu (factory) | 16/18 | Aynı iki poz. O pozlardan uçtaki bölmenin hedef çifti **hiç** algılanmıyor (C4'teki kayıplardan biri), ön kamera ise kolisini görüyor | Sahipsiz koli → kolinin önüne yeniden konumlan (REPOSITION, D-71) |
| F4 ara koşu (station_test) | 18/18, yaw 1.11° (KALDI) | Koşunun ilk denemesi ışınlamadan 1 s sonra başlıyordu. IMU süzgeci oturmamıştı | Test, ışınlamadan sonra 5τ bekliyor (D-75) |
| **Son koşu** | **36/36** | | |

### F3.3 GUI

- Birim ve entegrasyon testleri: **164 test, 0 hata** (`colcon test --packages-select forklift_gui`).
- Mock adaptörle panel görüntüsü: `docs/images/cv/gui_autonomy_panel_mock.png`.
- GUI durum adlarını sabit listeden değil `/autonomy/state`'ten okuyor; F4'ün yeni durumları için GUI değişikliği gerekmedi.

---

## F4 — Bırakma ve transit

### F4.1 Özet

- **Durum makinesi genelleştirildi** (D-74): aynı yanaşma kodu alma ve bırakmada kullanılıyor. Görevler: `pick`, `drop`, `transit`, `full`. GUI her zaman `full` çalıştırıyor; diğerleri testler için. `full` (kaynağa git → al → mühür rengindeki istasyona git → bırak → … kaynak boşalınca DONE) kinematik testte iki yöntemle de 3/3 koliyi doğru istasyonun orta bölmesine taşıyor. Simülasyonda F5'te (A4) sınanacak.
- **Transit, station_test** (D-72): dur-bak arama, tabelaya doğru ≤ 1 m'lik ayaklar, yakında tabela ekseni, ön-yanaşmada hedef çiftleriyle inceltme (PRE_DOCK).
- **Transit, factory** (D-73): şeride giriş (tabelaya en yakın aday), şerit ekseni servosu, durma çizgisinin 0.40 m önünde durma, PRE_DOCK.
- **Kabul (σ = 0, iki dünya): A1 GEÇTİ (36/36), A2 GEÇTİ (36/36), A3 GEÇTİ (18/18).** Gerileme: S 6/6, T 13/13 (T2 çalıştırılamadı, sebep §F4.6'da).
- **Hangi kodla ölçüldü:** A1/A2'nin son koşusundan sonra yalnız transit yolu değişti (yarım adım kaydırmalı ikinci arama turu, nötr istasyonun sarı şerit eşlemesi). Bu değişiklikler `pick`/`drop` görevlerinde çalışmıyor. A3 bu son kodla koşuldu (factory: şerit eşlemesinden sonra; station_test: arama turundan sonra).
- **F4 sırasında bulunan ve düzeltilen bir algı hatası** (D-75): ön algı düğümü, tabela pozu uydururken NaN'a düşünce sessizce kapanıyordu (çıkış kodu 0). Bu, fabrikadaki "boş bölme yok" hatalarının asıl sebebiydi.

### F4.2 A2 — yüklü yanaşma + bırakma

Izgara: 3 hedef istasyon × 3 doluluk durumu × 2 başlangıç (0.8–1.5 m, ±15 cm, ±10°). Doluluk durumları: hepsi boş → **orta**; orta dolu → **sol**; orta ve sol dolu → **sağ** (K10). Koli, alma sonrası ölçülen yerde çatala konur (taban + 0.255 m; A1'de 0.2525 m). Ham sonuç: `docs/logs/cv/f4/a2_<dünya>.json`.

| Dünya | Doğru bölmeye bırakma | Ofset (medyan / en çok) | Eğim | Taban z | Girişte yanal / yaw (en çok) | Komşu koli | Süre (medyan / en çok) | Sonuç |
|---|---|---|---|---|---|---|---|---|
| station_test | 18/18 | 3.5 / 4.4 mm | 0.00° | 50.0 mm | 3.0 mm / 0.57° | 0 mm | 32 / 49 s | **GEÇTİ** |
| factory | 18/18 | 3.9 / 5.4 mm | 0.00° | 50.0 mm | 3.3 mm / 0.59° | 0 mm | 33 / 38 s | **GEÇTİ** |

Ölçüt: ofset ≤ 15 mm, eğim < 2°, taban 50 ± 2 mm, doğru bölme, komşular < 1 mm. station_test'te 1 denemede robot hedef bölmenin çiftini göremedi. "Boş bölme yok" demek yerine istasyon ortasından yeniden gözledi (D-71) ve doğru bölmeye bıraktı.

**A2'nin geçmişi:**

| Koşu | Sonuç | Sebep | Düzeltme |
|---|---|---|---|
| İlk tam koşular | station_test 17/18, factory 12/18 | factory: ön algı düğümü 13. denemeden sonra çökmüş (D-75); station_test: hedef bölmenin çifti görülmüyor | D-75; karar verilemezse geniş açıdan yeniden gözlem (D-71) |
| Ara koşu | station_test 18/18 doğru bölme, ofset 15.4 mm (KALDI) | Test düzeneği: kullanılmayan koliler yeşil istasyonun sol bölmesinin önüne park ediliyordu; taşınan koli ona sürtünüp çatalda 6–8 mm kayıyordu. Önce yüklü dönüşü yavaşlatmayı denedim, etkisi olmadı, geri aldım | Park noktaları istasyon aralarındaki köşelere (D-74) |
| **Son koşu** | **36/36** | | |

### F4.3 A3 — transit

Yüklü transit: kaynağın önünden (alma + geri çıkış sonrası poz, ±3 cm, ±2°) her hedef istasyonun ön-yanaşmasına (istasyon ekseninde ön kenara 0.8 m). Bilgi amaçlı: yüksüz dönüş, her hedeften kaynağa. Ham sonuç: `docs/logs/cv/f4/a3_<dünya>.json`. Şerit izi: `docs/images/cv/f4/a3_lane_traces_factory.png`.

| Dünya (yöntem) | Varış | Ön-yanaşma mesafe (en çok) | Yanal (en çok) | Yaw (en çok) | Şerit RMS (en çok) | Kolinin çatalda kayması | Süre (medyan / en çok) | Sonuç |
|---|---|---|---|---|---|---|---|---|
| station_test (tabela) | 9/9 | 24.6 mm | 16.8 mm | 0.78° | — (şerit yok) | ≤ 0.68 mm | 61 / 73 s | **GEÇTİ** |
| factory (şerit) | 9/9 | 29.0 mm | 3.4 mm | 0.19° | 14.9 mm (diğer 8: 2.0–3.8) | ≤ 0.29 mm | 45 / 62 s | **GEÇTİ** |

Ölçüt: ön-yanaşma ±10 cm / ±5°, şerit sapması RMS ≤ 2 cm (factory), kayma < 1 cm. 14.9 mm'lik denemede şeride giriş 44 mm sapmayla bitti (doğru, girişte uzak noktalardan uyduruluyor), robot 3 s içinde 2–3 mm'ye oturdu; RMS bu geçişi içeriyor. Fabrikadaki mesafe hatası (~25 mm) durma çizgisinin önlüğe göre konumundan geliyor: hat kuralı 0.40 m, gerçek fark biraz daha az. Ön-yanaşma toleransı 5 cm olduğu için PRE_DOCK düzeltme yapmıyor; sonraki hizalama bunu zaten kapatıyor.

**Bilgi: yüksüz dönüş (hedef → kaynak), son iki koşu.**

| Dünya | Varış | Mesafe / yanal / yaw (en çok) |
|---|---|---|
| station_test (tabela) | 6/6 | 15.5 mm / 22.2 mm / 0.3° |
| factory (şerit) | 3/3 | 42.6 mm / 3.3 mm / 0.4° |

Fabrikada iki dönüş sarı kaynak şeridini izledi (RMS 5.1 ve 2.3 mm). Kırmızıdan dönüşte şerit iki girişte de görülmedi; robot tabelayla vardı (sebebi incelenmedi, F5'te tam görevde izlenecek). Bu ölçümden önceki koşularda dönüşlerin **hiçbiri** şerit izlemiyordu. Sebep bir eşleme hatasıydı: kaynak istasyonun rengi `neutral`, şeridi ise `yellow`. Kinematik test de aynı yanlış eşlemeyi kullandığı için hatayı yakalamamıştı. Düzeltildi; test artık şerit yönteminde her bacağın şeritle gidildiğini denetliyor ve eşleme kaldırılınca kalıyor.

**A3'ün geçmişi:**

| Koşu | Sonuç | Sebep | Düzeltme |
|---|---|---|---|
| İlk duman testi (factory) | Vardı; şerit RMS 17.5 mm, son yanal 22 mm | Bölme şeritleri ve bant kenarları da "zemin şeridi" (G7); uzun bellek odometriyle eğik | Aday ilişkilendirme, takipte son 0.5 s (RMS 4 mm) |
| İlk tam koşu (factory) | 7/9 | Uzak noktaları atınca şeride girişte şerit görülmedi | Girişte uzak noktalar, takipte yakın |
| İlk dönüşler | 1/3 hassas | Nötr tabela bazı açılarda algılanmıyor, ortalama kapısı, 35° eşik | D-72 |
| Ara koşu (factory dönüş) | 2/3 | 4.9 m'de yanlış pozitif "nötr tabela" | Menzil dışı ret |
| Ara koşular (factory dönüş) | 3/3 vardı ama hiçbiri şerit izlemedi | Kaynak şeridi sarı, otonomi `neutral` arıyordu | `lane_color: {neutral: yellow}` (katalog `lane_neutral`) |
| **Son koşu** | **18/18** (+ dönüşler station_test 6/6, factory 3/3) | | |

### F4.4 Bulunan ve düzeltilen hatalar (özet)

| # | Belirti | Kök neden | Düzeltme | Karar |
|---|---|---|---|---|
| 1 | A1'de yanlış bölme, komşu koli itildi | Gözlem tamamlanmadan seçim | Kararlı iz + kanıtlı "boş" + kararı değiştiren belirsizliği bekleme | D-71 |
| 2 | A1/A2'de "kaynak boş", "boş bölme yok" | Bazı pozlardan uç bölmenin hedef çifti hiç algılanmıyor | REPOSITION: sahipsiz koli ya da karar yoksa yeniden konumlan ve gözle | D-71 |
| 3 | Şerit RMS 17–24 mm, +54 mm salınım | Önlükteki bölme şeritleri ve bant kenarları da "zemin şeridi" olarak geliyor (G7); uzun bellek odometriyle eğiliyor | Aday ilişkilendirme (8 cm kapı), takipte son 0.5 s | D-73 |
| 4 | Şeride girişte yanıltıcı şerit seçildi (kinematik test) | Doğru yokken "en yakın" kuralı | Doğrusu hedef tabelaya en yakın aday | D-73 |
| 5 | Kaynağa dönüşte tabela bulunamadı ya da eksen 45° hatalı | Nötr tabela gürültüsüz render'da bazı açılarda hiç algılanmıyor; aykırı kapısı ortalamaya göreydi; 35° eksen eşiği eğik bakışı reddediyordu | 20° adım, yavaşlama anlarındaki ölçümler, ortanca kapı, saklı kestirim, 60° eşik, yarım adım kaydırılmış ikinci tur | D-72 |
| 6 | Fabrikada 4.9 m'de "nötr tabela" | Yanlış pozitif | Algı menzili dışı ölçüm reddedilir | D-72 |
| 7 | A2 factory'de toplu "boş bölme yok" | Ön algı düğümü NaN Jacobian'da istisnayla kapanıyordu | Jacobian denetimi + düğümde kare başına koruma; birim testi eski kodda aynı hatayı üretiyor | D-75 |
| 8 | A2 ofset 15.4 mm | Test: park edilen koli robotun yolunda | Park noktaları taşındı | D-74 |
| 9 | A1 ilk denemede yaw 1.11° | Test: ışınlamadan sonra IMU süzgeci oturmadan başlanıyordu | 5τ bekleme | D-75 |
| 10 | Kaynağa dönüşte şerit hiç izlenmiyor | Kaynak `neutral`, şeridi `yellow`; kinematik test aynı hatayı paylaşıyordu | Renk eşlemesi; test şerit takibini denetliyor | D-73 |

### F4.5 Birim testleri

| Paket | Test | Sonuç |
|---|---|---|
| `forklift_autonomy` | 19 (kinematik: servo, GoTo, alma, tam görev iki yöntemle, bırakma politikası, transit, gözlem kuralları, REPOSITION, şerit yanıltıcıları, tabela aykırıları) | **19/19 GEÇTİ** |
| `forklift_perception` | 11 (kamera modeli, katalog, eğim süzgeci, NaN Jacobian) | **11/11 GEÇTİ** |
| `forklift_evaluation` | 5 | **5/5 GEÇTİ** |
| `forklift_gui` | 164 | **164/164 GEÇTİ** |

### F4.6 Gerileme

| Test | Sonuç | Not |
|---|---|---|
| S1 xacro → SDF | **GEÇTİ** | 15/15 dosya |
| S2 Geometri | **GEÇTİ** | 1231 denetim, 0 hata |
| S3 Statik kararlılık | **GEÇTİ** | koli hareketi < 1e-7 mm |
| S6 Bırakma toleransı | **GEÇTİ** | ±2.5 cm'ye kadar başarılı, ±3 cm beklendiği gibi başarısız |
| S7 Alma | **GEÇTİ** | bağıl kayma 0.00 mm |
| S8 Performans | **GEÇTİ** | RTF 1.001, kameralar 30 Hz |

S4/S5 D-56 ile kaldırıldı (S5'in yerine C4). Ham sonuç: `docs/station_test_results.md`.

Bringup değiştiği için (`transit_method` argümanı) T testleri de koşuldu:

| Test | Sonuç |
|---|---|
| T1, T3–T14 (13 test) | **13/13 GEÇTİ** (T14 RTF 1.000; T8 yük alma senaryosu dahil) |
| T2 `colcon build --symlink-install` | **ÇALIŞTIRILAMADI**: betik bütün çalışma alanını yerinde symlink ile derliyor. Bu çalışma alanı symlink'siz derleniyor; bu oturumun başında aynı bayrak CMake önbelleğini bozdu ve 7 paketin `build/` dizini silinip yeniden derlenmek zorunda kalındı. Yerine: değişen 5 paket normal derlemede hatasız ve uyarısız (yalnız colcon'un "underlay" bilgi uyarısı) |

`docs/test_results.md` yalnız koşulan testlerle yeniden üretildi; T2 satırı o dosyada yok.

### F4.7 Açık noktalar ve öneriler (K-4)

| Konu | Durum | Öneri |
|---|---|---|
| Uç bölmenin hedef çifti bazı pozlardan hiç algılanmıyor | Otonomi yeniden konumlanarak tolere ediyor (A1'de 2, A2'de 1 deneme) | Algılayıcı tarafında incelenmeli (C4'teki %7 kayıpla aynı aile); F5'ten sonra ya da yol zorlaştırmayla birlikte |
| Nötr (kaynak) tabelası açıya göre açık-kapalı algılanıyor | Otonomi sık bakışla tolere ediyor | Algılayıcıda koyu blok yolu (D-66 yol A) eğik bakış için incelenmeli; gürültülü render'da (σ > 0) sorun muhtemelen azalır |
| Algılama düğümü çökmesi | Düzeltildi; test düzeneği düğümlerin yaşadığını denetlemiyordu | Kabul testleri düğüm çıkışını FAULT olarak raporlasın (F5) |
| Yol zorlaştırma (K-2'de "F4 sonunda") | F4 düz şeritlerle bitti; viraj, silik bant ve gölge plan §1'de dilim 1 dışında | Önerim: önce F5 (tam görev, hata enjeksiyonu, sürüm), sonra ayrı bir dilimde yol zorlaştırma. Şerit mesajı çoklu çizgi olduğu için arayüz hazır; değişecek yer `LaneTracker` (doğru → eğri) |
| `transit_method` dünyaya göre seçiliyor | Launch argümanı, bringup factory'de `lane` | Gerçek sahada "hat yapılandırması" olarak kalabilir; ya da şerit görülürse şerit, yoksa tabela (otomatik) |

### F4.8 Nasıl çalıştırılır

```bash
colcon build --packages-select forklift_interfaces forklift_perception forklift_autonomy forklift_evaluation forklift_bringup
source install/setup.bash
python3 -m pytest -q src/forklift_autonomy/test src/forklift_perception/test src/forklift_evaluation/test   # 35 birim testi
ros2 run forklift_evaluation mission_acceptance --test a1 --world factory --trials full
ros2 run forklift_evaluation mission_acceptance --test a2 --world station_test --trials full
ros2 run forklift_evaluation mission_acceptance --test a3 --world factory --trials full   # + dönüş (bilgi)
ros2 run forklift_evaluation mission_acceptance --test a3 --world station_test --trials return
# iki koşu aynı anda: her birine ayrı GZ_PARTITION ve ROS_DOMAIN_ID
ros2 launch forklift_bringup bringup.launch.py world:=factory    # GUI'den Otonom → Görevi başlat (tam görev)
```

---

## F5 — Tam görev, hata ve kurtarma

### F5.1 Özet

- **A4 tam görev: GEÇTİ.** Robot merkezden başlıyor, 3 koliyi kaynaktan alıp mühür rengindeki istasyonun orta bölmesine bırakıyor ve kaynak boşalınca bitiyor. Görev bir kez de **operatör GUI'sinden** başlatıldı (e-stop bırakma, [Otonom], [Görevi başlat]).
- **A5 hata enjeksiyonu: 8/8 GEÇTİ.** E-stop, teleop, mod değişimi, hedefi örtme (geçici ve kalıcı), taşırken koliyi kaldırma, alma öncesi koliyi alma, yerleri değişmiş istasyonlar.
- **Sahip kararı (2026-10-24):** kabul testleri bundan sonra yalnız asıl dünya olan **factory**'de koşuluyor (plan §11). Aşağıdaki son sonuçların hepsi factory'de ve **tek bir kod sürümüyle** alındı; A1–A3 de bu sürümle yeniden koşuldu.
- **F5'te bulunan ve düzeltilen önemli hata:** taşırken yük denetimi eklenince fabrikada yanlış "yük kaybı" çıktı. Sebep algıda: spot ışık altında kolinin üst yüzü doyuyor, ton sarıya kayıyor (D-79). Düzeltmeden sonra yüklü 1.00 / boş 0.00 ayrım; C6'da yanlış pozitif 0.

### F5.2 A4 — tam görev (factory)

Ham sonuç: `docs/logs/cv/f5/a4_factory.json`, `a4gui_factory.json`. Görüntüler: `docs/images/cv/f5/a4_gui_factory_*.png`.

| Deneme | Başlangıç | Sonuç | Teslim (istasyon / bölme, ofset) | Süre |
|---|---|---|---|---|
| 1 | merkez, 0° | DONE | kırmızı/orta 2.1 mm, yeşil/orta 0.8 mm, mavi/orta 2.2 mm | 463 s |
| 2 | (−0.3, 0.4), 135° (kaynağa bakmıyor) | DONE | 2.2 / 0.8 / 1.6 mm | 350 s |
| 3 (GUI'den) | merkez, 0° | DONE; 4 GUI adımı ✓ | 2.0 / 0.6 / 3.3 mm | 470 s |

Ölçüt: 3/3 doğru istasyon, eğim < 2° (ölçülen 0.00°), müdahale yok, ≤ 8 dk (480 s). **GEÇTİ.**

**Süre payı dar:** 350–470 s. GUI'li deneme, 3 sim paralel koşarken (RTF 0.93) 470 s sürdü. Tek sim koşusunda RTF 1.00.

**A4'ün geçmişi:**

| Koşu | Sonuç | Sebep | Düzeltme |
|---|---|---|---|
| İlk tam görev | 3/3 doğru, 467 s ve > 480 s | Zamanın çoğu arama (118–137 s), transit (120–139 s) ve PRE_DOCK (95 s) | Görev içi tabela hafızası, transit 0.25 m/s, tek PRE_DOCK düzeltmesi, son gereksiz dönüşün kaldırılması (D-76) → 347–412 s |
| Hata tepkileri eklendikten sonra (factory) | 0/2, 70–113 s'de FAULT | Yük denetimi yanlış "yük kaybı" verdi (algı, D-79) | Parlak kraft sınıfı |
| **Son koşu** | **3/3 deneme, 9/9 koli** | | |

station_test (bilgi; sahip kararından önce, D-79'un ilk sürümüyle): A4 **1/2** (`a4_station_test.json`: 445 s geçti; ikinci deneme 480 s dolduğunda transitteydi), A5 8/8. Daha önceki bir koşuda A4 2/2 ve GUI'den 1/1 (412–441 s) vardı; o dosyaların üzerine yazıldı, yalnız konsol çıktısında kaldı. station_test'teki GUI koşusu sahip kararıyla yarıda kesildi, dosyaları silindi.

### F5.3 A5 — hata enjeksiyonu (factory)

Ham sonuç: `docs/logs/cv/f5/a5_factory.json`. Betik: `ros2 run forklift_evaluation fault_acceptance --world factory`.

| # | Enjeksiyon | Beklenen (plan §7.2) | Ölçülen | Sonuç |
|---|---|---|---|---|
| E1 | Yüklü transitte e-stop | ≤ 0.6 s'de durma; DURAKLADI; bırakınca kendiliğinden devam yok; Devam → biter | Durma **0.38 s**, DURAKLADI 0.17 s'de; bırakıldıktan sonra 3 s hareket yok; Devam → ön-yanaşmada DONE | **GEÇTİ** |
| E2 | Girişte 0.5 s teleop | DURAKLADI; Devam → yeniden gözlem; alma | DURAKLADI 0.43 s'de; OBSERVE'den yeniden; mavi koli çatalda | **GEÇTİ** |
| E3 | Hizalamada Manuel mod | DURAKLADI, robot durur; Otonom + Devam → bırakma | Durma 0.29 s; Manuel'de hareket yok; bırakma ofseti 3.1 mm | **GEÇTİ** |
| E4 | Hedefi 3 s örtme (görsel pano) | RECOVER, sonra alma | RECOVER → yeniden gözlem → alma | **GEÇTİ** |
| E5 | Hedefi kalıcı örtme | FAULT (kurtarma tükendi), robot durur | 2 RECOVER → FAULT; robot duruyor | **GEÇTİ** |
| E6 | Taşırken koliyi kaldırma | FAULT (yük kaybı), robot durur | Kaldırmadan **0.98 s** sonra FAULT; robot duruyor | **GEÇTİ** |
| E7 | Kaldırmadan önce koliyi bölmeden alma | Yeniden deneme | Boş kaldırma → yeniden gözlem → soldaki sıradaki (yeşil) alındı | **GEÇTİ** |
| E8 | Kırmızı ↔ mavi istasyonların yeri değişmiş (zemin şeritleri eski düzende) | Koliler renge göre doğru | 3/3 doğru istasyon (1.4 / 0.7 / 1.8 mm), 409 s | **GEÇTİ** |

**A5'in geçmişi:**

| Koşu | Sonuç | Sebep | Düzeltme |
|---|---|---|---|
| İlk koşu (iki dünya) | 6/8 | E6: `transit` görevi çatalda koli olduğunu bilmiyordu, yük izlenmiyordu; E8: test düzeneği (ikinci sim oturumunda aynı yürütücü) | Transitte yük ilk ölçümlerden anlaşılıyor; oturum başına yürütücü |
| İkinci koşu (factory) | 5/8 | E1, E8: yanlış "yük kaybı" (D-79). E6 "GEÇTİ" görünüyordu ama FAULT enjeksiyondan **önce** gelmişti; test bunu ayırmıyordu | D-79; E6 ölçütü enjeksiyonun yapıldığını ve FAULT'un sonra geldiğini şart koşuyor; "durdu mu" ölçümü yavaşlama bittikten sonra |
| **Son koşu** | **8/8** | | |

### F5.4 A1–A3 (factory, son kod sürümü)

| Test | Sonuç | Öne çıkan |
|---|---|---|
| A1 yanaşma + alma | **18/18 GEÇTİ** | yanal ≤ 2.5 mm, yaw ≤ 0.54°, taşıyıcı önü −12.6…−10.7 mm, 2 denemede REPOSITION |
| A2 yüklü yanaşma + bırakma | **18/18 GEÇTİ** | ofset ≤ 4.9 mm, eğim 0.00°, taban 50.0 mm, doğru bölme 18/18 |
| A3 transit | **9/9 GEÇTİ** | ön-yanaşma ≤ 29 mm / 0.30°, şerit RMS ≤ 3.9 mm, kayma ≤ 0.29 mm |
| Bilgi: dönüş (hedef → kaynak) | 3/3 | ≤ 43 mm / 0.8°; ikisi sarı şeridi izledi (RMS 6.8 / 1.8 mm), biri tabelayla |

Ham sonuç: `docs/logs/cv/f3/a1_factory.json`, `docs/logs/cv/f4/a2_factory.json`, `a3_factory.json`. Bu factory dosyaları artık F5 koşusunu içeriyor; §F3 ve §F4'teki factory tabloları o fazların koduyla alınmıştı. station_test dosyaları F4 dönemindedir.

### F5.5 Yapılanlar

| Bileşen | Ne | Karar |
|---|---|---|
| Hızlandırma | Görev içi tabela hafızası, transit hızı, tek PRE_DOCK düzeltmesi, son dönüşün kaldırılması | D-76 |
| Hata tepkileri | RECOVER, ABORT, yük kaybı, alma yeniden denemesi, yanlış istasyon, tabelaya gitmeyen şerit | D-77 |
| GUI'den başlatma testi | Yalıtılmış Xephyr :99 (96 dpi), yalnız o ekrana fare olayları | D-78 |
| Yük algısı | Parlak / ton kaymış kraft | D-79 |
| Algılayıcı yuvası (R4) | `detectors.yaml → signs/parcels.backend: classical`; uygulanmamış `onnx` açık hata verir. F2'de yazılmamıştı, F5'te eklendi | — |
| Belgeler | ARCHITECTURE §10, README, DECISIONS D-71…D-79 | — |

### F5.6 Birim testleri

| Paket | Test | Sonuç |
|---|---|---|
| `forklift_autonomy` | 31 (F4'tekilere ek: kurtarma, kalıcı örtme, yük kaybı, alma yeniden denemesi, çatal takılması, yanlış istasyon ×2, yerleri değişmiş istasyonlar ×2, tabelaya gitmeyen şerit, transitte yük, hafıza, iki bitiş kipi) | **31/31 GEÇTİ** |
| `forklift_perception` | 12 (+ algılayıcı yuvası) | **12/12 GEÇTİ** |
| `forklift_evaluation` | 5 | **5/5 GEÇTİ** |

Her yeni kinematik testin, düzeltme öncesi koda karşı **kaldığı** denetlendi (yanlış istasyon testinin ilk iki sürümü senaryoyu hiç oluşturmuyordu; düzeltildi).

### F5.7 Gerileme

- C6 (yük, kayıtlı kümeler): yüksüz 1822/1822, yüklü 780/780; D-79'dan sonra değişmedi.
- GUI: F5'te GUI kodu değişmedi (164/164, F4).
- S ve T testleri: F5'te simülasyon ve bringup değişmedi; F4'teki sonuçlar geçerli (S 6/6, T 13/13). Koşulmadı.

### F5.8 Açık noktalar (K-5)

| Konu | Durum | Öneri |
|---|---|---|
| A4 süre payı | 350–470 s (sınır 480); yük altındaki makinede sınıra yakın | Bir sonraki dilimde transit adımlarının birleştirilmesi (dur-bak sayısı) |
| Uç bölmenin hedef çifti bazı pozlardan algılanmıyor | REPOSITION ile tolere ediliyor | Algılayıcıda incelenmeli (C4 ailesi) |
| Nötr tabela açıya göre açık-kapalı | Sık bakışla tolere ediliyor | Gürültülü render ve algılayıcı iyileştirmesi |
| Renk eşikleri ışığa duyarlı | Yük algısında doyma ve ton kayması bulundu (D-79) | Gerçek kamerada kalibrasyon aracı (R7); öğrenmeli yük ve koli algılayıcısı sonraki dilimde |
| Yol zorlaştırma | Dilim 1 dışında | Sonraki dilim (README Roadmap) |
| K-5 | — | 1) `feature/cv-autonomy` → `main` birleştirme, 2) push (ayrı onay), 3) `v0.4.0` tag + `release/v0.4.0` |

### F5.9 Nasıl çalıştırılır

```bash
colcon build --packages-select forklift_interfaces forklift_perception forklift_autonomy forklift_evaluation forklift_bringup
source install/setup.bash
ros2 run forklift_evaluation mission_acceptance --test a4 --world factory --trials full   # tam görev (~15 dk)
ros2 run forklift_evaluation mission_acceptance --test a4 --world factory --trials gui    # GUI'den başlatma (Xephyr :99)
ros2 run forklift_evaluation fault_acceptance --world factory                             # E1–E8 (~15 dk)
```

