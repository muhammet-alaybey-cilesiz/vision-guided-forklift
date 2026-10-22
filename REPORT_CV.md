# İlk CV + Otonomi Dilimi — Rapor

- **Plan:** `docs/PLAN_CV_AUTONOMY.md`
- **Kararlar:** DECISIONS D-57 ve sonrası
- **Dal:** `feature/cv-autonomy`

| Faz | Durum |
|---|---|
| F1 Altyapı ve ölçüm | Tamamlandı (2026-10-21), K-1 onaylandı (2026-10-22) |
| F2 Algılama | **Tamamlandı (2026-10-22), K-2 onayı bekleniyor** |
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
