# İlk CV + Otonomi Dilimi — Plan (v0.4.0)

- **Durum:** Sahip onayladı (2026-10-21). Yapılacak eklemeler ve çıkarmalar §11'deki değişiklik günlüğüne yazılır.
- **Dal:** `feature/cv-autonomy`. Dilim baştan sona çalışınca main'e alınır, `v0.4.0` tag'i ve `release/v0.4.0` dalı açılır.
- **İlgili belgeler:** DECISIONS D-42, D-46, D-53…D-56 (yeni kararlar D-57'den başlar); REPORT_STATIONS §10 ve §13; REPORT_FACTORY.
- **Sonuçlar:** Her fazın sonuçları `REPORT_CV.md`'ye yazılır.

---

## 1. Amaç ve kapsam

**Dilim 1'in sonunda robot şunu yapabilmeli:** Kaynak istasyondaki 3 koliyi tek tek almalı, her birini mühür rengine göre doğru hedef istasyonun boş bir bölmesine bırakmalı ve görevi bitirmeli. Bu, iki dünyada da (`station_test` ve `factory`), yan bölmeler dahil ve elle müdahale olmadan yapılmalı. Görev operatör GUI'sinden başlatılır (§5).

**Projenin üst amacı (sahip, 2026-10-21):** CV ve otonomi kodu, gerçek verilerle model eğitildiğinde büyük değişiklik yapmadan gerçek robota taşınabilmeli. Bu yüzden robot dünyayı **kameradan algılar**. Gazebo'nun verdiği konum ve nesne bilgisi yalnız değerlendirmede kullanılır. Kurallar §2'de.

| Alt dilim | İçerik | Robot hareket ediyor mu? | Neden bu sırada |
|---|---|---|---|
| **1A Algılama** | 5 algılayıcı, değerlendirme düzeneği, GT izdüşüm kontrolü | Hayır (`set_pose` ile poz ızgarası) | Kapalı döngüden önce algı hatası ölçülmeli. Statik testler deterministik ve ucuz; GT izdüşümü kamera modeli hatasını erken yakalar |
| **1B Yanaşma + alma** | Ön-yanaşma → bölme hizalama → giriş → kaldırma → geri çıkış | Evet (ön-yanaşma pozundan) | Robot zaten kaynağın önünde; yük yok; hedefler 0.12 m'ye kadar görünüyor (G4) |
| **1C Taşıma + bırakma** | Transit + yüklü yanaşma + indirme | Evet | 1B'nin yanaşma kodunu kullanır; 0.44 m'lik kör yolu ekler |
| **1D Görev + hatalar** | 3 koliyle tam döngü, hata ve kurtarma, GUI mod paneli, iki dünya, sürüm | Evet | Bütün parçaları birleştirir |

1B ve 1C ön-yanaşma pozundan (`set_pose`) başlar; böylece transitten bağımsız test edilir. Önce orta bölme, sonra yan bölmeler (±0.30 m yanal kaydırma).

**Kapsam dışı (sonraki dilimler):** viraj, kavşak, silik bant, gölge; öğrenmeli model eğitimi; aşamalı kamera gürültüsü; gz gürültüsünün neden uygulanmadığı.

---

## 2. Gerçeğe geçiş kuralları (sim-to-real)

| # | Kural | Gerçekteki karşılığı | Nasıl denetlenir |
|---|---|---|---|
| R1 | Algılama ve otonomi yalnız şunları kullanır: kamera görüntüsü + `camera_info`, URDF'ten TF (kalibrasyon), tekerlek odometrisi `/odom`, çatal eklemi `/joint_states`, `/e_stop`, operatör komutları | Kamera, kalibrasyon dosyası, tekerlek enkoderi, kaldırma enkoderi | C0: çalışma anında abone olunan topic listesi bu kümeyle sınırlı |
| R2 | **Yasak:** gz pozları (`dynamic_pose`, `pose/info`), gz servisleri, gz-transport importu, `stations.yaml`, `markers.yaml`, `factory_lanes.yaml`, `factory.yaml`, model adları, gz segmentasyon/bbox kameraları, GT etiketleri | Gerçekte yok | C0: statik import ve dosya adı taraması |
| R3 | Topic adları, frame'ler ve kalibrasyon parametreden gelir. Algılama `camera_info.D` ile bozulma düzeltmesi yapar (simde D = 0; kod yolu yapay bozulmayla test edilir) | Gerçek mercekte bozulma var | Birim testi |
| R4 | Her algılayıcının bir sözleşmesi var: görüntü + kamera modeli → tipli sonuç. Klasik ve öğrenmeli uygulamalar parametreyle değişir (`backend: classical \| onnx`). Eşikler kodda değil, yapılandırmada | Gerçek veride eğitilen model aynı yuvaya takılır | Arayüz testi |
| R5 | Etiketler standart biçimde (COCO / YOLO). Simdeki otomatik etiketler ile gerçek verideki elle etiketler aynı biçimde; eğitim ve değerlendirme betikleri ikisini de okur | Gerçek veri seti doğrudan girer | Biçim doğrulayıcı |
| R6 | Zaman mesaj damgasından alınır; sabit 30 Hz varsayılmaz, kare kaybı tolere edilir; `use_sim_time` yalnız bir parametredir | Gerçek kamera hızı değişken | Kare atlatma testi |
| R7 | `station_params.yaml` "katalog" bilgisidir: ölçü, şekil, renk tanımı var, konum ve doluluk yok. Renk eşikleri kameraya göre kalibre edilir (kalibrasyon aracıyla) | İstasyonun tasarım çizimi | — |
| R8 | Yalnız simde anlamlı yardımcılar (gürültü aktarıcısı, GT, otomatik etiketleme, `set_pose`) yalnız `forklift_evaluation` paketindedir | Gerçekte bu paket kurulmaz | Paket bağımlılıkları |
| R9 | Robota komut yalnız `/cmd_vel/auto` (Twist) ve `/fork/cmd_height` (Float64) ile gider. Gerçek sürücü aynı arayüzü sunmalı; güvenlik zinciri değişmez | Donanım sürücüsü | — |

**Odometri hakkında:** gz DiffDrive odometrisi tekerlek eklemlerinin dönüşünden hesaplanır; gerçek tekerlek enkoderinin karşılığıdır. Yalnız kısa göreli manevralarda kullanılır (≤ 1 m düz gitme, dönüşler). Harita ya da dünya koordinatı yok. Hesabın eklemlerden yapıldığı Faz 1'de doğrulanır.

---

## 3. Tasarımı belirleyen geometri

Bunlar plan aşamasında hesaplandı; Faz 1'de ölçülür. Ön kamera f ≈ 554 px, şerit kamerası f ≈ 381 px.

| # | Bulgu | Sonucu |
|---|---|---|
| G1 | Şerit kamerası zemini base_link'in 0.22–2.47 m önünde görüyor; 0.22 m'nin altı kör | Şerit ucu kameradan çıktıktan sonraki yol odometriyle gidilir |
| G2 | Çatallar şerit görüntüsünün alt-orta kısmında, yaklaşık son 127 satırda | Çatal maskesi `forklift_params` + TF'ten türetilir |
| G3 | Taşınan koli şerit görüntüsünde ~260. satırın altını, ortada ±133 px kapatıyor; şerit yalnız ≥ 0.59 m'de görülüyor | Taşırken şerit takibi görüntünün üst yarısıyla yapılır |
| G4 | Yüklüyken daire hedefler kamera çıkıntıya < 0.47 m yaklaşınca kayboluyor. Tam girişe ≈ 0.44 m kör yol kalıyor; yüksüz bu yol ≈ 9 cm | Kayıp anında yaw ≤ 0.5° hedefi (1° → 0.44 m'de ≈ 7.7 mm) |
| G5 | Tabela z 0.42–0.57'de, ön kenarın ≈ 0.32 m gerisinde; ön kamera tabelayı base_link ön kenara 0.59 m'den yaklaşınca kaybediyor | Renk doğrulaması 0.8 m'deki ön-yanaşmada yapılır |
| G6 | `/e_stop` 0.5 s susarsa gate sıfır basıyor; GUI e-stop basılı başlıyor | Testler 10 Hz `/e_stop=false` yayınlar; GUI'de operatör e-stop'u bırakır |
| G7 | Önlükteki çevre bandı istasyon renginde ve yaklaşma yönüne dik; hücre tehlike bandı sarı | Yön ve süreklilik süzgeci |
| G8 | `/cv/detections` yalnız ön kamerada çiziliyor | Şerit kamerası için debug görüntü topic'i |

---

## 4. Mimari

### 4.1 Paketler

| Paket | Tür | Görev | GT okur mu? | Ne zaman |
|---|---|---|---|---|
| `forklift_interfaces` | ament_cmake (rosidl) | Mesajlar ve servisler | — | F1 |
| `forklift_perception` | ament_python | Saf Python çekirdek (rclpy yok; kamera modeli, algılayıcılar) + iki ince ROS düğümü | **Hayır** | F1 (kamera modeli), F2 |
| `forklift_autonomy` | ament_python | Durum makinesi + hareket ilkelleri | **Hayır** | F3 |
| `forklift_evaluation` | ament_python | GT, C*/A* testleri, çevrimdışı değerlendirme, gürültü aktarıcısı, otomatik etiketleme | Evet (yalnız burası) | F1 |
| `forklift_gui` (değişiklik) | mevcut | Mod paneli (§5) | — | F3 |

`forklift_gazebo/scripts/station_acceptance.py` içindeki gz yardımcıları (Gz, SimSession, Pose, RobotGeom, …) içe aktarılabilir bir modüle ayrılır. `SimSession` dünya ve `control` seçimi kazanır (K9, onaylı). S testleri yeniden koşulur.

### 4.2 Düğümler ve topic'ler

**`forklift_perception`**

| Düğüm | Girdi | Çıktı |
|---|---|---|
| `front_perception` | `/front_camera/image_raw` (ya da `_noisy`), `camera_info`, TF | `/perception/signs`, `/perception/parcels`, `/cv/detections` (`vision_msgs/Detection2DArray`, GUI), `/perception/debug/front` |
| `lane_perception` | `/lane_camera/image_raw`, `camera_info`, TF, `/joint_states` | `/perception/lanes`, `/perception/bay_targets`, `/perception/load`, `/perception/debug/lane` |

Tüm renkler her karede algılanır; hangi rengi kullanacağını otonomi seçer.

**`forklift_interfaces`** (`header` = görüntü damgası, frame = `base_footprint`, zemin düzlemi)

| Tür | Ad | Ana alanlar |
|---|---|---|
| msg | `StationSign(Array)` | `color`, `confidence`, köşeler (px), `Pose2D station_in_base`, `pose_valid` |
| msg | `Lane(Array)` | `color`, `kind` (FLOOR/BAY), `points[]` (çoklu çizgi), `lateral_offset`, `heading`, `end_visible`, `end_distance`, `stop_bar_distance`, `confidence` |
| msg | `BayTarget(Array)` | `Pose2D entry_in_base` (çiftin orta noktası, θ = bölme ekseni), `baseline_measured`, iki hedefin px'i, `confidence` |
| msg | `Parcel(Array)` | `BoundingBox2D`, `seal_color`, `position_in_base`, `confidence` |
| msg | `LoadState` | `loaded`, `kraft_fraction` |
| msg | `AutonomyState` | `mode` (MANUAL/AUTO), `state`, `detail`, `target_color`, `bay_lateral`, `fault`, `parcels_done` |
| srv | `SetMode` | istek `mode`; yanıt `success`, `message` |

Şerit düz doğru olarak değil, çoklu çizgi olarak yayınlanır. Böylece virajlar mesajı ve denetleyiciyi değiştirmez.

**`forklift_autonomy` → `mission_manager`**

| Yön | Topic / servis |
|---|---|
| Çıkış | `/cmd_vel/auto` (AUTO kipinde, sıfırlar dahil sürekli 20 Hz), `/fork/cmd_height` (değişince), `/autonomy/state` |
| Giriş | `/perception/*`, `/odom`, `/joint_states`, `/e_stop`, `/cmd_vel/teleop` |
| Servis | `/autonomy/set_mode` (`SetMode`), `/autonomy/start`, `/autonomy/pause`, `/autonomy/resume`, `/autonomy/cancel` (`std_srvs/Trigger`) |

**`forklift_evaluation`:** `image_noise` aktarıcısı, `cv_acceptance.py`, `mission_acceptance.py`, `offline_eval.py`, `autolabel.py`, `gt_overlay.py`.

### 4.3 Komut zinciri, e-stop, saat

- Twist_mux önceliği değişmez: teleop 100 > auto 50. Teleop görülünce otonomi **DURAKLADI** durumuna geçer.
- E-stop yükselen kenarı → DURAKLADI. Çatal için D-32'deki gibi tek "tut" komutu gönderilir. Bırakılınca kendiliğinden devam edilmez; "Devam" komutu beklenir.
- Algılama ve otonomi `use_sim_time: true`; güvenlik zinciri duvar saatinde kalır (D-25).

### 4.4 Parametreler (tek kaynak kuralı, D-29)

| Dosya | İçerik | Türetilen (dosyaya yazılmaz) |
|---|---|---|
| `forklift_perception/config/perception.yaml` | HSV aralıkları, en küçük alanlar, ROI'ler, eşleştirme toleransları, süzgeçler, `backend` | Hedef ölçüleri ve taban, şerit genişlikleri → `station_params` (katalog). Çatal maskesi → `forklift_params` + TF |
| `forklift_autonomy/config/autonomy.yaml` | Hızlar, kazançlar, ön-yanaşma 0.8 m, giriş boşluğu, taşıma yüksekliği 0.06, zaman aşımları, yeniden deneme sayısı, bölme politikası | Tam giriş pozu, çatal uzunluğu → `forklift_params` |

Hedef geometrisi `station_layout.py`'den alınır (D-43). Bu modül konum dosyalarını okumaz; yalnız katalog hesabını yapar.

---

## 5. GUI: Manuel / Otonom modları (sahip isteği, 2026-10-21)

| Öğe | Davranış |
|---|---|
| Mod tuşları **[Manuel] [Otonom]** | Mod `mission_manager`'da tutulur (tek kaynak). GUI `/autonomy/set_mode` çağırır, gösterdiği modu `/autonomy/state`'ten okur. Otonomi düğümü yoksa [Otonom] pasif olur ve "Otonomi çevrimdışı" yazar |
| Manuel | Otonomi `/cmd_vel/auto` ve çatala komut göndermez. WASD ve çatal paneli etkin (bugünkü davranış) |
| Otonom | Çatal paneli pasif (çatal çakışması olmasın). Görev düğmeleri görünür: **[Görevi başlat] [Duraklat] [Devam] [İptal]**. Durum satırında durum, hedef renk, bölme, hata ve tamamlanan koli sayısı gösterilir |
| Görevi başlat | `/autonomy/start`. E-stop basılıysa düğme pasiftir ve sebebi yazılır |
| Otonom kipte WASD | Operatör devralır: GUI Manuel moda geçer, otonomi DURAKLADI durumuna geçer ve görevi saklar. Twist_mux önceliği bunu zaten garanti ediyor; GUI bunu görünür kılıyor |
| E-stop | Otonomi DURAKLADI; e-stop bırakılınca [Devam] ile sürer (yanaşmanın ortasındaysa PRE_DOCK'tan yeniden algılar) |
| Otonom → Manuel (görev sırasında) | Bir kez sıfır komutu ve çatal tut; görev DURAKLADI olarak saklanır |
| Metinler | Türkçe, `qsTr()` içinde |
| Testler | Yeni GUI birim ve entegrasyon testleri (U/I numaraları sürer); mock adaptörde otonomi taklidi |

Arayüz (msg/srv) F1'de tanımlanır, panel F3'te otonomi düğümüyle birlikte yapılır.

---

## 6. Algılama

Menzil ve hassasiyet satırlarındaki **tahmin** değerleri plan hesabıdır; **ölçülmüş** olanlar D-55'ten.

| Aşama | Kamera | Yöntem | Çıktı | Menzil | Hassasiyet | Riskler |
|---|---|---|---|---|---|---|
| Tabela rengi | Ön | Beyaz 4:1 dikdörtgen konturu → soldaki kare blok → blok rengi HSV (nötr = düşük doygunluk + koyu) → 4 köşeyle PnP (0.60 × 0.15 m) | Renk + istasyonun base'e göre pozu | 0.6–3.5 m | Tahmin: yön ±0.3°, PnP 3 m'de ±3 cm, yaw ±3° | Kırmızı varil ve yangın tüpü; nötr blok gölgeye benziyor; açılı bakış |
| Zemin şeridi (factory) | Şerit | HSV maskesi → çatal/koli maskesi → zemine izdüşüm (K + TF) → çoklu çizgi + 2. derece uydurma; durma çizgisi = beyaz dik bant | Şerit (renge göre), ofset, yön, uç ve durma mesafesi | 0.22–2.4 m (yüklü 0.59–2.4) | Tahmin: ≤ 5 mm, ≤ 1° | Çevre bandı, tehlike bandı (G7); ileride gölge (H/S kullanılır, V değil) |
| Bölme şeridi | Şerit | Aynı boru hattı, plaka düzlemi z 0.003 | Bölme başına şerit | ≈ 2 m → kanal girişi | Tahmin: ≤ 5 mm | Kısa (0.26 m); kaynakta sarı |
| Daire hedef çifti | Şerit | D-55: plaka dörtgeni → desen denetimi → koyuluk ağırlık merkezi → z 0.025 düzlemiyle ışın kesişimi → eşleştirme | Bölme giriş pozu, mesafe, soldan sağa sıra | Yüksüz 0.12–2.2 m, yüklü 0.47–2.2 m | **Ölçülmüş** (0.45–1.2 m, σ ≤ 0.02): eksen ≤ 0.22 mm, yaw ≤ 0.20° | Gölge; sarı bölme etiketi (ön çizgi kuralı eler); yakında 45° bakış |
| Koli, mühür, doluluk | Ön | Dilim 1: klasik (kraft maskesi → kutu → ROI içinde doygun kare = mühür → izdüşüm → bölme çiftiyle eşleştirme). Arayüz öğrenmeli modele açık (R4) | Koli listesi, doluluk, mühür rengi | 0.8–3 m | Tahmin: doluluk ve renk %100, IoU ≥ 0.8 | Renkli zemin ve bant (koli ROI'si ayırır) |
| Yük denetimi | Şerit | Alt-orta ROI'de kraft oranı | `loaded` | — | — | Koli düşerse ilk sinyal |
| Bölme etiketi (son yanaşma) | Şerit + ön | Arka plakadaki 45 mm sarı kare (bölme ekseninde, z 0.258): kontur → merkez → z 0.258 düzlemiyle ışın kesişimi | Bölme girişine mesafe + yanal | Tam girişten ~1 m'ye (kolinin üstünden) | F2'de ölçülecek (C9) | Numara okunmaz, yalnız geometri; kolinin üst kenarına yakın |

**Veri seti:** `autolabel.py` gz GT pozlarını K + TF ile görüntüye izdüşürür ve COCO/YOLO bbox yazar. Kaynaklar: (a) poz ızgarası + rastgele koli yerleşimi, iki dünya; (b) GUI veri seti kaydı (D-36) + eşzamanlı GT poz kaydı, damgadan birleştirilir. Gerçek veride aynı biçimde elle etiketleme yapılır (R5). Öğrenmeli model eğitimi sonraki dilimde (venv'de PyTorch; RTX 5070 için CUDA 12.8; kütüphane ve lisans o zaman seçilir).

**Gürültü:** Kabul ölçütü σ = 0. σ = 0.007 ve 0.02 sonuçları bilgi olarak raporlanır (çevrimdışı küme + canlı `image_noise`).

---

## 7. Davranış

### 7.1 Durum makinesi (düz FSM)

```
IDLE ─start─► FIND_STATION(renk) ─► TRANSIT ─► PRE_DOCK ─► SELECT_BAY ─► ALIGN_BAY ─► ENTER ─┬─► LIFT  ─► BACK_OUT ─► FIND_STATION(mühür rengi) …
                                                                                             └─► LOWER ─► BACK_OUT ─► FIND_STATION(kaynak) | DONE
Her durumdan: e-stop / teleop / Manuel mod / Duraklat ─► PAUSED ─Devam─► önceki durum (ALIGN/ENTER'dan ise PRE_DOCK)
Her durumdan: hata ─► RECOVER ─(N deneme)─► FAULT (dur, çatalı tut, operatörü bekle)
```

| Durum | İş | Çıkış |
|---|---|---|
| FIND_STATION | Yerinde dönerek ön kamerada istenen renkte tabela ara | Tabela ortada ve PnP geçerli |
| TRANSIT | station_test: tabela PnP'si → istasyon normali üzerinde 0.8 m'deki noktaya dön-git-dön (tabela göründükçe güncelle). factory: renkli şeridi takip et (pure pursuit); kavşakta istenen renk görünene kadar dön | Ön-yanaşma ±10 cm / ±5° |
| PRE_DOCK / SELECT_BAY | Tabela rengini doğrula; bölme çiftlerini ve doluluğu oku. Alma: soldan sağa ilk dolu bölme. Bırakma: önce orta boş bölme | Hedef bölmenin giriş pozu |
| ALIGN_BAY | Bölme ekseninde 0.6–0.8 m'ye gel; çatal q = 0 (alma) / 0.06 (bırakma) | Yanal ≤ 5 mm, yaw ≤ 0.5° |
| ENTER | Hizalama gölgesiz mesafede (yüklüyken kamera → ön çizgi ≥ 0.575 m) hedef çiftiyle tamamlanır, sonra yavaş yanaşma. Hedefler kaybolunca mesafe ve yanal konum **CV ile** arka plakadaki bölme etiketinden kestirilir (odometri yalnız kareler arası); durma ve indirme kararı bu kestirimle verilir (taşıyıcı önü x = −0.015 ± 0.01) | CV mesafesi hedefte |
| LIFT / LOWER | 0 ↔ 0.06; eklem hatası büyürse dur | Eklem ±2 mm; LIFT'te `loaded` |
| BACK_OUT | 0.5 m geri (odometri) | Mesafe |

### 7.2 Hata ve kurtarma

| Hata | Nasıl algılanır | Tepki |
|---|---|---|
| Hedef kayıp | Görünürlük yok, süre > T | Dur → 1 s bekle → 0.1 m geri → yeniden ara; N denemeden sonra FAULT |
| Yanlış istasyon | PRE_DOCK'ta tabela ≠ beklenen | İptal, geri çık, tabela aramasına geç (o istasyonu dışla); bulunamazsa FAULT |
| Koli düştü | Taşırken `loaded = false` | Dur, çatalı tut, FAULT("yük kaybı") |
| Alma başarısız | LIFT sonunda yük yok | İndir, geri çık, 1 kez yeniden dene → FAULT |
| Hedef bölme dolu | SELECT_BAY | Sıradaki boş bölme; boş yoksa FAULT |
| Kaynak boş | SELECT_BAY | DONE |
| Zaman aşımı | Durum başına süre (`autonomy.yaml`) | RECOVER → FAULT |
| Çatal takıldı | Eklem hatası büyüyor | Taşıma yüksekliğine çık, geri çık, FAULT |

---

## 8. Değerlendirme

GT yalnız `forklift_evaluation` içinde kullanılır. Kaynaklar: gz `dynamic_pose/info`, `markers.yaml`, `factory_lanes.yaml`, `stations.yaml` + `station_layout`.

| Çıktı | Metrikler |
|---|---|
| Tabela | Tespit oranı (mesafe ve açıya göre), renk doğruluğu, yanlış pozitif, PnP konum ve yaw hatası |
| Şeritler | Yanal (mm), yön (°), tespit oranı, yanlış renk, durma çizgisi mesafesi |
| Daire çifti | Eksen (mm), yaw (°), mesafe (mm), eşleştirme, yanlış çift, sıra |
| Koli | Doluluk karışıklık matrisi, mühür rengi, IoU, konum |
| Kapalı döngü | Girişte yanal/yaw (GT), bırakılan kolinin ofseti, eğim, taban z, komşu kolilerin hareketi, kayma, başarı, süre, kurtarma sayısı |

| # | Test | Koşul | Kriter (σ = 0) | Dünya |
|---|---|---|---|---|
| C0 | GT sızıntısı ve R1/R2 | Statik tarama + çalışma anındaki topic listesi | İzinli küme dışı yok | — |
| C1 | Tabela | 1–3.5 m, yön ±25°, 4 istasyon | Renk %100, yanlış pozitif 0; ≤ 3 m'de PnP ≤ 5 cm / ≤ 5° | İkisi |
| C2 | Zemin şeridi | Şerit boyunca, ±10 cm, ±15°, yüklü/yüksüz | ≤ 1 cm, ≤ 2°, tespit ≥ %95, yanlış renk 0 | factory |
| C3 | Bölme şeridi | 0.5–2 m, 12 bölme | ≤ 1 cm, ≤ 2° | İkisi |
| C4 | Hizalama (S5'in yerine) | Yüksüz 0.3–1.2 m, yüklü 0.5–1.2 m; ±5 cm, ±10° | Eksen ≤ 2 mm, yaw ≤ 0.5°, mesafe ≤ 5 mm; eşleştirme %100, yanlış çift 0, sıra doğru | İkisi |
| C5 | Koli, mühür, doluluk | 0.8–2.5 m, rastgele doluluk | %100, IoU ≥ 0.7, yanlış pozitif 0 | İkisi |
| C6 | Yük denetimi | Yüklü/yüksüz, q 0–0.06 | %100 | İkisi |
| C7 | Gürültü taraması | C1–C6, σ 0.007 / 0.02 | Bilgi | İkisi |
| C9 | Son yanaşma mesafesi (bölme etiketi) | Yüklü/yüksüz, ön çizgiye 0.14–0.60 m, yanal ±1.5 cm | Mesafe ≤ 5 mm, yanal ≤ 3 mm; tam girişe kadar her karede ölçüm | İkisi |
| C8 | Performans | Algılama açık | RTF ≥ 0.9, düğüm başına ≥ 15 Hz | İkisi |
| A1 | Yanaşma + alma | Ön-yanaşma ± (15 cm, 10°, 0.8–1.5 m), 3 bölme | Girişte ≤ 1 cm, ≤ 1°; alma başarılı; komşular < 1 mm | İkisi |
| A2 | Yüklü yanaşma + bırakma | Aynı ızgara, 3 hedef | Ofset ≤ 1.5 cm, eğim < 2°, taban 0.05 ± 2 mm | İkisi |
| A3 | Transit | Kaynak → her hedef | Ön-yanaşma ±10 cm / ±5°; şerit sapması RMS ≤ 2 cm; kayma < 1 cm | İkisi |
| A4 | Tam görev (GUI'den başlatma dahil) | 3 koli | 3/3 doğru, eğim < 2°, müdahale yok, ≤ 8 dk | İkisi |
| A5 | Hata enjeksiyonu | E-stop, teleop devralma, mod değişimi, hedefi örtme, koliyi kaldırma, yerleri değişmiş istasyonlar (`stations_file`) | §7.2'deki tepki; e-stop'ta ≤ 0.6 s'de durma | İkisi |

**Gerileme:** S1–S3 ve S6–S8 her fazda; T1–T14 bringup değişirse; GUI U/I testleri GUI değişince. S tablosunda S5 için "→ C4" notu.

---

## 9. Fazlar ve onay kapıları

| Faz | İş | Çıktı | Kapı |
|---|---|---|---|
| F0 | Plan | Bu belge | Onaylandı (2026-10-21) |
| **F1** Altyapı | `forklift_interfaces`; kamera modeli (perception çekirdeği); gz yardımcılarının modüle ayrılması + dünya/`control` seçimi; S gerilemesi; `forklift_evaluation` (GT, izdüşüm, kayıt); GT izdüşüm görüntüleri (iki dünya); G1–G5 ölçümü; odometrinin kaynağı; çevrimdışı küme; rclpy + gz aynı süreç denemesi; CPU bütçesi | Örtüşme görüntüleri, ölçülen sınırlar, küme özeti, REPORT_CV §F1 | **K-1** |
| F2 Algılama | Ön kamera, sonra şerit kamerası; `/cv/detections`; gürültü aktarıcısı; C0–C8 | Tablolar, GUI kaplama görüntüsü, gürültü bilgisi | **K-2** (yol zorlaştırmanın zamanı da) |
| F3 Alma + GUI modu | Hareket ilkelleri, FSM'nin yanaşma ve alma kısmı, GUI mod paneli; A1 | Tablolar, görüntüler, GUI testleri | **K-3** |
| F4 Bırakma + transit | A2, A3 | Tablolar, görüntüler | **K-4** |
| F5 Görev | A4, A5; REPORT_CV, ARCHITECTURE, DECISIONS, README | Rapor | **K-5:** main'e alma → push (ayrı onay) → `v0.4.0` |

---

## 10. Riskler ve kararlar

### 10.1 Riskler

| Risk | Önlem |
|---|---|
| Bırakmada 0.44 m kör yol | Kayıp anında yaw ≤ 0.5°; A2 ölçer; gerekirse ön kamerayla takip |
| G1–G5 hesapları yanlış çıkabilir | F1'de ölçülüyor |
| rclpy + gz-transport aynı süreçte takılabilir | F1 denemesi; olmazsa iki süreç |
| Gürültüsüz render iyimser sonuç verir | C7; aşamalı gürültü |
| Hedef plakasında gölge | C4 iki dünyada |
| Yüklü dönüşte koli kayması | A3; açısal ivme sınırı |
| Taşıyıcının çıkıntıya çarpması | −1.5 cm giriş boşluğu; son hız 0.05 m/s |
| Önlük eğimi 0.4° | Bilgi (≈ 3 mm izdüşüm hatası) |
| Python'un iki kamerayı 30 Hz'de işlemesi | C8; ROI veya kare atlama |
| Sim renklerine göre ayarlanmış eşikler gerçekte tutmaz | R4/R7: eşikler yapılandırmada, kalibrasyon aracı; öğrenmeli yuva |

### 10.2 Kararlar (sahip, 2026-10-21: hepsi önerildiği gibi onaylandı)

| # | Karar |
|---|---|
| K1 | v0.4.0 = 3 koli, iki dünya, yan bölmeler dahil |
| K2 | Robot `station_params` (katalog) ve `forklift_params` okur; konum dosyaları ve gz pozları yasak (§2) |
| K3 | Kısa göreli manevralarda tekerlek odometrisi; harita yok |
| K4 | station_test'te transit tabela PnP'siyle; station_test sade kalır |
| K5 | Koli algılama dilim 1'de klasik; öğrenmeli model sonraki dilimde (yuva hazır, R4) |
| K6 | Düz FSM |
| K7 | E-stop sonrası açık "Devam" komutu |
| K8 | Teleop → DURAKLADI; otonom kipte GUI çatal paneli pasif (§5) |
| K9 | `forklift_gazebo` test yardımcıları modüle ayrılır; dünya/`control` seçimi |
| K10 | Alma soldan sağa; bırakmada önce orta boş bölme |
| K11 | Algılama ve otonomi sim saatinde; güvenlik duvar saatinde |
| K12 | Test adları C0–C8, A1–A5; S5'in yerine C4 |
| K13 | **Değişti:** otonomi GUI'den başlatılır, GUI'de Manuel/Otonom modları var (§5). Servis ve `autostart` argümanı testler için kalır |
| K14 | C testlerinde `hall.clutter: full` |

---

## 11. Değişiklik günlüğü

| Tarih | Değişiklik | Kimden |
|---|---|---|
| 2026-10-21 | İlk sürüm; plan onaylandı | Sahip |
| 2026-10-21 | GUI Manuel/Otonom modları ve GUI'den başlatma eklendi (§5, K13) | Sahip |
| 2026-10-21 | Gerçeğe geçiş üst amacı ve R1–R9 kuralları eklendi (§2) | Sahip |
| 2026-10-22 | K-1: Sim kamerası gerçek kamera gibi dama tahtasıyla kalibre edilir, algılama `calibration_file` ile okur (D-59; ileride değişebilir) | Sahip |
| 2026-10-22 | K-1: Hizalama gölgesiz mesafede, sonra yanaşma; bırakma kararı CV mesafe kestirimiyle (bölme etiketi, C9; D-62) | Sahip |
| 2026-10-23 | K-2: Önerilen ölçütler kabul (C1 tabela rolü, C4 eşleştirme ≥ %90 ve yaw ≤ 0.6° (≤ 0.9 m), C9 ≤ 6 mm, şerit yönü ≥ 25 cm, C6 GT tanımı; D-70) | Sahip |
| 2026-10-23 | K-2: Gerçekte IMU gerekiyorsa simde de IMU: robota IMU eklendi, eğim IMU'dan (CV yöntemi yedek; D-67) | Sahip |
| 2026-10-23 | K-2: Yol zorlaştırma F4 sonunda (önerim; sahip "en uygun gördüğünü seç" dedi) | Sahip |
| 2026-10-23 | K-3: F3'ün kapısında beklenmedi; sahip F4'e geçilmesini istedi (F3'ün açık kalan A1 hataları F4'ün başında giderildi, REPORT_CV §F3) | Sahip |
| 2026-10-24 | K-4: Sahip commit'i ve F5'e geçişi onayladı. §F4.7'deki önerilere (yol zorlaştırmanın F5'ten sonra ayrı dilimde yapılması dahil) ayrıca yanıt verilmedi | Sahip |
