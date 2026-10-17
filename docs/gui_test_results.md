# forklift_gui entegrasyon testleri

Son koşu: 2026-10-16 00:41:31 (`integration_test.py --tests I1,I10,I11,I12,I2,I3,I4,I5,I6,I7,I8,I9`)

| # | Test | Ölçülen | Kriter | Sonuç |
|---|---|---|---|---|
| I1 | Build + lint + qmllint | build çıkış 0, uyarı 0; testler: forklift_gui 14 test/0 hata/0 başarısız/0 atlandı, forklift_safety 5 test/0 hata/0 başarısız/0 atlandı, forklift_bringup 3 test/0 hata/0 başarısız/0 atlandı | colcon build uyarısız; clang-format, cpplint, qmllint ve tüm testler temiz | **GEÇTİ** |
| I2 | Sürüş: 0.2 m/s, 3 s (W basılı) | odom 0.602 m (komut tepe 0.200 m/s; tuş basılı 3.12 s, sıfır olmayan komut 3.08 s) | 0.6 m ± %10 (0.54–0.66 m) | **GEÇTİ** |
| I3 | Çatal preset "Taşıma" | 1.30 s içinde ±5 mm; son yükseklik 60.5 mm; GUI olayı: Çatal hedefe ulaştı: 60.0 mm | 3 s içinde 0.060 ± 0.005 m ve GUI "ulaşıldı" | **GEÇTİ** |
| I4 | Hareket halindeyken e-stop → /cmd_vel sıfır | /e_stop=true → /cmd_vel sıfır 0.0 ms (kilit önce); Space tuşundan 17.9 ms; öncesi v=-0.200 m/s; robot durdu: True | ≤ 100 ms | **GEÇTİ** |
| I5 | GUI çökmesi (kill -9) sırasında sürüş | durma 0.383 s (kapı sıfırı 0.163 s); öncesi v=-0.200 m/s | ≤ 0.6 s | **GEÇTİ** |
| I6 | İki kamera görüntüsü, GUI FPS | front: ort 29.2, min 27.1 FPS; lane: ort 28.4, min 22.2 FPS | iki görüntü de gösteriliyor, FPS ≥ 25 | **GEÇTİ** |
| I7 | Sim yok → var: otomatik bağlanma | başta "bekleniyor": True; sim başlatma → genel sağlık normal 4.1 s; ilk /clock → normal 1.6 s | GUI sim olmadan açılır, bekler; sim gelince sağlık yeşile döner | **GEÇTİ** |
| I8 | Kapanış | pencere kapatma: çıkış 0, 0.37 s, düzenli=True; SIGINT: çıkış 0, 0.26 s, düzenli=True; SIGTERM: çıkış 0, 0.28 s, düzenli=True | crash yok, süreç 2 s içinde biter | **GEÇTİ** |
| I9 | Tespit kaplaması (sahte /cv/detections) | ölçek x 1.317 / y 1.316 px/px (oran 0.999); genişlik oranı 1.000, 1.001; içerik başlangıcı (28, 349) px; docs/images/gui_detection_overlay.png | kutular görüntüyle aynı doğrusal eşlemede (en-boy korunur), ekran görüntüsü | **GEÇTİ** |
| I10 | Sürekli veri seti kaydı (10 s, 2 Hz) | 21 kare çifti ({'front': 21, 'lane': 21}), 21 metadata satırı, kayıt süresi 10.10 s (beklenen ≈ 20.2 + ilk kare) | ≈ 20 kare çifti ve 20 metadata satırı | **GEÇTİ** |
| I11 | Performans (iki kamera açık) | GUI CPU 18.7 % (tek çekirdek), RSS 279 MB; RTF GUI yok 1.000, GUI var 1.000 (düşüş -0.0 %) | CPU raporlanır; RTF düşüşü < %10 | **GEÇTİ** |
| I12 | Mock mod (ROS olmadan) | çıkış 0; ROS grafiğinde düğüm: False; /e_stop mesajı: 0; sahte kamera FPS: front 32.6, lane 32.6; docs/images/gui_mock.png | --mock ile ROS olmadan açılır, paneller sahte veriyle dolar; ekran görüntüsü | **GEÇTİ** |
