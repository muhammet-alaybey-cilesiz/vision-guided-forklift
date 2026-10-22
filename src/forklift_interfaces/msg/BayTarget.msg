# Bir bölmenin girişi. İki kaynaktan gelebilir:
#   hedef çifti (D-55): orta nokta kanal ekseninde, çiftin doğrultusu yaw'ı verir
#   bölme etiketi (D-62): arka plakada, bölme ekseninde; son yanaşmada mesafe ve yanal konum
uint8 SOURCE_TARGET_PAIR=0
uint8 SOURCE_BAY_LABEL=1

uint8 source
float32 confidence                    # 0–1
geometry_msgs/Pose2D entry_in_base    # bölme girişi (çıkıntı ön çizgisi x = 0, kanal ekseni); theta = bölme ekseni (+x istasyonun içine)
bool yaw_valid                        # theta ölçüldü mü (tek etiket: hayır, robot yaw'ı varsayılır)
float32 distance                      # base_footprint'ten ön çizgiye, bölme ekseni boyunca [m]
float32 baseline_measured             # çift: iki hedef arası [m] (katalog 0.216); etiket: 0
geometry_msgs/Point[2] targets_px     # çift: sol ve sağ hedef merkezi; etiket: [merkez, merkez] (x = u, y = v)
