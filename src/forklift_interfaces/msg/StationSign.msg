# İstasyon tabelası (ön kamera). Renk bloğu istasyonun kimliğidir (D-56).
string COLOR_NEUTRAL=neutral
string COLOR_RED=red
string COLOR_GREEN=green
string COLOR_BLUE=blue

string color                          # COLOR_* (kaynak istasyon: neutral)
float32 confidence                    # 0–1
geometry_msgs/Point[4] corners_px     # tabela dörtgeni (x = u, y = v; sol üst, sağ üst, sağ alt, sol alt)
bool pose_valid                       # PnP başarılı mı
geometry_msgs/Pose2D station_in_base  # istasyon çerçevesi (ön kenar ortası, +x istasyonun içine) base_footprint'te
