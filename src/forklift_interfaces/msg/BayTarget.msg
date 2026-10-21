# Bir bölmenin daire hedef çifti (D-55). Çiftin orta noktası kanal eksenindedir.
float32 confidence                    # 0–1
geometry_msgs/Pose2D entry_in_base    # orta nokta (çıkıntı ön çizgisi, kanal ekseni); theta = bölme ekseni (+x istasyonun içine)
float32 distance                      # base_footprint'ten ön çizgiye x mesafesi [m]
float32 baseline_measured             # iki hedef arası ölçülen taban [m] (katalog değeri 0.216)
geometry_msgs/Point[2] targets_px     # sol ve sağ hedefin görüntüdeki merkezi (x = u, y = v)
