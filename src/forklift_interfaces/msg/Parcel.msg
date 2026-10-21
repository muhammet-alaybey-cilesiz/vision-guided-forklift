# Koli ve mühür (ön kamera).
float32 confidence                    # 0–1
vision_msgs/BoundingBox2D bbox        # görüntüde koli kutusu [px]
string seal_color                     # red | green | blue | unknown
float32 seal_confidence
bool position_valid
geometry_msgs/Point position_in_base  # koli ön yüzü alt orta noktası, base_footprint [m]
