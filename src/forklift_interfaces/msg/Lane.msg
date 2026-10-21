# Renkli şerit (şerit kamerası), zemin düzleminde. Düz doğru değil çoklu çizgi:
# viraj eklendiğinde mesaj değişmez.
uint8 KIND_FLOOR=0                    # zemin kılavuz şeridi (factory)
uint8 KIND_BAY=1                      # istasyon plakasındaki bölme şeridi

string color                          # yellow | red | green | blue
uint8 kind
float32 confidence                    # 0–1
geometry_msgs/Point[] points          # merkez çizgisi, yakından uzağa, base_footprint [m]
float32 lateral_offset                # robot ekseninden şeride yanal uzaklık, +y = sol [m]
float32 heading                       # şerit yönü − robot yönü [rad], + = sola dönük
bool end_visible                      # şeridin uzak ucu görüntüde mi
float32 end_distance                  # uzak uca x mesafesi [m] (end_visible ise)
bool stop_bar_visible                 # uçtaki beyaz durma çizgisi görünüyor mu
float32 stop_bar_distance             # durma çizgisine x mesafesi [m]
