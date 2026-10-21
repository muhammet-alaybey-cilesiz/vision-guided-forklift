# Otonominin durumu (GUI mod paneli bunu gösterir; mod burada tutulur, tek kaynak).
uint8 MODE_MANUAL=0
uint8 MODE_AUTO=1

std_msgs/Header header
uint8 mode
string state                          # IDLE | FIND_STATION | TRANSIT | PRE_DOCK | ... | PAUSED | FAULT | DONE
string detail                         # insan okuyacak kısa açıklama (Türkçe)
string target_color                   # şu anki hedef istasyon rengi (yoksa boş)
float32 bay_lateral                   # seçilen bölmenin yanal konumu [m] (yoksa NaN)
string fault                          # FAULT'ta sebep, değilse boş
uint32 parcels_done
uint32 parcels_total
