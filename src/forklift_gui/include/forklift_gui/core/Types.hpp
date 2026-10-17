// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__TYPES_HPP_
#define FORKLIFT_GUI__CORE__TYPES_HPP_

#include <QMetaType>
#include <QString>
#include <QVector>

namespace forklift_gui
{

// Portlardan core'a akan değer tipleri. ROS mesaj tiplerinden bağımsızdır;
// dönüşüm yalnızca ROS adaptöründe yapılır.

// Odometri örneği (nav_msgs/Odometry'nin GUI'nin kullandığı alanları).
struct OdomSample
{
  double x{0.0};        // [m] odom çerçevesi
  double y{0.0};        // [m]
  double yaw{0.0};      // [rad]
  double linear{0.0};   // [m/s] gövde x hızı
  double angular{0.0};  // [rad/s] z ekseni etrafında
  double stamp{0.0};    // [s] başlık zamanı (sim zamanı)
};

// Eklem durumu örneği: çatal yüksekliği ve tekerlek hızları.
struct JointSample
{
  bool hasFork{false};     // çatal eklemi mesajda var mı
  double forkHeight{0.0};  // [m]
  bool hasWheels{false};   // iki tekerlek eklemi de mesajda var mı
  double leftWheel{0.0};   // [rad/s]
  double rightWheel{0.0};  // [rad/s]
  double stamp{0.0};       // [s]
};

// Tek tespit kutusu; görüntü pikseli cinsinden merkez ve boyut.
struct DetectionBox
{
  double cx{0.0};
  double cy{0.0};
  double width{0.0};
  double height{0.0};
  QString label;
  double score{0.0};
};

// Bir kameranın tek görüntüsüne ait tespitler.
struct DetectionSet
{
  QString cameraId;
  QVector<DetectionBox> boxes;
  double stamp{0.0};     // [s] görüntü başlık zamanı
  qint64 receivedNs{0};  // alım anı, monoton saat [ns]
};

// Kamera karesinin üst bilgisi; kare ayrıca QVideoFrame olarak taşınır.
struct FrameMeta
{
  double stamp{0.0};     // [s] başlık zamanı (sim)
  qint64 receivedNs{0};  // ROS thread'inde alım anı, monoton saat [ns]
  QString encoding;      // ROS kodlaması: rgb8, bgr8, ...
};

// /rosout seviyeleri; değerler rcl_interfaces/msg/Log sabitleriyle aynı.
enum class LogLevel : int
{
  Debug = 10,
  Info = 20,
  Warn = 30,
  Error = 40,
  Fatal = 50
};

struct LogEntry
{
  double stamp{0.0};  // [s] mesajın kendi zamanı
  qint64 wallMs{0};   // GUI'ye ulaşma anı, epoch [ms]; görüntüleme için
  int level{static_cast<int>(LogLevel::Info)};
  QString node;
  QString text;
  bool local{false};  // GUI'nin kendi olayı mı
};

// Bir topic'in alım sayacı; sağlık modeli bunlardan frekans ve yaş hesaplar.
struct TopicCounter
{
  QString key;  // sağlık anahtarı: "odom", "front_image", ...
  quint64 count{0};
  qint64 lastNs{-1};  // son alım, monoton saat [ns]; -1 = hiç alınmadı
};

// ROS grafiği özeti (TopBar'daki ROS LED'i).
struct GraphStatus
{
  bool up{false};              // düğüm ayakta mı
  int peerNodes{0};            // keşfedilen diğer düğüm sayısı
  int inputsWithPublisher{0};  // yayıncısı görünen girdi topic'i sayısı
  bool mock{false};            // sahte adaptör
};

}  // namespace forklift_gui

Q_DECLARE_METATYPE(forklift_gui::OdomSample)
Q_DECLARE_METATYPE(forklift_gui::JointSample)
Q_DECLARE_METATYPE(forklift_gui::DetectionSet)
Q_DECLARE_METATYPE(forklift_gui::FrameMeta)
Q_DECLARE_METATYPE(forklift_gui::LogEntry)
Q_DECLARE_METATYPE(forklift_gui::GraphStatus)

#endif  // FORKLIFT_GUI__CORE__TYPES_HPP_
