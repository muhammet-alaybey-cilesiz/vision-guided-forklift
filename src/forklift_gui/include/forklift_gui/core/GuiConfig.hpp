// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__GUICONFIG_HPP_
#define FORKLIFT_GUI__CORE__GUICONFIG_HPP_

#include <QString>
#include <QStringList>
#include <QVector>

namespace forklift_gui
{

// gui_params.yaml'ın core'daki karşılığı. Değerlerin TEK kaynağı YAML'dır;
// buradaki sıfır ilk değerler yalnızca yer tutucudur ve yükleyici (ROS
// düğümü ya da --mock'ta YAML ayrıştırıcı) hepsini doldurur (DECISIONS D-29).

struct CameraConfig
{
  QString id;     // "front", "lane"; veri seti klasör adı da budur
  QString label;  // gösterilen ad (yapılandırma verisi)
  QString imageTopic;
  QString infoTopic;
  bool detections{false};  // DetectionOverlay bu kamerada mı
};

struct HealthTopicConfig
{
  QString key;               // health.ids öğesi
  QString topic;             // topics.* ya da cameras.* üzerinden çözülen ad
  double expectedRate{0.0};  // [Hz]; 0 = olay tabanlı, hız denetimi yok
  double warnAge{0.0};       // [s]; 0 = yaş denetimi yok
  double errorAge{0.0};      // [s]
  bool required{false};      // genel sağlığa katılır mı
};

struct GuiConfig
{
  struct Topics
  {
    QString odom;
    QString jointStates;
    QString clock;
    QString rosout;
    QString detections;
    QString cmdVel;
    QString forkCmd;
    QString eStop;
  } topics;

  struct Ros
  {
    int executorThreads{0};
  } ros;

  struct Cameras
  {
    QVector<CameraConfig> list;
    double staleAfter{0.0};       // [s]
    double fpsWindow{0.0};        // [s]
    double detectionMaxAge{0.0};  // [s]
  } cameras;

  struct Teleop
  {
    double publishRate{0.0};      // [Hz]
    double maxLinear{0.0};        // [m/s]
    double maxAngular{0.0};       // [rad/s]
    double maxLinearAccel{0.0};   // [m/s²]
    double maxAngularAccel{0.0};  // [rad/s²]
    double scaleMin{0.0};
    double scaleMax{0.0};
    double scaleStep{0.0};
    double scaleDefault{0.0};
  } teleop;

  struct Fork
  {
    QString jointName;
    double minHeight{0.0};         // [m]
    double maxHeight{0.0};         // [m]
    double keyStep{0.0};           // [m]
    double reachedTolerance{0.0};  // [m]
    double reachTimeout{0.0};      // [s]
    double resyncAfter{0.0};       // [s] bu kadar veri gelmezse yeniden bağlantı sayılır
    QStringList presetLabels;
    QVector<double> presetHeights;  // [m]
  } fork;

  struct Safety
  {
    double eStopRate{0.0};    // [Hz]
    double releaseHold{0.0};  // [s]
  } safety;

  struct Telemetry
  {
    double uiMaxRate{0.0};  // [Hz]
    double rtfWindow{0.0};  // [s]
    int trailMaxPoints{0};
    double trailMinStep{0.0};  // [m]
    QStringList wheelJoints;   // [sol, sağ] tekerlek eklem adları
  } telemetry;

  struct Health
  {
    double updateRate{0.0};  // [Hz]
    double rateWindow{0.0};  // [s] frekans ölçüm penceresi
    double warnRateRatio{0.0};
    QVector<HealthTopicConfig> topics;
  } health;

  struct Log
  {
    int maxEntries{0};
  } log;

  struct Dataset
  {
    QString rootDir;             // "~" ile başlayabilir; kullanımda açılır
    double continuousRate{0.0};  // [Hz]
  } dataset;

  struct Mock
  {
    int imageWidth{0};
    int imageHeight{0};
    double frameRate{0.0};        // [Hz]
    double odomRate{0.0};         // [Hz]
    double clockRate{0.0};        // [Hz]
    double linearAccel{0.0};      // [m/s²] sahte DiffDrive ivme sınırı
    double angularAccel{0.0};     // [rad/s²]
    double forkSpeed{0.0};        // [m/s]
    double wheelRadius{0.0};      // [m] sahte tekerlek hızı için
    double wheelSeparation{0.0};  // [m]
    double logPeriod{0.0};        // [s] sahte /rosout satırı aralığı
  } mock;

  // Değerler arası mantıksal tutarlılık; boş liste = geçerli.
  QStringList validate() const;

  // id ile kamera; yoksa nullptr.
  const CameraConfig * camera(const QString & id) const;
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__GUICONFIG_HPP_
