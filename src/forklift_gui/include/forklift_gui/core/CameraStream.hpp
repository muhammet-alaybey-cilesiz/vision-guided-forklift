// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__CAMERASTREAM_HPP_
#define FORKLIFT_GUI__CORE__CAMERASTREAM_HPP_

#include <QtQmlIntegration/qqmlintegration.h>

#include <QObject>
#include <QPointer>
#include <QSize>
#include <QTimer>
#include <QVariantList>
#include <QVideoFrame>
#include <QVideoSink>
#include <deque>

#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/core/Types.hpp"

namespace forklift_gui
{

// Tek kameranın GUI tarafı: hazır kareyi QML VideoOutput'un QVideoSink'ine
// verir, FPS / çözünürlük / kare yaşı / STALE ve tespit kutularını tutar.
// Kare dönüşümü burada değil, adaptörün thread'inde yapılır (D-27, D-34).
class CameraStream : public QObject
{
  Q_OBJECT
  QML_ELEMENT
  QML_UNCREATABLE("Backend.cameras üzerinden erişilir")

  Q_PROPERTY(QString cameraId READ cameraId CONSTANT)
  Q_PROPERTY(QString label READ label CONSTANT)
  Q_PROPERTY(bool showsDetections READ showsDetections CONSTANT)
  Q_PROPERTY(QVideoSink * videoSink READ videoSink WRITE setVideoSink NOTIFY videoSinkChanged)
  Q_PROPERTY(bool hasFrame READ hasFrame NOTIFY statsChanged)
  Q_PROPERTY(double fps READ fps NOTIFY statsChanged)
  // qmllint 6.4 QSize özellik tipini tanımıyor; boyutlar int olarak açılır.
  Q_PROPERTY(int frameWidth READ frameWidth NOTIFY statsChanged)
  Q_PROPERTY(int frameHeight READ frameHeight NOTIFY statsChanged)
  Q_PROPERTY(int infoWidth READ infoWidth NOTIFY statsChanged)
  Q_PROPERTY(int infoHeight READ infoHeight NOTIFY statsChanged)
  Q_PROPERTY(double frameAge READ frameAge NOTIFY statsChanged)
  Q_PROPERTY(bool stale READ stale NOTIFY statsChanged)
  Q_PROPERTY(QVariantList detections READ detections NOTIFY detectionsChanged)
  Q_PROPERTY(bool detectionsVisible READ detectionsVisible NOTIFY detectionsChanged)

public:
  CameraStream(
    const CameraConfig & camera, const GuiConfig::Cameras & config, double statsRate, ClockFn clock,
    QObject * parent = nullptr);

  QString cameraId() const { return camera_.id; }
  QString label() const { return camera_.label; }
  bool showsDetections() const { return camera_.detections; }
  QVideoSink * videoSink() const { return sink_; }
  void setVideoSink(QVideoSink * sink);
  bool hasFrame() const { return hasFrame_; }
  double fps() const { return fps_; }
  QSize frameSize() const { return frameSize_; }
  QSize infoSize() const { return infoSize_; }
  int frameWidth() const { return frameSize_.width(); }
  int frameHeight() const { return frameSize_.height(); }
  int infoWidth() const { return infoSize_.width(); }
  int infoHeight() const { return infoSize_.height(); }
  double frameAge() const { return frameAge_; }
  bool stale() const { return stale_; }
  QVariantList detections() const { return detections_; }
  bool detectionsVisible() const { return detectionsVisible_; }

  void onFrame(const QVideoFrame & frame, const forklift_gui::FrameMeta & meta);
  void onCameraInfo(const QSize & resolution);
  void onDetections(const forklift_gui::DetectionSet & set);

  // FPS / yaş / STALE istatistikleri health.update_rate ile yenilenir;
  // kare kendisi geldiği anda sink'e verilir.
  // Son kare ve üst bilgisi (veri seti kaydı için).
  QVideoFrame latestFrame() const { return latest_; }
  FrameMeta latestMeta() const { return latestMeta_; }
  qint64 latestArrivalNs() const { return lastArrivalNs_; }
  quint64 framesReceived() const { return framesReceived_; }

  void tick(qint64 nowNs);
  void start();
  void stop();

signals:
  void videoSinkChanged();
  void statsChanged();
  void detectionsChanged();

private:
  CameraConfig camera_;
  GuiConfig::Cameras config_;
  ClockFn clock_;
  QTimer timer_;

  QPointer<QVideoSink> sink_;
  QVideoFrame latest_;
  FrameMeta latestMeta_;
  quint64 framesReceived_{0};
  std::deque<qint64> arrivals_;  // FPS penceresi: GUI'ye varış anları [ns]
  qint64 lastArrivalNs_{-1};

  bool hasFrame_{false};
  double fps_{0.0};
  QSize frameSize_;
  QSize infoSize_;
  double frameAge_{-1.0};
  bool stale_{true};

  DetectionSet lastDetections_;
  qint64 lastDetectionsNs_{-1};
  QVariantList detections_;
  bool detectionsVisible_{false};
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__CAMERASTREAM_HPP_
