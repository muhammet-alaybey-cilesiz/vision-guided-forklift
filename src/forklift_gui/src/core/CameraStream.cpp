// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/CameraStream.hpp"

#include <QLoggingCategory>
#include <QVariantMap>
#include <cmath>
#include <utility>

namespace forklift_gui
{

// Kamera istatistikleri için tanı kategorisi; varsayılan kapalı (yalnız
// info ve üstü). Testler QT_LOGGING_RULES="forklift.gui.camera.debug=true"
// ile açar ve GUI'nin GÖSTERDİĞİ FPS'i okur (D-39, I6).
Q_LOGGING_CATEGORY(lcCamera, "forklift.gui.camera", QtInfoMsg)

CameraStream::CameraStream(
  const CameraConfig & camera, const GuiConfig::Cameras & config, double statsRate, ClockFn clock,
  QObject * parent)
: QObject(parent), camera_(camera), config_(config), clock_(std::move(clock))
{
  timer_.setInterval(static_cast<int>(std::lround(1000.0 / statsRate)));
  connect(&timer_, &QTimer::timeout, this, [this]() { tick(clock_()); });
}

void CameraStream::setVideoSink(QVideoSink * sink)
{
  if (sink_ == sink) {
    return;
  }
  sink_ = sink;
  if (sink_ && latest_.isValid()) {
    sink_->setVideoFrame(latest_);  // yeni görünüm boş kalmasın
  }
  emit videoSinkChanged();
}

void CameraStream::onFrame(const QVideoFrame & frame, const FrameMeta & meta)
{
  const qint64 now = clock_();
  latest_ = frame;
  latestMeta_ = meta;
  ++framesReceived_;
  lastArrivalNs_ = now;
  arrivals_.push_back(now);
  if (sink_) {
    // Hazır kare doğrudan VideoOutput'a; kopya yok (QVideoFrame paylaşımlı).
    sink_->setVideoFrame(frame);
  }
}

void CameraStream::onCameraInfo(const QSize & resolution) { infoSize_ = resolution; }

void CameraStream::onDetections(const DetectionSet & set)
{
  if (!camera_.detections) {
    return;
  }
  lastDetections_ = set;
  lastDetectionsNs_ = clock_();

  // Kutular görüntü boyutuna normalize edilir; QML yalnızca contentRect ile
  // çarpar (D-34). Boyut bilinmiyorsa (kare yok) kutu çizilemez.
  QSize size = latest_.isValid() ? latest_.size() : infoSize_;
  QVariantList boxes;
  if (size.width() > 0 && size.height() > 0) {
    const double w = size.width();
    const double h = size.height();
    for (const DetectionBox & b : set.boxes) {
      QVariantMap m;
      m.insert(QStringLiteral("x"), (b.cx - b.width / 2.0) / w);
      m.insert(QStringLiteral("y"), (b.cy - b.height / 2.0) / h);
      m.insert(QStringLiteral("w"), b.width / w);
      m.insert(QStringLiteral("h"), b.height / h);
      m.insert(QStringLiteral("label"), b.label);
      m.insert(QStringLiteral("score"), b.score);
      boxes.append(m);
    }
  }
  detections_ = boxes;
  detectionsVisible_ = !boxes.isEmpty();
  emit detectionsChanged();
}

void CameraStream::tick(qint64 nowNs)
{
  const qint64 windowNs = secondsToNs(config_.fpsWindow);
  while (!arrivals_.empty() && nowNs - arrivals_.front() > windowNs) {
    arrivals_.pop_front();
  }
  // FPS: penceredeki varışlar arasındaki aralıkların tersi.
  if (arrivals_.size() >= 2 && arrivals_.back() > arrivals_.front()) {
    fps_ =
      static_cast<double>(arrivals_.size() - 1) / nsToSeconds(arrivals_.back() - arrivals_.front());
  } else {
    fps_ = 0.0;
  }
  hasFrame_ = latest_.isValid();
  frameSize_ = hasFrame_ ? latest_.size() : QSize();
  frameAge_ = lastArrivalNs_ < 0 ? -1.0 : nsToSeconds(nowNs - lastArrivalNs_);
  stale_ = lastArrivalNs_ < 0 || frameAge_ > config_.staleAfter;
  emit statsChanged();
  qCDebug(lcCamera).noquote() << QStringLiteral("%1 fps=%2 age=%3 frames=%4 size=%5x%6")
                                   .arg(camera_.id)
                                   .arg(fps_, 0, 'f', 2)
                                   .arg(frameAge_, 0, 'f', 3)
                                   .arg(framesReceived_)
                                   .arg(frameSize_.width())
                                   .arg(frameSize_.height());

  if (detectionsVisible_ && nowNs - lastDetectionsNs_ > secondsToNs(config_.detectionMaxAge)) {
    detections_.clear();
    detectionsVisible_ = false;
    emit detectionsChanged();
  }
}

void CameraStream::start() { timer_.start(); }

void CameraStream::stop() { timer_.stop(); }

}  // namespace forklift_gui
