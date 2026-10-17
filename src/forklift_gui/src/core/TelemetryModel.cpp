// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/TelemetryModel.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace forklift_gui
{

namespace
{

constexpr double kRadToDeg = 180.0 / M_PI;

}  // namespace

TelemetryModel::TelemetryModel(const GuiConfig::Telemetry & config, ClockFn clock, QObject * parent)
: QObject(parent),
  config_(config),
  clock_(std::move(clock)),
  periodNs_(secondsToNs(1.0 / config.uiMaxRate))
{
  // Aralık periyottan kısa olmasın (bkz. ForkController).
  timer_.setInterval(static_cast<int>(std::ceil(1000.0 / config_.uiMaxRate)));
  connect(&timer_, &QTimer::timeout, this, [this]() { tick(clock_()); });
}

QVariantList TelemetryModel::trailPoints() const
{
  QVariantList out;
  out.reserve(trail_.size());
  for (const QPointF & p : trail_) {
    out.append(QVariant::fromValue(p));
  }
  return out;
}

double TelemetryModel::yawDegrees() const { return shown_.odom.yaw * kRadToDeg; }

void TelemetryModel::onOdometry(const OdomSample & sample)
{
  if (live_.hasOdom && sample.stamp < live_.odom.stamp) {
    // Sim zamanı geri gitti: simülasyon yeniden başladı. Eski iz ve mesafe
    // yeni oturuma taşınmaz; sıfırlama noktası mesafeye eklenmez.
    live_.distance = 0.0;
    trail_.clear();
    trailBounds_ = QRectF();
    trailDirty_ = true;
  } else if (live_.hasOdom) {
    live_.distance += std::hypot(sample.x - live_.odom.x, sample.y - live_.odom.y);
  }
  live_.odom = sample;
  live_.hasOdom = true;
  dirty_ = true;
  appendTrail(sample.x, sample.y);
}

void TelemetryModel::onJointStates(const JointSample & sample)
{
  if (sample.hasFork) {
    live_.forkHeight = sample.forkHeight;
    live_.hasFork = true;
    dirty_ = true;
  }
  if (sample.hasWheels) {
    live_.leftWheel = sample.leftWheel;
    live_.rightWheel = sample.rightWheel;
    live_.hasWheels = true;
    dirty_ = true;
  }
}

void TelemetryModel::onClock(double simSeconds, qint64 receivedNs)
{
  if (live_.hasClock && simSeconds < live_.simTime) {
    clockSamples_.clear();  // sim yeniden başladı; RTF penceresi baştan
  }
  live_.simTime = simSeconds;
  live_.hasClock = true;

  clockSamples_.emplace_back(receivedNs, simSeconds);
  const qint64 windowNs = secondsToNs(config_.rtfWindow);
  while (clockSamples_.size() > 2 && receivedNs - clockSamples_[1].first >= windowNs) {
    clockSamples_.pop_front();
  }
  // RTF = Δsim / Δduvar; pencerenin en az yarısı dolmadan gösterilmez.
  const auto & first = clockSamples_.front();
  const qint64 spanNs = receivedNs - first.first;
  if (clockSamples_.size() >= 2 && spanNs >= windowNs / 2) {
    live_.rtf = (simSeconds - first.second) / nsToSeconds(spanNs);
    live_.rtfValid = true;
  } else {
    live_.rtfValid = false;
  }
  dirty_ = true;
}

void TelemetryModel::tick(qint64 nowNs)
{
  // Saat akmıyorsa (sim durdu/kapandı) RTF geçersiz sayılır.
  if (
    live_.rtfValid && !clockSamples_.empty() &&
    nowNs - clockSamples_.back().first > secondsToNs(config_.rtfWindow)) {
    live_.rtfValid = false;
    dirty_ = true;
  }
  if (!dirty_ && !trailDirty_) {
    return;
  }
  if (lastNotifyNs_ >= 0 && nowNs - lastNotifyNs_ < periodNs_) {
    return;  // en fazla ui_max_rate
  }
  lastNotifyNs_ = nowNs;
  if (dirty_) {
    shown_ = live_;
    dirty_ = false;
    ++notifications_;
    emit changed();
  }
  if (trailDirty_) {
    trailDirty_ = false;
    emit trailChanged();
  }
}

void TelemetryModel::start() { timer_.start(); }

void TelemetryModel::stop() { timer_.stop(); }

void TelemetryModel::appendTrail(double x, double y)
{
  const QPointF p(x, y);
  if (!trail_.isEmpty()) {
    const QPointF last = trail_.constLast();
    if (std::hypot(p.x() - last.x(), p.y() - last.y()) < config_.trailMinStep) {
      return;
    }
  }
  trail_.append(p);
  bool shrink = false;
  while (trail_.size() > config_.trailMaxPoints) {
    trail_.removeFirst();
    shrink = true;
  }
  if (shrink || trail_.size() == 1) {
    // Baştan çıkan nokta sınırı daraltabilir: tam yeniden hesap (≤ N nokta).
    double minX = trail_.constFirst().x();
    double maxX = minX;
    double minY = trail_.constFirst().y();
    double maxY = minY;
    for (const QPointF & q : std::as_const(trail_)) {
      minX = std::min(minX, q.x());
      maxX = std::max(maxX, q.x());
      minY = std::min(minY, q.y());
      maxY = std::max(maxY, q.y());
    }
    trailBounds_ = QRectF(QPointF(minX, minY), QPointF(maxX, maxY));
  } else {
    const double minX = std::min(trailBounds_.left(), p.x());
    const double maxX = std::max(trailBounds_.right(), p.x());
    const double minY = std::min(trailBounds_.top(), p.y());
    const double maxY = std::max(trailBounds_.bottom(), p.y());
    trailBounds_ = QRectF(QPointF(minX, minY), QPointF(maxX, maxY));
  }
  trailDirty_ = true;
}

}  // namespace forklift_gui
