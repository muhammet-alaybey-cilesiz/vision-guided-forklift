// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/SafetyController.hpp"

#include <cmath>
#include <utility>

#include "forklift_gui/core/Types.hpp"

namespace forklift_gui
{

SafetyController::SafetyController(
  const GuiConfig::Safety & config, double uiMaxRate, IRobotCommandPort * port, ClockFn clock,
  QObject * parent)
: QObject(parent), config_(config), port_(port), clock_(std::move(clock))
{
  heartbeatTimer_.setTimerType(Qt::PreciseTimer);
  heartbeatTimer_.setInterval(static_cast<int>(std::lround(1000.0 / config_.eStopRate)));
  connect(&heartbeatTimer_, &QTimer::timeout, this, &SafetyController::heartbeat);
  progressTimer_.setInterval(static_cast<int>(std::ceil(1000.0 / uiMaxRate)));
  connect(&progressTimer_, &QTimer::timeout, this, [this]() { tick(clock_()); });
}

void SafetyController::engage()
{
  // Space ve buton yalnız etkinleştirir; zaten etkinse hiçbir şey değişmez.
  cancelRelease();
  if (engaged_) {
    return;
  }
  // Önce sürüş ve çatal durdurulur (senkron), sonra kilit yayınlanır.
  emit engaging();
  engaged_ = true;
  port_->publishEStop(true);
  emit engagedChanged(true);
  emit event(static_cast<int>(LogLevel::Warn), tr("E-STOP etkin"));
}

void SafetyController::beginRelease()
{
  if (!engaged_ || releasing_) {
    return;
  }
  releaseStartNs_ = clock_();
  setProgress(true, 0.0);
  if (heartbeatTimer_.isActive()) {
    progressTimer_.start();
  }
}

void SafetyController::cancelRelease()
{
  if (!releasing_) {
    return;
  }
  progressTimer_.stop();
  setProgress(false, 0.0);
}

void SafetyController::tick(qint64 nowNs)
{
  if (!releasing_) {
    return;
  }
  const double progress = nsToSeconds(nowNs - releaseStartNs_) / config_.releaseHold;
  if (progress < 1.0) {
    setProgress(true, progress);
    return;
  }
  // Bilinçli bırakma tamamlandı.
  progressTimer_.stop();
  setProgress(false, 0.0);
  engaged_ = false;
  port_->publishEStop(false);
  emit engagedChanged(false);
  emit event(static_cast<int>(LogLevel::Info), tr("E-STOP serbest bırakıldı"));
}

void SafetyController::heartbeat() { port_->publishEStop(engaged_); }

void SafetyController::start()
{
  heartbeat();
  heartbeatTimer_.start();
}

void SafetyController::stop()
{
  heartbeatTimer_.stop();
  progressTimer_.stop();
}

void SafetyController::setProgress(bool releasing, double progress)
{
  if (releasing == releasing_ && progress == progress_) {
    return;
  }
  releasing_ = releasing;
  progress_ = progress;
  emit releaseProgressChanged();
}

}  // namespace forklift_gui
