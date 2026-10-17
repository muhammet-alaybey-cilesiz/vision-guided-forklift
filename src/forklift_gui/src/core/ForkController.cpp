// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/ForkController.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace forklift_gui
{

namespace
{

constexpr double kMmPerM = 1000.0;
// Aynı hedefin tekrar yayınlanmasını önleyen eşik [m] (sayısal gürültü).
constexpr double kSameTargetEps = 1e-6;

}  // namespace

ForkController::ForkController(
  const GuiConfig::Fork & config, double uiMaxRate, IRobotCommandPort * port, ClockFn clock,
  QObject * parent)
: QObject(parent),
  config_(config),
  port_(port),
  clock_(std::move(clock)),
  uiPeriodNs_(secondsToNs(1.0 / uiMaxRate))
{
  // Aralık periyottan kısa olmasın; aksi halde zaman denetimi her ikinci
  // tıkı atlar ve gerçek oran yarıya düşer.
  timer_.setInterval(static_cast<int>(std::ceil(1000.0 / uiMaxRate)));
  connect(&timer_, &QTimer::timeout, this, [this]() { tick(clock_()); });
}

QList<double> ForkController::presetHeights() const
{
  return QList<double>(config_.presetHeights.cbegin(), config_.presetHeights.cend());
}

void ForkController::setTarget(double height)
{
  if (!enabled_ || !std::isfinite(height)) {
    return;  // e-stop etkinken çatal komutu gönderilmez (spesifikasyon §4.2)
  }
  const double clamped = std::clamp(height, config_.minHeight, config_.maxHeight);
  if (hasTarget_ && std::abs(clamped - target_) < kSameTargetEps) {
    return;  // yalnız hedef değişince yayın
  }
  commandTarget(clamped, true);
}

void ForkController::stepUp()
{
  const double base = hasTarget_ ? target_ : (hasMeasured_ ? measured_ : config_.minHeight);
  setTarget(base + config_.keyStep);
}

void ForkController::stepDown()
{
  const double base = hasTarget_ ? target_ : (hasMeasured_ ? measured_ : config_.minHeight);
  setTarget(base - config_.keyStep);
}

void ForkController::applyPreset(int index)
{
  if (index < 0 || index >= config_.presetHeights.size()) {
    return;
  }
  setTarget(config_.presetHeights.at(index));
}

void ForkController::onJointSample(const JointSample & sample)
{
  if (!sample.hasFork) {
    return;
  }
  const qint64 now = clock_();
  // İlk örnek ya da resync_after'dan uzun boşluk sonrası: yeniden bağlantı.
  const bool reconnect =
    !hasMeasured_ || (lastSampleNs_ >= 0 && now - lastSampleNs_ > secondsToNs(config_.resyncAfter));
  lastSampleNs_ = now;
  const bool hadMeasurement = hasMeasured_;
  measured_ = sample.forkHeight;
  hasMeasured_ = true;
  measuredDirty_ = true;
  if (reconnect) {
    // Yeniden başlayan bir sime eski hedef itilmesin: hedef = ölçülen, yayın yok.
    commandTarget(std::clamp(measured_, config_.minHeight, config_.maxHeight), false);
    if (hadMeasurement) {
      emit event(
        static_cast<int>(LogLevel::Info),
        tr("Çatal verisi yeniden geldi; hedef ölçülen yüksekliğe eşitlendi (%1 mm)")
          .arg(measured_ * kMmPerM, 0, 'f', 1));
    }
  }
  evaluate(now);
}

void ForkController::holdAtMeasured()
{
  // E-stop devreye girerken, /e_stop=true'dan önce çağrılır (D-32). Çatal
  // hedefte duruyorsa gereksiz komut gönderilmez.
  if (!hasMeasured_ || (hasTarget_ && std::abs(measured_ - target_) <= config_.reachedTolerance)) {
    return;
  }
  const double hold = std::clamp(measured_, config_.minHeight, config_.maxHeight);
  commandTarget(hold, true);
  emit event(
    static_cast<int>(LogLevel::Warn),
    tr("E-stop: çatal ölçülen yükseklikte tutuluyor (%1 mm)").arg(hold * kMmPerM, 0, 'f', 1));
}

void ForkController::setEnabled(bool enabled)
{
  if (enabled == enabled_) {
    return;
  }
  enabled_ = enabled;
  emit enabledChanged();
}

void ForkController::tick(qint64 nowNs)
{
  evaluate(nowNs);
  if (measuredDirty_ && (lastNotifyNs_ < 0 || nowNs - lastNotifyNs_ >= uiPeriodNs_)) {
    measuredShown_ = measured_;
    measuredDirty_ = false;
    lastNotifyNs_ = nowNs;
    emit measuredChanged();
  }
}

void ForkController::start() { timer_.start(); }

void ForkController::stop() { timer_.stop(); }

void ForkController::commandTarget(double height, bool publish)
{
  hasTarget_ = true;
  target_ = height;
  const qint64 now = clock_();
  targetSetNs_ = now;
  // Zaman aşımı yalnız gönderilen komut için izlenir (eşitlemede değil).
  waitingForReach_ = publish;
  if (timedOut_) {
    timedOut_ = false;
    emit statusChanged();
  }
  if (publish) {
    port_->publishForkTarget(height);
  }
  emit targetChanged();
  evaluate(now);
}

void ForkController::evaluate(qint64 nowNs)
{
  const bool reached =
    hasTarget_ && hasMeasured_ && std::abs(measured_ - target_) <= config_.reachedTolerance;
  bool timedOut = timedOut_;
  if (waitingForReach_) {
    if (reached) {
      waitingForReach_ = false;
      emit event(
        static_cast<int>(LogLevel::Info),
        tr("Çatal hedefe ulaştı: %1 mm").arg(target_ * kMmPerM, 0, 'f', 1));
    } else if (nowNs - targetSetNs_ > secondsToNs(config_.reachTimeout)) {
      waitingForReach_ = false;
      timedOut = true;
      emit event(
        static_cast<int>(LogLevel::Warn),
        tr("Çatal hedefe %1 s içinde ulaşamadı (hedef %2 mm, ölçülen %3)")
          .arg(config_.reachTimeout, 0, 'f', 1)
          .arg(target_ * kMmPerM, 0, 'f', 1)
          .arg(
            hasMeasured_ ? QStringLiteral("%1 mm").arg(measured_ * kMmPerM, 0, 'f', 1)
                         : tr("yok")));
    }
  }
  if (reached) {
    timedOut = false;  // geç de olsa ulaşıldıysa uyarı kalkar
  }
  if (reached != reached_ || timedOut != timedOut_) {
    reached_ = reached;
    timedOut_ = timedOut;
    emit statusChanged();
  }
}

}  // namespace forklift_gui
