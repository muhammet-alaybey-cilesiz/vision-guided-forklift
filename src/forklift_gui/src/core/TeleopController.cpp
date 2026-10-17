// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/TeleopController.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace forklift_gui
{

namespace
{

double clampAbs(double value, double limit) { return std::clamp(value, -limit, limit); }

// current'ı target'a en fazla maxStep kadar yaklaştırır (rampa adımı).
double approach(double current, double target, double maxStep)
{
  const double delta = target - current;
  if (std::abs(delta) <= maxStep) {
    return target;
  }
  return current + (delta > 0.0 ? maxStep : -maxStep);
}

int toIndex(TeleopController::Direction direction) { return static_cast<int>(direction); }

}  // namespace

TeleopController::TeleopController(
  const GuiConfig::Teleop & config, IRobotCommandPort * port, ClockFn clock, QObject * parent)
: QObject(parent),
  config_(config),
  port_(port),
  clock_(std::move(clock)),
  scale_(config.scaleDefault)
{
  timer_.setTimerType(Qt::PreciseTimer);
  timer_.setInterval(static_cast<int>(std::lround(1000.0 / config_.publishRate)));
  connect(&timer_, &QTimer::timeout, this, [this]() { tick(clock_()); });
}

void TeleopController::setSpeedScale(double scale)
{
  const double clamped = std::clamp(scale, config_.scaleMin, config_.scaleMax);
  if (std::abs(clamped - scale_) < 1e-9) {
    return;
  }
  scale_ = clamped;
  emit speedScaleChanged();
}

void TeleopController::setJoystick(double x, double y)
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return;
  }
  // Birim diske kırp: köşegende de |komut| sınırı aşmasın.
  const double r = std::hypot(x, y);
  if (r > 1.0) {
    x /= r;
    y /= r;
  }
  joyX_ = x;
  joyY_ = y;
  const bool wasHeld = joystickHeld_;
  joystickHeld_ = true;
  if (!wasHeld) {
    emit inputChanged();
  }
  updateActivation();
}

void TeleopController::releaseJoystick()
{
  if (!joystickHeld_) {
    return;
  }
  joystickHeld_ = false;
  joyX_ = 0.0;
  joyY_ = 0.0;
  emit inputChanged();
  updateActivation();
}

void TeleopController::setKeyState(Direction direction, bool pressed)
{
  keys_[toIndex(direction)] = pressed;
  updateActivation();
}

void TeleopController::setWindowActive(bool windowActive)
{
  windowActive_ = windowActive;
  if (!windowActive) {
    // Odak kaybında tuş bırakma olayları gelmez; tüm girdileri bırak.
    std::fill(std::begin(keys_), std::end(keys_), false);
    if (joystickHeld_) {
      joystickHeld_ = false;
      joyX_ = 0.0;
      joyY_ = 0.0;
      emit inputChanged();
    }
  }
  updateActivation();
}

void TeleopController::setEnabled(bool enabled)
{
  if (enabled == enabled_) {
    return;
  }
  enabled_ = enabled;
  if (!enabled) {
    // E-stop sırasında basılı kalan tuşlar serbest bırakmadan sonra robotu
    // kendiliğinden hareket ettirmesin: operatör yeniden basmalı.
    std::fill(std::begin(keys_), std::end(keys_), false);
    joystickHeld_ = false;
    joyX_ = 0.0;
    joyY_ = 0.0;
    emit inputChanged();
  }
  emit enabledChanged();
  updateActivation();
}

void TeleopController::emergencyStop()
{
  std::fill(std::begin(keys_), std::end(keys_), false);
  joystickHeld_ = false;
  joyX_ = 0.0;
  joyY_ = 0.0;
  emit inputChanged();
  // Etkin olmasa da tek sıfır gönderilir: twist_mux 'teleop' girdisini seçer
  // ve sıfırı iletir; otonomi sürerken de robot durur (savunma derinliği).
  const bool wasActive = active_;
  active_ = false;
  lastTickNs_ = -1;
  setCommand(0.0, 0.0);
  port_->publishVelocity(0.0, 0.0);
  if (wasActive) {
    emit activeChanged();
  }
}

void TeleopController::tick(qint64 nowNs)
{
  if (!active_) {
    return;
  }
  const double nominalDt = 1.0 / config_.publishRate;
  double dt = nominalDt;
  if (lastTickNs_ >= 0) {
    // Takılma sonrası tek adımda büyük sıçrama olmasın: en fazla 2 periyot.
    dt = std::clamp(nsToSeconds(nowNs - lastTickNs_), 0.0, 2.0 * nominalDt);
  }
  lastTickNs_ = nowNs;

  double linearAxis = 0.0;
  double angularAxis = 0.0;
  axesFromInputs(linearAxis, angularAxis);

  // Zarf: mutlak limit × ölçek. Hedef zarf içinde, komut ona rampayla gider.
  const double linearLimit = config_.maxLinear * scale_;
  const double angularLimit = config_.maxAngular * scale_;
  const double linearTarget = linearAxis * linearLimit;
  const double angularTarget = angularAxis * angularLimit;

  double linear = approach(linear_, linearTarget, config_.maxLinearAccel * dt);
  double angular = approach(angular_, angularTarget, config_.maxAngularAccel * dt);
  // Ölçek düşürüldüyse zarf hemen uygulanır; fiziksel yavaşlamayı DiffDrive
  // 1.0 m/s² ile sınırlar (D-09, D-30).
  linear = clampAbs(linear, linearLimit);
  angular = clampAbs(angular, angularLimit);

  setCommand(linear, angular);
  port_->publishVelocity(linear, angular);
}

void TeleopController::start() { timer_.start(); }

void TeleopController::stop() { timer_.stop(); }

bool TeleopController::anyKeyHeld() const
{
  return std::any_of(std::begin(keys_), std::end(keys_), [](bool k) { return k; });
}

void TeleopController::axesFromInputs(double & linearAxis, double & angularAxis) const
{
  if (anyKeyHeld()) {
    // Tuş basılıysa klavye öncelikli; W+S ya da A+D birbirini götürür.
    linearAxis = (keys_[toIndex(Direction::Forward)] ? 1.0 : 0.0) -
                 (keys_[toIndex(Direction::Backward)] ? 1.0 : 0.0);
    // REP-103: sola dönüş (A) pozitif ω.
    angularAxis = (keys_[toIndex(Direction::Left)] ? 1.0 : 0.0) -
                  (keys_[toIndex(Direction::Right)] ? 1.0 : 0.0);
  } else if (joystickHeld_) {
    linearAxis = joyY_;
    angularAxis = -joyX_;
  } else {
    linearAxis = 0.0;
    angularAxis = 0.0;
  }
}

void TeleopController::updateActivation()
{
  const bool wanted = enabled_ && windowActive_ && (anyKeyHeld() || joystickHeld_);
  if (wanted == active_) {
    return;
  }
  if (wanted) {
    active_ = true;
    lastTickNs_ = -1;
    emit activeChanged();
    // İlk komut periyot beklemeden gider; zamanlayıcı fazı buna hizalanır.
    if (timer_.isActive()) {
      timer_.start();
    }
    tick(clock_());
  } else {
    publishZeroAndStop();
  }
}

void TeleopController::publishZeroAndStop()
{
  active_ = false;
  lastTickNs_ = -1;
  setCommand(0.0, 0.0);
  port_->publishVelocity(0.0, 0.0);
  emit activeChanged();
}

void TeleopController::setCommand(double linear, double angular)
{
  if (linear == linear_ && angular == angular_) {
    return;
  }
  linear_ = linear;
  angular_ = angular;
  emit commandChanged();
}

}  // namespace forklift_gui
