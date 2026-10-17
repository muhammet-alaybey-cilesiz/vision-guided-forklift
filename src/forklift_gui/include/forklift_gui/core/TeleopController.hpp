// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__TELEOPCONTROLLER_HPP_
#define FORKLIFT_GUI__CORE__TELEOPCONTROLLER_HPP_

#include <QtQmlIntegration/qqmlintegration.h>

#include <QObject>
#include <QTimer>

#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/core/ports/IRobotCommandPort.hpp"

namespace forklift_gui
{

// Klavye / sanal joystick → hız komutu. Kurallar (DECISIONS D-30):
//  * Komut yalnızca ölü adam (tuş basılı ya da joystick tutulu) etkinken,
//    publish_rate ile gider; bırakınca TEK sıfır komutu, ardından sessizlik.
//  * Etkinken her değişim ivme sınırıyla rampalanır; zarf daralması
//    (ölçek düşürme) hemen uygulanır, böylece |komut| ≤ limit × ölçek her an.
//  * Pencere odağı kaybı ve e-stop güvenlik durdurmasıdır: anında sıfır.
class TeleopController : public QObject
{
  Q_OBJECT
  QML_ELEMENT
  QML_UNCREATABLE("Backend.teleop üzerinden erişilir")

  Q_PROPERTY(double linearCommand READ linearCommand NOTIFY commandChanged)
  Q_PROPERTY(double angularCommand READ angularCommand NOTIFY commandChanged)
  Q_PROPERTY(bool active READ active NOTIFY activeChanged)
  Q_PROPERTY(bool enabled READ enabled NOTIFY enabledChanged)
  Q_PROPERTY(bool joystickHeld READ joystickHeld NOTIFY inputChanged)
  Q_PROPERTY(double speedScale READ speedScale WRITE setSpeedScale NOTIFY speedScaleChanged)
  Q_PROPERTY(double scaleMin READ scaleMin CONSTANT)
  Q_PROPERTY(double scaleMax READ scaleMax CONSTANT)
  Q_PROPERTY(double scaleStep READ scaleStep CONSTANT)
  Q_PROPERTY(double maxLinear READ maxLinear CONSTANT)
  Q_PROPERTY(double maxAngular READ maxAngular CONSTANT)

public:
  enum class Direction
  {
    Forward,
    Backward,
    Left,
    Right
  };

  TeleopController(
    const GuiConfig::Teleop & config, IRobotCommandPort * port, ClockFn clock,
    QObject * parent = nullptr);

  double linearCommand() const { return linear_; }
  double angularCommand() const { return angular_; }
  bool active() const { return active_; }
  bool enabled() const { return enabled_; }
  bool joystickHeld() const { return joystickHeld_; }
  double speedScale() const { return scale_; }
  double scaleMin() const { return config_.scaleMin; }
  double scaleMax() const { return config_.scaleMax; }
  double scaleStep() const { return config_.scaleStep; }
  double maxLinear() const { return config_.maxLinear; }
  double maxAngular() const { return config_.maxAngular; }

  void setSpeedScale(double scale);

  // Sanal joystick: x sağa +, y ileri +; birim diske kırpılır.
  Q_INVOKABLE void setJoystick(double x, double y);
  Q_INVOKABLE void releaseJoystick();

  // Klavye (Backend::handleKey üzerinden; otomatik tekrar süzülmüş).
  void setKeyState(Direction direction, bool pressed);

  // Pencere odağı; kaybında tüm girdiler bırakılmış sayılır.
  void setWindowActive(bool windowActive);

  // E-stop durumu: false iken komut üretilmez.
  void setEnabled(bool enabled);

  // E-stop devreye girerken, /e_stop yayınlanmadan HEMEN önce çağrılır:
  // etkinse tek sıfır komutu gönderir ve yayını keser (D-31).
  void emergencyStop();

  // Periyodik adım; uygulamada iç zamanlayıcı çağırır, testler doğrudan.
  void tick(qint64 nowNs);

  void start();
  void stop();

signals:
  void commandChanged();
  void activeChanged();
  void enabledChanged();
  void inputChanged();
  void speedScaleChanged();

private:
  bool anyKeyHeld() const;
  void axesFromInputs(double & linearAxis, double & angularAxis) const;
  void updateActivation();
  void publishZeroAndStop();
  void setCommand(double linear, double angular);

  GuiConfig::Teleop config_;
  IRobotCommandPort * port_;
  ClockFn clock_;
  QTimer timer_;

  bool keys_[4]{false, false, false, false};
  bool joystickHeld_{false};
  double joyX_{0.0};
  double joyY_{0.0};
  bool windowActive_{true};
  bool enabled_{false};
  bool active_{false};
  double scale_{0.0};
  double linear_{0.0};
  double angular_{0.0};
  qint64 lastTickNs_{-1};
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__TELEOPCONTROLLER_HPP_
