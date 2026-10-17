// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__PORTS__IROBOTCOMMANDPORT_HPP_
#define FORKLIFT_GUI__CORE__PORTS__IROBOTCOMMANDPORT_HPP_

#include <QObject>

namespace forklift_gui
{

// GUI → robot komut portu. Yalnızca GUI thread'inden çağrılır.
// ROS adaptöründe rclcpp yayıncıları thread güvenlidir (DECISIONS D-27).
class IRobotCommandPort : public QObject
{
  Q_OBJECT

public:
  using QObject::QObject;
  ~IRobotCommandPort() override = default;

  // Sürüş komutu: doğrusal [m/s], açısal [rad/s] (/cmd_vel/teleop).
  virtual void publishVelocity(double linear, double angular) = 0;

  // Çatal hedef yüksekliği [m] (/fork/cmd_height).
  virtual void publishForkTarget(double height) = 0;

  // Acil durdurma durumu (/e_stop); sürekli yayın SafetyController'dadır.
  virtual void publishEStop(bool engaged) = 0;
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__PORTS__IROBOTCOMMANDPORT_HPP_
