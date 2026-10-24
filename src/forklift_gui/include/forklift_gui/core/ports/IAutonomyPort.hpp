// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__PORTS__IAUTONOMYPORT_HPP_
#define FORKLIFT_GUI__CORE__PORTS__IAUTONOMYPORT_HPP_

#include <QObject>
#include <QString>

#include "forklift_gui/core/Types.hpp"

namespace forklift_gui
{

// Görev yöneticisi portu (D-57): mod ve görev komutları, durum. request() GUI thread'inden
// çağrılır ve hemen döner; sonuç commandFinished() ile GUI thread'inde gelir.
class IAutonomyPort : public QObject
{
  Q_OBJECT

public:
  enum Command
  {
    SetManual = 0,
    SetAuto,
    Start,
    Pause,
    Resume,
    Cancel
  };
  Q_ENUM(Command)

  using QObject::QObject;
  ~IAutonomyPort() override = default;

  virtual void request(Command command) = 0;

signals:
  void statusReceived(const forklift_gui::AutonomyStatus & status);
  void commandFinished(int command, bool ok, const QString & message);
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__PORTS__IAUTONOMYPORT_HPP_
