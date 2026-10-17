// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__PORTS__ILOGPORT_HPP_
#define FORKLIFT_GUI__CORE__PORTS__ILOGPORT_HPP_

#include <QObject>
#include <QString>
#include <QVector>

#include "forklift_gui/core/Types.hpp"

namespace forklift_gui
{

// Log portu: /rosout kayıtları gelir, GUI olayları ROS log'una gider.
class ILogPort : public QObject
{
  Q_OBJECT

public:
  using QObject::QObject;
  ~ILogPort() override = default;

  // GUI olayını (e-stop, kayıt, bağlantı...) düğümün ROS log'una yazar;
  // böylece rosbag ve entegrasyon testleri de görür (D-39).
  virtual void logEvent(int level, const QString & text) = 0;

signals:
  // Toplu teslim; GUI'nin kendi düğümünün kayıtları süzülmüştür.
  void entriesReceived(const QVector<forklift_gui::LogEntry> & entries);
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__PORTS__ILOGPORT_HPP_
