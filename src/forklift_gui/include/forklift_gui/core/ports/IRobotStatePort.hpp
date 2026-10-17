// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__PORTS__IROBOTSTATEPORT_HPP_
#define FORKLIFT_GUI__CORE__PORTS__IROBOTSTATEPORT_HPP_

#include <QObject>
#include <QVector>

#include "forklift_gui/core/Types.hpp"

namespace forklift_gui
{

// Robot → GUI durum portu. Sinyaller HER ZAMAN GUI thread'inde yayılır;
// adaptör, ROS thread'inden gelen veriyi kuyruklu çağrıyla aktarır.
class IRobotStatePort : public QObject
{
  Q_OBJECT

public:
  using QObject::QObject;
  ~IRobotStatePort() override = default;

  // Abone olunan tüm topic'lerin sayaçlarının anlık kopyası. Thread
  // güvenlidir; sağlık modeli GUI thread'inden periyodik okur (D-27).
  virtual QVector<TopicCounter> topicCounters() const = 0;

  // ROS grafiğinin son bilinen durumu.
  virtual GraphStatus graphStatus() const = 0;

signals:
  void odometryReceived(const forklift_gui::OdomSample & sample);
  void jointStatesReceived(const forklift_gui::JointSample & sample);
  // Sim zamanı [s] ve alım anı (monoton saat) [ns]; kaynağında ≤ 30 Hz.
  void clockReceived(double simSeconds, qint64 receivedNs);
  void detectionsReceived(const forklift_gui::DetectionSet & detections);
  void graphStatusChanged(const forklift_gui::GraphStatus & status);
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__PORTS__IROBOTSTATEPORT_HPP_
