// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__PORTS__ICAMERAPORT_HPP_
#define FORKLIFT_GUI__CORE__PORTS__ICAMERAPORT_HPP_

#include <QObject>
#include <QSize>
#include <QString>
#include <QVideoFrame>

#include "forklift_gui/core/Types.hpp"

namespace forklift_gui
{

// Kamera portu. Dönüşüm (ROS Image → QVideoFrame) adaptörün kendi
// thread'inde yapılır; GUI thread'ine yalnızca hazır kare gelir (D-27).
class ICameraPort : public QObject
{
  Q_OBJECT

public:
  using QObject::QObject;
  ~ICameraPort() override = default;

signals:
  void frameReceived(
    const QString & cameraId, const QVideoFrame & frame, const forklift_gui::FrameMeta & meta);
  void cameraInfoReceived(const QString & cameraId, const QSize & resolution);
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__PORTS__ICAMERAPORT_HPP_
