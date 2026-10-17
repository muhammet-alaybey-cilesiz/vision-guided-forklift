// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__BACKEND_HPP_
#define FORKLIFT_GUI__CORE__BACKEND_HPP_

#include <QtQmlIntegration/qqmlintegration.h>

#include <QList>
#include <QObject>
#include <QQmlListProperty>

#include "forklift_gui/core/CameraStream.hpp"
#include "forklift_gui/core/DatasetRecorder.hpp"
#include "forklift_gui/core/ForkController.hpp"
#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/LogModel.hpp"
#include "forklift_gui/core/SafetyController.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/core/TelemetryModel.hpp"
#include "forklift_gui/core/TeleopController.hpp"
#include "forklift_gui/core/TopicHealthModel.hpp"
#include "forklift_gui/core/ports/ICameraPort.hpp"
#include "forklift_gui/core/ports/ILogPort.hpp"
#include "forklift_gui/core/ports/IRobotCommandPort.hpp"
#include "forklift_gui/core/ports/IRobotStatePort.hpp"

class QQmlEngine;
class QJSEngine;

namespace forklift_gui
{

// QML'in tek giriş noktası (DECISIONS D-28). main.cpp (composition root)
// adaptörü seçer, Backend'i portlarla kurar ve setInstance() ile kaydeder;
// QML_SINGLETON fabrikası create() bu örneği döndürür. View-model'ler
// CONSTANT özellikler olarak açılır. Klavye eşlemesi de burada (C++) yapılır.
class Backend : public QObject
{
  Q_OBJECT
  QML_ELEMENT
  QML_SINGLETON

  Q_PROPERTY(forklift_gui::TeleopController * teleop READ teleop CONSTANT)
  Q_PROPERTY(forklift_gui::ForkController * fork READ fork CONSTANT)
  Q_PROPERTY(forklift_gui::SafetyController * safety READ safety CONSTANT)
  Q_PROPERTY(forklift_gui::TelemetryModel * telemetry READ telemetry CONSTANT)
  Q_PROPERTY(forklift_gui::TopicHealthModel * health READ health CONSTANT)
  Q_PROPERTY(forklift_gui::LogModel * log READ log CONSTANT)
  Q_PROPERTY(forklift_gui::LogFilterModel * logView READ logView CONSTANT)
  Q_PROPERTY(forklift_gui::DatasetRecorder * dataset READ dataset CONSTANT)
  Q_PROPERTY(QQmlListProperty<forklift_gui::CameraStream> cameras READ cameras CONSTANT)
  Q_PROPERTY(bool mockMode READ mockMode CONSTANT)
  Q_PROPERTY(bool helpVisible READ helpVisible WRITE setHelpVisible NOTIFY helpVisibleChanged)

public:
  struct Ports
  {
    IRobotCommandPort * command{nullptr};
    IRobotStatePort * state{nullptr};
    ICameraPort * camera{nullptr};
    ILogPort * log{nullptr};
  };

  Backend(
    const GuiConfig & config, const Ports & ports, bool mockMode, ClockFn clock,
    QObject * parent = nullptr);
  ~Backend() override;

  // QML_SINGLETON fabrikası: main.cpp'nin kurduğu örneği döndürür.
  static Backend * create(QQmlEngine * qmlEngine, QJSEngine * jsEngine);
  static void setInstance(Backend * instance);

  TeleopController * teleop() const { return teleop_; }
  ForkController * fork() const { return fork_; }
  SafetyController * safety() const { return safety_; }
  TelemetryModel * telemetry() const { return telemetry_; }
  TopicHealthModel * health() const { return health_; }
  LogModel * log() const { return log_; }
  LogFilterModel * logView() const { return logView_; }
  DatasetRecorder * dataset() const { return dataset_; }
  QQmlListProperty<CameraStream> cameras();
  const QList<CameraStream *> & cameraList() const { return cameras_; }
  CameraStream * camera(const QString & id) const;
  bool mockMode() const { return mockMode_; }
  bool helpVisible() const { return helpVisible_; }
  void setHelpVisible(bool visible);

  // Klavye: W/S/A/D sürüş, R/F çatal adımı, 1–4 preset, C kayıt, F1 yardım,
  // Space e-stop. Otomatik tekrar sürüşte yok sayılır. İşlendiyse true.
  Q_INVOKABLE bool handleKey(int key, bool pressed, bool autoRepeat);

  // Pencere odağı (uygulama katmanı bağlar).
  void setWindowActive(bool active);

  // Zamanlayıcıları başlatır / durdurur (testlerde çağrılmaz).
  void start();
  void stop();

signals:
  void helpVisibleChanged();

private:
  void onEvent(int level, const QString & text);

  GuiConfig config_;
  Ports ports_;
  bool mockMode_;
  bool helpVisible_{false};

  TeleopController * teleop_;
  ForkController * fork_;
  SafetyController * safety_;
  TelemetryModel * telemetry_;
  TopicHealthModel * health_;
  LogModel * log_;
  LogFilterModel * logView_;
  QList<CameraStream *> cameras_;
  DatasetRecorder * dataset_;
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__BACKEND_HPP_
