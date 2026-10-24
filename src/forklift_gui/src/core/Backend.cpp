// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/Backend.hpp"

#include <QJSEngine>
#include <QQmlEngine>
#include <utility>

namespace forklift_gui
{

namespace
{

Backend * g_instance = nullptr;

}  // namespace

Backend::Backend(
  const GuiConfig & config, const Ports & ports, bool mockMode, ClockFn clock, QObject * parent)
: QObject(parent), config_(config), ports_(ports), mockMode_(mockMode)
{
  teleop_ = new TeleopController(config.teleop, ports.command, clock, this);
  fork_ = new ForkController(config.fork, config.telemetry.uiMaxRate, ports.command, clock, this);
  safety_ =
    new SafetyController(config.safety, config.telemetry.uiMaxRate, ports.command, clock, this);
  telemetry_ = new TelemetryModel(config.telemetry, clock, this);
  health_ = new TopicHealthModel(config, ports.state, clock, this);
  log_ = new LogModel(config.log.maxEntries, this);
  logView_ = new LogFilterModel(log_, this);
  for (const CameraConfig & cam : config.cameras.list) {
    cameras_.append(new CameraStream(cam, config.cameras, config.health.updateRate, clock, this));
  }
  dataset_ = new DatasetRecorder(config, cameras_, telemetry_, fork_, clock, this);
  autonomy_ = new AutonomyController(config.autonomy, ports.autonomy, clock, this);

  // E-stop: önce sürüş ve çatal durdurulur (senkron), sonra kilit yayınlanır;
  // etkin olduğu sürece sürüş ve çatal kontrolleri devre dışı (D-31, D-32).
  connect(safety_, &SafetyController::engaging, this, [this]() {
    teleop_->emergencyStop();
    fork_->holdAtMeasured();
  });
  connect(safety_, &SafetyController::engagedChanged, this, [this](bool engaged) {
    teleop_->setEnabled(!engaged);
    autonomy_->setEStopEngaged(engaged);
    updateForkEnabled();
  });
  teleop_->setEnabled(!safety_->engaged());
  autonomy_->setEStopEngaged(safety_->engaged());
  // Otonom modda çatala tek kaynak (görev yöneticisi) komut verir; operatör sürüş girdisi
  // Otonom modu bırakır (D-57).
  connect(
    autonomy_, &AutonomyController::autoActiveChanged, this, [this](bool) { updateForkEnabled(); });
  connect(teleop_, &TeleopController::activeChanged, this, [this]() {
    if (teleop_->active()) {
      autonomy_->teleopTakeover();
    }
  });
  updateForkEnabled();

  // Port → model. Port sinyalleri GUI thread'inde yayılır (doğrudan bağlantı).
  connect(ports.state, &IRobotStatePort::odometryReceived, telemetry_, &TelemetryModel::onOdometry);
  connect(
    ports.state, &IRobotStatePort::jointStatesReceived, telemetry_, &TelemetryModel::onJointStates);
  connect(
    ports.state, &IRobotStatePort::jointStatesReceived, fork_, &ForkController::onJointSample);
  connect(ports.state, &IRobotStatePort::clockReceived, telemetry_, &TelemetryModel::onClock);
  connect(
    ports.state, &IRobotStatePort::detectionsReceived, this, [this](const DetectionSet & set) {
      if (CameraStream * cam = camera(set.cameraId)) {
        cam->onDetections(set);
      }
    });
  connect(
    ports.state, &IRobotStatePort::graphStatusChanged, health_, &TopicHealthModel::onGraphStatus);
  health_->onGraphStatus(ports.state->graphStatus());
  connect(
    ports.camera, &ICameraPort::frameReceived, this,
    [this](const QString & id, const QVideoFrame & frame, const FrameMeta & meta) {
      if (CameraStream * cam = camera(id)) {
        cam->onFrame(frame, meta);
      }
    });
  connect(
    ports.camera, &ICameraPort::cameraInfoReceived, this,
    [this](const QString & id, const QSize & size) {
      if (CameraStream * cam = camera(id)) {
        cam->onCameraInfo(size);
      }
    });
  connect(ports.log, &ILogPort::entriesReceived, log_, &LogModel::append);

  // GUI olayları: log paneline ve düğümün ROS log'una (D-39).
  connect(fork_, &ForkController::event, this, &Backend::onEvent);
  connect(safety_, &SafetyController::event, this, &Backend::onEvent);
  connect(health_, &TopicHealthModel::event, this, &Backend::onEvent);
  connect(dataset_, &DatasetRecorder::event, this, &Backend::onEvent);
  connect(autonomy_, &AutonomyController::logEvent, this, &Backend::onEvent);
}

void Backend::updateForkEnabled()
{
  fork_->setEnabled(!safety_->engaged() && !autonomy_->autoActive());
}

Backend::~Backend()
{
  if (g_instance == this) {
    g_instance = nullptr;
  }
}

Backend * Backend::create(QQmlEngine * qmlEngine, QJSEngine * jsEngine)
{
  Q_UNUSED(qmlEngine)
  Q_UNUSED(jsEngine)
  // Ömrü main.cpp yönetir; QML motoru silmesin.
  QJSEngine::setObjectOwnership(g_instance, QJSEngine::CppOwnership);
  return g_instance;
}

void Backend::setInstance(Backend * instance) { g_instance = instance; }

QQmlListProperty<CameraStream> Backend::cameras()
{
  return QQmlListProperty<CameraStream>(this, &cameras_);
}

CameraStream * Backend::camera(const QString & id) const
{
  for (CameraStream * cam : cameras_) {
    if (cam->cameraId() == id) {
      return cam;
    }
  }
  return nullptr;
}

void Backend::setHelpVisible(bool visible)
{
  if (visible == helpVisible_) {
    return;
  }
  helpVisible_ = visible;
  emit helpVisibleChanged();
}

bool Backend::handleKey(int key, bool pressed, bool autoRepeat)
{
  using Direction = TeleopController::Direction;
  switch (key) {
    // Sürüş: basılı tut / bırak; otomatik tekrar ölü adamı etkilemez.
    case Qt::Key_W:
      if (!autoRepeat) {
        teleop_->setKeyState(Direction::Forward, pressed);
      }
      return true;
    case Qt::Key_S:
      if (!autoRepeat) {
        teleop_->setKeyState(Direction::Backward, pressed);
      }
      return true;
    case Qt::Key_A:
      if (!autoRepeat) {
        teleop_->setKeyState(Direction::Left, pressed);
      }
      return true;
    case Qt::Key_D:
      if (!autoRepeat) {
        teleop_->setKeyState(Direction::Right, pressed);
      }
      return true;
    // E-stop: yalnız etkinleştirir, asla sıfırlamaz.
    case Qt::Key_Space:
      if (pressed && !autoRepeat) {
        safety_->engage();
      }
      return true;
    // Çatal: R/F basılı tutulursa adım tekrarlanır.
    case Qt::Key_R:
      if (pressed) {
        fork_->stepUp();
      }
      return true;
    case Qt::Key_F:
      if (pressed) {
        fork_->stepDown();
      }
      return true;
    case Qt::Key_C:
      if (pressed && !autoRepeat) {
        dataset_->capture();
      }
      return true;
    case Qt::Key_F1:
      if (pressed && !autoRepeat) {
        setHelpVisible(!helpVisible_);
      }
      return true;
    case Qt::Key_Escape:
      if (pressed && helpVisible_) {
        setHelpVisible(false);
        return true;
      }
      return false;
    default:
      break;
  }
  // 1–9: preset'ler (yapılandırmadaki sırayla).
  if (key >= Qt::Key_1 && key <= Qt::Key_9) {
    if (pressed && !autoRepeat) {
      fork_->applyPreset(key - Qt::Key_1);
    }
    return true;
  }
  return false;
}

void Backend::setWindowActive(bool active) { teleop_->setWindowActive(active); }

void Backend::start()
{
  teleop_->start();
  fork_->start();
  safety_->start();
  telemetry_->start();
  health_->start();
  autonomy_->startTicking(200);
  for (CameraStream * cam : std::as_const(cameras_)) {
    cam->start();
  }
}

void Backend::stop()
{
  // Düzenli kapanış e-stop gibidir: operatör artık tuş tutamaz. Sıfır teleop,
  // çatal tutma ve /e_stop=true hemen gider; kilit zaman aşımını (0.5 s)
  // beklemeye gerek kalmaz (D-31).
  if (!safety_->engaged()) {
    safety_->engage();
  }
  dataset_->setRecording(false);
  teleop_->stop();
  fork_->stop();
  safety_->stop();
  telemetry_->stop();
  health_->stop();
  autonomy_->stopTicking();
  for (CameraStream * cam : std::as_const(cameras_)) {
    cam->stop();
  }
  dataset_->waitForIdle(5000);
}

void Backend::onEvent(int level, const QString & text)
{
  log_->addLocal(level, text);
  ports_.log->logEvent(level, text);
}

}  // namespace forklift_gui
