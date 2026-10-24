// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/AutonomyController.hpp"

#include <QStringList>

namespace forklift_gui
{

namespace
{

const QStringList kRunning = {
  QStringLiteral("OBSERVE"),  QStringLiteral("SELECT_BAY"), QStringLiteral("ALIGN"),
  QStringLiteral("SETTLE"),   QStringLiteral("ENTER"),      QStringLiteral("LIFT"),
  QStringLiteral("BACK_OUT"), QStringLiteral("VERIFY"),     QStringLiteral("FIND_STATION"),
  QStringLiteral("TRANSIT"),  QStringLiteral("PRE_DOCK"),   QStringLiteral("LOWER")};

QString commandName(int command)
{
  switch (command) {
    case IAutonomyPort::SetManual:
      return QObject::tr("Manuel mod");
    case IAutonomyPort::SetAuto:
      return QObject::tr("Otonom mod");
    case IAutonomyPort::Start:
      return QObject::tr("Görevi başlat");
    case IAutonomyPort::Pause:
      return QObject::tr("Duraklat");
    case IAutonomyPort::Resume:
      return QObject::tr("Devam");
    case IAutonomyPort::Cancel:
      return QObject::tr("İptal");
    default:
      return QString::number(command);
  }
}

}  // namespace

AutonomyController::AutonomyController(
  const GuiConfig::Autonomy & config, IAutonomyPort * port, ClockFn clock, QObject * parent)
: QObject(parent), config_(config), port_(port), clock_(std::move(clock))
{
  if (port_ != nullptr) {
    connect(port_, &IAutonomyPort::statusReceived, this, &AutonomyController::onStatus);
    connect(port_, &IAutonomyPort::commandFinished, this, &AutonomyController::onCommandFinished);
  }
  connect(&timer_, &QTimer::timeout, this, [this]() { tick(clock_()); });
}

bool AutonomyController::missionActive() const { return kRunning.contains(status_.state); }

bool AutonomyController::canStart() const
{
  return autoMode() && !busy() && !eStopEngaged_ && !missionActive() &&
         status_.state != QStringLiteral("PAUSED");
}

bool AutonomyController::canPause() const { return online_ && !busy() && missionActive(); }

bool AutonomyController::canResume() const
{
  return autoMode() && !busy() && !eStopEngaged_ && status_.state == QStringLiteral("PAUSED");
}

bool AutonomyController::canCancel() const
{
  return online_ && !busy() && (missionActive() || status_.state == QStringLiteral("PAUSED"));
}

void AutonomyController::setManual() { send(IAutonomyPort::SetManual); }

void AutonomyController::setAuto()
{
  if (canSetAuto()) {
    send(IAutonomyPort::SetAuto);
  }
}

void AutonomyController::start()
{
  if (canStart()) {
    send(IAutonomyPort::Start);
  }
}

void AutonomyController::pause()
{
  if (canPause()) {
    send(IAutonomyPort::Pause);
  }
}

void AutonomyController::resume()
{
  if (canResume()) {
    send(IAutonomyPort::Resume);
  }
}

void AutonomyController::cancel()
{
  if (canCancel()) {
    send(IAutonomyPort::Cancel);
  }
}

void AutonomyController::teleopTakeover()
{
  if (autoMode()) {
    emit logEvent(
      static_cast<int>(LogLevel::Warn), tr("Operatör sürüşü devraldı: Manuel moda geçiliyor"));
    send(IAutonomyPort::SetManual);
  }
}

void AutonomyController::setEStopEngaged(bool engaged)
{
  if (engaged != eStopEngaged_) {
    eStopEngaged_ = engaged;
    emit changed();
  }
}

void AutonomyController::send(IAutonomyPort::Command command)
{
  if (port_ == nullptr || !online_) {
    return;
  }
  pending_ = command;
  emit changed();
  port_->request(command);
}

void AutonomyController::onStatus(const AutonomyStatus & status)
{
  const bool wasOnline = online_;
  status_ = status;
  online_ = true;
  if (!wasOnline) {
    emit logEvent(static_cast<int>(LogLevel::Info), tr("Görev yöneticisi çevrimiçi"));
  }
  emit changed();
  if (autoActive() != lastAutoActive_) {
    lastAutoActive_ = autoActive();
    emit autoActiveChanged(lastAutoActive_);
  }
}

void AutonomyController::onCommandFinished(int command, bool ok, const QString & message)
{
  if (pending_ == command) {
    pending_ = -1;
  }
  lastMessage_ = commandName(command) + QStringLiteral(": ") + message;
  emit logEvent(static_cast<int>(ok ? LogLevel::Info : LogLevel::Warn), lastMessage_);
  emit changed();
}

void AutonomyController::tick(qint64 nowNs)
{
  if (!online_) {
    return;
  }
  const qint64 age = nowNs - status_.receivedNs;
  if (age > static_cast<qint64>(config_.offlineAfter * 1e9)) {
    online_ = false;
    pending_ = -1;
    emit logEvent(
      static_cast<int>(LogLevel::Warn), tr("Görev yöneticisi çevrimdışı (durum gelmiyor)"));
    emit changed();
    if (lastAutoActive_) {
      lastAutoActive_ = false;
      emit autoActiveChanged(false);
    }
  }
}

void AutonomyController::startTicking(int intervalMs) { timer_.start(intervalMs); }

void AutonomyController::stopTicking() { timer_.stop(); }

}  // namespace forklift_gui
