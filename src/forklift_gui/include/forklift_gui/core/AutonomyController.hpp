// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__AUTONOMYCONTROLLER_HPP_
#define FORKLIFT_GUI__CORE__AUTONOMYCONTROLLER_HPP_

#include <QtQmlIntegration/qqmlintegration.h>

#include <QObject>
#include <QString>
#include <QTimer>

#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/core/Types.hpp"
#include "forklift_gui/core/ports/IAutonomyPort.hpp"

namespace forklift_gui
{

// Manuel / Otonom mod paneli (D-57). Mod görev yöneticisinde tutulur; bu sınıf istekleri
// gönderir ve gelen durumu gösterir:
//  * Durum offline_after içinde gelmezse çevrimdışı: Otonom ve görev düğmeleri pasif.
//  * Görev başlatma yalnız Otonom modda ve e-stop bırakılmışken.
//  * Otonom modda operatör sürüş girdisi verirse (WASD / joystick) Manuel moda geçilir;
//    görev yöneticisi görevi DURAKLADI'ya alır (teleop önceliği zaten twist_mux'ta).
//  * autoActive() iken çatal paneli pasiftir (çatala tek kaynak komut versin).
class AutonomyController : public QObject
{
  Q_OBJECT
  QML_ELEMENT
  QML_UNCREATABLE("Backend.autonomy üzerinden erişilir")

  Q_PROPERTY(bool online READ online NOTIFY changed)
  Q_PROPERTY(bool autoMode READ autoMode NOTIFY changed)
  Q_PROPERTY(QString state READ state NOTIFY changed)
  Q_PROPERTY(QString detail READ detail NOTIFY changed)
  Q_PROPERTY(QString fault READ fault NOTIFY changed)
  Q_PROPERTY(QString targetColor READ targetColor NOTIFY changed)
  Q_PROPERTY(int parcelsDone READ parcelsDone NOTIFY changed)
  Q_PROPERTY(int parcelsTotal READ parcelsTotal NOTIFY changed)
  Q_PROPERTY(bool busy READ busy NOTIFY changed)
  Q_PROPERTY(QString lastMessage READ lastMessage NOTIFY changed)
  Q_PROPERTY(bool canSetAuto READ canSetAuto NOTIFY changed)
  Q_PROPERTY(bool canStart READ canStart NOTIFY changed)
  Q_PROPERTY(bool canPause READ canPause NOTIFY changed)
  Q_PROPERTY(bool canResume READ canResume NOTIFY changed)
  Q_PROPERTY(bool canCancel READ canCancel NOTIFY changed)

public:
  AutonomyController(
    const GuiConfig::Autonomy & config, IAutonomyPort * port, ClockFn clock,
    QObject * parent = nullptr);

  bool online() const { return online_; }
  bool autoMode() const { return online_ && status_.mode == 1; }
  // Otonomi şu an robotu yönetebilir mi (Otonom mod ve çevrimiçi).
  bool autoActive() const { return autoMode(); }
  QString state() const { return online_ ? status_.state : QString(); }
  QString detail() const { return status_.detail; }
  QString fault() const { return status_.fault; }
  QString targetColor() const { return status_.targetColor; }
  int parcelsDone() const { return status_.parcelsDone; }
  int parcelsTotal() const { return status_.parcelsTotal; }
  bool busy() const { return pending_ >= 0; }
  QString lastMessage() const { return lastMessage_; }
  bool canSetAuto() const { return online_ && !busy(); }
  bool canStart() const;
  bool canPause() const;
  bool canResume() const;
  bool canCancel() const;

  Q_INVOKABLE void setManual();
  Q_INVOKABLE void setAuto();
  Q_INVOKABLE void start();
  Q_INVOKABLE void pause();
  Q_INVOKABLE void resume();
  Q_INVOKABLE void cancel();

  // Operatör sürüş girdisi verdi: Otonom moddaysa Manuel'e geçilir.
  void teleopTakeover();
  void setEStopEngaged(bool engaged);
  void onStatus(const forklift_gui::AutonomyStatus & status);
  void onCommandFinished(int command, bool ok, const QString & message);
  // Çevrimdışı denetimi; uygulamada zamanlayıcı çağırır.
  void tick(qint64 nowNs);

  void startTicking(int intervalMs);
  void stopTicking();

signals:
  void changed();
  void autoActiveChanged(bool active);
  void logEvent(int level, const QString & text);

private:
  void send(IAutonomyPort::Command command);
  bool missionActive() const;

  GuiConfig::Autonomy config_;
  IAutonomyPort * port_;
  ClockFn clock_;
  AutonomyStatus status_;
  bool online_{false};
  bool eStopEngaged_{true};
  int pending_{-1};
  QString lastMessage_;
  bool lastAutoActive_{false};
  QTimer timer_;
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__AUTONOMYCONTROLLER_HPP_
