// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__SAFETYCONTROLLER_HPP_
#define FORKLIFT_GUI__CORE__SAFETYCONTROLLER_HPP_

#include <QtQmlIntegration/qqmlintegration.h>

#include <QObject>
#include <QTimer>

#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/core/ports/IRobotCommandPort.hpp"

namespace forklift_gui
{

// Acil durdurma mandalı (DECISIONS D-31):
//  * Uygulama e-stop ETKİN açılır; durum e_stop_rate ile sürekli yayınlanır.
//  * engage(): buton ya da Space. Yeni durum heartbeat'i beklemeden gider.
//  * Sıfırlama yalnızca butonu release_hold süre basılı tutarak; Space sıfırlamaz.
//  * engaging() sinyali /e_stop=true yayınlanmadan ÖNCE senkron yayılır;
//    Backend bunu sürüşü ve çatalı durdurmak için kullanır.
class SafetyController : public QObject
{
  Q_OBJECT
  QML_ELEMENT
  QML_UNCREATABLE("Backend.safety üzerinden erişilir")

  Q_PROPERTY(bool engaged READ engaged NOTIFY engagedChanged)
  Q_PROPERTY(bool releasing READ releasing NOTIFY releaseProgressChanged)
  Q_PROPERTY(double releaseProgress READ releaseProgress NOTIFY releaseProgressChanged)
  Q_PROPERTY(double releaseHoldSeconds READ releaseHoldSeconds CONSTANT)

public:
  SafetyController(
    const GuiConfig::Safety & config, double uiMaxRate, IRobotCommandPort * port, ClockFn clock,
    QObject * parent = nullptr);

  bool engaged() const { return engaged_; }
  bool releasing() const { return releasing_; }
  double releaseProgress() const { return progress_; }
  double releaseHoldSeconds() const { return config_.releaseHold; }

  Q_INVOKABLE void engage();
  Q_INVOKABLE void beginRelease();
  Q_INVOKABLE void cancelRelease();

  // Bırakma ilerlemesi; uygulamada ui_max_rate zamanlayıcısı çağırır.
  void tick(qint64 nowNs);
  // Mevcut durumu yayınlar; uygulamada e_stop_rate zamanlayıcısı çağırır.
  void heartbeat();

  void start();
  void stop();

signals:
  void engaging();
  void engagedChanged(bool engaged);
  void releaseProgressChanged();
  void event(int level, const QString & text);

private:
  void setProgress(bool releasing, double progress);

  GuiConfig::Safety config_;
  IRobotCommandPort * port_;
  ClockFn clock_;
  QTimer heartbeatTimer_;
  QTimer progressTimer_;

  bool engaged_{true};
  bool releasing_{false};
  qint64 releaseStartNs_{-1};
  double progress_{0.0};
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__SAFETYCONTROLLER_HPP_
