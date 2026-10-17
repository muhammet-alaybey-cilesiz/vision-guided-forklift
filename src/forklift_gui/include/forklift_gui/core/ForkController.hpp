// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__FORKCONTROLLER_HPP_
#define FORKLIFT_GUI__CORE__FORKCONTROLLER_HPP_

#include <QtQmlIntegration/qqmlintegration.h>

#include <QList>
#include <QObject>
#include <QStringList>
#include <QTimer>

#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/core/Types.hpp"
#include "forklift_gui/core/ports/IRobotCommandPort.hpp"

namespace forklift_gui
{

// Çatal hedefi, preset'ler ve "hedefe ulaşıldı" durumu (DECISIONS D-33):
//  * Hedef [min, max] aralığına kırpılır; komut yalnız hedef değişince gider.
//  * |ölçülen − hedef| ≤ tolerans → ulaşıldı; hedef değişiminden itibaren
//    reach_timeout içinde ulaşılamazsa uyarı.
//  * Yeniden bağlantıda hedef ölçülen yüksekliğe eşitlenir, komut gönderilmez.
//  * E-stop'ta: devreye girerken bir kez "ölçülende tut", sonra komut yok (D-32).
// Ölçülen değer her örnekte iç durumu günceller, QML bildirimi tick() ile
// en fazla ui_max_rate'te yapılır.
class ForkController : public QObject
{
  Q_OBJECT
  QML_ELEMENT
  QML_UNCREATABLE("Backend.fork üzerinden erişilir")

  Q_PROPERTY(bool hasTarget READ hasTarget NOTIFY targetChanged)
  Q_PROPERTY(double target READ target NOTIFY targetChanged)
  Q_PROPERTY(bool hasMeasured READ hasMeasured NOTIFY measuredChanged)
  Q_PROPERTY(double measured READ measured NOTIFY measuredChanged)
  Q_PROPERTY(bool reached READ reached NOTIFY statusChanged)
  Q_PROPERTY(bool timedOut READ timedOut NOTIFY statusChanged)
  Q_PROPERTY(bool enabled READ enabled NOTIFY enabledChanged)
  Q_PROPERTY(double minHeight READ minHeight CONSTANT)
  Q_PROPERTY(double maxHeight READ maxHeight CONSTANT)
  Q_PROPERTY(double step READ step CONSTANT)
  Q_PROPERTY(double tolerance READ tolerance CONSTANT)
  Q_PROPERTY(QStringList presetLabels READ presetLabels CONSTANT)
  Q_PROPERTY(QList<double> presetHeights READ presetHeights CONSTANT)

public:
  ForkController(
    const GuiConfig::Fork & config, double uiMaxRate, IRobotCommandPort * port, ClockFn clock,
    QObject * parent = nullptr);

  bool hasTarget() const { return hasTarget_; }
  double target() const { return target_; }
  bool hasMeasured() const { return hasMeasured_; }
  double measured() const { return measuredShown_; }
  // Kısılmamış son ölçüm (veri seti kaydı için).
  double latestMeasured() const { return measured_; }
  bool reached() const { return reached_; }
  bool timedOut() const { return timedOut_; }
  bool enabled() const { return enabled_; }
  double minHeight() const { return config_.minHeight; }
  double maxHeight() const { return config_.maxHeight; }
  double step() const { return config_.keyStep; }
  double tolerance() const { return config_.reachedTolerance; }
  QStringList presetLabels() const { return config_.presetLabels; }
  QList<double> presetHeights() const;

  Q_INVOKABLE void setTarget(double height);
  Q_INVOKABLE void stepUp();
  Q_INVOKABLE void stepDown();
  Q_INVOKABLE void applyPreset(int index);

  void onJointSample(const forklift_gui::JointSample & sample);
  void holdAtMeasured();
  void setEnabled(bool enabled);

  void tick(qint64 nowNs);
  void start();
  void stop();

signals:
  void targetChanged();
  void measuredChanged();
  void statusChanged();
  void enabledChanged();
  void event(int level, const QString & text);

private:
  void commandTarget(double height, bool publish);
  void evaluate(qint64 nowNs);

  GuiConfig::Fork config_;
  IRobotCommandPort * port_;
  ClockFn clock_;
  QTimer timer_;
  qint64 uiPeriodNs_;

  bool enabled_{false};
  bool hasTarget_{false};
  double target_{0.0};
  qint64 targetSetNs_{-1};
  bool waitingForReach_{false};
  bool hasMeasured_{false};
  double measured_{0.0};
  double measuredShown_{0.0};
  bool measuredDirty_{false};
  qint64 lastSampleNs_{-1};
  qint64 lastNotifyNs_{-1};
  bool reached_{false};
  bool timedOut_{false};
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__FORKCONTROLLER_HPP_
