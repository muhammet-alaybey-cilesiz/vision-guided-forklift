// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__TELEMETRYMODEL_HPP_
#define FORKLIFT_GUI__CORE__TELEMETRYMODEL_HPP_

#include <QtQmlIntegration/qqmlintegration.h>

#include <QList>
#include <QObject>
#include <QPointF>
#include <QRectF>
#include <QTimer>
#include <QVariantList>
#include <deque>
#include <utility>

#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/core/Types.hpp"

namespace forklift_gui
{

// Poz, hız, toplam mesafe, çatal yüksekliği, sim zamanı, RTF ve odometri izi.
// Gelen örnekler iç durumu hemen günceller; QML'e bildirim tick() ile en
// fazla ui_max_rate'te gider (odometri 50 Hz gelse bile, spesifikasyon §2.2).
class TelemetryModel : public QObject
{
  Q_OBJECT
  QML_ELEMENT
  QML_UNCREATABLE("Backend.telemetry üzerinden erişilir")

  Q_PROPERTY(bool hasOdom READ hasOdom NOTIFY changed)
  Q_PROPERTY(double x READ x NOTIFY changed)
  Q_PROPERTY(double y READ y NOTIFY changed)
  Q_PROPERTY(double yaw READ yaw NOTIFY changed)
  Q_PROPERTY(double yawDegrees READ yawDegrees NOTIFY changed)
  Q_PROPERTY(double linear READ linear NOTIFY changed)
  Q_PROPERTY(double angular READ angular NOTIFY changed)
  Q_PROPERTY(double distance READ distance NOTIFY changed)
  Q_PROPERTY(bool hasFork READ hasFork NOTIFY changed)
  Q_PROPERTY(double forkHeight READ forkHeight NOTIFY changed)
  Q_PROPERTY(bool hasWheels READ hasWheels NOTIFY changed)
  Q_PROPERTY(double leftWheel READ leftWheel NOTIFY changed)
  Q_PROPERTY(double rightWheel READ rightWheel NOTIFY changed)
  Q_PROPERTY(bool hasClock READ hasClock NOTIFY changed)
  Q_PROPERTY(double simTime READ simTime NOTIFY changed)
  Q_PROPERTY(bool rtfValid READ rtfValid NOTIFY changed)
  Q_PROPERTY(double rtf READ rtf NOTIFY changed)
  // qmllint 6.4 QPointF/QRectF özellik tiplerini tanımıyor: iz QVariantList
  // (point öğeleri), sınırlar dört double olarak açılır.
  Q_PROPERTY(QVariantList trailPoints READ trailPoints NOTIFY trailChanged)
  Q_PROPERTY(double trailMinX READ trailMinX NOTIFY trailChanged)
  Q_PROPERTY(double trailMaxX READ trailMaxX NOTIFY trailChanged)
  Q_PROPERTY(double trailMinY READ trailMinY NOTIFY trailChanged)
  Q_PROPERTY(double trailMaxY READ trailMaxY NOTIFY trailChanged)

public:
  TelemetryModel(const GuiConfig::Telemetry & config, ClockFn clock, QObject * parent = nullptr);

  bool hasOdom() const { return shown_.hasOdom; }
  double x() const { return shown_.odom.x; }
  double y() const { return shown_.odom.y; }
  double yaw() const { return shown_.odom.yaw; }
  double yawDegrees() const;
  double linear() const { return shown_.odom.linear; }
  double angular() const { return shown_.odom.angular; }
  double distance() const { return shown_.distance; }
  bool hasFork() const { return shown_.hasFork; }
  double forkHeight() const { return shown_.forkHeight; }
  bool hasWheels() const { return shown_.hasWheels; }
  double leftWheel() const { return shown_.leftWheel; }
  double rightWheel() const { return shown_.rightWheel; }
  bool hasClock() const { return shown_.hasClock; }
  double simTime() const { return shown_.simTime; }
  bool rtfValid() const { return shown_.rtfValid; }
  double rtf() const { return shown_.rtf; }
  QList<QPointF> trail() const { return trail_; }
  QRectF trailBounds() const { return trailBounds_; }
  QVariantList trailPoints() const;
  double trailMinX() const { return trailBounds_.left(); }
  double trailMaxX() const { return trailBounds_.right(); }
  double trailMinY() const { return trailBounds_.top(); }
  double trailMaxY() const { return trailBounds_.bottom(); }

  // En güncel (bildirimi beklemeyen) değerler; veri seti kaydı için.
  OdomSample latestOdom() const { return live_.odom; }
  bool latestHasOdom() const { return live_.hasOdom; }
  double latestSimTime() const { return live_.simTime; }
  bool latestHasClock() const { return live_.hasClock; }

  void onOdometry(const forklift_gui::OdomSample & sample);
  void onJointStates(const forklift_gui::JointSample & sample);
  void onClock(double simSeconds, qint64 receivedNs);

  void tick(qint64 nowNs);
  void start();
  void stop();

  // Test: yayılan bildirim sayısı (30 Hz sınırı doğrulaması).
  int notificationCount() const { return notifications_; }

signals:
  void changed();
  void trailChanged();

private:
  struct State
  {
    bool hasOdom{false};
    OdomSample odom;
    double distance{0.0};
    bool hasFork{false};
    double forkHeight{0.0};
    bool hasWheels{false};
    double leftWheel{0.0};   // [rad/s]
    double rightWheel{0.0};  // [rad/s]
    bool hasClock{false};
    double simTime{0.0};
    bool rtfValid{false};
    double rtf{0.0};
  };

  void appendTrail(double x, double y);

  GuiConfig::Telemetry config_;
  ClockFn clock_;
  QTimer timer_;
  qint64 periodNs_;

  State live_;
  State shown_;
  bool dirty_{false};
  bool trailDirty_{false};
  qint64 lastNotifyNs_{-1};
  int notifications_{0};

  // RTF penceresi: (alım anı [ns], sim zamanı [s])
  std::deque<std::pair<qint64, double>> clockSamples_;

  QList<QPointF> trail_;
  QRectF trailBounds_;
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__TELEMETRYMODEL_HPP_
