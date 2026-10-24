// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__MOCK__MOCKROBOTADAPTER_HPP_
#define FORKLIFT_GUI__MOCK__MOCKROBOTADAPTER_HPP_

#include <QHash>
#include <QMutex>
#include <QObject>
#include <QPair>
#include <QThread>
#include <QTimer>
#include <QVector>
#include <QVideoFrame>

#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/core/Types.hpp"
#include "forklift_gui/core/ports/IAutonomyPort.hpp"
#include "forklift_gui/core/ports/ICameraPort.hpp"
#include "forklift_gui/core/ports/ILogPort.hpp"
#include "forklift_gui/core/ports/IRobotCommandPort.hpp"
#include "forklift_gui/core/ports/IRobotStatePort.hpp"

namespace forklift_gui
{
namespace mock
{

// Komut kaydı: birim testler port çıkışını bununla doğrular.
struct CommandRecord
{
  enum class Kind
  {
    Velocity,
    ForkTarget,
    EStop
  };
  Kind kind{Kind::Velocity};
  double a{0.0};      // doğrusal hız ya da çatal hedefi
  double b{0.0};      // açısal hız
  bool flag{false};   // e-stop durumu
  qint64 stampNs{0};  // enjekte edilen saate göre
};

class MockCommandPort : public IRobotCommandPort
{
  Q_OBJECT

public:
  explicit MockCommandPort(ClockFn clock, QObject * parent = nullptr);

  void publishVelocity(double linear, double angular) override;
  void publishForkTarget(double height) override;
  void publishEStop(bool engaged) override;

  const QVector<CommandRecord> & records() const { return records_; }
  void clearRecords() { records_.clear(); }

signals:
  void velocityCommanded(double linear, double angular);
  void forkCommanded(double height);
  void eStopCommanded(bool engaged);

private:
  ClockFn clock_;
  QVector<CommandRecord> records_;
};

class MockStatePort : public IRobotStatePort
{
  Q_OBJECT

public:
  using IRobotStatePort::IRobotStatePort;

  QVector<TopicCounter> topicCounters() const override;
  GraphStatus graphStatus() const override;

  // Testler ve sahte robot için sayaç ve grafik denetimi (thread güvenli).
  void setCounter(const QString & key, quint64 count, qint64 lastNs);
  void bump(const QString & key, qint64 nowNs);
  void setGraphStatus(const forklift_gui::GraphStatus & status);

private:
  mutable QMutex mutex_;
  QHash<QString, TopicCounter> counters_;
  GraphStatus graph_;
};

class MockCameraPort : public ICameraPort
{
  Q_OBJECT

public:
  using ICameraPort::ICameraPort;
};

class MockLogPort : public ILogPort
{
  Q_OBJECT

public:
  using ILogPort::ILogPort;

  void logEvent(int level, const QString & text) override;
  const QVector<QPair<int, QString>> & events() const { return events_; }

private:
  QVector<QPair<int, QString>> events_;
};

// Sentetik kamera kareleri ve tespitler; kendi QThread'inde çalışır, böylece
// mock da ROS adaptörüyle aynı thread'ler arası yolu kullanır (D-38).
class FrameWorker : public QObject
{
  Q_OBJECT

public:
  struct Pose
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double fork{0.0};
    double simTime{0.0};
  };

  explicit FrameWorker(const GuiConfig & config);

  void setPose(const Pose & pose);  // GUI thread'inden

public slots:
  void begin();
  void end();

signals:
  void frameReady(
    const QString & cameraId, const QVideoFrame & frame, const forklift_gui::FrameMeta & meta);
  void detectionsReady(const forklift_gui::DetectionSet & set);

private:
  void render();

  GuiConfig config_;
  QTimer * timer_{nullptr};
  mutable QMutex poseMutex_;
  Pose pose_;
  quint64 frameIndex_{0};
};

// Sahte robot: DiffDrive gibi son komutu tutan tek tekerlekli kinematik,
// 0.05 m/s çatal, RTF 1 saat, sentetik kareler/tespitler ve /rosout satırları.
// E-stop etkinken hedef hız sıfırdır (cmd_vel_gate davranışı, D-24).
// Sahte görev yöneticisi: komutları kaydeder, hemen yanıtlar; görev advance() ile
// durumdan duruma ilerler (uygulamada zamanlayıcı, testlerde elle).
class MockAutonomyPort : public IAutonomyPort
{
  Q_OBJECT

public:
  explicit MockAutonomyPort(ClockFn clock, QObject * parent = nullptr);

  void request(Command command) override;
  // Durumu yayınlar (uygulamada 5 Hz).
  void publish();
  // Görevi bir durum ilerletir.
  void advance();

  const QVector<int> & requests() const { return requests_; }
  AutonomyStatus status() const { return status_; }
  // Testler: yanıtın başarısız olmasını zorlar.
  void setReject(bool reject) { reject_ = reject; }

private:
  ClockFn clock_;
  AutonomyStatus status_;
  QString pausedFrom_;
  QVector<int> requests_;
  bool reject_{false};
};

class MockRobotAdapter : public QObject
{
  Q_OBJECT

public:
  MockRobotAdapter(const GuiConfig & config, ClockFn clock, QObject * parent = nullptr);
  ~MockRobotAdapter() override;

  MockCommandPort * command() { return command_; }
  MockStatePort * state() { return state_; }
  MockCameraPort * camera() { return camera_; }
  MockLogPort * log() { return log_; }
  MockAutonomyPort * autonomy() { return autonomy_; }

  // Zamanlayıcılar ve kare iş parçacığı; birim testler çağırmaz.
  void start();
  void stop();

private:
  void step();
  void clockTick();
  void logTick();
  void onFrame(const QString & cameraId, const QVideoFrame & frame, const FrameMeta & meta);
  void onDetections(const DetectionSet & set);
  QString healthKeyForTopic(const QString & topic) const;

  GuiConfig config_;
  ClockFn clock_;
  MockCommandPort * command_;
  MockStatePort * state_;
  MockCameraPort * camera_;
  MockLogPort * log_;
  MockAutonomyPort * autonomy_;
  QTimer autonomyTimer_;
  int autonomyTicks_{0};

  QTimer stepTimer_;
  QTimer clockTimer_;
  QTimer logTimer_;
  QThread workerThread_;
  FrameWorker * worker_{nullptr};

  // Sahte robot durumu (GUI thread'i)
  qint64 startNs_{-1};
  qint64 lastStepNs_{-1};
  qint64 lastClockEmitNs_{-1};
  double simTime_{0.0};
  double cmdLinear_{0.0};
  double cmdAngular_{0.0};
  bool eStop_{true};
  double forkTarget_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double v_{0.0};
  double w_{0.0};
  double fork_{0.0};
  int logIndex_{0};
};

}  // namespace mock
}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__MOCK__MOCKROBOTADAPTER_HPP_
