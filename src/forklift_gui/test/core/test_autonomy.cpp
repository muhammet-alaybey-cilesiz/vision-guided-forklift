// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// U9: Manuel / Otonom mod paneli (D-57). Görev yöneticisi çevrimdışıyken düğmeler pasif;
// Otonom moda geçiş isteği gider; Otonom modda çatal paneli pasif; e-stop basılıyken görev
// başlamaz; sürüş girdisi Otonom'dan Manuel'e geçirir; durum gelmezse çevrimdışı olur.

#include <QtTest>
#include <memory>

#include "TestSupport.hpp"
#include "forklift_gui/core/Backend.hpp"
#include "forklift_gui/mock/MockRobotAdapter.hpp"

using forklift_gui::Backend;
using forklift_gui::GuiConfig;
using forklift_gui::IAutonomyPort;
using forklift_gui::JointSample;
using forklift_gui::mock::MockAutonomyPort;
using forklift_gui::mock::MockRobotAdapter;
using forklift_gui::test::FakeClock;

class TestAutonomy : public QObject
{
  Q_OBJECT

private slots:
  void init();
  void cleanup();
  void offlineUntilStatus();
  void autoModeDisablesForkAndNeedsEStopReleaseToStart();
  void driveInputTakesOver();
  void pauseResumeCancel();
  void goesOfflineWhenStatusStops();

private:
  // publish: sahte görev yöneticisi durumunu her adımda yayınlasın mı (uygulamada 5 Hz)
  void run(double seconds, bool publish = true);
  void releaseEStop();
  int requests(int command) const;

  GuiConfig cfg_;
  FakeClock clock_;
  std::unique_ptr<MockRobotAdapter> adapter_;
  std::unique_ptr<Backend> backend_;
  MockAutonomyPort * port_{nullptr};
};

void TestAutonomy::init()
{
  cfg_ = forklift_gui::test::loadTestConfig();
  clock_ = FakeClock();
  adapter_ = std::make_unique<MockRobotAdapter>(cfg_, clock_.fn());
  Backend::Ports ports;
  ports.command = adapter_->command();
  ports.state = adapter_->state();
  ports.camera = adapter_->camera();
  ports.log = adapter_->log();
  ports.autonomy = adapter_->autonomy();
  port_ = adapter_->autonomy();
  backend_ = std::make_unique<Backend>(cfg_, ports, true, clock_.fn());
  JointSample js;
  js.hasFork = true;
  js.forkHeight = 0.0;
  emit adapter_->state()->jointStatesReceived(js);
}

void TestAutonomy::cleanup()
{
  backend_.reset();
  adapter_.reset();
}

void TestAutonomy::run(double seconds, bool publish)
{
  const double dt = 1.0 / cfg_.teleop.publishRate;
  for (double t = 0.0; t < seconds; t += dt) {
    clock_.advance(dt);
    if (publish) {
      port_->publish();
    }
    backend_->teleop()->tick(clock_.now);
    backend_->fork()->tick(clock_.now);
    backend_->safety()->tick(clock_.now);
    backend_->autonomy()->tick(clock_.now);
  }
}

void TestAutonomy::releaseEStop()
{
  backend_->safety()->beginRelease();
  run(cfg_.safety.releaseHold + 0.1);
  QVERIFY(!backend_->safety()->engaged());
}

int TestAutonomy::requests(int command) const
{
  return static_cast<int>(port_->requests().count(command));
}

void TestAutonomy::offlineUntilStatus()
{
  auto * a = backend_->autonomy();
  QVERIFY(!a->online());
  a->setAuto();
  QCOMPARE(requests(IAutonomyPort::SetAuto), 0);  // çevrimdışıyken istek gitmez
  port_->publish();
  QVERIFY(a->online());
  QVERIFY(!a->autoMode());
  QVERIFY(a->canSetAuto());
}

void TestAutonomy::autoModeDisablesForkAndNeedsEStopReleaseToStart()
{
  auto * a = backend_->autonomy();
  port_->publish();
  releaseEStop();
  QVERIFY(backend_->fork()->enabled());
  a->setAuto();
  QCOMPARE(requests(IAutonomyPort::SetAuto), 1);
  QVERIFY(a->autoMode());
  QVERIFY(!backend_->fork()->enabled());  // otonom modda çatal paneli pasif
  QVERIFY(a->canStart());
  backend_->safety()->engage();
  QVERIFY(!a->canStart());  // e-stop basılı: başlatılamaz
  a->start();
  QCOMPARE(requests(IAutonomyPort::Start), 0);
  releaseEStop();
  a->start();
  QCOMPARE(requests(IAutonomyPort::Start), 1);
  QCOMPARE(a->state(), QStringLiteral("OBSERVE"));
  QVERIFY(!a->canStart());  // görev çalışırken yeniden başlatılmaz
}

void TestAutonomy::driveInputTakesOver()
{
  auto * a = backend_->autonomy();
  port_->publish();
  releaseEStop();
  a->setAuto();
  a->start();
  QVERIFY(backend_->handleKey(Qt::Key_W, true, false));
  run(0.2);
  QVERIFY(backend_->handleKey(Qt::Key_W, false, false));
  QCOMPARE(requests(IAutonomyPort::SetManual), 1);
  QVERIFY(!a->autoMode());
  QCOMPARE(a->state(), QStringLiteral("PAUSED"));
  QVERIFY(backend_->fork()->enabled());  // Manuel'de çatal yeniden etkin
}

void TestAutonomy::pauseResumeCancel()
{
  auto * a = backend_->autonomy();
  port_->publish();
  releaseEStop();
  a->setAuto();
  a->start();
  QVERIFY(a->canPause());
  a->pause();
  QCOMPARE(a->state(), QStringLiteral("PAUSED"));
  QVERIFY(a->canResume());
  a->resume();
  QCOMPARE(a->state(), QStringLiteral("OBSERVE"));
  a->cancel();
  QCOMPARE(a->state(), QStringLiteral("IDLE"));
  port_->setReject(true);
  a->start();
  QVERIFY(a->lastMessage().contains(QStringLiteral("reddedildi")));
  QVERIFY(!a->busy());
}

void TestAutonomy::goesOfflineWhenStatusStops()
{
  auto * a = backend_->autonomy();
  port_->publish();
  releaseEStop();
  a->setAuto();
  QVERIFY(!backend_->fork()->enabled());
  run(cfg_.autonomy.offlineAfter + 0.5, false);
  QVERIFY(!a->online());
  QVERIFY(!a->autoMode());
  QVERIFY(backend_->fork()->enabled());  // görev yöneticisi yoksa operatör çatalı kullanabilir
}

QTEST_GUILESS_MAIN(TestAutonomy)
#include "test_autonomy.moc"
