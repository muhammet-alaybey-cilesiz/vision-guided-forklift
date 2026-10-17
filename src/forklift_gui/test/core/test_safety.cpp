// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// U5: e-stop mandalı. Açılışta etkin; Space etkinleştirir, sıfırlamaz;
// etkinken sürüş ve çatal komutu çıkmaz; sıfırlama yalnız basılı tutarak.

#include <QtTest>
#include <memory>

#include "TestSupport.hpp"
#include "forklift_gui/core/Backend.hpp"
#include "forklift_gui/mock/MockRobotAdapter.hpp"

using forklift_gui::Backend;
using forklift_gui::GuiConfig;
using forklift_gui::JointSample;
using forklift_gui::mock::CommandRecord;
using forklift_gui::mock::MockRobotAdapter;
using forklift_gui::test::FakeClock;
using Kind = CommandRecord::Kind;

class TestSafety : public QObject
{
  Q_OBJECT

private slots:
  void init();
  void cleanup();
  void latch();  // U5
  void releaseNeedsFullHold();
  void zeroBeforeLock();

private:
  int count(Kind kind) const;
  void pressKey(int key);
  void releaseKey(int key);
  void run(double seconds);
  void releaseEStop();

  GuiConfig cfg_;
  FakeClock clock_;
  std::unique_ptr<MockRobotAdapter> adapter_;
  std::unique_ptr<Backend> backend_;
};

void TestSafety::init()
{
  cfg_ = forklift_gui::test::loadTestConfig();
  clock_ = FakeClock();
  adapter_ = std::make_unique<MockRobotAdapter>(cfg_, clock_.fn());
  Backend::Ports ports;
  ports.command = adapter_->command();
  ports.state = adapter_->state();
  ports.camera = adapter_->camera();
  ports.log = adapter_->log();
  backend_ = std::make_unique<Backend>(cfg_, ports, true, clock_.fn());
  // Çatal ölçümü olsun (hedef eşitlenir, komut gitmez).
  JointSample js;
  js.hasFork = true;
  js.forkHeight = 0.0;
  emit adapter_->state()->jointStatesReceived(js);
}

void TestSafety::cleanup()
{
  backend_.reset();
  adapter_.reset();
}

int TestSafety::count(Kind kind) const
{
  int n = 0;
  for (const CommandRecord & r : adapter_->command()->records()) {
    n += r.kind == kind ? 1 : 0;
  }
  return n;
}

void TestSafety::pressKey(int key) { QVERIFY(backend_->handleKey(key, true, false)); }

void TestSafety::releaseKey(int key) { QVERIFY(backend_->handleKey(key, false, false)); }

void TestSafety::run(double seconds)
{
  const double dt = 1.0 / cfg_.teleop.publishRate;
  for (double t = 0.0; t < seconds; t += dt) {
    clock_.advance(dt);
    backend_->teleop()->tick(clock_.now);
    backend_->fork()->tick(clock_.now);
    backend_->safety()->tick(clock_.now);
  }
}

void TestSafety::releaseEStop()
{
  backend_->safety()->beginRelease();
  run(cfg_.safety.releaseHold + 0.1);
  QVERIFY(!backend_->safety()->engaged());
}

void TestSafety::latch()
{
  auto * safety = backend_->safety();
  auto * cmd = adapter_->command();

  // 1) Açılışta etkin; heartbeat 'true' yayınlar.
  QVERIFY(safety->engaged());
  safety->heartbeat();
  QCOMPARE(cmd->records().back().kind, Kind::EStop);
  QCOMPARE(cmd->records().back().flag, true);

  // 2) Etkinken sürüş ve çatal komutu çıkmaz (klavye, joystick, preset, adım).
  cmd->clearRecords();
  pressKey(Qt::Key_W);
  backend_->teleop()->setJoystick(0.0, 1.0);
  run(1.0);
  releaseKey(Qt::Key_W);
  backend_->teleop()->releaseJoystick();
  pressKey(Qt::Key_3);
  pressKey(Qt::Key_R);
  backend_->fork()->setTarget(0.1);
  QCOMPARE(count(Kind::Velocity), 0);
  QCOMPARE(count(Kind::ForkTarget), 0);
  QVERIFY(!backend_->teleop()->enabled());
  QVERIFY(!backend_->fork()->enabled());

  // 3) Space etkinken sıfırlamaz (kaç kez basılırsa basılsın).
  for (int i = 0; i < 3; ++i) {
    pressKey(Qt::Key_Space);
    releaseKey(Qt::Key_Space);
  }
  QVERIFY(safety->engaged());

  // 4) Basılı tutma ile serbest bırakılır; 'false' hemen yayınlanır.
  cmd->clearRecords();
  releaseEStop();
  QCOMPARE(count(Kind::EStop), 1);
  QCOMPARE(cmd->records().back().flag, false);
  QVERIFY(backend_->teleop()->enabled());

  // 5) Serbestken sürüş çalışır.
  cmd->clearRecords();
  pressKey(Qt::Key_W);
  run(0.5);
  QVERIFY(count(Kind::Velocity) > 0);

  // 6) Space etkinleştirir; sonrasında sürüş komutu çıkmaz.
  pressKey(Qt::Key_Space);
  QVERIFY(safety->engaged());
  const int velocitiesAtEngage = count(Kind::Velocity);
  run(1.0);
  pressKey(Qt::Key_W);
  run(1.0);
  QCOMPARE(count(Kind::Velocity), velocitiesAtEngage);
}

// Basılı tutma süresi dolmadan bırakılırsa e-stop etkin kalır; Space de
// süren bir bırakmayı iptal eder.
void TestSafety::releaseNeedsFullHold()
{
  auto * safety = backend_->safety();
  safety->beginRelease();
  run(cfg_.safety.releaseHold * 0.8);
  QVERIFY(safety->releasing());
  QVERIFY(safety->releaseProgress() > 0.5 && safety->releaseProgress() < 1.0);
  safety->cancelRelease();
  QVERIFY(safety->engaged());
  QVERIFY(!safety->releasing());
  QCOMPARE(safety->releaseProgress(), 0.0);
  run(cfg_.safety.releaseHold);
  QVERIFY(safety->engaged());

  safety->beginRelease();
  run(cfg_.safety.releaseHold * 0.5);
  pressKey(Qt::Key_Space);
  QVERIFY(!safety->releasing());
  run(cfg_.safety.releaseHold);
  QVERIFY(safety->engaged());
}

// E-stop devreye girerken: önce tek sıfır sürüş komutu ve (çatal hareket
// halindeyse) "ölçülende tut", SONRA /e_stop=true (D-31, D-32).
void TestSafety::zeroBeforeLock()
{
  releaseEStop();
  auto * cmd = adapter_->command();
  // Gerçekte /joint_states 50 Hz akar; resync_after'dan uzun boşluk yeniden
  // bağlantı sayılır (D-33). Bu yüzden önce taze bir ölçüm verilir.
  JointSample js;
  js.hasFork = true;
  js.forkHeight = 0.0;
  emit adapter_->state()->jointStatesReceived(js);
  // Çatal 0'dan 0.06'ya giderken (ölçülen 0.02) ve robot sürülürken.
  backend_->fork()->setTarget(0.06);
  js.forkHeight = 0.02;
  emit adapter_->state()->jointStatesReceived(js);
  pressKey(Qt::Key_W);
  run(0.5);
  cmd->clearRecords();

  pressKey(Qt::Key_Space);
  const auto & r = cmd->records();
  QCOMPARE(r.size(), 3);
  QCOMPARE(r.at(0).kind, Kind::Velocity);
  QCOMPARE(r.at(0).a, 0.0);
  QCOMPARE(r.at(0).b, 0.0);
  QCOMPARE(r.at(1).kind, Kind::ForkTarget);
  QVERIFY(std::abs(r.at(1).a - 0.02) < 1e-9);
  QCOMPARE(r.at(2).kind, Kind::EStop);
  QCOMPARE(r.at(2).flag, true);
}

QTEST_MAIN(TestSafety)
#include "test_safety.moc"
