// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// U6: çatal hedefi. Limit dışı hedef kırpılır; ±tolerans içinde "ulaşıldı";
// reach_timeout içinde ulaşılamazsa uyarı. Ek: preset, adım, yalnız
// değişimde yayın, e-stop'ta komut yok, yeniden bağlantıda eşitleme.

#include <QSignalSpy>
#include <QtTest>
#include <memory>
#include <vector>

#include "TestSupport.hpp"
#include "forklift_gui/core/ForkController.hpp"
#include "forklift_gui/mock/MockRobotAdapter.hpp"

using forklift_gui::ForkController;
using forklift_gui::GuiConfig;
using forklift_gui::JointSample;
using forklift_gui::mock::CommandRecord;
using forklift_gui::mock::MockCommandPort;
using forklift_gui::test::FakeClock;

class TestFork : public QObject
{
  Q_OBJECT

private slots:
  void init();
  void cleanup();
  void target();  // U6
  void reachedTolerance();
  void timeoutWarning();
  void presetsAndSteps();
  void noCommandWhenDisabled();
  void resyncAfterGap();

private:
  std::vector<double> targets() const;
  void measure(double height);

  GuiConfig cfg_;
  FakeClock clock_;
  std::unique_ptr<MockCommandPort> port_;
  std::unique_ptr<ForkController> fork_;
};

void TestFork::init()
{
  cfg_ = forklift_gui::test::loadTestConfig();
  clock_ = FakeClock();
  port_ = std::make_unique<MockCommandPort>(clock_.fn());
  fork_ =
    std::make_unique<ForkController>(cfg_.fork, cfg_.telemetry.uiMaxRate, port_.get(), clock_.fn());
  fork_->setEnabled(true);
  measure(0.0);  // ilk örnek: hedef = ölçülen, yayın yok
  QVERIFY(targets().empty());
  QVERIFY(fork_->hasTarget());
  QCOMPARE(fork_->target(), 0.0);
}

void TestFork::cleanup()
{
  fork_.reset();
  port_.reset();
}

std::vector<double> TestFork::targets() const
{
  std::vector<double> out;
  for (const CommandRecord & r : port_->records()) {
    if (r.kind == CommandRecord::Kind::ForkTarget) {
      out.push_back(r.a);
    }
  }
  return out;
}

void TestFork::measure(double height)
{
  JointSample s;
  s.hasFork = true;
  s.forkHeight = height;
  fork_->onJointSample(s);
}

void TestFork::target()
{
  const auto & f = cfg_.fork;
  fork_->setTarget(f.maxHeight + 0.5);  // üst sınır dışı
  fork_->setTarget(f.minHeight - 0.5);  // alt sınır dışı
  fork_->setTarget(0.06);
  fork_->setTarget(0.06);         // aynı hedef: yayın yok
  fork_->setTarget(0.06 + 1e-8);  // sayısal gürültü: yayın yok
  const auto t = targets();
  QCOMPARE(t.size(), size_t(3));
  QCOMPARE(t[0], f.maxHeight);
  QCOMPARE(t[1], f.minHeight);
  QCOMPARE(t[2], 0.06);
  QCOMPARE(fork_->target(), 0.06);
}

void TestFork::reachedTolerance()
{
  const double tol = cfg_.fork.reachedTolerance;
  fork_->setTarget(0.06);
  QVERIFY(!fork_->reached());
  measure(0.06 - tol - 0.0005);  // 5.5 mm uzak
  QVERIFY(!fork_->reached());
  measure(0.06 - tol + 0.0005);  // 4.5 mm uzak
  QVERIFY(fork_->reached());
  measure(0.06 + tol - 0.0005);  // üstten 4.5 mm
  QVERIFY(fork_->reached());
  measure(0.06 + tol + 0.0005);  // üstten 5.5 mm
  QVERIFY(!fork_->reached());
}

void TestFork::timeoutWarning()
{
  const double timeout = cfg_.fork.reachTimeout;
  QSignalSpy events(fork_.get(), &ForkController::event);
  fork_->setTarget(0.10);
  // Çatal 60 mm'de takılı kalıyor.
  for (double t = 0.0; t < timeout - 0.1; t += 0.05) {
    clock_.advance(0.05);
    measure(0.06);
    fork_->tick(clock_.now);
  }
  QVERIFY(!fork_->timedOut());
  clock_.advance(0.2);
  measure(0.06);
  fork_->tick(clock_.now);
  QVERIFY(fork_->timedOut());
  QVERIFY(!fork_->reached());
  QCOMPARE(events.count(), 1);  // uyarı bir kez
  // Sonradan ulaşılırsa uyarı kalkar.
  measure(0.10);
  QVERIFY(fork_->reached());
  QVERIFY(!fork_->timedOut());
  // Yeni hedef zaman aşımını yeniden başlatır.
  fork_->setTarget(0.02);
  QVERIFY(!fork_->timedOut());
}

void TestFork::presetsAndSteps()
{
  const auto & f = cfg_.fork;
  QCOMPARE(fork_->presetLabels(), f.presetLabels);
  fork_->setTarget(0.1);  // ilk preset (0.000) başlangıç hedefiyle aynı olmasın
  port_->clearRecords();
  for (int i = 0; i < f.presetHeights.size(); ++i) {
    fork_->applyPreset(i);
    QCOMPARE(fork_->target(), f.presetHeights.at(i));
  }
  fork_->applyPreset(-1);
  fork_->applyPreset(static_cast<int>(f.presetHeights.size()));
  QCOMPARE(targets().size(), static_cast<size_t>(f.presetHeights.size()));

  fork_->applyPreset(2);  // Taşıma 0.060
  port_->clearRecords();
  fork_->stepUp();
  fork_->stepUp();
  fork_->stepDown();
  const auto t = targets();
  QCOMPARE(t.size(), size_t(3));
  QVERIFY(std::abs(t[0] - (f.presetHeights.at(2) + f.keyStep)) < 1e-12);
  QVERIFY(std::abs(t[1] - (f.presetHeights.at(2) + 2 * f.keyStep)) < 1e-12);
  QVERIFY(std::abs(t[2] - (f.presetHeights.at(2) + f.keyStep)) < 1e-12);
  // Üst sınırda adım kırpılır, aynı hedef yeniden yayınlanmaz.
  fork_->setTarget(f.maxHeight);
  port_->clearRecords();
  fork_->stepUp();
  QVERIFY(targets().empty());
}

void TestFork::noCommandWhenDisabled()
{
  fork_->setEnabled(false);
  fork_->setTarget(0.1);
  fork_->applyPreset(1);
  fork_->stepUp();
  QVERIFY(targets().empty());
}

void TestFork::resyncAfterGap()
{
  fork_->setTarget(0.06);
  measure(0.06);
  port_->clearRecords();
  // Sim yeniden başladı: resync_after'dan uzun boşluk, çatal 0'da.
  clock_.advance(cfg_.fork.resyncAfter + 0.5);
  measure(0.0);
  QCOMPARE(fork_->target(), 0.0);
  QVERIFY(fork_->reached());
  QVERIFY(targets().empty());  // eski hedef yeni sime itilmez
}

QTEST_MAIN(TestFork)
#include "test_fork.moc"
