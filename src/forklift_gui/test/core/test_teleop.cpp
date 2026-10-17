// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// U1–U4: hız limiti ve ölçek, rampa, ölü adam, odak kaybı.

#include <QtTest>
#include <cmath>
#include <vector>

#include "TestSupport.hpp"
#include "forklift_gui/core/TeleopController.hpp"
#include "forklift_gui/mock/MockRobotAdapter.hpp"

using forklift_gui::GuiConfig;
using forklift_gui::TeleopController;
using forklift_gui::mock::CommandRecord;
using forklift_gui::mock::MockCommandPort;
using forklift_gui::test::FakeClock;
using Direction = TeleopController::Direction;

namespace
{

constexpr double kEps = 1e-9;

struct Twist
{
  qint64 t;
  double linear;
  double angular;
};

std::vector<Twist> twists(const MockCommandPort & port)
{
  std::vector<Twist> out;
  for (const CommandRecord & r : port.records()) {
    if (r.kind == CommandRecord::Kind::Velocity) {
      out.push_back({r.stampNs, r.a, r.b});
    }
  }
  return out;
}

}  // namespace

class TestTeleop : public QObject
{
  Q_OBJECT

private slots:
  void init();
  void cleanup();
  void speedLimitAndScale();  // U1
  void ramp();                // U2
  void deadman();             // U3
  void focusLoss();           // U4
  void noOutputWhenDisabled();

private:
  // publish_rate periyoduyla 'seconds' boyunca tick.
  void run(double seconds);

  GuiConfig cfg_;
  FakeClock clock_;
  MockCommandPort * port_{nullptr};
  TeleopController * teleop_{nullptr};
};

void TestTeleop::init()
{
  cfg_ = forklift_gui::test::loadTestConfig();
  clock_ = FakeClock();
  port_ = new MockCommandPort(clock_.fn());
  teleop_ = new TeleopController(cfg_.teleop, port_, clock_.fn());
  teleop_->setEnabled(true);  // e-stop serbest
}

void TestTeleop::cleanup()
{
  delete teleop_;
  delete port_;
}

void TestTeleop::run(double seconds)
{
  const double dt = 1.0 / cfg_.teleop.publishRate;
  const int steps = static_cast<int>(std::lround(seconds / dt));
  for (int i = 0; i < steps; ++i) {
    clock_.advance(dt);
    teleop_->tick(clock_.now);
  }
}

// U1: komut her koşulda limit × ölçek içinde; kararlı durumda tam o değer.
void TestTeleop::speedLimitAndScale()
{
  const auto & t = cfg_.teleop;
  QVERIFY(t.maxLinear > 0.0 && t.maxAngular > 0.0);

  // Ölçek sınırlara kırpılır.
  teleop_->setSpeedScale(10.0);
  QCOMPARE(teleop_->speedScale(), t.scaleMax);
  teleop_->setSpeedScale(0.0);
  QCOMPARE(teleop_->speedScale(), t.scaleMin);

  struct Input
  {
    const char * name;
    bool keys[4];  // ileri, geri, sol, sağ
    bool joystick;
    double jx;
    double jy;
    double expectLinear;  // limit birimi
    double expectAngular;
  };
  const double diag = std::sqrt(0.5);
  const Input inputs[] = {
    {"W", {true, false, false, false}, false, 0, 0, 1.0, 0.0},
    {"W+A", {true, false, true, false}, false, 0, 0, 1.0, 1.0},
    {"S+D", {false, true, false, true}, false, 0, 0, -1.0, -1.0},
    {"joy köşegen (disk dışı)", {false, false, false, false}, true, 5.0, 5.0, diag, -diag},
    {"joy ileri", {false, false, false, false}, true, 0.0, 1.0, 1.0, 0.0},
  };
  for (const double scale : {t.scaleMin, 0.5, t.scaleMax}) {
    for (const Input & in : inputs) {
      teleop_->setSpeedScale(scale);
      port_->clearRecords();
      const double linLimit = t.maxLinear * scale;
      const double angLimit = t.maxAngular * scale;
      if (in.joystick) {
        teleop_->setJoystick(in.jx, in.jy);
      } else {
        for (int k = 0; k < 4; ++k) {
          teleop_->setKeyState(static_cast<Direction>(k), in.keys[k]);
        }
      }
      run(3.0);  // rampa bitmiş olsun (0.3/0.5 = 0.6 s)
      const auto out = twists(*port_);
      QVERIFY2(!out.empty(), in.name);
      for (const Twist & tw : out) {
        QVERIFY2(std::abs(tw.linear) <= linLimit + kEps, in.name);
        QVERIFY2(std::abs(tw.angular) <= angLimit + kEps, in.name);
      }
      QVERIFY2(std::abs(out.back().linear - in.expectLinear * linLimit) < 1e-6, in.name);
      QVERIFY2(std::abs(out.back().angular - in.expectAngular * angLimit) < 1e-6, in.name);
      // Bırak (ölü adam) ve bir sonraki girdiye temiz başla.
      for (int k = 0; k < 4; ++k) {
        teleop_->setKeyState(static_cast<Direction>(k), false);
      }
      teleop_->releaseJoystick();
      QVERIFY(!teleop_->active());
    }
  }

  // Sürerken ölçek düşürülürse bir sonraki komut yeni zarfın içindedir.
  teleop_->setSpeedScale(t.scaleMax);
  teleop_->setKeyState(Direction::Forward, true);
  run(3.0);
  QVERIFY(std::abs(teleop_->linearCommand() - t.maxLinear * t.scaleMax) < 1e-6);
  teleop_->setSpeedScale(t.scaleMin);
  port_->clearRecords();
  run(0.1);
  for (const Twist & tw : twists(*port_)) {
    QVERIFY(std::abs(tw.linear) <= t.maxLinear * t.scaleMin + kEps);
  }
}

// U2: ardışık komutlar arasındaki değişim ivme limitini aşmaz (hızlanma,
// yavaşlama ve yön değişimi; ölü adam etkin kaldıkça).
void TestTeleop::ramp()
{
  const auto & t = cfg_.teleop;
  teleop_->setSpeedScale(t.scaleMax);
  const qint64 t0 = clock_.now;
  // A basılı kalır ki W→S geçişinde ölü adam bırakılmış sayılmasın.
  teleop_->setKeyState(Direction::Left, true);
  teleop_->setKeyState(Direction::Forward, true);
  run(2.0);
  teleop_->setKeyState(Direction::Forward, false);
  teleop_->setKeyState(Direction::Backward, true);
  run(2.0);

  const auto out = twists(*port_);
  QVERIFY(out.size() > 50);
  // İlk komut duruştan: en fazla bir nominal periyotluk adım.
  const double dt0 = 1.0 / t.publishRate;
  QVERIFY(std::abs(out.front().linear) <= t.maxLinearAccel * dt0 + kEps);
  QVERIFY(std::abs(out.front().angular) <= t.maxAngularAccel * dt0 + kEps);
  QVERIFY(out.front().t == t0);
  bool sawReversal = false;
  for (size_t i = 1; i < out.size(); ++i) {
    const double dt = forklift_gui::nsToSeconds(out[i].t - out[i - 1].t);
    QVERIFY(dt > 0.0);
    QVERIFY2(
      std::abs(out[i].linear - out[i - 1].linear) <= t.maxLinearAccel * dt + kEps,
      qPrintable(QStringLiteral("doğrusal adım %1").arg(i)));
    QVERIFY2(
      std::abs(out[i].angular - out[i - 1].angular) <= t.maxAngularAccel * dt + kEps,
      qPrintable(QStringLiteral("açısal adım %1").arg(i)));
    if (out[i - 1].linear > 0.0 && out[i].linear < 0.0) {
      sawReversal = true;
    }
  }
  QVERIFY(sawReversal);
  // Yön değişiminden sonra hedefe (−limit) ulaşıldı.
  QVERIFY(std::abs(out.back().linear + t.maxLinear * t.scaleMax) < 1e-6);
}

// U3: tuş bırakılınca tek sıfır komutu gider, ardından yayın durur.
void TestTeleop::deadman()
{
  // Girdi yokken hiçbir şey yayınlanmaz.
  run(1.0);
  QVERIFY(twists(*port_).empty());

  teleop_->setKeyState(Direction::Forward, true);
  run(1.0);
  const size_t whileHeld = twists(*port_).size();
  QVERIFY(whileHeld >= static_cast<size_t>(cfg_.teleop.publishRate) - 1);
  QVERIFY(teleop_->linearCommand() > 0.0);

  teleop_->setKeyState(Direction::Forward, false);
  // Sıfır, tick beklemeden hemen gönderilir.
  auto out = twists(*port_);
  QCOMPARE(out.size(), whileHeld + 1);
  QCOMPARE(out.back().linear, 0.0);
  QCOMPARE(out.back().angular, 0.0);
  QVERIFY(!teleop_->active());
  run(2.0);
  QCOMPARE(twists(*port_).size(), whileHeld + 1);  // sessizlik

  // Sanal joystick için aynı kural.
  port_->clearRecords();
  teleop_->setJoystick(0.0, 1.0);
  run(0.5);
  QVERIFY(!twists(*port_).empty());
  const size_t held = twists(*port_).size();
  teleop_->releaseJoystick();
  out = twists(*port_);
  QCOMPARE(out.size(), held + 1);
  QCOMPARE(out.back().linear, 0.0);
  run(1.0);
  QCOMPARE(twists(*port_).size(), held + 1);
}

// U4: pencere odağı kaybedilince komut anında sıfır; tuşlar bırakılmış sayılır.
void TestTeleop::focusLoss()
{
  teleop_->setKeyState(Direction::Forward, true);
  teleop_->setKeyState(Direction::Left, true);
  run(1.0);
  QVERIFY(teleop_->active());
  const size_t before = twists(*port_).size();

  teleop_->setWindowActive(false);
  auto out = twists(*port_);
  QCOMPARE(out.size(), before + 1);
  QCOMPARE(out.back().linear, 0.0);
  QCOMPARE(out.back().angular, 0.0);
  QVERIFY(!teleop_->active());
  run(1.0);
  QCOMPARE(twists(*port_).size(), before + 1);

  // Odak geri gelince takılı tuş yüzünden kendiliğinden hareket olmaz.
  teleop_->setWindowActive(true);
  run(1.0);
  QCOMPARE(twists(*port_).size(), before + 1);
  QVERIFY(!teleop_->active());
}

// E-stop'ta (enabled=false) hiçbir girdi komut üretmez.
void TestTeleop::noOutputWhenDisabled()
{
  teleop_->setEnabled(false);
  teleop_->setKeyState(Direction::Forward, true);
  teleop_->setJoystick(0.0, 1.0);
  run(1.0);
  QVERIFY(twists(*port_).empty());
  QVERIFY(!teleop_->active());
}

QTEST_MAIN(TestTeleop)
#include "test_teleop.moc"
