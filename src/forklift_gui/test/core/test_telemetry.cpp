// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// Ek: telemetri kısma (§2.2: QML bildirimleri ≤ ui_max_rate), RTF, mesafe,
// iz ve sim yeniden başlama algısı.

#include <QSignalSpy>
#include <QtTest>
#include <cmath>
#include <memory>

#include "TestSupport.hpp"
#include "forklift_gui/core/TelemetryModel.hpp"

using forklift_gui::GuiConfig;
using forklift_gui::OdomSample;
using forklift_gui::TelemetryModel;
using forklift_gui::test::FakeClock;

class TestTelemetry : public QObject
{
  Q_OBJECT

private slots:
  void init();
  void cleanup();
  void throttle();
  void rtf();
  void distanceAndTrail();
  void simRestart();

private:
  GuiConfig cfg_;
  FakeClock clock_;
  std::unique_ptr<TelemetryModel> model_;
};

void TestTelemetry::init()
{
  cfg_ = forklift_gui::test::loadTestConfig();
  clock_ = FakeClock();
  model_ = std::make_unique<TelemetryModel>(cfg_.telemetry, clock_.fn());
}

void TestTelemetry::cleanup() { model_.reset(); }

// 50 Hz odometri ve 1 kHz tık: bildirim sayısı ui_max_rate'i aşmaz.
void TestTelemetry::throttle()
{
  QSignalSpy changed(model_.get(), &TelemetryModel::changed);
  const double seconds = 2.0;
  double nextOdom = 0.0;
  for (double t = 0.0; t < seconds; t += 0.001) {
    clock_.advance(0.001);
    if (t >= nextOdom) {
      OdomSample s;
      s.x = t;
      s.stamp = t;
      model_->onOdometry(s);
      nextOdom += 1.0 / 50.0;
    }
    model_->tick(clock_.now);
  }
  const double rate = changed.count() / seconds;
  QVERIFY2(rate <= cfg_.telemetry.uiMaxRate + 0.5, qPrintable(QString::number(rate)));
  QVERIFY2(rate >= cfg_.telemetry.uiMaxRate * 0.9, qPrintable(QString::number(rate)));
  QVERIFY(std::abs(model_->x() - (seconds - 0.02)) < 0.05);  // son değer yansıyor
}

void TestTelemetry::rtf()
{
  // Sim, duvar saatinin yarı hızında: RTF ≈ 0.5.
  double sim = 0.0;
  for (int i = 0; i < 100; ++i) {
    clock_.advance(1.0 / 30.0);
    sim += 0.5 / 30.0;
    model_->onClock(sim, clock_.now);
    model_->tick(clock_.now);
  }
  QVERIFY(model_->rtfValid());
  QVERIFY(std::abs(model_->rtf() - 0.5) < 0.01);
  QVERIFY(std::abs(model_->simTime() - sim) < 0.02);
  // Saat durursa RTF geçersizleşir.
  clock_.advance(cfg_.telemetry.rtfWindow + 0.5);
  model_->tick(clock_.now);
  QVERIFY(!model_->rtfValid());
}

void TestTelemetry::distanceAndTrail()
{
  // 1 m ileri, 1 m sola: mesafe 2 m; iz trail_min_step aralıklı.
  for (int i = 0; i <= 100; ++i) {
    OdomSample s;
    s.x = i * 0.01;
    s.stamp = i * 0.02;
    model_->onOdometry(s);
  }
  for (int i = 1; i <= 100; ++i) {
    OdomSample s;
    s.x = 1.0;
    s.y = i * 0.01;
    s.stamp = 2.0 + i * 0.02;
    model_->onOdometry(s);
  }
  clock_.advance(1.0);
  model_->tick(clock_.now);
  QVERIFY(std::abs(model_->distance() - 2.0) < 1e-9);
  QVERIFY(model_->trail().size() > 150);
  QVERIFY(model_->trail().size() <= cfg_.telemetry.trailMaxPoints);
  const QRectF b = model_->trailBounds();
  QVERIFY(std::abs(b.left()) < 1e-9 && std::abs(b.right() - 1.0) < 1e-9);
  QVERIFY(std::abs(b.top()) < 1e-9 && std::abs(b.bottom() - 1.0) < 1e-9);
}

void TestTelemetry::simRestart()
{
  OdomSample s;
  s.x = 2.0;
  s.stamp = 50.0;
  model_->onOdometry(s);
  s.x = 2.5;
  s.stamp = 51.0;
  model_->onOdometry(s);
  // Sim yeniden başladı: zaman geri gitti, odometri sıfırda.
  s.x = 0.0;
  s.stamp = 0.1;
  model_->onOdometry(s);
  clock_.advance(1.0);
  model_->tick(clock_.now);
  QCOMPARE(model_->distance(), 0.0);  // sıçrama mesafeye eklenmedi
  QCOMPARE(model_->trail().size(), 1);
}

QTEST_MAIN(TestTelemetry)
#include "test_telemetry.moc"
