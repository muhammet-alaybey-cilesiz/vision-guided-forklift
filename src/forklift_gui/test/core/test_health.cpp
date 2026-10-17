// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// U7: topic sağlığı. Mesaj kesilince durum eşiklere göre önce sarı, sonra
// kırmızı olur. Ek: bekleniyor, düşük frekans uyarısı, özet ve sim durumu.

#include <QtTest>
#include <memory>

#include "TestSupport.hpp"
#include "forklift_gui/core/TopicHealthModel.hpp"
#include "forklift_gui/mock/MockRobotAdapter.hpp"

using forklift_gui::GuiConfig;
using forklift_gui::HealthTopicConfig;
using forklift_gui::TopicHealthModel;
using forklift_gui::mock::MockStatePort;
using forklift_gui::test::FakeClock;

class TestHealth : public QObject
{
  Q_OBJECT

private slots:
  void init();
  void cleanup();
  void staleness();  // U7
  void lowRateWarns();
  void summary();

private:
  const HealthTopicConfig & topic(const QString & key) const;
  // 'key' topic'ini 'rate' Hz ile 'seconds' boyunca besler; sağlık modelini
  // update_rate ile tıklatır.
  void feed(const QString & key, double rate, double seconds);
  void idle(double seconds);

  GuiConfig cfg_;
  FakeClock clock_;
  std::unique_ptr<MockStatePort> state_;
  std::unique_ptr<TopicHealthModel> model_;
  quint64 counts_[32]{};
};

void TestHealth::init()
{
  cfg_ = forklift_gui::test::loadTestConfig();
  clock_ = FakeClock();
  state_ = std::make_unique<MockStatePort>();
  model_ = std::make_unique<TopicHealthModel>(cfg_, state_.get(), clock_.fn());
  std::fill(std::begin(counts_), std::end(counts_), 0);
}

void TestHealth::cleanup()
{
  model_.reset();
  state_.reset();
}

const HealthTopicConfig & TestHealth::topic(const QString & key) const
{
  for (const auto & h : cfg_.health.topics) {
    if (h.key == key) {
      return h;
    }
  }
  qFatal("bilinmeyen sağlık anahtarı");
}

void TestHealth::feed(const QString & key, double rate, double seconds)
{
  const double tick = 1.0 / cfg_.health.updateRate;
  const double step = 1.0 / rate;
  int index = 0;
  for (int i = 0; i < cfg_.health.topics.size(); ++i) {
    if (cfg_.health.topics[i].key == key) {
      index = i;
    }
  }
  double nextTick = tick;
  for (double t = 0.0; t < seconds; t += step) {
    clock_.advance(step);
    ++counts_[index];
    state_->setCounter(key, counts_[index], clock_.now);
    if (t >= nextTick) {
      model_->tick(clock_.now);
      nextTick += tick;
    }
  }
  model_->tick(clock_.now);
}

void TestHealth::idle(double seconds)
{
  clock_.advance(seconds);
  model_->tick(clock_.now);
}

void TestHealth::staleness()
{
  const auto & odom = topic(QStringLiteral("odom"));
  QVERIFY(odom.warnAge > 0.0 && odom.errorAge > odom.warnAge);

  // Mesaj yokken "bekleniyor".
  model_->tick(clock_.now);
  QCOMPARE(model_->stateOf(QStringLiteral("odom")), TopicHealthModel::Waiting);

  // Beklenen frekansta akarken normal ve frekans ölçülüyor.
  feed(QStringLiteral("odom"), odom.expectedRate, 3.0);
  QCOMPARE(model_->stateOf(QStringLiteral("odom")), TopicHealthModel::Ok);
  QVERIFY(std::abs(model_->rateOf(QStringLiteral("odom")) - odom.expectedRate) < 2.0);

  // Kesildi: warn_age altında hâlâ normal, aşınca sarı, error_age aşınca kırmızı.
  idle(odom.warnAge * 0.5);
  QCOMPARE(model_->stateOf(QStringLiteral("odom")), TopicHealthModel::Ok);
  idle(odom.warnAge);  // toplam 1.5 × warn_age
  QCOMPARE(model_->stateOf(QStringLiteral("odom")), TopicHealthModel::Warn);
  idle(odom.errorAge);
  QCOMPARE(model_->stateOf(QStringLiteral("odom")), TopicHealthModel::Error);

  // Akış geri gelince normale döner.
  feed(QStringLiteral("odom"), odom.expectedRate, 2.0);
  QCOMPARE(model_->stateOf(QStringLiteral("odom")), TopicHealthModel::Ok);
}

void TestHealth::lowRateWarns()
{
  const auto & odom = topic(QStringLiteral("odom"));
  // Beklenenin warn_rate_ratio'sunun altında (yaş taze kalsa da) sarı.
  const double slow = odom.expectedRate * cfg_.health.warnRateRatio * 0.6;
  feed(QStringLiteral("odom"), slow, 3.0);
  // Yaş uyarısı karışmasın diye bu test yalnız oranı denetler.
  QVERIFY(1.0 / slow < odom.errorAge);
  QCOMPARE(model_->stateOf(QStringLiteral("odom")), TopicHealthModel::Warn);
}

void TestHealth::summary()
{
  // Hiçbir şey yokken genel durum "bekleniyor", sim de öyle.
  model_->tick(clock_.now);
  QCOMPARE(model_->overallState(), static_cast<int>(TopicHealthModel::Waiting));
  QCOMPARE(model_->simState(), static_cast<int>(TopicHealthModel::Waiting));

  // Yalnız saat akıyor: sim normal, genel hâlâ bekleniyor (diğer zorunlular).
  feed(QStringLiteral("clock"), 1000.0, 2.0);
  QCOMPARE(model_->simState(), static_cast<int>(TopicHealthModel::Ok));
  QCOMPARE(model_->overallState(), static_cast<int>(TopicHealthModel::Waiting));

  // Opsiyonel topic'ler (rosout, detections) genel durumu etkilemez.
  for (const auto & h : cfg_.health.topics) {
    QVERIFY2(h.required || h.expectedRate == 0.0, qPrintable(h.key));
  }
}

QTEST_MAIN(TestHealth)
#include "test_health.moc"
