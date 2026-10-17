// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/TopicHealthModel.hpp"

#include <cmath>
#include <utility>

namespace forklift_gui
{

namespace
{

// Özet için önem sırası: Ok < Bekleniyor < Sarı < Kırmızı.
int severity(int state)
{
  switch (state) {
    case TopicHealthModel::Ok:
      return 0;
    case TopicHealthModel::Waiting:
      return 1;
    case TopicHealthModel::Warn:
      return 2;
    default:
      return 3;
  }
}

QString stateName(int state)
{
  switch (state) {
    case TopicHealthModel::Ok:
      return QObject::tr("normal");
    case TopicHealthModel::Waiting:
      return QObject::tr("bekleniyor");
    case TopicHealthModel::Warn:
      return QObject::tr("uyarı");
    default:
      return QObject::tr("hata");
  }
}

}  // namespace

TopicHealthModel::TopicHealthModel(
  const GuiConfig & config, IRobotStatePort * state, ClockFn clock, QObject * parent)
: QAbstractListModel(parent), config_(config.health), state_(state), clock_(std::move(clock))
{
  rows_.reserve(config_.topics.size());
  for (const auto & topic : config_.topics) {
    Row row;
    row.config = topic;
    rows_.push_back(row);
    if (topic.topic == config.topics.clock) {
      clockKey_ = topic.key;
    }
  }
  timer_.setInterval(static_cast<int>(std::lround(1000.0 / config_.updateRate)));
  connect(&timer_, &QTimer::timeout, this, [this]() { tick(clock_()); });
}

int TopicHealthModel::rowCount(const QModelIndex & parent) const
{
  return parent.isValid() ? 0 : static_cast<int>(rows_.size());
}

QVariant TopicHealthModel::data(const QModelIndex & index, int role) const
{
  if (!index.isValid() || index.row() < 0 || index.row() >= rows_.size()) {
    return {};
  }
  const Row & row = rows_.at(index.row());
  switch (role) {
    case KeyRole:
      return row.config.key;
    case Qt::DisplayRole:
    case TopicRole:
      return row.config.topic;
    case StatusRole:
      return static_cast<int>(row.state);
    case RateRole:
      return row.rate;
    case AgeRole:
      return row.age;
    case ExpectedRateRole:
      return row.config.expectedRate;
    case EssentialRole:
      return row.config.required;
    default:
      return {};
  }
}

QHash<int, QByteArray> TopicHealthModel::roleNames() const
{
  return {
    {KeyRole, "key"},
    {TopicRole, "topic"},
    {StatusRole, "status"},
    {RateRole, "rate"},
    {AgeRole, "age"},
    {ExpectedRateRole, "expectedRate"},
    {EssentialRole, "essential"},
  };
}

TopicHealthModel::State TopicHealthModel::stateOf(const QString & key) const
{
  for (const Row & row : rows_) {
    if (row.config.key == key) {
      return row.state;
    }
  }
  return Waiting;
}

double TopicHealthModel::rateOf(const QString & key) const
{
  for (const Row & row : rows_) {
    if (row.config.key == key) {
      return row.rate;
    }
  }
  return 0.0;
}

void TopicHealthModel::tick(qint64 nowNs)
{
  QHash<QString, TopicCounter> counters;
  for (const TopicCounter & c : state_->topicCounters()) {
    counters.insert(c.key, c);
  }
  const qint64 windowNs = secondsToNs(config_.rateWindow);

  for (int i = 0; i < rows_.size(); ++i) {
    Row & row = rows_[i];
    const TopicCounter counter = counters.value(row.config.key);

    // Frekans: pencere başlangıcından hemen önceki okuma ile şimdiki arasında.
    row.history.emplace_back(nowNs, counter.count);
    while (row.history.size() > 2 && nowNs - row.history[1].first >= windowNs) {
      row.history.pop_front();
    }
    const auto & first = row.history.front();
    const qint64 spanNs = nowNs - first.first;
    row.rate =
      spanNs > 0 ? static_cast<double>(counter.count - first.second) / nsToSeconds(spanNs) : 0.0;
    row.age = counter.lastNs < 0 ? -1.0 : nsToSeconds(nowNs - counter.lastNs);

    const State next = evaluate(row, counter, nowNs);
    row.state = next;
    const QModelIndex idx = index(i);
    emit dataChanged(idx, idx, {StatusRole, RateRole, AgeRole});
  }
  updateSummary();
}

void TopicHealthModel::onGraphStatus(const GraphStatus & status)
{
  graph_ = status;
  updateSummary();
}

void TopicHealthModel::start() { timer_.start(); }

void TopicHealthModel::stop() { timer_.stop(); }

TopicHealthModel::State TopicHealthModel::evaluate(
  const Row & row, const TopicCounter & counter, qint64 nowNs) const
{
  if (counter.lastNs < 0) {
    return Waiting;
  }
  const HealthTopicConfig & cfg = row.config;
  const double age = nsToSeconds(nowNs - counter.lastNs);
  if (cfg.errorAge > 0.0 && age > cfg.errorAge) {
    return Error;
  }
  if (cfg.warnAge > 0.0 && age > cfg.warnAge) {
    return Warn;
  }
  // Frekans denetimi yalnız pencere dolduktan sonra (açılışta yanlış sarı olmasın).
  const bool windowFull =
    !row.history.empty() && nowNs - row.history.front().first >= secondsToNs(config_.rateWindow);
  if (cfg.expectedRate > 0.0 && windowFull && row.rate < cfg.expectedRate * config_.warnRateRatio) {
    return Warn;
  }
  return Ok;
}

void TopicHealthModel::updateSummary()
{
  int overall = Ok;
  for (const Row & row : rows_) {
    if (row.config.required && severity(row.state) > severity(overall)) {
      overall = row.state;
    }
  }
  const int sim = clockKey_.isEmpty() ? static_cast<int>(Waiting) : stateOf(clockKey_);
  int ros = Waiting;
  if (graph_.mock) {
    ros = Ok;
  } else if (!graph_.up) {
    ros = Error;
  } else if (graph_.inputsWithPublisher > 0) {
    ros = Ok;
  }

  const bool changed = overall != overall_ || sim != simState_ || ros != rosState_;
  if (sim != simState_) {
    emit event(
      static_cast<int>(sim == Ok ? LogLevel::Info : LogLevel::Warn),
      tr("Simülasyon saati (/clock): %1").arg(stateName(sim)));
  }
  if (overall != overall_) {
    emit event(
      static_cast<int>(overall == Ok ? LogLevel::Info : LogLevel::Warn),
      tr("Genel sağlık: %1").arg(stateName(overall)));
  }
  if (ros != rosState_) {
    emit event(
      static_cast<int>(ros == Ok ? LogLevel::Info : LogLevel::Warn),
      tr("ROS bağlantısı: %1").arg(stateName(ros)));
  }
  overall_ = overall;
  simState_ = sim;
  rosState_ = ros;
  if (changed) {
    emit summaryChanged();
  }
}

}  // namespace forklift_gui
