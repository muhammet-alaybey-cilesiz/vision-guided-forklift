// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__TOPICHEALTHMODEL_HPP_
#define FORKLIFT_GUI__CORE__TOPICHEALTHMODEL_HPP_

#include <QtQmlIntegration/qqmlintegration.h>

#include <QAbstractListModel>
#include <QHash>
#include <QTimer>
#include <QVector>
#include <deque>
#include <utility>

#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/core/Types.hpp"
#include "forklift_gui/core/ports/IRobotStatePort.hpp"

namespace forklift_gui
{

// Topic başına sağlık: ölçülen frekans, son mesaj yaşı ve durum (D-35).
//   Bekleniyor (gri): henüz mesaj yok
//   Sarı          : frekans < warn_rate_ratio × beklenen ya da yaş > warn_age
//   Kırmızı       : yaş > error_age
// Sayaçları porttan health.update_rate ile okur (ROS thread'ine dokunmadan).
class TopicHealthModel : public QAbstractListModel
{
  Q_OBJECT
  QML_ELEMENT
  QML_UNCREATABLE("Backend.health üzerinden erişilir")

  Q_PROPERTY(int overallState READ overallState NOTIFY summaryChanged)
  Q_PROPERTY(int simState READ simState NOTIFY summaryChanged)
  Q_PROPERTY(int rosState READ rosState NOTIFY summaryChanged)
  Q_PROPERTY(bool mock READ mock NOTIFY summaryChanged)

public:
  enum State
  {
    Waiting = 0,
    Ok = 1,
    Warn = 2,
    Error = 3
  };
  Q_ENUM(State)

  enum Role
  {
    KeyRole = Qt::UserRole + 1,
    TopicRole,
    StatusRole,
    RateRole,
    AgeRole,
    ExpectedRateRole,
    EssentialRole
  };

  TopicHealthModel(
    const GuiConfig & config, IRobotStatePort * state, ClockFn clock, QObject * parent = nullptr);

  int rowCount(const QModelIndex & parent = QModelIndex()) const override;
  QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const override;
  QHash<int, QByteArray> roleNames() const override;

  int overallState() const { return overall_; }
  int simState() const { return simState_; }
  int rosState() const { return rosState_; }
  bool mock() const { return graph_.mock; }

  // Test ve özet için: anahtara göre durum, frekans [Hz], yaş [s].
  State stateOf(const QString & key) const;
  double rateOf(const QString & key) const;

  void tick(qint64 nowNs);
  void onGraphStatus(const forklift_gui::GraphStatus & status);
  void start();
  void stop();

signals:
  void summaryChanged();
  void event(int level, const QString & text);

private:
  struct Row
  {
    HealthTopicConfig config;
    State state{Waiting};
    double rate{0.0};
    double age{-1.0};
    // Frekans penceresi: (okuma anı [ns], sayaç)
    std::deque<std::pair<qint64, quint64>> history;
  };

  State evaluate(const Row & row, const TopicCounter & counter, qint64 nowNs) const;
  void updateSummary();

  GuiConfig::Health config_;
  IRobotStatePort * state_;
  ClockFn clock_;
  QTimer timer_;
  QVector<Row> rows_;
  QString clockKey_;
  GraphStatus graph_;
  int overall_{Waiting};
  int simState_{Waiting};
  int rosState_{Waiting};
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__TOPICHEALTHMODEL_HPP_
