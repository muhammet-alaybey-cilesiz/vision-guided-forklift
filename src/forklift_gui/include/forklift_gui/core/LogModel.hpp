// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__LOGMODEL_HPP_
#define FORKLIFT_GUI__CORE__LOGMODEL_HPP_

#include <QtQmlIntegration/qqmlintegration.h>

#include <QAbstractListModel>
#include <QSortFilterProxyModel>
#include <QString>
#include <QVector>
#include <deque>

#include "forklift_gui/core/Types.hpp"

namespace forklift_gui
{

// /rosout kayıtları ve GUI'nin kendi olayları; son max_entries kayıt tutulur.
class LogModel : public QAbstractListModel
{
  Q_OBJECT
  QML_ELEMENT
  QML_UNCREATABLE("Backend.log üzerinden erişilir")

  Q_PROPERTY(int count READ count NOTIFY countChanged)
  Q_PROPERTY(int capacity READ capacity CONSTANT)

public:
  enum Role
  {
    TimeRole = Qt::UserRole + 1,
    LevelRole,
    LevelNameRole,
    NodeRole,
    MessageRole,
    LocalRole
  };

  explicit LogModel(int capacity, QObject * parent = nullptr);

  int rowCount(const QModelIndex & parent = QModelIndex()) const override;
  QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const override;
  QHash<int, QByteArray> roleNames() const override;

  int count() const { return static_cast<int>(entries_.size()); }
  int capacity() const { return capacity_; }

  void append(const QVector<forklift_gui::LogEntry> & entries);
  // GUI olayı; node adı yerel olarak "gui" gösterilir.
  void addLocal(int level, const QString & text);
  Q_INVOKABLE void clear();

  const forklift_gui::LogEntry & entryAt(int row) const;

  // Seviye adı: DEBUG, INFO, WARN, ERROR, FATAL.
  static QString levelName(int level);

signals:
  void countChanged();

private:
  int capacity_;
  std::deque<LogEntry> entries_;
};

// Seviye filtresi ve metin araması (büyük/küçük harf duyarsız, düğüm + mesaj).
class LogFilterModel : public QSortFilterProxyModel
{
  Q_OBJECT
  QML_ELEMENT
  QML_UNCREATABLE("Backend.logView üzerinden erişilir")

  Q_PROPERTY(int minLevel READ minLevel WRITE setMinLevel NOTIFY minLevelChanged)
  Q_PROPERTY(QString search READ search WRITE setSearch NOTIFY searchChanged)

public:
  explicit LogFilterModel(LogModel * source, QObject * parent = nullptr);

  int minLevel() const { return minLevel_; }
  void setMinLevel(int level);
  QString search() const { return search_; }
  void setSearch(const QString & text);

signals:
  void minLevelChanged();
  void searchChanged();

protected:
  bool filterAcceptsRow(int sourceRow, const QModelIndex & sourceParent) const override;

private:
  LogModel * source_;
  int minLevel_{0};
  QString search_;
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__LOGMODEL_HPP_
