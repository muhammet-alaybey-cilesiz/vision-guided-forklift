// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/LogModel.hpp"

#include <QDateTime>
#include <algorithm>

namespace forklift_gui
{

LogModel::LogModel(int capacity, QObject * parent) : QAbstractListModel(parent), capacity_(capacity)
{
}

int LogModel::rowCount(const QModelIndex & parent) const { return parent.isValid() ? 0 : count(); }

QVariant LogModel::data(const QModelIndex & index, int role) const
{
  if (!index.isValid() || index.row() < 0 || index.row() >= count()) {
    return {};
  }
  const LogEntry & e = entries_.at(static_cast<size_t>(index.row()));
  switch (role) {
    case TimeRole:
      // /rosout damgası sim zamanında olabilir; GUI'ye varış anı gösterilir.
      return QDateTime::fromMSecsSinceEpoch(e.wallMs).toString(QStringLiteral("HH:mm:ss.zzz"));
    case LevelRole:
      return e.level;
    case LevelNameRole:
      return levelName(e.level);
    case NodeRole:
      return e.node;
    case Qt::DisplayRole:
    case MessageRole:
      return e.text;
    case LocalRole:
      return e.local;
    default:
      return {};
  }
}

QHash<int, QByteArray> LogModel::roleNames() const
{
  return {
    {TimeRole, "time"}, {LevelRole, "level"},     {LevelNameRole, "levelName"},
    {NodeRole, "node"}, {MessageRole, "message"}, {LocalRole, "local"},
  };
}

void LogModel::append(const QVector<LogEntry> & entries)
{
  if (entries.isEmpty()) {
    return;
  }
  // Kapasiteyi aşacak kadar gelirse yalnız son 'capacity' kayıt alınır.
  const int incoming = static_cast<int>(std::min<qsizetype>(entries.size(), capacity_));
  const qsizetype skip = entries.size() - incoming;

  const int overflow = count() + incoming - capacity_;
  if (overflow > 0) {
    beginRemoveRows(QModelIndex(), 0, overflow - 1);
    entries_.erase(entries_.begin(), entries_.begin() + overflow);
    endRemoveRows();
  }
  beginInsertRows(QModelIndex(), count(), count() + incoming - 1);
  for (qsizetype i = skip; i < entries.size(); ++i) {
    entries_.push_back(entries.at(i));
  }
  endInsertRows();
  emit countChanged();
}

void LogModel::addLocal(int level, const QString & text)
{
  LogEntry e;
  e.wallMs = QDateTime::currentMSecsSinceEpoch();
  e.stamp = static_cast<double>(e.wallMs) / 1000.0;
  e.level = level;
  e.node = QStringLiteral("gui");
  e.text = text;
  e.local = true;
  append({e});
}

void LogModel::clear()
{
  if (entries_.empty()) {
    return;
  }
  beginResetModel();
  entries_.clear();
  endResetModel();
  emit countChanged();
}

const LogEntry & LogModel::entryAt(int row) const { return entries_.at(static_cast<size_t>(row)); }

QString LogModel::levelName(int level)
{
  if (level >= static_cast<int>(LogLevel::Fatal)) {
    return QStringLiteral("FATAL");
  }
  if (level >= static_cast<int>(LogLevel::Error)) {
    return QStringLiteral("ERROR");
  }
  if (level >= static_cast<int>(LogLevel::Warn)) {
    return QStringLiteral("WARN");
  }
  if (level >= static_cast<int>(LogLevel::Info)) {
    return QStringLiteral("INFO");
  }
  return QStringLiteral("DEBUG");
}

LogFilterModel::LogFilterModel(LogModel * source, QObject * parent)
: QSortFilterProxyModel(parent), source_(source)
{
  setSourceModel(source);
}

void LogFilterModel::setMinLevel(int level)
{
  if (level == minLevel_) {
    return;
  }
  minLevel_ = level;
  invalidateFilter();
  emit minLevelChanged();
}

void LogFilterModel::setSearch(const QString & text)
{
  if (text == search_) {
    return;
  }
  search_ = text;
  invalidateFilter();
  emit searchChanged();
}

bool LogFilterModel::filterAcceptsRow(int sourceRow, const QModelIndex & sourceParent) const
{
  Q_UNUSED(sourceParent)
  const LogEntry & e = source_->entryAt(sourceRow);
  if (e.level < minLevel_) {
    return false;
  }
  if (search_.isEmpty()) {
    return true;
  }
  return e.text.contains(search_, Qt::CaseInsensitive) ||
         e.node.contains(search_, Qt::CaseInsensitive);
}

}  // namespace forklift_gui
