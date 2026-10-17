// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// Ek: log modeli kapasitesi (son N kayıt), seviye filtresi ve metin araması.

#include <QtTest>

#include "TestSupport.hpp"
#include "forklift_gui/core/LogModel.hpp"

using forklift_gui::LogEntry;
using forklift_gui::LogFilterModel;
using forklift_gui::LogLevel;
using forklift_gui::LogModel;

namespace
{

LogEntry entry(int level, const QString & node, const QString & text)
{
  LogEntry e;
  e.level = level;
  e.node = node;
  e.text = text;
  return e;
}

}  // namespace

class TestLog : public QObject
{
  Q_OBJECT

private slots:
  void capacity();
  void filter();
};

void TestLog::capacity()
{
  const int cap = forklift_gui::test::loadTestConfig().log.maxEntries;
  LogModel model(cap);
  QVector<LogEntry> batch;
  for (int i = 0; i < cap + 37; ++i) {
    batch << entry(20, QStringLiteral("n"), QString::number(i));
  }
  model.append(batch.mid(0, 10));
  model.append(batch.mid(10));
  QCOMPARE(model.rowCount(), cap);
  // En eski kayıtlar düştü; sonuncusu en sonda.
  QCOMPARE(model.entryAt(0).text, QString::number(37));
  QCOMPARE(model.entryAt(cap - 1).text, QString::number(cap + 36));
  model.addLocal(static_cast<int>(LogLevel::Warn), QStringLiteral("E-STOP etkin"));
  QCOMPARE(model.rowCount(), cap);
  QVERIFY(model.entryAt(cap - 1).local);
  QCOMPARE(model.entryAt(cap - 1).node, QStringLiteral("gui"));
}

void TestLog::filter()
{
  LogModel model(100);
  model.append({
    entry(10, QStringLiteral("cv"), QStringLiteral("debug satırı")),
    entry(20, QStringLiteral("sim"), QStringLiteral("Simülasyon başladı")),
    entry(30, QStringLiteral("cv"), QStringLiteral("Şerit kontrastı düşük")),
    entry(40, QStringLiteral("bridge"), QStringLiteral("Bağlantı koptu")),
  });
  LogFilterModel view(&model);
  QCOMPARE(view.rowCount(), 4);
  view.setMinLevel(30);
  QCOMPARE(view.rowCount(), 2);
  view.setMinLevel(0);
  view.setSearch(QStringLiteral("CV"));  // büyük/küçük harf duyarsız, düğüm adında
  QCOMPARE(view.rowCount(), 2);
  view.setSearch(QStringLiteral("bağlantı"));
  QCOMPARE(view.rowCount(), 1);
  view.setMinLevel(50);
  QCOMPARE(view.rowCount(), 0);
}

QTEST_MAIN(TestLog)
#include "test_log.moc"
