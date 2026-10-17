// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// U8: veri seti kaydı. Dosyalar oluşuyor; metadata.jsonl satırı doğru
// alanları içeriyor. Ek: eksik ya da eski karede kayıt atlanır.

#include <QFile>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QVideoFrameFormat>
#include <QtTest>
#include <cstring>
#include <memory>

#include "TestSupport.hpp"
#include "forklift_gui/core/Backend.hpp"
#include "forklift_gui/mock/MockRobotAdapter.hpp"

using forklift_gui::Backend;
using forklift_gui::FrameMeta;
using forklift_gui::GuiConfig;
using forklift_gui::JointSample;
using forklift_gui::OdomSample;
using forklift_gui::mock::MockRobotAdapter;
using forklift_gui::test::FakeClock;

namespace
{

// rgb8 kaynaklı bir kareyi taklit eder: düz renk RGBX8888.
QVideoFrame solidFrame(int width, int height, QRgb color)
{
  QVideoFrame frame(QVideoFrameFormat(QSize(width, height), QVideoFrameFormat::Format_RGBX8888));
  if (!frame.map(QVideoFrame::WriteOnly)) {
    return {};
  }
  const uchar px[4] = {
    static_cast<uchar>(qRed(color)), static_cast<uchar>(qGreen(color)),
    static_cast<uchar>(qBlue(color)), 255};
  for (int y = 0; y < height; ++y) {
    uchar * row = frame.bits(0) + y * frame.bytesPerLine(0);
    for (int x = 0; x < width; ++x) {
      std::memcpy(row + x * 4, px, 4);
    }
  }
  frame.unmap();
  return frame;
}

}  // namespace

class TestDataset : public QObject
{
  Q_OBJECT

private slots:
  void init();
  void cleanup();
  void metadata();  // U8
  void skipsWithoutFreshFrames();

private:
  void giveFrames();

  QTemporaryDir dir_;
  GuiConfig cfg_;
  FakeClock clock_;
  std::unique_ptr<MockRobotAdapter> adapter_;
  std::unique_ptr<Backend> backend_;
};

void TestDataset::init()
{
  QVERIFY(dir_.isValid());
  cfg_ = forklift_gui::test::loadTestConfig();
  cfg_.dataset.rootDir = dir_.path() + QStringLiteral("/dataset");
  clock_ = FakeClock();
  adapter_ = std::make_unique<MockRobotAdapter>(cfg_, clock_.fn());
  Backend::Ports ports;
  ports.command = adapter_->command();
  ports.state = adapter_->state();
  ports.camera = adapter_->camera();
  ports.log = adapter_->log();
  backend_ = std::make_unique<Backend>(cfg_, ports, true, clock_.fn());
}

void TestDataset::cleanup()
{
  backend_.reset();
  adapter_.reset();
}

void TestDataset::giveFrames()
{
  const QRgb colors[] = {qRgb(200, 30, 40), qRgb(20, 160, 90)};
  int i = 0;
  for (const auto & cam : cfg_.cameras.list) {
    FrameMeta meta;
    meta.stamp = 12.25 + i;
    meta.receivedNs = clock_.now;
    meta.encoding = QStringLiteral("rgb8");
    emit adapter_->camera()->frameReceived(cam.id, solidFrame(64, 48, colors[i % 2]), meta);
    ++i;
  }
}

void TestDataset::metadata()
{
  OdomSample odom;
  odom.x = 1.25;
  odom.y = -0.5;
  odom.yaw = 0.75;
  odom.stamp = 12.2;
  emit adapter_->state()->odometryReceived(odom);
  emit adapter_->state()->clockReceived(12.3, clock_.now);
  JointSample js;
  js.hasFork = true;
  js.forkHeight = 0.06;
  emit adapter_->state()->jointStatesReceived(js);
  giveFrames();

  auto * recorder = backend_->dataset();
  QVERIFY(recorder->capture());
  QVERIFY(recorder->waitForIdle(5000));
  QCOMPARE(recorder->captureCount(), 1);
  QVERIFY(recorder->lastError().isEmpty());

  // Dosyalar: <kök>/<oturum>/<kamera>/000001.png ve metadata.jsonl
  const QString session = recorder->sessionDir();
  QVERIFY(session.startsWith(cfg_.dataset.rootDir));
  const QRgb colors[] = {qRgb(200, 30, 40), qRgb(20, 160, 90)};
  int i = 0;
  for (const auto & cam : cfg_.cameras.list) {
    const QString png = session + QLatin1Char('/') + cam.id + QStringLiteral("/000001.png");
    QImage image(png);
    QVERIFY2(!image.isNull(), qPrintable(png));
    QCOMPARE(image.size(), QSize(64, 48));
    QCOMPARE(image.pixel(10, 10) & 0xffffff, colors[i % 2] & 0xffffff);
    ++i;
  }

  QFile meta(session + QStringLiteral("/metadata.jsonl"));
  QVERIFY(meta.open(QIODevice::ReadOnly));
  const QList<QByteArray> lines = meta.readAll().split('\n');
  QCOMPARE(lines.size(), 2);  // bir satır + sondaki boş parça
  QJsonParseError error{};
  const QJsonObject o = QJsonDocument::fromJson(lines.at(0), &error).object();
  QCOMPARE(error.error, QJsonParseError::NoError);

  QCOMPARE(o.value(QStringLiteral("index")).toInt(), 1);
  QVERIFY(!o.value(QStringLiteral("wall_time")).toString().isEmpty());
  QVERIFY(o.value(QStringLiteral("wall_time_unix")).toDouble() > 1.0e9);
  QCOMPARE(o.value(QStringLiteral("sim_time")).toDouble(), 12.3);
  const QJsonObject pose = o.value(QStringLiteral("pose")).toObject();
  QCOMPARE(pose.value(QStringLiteral("x")).toDouble(), 1.25);
  QCOMPARE(pose.value(QStringLiteral("y")).toDouble(), -0.5);
  QCOMPARE(pose.value(QStringLiteral("yaw")).toDouble(), 0.75);
  QCOMPARE(o.value(QStringLiteral("fork_height")).toDouble(), 0.06);
  const QJsonObject images = o.value(QStringLiteral("images")).toObject();
  i = 0;
  for (const auto & cam : cfg_.cameras.list) {
    const QJsonObject img = images.value(cam.id).toObject();
    QCOMPARE(img.value(QStringLiteral("file")).toString(), cam.id + QStringLiteral("/000001.png"));
    QCOMPARE(img.value(QStringLiteral("stamp")).toDouble(), 12.25 + i);
    QCOMPARE(img.value(QStringLiteral("width")).toInt(), 64);
    QCOMPARE(img.value(QStringLiteral("encoding")).toString(), QStringLiteral("rgb8"));
    ++i;
  }

  // İkinci kayıt aynı oturuma ikinci satırı ve 000002.png'yi ekler.
  QVERIFY(recorder->capture());
  QVERIFY(recorder->waitForIdle(5000));
  QCOMPARE(recorder->captureCount(), 2);
  QVERIFY(QFile::exists(
    session + QLatin1Char('/') + cfg_.cameras.list.at(0).id + QStringLiteral("/000002.png")));
}

void TestDataset::skipsWithoutFreshFrames()
{
  auto * recorder = backend_->dataset();
  // Hiç kare yok.
  QVERIFY(!recorder->capture());
  QVERIFY(!recorder->lastError().isEmpty());
  QVERIFY(recorder->sessionDir().isEmpty());
  // Kareler var ama stale_after'dan eski.
  giveFrames();
  clock_.advance(cfg_.cameras.staleAfter + 0.5);
  QVERIFY(!recorder->capture());
  QCOMPARE(recorder->captureCount(), 0);
}

QTEST_MAIN(TestDataset)
#include "test_dataset.moc"
