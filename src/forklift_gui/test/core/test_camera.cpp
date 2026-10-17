// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// Ek: CameraStream. FPS ölçümü, stale_after'dan eski karede STALE, tespit
// kutularının görüntü boyutuna normalize edilmesi ve detection_max_age sonrası
// gizlenmesi (spesifikasyon §4.4).

#include <QVideoFrameFormat>
#include <QtTest>
#include <memory>

#include "TestSupport.hpp"
#include "forklift_gui/core/CameraStream.hpp"

using forklift_gui::CameraStream;
using forklift_gui::DetectionBox;
using forklift_gui::DetectionSet;
using forklift_gui::FrameMeta;
using forklift_gui::GuiConfig;
using forklift_gui::test::FakeClock;

class TestCamera : public QObject
{
  Q_OBJECT

private slots:
  void init();
  void cleanup();
  void fpsAndStale();
  void detectionsNormalizedAndExpire();

private:
  void feed(double rate, double seconds);

  GuiConfig cfg_;
  FakeClock clock_;
  std::unique_ptr<CameraStream> stream_;
  QVideoFrame frame_;
};

void TestCamera::init()
{
  cfg_ = forklift_gui::test::loadTestConfig();
  clock_ = FakeClock();
  // Tespit kaplaması olan kamera (gui_params: cameras.front.detections true).
  const forklift_gui::CameraConfig * cam = nullptr;
  for (const auto & c : cfg_.cameras.list) {
    if (c.detections) {
      cam = &c;
    }
  }
  QVERIFY(cam != nullptr);
  stream_ = std::make_unique<CameraStream>(*cam, cfg_.cameras, cfg_.health.updateRate, clock_.fn());
  frame_ = QVideoFrame(QVideoFrameFormat(QSize(640, 480), QVideoFrameFormat::Format_RGBX8888));
}

void TestCamera::cleanup() { stream_.reset(); }

void TestCamera::feed(double rate, double seconds)
{
  for (double t = 0.0; t < seconds; t += 1.0 / rate) {
    clock_.advance(1.0 / rate);
    stream_->onFrame(frame_, FrameMeta());
  }
}

void TestCamera::fpsAndStale()
{
  stream_->tick(clock_.now);
  QVERIFY(stream_->stale());  // hiç kare yok
  QVERIFY(!stream_->hasFrame());

  feed(30.0, 2.0);
  stream_->tick(clock_.now);
  QVERIFY(std::abs(stream_->fps() - 30.0) < 0.5);
  QVERIFY(!stream_->stale());
  QCOMPARE(stream_->frameWidth(), 640);
  QCOMPARE(stream_->frameHeight(), 480);

  // Kare kesildi: stale_after'a kadar taze, sonra STALE; FPS sıfıra iner.
  clock_.advance(cfg_.cameras.staleAfter * 0.5);
  stream_->tick(clock_.now);
  QVERIFY(!stream_->stale());
  clock_.advance(cfg_.cameras.staleAfter);
  stream_->tick(clock_.now);
  QVERIFY(stream_->stale());
  QVERIFY(stream_->hasFrame());  // son kare durur (donmuş görüntü + STALE)
  QCOMPARE(stream_->fps(), 0.0);
}

void TestCamera::detectionsNormalizedAndExpire()
{
  feed(30.0, 0.2);
  DetectionSet set;
  set.cameraId = stream_->cameraId();
  DetectionBox b;
  b.cx = 320.0;
  b.cy = 240.0;
  b.width = 64.0;
  b.height = 48.0;
  b.label = QStringLiteral("seal_red");
  b.score = 0.9;
  set.boxes.append(b);
  stream_->onDetections(set);
  QVERIFY(stream_->detectionsVisible());
  const QVariantMap m = stream_->detections().constFirst().toMap();
  QCOMPARE(m.value(QStringLiteral("x")).toDouble(), (320.0 - 32.0) / 640.0);
  QCOMPARE(m.value(QStringLiteral("y")).toDouble(), (240.0 - 24.0) / 480.0);
  QCOMPARE(m.value(QStringLiteral("w")).toDouble(), 0.1);
  QCOMPARE(m.value(QStringLiteral("h")).toDouble(), 0.1);
  QCOMPARE(m.value(QStringLiteral("label")).toString(), QStringLiteral("seal_red"));

  // Yeni tespit gelmezse detection_max_age sonra kutular gizlenir.
  clock_.advance(cfg_.cameras.detectionMaxAge + 0.1);
  stream_->tick(clock_.now);
  QVERIFY(!stream_->detectionsVisible());
  QVERIFY(stream_->detections().isEmpty());
}

QTEST_MAIN(TestCamera)
#include "test_camera.moc"
