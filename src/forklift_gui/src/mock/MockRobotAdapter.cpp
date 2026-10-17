// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/mock/MockRobotAdapter.hpp"

#include <QDateTime>
#include <QFont>
#include <QImage>
#include <QMutexLocker>
#include <QPainter>
#include <QPolygonF>
#include <QVideoFrameFormat>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace forklift_gui
{
namespace mock
{

namespace
{

double approach(double current, double target, double maxStep)
{
  const double delta = target - current;
  if (std::abs(delta) <= maxStep) {
    return target;
  }
  return current + (delta > 0.0 ? maxStep : -maxStep);
}

// Sahne sabitleri: sentetik görüntünün geometrisi, ölçüm değil (yalnız mock).
constexpr double kParcelX = 1.5;        // [m] lift_test dünyasındaki koli
constexpr double kParcelSize = 0.2;     // [m]
constexpr double kCameraHeight = 0.45;  // [m] ön kamera
constexpr double kFocalRatio = 0.866;   // f / genişlik, 60° yatay görüş

QVideoFrame toVideoFrame(const QImage & image)
{
  // QImage::Format_RGB32 küçük-sonlu bellekte BGRX'tir.
  QVideoFrame frame(QVideoFrameFormat(image.size(), QVideoFrameFormat::Format_BGRX8888));
  if (!frame.map(QVideoFrame::WriteOnly)) {
    return {};
  }
  const int rowBytes = image.width() * 4;
  for (int row = 0; row < image.height(); ++row) {
    std::memcpy(frame.bits(0) + row * frame.bytesPerLine(0), image.constScanLine(row), rowBytes);
  }
  frame.unmap();
  return frame;
}

void drawOverlayText(QPainter & p, const QString & title, double simTime, const QSize & size)
{
  QFont font = p.font();
  font.setPixelSize(std::max(12, size.height() / 30));
  font.setBold(true);
  p.setFont(font);
  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(
    QRect(10, 8, size.width() - 20, size.height() / 10), Qt::AlignLeft | Qt::AlignTop,
    QStringLiteral("MOCK · %1 · t=%2 s").arg(title).arg(simTime, 0, 'f', 2));
}

}  // namespace

// ---------------------------------------------------------------- portlar --

MockCommandPort::MockCommandPort(ClockFn clock, QObject * parent)
: IRobotCommandPort(parent), clock_(std::move(clock))
{
}

void MockCommandPort::publishVelocity(double linear, double angular)
{
  CommandRecord r;
  r.kind = CommandRecord::Kind::Velocity;
  r.a = linear;
  r.b = angular;
  r.stampNs = clock_();
  records_.append(r);
  emit velocityCommanded(linear, angular);
}

void MockCommandPort::publishForkTarget(double height)
{
  CommandRecord r;
  r.kind = CommandRecord::Kind::ForkTarget;
  r.a = height;
  r.stampNs = clock_();
  records_.append(r);
  emit forkCommanded(height);
}

void MockCommandPort::publishEStop(bool engaged)
{
  CommandRecord r;
  r.kind = CommandRecord::Kind::EStop;
  r.flag = engaged;
  r.stampNs = clock_();
  records_.append(r);
  emit eStopCommanded(engaged);
}

QVector<TopicCounter> MockStatePort::topicCounters() const
{
  QMutexLocker lock(&mutex_);
  return QVector<TopicCounter>(counters_.cbegin(), counters_.cend());
}

GraphStatus MockStatePort::graphStatus() const
{
  QMutexLocker lock(&mutex_);
  return graph_;
}

void MockStatePort::setCounter(const QString & key, quint64 count, qint64 lastNs)
{
  QMutexLocker lock(&mutex_);
  TopicCounter & c = counters_[key];
  c.key = key;
  c.count = count;
  c.lastNs = lastNs;
}

void MockStatePort::bump(const QString & key, qint64 nowNs)
{
  QMutexLocker lock(&mutex_);
  TopicCounter & c = counters_[key];
  c.key = key;
  ++c.count;
  c.lastNs = nowNs;
}

void MockStatePort::setGraphStatus(const GraphStatus & status)
{
  {
    QMutexLocker lock(&mutex_);
    graph_ = status;
  }
  emit graphStatusChanged(status);
}

void MockLogPort::logEvent(int level, const QString & text) { events_.append({level, text}); }

// ------------------------------------------------------------ kare işçisi --

FrameWorker::FrameWorker(const GuiConfig & config) : config_(config) {}

void FrameWorker::setPose(const Pose & pose)
{
  QMutexLocker lock(&poseMutex_);
  pose_ = pose;
}

void FrameWorker::begin()
{
  // Zamanlayıcı bu thread'de yaratılır; olayları da burada işlenir.
  timer_ = new QTimer(this);
  timer_->setTimerType(Qt::PreciseTimer);
  timer_->setInterval(static_cast<int>(std::lround(1000.0 / config_.mock.frameRate)));
  connect(timer_, &QTimer::timeout, this, &FrameWorker::render);
  timer_->start();
}

void FrameWorker::end()
{
  if (timer_) {
    timer_->stop();
  }
}

void FrameWorker::render()
{
  Pose pose;
  {
    QMutexLocker lock(&poseMutex_);
    pose = pose_;
  }
  const QSize size(config_.mock.imageWidth, config_.mock.imageHeight);
  const double w = size.width();
  const double h = size.height();
  const double focal = kFocalRatio * w;
  ++frameIndex_;

  for (const CameraConfig & cam : config_.cameras.list) {
    QImage image(size, QImage::Format_RGB32);
    QPainter p(&image);
    p.setRenderHint(QPainter::Antialiasing);
    DetectionSet detections;
    detections.cameraId = cam.id;
    detections.stamp = pose.simTime;

    if (cam.detections) {
      // Ön kamera sahnesi: ufuk, perspektif zemin ızgarası, mühürlü koli.
      const double horizon = 0.30 * h;
      const double vanishX = w / 2.0 - focal * std::tan(pose.yaw) * 0.5;
      QLinearGradient sky(0, 0, 0, horizon);
      sky.setColorAt(0.0, QColor(46, 52, 64));
      sky.setColorAt(1.0, QColor(76, 84, 98));
      p.fillRect(QRectF(0, 0, w, horizon), sky);
      QLinearGradient floor(0, horizon, 0, h);
      floor.setColorAt(0.0, QColor(96, 98, 102));
      floor.setColorAt(1.0, QColor(140, 142, 146));
      p.fillRect(QRectF(0, horizon, w, h - horizon), floor);
      p.setPen(QPen(QColor(170, 172, 176, 110), 1));
      for (int i = -8; i <= 8; ++i) {
        p.drawLine(QPointF(vanishX, horizon), QPointF(w / 2.0 + i * w / 6.0, h));
      }
      // Koli: dünya (1.5, 0) → robot çerçevesi → basit iğne deliği izdüşümü.
      const double dxw = kParcelX - pose.x;
      const double dyw = -pose.y;
      const double dx = dxw * std::cos(pose.yaw) + dyw * std::sin(pose.yaw);
      const double dy = -dxw * std::sin(pose.yaw) + dyw * std::cos(pose.yaw);
      if (dx > 0.25) {
        const double side = focal * kParcelSize / dx;
        const double u = w / 2.0 - focal * dy / dx;
        const double bottom = horizon + focal * kCameraHeight / dx;
        const QRectF box(u - side / 2.0, bottom - side, side, side);
        p.fillRect(box, QColor(150, 105, 60));
        p.setPen(QPen(QColor(90, 60, 30), 2));
        p.drawRect(box);
        const QRectF seal(
          box.center().x() - side / 4.0, box.center().y() - side / 4.0, side / 2.0, side / 2.0);
        p.fillRect(seal, QColor(210, 30, 30));
        if (frameIndex_ % 3 == 0) {
          DetectionBox b;
          b.cx = seal.center().x();
          b.cy = seal.center().y();
          b.width = seal.width() * 1.15;
          b.height = seal.height() * 1.15;
          b.label = QStringLiteral("seal_red");
          b.score = 0.93;
          detections.boxes.append(b);
        }
      }
    } else {
      // Şerit kamerası: zemin, sarı şerit bandı ve beyaz kesik çizgiler.
      p.fillRect(QRectF(0, 0, w, h), QColor(120, 122, 126));
      const double lateral = -pose.y;
      const double bottomX = w / 2.0 + lateral * w * 1.6 - pose.yaw * w * 0.25;
      const double topX = w / 2.0 + lateral * w * 0.8 - pose.yaw * w * 0.9;
      const double bottomHalf = w * 0.06;
      const double topHalf = w * 0.03;
      QPolygonF band;
      band << QPointF(bottomX - bottomHalf, h) << QPointF(bottomX + bottomHalf, h)
           << QPointF(topX + topHalf, 0) << QPointF(topX - topHalf, 0);
      p.setPen(Qt::NoPen);
      p.setBrush(QColor(230, 190, 40));
      p.drawPolygon(band);
      const double phase = std::fmod(std::max(pose.x, 0.0), 0.2) / 0.2;
      p.setBrush(QColor(245, 245, 245));
      for (int i = 0; i < 6; ++i) {
        const double t0 = std::fmod(i / 6.0 + phase, 1.0);
        const double t1 = std::min(t0 + 0.07, 1.0);
        const double y0 = h * (1.0 - t0);
        const double y1 = h * (1.0 - t1);
        const double x0 = bottomX + (topX - bottomX) * t0 + w * 0.22;
        const double x1 = bottomX + (topX - bottomX) * t1 + w * 0.22;
        p.drawRect(QRectF(std::min(x0, x1) - 3, y1, 6 + std::abs(x1 - x0), y0 - y1));
      }
    }
    drawOverlayText(p, cam.label, pose.simTime, size);
    p.end();

    FrameMeta meta;
    meta.stamp = pose.simTime;
    meta.receivedNs = steadyNowNs();
    meta.encoding = QStringLiteral("bgra8");
    const QVideoFrame frame = toVideoFrame(image);
    if (frame.isValid()) {
      emit frameReady(cam.id, frame, meta);
    }
    if (!detections.boxes.isEmpty()) {
      detections.receivedNs = meta.receivedNs;
      emit detectionsReady(detections);
    }
  }
}

// ------------------------------------------------------------ sahte robot --

MockRobotAdapter::MockRobotAdapter(const GuiConfig & config, ClockFn clock, QObject * parent)
: QObject(parent),
  config_(config),
  clock_(std::move(clock)),
  command_(new MockCommandPort(clock_, this)),
  state_(new MockStatePort(this)),
  camera_(new MockCameraPort(this)),
  log_(new MockLogPort(this))
{
  connect(command_, &MockCommandPort::velocityCommanded, this, [this](double v, double w) {
    cmdLinear_ = v;
    cmdAngular_ = w;
  });
  connect(command_, &MockCommandPort::forkCommanded, this, [this](double h) {
    forkTarget_ = std::clamp(h, config_.fork.minHeight, config_.fork.maxHeight);
  });
  connect(command_, &MockCommandPort::eStopCommanded, this, [this](bool on) { eStop_ = on; });

  stepTimer_.setTimerType(Qt::PreciseTimer);
  stepTimer_.setInterval(static_cast<int>(std::lround(1000.0 / config_.mock.odomRate)));
  connect(&stepTimer_, &QTimer::timeout, this, &MockRobotAdapter::step);
  clockTimer_.setTimerType(Qt::PreciseTimer);
  clockTimer_.setInterval(static_cast<int>(std::lround(1000.0 / config_.mock.clockRate)));
  connect(&clockTimer_, &QTimer::timeout, this, &MockRobotAdapter::clockTick);
  logTimer_.setInterval(static_cast<int>(std::lround(1000.0 * config_.mock.logPeriod)));
  connect(&logTimer_, &QTimer::timeout, this, &MockRobotAdapter::logTick);
}

MockRobotAdapter::~MockRobotAdapter() { stop(); }

void MockRobotAdapter::start()
{
  startNs_ = clock_();
  lastStepNs_ = startNs_;
  GraphStatus graph;
  graph.up = true;
  graph.mock = true;
  state_->setGraphStatus(graph);

  worker_ = new FrameWorker(config_);
  worker_->moveToThread(&workerThread_);
  connect(&workerThread_, &QThread::finished, worker_, &QObject::deleteLater);
  // İşçi thread'inden GUI thread'ine kuyruklu sinyal; kare hazır gelir.
  connect(
    worker_, &FrameWorker::frameReady, this, &MockRobotAdapter::onFrame, Qt::QueuedConnection);
  connect(
    worker_, &FrameWorker::detectionsReady, this, &MockRobotAdapter::onDetections,
    Qt::QueuedConnection);
  workerThread_.start();
  QMetaObject::invokeMethod(worker_, &FrameWorker::begin, Qt::QueuedConnection);

  stepTimer_.start();
  clockTimer_.start();
  logTimer_.start();
  logTick();
}

void MockRobotAdapter::stop()
{
  stepTimer_.stop();
  clockTimer_.stop();
  logTimer_.stop();
  if (workerThread_.isRunning()) {
    QMetaObject::invokeMethod(worker_, &FrameWorker::end, Qt::BlockingQueuedConnection);
    workerThread_.quit();
    workerThread_.wait();
    worker_ = nullptr;
  }
}

void MockRobotAdapter::step()
{
  const qint64 now = clock_();
  const double dt = std::clamp(nsToSeconds(now - lastStepNs_), 0.0, 0.1);
  lastStepNs_ = now;

  // DiffDrive gibi: son komut tutulur, ivme sınırıyla izlenir.
  const double targetV = eStop_ ? 0.0 : cmdLinear_;
  const double targetW = eStop_ ? 0.0 : cmdAngular_;
  v_ = approach(v_, targetV, config_.mock.linearAccel * dt);
  w_ = approach(w_, targetW, config_.mock.angularAccel * dt);
  yaw_ = std::remainder(yaw_ + w_ * dt, 2.0 * M_PI);
  x_ += v_ * std::cos(yaw_) * dt;
  y_ += v_ * std::sin(yaw_) * dt;
  fork_ = approach(fork_, forkTarget_, config_.mock.forkSpeed * dt);

  OdomSample odom;
  odom.x = x_;
  odom.y = y_;
  odom.yaw = yaw_;
  odom.linear = v_;
  odom.angular = w_;
  odom.stamp = simTime_;
  state_->bump(healthKeyForTopic(config_.topics.odom), now);
  emit state_->odometryReceived(odom);

  JointSample joints;
  joints.hasFork = true;
  joints.forkHeight = fork_;
  joints.hasWheels = true;
  const double half = config_.mock.wheelSeparation / 2.0;
  joints.leftWheel = (v_ - w_ * half) / config_.mock.wheelRadius;
  joints.rightWheel = (v_ + w_ * half) / config_.mock.wheelRadius;
  joints.stamp = simTime_;
  state_->bump(healthKeyForTopic(config_.topics.jointStates), now);
  emit state_->jointStatesReceived(joints);

  if (worker_) {
    FrameWorker::Pose pose;
    pose.x = x_;
    pose.y = y_;
    pose.yaw = yaw_;
    pose.fork = fork_;
    pose.simTime = simTime_;
    worker_->setPose(pose);
  }
}

void MockRobotAdapter::clockTick()
{
  const qint64 now = clock_();
  simTime_ = nsToSeconds(now - startNs_);  // RTF 1
  state_->bump(healthKeyForTopic(config_.topics.clock), now);
  // Port sözleşmesi: saat sinyali kaynağında ≤ ui_max_rate.
  if (
    lastClockEmitNs_ < 0 ||
    now - lastClockEmitNs_ >= secondsToNs(1.0 / config_.telemetry.uiMaxRate)) {
    lastClockEmitNs_ = now;
    emit state_->clockReceived(simTime_, now);
  }
}

void MockRobotAdapter::logTick()
{
  static const struct
  {
    LogLevel level;
    const char * node;
    const char * text;
  } kLines[] = {
    {LogLevel::Info, "mock_sim", "Sahte simülasyon çalışıyor (RTF 1.0)"},
    {LogLevel::Debug, "mock_bridge", "Köprü: 9 topic eşleşti"},
    {LogLevel::Info, "mock_cv", "Mühür tespiti: seal_red (0.93)"},
    {LogLevel::Warn, "mock_cv", "Şerit kamerası: düşük kontrast, eşik uyarlanıyor"},
    {LogLevel::Info, "mock_sim", "Çatal eklemi hedefte"},
    {LogLevel::Error, "mock_bridge", "Örnek hata satırı: bağlantı yeniden kuruldu"},
  };
  const auto & line = kLines[logIndex_ % static_cast<int>(std::size(kLines))];
  ++logIndex_;
  LogEntry e;
  e.wallMs = QDateTime::currentMSecsSinceEpoch();
  e.stamp = simTime_;
  e.level = static_cast<int>(line.level);
  e.node = QString::fromUtf8(line.node);
  e.text = QString::fromUtf8(line.text);
  state_->bump(healthKeyForTopic(config_.topics.rosout), clock_());
  emit log_->entriesReceived({e});
}

void MockRobotAdapter::onFrame(
  const QString & cameraId, const QVideoFrame & frame, const FrameMeta & meta)
{
  const qint64 now = clock_();
  if (const CameraConfig * cam = config_.camera(cameraId)) {
    state_->bump(healthKeyForTopic(cam->imageTopic), now);
    state_->bump(healthKeyForTopic(cam->infoTopic), now);
  }
  emit camera_->frameReceived(cameraId, frame, meta);
  emit camera_->cameraInfoReceived(cameraId, frame.size());
}

void MockRobotAdapter::onDetections(const DetectionSet & set)
{
  state_->bump(healthKeyForTopic(config_.topics.detections), clock_());
  emit state_->detectionsReceived(set);
}

QString MockRobotAdapter::healthKeyForTopic(const QString & topic) const
{
  for (const HealthTopicConfig & h : config_.health.topics) {
    if (h.topic == topic) {
      return h.key;
    }
  }
  return topic;  // sağlık listesinde yoksa topic adıyla sayılır (gösterilmez)
}

}  // namespace mock
}  // namespace forklift_gui
