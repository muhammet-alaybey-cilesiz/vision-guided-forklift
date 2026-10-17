// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/DatasetRecorder.hpp"

#include <QCoreApplication>
#include <QDateTime>
#include <QDeadlineTimer>
#include <QFile>
#include <QFutureWatcher>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVideoFrameFormat>
#include <QtConcurrent/QtConcurrentRun>
#include <cmath>
#include <utility>

namespace forklift_gui
{

namespace
{

// "~/" ile başlayan yolu ev dizinine açar.
QString expandHome(const QString & path)
{
  if (path == QStringLiteral("~")) {
    return QDir::homePath();
  }
  if (path.startsWith(QStringLiteral("~/"))) {
    return QDir::homePath() + path.mid(1);
  }
  return path;
}

// QVideoFrame → QImage derin kopya (GUI thread'inde, ~1 ms). RGBX dolgusu
// atılır; PNG RGB888 (gri tonlu kaynaklar Grayscale8) yazılır.
QImage frameToImage(const QVideoFrame & source)
{
  QVideoFrame frame(source);
  if (!frame.map(QVideoFrame::ReadOnly)) {
    return {};
  }
  const QImage::Format format = QVideoFrameFormat::imageFormatFromPixelFormat(frame.pixelFormat());
  QImage out;
  if (format != QImage::Format_Invalid) {
    const QImage view(frame.bits(0), frame.width(), frame.height(), frame.bytesPerLine(0), format);
    const QImage::Format target =
      format == QImage::Format_Grayscale8 ? QImage::Format_Grayscale8 : QImage::Format_RGB888;
    // Aynı formatta convertToFormat sığ kopya döndürür; unmap sonrası
    // geçersiz olmasın diye açıkça kopyalanır.
    out = format == target ? view.copy() : view.convertToFormat(target);
  }
  frame.unmap();
  return out;
}

struct Shot
{
  QString cameraId;
  QImage image;
  QString relativePath;
};

// İşçi iş parçacığında: PNG'ler ve tek JSONL satırı. Boş dönüş = başarı.
QString writeCapture(
  const QString & sessionDir, const QVector<Shot> & shots, const QByteArray & jsonLine)
{
  for (const Shot & shot : shots) {
    const QString path = sessionDir + QLatin1Char('/') + shot.relativePath;
    if (!shot.image.save(path, "PNG")) {
      return QObject::tr("PNG yazılamadı: %1").arg(path);
    }
  }
  QFile meta(sessionDir + QStringLiteral("/metadata.jsonl"));
  if (!meta.open(QIODevice::WriteOnly | QIODevice::Append)) {
    return QObject::tr("metadata.jsonl açılamadı: %1").arg(meta.errorString());
  }
  if (meta.write(jsonLine) != jsonLine.size()) {
    return QObject::tr("metadata.jsonl yazılamadı: %1").arg(meta.errorString());
  }
  return {};
}

}  // namespace

DatasetRecorder::DatasetRecorder(
  const GuiConfig & config, const QList<CameraStream *> & cameras, TelemetryModel * telemetry,
  ForkController * fork, ClockFn clock, QObject * parent)
: QObject(parent),
  config_(config),
  cameras_(cameras),
  telemetry_(telemetry),
  fork_(fork),
  clock_(std::move(clock)),
  root_(expandHome(config.dataset.rootDir)),
  rate_(config.dataset.continuousRate)
{
  pool_.setMaxThreadCount(1);  // yazım sırası korunur
  timer_.setTimerType(Qt::PreciseTimer);
  timer_.setInterval(static_cast<int>(std::lround(1000.0 / rate_)));
  connect(&timer_, &QTimer::timeout, this, [this]() { capture(); });
}

DatasetRecorder::~DatasetRecorder()
{
  timer_.stop();
  pool_.waitForDone();
}

void DatasetRecorder::setRecording(bool on)
{
  if (on == recording_) {
    return;
  }
  recording_ = on;
  if (on) {
    timer_.start();
    emit event(
      static_cast<int>(LogLevel::Info), tr("Sürekli kayıt başladı (%1 Hz)").arg(rate_, 0, 'g', 3));
    capture();
  } else {
    timer_.stop();
    emit event(
      static_cast<int>(LogLevel::Info),
      tr("Sürekli kayıt durdu (oturumda %1 kayıt)").arg(captureCount_));
  }
  emit recordingChanged();
}

void DatasetRecorder::toggleRecording() { setRecording(!recording_); }

bool DatasetRecorder::capture()
{
  const qint64 now = clock_();
  const qint64 staleNs = secondsToNs(config_.cameras.staleAfter);

  // Tüm kameraların taze karesi olmalı; biri eksikse kayıt bütünüyle atlanır.
  QVector<Shot> shots;
  QJsonObject images;
  for (CameraStream * cam : std::as_const(cameras_)) {
    const QVideoFrame frame = cam->latestFrame();
    if (!frame.isValid() || cam->latestArrivalNs() < 0 || now - cam->latestArrivalNs() > staleNs) {
      lastError_ = tr("Kayıt atlandı: %1 karesi yok ya da eski").arg(cam->label());
      emit lastErrorChanged();
      emit event(static_cast<int>(LogLevel::Warn), lastError_);
      return false;
    }
    Shot shot;
    shot.cameraId = cam->cameraId();
    shot.image = frameToImage(frame);
    if (shot.image.isNull()) {
      lastError_ = tr("Kayıt atlandı: %1 karesi dönüştürülemedi").arg(cam->label());
      emit lastErrorChanged();
      emit event(static_cast<int>(LogLevel::Warn), lastError_);
      return false;
    }
    shots.push_back(shot);
  }
  if (!ensureSession()) {
    return false;
  }

  const int index = ++nextIndex_;
  const QString name = QStringLiteral("%1.png").arg(index, 6, 10, QLatin1Char('0'));
  for (int i = 0; i < shots.size(); ++i) {
    Shot & shot = shots[i];
    shot.relativePath = shot.cameraId + QLatin1Char('/') + name;
    const FrameMeta meta = cameras_.at(i)->latestMeta();
    QJsonObject img;
    img.insert(QStringLiteral("file"), shot.relativePath);
    img.insert(QStringLiteral("stamp"), meta.stamp);
    img.insert(QStringLiteral("width"), shot.image.width());
    img.insert(QStringLiteral("height"), shot.image.height());
    img.insert(QStringLiteral("encoding"), meta.encoding);
    images.insert(shot.cameraId, img);
  }

  const QDateTime wall = QDateTime::currentDateTime();
  QJsonObject line;
  line.insert(QStringLiteral("index"), index);
  line.insert(QStringLiteral("wall_time"), wall.toString(Qt::ISODateWithMs));
  line.insert(
    QStringLiteral("wall_time_unix"), static_cast<double>(wall.toMSecsSinceEpoch()) / 1000.0);
  line.insert(
    QStringLiteral("sim_time"),
    telemetry_->latestHasClock() ? QJsonValue(telemetry_->latestSimTime()) : QJsonValue());
  if (telemetry_->latestHasOdom()) {
    const OdomSample odom = telemetry_->latestOdom();
    QJsonObject pose;
    pose.insert(QStringLiteral("x"), odom.x);
    pose.insert(QStringLiteral("y"), odom.y);
    pose.insert(QStringLiteral("yaw"), odom.yaw);
    pose.insert(QStringLiteral("stamp"), odom.stamp);
    line.insert(QStringLiteral("pose"), pose);
  } else {
    line.insert(QStringLiteral("pose"), QJsonValue());
  }
  line.insert(
    QStringLiteral("fork_height"),
    fork_->hasMeasured() ? QJsonValue(fork_->latestMeasured()) : QJsonValue());
  line.insert(
    QStringLiteral("fork_target"), fork_->hasTarget() ? QJsonValue(fork_->target()) : QJsonValue());
  line.insert(QStringLiteral("images"), images);
  const QByteArray json = QJsonDocument(line).toJson(QJsonDocument::Compact) + '\n';

  ++pending_;
  emit pendingWritesChanged();
  auto * watcher = new QFutureWatcher<QString>(this);
  connect(watcher, &QFutureWatcher<QString>::finished, this, [this, watcher]() {
    const QString error = watcher->result();
    watcher->deleteLater();
    onWriteFinished(error.isEmpty(), error);
  });
  watcher->setFuture(QtConcurrent::run(&pool_, writeCapture, sessionDir_, shots, json));
  return true;
}

bool DatasetRecorder::waitForIdle(int timeoutMs)
{
  const QDeadlineTimer deadline(timeoutMs);
  if (!pool_.waitForDone(timeoutMs)) {
    return false;
  }
  // Tamamlanma bildirimleri (QFutureWatcher) GUI thread'inde kuyrukta;
  // sayaç sıfırlanana ya da süre dolana kadar işlenir.
  while (pending_ > 0 && !deadline.hasExpired()) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
  }
  return pending_ == 0;
}

bool DatasetRecorder::ensureSession()
{
  if (!sessionDir_.isEmpty()) {
    return true;
  }
  const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
  const QString dir = root_ + QLatin1Char('/') + stamp;
  QDir d;
  for (CameraStream * cam : std::as_const(cameras_)) {
    if (!d.mkpath(dir + QLatin1Char('/') + cam->cameraId())) {
      lastError_ = tr("Veri seti dizini oluşturulamadı: %1").arg(dir);
      emit lastErrorChanged();
      emit event(static_cast<int>(LogLevel::Error), lastError_);
      return false;
    }
  }
  sessionDir_ = dir;
  emit sessionDirChanged();
  emit event(static_cast<int>(LogLevel::Info), tr("Veri seti oturumu: %1").arg(dir));
  return true;
}

void DatasetRecorder::onWriteFinished(bool ok, const QString & error)
{
  --pending_;
  emit pendingWritesChanged();
  if (ok) {
    ++captureCount_;
    emit captureCountChanged();
    return;
  }
  lastError_ = error;
  emit lastErrorChanged();
  emit event(static_cast<int>(LogLevel::Error), error);
}

}  // namespace forklift_gui
