// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__DATASETRECORDER_HPP_
#define FORKLIFT_GUI__CORE__DATASETRECORDER_HPP_

#include <QtQmlIntegration/qqmlintegration.h>

#include <QDir>
#include <QFutureSynchronizer>
#include <QList>
#include <QObject>
#include <QString>
#include <QThreadPool>
#include <QTimer>

#include "forklift_gui/core/CameraStream.hpp"
#include "forklift_gui/core/ForkController.hpp"
#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/core/TelemetryModel.hpp"

namespace forklift_gui
{

// Veri seti kaydı (spesifikasyon §4.5, DECISIONS D-36): her kayıtta tüm
// kameraların son karesi PNG olarak <kök>/<oturum>/<kamera>/NNNNNN.png'ye,
// bir satır da metadata.jsonl'e yazılır. Disk işi tek iş parçacıklı bir
// havuzda yapılır; GUI thread'i bloklanmaz, yazım sırası korunur.
class DatasetRecorder : public QObject
{
  Q_OBJECT
  QML_ELEMENT
  QML_UNCREATABLE("Backend.dataset üzerinden erişilir")

  Q_PROPERTY(bool recording READ recording WRITE setRecording NOTIFY recordingChanged)
  Q_PROPERTY(int captureCount READ captureCount NOTIFY captureCountChanged)
  Q_PROPERTY(int pendingWrites READ pendingWrites NOTIFY pendingWritesChanged)
  Q_PROPERTY(QString sessionDir READ sessionDir NOTIFY sessionDirChanged)
  Q_PROPERTY(QString rootDir READ rootDir CONSTANT)
  Q_PROPERTY(double continuousRate READ continuousRate CONSTANT)
  Q_PROPERTY(QString lastError READ lastError NOTIFY lastErrorChanged)

public:
  DatasetRecorder(
    const GuiConfig & config, const QList<CameraStream *> & cameras, TelemetryModel * telemetry,
    ForkController * fork, ClockFn clock, QObject * parent = nullptr);
  ~DatasetRecorder() override;

  bool recording() const { return recording_; }
  void setRecording(bool on);
  int captureCount() const { return captureCount_; }
  int pendingWrites() const { return pending_; }
  QString sessionDir() const { return sessionDir_; }
  QString rootDir() const { return root_; }
  double continuousRate() const { return rate_; }
  QString lastError() const { return lastError_; }

  // Anlık kayıt (C tuşu / buton). Kare eksik ya da STALE ise atlanır.
  Q_INVOKABLE bool capture();
  Q_INVOKABLE void toggleRecording();

  // Test ve kapanış: bekleyen yazımların bitmesini bekler.
  bool waitForIdle(int timeoutMs);

signals:
  void recordingChanged();
  void captureCountChanged();
  void pendingWritesChanged();
  void sessionDirChanged();
  void lastErrorChanged();
  void event(int level, const QString & text);

private:
  bool ensureSession();
  void onWriteFinished(bool ok, const QString & error);

  GuiConfig config_;
  QList<CameraStream *> cameras_;
  TelemetryModel * telemetry_;
  ForkController * fork_;
  ClockFn clock_;
  QString root_;
  double rate_;
  QThreadPool pool_;
  QTimer timer_;

  bool recording_{false};
  int nextIndex_{0};  // dosya numarası; başarısız yazımda da tekrar kullanılmaz
  int captureCount_{0};
  int pending_{0};
  QString sessionDir_;
  QString lastError_;
};

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__DATASETRECORDER_HPP_
