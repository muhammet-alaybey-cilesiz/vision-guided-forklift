// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/core/GuiConfig.hpp"

#include <QSet>

namespace forklift_gui
{

namespace
{

// Pozitiflik denetimi için kısa yardımcılar; hata metni parametre adını taşır.
void requirePositive(QStringList & errors, const char * name, double value)
{
  if (!(value > 0.0)) {
    errors << QStringLiteral("%1 pozitif olmalı (değer: %2)").arg(QLatin1String(name)).arg(value);
  }
}

void requireNonEmpty(QStringList & errors, const char * name, const QString & value)
{
  if (value.trimmed().isEmpty()) {
    errors << QStringLiteral("%1 boş olamaz").arg(QLatin1String(name));
  }
}

}  // namespace

QStringList GuiConfig::validate() const
{
  QStringList errors;

  requireNonEmpty(errors, "topics.odom", topics.odom);
  requireNonEmpty(errors, "topics.joint_states", topics.jointStates);
  requireNonEmpty(errors, "topics.clock", topics.clock);
  requireNonEmpty(errors, "topics.rosout", topics.rosout);
  requireNonEmpty(errors, "topics.detections", topics.detections);
  requireNonEmpty(errors, "topics.cmd_vel", topics.cmdVel);
  requireNonEmpty(errors, "topics.fork_cmd", topics.forkCmd);
  requireNonEmpty(errors, "topics.e_stop", topics.eStop);

  if (ros.executorThreads < 1) {
    errors << QStringLiteral("ros.executor_threads en az 1 olmalı");
  }

  if (cameras.list.isEmpty()) {
    errors << QStringLiteral("cameras.ids en az bir kamera içermeli");
  }
  QSet<QString> cameraIds;
  for (const auto & cam : cameras.list) {
    if (cameraIds.contains(cam.id)) {
      errors << QStringLiteral("cameras.ids tekrar eden kimlik: %1").arg(cam.id);
    }
    cameraIds.insert(cam.id);
    if (cam.imageTopic.isEmpty() || cam.infoTopic.isEmpty()) {
      errors << QStringLiteral("cameras.%1 topic adları boş olamaz").arg(cam.id);
    }
  }
  requirePositive(errors, "cameras.stale_after", cameras.staleAfter);
  requirePositive(errors, "cameras.fps_window", cameras.fpsWindow);
  requirePositive(errors, "cameras.detection_max_age", cameras.detectionMaxAge);

  requirePositive(errors, "teleop.publish_rate", teleop.publishRate);
  requirePositive(errors, "teleop.max_linear", teleop.maxLinear);
  requirePositive(errors, "teleop.max_angular", teleop.maxAngular);
  requirePositive(errors, "teleop.max_linear_accel", teleop.maxLinearAccel);
  requirePositive(errors, "teleop.max_angular_accel", teleop.maxAngularAccel);
  requirePositive(errors, "teleop.scale_min", teleop.scaleMin);
  requirePositive(errors, "teleop.scale_step", teleop.scaleStep);
  if (!(teleop.scaleMin <= teleop.scaleDefault && teleop.scaleDefault <= teleop.scaleMax)) {
    errors << QStringLiteral("teleop: scale_min <= scale_default <= scale_max olmalı");
  }

  requireNonEmpty(errors, "fork.joint_name", fork.jointName);
  if (!(fork.minHeight < fork.maxHeight)) {
    errors << QStringLiteral("fork: min_height < max_height olmalı");
  }
  requirePositive(errors, "fork.key_step", fork.keyStep);
  requirePositive(errors, "fork.reached_tolerance", fork.reachedTolerance);
  requirePositive(errors, "fork.reach_timeout", fork.reachTimeout);
  requirePositive(errors, "fork.resync_after", fork.resyncAfter);
  if (fork.presetLabels.size() != fork.presetHeights.size()) {
    errors << QStringLiteral("fork: preset_labels ve preset_heights aynı uzunlukta olmalı");
  }
  for (const double h : fork.presetHeights) {
    if (h < fork.minHeight || h > fork.maxHeight) {
      errors << QStringLiteral("fork.preset_heights öğesi sınır dışında: %1").arg(h);
    }
  }

  requirePositive(errors, "safety.e_stop_rate", safety.eStopRate);
  requirePositive(errors, "safety.release_hold", safety.releaseHold);

  requirePositive(errors, "telemetry.ui_max_rate", telemetry.uiMaxRate);
  requirePositive(errors, "telemetry.rtf_window", telemetry.rtfWindow);
  requirePositive(errors, "telemetry.trail_min_step", telemetry.trailMinStep);
  if (telemetry.trailMaxPoints < 2) {
    errors << QStringLiteral("telemetry.trail_max_points en az 2 olmalı");
  }
  if (telemetry.wheelJoints.size() != 2) {
    errors << QStringLiteral("telemetry.wheel_joints tam iki eklem adı içermeli (sol, sağ)");
  }

  requirePositive(errors, "health.update_rate", health.updateRate);
  requirePositive(errors, "health.rate_window", health.rateWindow);
  if (!(health.warnRateRatio > 0.0 && health.warnRateRatio <= 1.0)) {
    errors << QStringLiteral("health.warn_rate_ratio (0, 1] aralığında olmalı");
  }
  for (const auto & h : health.topics) {
    if (h.expectedRate < 0.0 || h.warnAge < 0.0 || h.errorAge < 0.0) {
      errors << QStringLiteral("health.%1 değerleri negatif olamaz").arg(h.key);
    }
    if (h.warnAge > 0.0 && h.errorAge > 0.0 && h.errorAge < h.warnAge) {
      errors << QStringLiteral("health.%1: error_age >= warn_age olmalı").arg(h.key);
    }
    if (h.topic.isEmpty()) {
      errors << QStringLiteral("health.%1 bir topic'e çözülemedi").arg(h.key);
    }
  }

  if (log.maxEntries < 1) {
    errors << QStringLiteral("log.max_entries en az 1 olmalı");
  }

  requireNonEmpty(errors, "dataset.root_dir", dataset.rootDir);
  requirePositive(errors, "dataset.continuous_rate", dataset.continuousRate);

  if (mock.imageWidth < 1 || mock.imageHeight < 1) {
    errors << QStringLiteral("mock.image_width / image_height pozitif olmalı");
  }
  requirePositive(errors, "mock.frame_rate", mock.frameRate);
  requirePositive(errors, "mock.odom_rate", mock.odomRate);
  requirePositive(errors, "mock.clock_rate", mock.clockRate);
  requirePositive(errors, "mock.linear_accel", mock.linearAccel);
  requirePositive(errors, "mock.angular_accel", mock.angularAccel);
  requirePositive(errors, "mock.fork_speed", mock.forkSpeed);
  requirePositive(errors, "mock.wheel_radius", mock.wheelRadius);
  requirePositive(errors, "mock.wheel_separation", mock.wheelSeparation);
  requirePositive(errors, "mock.log_period", mock.logPeriod);

  return errors;
}

const CameraConfig * GuiConfig::camera(const QString & id) const
{
  for (const auto & cam : cameras.list) {
    if (cam.id == id) {
      return &cam;
    }
  }
  return nullptr;
}

}  // namespace forklift_gui
