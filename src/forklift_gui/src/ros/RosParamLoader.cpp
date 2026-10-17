// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/ros/RosParamLoader.hpp"

#include <QHash>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/parameter_map.hpp"

namespace forklift_gui
{
namespace ros_adapter
{

namespace
{

// Parametre kaynağı: canlı düğüm ya da YAML'dan okunmuş harita.
class Source
{
public:
  virtual ~Source() = default;
  // Tanımsızsa PARAMETER_NOT_SET döner.
  virtual rclcpp::ParameterValue get(const std::string & name) = 0;
};

class NodeSource : public Source
{
public:
  explicit NodeSource(rclcpp::Node & node) : node_(node) {}

  rclcpp::ParameterValue get(const std::string & name) override
  {
    if (!node_.has_parameter(name)) {
      // Varsayılansız bildirim: değer yalnızca --params-file / -p ile gelir.
      // Dinamik tip, tamsayı/double esnekliğini yükleyicide çözmek için.
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing = true;
      node_.declare_parameter(name, rclcpp::ParameterValue(), descriptor);
    }
    return node_.get_parameter(name).get_parameter_value();
  }

private:
  rclcpp::Node & node_;
};

class MapSource : public Source
{
public:
  void set(const std::string & name, const rclcpp::ParameterValue & value)
  {
    values_[name] = value;
  }

  rclcpp::ParameterValue get(const std::string & name) override
  {
    const auto it = values_.find(name);
    return it == values_.end() ? rclcpp::ParameterValue() : it->second;
  }

private:
  std::map<std::string, rclcpp::ParameterValue> values_;
};

// Tipli okuma; hatalar toplanır, ilk hatada durulmaz.
class Reader
{
public:
  explicit Reader(Source & source) : source_(source) {}

  QString str(const std::string & name)
  {
    const auto v = fetch(name, rclcpp::ParameterType::PARAMETER_STRING, "string");
    return v ? QString::fromStdString(v->get<std::string>()) : QString();
  }

  double dbl(const std::string & name)
  {
    const auto v = source_.get(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return v.get<double>();
    }
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(v.get<int64_t>());
    }
    fail(name, v, "double");
    return 0.0;
  }

  int integer(const std::string & name)
  {
    const auto v = fetch(name, rclcpp::ParameterType::PARAMETER_INTEGER, "integer");
    return v ? static_cast<int>(v->get<int64_t>()) : 0;
  }

  bool boolean(const std::string & name)
  {
    const auto v = fetch(name, rclcpp::ParameterType::PARAMETER_BOOL, "bool");
    return v ? v->get<bool>() : false;
  }

  QStringList strList(const std::string & name)
  {
    QStringList out;
    const auto v = fetch(name, rclcpp::ParameterType::PARAMETER_STRING_ARRAY, "string[]");
    if (v) {
      for (const auto & s : v->get<std::vector<std::string>>()) {
        out << QString::fromStdString(s);
      }
    }
    return out;
  }

  QVector<double> dblList(const std::string & name)
  {
    QVector<double> out;
    const auto v = source_.get(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      for (const double d : v.get<std::vector<double>>()) {
        out << d;
      }
    } else if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      for (const int64_t i : v.get<std::vector<int64_t>>()) {
        out << static_cast<double>(i);
      }
    } else {
      fail(name, v, "double[]");
    }
    return out;
  }

  QStringList errors;

private:
  std::optional<rclcpp::ParameterValue> fetch(
    const std::string & name, rclcpp::ParameterType type, const char * typeName)
  {
    const auto v = source_.get(name);
    if (v.get_type() != type) {
      fail(name, v, typeName);
      return std::nullopt;
    }
    return v;
  }

  void fail(const std::string & name, const rclcpp::ParameterValue & v, const char * expected)
  {
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      errors << QStringLiteral("%1: tanımlı değil").arg(QString::fromStdString(name));
    } else {
      errors << QStringLiteral("%1: tip %2 bekleniyordu, %3 geldi")
                  .arg(
                    QString::fromStdString(name), QLatin1String(expected),
                    QString::fromStdString(rclcpp::to_string(v.get_type())));
    }
  }

  Source & source_;
};

std::string key(const QString & prefix, const char * leaf)
{
  return (prefix + QLatin1Char('.') + QLatin1String(leaf)).toStdString();
}

// health anahtarı → topic adı: topics.<anahtar> ya da <kamera>_image / _info.
QString resolveHealthTopic(const GuiConfig & c, const QString & healthKey)
{
  const QHash<QString, QString> topics{
    {QStringLiteral("odom"), c.topics.odom},
    {QStringLiteral("joint_states"), c.topics.jointStates},
    {QStringLiteral("clock"), c.topics.clock},
    {QStringLiteral("rosout"), c.topics.rosout},
    {QStringLiteral("detections"), c.topics.detections},
    {QStringLiteral("cmd_vel"), c.topics.cmdVel},
    {QStringLiteral("fork_cmd"), c.topics.forkCmd},
    {QStringLiteral("e_stop"), c.topics.eStop},
  };
  if (topics.contains(healthKey)) {
    return topics.value(healthKey);
  }
  for (const CameraConfig & cam : c.cameras.list) {
    if (healthKey == cam.id + QStringLiteral("_image")) {
      return cam.imageTopic;
    }
    if (healthKey == cam.id + QStringLiteral("_info")) {
      return cam.infoTopic;
    }
  }
  return {};
}

GuiConfig read(Source & source)
{
  Reader r(source);
  GuiConfig c;

  c.topics.odom = r.str("topics.odom");
  c.topics.jointStates = r.str("topics.joint_states");
  c.topics.clock = r.str("topics.clock");
  c.topics.rosout = r.str("topics.rosout");
  c.topics.detections = r.str("topics.detections");
  c.topics.cmdVel = r.str("topics.cmd_vel");
  c.topics.forkCmd = r.str("topics.fork_cmd");
  c.topics.eStop = r.str("topics.e_stop");

  c.ros.executorThreads = r.integer("ros.executor_threads");

  for (const QString & id : r.strList("cameras.ids")) {
    const QString p = QStringLiteral("cameras.") + id;
    CameraConfig cam;
    cam.id = id;
    cam.label = r.str(key(p, "label"));
    cam.imageTopic = r.str(key(p, "image_topic"));
    cam.infoTopic = r.str(key(p, "info_topic"));
    cam.detections = r.boolean(key(p, "detections"));
    c.cameras.list << cam;
  }
  c.cameras.staleAfter = r.dbl("cameras.stale_after");
  c.cameras.fpsWindow = r.dbl("cameras.fps_window");
  c.cameras.detectionMaxAge = r.dbl("cameras.detection_max_age");

  c.teleop.publishRate = r.dbl("teleop.publish_rate");
  c.teleop.maxLinear = r.dbl("teleop.max_linear");
  c.teleop.maxAngular = r.dbl("teleop.max_angular");
  c.teleop.maxLinearAccel = r.dbl("teleop.max_linear_accel");
  c.teleop.maxAngularAccel = r.dbl("teleop.max_angular_accel");
  c.teleop.scaleMin = r.dbl("teleop.scale_min");
  c.teleop.scaleMax = r.dbl("teleop.scale_max");
  c.teleop.scaleStep = r.dbl("teleop.scale_step");
  c.teleop.scaleDefault = r.dbl("teleop.scale_default");

  c.fork.jointName = r.str("fork.joint_name");
  c.fork.minHeight = r.dbl("fork.min_height");
  c.fork.maxHeight = r.dbl("fork.max_height");
  c.fork.keyStep = r.dbl("fork.key_step");
  c.fork.reachedTolerance = r.dbl("fork.reached_tolerance");
  c.fork.reachTimeout = r.dbl("fork.reach_timeout");
  c.fork.resyncAfter = r.dbl("fork.resync_after");
  c.fork.presetLabels = r.strList("fork.preset_labels");
  c.fork.presetHeights = r.dblList("fork.preset_heights");

  c.safety.eStopRate = r.dbl("safety.e_stop_rate");
  c.safety.releaseHold = r.dbl("safety.release_hold");

  c.telemetry.uiMaxRate = r.dbl("telemetry.ui_max_rate");
  c.telemetry.rtfWindow = r.dbl("telemetry.rtf_window");
  c.telemetry.trailMaxPoints = r.integer("telemetry.trail_max_points");
  c.telemetry.trailMinStep = r.dbl("telemetry.trail_min_step");
  c.telemetry.wheelJoints = r.strList("telemetry.wheel_joints");

  c.health.updateRate = r.dbl("health.update_rate");
  c.health.rateWindow = r.dbl("health.rate_window");
  c.health.warnRateRatio = r.dbl("health.warn_rate_ratio");
  for (const QString & healthKey : r.strList("health.ids")) {
    const QString p = QStringLiteral("health.") + healthKey;
    HealthTopicConfig h;
    h.key = healthKey;
    h.expectedRate = r.dbl(key(p, "expected_rate"));
    h.warnAge = r.dbl(key(p, "warn_age"));
    h.errorAge = r.dbl(key(p, "error_age"));
    h.required = r.boolean(key(p, "required"));
    c.health.topics << h;
  }

  c.log.maxEntries = r.integer("log.max_entries");

  c.dataset.rootDir = r.str("dataset.root_dir");
  c.dataset.continuousRate = r.dbl("dataset.continuous_rate");

  c.mock.imageWidth = r.integer("mock.image_width");
  c.mock.imageHeight = r.integer("mock.image_height");
  c.mock.frameRate = r.dbl("mock.frame_rate");
  c.mock.odomRate = r.dbl("mock.odom_rate");
  c.mock.clockRate = r.dbl("mock.clock_rate");
  c.mock.linearAccel = r.dbl("mock.linear_accel");
  c.mock.angularAccel = r.dbl("mock.angular_accel");
  c.mock.forkSpeed = r.dbl("mock.fork_speed");
  c.mock.wheelRadius = r.dbl("mock.wheel_radius");
  c.mock.wheelSeparation = r.dbl("mock.wheel_separation");
  c.mock.logPeriod = r.dbl("mock.log_period");

  // Topic adları okunduktan sonra sağlık anahtarları çözülür.
  for (HealthTopicConfig & h : c.health.topics) {
    h.topic = resolveHealthTopic(c, h.key);
  }

  QStringList errors = r.errors;
  if (errors.isEmpty()) {
    errors = c.validate();  // tip hataları varken tutarlılık mesajları gürültü olur
  }
  if (!errors.isEmpty()) {
    throw ConfigError(
      ("gui_params: eksik ya da hatalı parametreler:\n  - " + errors.join(QStringLiteral("\n  - ")))
        .toStdString());
  }
  return c;
}

}  // namespace

GuiConfig loadConfig(rclcpp::Node & node)
{
  NodeSource source(node);
  return read(source);
}

GuiConfig loadConfigFromYaml(const std::string & path, const std::string & nodeName)
{
  rclcpp::ParameterMap map;
  try {
    map = rclcpp::parameter_map_from_yaml_file(path);
  } catch (const std::exception & e) {
    throw ConfigError("gui_params okunamadı (" + path + "): " + e.what());
  }
  MapSource source;
  // Önce joker (/**), sonra düğüme özel değerler (öncelikli).
  for (const std::string & ns : {std::string("/**"), nodeName}) {
    const auto it = map.find(ns);
    if (it == map.end()) {
      continue;
    }
    for (const rclcpp::Parameter & p : it->second) {
      source.set(p.get_name(), p.get_parameter_value());
    }
  }
  return read(source);
}

}  // namespace ros_adapter
}  // namespace forklift_gui
