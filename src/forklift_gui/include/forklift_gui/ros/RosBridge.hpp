// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__ROS__ROSBRIDGE_HPP_
#define FORKLIFT_GUI__ROS__ROSBRIDGE_HPP_

#include <QHash>
#include <QSize>
#include <QStringList>
#include <QTimer>
#include <QVector>
#include <QVideoFrame>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/Types.hpp"
#include "forklift_gui/core/ports/ICameraPort.hpp"
#include "forklift_gui/core/ports/ILogPort.hpp"
#include "forklift_gui/core/ports/IRobotCommandPort.hpp"
#include "forklift_gui/core/ports/IRobotStatePort.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace forklift_gui
{
namespace ros_adapter
{

// "En son değer kazanır" posta kutusu: ROS thread'i yazar, GUI thread'i alır.
// put() yalnız bekleyen teslim yoksa true döner; böylece akış başına
// kuyrukta en fazla bir olay olur (D-27).
template <typename T>
class Mailbox
{
public:
  bool put(T value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = std::move(value);
    has_ = true;
    if (pending_) {
      return false;
    }
    pending_ = true;
    return true;
  }

  std::optional<T> take()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = false;
    if (!has_) {
      return std::nullopt;
    }
    has_ = false;
    return std::move(value_);
  }

private:
  std::mutex mutex_;
  T value_{};
  bool has_{false};
  bool pending_{false};
};

// Topic başına alım sayacı; ROS thread'i artırır, GUI thread'i anlık kopya
// alır. Yuvalar kurulumda sabitlenir, kilitsiz okunur.
class CounterRegistry
{
public:
  explicit CounterRegistry(const QStringList & keys);
  int indexOf(const QString & key) const { return index_.value(key, -1); }
  void bump(int index, qint64 nowNs);
  QVector<TopicCounter> snapshot() const;

private:
  struct Slot
  {
    QString key;
    std::atomic<quint64> count{0};
    std::atomic<qint64> lastNs{-1};
  };
  std::vector<std::unique_ptr<Slot>> slots_;
  QHash<QString, int> index_;
};

class RosCommandAdapter : public IRobotCommandPort
{
  Q_OBJECT

public:
  RosCommandAdapter(rclcpp::Node & node, const GuiConfig::Topics & topics, QObject * parent);
  void publishVelocity(double linear, double angular) override;
  void publishForkTarget(double height) override;
  void publishEStop(bool engaged) override;

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVel_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fork_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr eStop_;
};

class RosStateAdapter : public IRobotStatePort
{
  Q_OBJECT

public:
  RosStateAdapter(const CounterRegistry & counters, QObject * parent);
  QVector<TopicCounter> topicCounters() const override { return counters_.snapshot(); }
  GraphStatus graphStatus() const override { return graph_; }
  void setGraphStatus(const GraphStatus & status);

private:
  const CounterRegistry & counters_;
  GraphStatus graph_;
};

class RosCameraAdapter : public ICameraPort
{
  Q_OBJECT

public:
  using ICameraPort::ICameraPort;
};

class RosLogAdapter : public ILogPort
{
  Q_OBJECT

public:
  RosLogAdapter(rclcpp::Logger logger, QObject * parent);
  void logEvent(int level, const QString & text) override;

private:
  rclcpp::Logger logger_;
};

// ROS düğümü + ayrı thread'de MultiThreadedExecutor + port adaptörleri.
// Mesaj ↔ Qt tipi dönüşümü yalnız burada yapılır. ROS callback'leri hiçbir
// QObject'e dokunmaz; veri GUI thread'ine kuyruklu çağrıyla geçer (D-27).
class RosBridge
{
public:
  RosBridge(rclcpp::Node::SharedPtr node, const GuiConfig & config);
  ~RosBridge();

  RosBridge(const RosBridge &) = delete;
  RosBridge & operator=(const RosBridge &) = delete;

  IRobotCommandPort * commandPort() const { return command_.get(); }
  IRobotStatePort * statePort() const { return state_.get(); }
  ICameraPort * cameraPort() const { return camera_.get(); }
  ILogPort * logPort() const { return log_.get(); }

  // Abonelikler ve executor thread'i; stop() iptal eder ve join eder.
  void start();
  void stop();

private:
  struct CameraIo
  {
    QString id;
    int imageCounter{-1};
    int infoCounter{-1};
    rclcpp::CallbackGroup::SharedPtr group;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info;
    Mailbox<std::pair<QVideoFrame, FrameMeta>> frames;
    std::mutex infoMutex;
    QSize lastInfo;
    std::atomic<bool> warnedEncoding{false};
  };

  int counterFor(const QString & topic) const;
  void onOdom(const nav_msgs::msg::Odometry & msg);
  void onJointStates(const sensor_msgs::msg::JointState & msg);
  void onClock(const rosgraph_msgs::msg::Clock & msg);
  void onDetections(const vision_msgs::msg::Detection2DArray & msg);
  void onRosout(const rcl_interfaces::msg::Log & msg);
  void onImage(CameraIo & cam, const sensor_msgs::msg::Image & msg);
  void onCameraInfo(CameraIo & cam, const sensor_msgs::msg::CameraInfo & msg);
  void pollGraph();

  rclcpp::Node::SharedPtr node_;
  GuiConfig config_;
  CounterRegistry counters_;
  std::string ownLogger_;
  std::string forkJoint_;
  std::string leftWheelJoint_;
  std::string rightWheelJoint_;
  QString detectionCamera_;
  qint64 clockPeriodNs_;

  std::unique_ptr<RosCommandAdapter> command_;
  std::unique_ptr<RosStateAdapter> state_;
  std::unique_ptr<RosCameraAdapter> camera_;
  std::unique_ptr<RosLogAdapter> log_;

  rclcpp::CallbackGroup::SharedPtr stateGroup_;
  rclcpp::CallbackGroup::SharedPtr logGroup_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockSub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detectionSub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosoutSub_;
  std::vector<std::unique_ptr<CameraIo>> cameras_;

  int odomCounter_{-1};
  int jointCounter_{-1};
  int clockCounter_{-1};
  int detectionCounter_{-1};
  int rosoutCounter_{-1};

  Mailbox<OdomSample> odomBox_;
  Mailbox<JointSample> jointBox_;
  Mailbox<DetectionSet> detectionBox_;
  std::atomic<qint64> lastClockPostNs_{-1};
  std::mutex logMutex_;
  std::vector<LogEntry> logQueue_;
  bool logPending_{false};

  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinThread_;
  QTimer graphTimer_;
  bool running_{false};
};

}  // namespace ros_adapter
}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__ROS__ROSBRIDGE_HPP_
