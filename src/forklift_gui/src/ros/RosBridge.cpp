// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/ros/RosBridge.hpp"

#include <QDateTime>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/ros/ImageConverter.hpp"

namespace forklift_gui
{
namespace ros_adapter
{

namespace
{

// Log kuyruğu üst sınırı: GUI takılırsa bellek büyümesin (en eskiler düşer).
constexpr size_t kMaxQueuedLogs = 2000;
// Grafik (ROS bağlantısı) yoklama aralığı [ms]; yalnız TopBar LED'i için.
constexpr int kGraphPollMs = 1000;

double stampToSeconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

// Kuaterniyondan z ekseni etrafında dönüş (yaw).
double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// Komutlar ve e-stop: güvenilir, uçucu, yalnız son değer (D-26).
rclcpp::QoS commandQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

// Görüntü ve camera_info: en yeni kare yeter, kuyruk gecikmesi olmasın.
rclcpp::QoS imageQos() { return rclcpp::SensorDataQoS().keep_last(1); }

// /rosout: GUI'den önce başlayan düğümlerin son kayıtları da gelsin.
rclcpp::QoS rosoutQos() { return rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local(); }

}  // namespace

// ------------------------------------------------------------- sayaçlar --

CounterRegistry::CounterRegistry(const QStringList & keys)
{
  for (const QString & key : keys) {
    if (index_.contains(key)) {
      continue;
    }
    auto slot = std::make_unique<Slot>();
    slot->key = key;
    index_.insert(key, static_cast<int>(slots_.size()));
    slots_.push_back(std::move(slot));
  }
}

void CounterRegistry::bump(int index, qint64 nowNs)
{
  if (index < 0 || index >= static_cast<int>(slots_.size())) {
    return;
  }
  Slot & s = *slots_[static_cast<size_t>(index)];
  s.count.fetch_add(1, std::memory_order_relaxed);
  s.lastNs.store(nowNs, std::memory_order_relaxed);
}

QVector<TopicCounter> CounterRegistry::snapshot() const
{
  QVector<TopicCounter> out;
  out.reserve(static_cast<int>(slots_.size()));
  for (const auto & s : slots_) {
    TopicCounter c;
    c.key = s->key;
    c.count = s->count.load(std::memory_order_relaxed);
    c.lastNs = s->lastNs.load(std::memory_order_relaxed);
    out.append(c);
  }
  return out;
}

// ------------------------------------------------------------ adaptörler --

RosCommandAdapter::RosCommandAdapter(
  rclcpp::Node & node, const GuiConfig::Topics & topics, QObject * parent)
: IRobotCommandPort(parent),
  cmdVel_(
    node.create_publisher<geometry_msgs::msg::Twist>(topics.cmdVel.toStdString(), commandQos())),
  fork_(node.create_publisher<std_msgs::msg::Float64>(topics.forkCmd.toStdString(), commandQos())),
  eStop_(node.create_publisher<std_msgs::msg::Bool>(topics.eStop.toStdString(), commandQos()))
{
}

RosAutonomyAdapter::RosAutonomyAdapter(
  rclcpp::Node & node, const GuiConfig::Autonomy & config, QObject * parent)
: IAutonomyPort(parent),
  mode_(node.create_client<forklift_interfaces::srv::SetMode>(config.modeService.toStdString()))
{
  const std::pair<int, QString> names[] = {
    {Start, config.startService},
    {Pause, config.pauseService},
    {Resume, config.resumeService},
    {Cancel, config.cancelService}};
  for (const auto & [cmd, name] : names) {
    triggers_[cmd] = node.create_client<std_srvs::srv::Trigger>(name.toStdString());
  }
}

void RosAutonomyAdapter::request(Command command)
{
  using SetMode = forklift_interfaces::srv::SetMode;
  using Trigger = std_srvs::srv::Trigger;
  const int cmd = command;
  if (command == SetManual || command == SetAuto) {
    if (!mode_->service_is_ready()) {
      finish(cmd, false, tr("servis yok"));
      return;
    }
    auto req = std::make_shared<SetMode::Request>();
    req->mode = command == SetAuto ? SetMode::Request::MODE_AUTO : SetMode::Request::MODE_MANUAL;
    mode_->async_send_request(req, [this, cmd](rclcpp::Client<SetMode>::SharedFuture f) {
      const auto r = f.get();
      finish(cmd, r->success, QString::fromStdString(r->message));
    });
    return;
  }
  auto it = triggers_.find(cmd);
  if (it == triggers_.end() || !it->second->service_is_ready()) {
    finish(cmd, false, tr("servis yok"));
    return;
  }
  it->second->async_send_request(
    std::make_shared<Trigger::Request>(), [this, cmd](rclcpp::Client<Trigger>::SharedFuture f) {
      const auto r = f.get();
      finish(cmd, r->success, QString::fromStdString(r->message));
    });
}

void RosAutonomyAdapter::finish(int command, bool ok, const QString & message)
{
  QMetaObject::invokeMethod(
    this, [this, command, ok, message]() { emit commandFinished(command, ok, message); },
    Qt::QueuedConnection);
}

void RosAutonomyAdapter::postStatus(const AutonomyStatus & status)
{
  QMetaObject::invokeMethod(
    this, [this, status]() { emit statusReceived(status); }, Qt::QueuedConnection);
}

void RosCommandAdapter::publishVelocity(double linear, double angular)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  cmdVel_->publish(msg);
}

void RosCommandAdapter::publishForkTarget(double height)
{
  std_msgs::msg::Float64 msg;
  msg.data = height;
  fork_->publish(msg);
}

void RosCommandAdapter::publishEStop(bool engaged)
{
  std_msgs::msg::Bool msg;
  msg.data = engaged;
  eStop_->publish(msg);
}

RosStateAdapter::RosStateAdapter(const CounterRegistry & counters, QObject * parent)
: IRobotStatePort(parent), counters_(counters)
{
}

void RosStateAdapter::setGraphStatus(const GraphStatus & status)
{
  const bool changed = status.up != graph_.up || status.peerNodes != graph_.peerNodes ||
                       status.inputsWithPublisher != graph_.inputsWithPublisher;
  graph_ = status;
  if (changed) {
    emit graphStatusChanged(status);
  }
}

RosLogAdapter::RosLogAdapter(rclcpp::Logger logger, QObject * parent)
: ILogPort(parent), logger_(std::move(logger))
{
}

void RosLogAdapter::logEvent(int level, const QString & text)
{
  const QByteArray utf8 = text.toUtf8();
  if (level >= static_cast<int>(LogLevel::Error)) {
    RCLCPP_ERROR(logger_, "%s", utf8.constData());
  } else if (level >= static_cast<int>(LogLevel::Warn)) {
    RCLCPP_WARN(logger_, "%s", utf8.constData());
  } else if (level >= static_cast<int>(LogLevel::Info)) {
    RCLCPP_INFO(logger_, "%s", utf8.constData());
  } else {
    RCLCPP_DEBUG(logger_, "%s", utf8.constData());
  }
}

// --------------------------------------------------------------- köprü --

namespace
{

QStringList healthKeys(const GuiConfig & config)
{
  QStringList keys;
  for (const HealthTopicConfig & h : config.health.topics) {
    keys << h.key;
  }
  return keys;
}

}  // namespace

RosBridge::RosBridge(rclcpp::Node::SharedPtr node, const GuiConfig & config)
: node_(std::move(node)),
  config_(config),
  counters_(healthKeys(config)),
  ownLogger_(node_->get_logger().get_name()),
  forkJoint_(config.fork.jointName.toStdString()),
  leftWheelJoint_(config.telemetry.wheelJoints.value(0).toStdString()),
  rightWheelJoint_(config.telemetry.wheelJoints.value(1).toStdString()),
  clockPeriodNs_(secondsToNs(1.0 / config.telemetry.uiMaxRate))
{
  command_ = std::make_unique<RosCommandAdapter>(*node_, config_.topics, nullptr);
  state_ = std::make_unique<RosStateAdapter>(counters_, nullptr);
  camera_ = std::make_unique<RosCameraAdapter>(nullptr);
  log_ = std::make_unique<RosLogAdapter>(node_->get_logger(), nullptr);
  autonomy_ = std::make_unique<RosAutonomyAdapter>(*node_, config_.autonomy, nullptr);
  for (const CameraConfig & cam : config_.cameras.list) {
    if (cam.detections && detectionCamera_.isEmpty()) {
      detectionCamera_ = cam.id;
    }
  }
  graphTimer_.setInterval(kGraphPollMs);
  QObject::connect(&graphTimer_, &QTimer::timeout, state_.get(), [this]() { pollGraph(); });
}

RosBridge::~RosBridge()
{
  stop();
  // Önce QObject adaptörleri: kuyrukta bekleyen teslim lambdaları onlarla
  // birlikte silinir; CameraIo ve posta kutuları ancak ondan sonra gider.
  command_.reset();
  state_.reset();
  camera_.reset();
  log_.reset();
  autonomy_.reset();
}

int RosBridge::counterFor(const QString & topic) const
{
  for (const HealthTopicConfig & h : config_.health.topics) {
    if (h.topic == topic) {
      return counters_.indexOf(h.key);
    }
  }
  return -1;
}

void RosBridge::start()
{
  if (running_) {
    return;
  }
  running_ = true;
  using rclcpp::CallbackGroupType;
  stateGroup_ = node_->create_callback_group(CallbackGroupType::MutuallyExclusive);
  logGroup_ = node_->create_callback_group(CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions stateOpts;
  stateOpts.callback_group = stateGroup_;
  rclcpp::SubscriptionOptions logOpts;
  logOpts.callback_group = logGroup_;

  const GuiConfig::Topics & t = config_.topics;
  odomCounter_ = counterFor(t.odom);
  jointCounter_ = counterFor(t.jointStates);
  clockCounter_ = counterFor(t.clock);
  detectionCounter_ = counterFor(t.detections);
  rosoutCounter_ = counterFor(t.rosout);

  odomSub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    t.odom.toStdString(), rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry & m) { onOdom(m); }, stateOpts);
  jointSub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    t.jointStates.toStdString(), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState & m) { onJointStates(m); }, stateOpts);
  clockSub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    t.clock.toStdString(), rclcpp::ClockQoS(),
    [this](const rosgraph_msgs::msg::Clock & m) { onClock(m); }, stateOpts);
  detectionSub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    t.detections.toStdString(), rclcpp::SensorDataQoS(),
    [this](const vision_msgs::msg::Detection2DArray & m) { onDetections(m); }, stateOpts);
  autonomySub_ = node_->create_subscription<forklift_interfaces::msg::AutonomyState>(
    config_.autonomy.stateTopic.toStdString(), rclcpp::QoS(5),
    [this](const forklift_interfaces::msg::AutonomyState & m) { onAutonomyState(m); }, stateOpts);
  rosoutSub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
    t.rosout.toStdString(), rosoutQos(),
    [this](const rcl_interfaces::msg::Log & m) { onRosout(m); }, logOpts);

  // Her kameranın kendi grubu: bir kameranın dönüşümü diğerini ve
  // odometriyi bekletmez.
  for (const CameraConfig & cfg : config_.cameras.list) {
    auto cam = std::make_unique<CameraIo>();
    cam->id = cfg.id;
    cam->imageCounter = counterFor(cfg.imageTopic);
    cam->infoCounter = counterFor(cfg.infoTopic);
    cam->group = node_->create_callback_group(CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cam->group;
    CameraIo * raw = cam.get();
    cam->image = node_->create_subscription<sensor_msgs::msg::Image>(
      cfg.imageTopic.toStdString(), imageQos(),
      [this, raw](const sensor_msgs::msg::Image & m) { onImage(*raw, m); }, opts);
    cam->info = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      cfg.infoTopic.toStdString(), imageQos(),
      [this, raw](const sensor_msgs::msg::CameraInfo & m) { onCameraInfo(*raw, m); }, opts);
    cameras_.push_back(std::move(cam));
  }

  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), static_cast<size_t>(config_.ros.executorThreads));
  executor_->add_node(node_);
  spinThread_ = std::thread([this]() { executor_->spin(); });

  pollGraph();
  graphTimer_.start();
}

void RosBridge::stop()
{
  if (!running_) {
    return;
  }
  running_ = false;
  graphTimer_.stop();
  // Önce executor durur ve thread birleşir; ardından callback'ler kesin
  // bitmiş olur ve abonelikler güvenle bırakılır.
  executor_->cancel();
  if (spinThread_.joinable()) {
    spinThread_.join();
  }
  executor_->remove_node(node_);
  executor_.reset();
  odomSub_.reset();
  jointSub_.reset();
  clockSub_.reset();
  detectionSub_.reset();
  rosoutSub_.reset();
  autonomySub_.reset();
  // CameraIo nesneleri yıkıma kadar kalır (bekleyen teslimler işaret eder).
  for (auto & cam : cameras_) {
    cam->image.reset();
    cam->info.reset();
  }
}

void RosBridge::onAutonomyState(const forklift_interfaces::msg::AutonomyState & msg)
{
  AutonomyStatus s;
  s.mode = msg.mode;
  s.state = QString::fromStdString(msg.state);
  s.detail = QString::fromStdString(msg.detail);
  s.targetColor = QString::fromStdString(msg.target_color);
  s.fault = QString::fromStdString(msg.fault);
  s.parcelsDone = static_cast<int>(msg.parcels_done);
  s.parcelsTotal = static_cast<int>(msg.parcels_total);
  s.receivedNs = steadyNowNs();
  autonomy_->postStatus(s);
}

void RosBridge::onOdom(const nav_msgs::msg::Odometry & msg)
{
  counters_.bump(odomCounter_, steadyNowNs());
  OdomSample s;
  s.x = msg.pose.pose.position.x;
  s.y = msg.pose.pose.position.y;
  const auto & q = msg.pose.pose.orientation;
  s.yaw = yawFromQuaternion(q.x, q.y, q.z, q.w);
  s.linear = msg.twist.twist.linear.x;
  s.angular = msg.twist.twist.angular.z;
  s.stamp = stampToSeconds(msg.header.stamp);
  if (odomBox_.put(s)) {
    QMetaObject::invokeMethod(
      state_.get(),
      [this]() {
        if (auto v = odomBox_.take()) {
          emit state_->odometryReceived(*v);
        }
      },
      Qt::QueuedConnection);
  }
}

void RosBridge::onJointStates(const sensor_msgs::msg::JointState & msg)
{
  counters_.bump(jointCounter_, steadyNowNs());
  JointSample s;
  s.stamp = stampToSeconds(msg.header.stamp);
  int left = -1;
  int right = -1;
  for (size_t i = 0; i < msg.name.size(); ++i) {
    const std::string & n = msg.name[i];
    if (n == forkJoint_ && i < msg.position.size()) {
      s.hasFork = true;
      s.forkHeight = msg.position[i];
    } else if (n == leftWheelJoint_) {
      left = static_cast<int>(i);
    } else if (n == rightWheelJoint_) {
      right = static_cast<int>(i);
    }
  }
  if (left >= 0 && right >= 0 && static_cast<size_t>(std::max(left, right)) < msg.velocity.size()) {
    s.hasWheels = true;
    s.leftWheel = msg.velocity[static_cast<size_t>(left)];
    s.rightWheel = msg.velocity[static_cast<size_t>(right)];
  }
  if (jointBox_.put(s)) {
    QMetaObject::invokeMethod(
      state_.get(),
      [this]() {
        if (auto v = jointBox_.take()) {
          emit state_->jointStatesReceived(*v);
        }
      },
      Qt::QueuedConnection);
  }
}

void RosBridge::onClock(const rosgraph_msgs::msg::Clock & msg)
{
  const qint64 now = steadyNowNs();
  counters_.bump(clockCounter_, now);
  // /clock 1 kHz akar; GUI'ye kaynağında en fazla ui_max_rate ile geçer.
  const qint64 last = lastClockPostNs_.load(std::memory_order_relaxed);
  if (last >= 0 && now - last < clockPeriodNs_) {
    return;
  }
  lastClockPostNs_.store(now, std::memory_order_relaxed);
  const double sim = stampToSeconds(msg.clock);
  QMetaObject::invokeMethod(
    state_.get(), [this, sim, now]() { emit state_->clockReceived(sim, now); },
    Qt::QueuedConnection);
}

void RosBridge::onDetections(const vision_msgs::msg::Detection2DArray & msg)
{
  const qint64 now = steadyNowNs();
  counters_.bump(detectionCounter_, now);
  if (detectionCamera_.isEmpty()) {
    return;
  }
  DetectionSet set;
  set.cameraId = detectionCamera_;
  set.stamp = stampToSeconds(msg.header.stamp);
  set.receivedNs = now;
  for (const auto & d : msg.detections) {
    DetectionBox b;
    b.cx = d.bbox.center.position.x;
    b.cy = d.bbox.center.position.y;
    b.width = d.bbox.size_x;
    b.height = d.bbox.size_y;
    // En yüksek skorlu hipotez etiket olur.
    double best = -1.0;
    for (const auto & r : d.results) {
      if (r.hypothesis.score > best) {
        best = r.hypothesis.score;
        b.label = QString::fromStdString(r.hypothesis.class_id);
        b.score = r.hypothesis.score;
      }
    }
    if (b.label.isEmpty()) {
      b.label = QString::fromStdString(d.id);
    }
    set.boxes.append(b);
  }
  if (detectionBox_.put(set)) {
    QMetaObject::invokeMethod(
      state_.get(),
      [this]() {
        if (auto v = detectionBox_.take()) {
          emit state_->detectionsReceived(*v);
        }
      },
      Qt::QueuedConnection);
  }
}

void RosBridge::onRosout(const rcl_interfaces::msg::Log & msg)
{
  counters_.bump(rosoutCounter_, steadyNowNs());
  if (msg.name == ownLogger_) {
    return;  // GUI olayları LogPanel'e zaten yerel olarak eklendi
  }
  LogEntry e;
  e.stamp = stampToSeconds(msg.stamp);
  e.wallMs = QDateTime::currentMSecsSinceEpoch();
  e.level = msg.level;
  e.node = QString::fromStdString(msg.name);
  e.text = QString::fromStdString(msg.msg);
  bool post = false;
  {
    std::lock_guard<std::mutex> lock(logMutex_);
    logQueue_.push_back(std::move(e));
    if (logQueue_.size() > kMaxQueuedLogs) {
      logQueue_.erase(logQueue_.begin());
    }
    if (!logPending_) {
      logPending_ = true;
      post = true;
    }
  }
  if (post) {
    QMetaObject::invokeMethod(
      log_.get(),
      [this]() {
        std::vector<LogEntry> batch;
        {
          std::lock_guard<std::mutex> lock(logMutex_);
          batch.swap(logQueue_);
          logPending_ = false;
        }
        emit log_->entriesReceived(QVector<LogEntry>(batch.begin(), batch.end()));
      },
      Qt::QueuedConnection);
  }
}

void RosBridge::onImage(CameraIo & cam, const sensor_msgs::msg::Image & msg)
{
  const qint64 now = steadyNowNs();
  counters_.bump(cam.imageCounter, now);
  QString error;
  QVideoFrame frame = toVideoFrame(msg, &error);
  if (!frame.isValid()) {
    if (!cam.warnedEncoding.exchange(true)) {
      RCLCPP_WARN(
        node_->get_logger(), "%s karesi gösterilemiyor: %s", qPrintable(cam.id), qPrintable(error));
    }
    return;
  }
  FrameMeta meta;
  meta.stamp = stampToSeconds(msg.header.stamp);
  meta.receivedNs = now;
  meta.encoding = QString::fromStdString(msg.encoding);
  if (cam.frames.put({std::move(frame), meta})) {
    CameraIo * raw = &cam;
    QMetaObject::invokeMethod(
      camera_.get(),
      [this, raw]() {
        if (auto v = raw->frames.take()) {
          emit camera_->frameReceived(raw->id, v->first, v->second);
        }
      },
      Qt::QueuedConnection);
  }
}

void RosBridge::onCameraInfo(CameraIo & cam, const sensor_msgs::msg::CameraInfo & msg)
{
  counters_.bump(cam.infoCounter, steadyNowNs());
  const QSize size(static_cast<int>(msg.width), static_cast<int>(msg.height));
  {
    std::lock_guard<std::mutex> lock(cam.infoMutex);
    if (size == cam.lastInfo) {
      return;  // yalnız değişince GUI'ye
    }
    cam.lastInfo = size;
  }
  const QString id = cam.id;
  QMetaObject::invokeMethod(
    camera_.get(), [this, id, size]() { emit camera_->cameraInfoReceived(id, size); },
    Qt::QueuedConnection);
}

void RosBridge::pollGraph()
{
  // GUI thread'inde; grafik API'si thread güvenli.
  GraphStatus g;
  g.up = rclcpp::ok();
  if (g.up) {
    const auto names = node_->get_node_names();
    g.peerNodes = std::max(0, static_cast<int>(names.size()) - 1);
    QStringList inputs{config_.topics.odom, config_.topics.jointStates, config_.topics.clock};
    for (const CameraConfig & cam : config_.cameras.list) {
      inputs << cam.imageTopic;
    }
    for (const QString & topic : inputs) {
      if (node_->count_publishers(topic.toStdString()) > 0) {
        ++g.inputsWithPublisher;
      }
    }
  }
  state_->setGraphStatus(g);
}

}  // namespace ros_adapter
}  // namespace forklift_gui
