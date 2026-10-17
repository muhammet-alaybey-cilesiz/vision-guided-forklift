// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// cmd_vel_gate: twist_mux çıkışı ile DiffDrive köprüsü arasındaki emniyet
// kapısı (DECISIONS D-24). twist_mux kilitlenince yalnız iletmeyi keser ve
// gz DiffDrive son komutu süresiz tutar; bu düğüm "iletim yok"u "sıfır
// komut"a çevirir:
//   * /e_stop serbest ve tazeyken girdiyi olduğu gibi iletir,
//   * /e_stop yükselen kenarında HEMEN sıfır yayınlar,
//   * e-stop etkin / eskimiş ya da girdi sessizken zero_rate ile sıfır yayınlar.
// Tüm zamanlar duvar (monoton) saatidir (D-25). Parametrelerin varsayılanı
// yoktur; forklift_safety/config/cmd_vel_gate.yaml verilmelidir.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "forklift_safety/GateLogic.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace forklift_safety
{

namespace
{

int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

rclcpp::QoS reliableLast() { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); }

}  // namespace

class CmdVelGate : public rclcpp::Node
{
public:
  CmdVelGate() : Node("cmd_vel_gate")
  {
    GateConfig config;
    config.inputTimeout = declare_parameter<double>("input_timeout");
    config.eStopTimeout = declare_parameter<double>("e_stop_timeout");
    config.zeroRate = declare_parameter<double>("zero_rate");
    const double checkRate = declare_parameter<double>("check_rate");
    const std::string input = declare_parameter<std::string>("input_topic");
    const std::string output = declare_parameter<std::string>("output_topic");
    const std::string eStop = declare_parameter<std::string>("e_stop_topic");
    logic_ = std::make_unique<GateLogic>(config);

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(output, reliableLast());
    inputSub_ = create_subscription<geometry_msgs::msg::Twist>(
      input, reliableLast(), [this](const geometry_msgs::msg::Twist & msg) {
        if (logic_->onInput(steadyNowNs())) {
          publisher_->publish(msg);
          reportState(false);
        }
      });
    eStopSub_ = create_subscription<std_msgs::msg::Bool>(
      eStop, reliableLast(), [this](const std_msgs::msg::Bool & msg) {
        const int64_t now = steadyNowNs();
        if (logic_->onEStop(msg.data, now)) {
          publishZero(now);  // kilitlenme anı: gecikmesiz sıfır
        }
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / checkRate), [this]() {
      const int64_t now = steadyNowNs();
      if (logic_->onTimer(now)) {
        publishZero(now);
      }
    });
    RCLCPP_INFO(
      get_logger(), "%s -> %s (e-stop: %s, girdi zaman aşımı %.2f s, kilit %.2f s)", input.c_str(),
      output.c_str(), eStop.c_str(), config.inputTimeout, config.eStopTimeout);
  }

private:
  void publishZero(int64_t nowNs)
  {
    publisher_->publish(geometry_msgs::msg::Twist());
    logic_->markZeroPublished(nowNs);
    reportState(true);
  }

  // Durum geçişlerini bir kez loglar (her sıfırda değil).
  void reportState(bool stopped)
  {
    if (stopped == stopped_) {
      return;
    }
    stopped_ = stopped;
    if (stopped) {
      RCLCPP_WARN(get_logger(), "Durdurma: e-stop etkin/eski ya da girdi yok; sıfır komut");
    } else {
      RCLCPP_INFO(get_logger(), "Komut iletimi devam ediyor");
    }
  }

  std::unique_ptr<GateLogic> logic_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr inputSub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr eStopSub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool stopped_{false};
};

}  // namespace forklift_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<forklift_safety::CmdVelGate>());
  } catch (const std::exception & e) {
    // Eksik parametre: düğüm başlamaz, launch hatayı gösterir.
    std::fprintf(stderr, "cmd_vel_gate: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
