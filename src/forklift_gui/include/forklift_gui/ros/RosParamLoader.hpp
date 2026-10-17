// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__ROS__ROSPARAMLOADER_HPP_
#define FORKLIFT_GUI__ROS__ROSPARAMLOADER_HPP_

#include <stdexcept>
#include <string>

#include "forklift_gui/core/GuiConfig.hpp"
#include "rclcpp/rclcpp.hpp"

namespace forklift_gui
{
namespace ros_adapter
{

// Eksik ya da hatalı tipteki tüm parametreleri tek mesajda toplar.
class ConfigError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Düğümde her parametreyi VARSAYILANSIZ bildirir ve okur; eksik anahtar
// başlangıcı adıyla durdurur (DECISIONS D-29). Tamsayı, double yerine kabul edilir.
GuiConfig loadConfig(rclcpp::Node & node);

// --mock: aynı YAML'ı ROS grafiği başlatmadan ayrıştırır
// (rclcpp::parameter_map_from_yaml_file). nodeName tam ad: "/forklift_gui".
GuiConfig loadConfigFromYaml(const std::string & path, const std::string & nodeName);

}  // namespace ros_adapter
}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__ROS__ROSPARAMLOADER_HPP_
