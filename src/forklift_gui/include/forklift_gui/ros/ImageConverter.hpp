// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__ROS__IMAGECONVERTER_HPP_
#define FORKLIFT_GUI__ROS__IMAGECONVERTER_HPP_

#include <QString>
#include <QVideoFrame>

#include "sensor_msgs/msg/image.hpp"

namespace forklift_gui
{
namespace ros_adapter
{

// sensor_msgs/Image → QVideoFrame; ROS callback thread'inde çağrılır (D-27).
// Qt 6.4'te 24 bit RGB video formatı yok: rgb8/bgr8 satır satır RGBX/BGRX'e
// açılır; rgba8/bgra8/mono8 doğrudan kopyalanır (D-34). Desteklenmeyen
// kodlama ya da tutarsız boyutta geçersiz kare döner, error doldurulur.
QVideoFrame toVideoFrame(const sensor_msgs::msg::Image & image, QString * error);

}  // namespace ros_adapter
}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__ROS__IMAGECONVERTER_HPP_
