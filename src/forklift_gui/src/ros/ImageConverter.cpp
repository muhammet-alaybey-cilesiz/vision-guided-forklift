// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_gui/ros/ImageConverter.hpp"

#include <QVideoFrameFormat>
#include <cstring>
#include <string>

namespace forklift_gui
{
namespace ros_adapter
{

namespace
{

struct Layout
{
  QVideoFrameFormat::PixelFormat format;
  int sourceBytes;  // kaynak piksel başına bayt
  bool expand;      // 3 → 4 bayt (dolgu 0xff)
};

bool layoutFor(const std::string & encoding, Layout & out)
{
  if (encoding == "rgb8") {
    out = {QVideoFrameFormat::Format_RGBX8888, 3, true};
  } else if (encoding == "bgr8") {
    out = {QVideoFrameFormat::Format_BGRX8888, 3, true};
  } else if (encoding == "rgba8") {
    out = {QVideoFrameFormat::Format_RGBA8888, 4, false};
  } else if (encoding == "bgra8") {
    out = {QVideoFrameFormat::Format_BGRA8888, 4, false};
  } else if (encoding == "mono8") {
    out = {QVideoFrameFormat::Format_Y8, 1, false};
  } else {
    return false;
  }
  return true;
}

}  // namespace

QVideoFrame toVideoFrame(const sensor_msgs::msg::Image & image, QString * error)
{
  Layout layout{};
  if (!layoutFor(image.encoding, layout)) {
    if (error) {
      *error =
        QStringLiteral("desteklenmeyen kodlama: %1").arg(QString::fromStdString(image.encoding));
    }
    return {};
  }
  const int width = static_cast<int>(image.width);
  const int height = static_cast<int>(image.height);
  const size_t rowBytes = static_cast<size_t>(width) * layout.sourceBytes;
  if (
    width <= 0 || height <= 0 || image.step < rowBytes ||
    image.data.size() < static_cast<size_t>(image.step) * height) {
    if (error) {
      *error = QStringLiteral("tutarsız görüntü: %1x%2, step %3, %4 bayt")
                 .arg(width)
                 .arg(height)
                 .arg(image.step)
                 .arg(image.data.size());
    }
    return {};
  }

  QVideoFrame frame(QVideoFrameFormat(QSize(width, height), layout.format));
  if (!frame.map(QVideoFrame::WriteOnly)) {
    if (error) {
      *error = QStringLiteral("QVideoFrame eşlenemedi");
    }
    return {};
  }
  uchar * dst = frame.bits(0);
  const int dstStride = frame.bytesPerLine(0);
  const uint8_t * src = image.data.data();
  for (int y = 0; y < height; ++y) {
    const uint8_t * s = src + static_cast<size_t>(y) * image.step;
    uchar * d = dst + static_cast<size_t>(y) * dstStride;
    if (layout.expand) {
      for (int x = 0; x < width; ++x) {
        d[0] = s[0];
        d[1] = s[1];
        d[2] = s[2];
        d[3] = 0xff;
        s += 3;
        d += 4;
      }
    } else {
      std::memcpy(d, s, rowBytes);
    }
  }
  frame.unmap();
  return frame;
}

}  // namespace ros_adapter
}  // namespace forklift_gui
