// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef CORE__TESTSUPPORT_HPP_
#define CORE__TESTSUPPORT_HPP_

#include "forklift_gui/core/GuiConfig.hpp"
#include "forklift_gui/core/SteadyClock.hpp"
#include "forklift_gui/ros/RosParamLoader.hpp"

namespace forklift_gui
{
namespace test
{

// Testler paketin gerçek gui_params.yaml dosyasıyla koşar: limitler ve
// eşikler testte tekrar yazılmaz (D-29).
inline GuiConfig loadTestConfig()
{
  return ros_adapter::loadConfigFromYaml(FORKLIFT_GUI_PARAMS, "/forklift_gui");
}

// Elle ilerletilen sahte saat; denetleyiciler bununla deterministik (D-38).
struct FakeClock
{
  qint64 now{1000000000};

  ClockFn fn()
  {
    return [this]() { return now; };
  }

  void advance(double seconds) { now += secondsToNs(seconds); }
};

}  // namespace test
}  // namespace forklift_gui

#endif  // CORE__TESTSUPPORT_HPP_
