// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_GUI__CORE__STEADYCLOCK_HPP_
#define FORKLIFT_GUI__CORE__STEADYCLOCK_HPP_

#include <QtGlobal>
#include <chrono>
#include <functional>

namespace forklift_gui
{

// Tüm katmanların ortak monoton saati [ns]. ROS thread'inde yazılan alım
// zamanları ile GUI thread'indeki yaş hesapları aynı saati kullanmalıdır.
inline qint64 steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

// Denetleyicilere enjekte edilen saat. Uygulamada steadyNowNs, testlerde
// elle ilerletilen sahte saat kullanılır (DECISIONS D-38).
using ClockFn = std::function<qint64()>;

constexpr double kNsPerSecond = 1e9;

inline double nsToSeconds(qint64 ns) { return static_cast<double>(ns) / kNsPerSecond; }

inline qint64 secondsToNs(double seconds) { return static_cast<qint64>(seconds * kNsPerSecond); }

}  // namespace forklift_gui

#endif  // FORKLIFT_GUI__CORE__STEADYCLOCK_HPP_
