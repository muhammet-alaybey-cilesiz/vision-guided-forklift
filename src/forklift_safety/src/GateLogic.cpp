// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "forklift_safety/GateLogic.hpp"

namespace forklift_safety
{

namespace
{

int64_t toNs(double seconds) { return static_cast<int64_t>(seconds * 1e9); }

}  // namespace

GateLogic::GateLogic(const GateConfig & config) : config_(config) {}

bool GateLogic::onInput(int64_t nowNs)
{
  lastInputNs_ = nowNs;
  return !blocked(nowNs);
}

bool GateLogic::onEStop(bool engaged, int64_t nowNs)
{
  lastEStopNs_ = nowNs;
  const bool rising = engaged && !eStop_;
  eStop_ = engaged;
  return rising;
}

bool GateLogic::onTimer(int64_t nowNs)
{
  if (!stopped(nowNs)) {
    return false;
  }
  return lastZeroNs_ < 0 || nowNs - lastZeroNs_ >= toNs(1.0 / config_.zeroRate);
}

bool GateLogic::blocked(int64_t nowNs) const
{
  return eStop_ || lastEStopNs_ < 0 || nowNs - lastEStopNs_ > toNs(config_.eStopTimeout);
}

bool GateLogic::stopped(int64_t nowNs) const
{
  return blocked(nowNs) || lastInputNs_ < 0 || nowNs - lastInputNs_ > toNs(config_.inputTimeout);
}

}  // namespace forklift_safety
