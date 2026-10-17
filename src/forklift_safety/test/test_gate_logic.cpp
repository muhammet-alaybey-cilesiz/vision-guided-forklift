// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// GateLogic: e-stop yükselen kenarında anında sıfır, kilitliyken girdi
// düşer, girdi sessizliğinde zero_rate ile sıfır, e-stop eskiyince kilit.

#include <gtest/gtest.h>

#include <cstdint>

#include "forklift_safety/GateLogic.hpp"

using forklift_safety::GateConfig;
using forklift_safety::GateLogic;

namespace
{

constexpr int64_t kMs = 1000000;

GateConfig config()
{
  GateConfig c;
  c.inputTimeout = 0.2;
  c.eStopTimeout = 0.5;
  c.zeroRate = 10.0;
  return c;
}

}  // namespace

TEST(GateLogic, StartsBlockedUntilEStopReleased)
{
  GateLogic g(config());
  EXPECT_TRUE(g.blocked(0));
  EXPECT_FALSE(g.onInput(0));  // hiç /e_stop gelmedi: kilitli
  EXPECT_FALSE(g.onEStop(false, 10 * kMs));
  EXPECT_TRUE(g.onInput(20 * kMs));
}

TEST(GateLogic, RisingEdgeRequestsImmediateZeroAndDropsInput)
{
  GateLogic g(config());
  g.onEStop(false, 0);
  EXPECT_TRUE(g.onInput(10 * kMs));
  EXPECT_TRUE(g.onEStop(true, 20 * kMs));    // yükselen kenar
  EXPECT_FALSE(g.onEStop(true, 120 * kMs));  // tekrar eden true kenar değil
  EXPECT_FALSE(g.onInput(130 * kMs));        // kilitliyken iletilmez
}

TEST(GateLogic, SilenceProducesRateLimitedZeros)
{
  GateLogic g(config());
  g.onEStop(false, 0);
  g.onInput(0);
  EXPECT_FALSE(g.onTimer(100 * kMs));  // girdi taze
  g.onEStop(false, 100 * kMs);
  EXPECT_TRUE(g.onTimer(250 * kMs));  // 0.25 s sessizlik > 0.2 s
  g.markZeroPublished(250 * kMs);
  EXPECT_FALSE(g.onTimer(300 * kMs));  // 10 Hz: 100 ms dolmadı
  EXPECT_TRUE(g.onTimer(350 * kMs));
  g.markZeroPublished(350 * kMs);
  g.onEStop(false, 360 * kMs);
  EXPECT_TRUE(g.onInput(370 * kMs));  // girdi dönünce iletim ve sıfır biter
  EXPECT_FALSE(g.onTimer(380 * kMs));
}

TEST(GateLogic, StaleEStopBlocks)
{
  GateLogic g(config());
  g.onEStop(false, 0);
  EXPECT_TRUE(g.onInput(100 * kMs));
  EXPECT_FALSE(g.onInput(600 * kMs));  // 0.6 s'dir /e_stop yok: GUI çökmüş say
  EXPECT_TRUE(g.blocked(600 * kMs));
  EXPECT_TRUE(g.onTimer(600 * kMs));  // durdurma sıfırı
}
