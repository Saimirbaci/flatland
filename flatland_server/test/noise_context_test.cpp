/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	noise_context_test.cpp
 * @brief	Unit tests for NoiseContextProvider (surface/speed/lighting/age)
 * @author Saimir Baci
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <flatland_server/noise_context.h>
#include <gtest/gtest.h>

using flatland_server::NoiseContext;
using flatland_server::NoiseContextProvider;

// A default provider (no world configured) yields the legacy-safe context:
// surface 0, lighting 1.0, and the supplied sensor age.
TEST(NoiseContextProviderTest, defaults_are_legacy_safe) {
  NoiseContextProvider::Get().Reset();
  NoiseContext ctx = NoiseContextProvider::Get().Build(nullptr, 5.0);
  EXPECT_EQ(ctx.surface_id, 0);
  EXPECT_DOUBLE_EQ(ctx.lighting, 1.0);
  EXPECT_DOUBLE_EQ(ctx.speed, 0.0);
  EXPECT_DOUBLE_EQ(ctx.sensor_age, 5.0);
}

// Lighting is stored and clamped to [0, 1].
TEST(NoiseContextProviderTest, lighting_is_clamped) {
  NoiseContextProvider::Get().Reset();
  NoiseContextProvider::Get().SetLighting(0.3);
  EXPECT_DOUBLE_EQ(NoiseContextProvider::Get().Lighting(), 0.3);
  NoiseContextProvider::Get().SetLighting(5.0);
  EXPECT_DOUBLE_EQ(NoiseContextProvider::Get().Lighting(), 1.0);
  NoiseContextProvider::Get().SetLighting(-2.0);
  EXPECT_DOUBLE_EQ(NoiseContextProvider::Get().Lighting(), 0.0);
}

// Negative sensor ages are floored at zero.
TEST(NoiseContextProviderTest, sensor_age_is_non_negative) {
  NoiseContextProvider::Get().Reset();
  NoiseContext ctx = NoiseContextProvider::Get().Build(nullptr, -3.0);
  EXPECT_DOUBLE_EQ(ctx.sensor_age, 0.0);
}

// Surface bucketing follows the documented thresholds against a friction field.
TEST(NoiseContextProviderTest, surface_bucketing_thresholds) {
  NoiseContextProvider::Get().Reset();
  // friction factor depends only on x for this stub: x < 0 -> wet.
  NoiseContextProvider::Get().SetSurfaceSampler([](double x, double /*y*/) {
    if (x < -1.0) return 0.4;  // slipperiest -> bucket 2
    if (x < 0.0) return 0.7;   // medium -> bucket 1
    return 1.0;                // dry -> bucket 0
  });
  EXPECT_EQ(NoiseContextProvider::Get().SurfaceIdAt(2.0, 0.0), 0);
  EXPECT_EQ(NoiseContextProvider::Get().SurfaceIdAt(-0.5, 0.0), 1);
  EXPECT_EQ(NoiseContextProvider::Get().SurfaceIdAt(-2.0, 0.0), 2);
}

// Reset() drops the surface sampler and restores default lighting.
TEST(NoiseContextProviderTest, reset_restores_defaults) {
  NoiseContextProvider::Get().SetLighting(0.1);
  NoiseContextProvider::Get().SetSurfaceSampler(
      [](double, double) { return 0.0; });
  NoiseContextProvider::Get().Reset();
  EXPECT_DOUBLE_EQ(NoiseContextProvider::Get().Lighting(), 1.0);
  // No sampler -> always bucket 0 regardless of position.
  EXPECT_EQ(NoiseContextProvider::Get().SurfaceIdAt(-100.0, 0.0), 0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
