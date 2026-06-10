/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	noise_model_test.cpp
 * @brief	Unit tests for the parametric_linear NoiseModel loader +
 * sampling
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

#include <flatland_server/exceptions.h>
#include <flatland_server/noise_context.h>
#include <flatland_server/noise_model.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>

using flatland_server::NoiseContext;
using flatland_server::NoiseModel;
using flatland_server::NoiseParams;
using flatland_server::YAMLException;

namespace {

// Write `content` to a unique temp file and return its path.
std::string WriteTemp(const std::string &name, const std::string &content) {
  std::string path = std::string(testing::TempDir()) + "/" + name;
  std::ofstream out(path);
  out << content;
  out.close();
  return path;
}

const char *kCalibrated = R"JSON({
  "schema_version": 1,
  "model_type": "parametric_linear",
  "channels": {
    "range": {
      "base_std": 0.01,
      "std_coef": { "speed": 0.05, "age": 0.001, "darkness": 0.02 },
      "surface_std_offset": { "0": 0.0, "1": 0.01, "2": 0.03 },
      "base_mean": 0.0
    }
  }
})JSON";

}  // namespace

// Legacy() is a pass-through: any channel returns {0, fallback_std}.
TEST(NoiseModelTest, legacy_passthrough_uses_fallback) {
  auto m = NoiseModel::Legacy();
  EXPECT_TRUE(m->IsLegacy());
  NoiseContext ctx;
  ctx.speed = 3.0;  // ignored in legacy mode
  NoiseParams p = m->GetParams("range", ctx, 0.07);
  EXPECT_DOUBLE_EQ(p.mean, 0.0);
  EXPECT_DOUBLE_EQ(p.std, 0.07);
}

// A valid file loads and is not legacy; a known channel uses its coefficients.
TEST(NoiseModelTest, loads_valid_file) {
  std::string path = WriteTemp("nm_valid.json", kCalibrated);
  auto m = NoiseModel::FromFile(path);
  EXPECT_FALSE(m->IsLegacy());
  EXPECT_TRUE(m->HasChannel("range"));

  NoiseContext ctx;
  ctx.speed = 2.0;
  ctx.sensor_age = 10.0;
  ctx.lighting = 0.5;  // darkness = 0.5
  ctx.surface_id = 2;
  // std = 0.01 + 0.05*2 + 0.001*10 + 0.02*0.5 + 0.03 = 0.01+0.1+0.01+0.01+0.03
  NoiseParams p = m->GetParams("range", ctx, 0.0);
  EXPECT_NEAR(p.std, 0.16, 1e-9);
  EXPECT_DOUBLE_EQ(p.mean, 0.0);
}

// An unmodelled channel falls back to the supplied legacy std.
TEST(NoiseModelTest, unknown_channel_falls_back) {
  std::string path = WriteTemp("nm_valid2.json", kCalibrated);
  auto m = NoiseModel::FromFile(path);
  NoiseContext ctx;
  NoiseParams p = m->GetParams("imu_yaw", ctx, 0.123);
  EXPECT_DOUBLE_EQ(p.std, 0.123);
}

// Wrong schema_version is rejected.
TEST(NoiseModelTest, rejects_bad_schema_version) {
  std::string path = WriteTemp(
      "nm_badver.json",
      "{\"schema_version\": 2, \"model_type\": \"parametric_linear\", "
      "\"channels\": {}}");
  EXPECT_THROW(NoiseModel::FromFile(path), YAMLException);
}

// Unsupported model_type is rejected.
TEST(NoiseModelTest, rejects_bad_model_type) {
  std::string path =
      WriteTemp("nm_badtype.json",
                "{\"schema_version\": 1, \"model_type\": \"neural_net\", "
                "\"channels\": {}}");
  EXPECT_THROW(NoiseModel::FromFile(path), YAMLException);
}

// A missing file is rejected (and LoadOrLegacy degrades gracefully instead).
TEST(NoiseModelTest, missing_file_throws_but_loadorlegacy_recovers) {
  EXPECT_THROW(NoiseModel::FromFile("/no/such/noise_model.json"),
               YAMLException);
  auto m = NoiseModel::LoadOrLegacy("/no/such/noise_model.json", "t/laser");
  EXPECT_TRUE(m->IsLegacy());
  // Empty path -> legacy without any warning.
  EXPECT_TRUE(NoiseModel::LoadOrLegacy("", "t/laser")->IsLegacy());
}

// std grows monotonically with speed (calibrated coefficient is positive).
TEST(NoiseModelTest, std_increases_with_speed) {
  std::string path = WriteTemp("nm_mono.json", kCalibrated);
  auto m = NoiseModel::FromFile(path);
  NoiseContext slow, fast;
  slow.speed = 0.0;
  fast.speed = 1.0;
  EXPECT_GT(m->GetParams("range", fast, 0.0).std,
            m->GetParams("range", slow, 0.0).std);
}

// Sampling is deterministic for a fixed engine seed and draws shift with
// context (a calibrated model produces a wider empirical spread at speed).
TEST(NoiseModelTest, sampling_is_deterministic_and_context_sensitive) {
  std::string path = WriteTemp("nm_det.json", kCalibrated);
  auto m = NoiseModel::FromFile(path);

  NoiseContext ctx;
  ctx.speed = 0.5;
  std::default_random_engine e1(42), e2(42);
  for (int i = 0; i < 50; i++) {
    EXPECT_DOUBLE_EQ(m->Sample("range", ctx, e1, 0.0),
                     m->Sample("range", ctx, e2, 0.0));
  }

  // Empirical std should be larger at higher speed.
  auto empirical_std = [&](double speed) {
    NoiseContext c;
    c.speed = speed;
    std::default_random_engine e(7);
    double sum = 0.0, sumsq = 0.0;
    const int n = 20000;
    for (int i = 0; i < n; i++) {
      double s = m->Sample("range", c, e, 0.0);
      sum += s;
      sumsq += s * s;
    }
    double mean = sum / n;
    return std::sqrt(sumsq / n - mean * mean);
  };
  EXPECT_GT(empirical_std(1.0), empirical_std(0.0));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
