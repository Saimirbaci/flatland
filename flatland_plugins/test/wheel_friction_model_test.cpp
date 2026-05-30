/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name  wheel_friction_model_test.cpp
 * @brief Test the anisotropic wheel-ground friction model
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

#include <flatland_plugins/wheel_friction_model.h>
#include <yaml-cpp/yaml.h>
#include <gtest/gtest.h>

using namespace flatland_plugins;

/**
 * Default-constructed model is disabled and carries the calibrated defaults.
 */
TEST(WheelFrictionModelTest, test_defaults) {
  WheelFrictionModel m;
  EXPECT_FALSE(m.enabled_);
  EXPECT_NEAR(1.0, m.mu_long_static_, 1e-6);
  EXPECT_NEAR(0.8, m.mu_long_kinetic_, 1e-6);
  EXPECT_NEAR(1.1, m.mu_lat_static_, 1e-6);
  EXPECT_NEAR(0.9, m.mu_lat_kinetic_, 1e-6);
  EXPECT_NEAR(0.05, m.static_slip_threshold_, 1e-6);
}

/**
 * Configure parses every parameter from the YAML node.
 */
TEST(WheelFrictionModelTest, test_configure) {
  WheelFrictionModel m;
  YAML::Node config;
  config["enabled"] = true;
  config["mu_long_static"] = 1.3;
  config["mu_long_kinetic"] = 0.7;
  config["mu_lat_static"] = 1.5;
  config["mu_lat_kinetic"] = 1.1;
  config["static_slip_threshold"] = 0.02;
  m.Configure(config);

  EXPECT_TRUE(m.enabled_);
  EXPECT_NEAR(1.3, m.mu_long_static_, 1e-6);
  EXPECT_NEAR(0.7, m.mu_long_kinetic_, 1e-6);
  EXPECT_NEAR(1.5, m.mu_lat_static_, 1e-6);
  EXPECT_NEAR(1.1, m.mu_lat_kinetic_, 1e-6);
  EXPECT_NEAR(0.02, m.static_slip_threshold_, 1e-6);
}

/**
 * A large slip saturates each axis at its kinetic Coulomb ceiling mu*N, and the
 * lateral and longitudinal ceilings differ (anisotropy).
 */
TEST(WheelFrictionModelTest, test_clamp_to_mu_N_and_anisotropy) {
  WheelFrictionModel m;  // defaults: mu_long_kinetic 0.8, mu_lat_kinetic 0.9
  double N = 100.0;
  double dt = 0.01;

  // Slip far above the threshold -> kinetic regime, force way over the ceiling.
  b2Vec2 f = m.ComputeWheelForce(100.0, 100.0, N, dt);

  EXPECT_NEAR(0.8 * N, f.x, 1e-2);  // longitudinal ceiling
  EXPECT_NEAR(0.9 * N, f.y, 1e-2);  // lateral ceiling (higher -> anisotropic)
  EXPECT_GT(f.y, f.x);
}

/**
 * The static coefficient applies below the slip threshold and the (lower)
 * kinetic coefficient above it, so the longitudinal ceiling drops as the wheel
 * transitions from gripping to slipping.
 */
TEST(WheelFrictionModelTest, test_static_to_kinetic_transition) {
  WheelFrictionModel m;  // mu_long_static 1.0, mu_long_kinetic 0.8, thr 0.05
  double N = 10.0;
  double dt = 0.001;  // small dt so the nulling force exceeds the ceiling

  // Just below threshold: static ceiling 1.0 * N = 10
  b2Vec2 grip = m.ComputeWheelForce(0.04, 0.0, N, dt);
  EXPECT_NEAR(1.0 * N, grip.x, 1e-2);

  // Just above threshold: kinetic ceiling 0.8 * N = 8
  b2Vec2 slip = m.ComputeWheelForce(0.06, 0.0, N, dt);
  EXPECT_NEAR(0.8 * N, slip.x, 1e-2);

  EXPECT_GT(grip.x, slip.x);  // static grip exceeds kinetic
}

/**
 * Below the Coulomb ceiling the model returns the proportional nulling force
 * F = (N/g) * slip / dt (the grip regime), not a saturated value.
 */
TEST(WheelFrictionModelTest, test_grip_regime_nulling_force) {
  WheelFrictionModel m;
  double N = 100.0;
  double dt = 0.1;
  double slip = 0.01;  // below threshold; nulling force well under mu*N

  double effective_mass = N / WheelFrictionModel::kGravity;
  double expected = effective_mass * slip / dt;

  b2Vec2 f = m.ComputeWheelForce(slip, 0.0, N, dt);
  EXPECT_NEAR(expected, f.x, 1e-3);
  EXPECT_LT(f.x, 1.0 * N);  // not saturated
}

/**
 * Friction resists the slip: the force shares the sign of the slip velocity
 * (it drives the actual contact velocity toward the commanded one).
 */
TEST(WheelFrictionModelTest, test_sign_resists_slip) {
  WheelFrictionModel m;
  double N = 100.0;
  double dt = 0.01;

  b2Vec2 pos = m.ComputeWheelForce(50.0, 0.0, N, dt);
  b2Vec2 neg = m.ComputeWheelForce(-50.0, 0.0, N, dt);
  EXPECT_GT(pos.x, 0.0);
  EXPECT_LT(neg.x, 0.0);
  EXPECT_NEAR(-pos.x, neg.x, 1e-3);  // symmetric
}

/**
 * Degenerate inputs produce no force instead of a divide-by-zero / NaN.
 */
TEST(WheelFrictionModelTest, test_degenerate_inputs) {
  WheelFrictionModel m;
  b2Vec2 zero_dt = m.ComputeWheelForce(1.0, 1.0, 100.0, 0.0);
  EXPECT_NEAR(0.0, zero_dt.x, 1e-9);
  EXPECT_NEAR(0.0, zero_dt.y, 1e-9);

  b2Vec2 zero_load = m.ComputeWheelForce(1.0, 1.0, 0.0, 0.01);
  EXPECT_NEAR(0.0, zero_load.x, 1e-9);
  EXPECT_NEAR(0.0, zero_load.y, 1e-9);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
