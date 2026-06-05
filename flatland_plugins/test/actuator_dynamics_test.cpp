/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name  actuator_dynamics_test.cpp
 * @brief Unit tests for the ActuatorDynamics utility class
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

#include <flatland_plugins/actuator_dynamics.h>
#include <ros/time.h>
#include <yaml-cpp/yaml.h>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace flatland_plugins;

// Defaults leave the model a disabled no-op.
TEST(ActuatorDynamicsTest, configure_blank) {
  ActuatorDynamics a;
  EXPECT_NEAR(0.0, a.command_latency_, 1e-9);
  EXPECT_NEAR(0.0, a.deadband_, 1e-9);
  EXPECT_FALSE(a.deadband_rescale_);
  EXPECT_NEAR(0.0, a.max_effort_, 1e-9);
  EXPECT_FALSE(a.Enabled());
}

// An empty YAML node configures all defaults (pure pass-through).
TEST(ActuatorDynamicsTest, configure_empty_yaml) {
  ActuatorDynamics a;
  a.Configure(YAML::Node());
  EXPECT_NEAR(0.0, a.command_latency_, 1e-9);
  EXPECT_NEAR(0.0, a.deadband_, 1e-9);
  EXPECT_NEAR(0.0, a.max_effort_, 1e-9);
  EXPECT_FALSE(a.Enabled());
}

TEST(ActuatorDynamicsTest, configure_params) {
  ActuatorDynamics a;
  YAML::Node config;
  config["command_latency"] = 0.2;
  config["deadband"] = 0.05;
  config["deadband_rescale"] = true;
  config["max_force"] = 30.0;
  a.Configure(config);
  EXPECT_NEAR(0.2, a.command_latency_, 1e-9);
  EXPECT_NEAR(0.05, a.deadband_, 1e-9);
  EXPECT_TRUE(a.deadband_rescale_);
  EXPECT_NEAR(30.0, a.max_effort_, 1e-9);
  EXPECT_TRUE(a.Enabled());
}

// max_torque maps onto the same generic effort limit as max_force.
TEST(ActuatorDynamicsTest, configure_max_torque_maps_to_effort) {
  ActuatorDynamics a;
  YAML::Node config;
  config["max_torque"] = 12.5;
  a.Configure(config);
  EXPECT_NEAR(12.5, a.max_effort_, 1e-9);
  EXPECT_NEAR(12.5, a.ForceCap(), 1e-9);
}

// Deadband disabled is an identity pass-through.
TEST(ActuatorDynamicsTest, deadband_disabled_is_identity) {
  ActuatorDynamics a;
  EXPECT_NEAR(0.001, a.ApplyDeadband(0.001), 1e-9);
  EXPECT_NEAR(-5.0, a.ApplyDeadband(-5.0), 1e-9);
}

// Hard deadband zeroes sub-threshold commands and passes the rest unchanged.
TEST(ActuatorDynamicsTest, deadband_hard) {
  ActuatorDynamics a;
  YAML::Node config;
  config["deadband"] = 0.3;
  a.Configure(config);

  EXPECT_NEAR(0.0, a.ApplyDeadband(0.1), 1e-9);
  EXPECT_NEAR(0.0, a.ApplyDeadband(-0.29), 1e-9);
  EXPECT_NEAR(0.0, a.ApplyDeadband(0.0), 1e-9);
  // At and above threshold, the command passes through unchanged.
  EXPECT_NEAR(0.5, a.ApplyDeadband(0.5), 1e-9);
  EXPECT_NEAR(-0.4, a.ApplyDeadband(-0.4), 1e-9);
}

// Rescaled deadband zeroes sub-threshold and is continuous across the edge.
TEST(ActuatorDynamicsTest, deadband_rescaled) {
  ActuatorDynamics a;
  YAML::Node config;
  config["deadband"] = 0.3;
  config["deadband_rescale"] = true;
  a.Configure(config);

  EXPECT_NEAR(0.0, a.ApplyDeadband(0.2), 1e-9);
  // Just above the threshold maps to ~0 (no discontinuous jump).
  EXPECT_NEAR(0.0, a.ApplyDeadband(0.3), 1e-9);
  EXPECT_NEAR(0.2, a.ApplyDeadband(0.5), 1e-9);
  EXPECT_NEAR(-0.2, a.ApplyDeadband(-0.5), 1e-9);
}

// Latency disabled is a pass-through returning the latest pushed command.
TEST(ActuatorDynamicsTest, latency_disabled_passthrough) {
  ActuatorDynamics a;  // command_latency_ == 0
  ros::Time t(0.0);
  a.Push(1.0, t);
  EXPECT_NEAR(1.0, a.Pull(t), 1e-9);
  t = ros::Time(0.1);
  a.Push(2.0, t);
  EXPECT_NEAR(2.0, a.Pull(t), 1e-9);
}

// The FIFO returns the command delayed by exactly the configured latency.
// dt = 0.1 s, latency = 0.5 s => a 5-step delay. dt/latency chosen so every
// stamp lands on an exact nanosecond boundary (i * 1e8 ns), making the boundary
// comparison exact rather than subject to float rounding.
TEST(ActuatorDynamicsTest, latency_delays_by_n_steps) {
  ActuatorDynamics a;
  YAML::Node config;
  config["command_latency"] = 0.5;
  a.Configure(config);

  const double dt = 0.1;
  for (int i = 0; i < 20; i++) {
    ros::Time t(i * dt);
    double value = (i + 1) * 10.0;  // distinct, non-zero per step
    a.Push(value, t);
    double out = a.Pull(t);
    if (i < 5) {
      // Command has not arrived yet: zero-order hold at the initial value (0).
      EXPECT_NEAR(0.0, out, 1e-6) << "step " << i;
    } else {
      // The command issued 5 steps ago (index i-5, value (i-4)*10) is active.
      EXPECT_NEAR((i - 4) * 10.0, out, 1e-6) << "step " << i;
    }
  }
}

// Zero-order hold: with no new command the last arrived value persists.
TEST(ActuatorDynamicsTest, latency_zero_order_hold) {
  ActuatorDynamics a;
  YAML::Node config;
  config["command_latency"] = 0.2;  // 2 steps at dt = 0.1
  a.Configure(config);

  const double dt = 0.1;
  // Push a single command at t=0, then push zeros thereafter is NOT done; we
  // simply stop pushing and keep pulling -- the value must hold.
  a.Push(7.0, ros::Time(0.0));
  EXPECT_NEAR(0.0, a.Pull(ros::Time(0.0)), 1e-6);
  EXPECT_NEAR(0.0, a.Pull(ros::Time(1 * dt)), 1e-6);
  // At t = 0.2 the command issued at t=0 has arrived and then holds.
  EXPECT_NEAR(7.0, a.Pull(ros::Time(2 * dt)), 1e-6);
  EXPECT_NEAR(7.0, a.Pull(ros::Time(10 * dt)), 1e-6);
}

// The latency buffer must not grow without bound; pushing far past the cap and
// then pulling in the distant future still returns the newest command.
TEST(ActuatorDynamicsTest, latency_buffer_is_bounded) {
  ActuatorDynamics a;
  YAML::Node config;
  config["command_latency"] = 1.0;
  a.Configure(config);

  const size_t n = ActuatorDynamics::kMaxBufferSize + 100;
  double last_value = 0.0;
  for (size_t i = 0; i < n; i++) {
    last_value = static_cast<double>(i);
    a.Push(last_value, ros::Time(i * 0.001));
  }
  // Pull well beyond every stamp: all in-flight commands have arrived, so the
  // newest pushed value is active. (A correct bound drops the OLDEST entries.)
  EXPECT_NEAR(last_value, a.Pull(ros::Time(1e6)), 1e-6);
}

// AccelerationCap = effort / inertia, disabled (+inf) when effort or inertia is
// non-positive.
TEST(ActuatorDynamicsTest, acceleration_cap) {
  ActuatorDynamics a;
  YAML::Node config;
  config["max_force"] = 25.0;
  a.Configure(config);

  EXPECT_NEAR(1.0, a.AccelerationCap(25.0), 1e-9);  // 25 N / 25 kg
  EXPECT_NEAR(2.5, a.AccelerationCap(10.0), 1e-9);
  // Non-positive inertia disables the cap (detached unit setups).
  EXPECT_TRUE(std::isinf(a.AccelerationCap(0.0)));
  EXPECT_TRUE(std::isinf(a.AccelerationCap(-1.0)));
}

// With no effort limit the acceleration cap is infinite (disabled).
TEST(ActuatorDynamicsTest, acceleration_cap_disabled) {
  ActuatorDynamics a;
  EXPECT_TRUE(std::isinf(a.AccelerationCap(25.0)));
  EXPECT_NEAR(0.0, a.ForceCap(), 1e-9);
}

// A fully default (empty) actuator is a pure no-op across every primitive.
TEST(ActuatorDynamicsTest, empty_configure_is_noop) {
  ActuatorDynamics a;
  a.Configure(YAML::Node());
  EXPECT_FALSE(a.Enabled());
  EXPECT_NEAR(3.3, a.ApplyDeadband(3.3), 1e-9);
  ros::Time t(2.0);
  a.Push(3.3, t);
  EXPECT_NEAR(3.3, a.Pull(t), 1e-9);  // pass-through, no delay
  EXPECT_TRUE(std::isinf(a.AccelerationCap(10.0)));
  EXPECT_NEAR(0.0, a.ForceCap(), 1e-9);
}

// --- Effort-scale overload (motor_degradation) ----------------------------
// effort_scale shrinks the effort cap: scale 1.0 reproduces the unscaled value,
// scale 0.5 halves the cap, scale 0.0 disables drive entirely.
TEST(ActuatorDynamicsTest, acceleration_cap_effort_scale) {
  ActuatorDynamics a;
  YAML::Node config;
  config["max_force"] = 25.0;
  a.Configure(config);

  // scale 1.0 == the legacy single-arg behaviour (25 N / 25 kg = 1.0).
  EXPECT_NEAR(1.0, a.AccelerationCap(25.0, 1.0), 1e-9);
  EXPECT_NEAR(a.AccelerationCap(25.0), a.AccelerationCap(25.0, 1.0), 1e-9);
  // scale 0.5 halves the achievable acceleration.
  EXPECT_NEAR(0.5, a.AccelerationCap(25.0, 0.5), 1e-9);
  // scale 0.0 on a CONFIGURED limit -> a dead motor: cap clamps accel to 0
  // (NOT the +inf "no limit configured" sentinel).
  EXPECT_NEAR(0.0, a.AccelerationCap(25.0, 0.0), 1e-9);
}

// The friction-path ForceCap scales the same way; scale 1.0 is unchanged.
TEST(ActuatorDynamicsTest, force_cap_effort_scale) {
  ActuatorDynamics a;
  YAML::Node config;
  config["max_force"] = 30.0;
  a.Configure(config);

  EXPECT_NEAR(30.0, a.ForceCap(), 1e-9);
  EXPECT_NEAR(30.0, a.ForceCap(1.0), 1e-9);
  EXPECT_NEAR(15.0, a.ForceCap(0.5), 1e-9);
  EXPECT_NEAR(0.0, a.ForceCap(0.0), 1e-9);
}

// An actuator with no effort limit cannot be degraded (nothing to shrink).
TEST(ActuatorDynamicsTest, effort_scale_noop_when_no_effort_limit) {
  ActuatorDynamics a;  // max_effort_ == 0
  EXPECT_TRUE(std::isinf(a.AccelerationCap(25.0, 0.5)));
  EXPECT_NEAR(0.0, a.ForceCap(0.5), 1e-9);
}

// --- Extra-latency Pull (controller_latency) ------------------------------
// extra_latency == 0 reproduces the byte-for-byte pass-through (no delay).
TEST(ActuatorDynamicsTest, extra_latency_zero_is_passthrough) {
  ActuatorDynamics a;  // command_latency_ == 0
  ros::Time t(0.0);
  a.Push(1.0, t);
  EXPECT_NEAR(1.0, a.Pull(t, 0.0), 1e-9);
  t = ros::Time(0.1);
  a.Push(2.0, t);
  EXPECT_NEAR(2.0, a.Pull(t, 0.0), 1e-9);
}

// A pure extra_latency (no configured command_latency) delays the command by
// exactly that amount: pushed at t, emerges at t + extra.
TEST(ActuatorDynamicsTest, extra_latency_delays_with_no_base_latency) {
  ActuatorDynamics a;  // command_latency_ == 0
  const double dt = 0.1;
  const double extra = 0.5;  // 5-step delay
  for (int i = 0; i < 20; i++) {
    ros::Time t(i * dt);
    double value = (i + 1) * 10.0;
    a.Push(value, t);
    double out = a.Pull(t, extra);
    if (i < 5) {
      EXPECT_NEAR(0.0, out, 1e-6) << "step " << i;
    } else {
      EXPECT_NEAR((i - 4) * 10.0, out, 1e-6) << "step " << i;
    }
  }
}

// extra_latency stacks on top of the configured command_latency: total delay is
// base + extra. base 0.2 + extra 0.3 = 0.5 s = 5 steps at dt = 0.1.
TEST(ActuatorDynamicsTest, extra_latency_adds_to_base_latency) {
  ActuatorDynamics a;
  YAML::Node config;
  config["command_latency"] = 0.2;
  a.Configure(config);

  const double dt = 0.1;
  const double extra = 0.3;
  for (int i = 0; i < 20; i++) {
    ros::Time t(i * dt);
    double value = (i + 1) * 10.0;
    a.Push(value, t);
    double out = a.Pull(t, extra);
    if (i < 5) {
      EXPECT_NEAR(0.0, out, 1e-6) << "step " << i;
    } else {
      EXPECT_NEAR((i - 4) * 10.0, out, 1e-6) << "step " << i;
    }
  }
}

// The buffer scan is non-destructive, so latency may INCREASE mid-run without
// having discarded an older command it now needs. Push a distinct command every
// step; first read with a small latency, then read a later step with a larger
// latency and confirm it correctly reaches the older command.
TEST(ActuatorDynamicsTest, increasing_latency_reaches_older_command) {
  ActuatorDynamics a;
  YAML::Node config;
  config["command_latency"] = 0.1;  // base 1-step delay at dt = 0.1
  a.Configure(config);

  const double dt = 0.1;
  // Push commands for 0..9 with value == step index * 10.
  for (int i = 0; i < 10; i++) {
    a.Push(i * 10.0, ros::Time(i * dt));
  }

  // At t = 0.9 with the base latency (0.1) the active command is step 8 (= 80).
  EXPECT_NEAR(80.0, a.Pull(ros::Time(9 * dt), 0.0), 1e-6);
  // At the same time with a larger total latency (0.1 + 0.4 = 0.5) the active
  // command is the OLDER step 4 (= 40); a destructive drain would have lost it.
  EXPECT_NEAR(40.0, a.Pull(ros::Time(9 * dt), 0.4), 1e-6);
  // And the small-latency read still works afterwards (non-destructive).
  EXPECT_NEAR(80.0, a.Pull(ros::Time(9 * dt), 0.0), 1e-6);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
