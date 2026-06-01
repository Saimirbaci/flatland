/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name  diff_drive_actuator_test.cpp
 * @brief End-to-end tests for actuator dynamics (latency, deadband, force cap)
 *        wired into the DiffDrive plugin
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

#include <flatland_plugins/diff_drive.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class DiffDriveActuatorTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  nav_msgs::Odometry odom;
  World* w;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
    w = nullptr;
  }

  void TearDown() override {
    if (w != nullptr) {
      delete w;
    }
  }

  void GroundTruthSubscriberCB(const nav_msgs::Odometry& msg) { odom = msg; }

  // Load a world under diff_drive_actuator_tests/ and return its DiffDrive
  // plugin, subscribing to its ground-truth odometry.
  DiffDrive* LoadWorld(const std::string& world_file, ros::NodeHandle& nh,
                       ros::Subscriber& sub) {
    boost::filesystem::path world_yaml =
        this_file_dir / fs::path("diff_drive_actuator_tests") /
        fs::path(world_file);
    w = World::MakeWorld(world_yaml.string());
    sub = nh.subscribe("odometry/ground_truth", 1,
                       &DiffDriveActuatorTest::GroundTruthSubscriberCB, this);
    DiffDrive* dd =
        dynamic_cast<DiffDrive*>(w->plugin_manager_.model_plugins_[0].get());
    return dd;
  }

  void Step(Timekeeper& timekeeper, unsigned int n) {
    for (unsigned int i = 0; i < n; i++) {
      w->Update(timekeeper);
      ros::spinOnce();
    }
  }
};

// Baseline: no actuator subnode => legacy kinematic path. A forward command is
// tracked almost immediately (no latency, no deadband, no acceleration cap).
TEST_F(DiffDriveActuatorTest, baseline_no_actuator_tracks_immediately) {
  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub;
  DiffDrive* dd = LoadWorld("world_baseline.yaml", nh, sub);
  ASSERT_NE(nullptr, dd);
  EXPECT_FALSE(dd->use_friction_drive_);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;
  dd->TwistCallback(cmd);

  // Kinematic path applies the command directly: within a couple of steps the
  // body is already at the commanded velocity.
  Step(timekeeper, 3);
  EXPECT_NEAR(0.5, odom.twist.twist.linear.x, 0.02);
}

// Command latency: a 0.2 s deadtime means the body does not move for the first
// ~0.15 s, then tracks the command once the delayed command arrives.
TEST_F(DiffDriveActuatorTest, command_latency_delays_motion) {
  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub;
  DiffDrive* dd = LoadWorld("world_latency.yaml", nh, sub);
  ASSERT_NE(nullptr, dd);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;
  dd->TwistCallback(cmd);

  // Within the latency window (15 steps = 0.15 s < 0.2 s) there is no motion.
  Step(timekeeper, 15);
  EXPECT_NEAR(0.0, odom.twist.twist.linear.x, 0.02);
  EXPECT_NEAR(0.0, odom.pose.pose.position.x, 0.01);

  // Past the latency (well beyond 0.2 s) the command takes effect.
  Step(timekeeper, 40);
  EXPECT_NEAR(0.5, odom.twist.twist.linear.x, 0.05);
  EXPECT_GT(odom.pose.pose.position.x, 0.05);
}

// Deadband: a sub-threshold command (0.2 < 0.3 m/s) produces no motion; a
// command above threshold drives normally.
TEST_F(DiffDriveActuatorTest, deadband_blocks_subthreshold) {
  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub;
  DiffDrive* dd = LoadWorld("world_deadband.yaml", nh, sub);
  ASSERT_NE(nullptr, dd);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.2;  // below the 0.3 m/s deadband
  dd->TwistCallback(cmd);
  Step(timekeeper, 50);
  EXPECT_NEAR(0.0, odom.twist.twist.linear.x, 0.01);
  EXPECT_NEAR(0.0, odom.pose.pose.position.x, 0.01);

  // Above the deadband the command passes through and drives the body.
  cmd.linear.x = 0.5;
  dd->TwistCallback(cmd);
  Step(timekeeper, 50);
  EXPECT_NEAR(0.5, odom.twist.twist.linear.x, 0.05);
  EXPECT_GT(odom.pose.pose.position.x, 0.05);
}

// Force cap (kinematic mode): a large velocity step is bounded to a_max = F/m =
// 25 N / 25 kg = 1.0 m/s^2, so after N steps the velocity is ~N*dt*a_max rather
// than the command, and it never exceeds the command.
TEST_F(DiffDriveActuatorTest, torque_cap_bounds_acceleration_kinematic) {
  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub;
  DiffDrive* dd = LoadWorld("world_torque.yaml", nh, sub);
  ASSERT_NE(nullptr, dd);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 1.0;  // commanded step far exceeding what F/m allows in 1 step
  dd->TwistCallback(cmd);

  // After 10 steps (0.1 s) the bounded velocity is ~a_max * t = 1.0 * 0.1 =
  // 0.1, well short of the 1.0 command.
  Step(timekeeper, 10);
  EXPECT_NEAR(0.10, odom.twist.twist.linear.x, 0.03);
  EXPECT_LT(odom.twist.twist.linear.x, 0.5);

  // Given enough time the velocity ramps up to the command.
  Step(timekeeper, 150);
  EXPECT_NEAR(1.0, odom.twist.twist.linear.x, 0.05);
}

// Force cap (friction mode): the same bound holds when the force-based traction
// path is active. The wheels have ample grip, so the 25 N motor force cap (not
// grip) is what bounds the acceleration to ~1.0 m/s^2.
TEST_F(DiffDriveActuatorTest, torque_cap_bounds_acceleration_friction) {
  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub;
  DiffDrive* dd = LoadWorld("world_torque_friction.yaml", nh, sub);
  ASSERT_NE(nullptr, dd);
  EXPECT_TRUE(dd->use_friction_drive_);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 1.0;
  dd->TwistCallback(cmd);

  // Acceleration is force-limited: after 10 steps the body is moving but the
  // velocity is bounded well below the command (a_max ~ 1.0 m/s^2).
  Step(timekeeper, 10);
  EXPECT_GT(odom.twist.twist.linear.x, 0.0);
  EXPECT_LT(odom.twist.twist.linear.x, 0.3);

  // With enough time it approaches the command.
  Step(timekeeper, 250);
  EXPECT_NEAR(1.0, odom.twist.twist.linear.x, 0.1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "diff_drive_actuator_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
