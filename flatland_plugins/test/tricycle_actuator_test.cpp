/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name  tricycle_actuator_test.cpp
 * @brief End-to-end test for the drive-actuator deadband wired into the
 *        TricycleDrive plugin
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

#include <flatland_plugins/tricycle_drive.h>
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

class TricycleActuatorTest : public ::testing::Test {
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

  void Step(Timekeeper& timekeeper, unsigned int n) {
    for (unsigned int i = 0; i < n; i++) {
      w->Update(timekeeper);
      ros::spinOnce();
    }
  }
};

// Drive-actuator deadband: a sub-threshold forward command (0.2 < 0.3 m/s) is
// zeroed (the robot holds its start pose); a command above the deadband drives.
TEST_F(TricycleActuatorTest, drive_deadband_blocks_subthreshold) {
  boost::filesystem::path world_yaml =
      this_file_dir / fs::path("tricycle_actuator_tests/world_deadband.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe("odometry/ground_truth", 1,
                   &TricycleActuatorTest::GroundTruthSubscriberCB, this);

  TricycleDrive* td =
      dynamic_cast<TricycleDrive*>(w->plugin_manager_.model_plugins_[0].get());
  ASSERT_NE(nullptr, td);

  // Settle and confirm the start pose (model spawned at x = 12).
  Step(timekeeper, 100);
  EXPECT_NEAR(12.0, odom.pose.pose.position.x, 0.05);

  // Sub-threshold command: no motion.
  geometry_msgs::Twist cmd;
  cmd.angular.z = 0.0;
  cmd.linear.x = 0.2;  // below the 0.3 m/s deadband
  td->TwistCallback(cmd);
  Step(timekeeper, 100);
  EXPECT_NEAR(0.0, odom.twist.twist.linear.x, 0.02);
  EXPECT_NEAR(12.0, odom.pose.pose.position.x, 0.05);

  // Above the deadband the command drives the robot forward.
  cmd.linear.x = 0.5;
  td->TwistCallback(cmd);
  Step(timekeeper, 100);
  EXPECT_GT(odom.twist.twist.linear.x, 0.1);
  EXPECT_GT(odom.pose.pose.position.x, 12.1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tricycle_actuator_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
