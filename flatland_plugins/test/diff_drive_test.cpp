/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  diff_drive_test.cpp
 * @brief test diff drive plugin
 * @author Chunshang Li
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
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
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class DiffDrivePluginTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
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
};

TEST_F(DiffDrivePluginTest, load_test) {
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
      "flatland_server", "flatland_server::ModelPlugin");

  try {
    boost::shared_ptr<flatland_server::ModelPlugin> plugin =
        loader.createInstance("flatland_plugins::DiffDrive");
  } catch (pluginlib::PluginlibException& e) {
    FAIL() << "Failed to load diff drive Drive plugin. " << e.what();
  }
}

// Force-based (friction) traction: commanding forward velocity should drive the
// robot forward through the wheel-ground friction model. The velocity ramps up
// under the bounded traction force (force-limited) rather than jumping to the
// command in a single step the way the ideal kinematic path does.
TEST_F(DiffDrivePluginTest, friction_drive_forward) {
  world_yaml = this_file_dir / fs::path("diff_drive_tests/world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_1;
  DiffDrivePluginTest* obj = dynamic_cast<DiffDrivePluginTest*>(this);
  sub_1 = nh.subscribe("odometry/ground_truth", 1, &DiffDrivePluginTest::GroundTruthSubscriberCB, obj);

  DiffDrive* dd =
      dynamic_cast<DiffDrive*>(w->plugin_manager_.model_plugins_[0].get());
  ASSERT_NE(nullptr, dd);
  EXPECT_TRUE(dd->use_friction_drive_);

  // Let things settle at rest
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }
  EXPECT_NEAR(0, odom.twist.twist.linear.x, 0.01);
  EXPECT_NEAR(0, odom.pose.pose.position.x, 0.01);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.5;  // m/s
  cmd_vel.angular.z = 0.0;
  dd->TwistCallback(cmd_vel);

  // Traction is force-limited: after a couple of steps the body is moving but
  // has not reached the command (the kinematic path would jump there at once).
  for (unsigned int i = 0; i < 2; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }
  EXPECT_GT(odom.twist.twist.linear.x, 0.0);
  EXPECT_LT(odom.twist.twist.linear.x, 0.5);

  // With enough time the high-grip wheels reach the commanded velocity
  for (unsigned int i = 0; i < 200; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }
  EXPECT_NEAR(0.5, odom.twist.twist.linear.x, 0.05);  // tracks command
  EXPECT_GT(odom.pose.pose.position.x, 0.2);          // drove forward
  EXPECT_NEAR(0.0, odom.pose.pose.position.y, 0.05);  // bounded side-slip
}

// Friction traction must still turn: a pure angular command produces body yaw
// via the differential of the two wheels' longitudinal traction forces.
TEST_F(DiffDrivePluginTest, friction_drive_rotate) {
  world_yaml = this_file_dir / fs::path("diff_drive_tests/world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_1;
  DiffDrivePluginTest* obj = dynamic_cast<DiffDrivePluginTest*>(this);
  sub_1 = nh.subscribe("odometry/ground_truth", 1, &DiffDrivePluginTest::GroundTruthSubscriberCB, obj);

  DiffDrive* dd =
      dynamic_cast<DiffDrive*>(w->plugin_manager_.model_plugins_[0].get());
  ASSERT_NE(nullptr, dd);

  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.5;  // rad/s
  dd->TwistCallback(cmd_vel);
  for (unsigned int i = 0; i < 200; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }

  // rotating in place: yawing, with the body roughly holding position
  EXPECT_GT(odom.twist.twist.angular.z, 0.1);
  EXPECT_NEAR(0.0, odom.pose.pose.position.x, 0.1);
  EXPECT_NEAR(0.0, odom.pose.pose.position.y, 0.1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "diff_drive_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
