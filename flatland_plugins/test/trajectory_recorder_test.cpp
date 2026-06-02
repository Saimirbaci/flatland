/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name  trajectory_recorder_test.cpp
 * @brief test the TrajectoryRecorder plugin
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

#include <flatland_plugins/trajectory_recorder.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <nav_msgs/Path.h>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class TrajectoryRecorderTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }
};

/**
 * The actual path accumulates one pose per step and is capped at max_length
 * (5 in the test model), with the first recorded pose matching the spawn pose.
 */
TEST_F(TrajectoryRecorderTest, actual_path_grows_then_caps) {
  fs::path world_yaml =
      this_file_dir /
      fs::path("trajectory_recorder_tests/trajectory.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.1);
  World* w = World::MakeWorld(world_yaml.string());

  TrajectoryRecorder* plugin = dynamic_cast<TrajectoryRecorder*>(
      w->plugin_manager_.model_plugins_[0].get());
  ASSERT_NE(plugin, nullptr);

  // Three steps -> three poses (below the cap of 5).
  for (int i = 0; i < 3; i++) {
    w->Update(timekeeper);
  }
  EXPECT_EQ(3u, plugin->actual_path_.poses.size());
  // The model was spawned at (1, 2); the recorded pose reflects the body pose.
  EXPECT_NEAR(1.0, plugin->actual_path_.poses.front().pose.position.x, 1e-3);
  EXPECT_NEAR(2.0, plugin->actual_path_.poses.front().pose.position.y, 1e-3);
  EXPECT_EQ("map", plugin->actual_path_.header.frame_id);

  // Many more steps -> the path is capped at max_length, oldest dropped.
  for (int i = 0; i < 20; i++) {
    w->Update(timekeeper);
  }
  EXPECT_EQ(5u, plugin->actual_path_.poses.size());

  delete w;
}

/**
 * The actual path is published on the namespaced topic and is observable by a
 * subscriber after stepping.
 */
TEST_F(TrajectoryRecorderTest, publishes_actual_path) {
  fs::path world_yaml =
      this_file_dir /
      fs::path("trajectory_recorder_tests/trajectory.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.1);
  World* w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  nav_msgs::Path::ConstPtr received;
  ros::Subscriber sub = nh.subscribe<nav_msgs::Path>(
      "trajectory/actual", 1,
      [&](const nav_msgs::Path::ConstPtr& msg) { received = msg; });

  ros::Time start = ros::Time::now();
  while (!received && (ros::Time::now() - start).toSec() < 5.0) {
    w->Update(timekeeper);
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }

  ASSERT_TRUE(received != nullptr) << "no actual Path published";
  EXPECT_EQ("map", received->header.frame_id);
  EXPECT_GT(received->poses.size(), 0u);

  delete w;
}

/**
 * An externally supplied planned path is re-stamped into the world frame and
 * republished on the planned topic for side-by-side comparison.
 */
TEST_F(TrajectoryRecorderTest, republishes_planned_path) {
  fs::path world_yaml =
      this_file_dir /
      fs::path("trajectory_recorder_tests/trajectory.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.1);
  World* w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Publisher planned_in = nh.advertise<nav_msgs::Path>("planned_in", 1);

  nav_msgs::Path::ConstPtr received;
  ros::Subscriber sub = nh.subscribe<nav_msgs::Path>(
      "trajectory/planned", 1,
      [&](const nav_msgs::Path::ConstPtr& msg) { received = msg; });

  // Wait for the plugin's subscriber to connect before publishing.
  ros::Time connect_start = ros::Time::now();
  while (planned_in.getNumSubscribers() == 0 &&
         (ros::Time::now() - connect_start).toSec() < 5.0) {
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }

  nav_msgs::Path plan;
  plan.header.frame_id = "odom";  // deliberately a different frame
  geometry_msgs::PoseStamped p;
  p.pose.position.x = 7.0;
  p.pose.position.y = 8.0;
  plan.poses.push_back(p);
  planned_in.publish(plan);

  ros::Time start = ros::Time::now();
  while (!received && (ros::Time::now() - start).toSec() < 5.0) {
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }

  ASSERT_TRUE(received != nullptr) << "planned path not republished";
  // Re-stamped into the world frame, geometry preserved.
  EXPECT_EQ("map", received->header.frame_id);
  ASSERT_EQ(1u, received->poses.size());
  EXPECT_NEAR(7.0, received->poses[0].pose.position.x, 1e-6);
  EXPECT_NEAR(8.0, received->poses[0].pose.position.y, 1e-6);

  delete w;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_recorder_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
