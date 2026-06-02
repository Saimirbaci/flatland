/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name  occupancy_publisher_test.cpp
 * @brief test the occupancy-grid diagnostic overlay publisher
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

#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;
using namespace flatland_server;

class OccupancyPublisherTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }
};

/**
 * Every image-based layer should build an occupancy grid whose dimensions,
 * resolution and origin match the source map (the shared 5x5, 0.05 m/px map
 * with origin (0.05, -0.05)). The cells are a binary occupied/free view.
 */
TEST_F(OccupancyPublisherTest, layer_builds_occupancy_grid) {
  fs::path world_yaml =
      this_file_dir / fs::path("diagnostics_tests/occupancy.world.yaml");
  World* w = World::MakeWorld(world_yaml.string());

  ASSERT_EQ(1u, w->layers_.size());
  Layer* layer = w->layers_.front();
  ASSERT_TRUE(layer->HasOccupancyGrid());

  const nav_msgs::OccupancyGrid& grid = layer->GetOccupancyGrid();
  EXPECT_EQ("map", grid.header.frame_id);
  EXPECT_EQ(5u, grid.info.width);
  EXPECT_EQ(5u, grid.info.height);
  EXPECT_NEAR(0.05, grid.info.resolution, 1e-6);
  EXPECT_NEAR(0.05, grid.info.origin.position.x, 1e-6);
  EXPECT_NEAR(-0.05, grid.info.origin.position.y, 1e-6);

  ASSERT_EQ(25u, grid.data.size());
  for (int8_t cell : grid.data) {
    EXPECT_TRUE(cell == 0 || cell == 100) << "unexpected cell value " << cell;
  }

  delete w;
}

/**
 * World::PublishDiagnostics latches the grid on the stable, name-agnostic topic
 * so a late subscriber (rviz / the panel) still receives it.
 */
TEST_F(OccupancyPublisherTest, publishes_latched_occupancy) {
  fs::path world_yaml =
      this_file_dir / fs::path("diagnostics_tests/occupancy.world.yaml");
  World* w = World::MakeWorld(world_yaml.string());
  w->PublishDiagnostics();

  // Subscribe after publishing: the latched message must still arrive.
  ros::NodeHandle nh;
  nav_msgs::OccupancyGrid::ConstPtr received;
  ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>(
      "/flatland_server/occupancy", 1,
      [&](const nav_msgs::OccupancyGrid::ConstPtr& msg) { received = msg; });

  ros::Time start = ros::Time::now();
  while (!received && (ros::Time::now() - start).toSec() < 5.0) {
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }

  ASSERT_TRUE(received != nullptr) << "no latched occupancy grid received";
  EXPECT_EQ(5u, received->info.width);
  EXPECT_EQ(5u, received->info.height);

  delete w;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "occupancy_publisher_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
