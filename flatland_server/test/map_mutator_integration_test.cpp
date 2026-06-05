/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	map_mutator_integration_test.cpp
 * @brief	Loads a world with a `mutation:` block and checks it loads + is
 *          reproducible, and that the sealed manifest is written
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

#include <flatland_server/layer.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <string>

namespace fs = boost::filesystem;
using namespace flatland_server;

class MapMutatorIntegrationTest : public ::testing::Test {
 protected:
  fs::path world_yaml;

  void SetUp() override {
    world_yaml = fs::path(__FILE__).parent_path() /
                 fs::path("map_mutator_tests/mutation_world.yaml");
  }
};

/**
 * A world whose layer carries an enabled `mutation:` block loads end-to-end:
 * the mutated bitmap is still a valid layer with a published-able occupancy
 * grid, and the sealed manifest sidecar is written out-of-band.
 */
TEST_F(MapMutatorIntegrationTest, loads_mutated_world_and_writes_manifest) {
  const std::string manifest_path = "/tmp/flatland_map_mutation_test.json";
  std::remove(manifest_path.c_str());

  World* w = World::MakeWorld(world_yaml.string(), 42);
  ASSERT_NE(nullptr, w);
  ASSERT_FALSE(w->layers_.empty());

  Layer* layer = w->layers_.front();
  ASSERT_NE(nullptr, layer);
  EXPECT_TRUE(layer->HasOccupancyGrid());
  EXPECT_GT(layer->GetOccupancyGrid().data.size(), 0u);

  // The sealed manifest must exist and be non-empty.
  std::ifstream manifest(manifest_path);
  ASSERT_TRUE(manifest.good());
  manifest.seekg(0, std::ios::end);
  EXPECT_GT(manifest.tellg(), 0);

  delete w;
}

/**
 * The same global seed reproduces the same mutated occupancy grid, so a run is
 * deterministic end-to-end (eval can replay a novel map exactly).
 */
TEST_F(MapMutatorIntegrationTest, same_seed_reproduces_occupancy_grid) {
  World* w1 = World::MakeWorld(world_yaml.string(), 7);
  ASSERT_FALSE(w1->layers_.empty());
  const std::vector<int8_t> grid1 =
      w1->layers_.front()->GetOccupancyGrid().data;
  delete w1;

  World* w2 = World::MakeWorld(world_yaml.string(), 7);
  ASSERT_FALSE(w2->layers_.empty());
  const std::vector<int8_t> grid2 =
      w2->layers_.front()->GetOccupancyGrid().data;
  delete w2;

  EXPECT_EQ(grid1, grid2);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_mutator_integration_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
