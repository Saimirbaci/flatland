/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	determinism_test.cpp
 * @brief	Integration test: same seed + same world ⇒ reproducible run
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

#include <Box2D/Box2D.h>
#include <flatland_server/body.h>
#include <flatland_server/model.h>
#include <flatland_server/model_body.h>
#include <flatland_server/random.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>

using namespace flatland_server;

class DeterminismTest : public ::testing::Test {
 protected:
  boost::filesystem::path world_yaml;

  void SetUp() override {
    world_yaml = boost::filesystem::path(__FILE__).parent_path() /
                 "conestogo_office_test" / "world.yaml";
  }
};

// Build the world with the given seed, step it `steps` fixed steps, and return a
// flat vector of every model body's (x, y, angle). Because physics is a
// single-threaded fixed-step integration and no wall-clock leaks into the sim,
// two builds with the same seed must produce byte-identical trajectories.
static std::vector<double> RunAndCapture(const std::string &world_path, int seed,
                                         int steps) {
  World *w = World::MakeWorld(world_path, seed);
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  for (int i = 0; i < steps; i++) {
    w->Update(tk);
  }

  std::vector<double> poses;
  for (Model *m : w->models_) {
    for (ModelBody *b : m->bodies_) {
      b2Body *pb = b->physics_body_;
      poses.push_back(pb->GetPosition().x);
      poses.push_back(pb->GetPosition().y);
      poses.push_back(pb->GetAngle());
    }
  }
  delete w;
  return poses;
}

// Two runs with the same seed produce identical model trajectories.
TEST_F(DeterminismTest, same_seed_same_poses) {
  std::vector<double> a = RunAndCapture(world_yaml.string(), 42, 50);
  std::vector<double> b = RunAndCapture(world_yaml.string(), 42, 50);
  ASSERT_FALSE(a.empty());
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); i++) {
    EXPECT_DOUBLE_EQ(a[i], b[i]);
  }
}

// Building a world applies the requested seed to the RNG authority.
TEST_F(DeterminismTest, seed_is_applied) {
  World *w = World::MakeWorld(world_yaml.string(), 99);
  EXPECT_TRUE(RngManager::Get().IsSeeded());
  EXPECT_EQ(RngManager::Get().GetSeed(), 99u);
  delete w;
}

// The integrated build seeds the RNG authority, and the seed actually changes
// the derived noise stream that plugins consume.
TEST_F(DeterminismTest, seed_changes_noise_sequence) {
  World *w1 = World::MakeWorld(world_yaml.string(), 1);
  std::default_random_engine e1 =
      RngManager::Get().DeriveEngine("robot/laser");
  delete w1;

  World *w2 = World::MakeWorld(world_yaml.string(), 2);
  std::default_random_engine e2 =
      RngManager::Get().DeriveEngine("robot/laser");
  delete w2;

  bool differ = false;
  for (int i = 0; i < 50; i++) {
    if (e1() != e2()) {
      differ = true;
      break;
    }
  }
  EXPECT_TRUE(differ);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "determinism_test");
  return RUN_ALL_TESTS();
}
