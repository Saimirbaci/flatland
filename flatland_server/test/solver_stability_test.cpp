/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   solver_stability_test.cpp
 * @brief  Contact-solver stability tests at AGV mass/speed ranges
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
#include <flatland_server/model.h>
#include <flatland_server/model_body.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <string>

namespace fs = boost::filesystem;
using namespace flatland_server;

class SolverStabilityTest : public ::testing::Test {
 protected:
  boost::filesystem::path this_file_dir;
  World *w;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
    w = nullptr;
  }

  void TearDown() override {
    if (w != nullptr) {
      delete w;
    }
  }

  // Look up a model by name and return the Box2D body of its first body.
  b2Body *GetModelBody(const std::string &name) {
    for (Model *m : w->models_) {
      if (m->GetName() == name) {
        EXPECT_FALSE(m->bodies_.empty());
        return m->bodies_[0]->physics_body_;
      }
    }
    ADD_FAILURE() << "model \"" << name << "\" not found in world";
    return nullptr;
  }

  // Advance the world by `steps` fixed steps of `step_size` seconds, holding
  // the AGV body's linear velocity constant each step so it keeps driving into
  // the wall instead of just coasting (mimics a commanded drive without a
  // controller/cmd_vel publisher).
  void Drive(b2Body *agv, const b2Vec2 &velocity, double step_size, int steps) {
    Timekeeper timekeeper;
    timekeeper.SetMaxStepSize(step_size);
    for (int i = 0; i < steps; i++) {
      agv->SetLinearVelocity(velocity);
      w->Update(timekeeper);
    }
  }
};

/**
 * A ~1500 kg AGV driven into a static wall must settle against the wall face
 * with only a small, bounded penetration -- not sink through it. The wall body
 * sits at x=3 with a 0.2 m-thick slab (face at x=2.9); the AGV is a circle of
 * radius 0.5, so its centre should rest near x=2.4 and never cross the face.
 */
TEST_F(SolverStabilityTest, heavy_agv_bounded_penetration) {
  fs::path world_yaml =
      this_file_dir / fs::path("agv_stress_tests/penetration_world.yaml");
  w = World::MakeWorld(world_yaml.string());

  b2Body *agv = GetModelBody("agv");
  ASSERT_NE(agv, nullptr);

  // Drive at 2 m/s toward the wall for 6 simulated seconds (200 Hz).
  Drive(agv, b2Vec2(2.0, 0.0), 0.005, 1200);

  const double wall_face_x = 2.9;       // inner face of the static wall slab
  const double contact_center_x = 2.4;  // wall_face - agv radius
  double final_x = agv->GetPosition().x;

  // It actually reached the wall (travelled most of the way).
  EXPECT_GT(final_x, 2.0) << "AGV did not reach the wall";
  // It never crossed the wall face -> no pass-through.
  EXPECT_LT(final_x, wall_face_x) << "AGV centre crossed the wall face";
  // Penetration past the resting contact point is small and bounded.
  EXPECT_LT(final_x - contact_center_x, 0.1)
      << "AGV penetrated the wall by more than 0.1 m";
}

/**
 * A small, fast (~50 kg) AGV launched at a thin wall must be stopped on the
 * near side by continuous collision detection. At 30 m/s and a 0.005 s step the
 * body moves 0.15 m/step -- comparable to the 0.2 m wall thickness -- so
 * without CCD it would tunnel straight through. The world enables
 * continuous_physics, so the swept collision is caught.
 */
TEST_F(SolverStabilityTest, fast_agv_no_tunneling) {
  fs::path world_yaml =
      this_file_dir / fs::path("agv_stress_tests/tunneling_world.yaml");
  w = World::MakeWorld(world_yaml.string());
  ASSERT_TRUE(w->continuous_physics_);

  b2Body *agv = GetModelBody("agv");
  ASSERT_NE(agv, nullptr);

  // Launch at 30 m/s for 2 simulated seconds.
  Drive(agv, b2Vec2(30.0, 0.0), 0.005, 400);

  const double wall_face_x = 2.9;
  double final_x = agv->GetPosition().x;

  // The body started at x=0 and was moving fast; it must have been stopped on
  // the near side of the wall rather than tunnelling past it.
  EXPECT_LT(final_x, wall_face_x)
      << "fast AGV tunnelled through the wall (final x=" << final_x << ")";
  EXPECT_GT(final_x, 1.0) << "fast AGV did not advance toward the wall";
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  ros::init(argc, argv, "solver_stability_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
