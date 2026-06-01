/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name  variable_payload_test.cpp
 * @brief test the VariablePayload plugin
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

#include <flatland_plugins/variable_payload.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <std_msgs/Float64.h>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class VariablePayloadTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }

  static bool fltcmp(double n1, double n2, double epsilon = 1e-4) {
    return std::fabs(n1 - n2) < epsilon;
  }
};

/**
 * The plugin should apply the full-tank mass and the corresponding (forward)
 * center of gravity as soon as the world is loaded.
 */
TEST_F(VariablePayloadTest, initial_mass_and_cog) {
  world_yaml =
      this_file_dir / fs::path("variable_payload_tests/water_tank.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.5);
  World* w = World::MakeWorld(world_yaml.string());

  VariablePayload* plugin = dynamic_cast<VariablePayload*>(
      w->plugin_manager_.model_plugins_[0].get());
  ASSERT_NE(plugin, nullptr);
  b2Body* body = plugin->body_->physics_body_;

  // base 300 kg + full payload 200 kg = 500 kg.
  EXPECT_TRUE(fltcmp(body->GetMass(), 500.0));
  // CoG = (300*0 + 200*0.3) / 500 = 0.12 m forward, on the x axis.
  EXPECT_TRUE(fltcmp(body->GetLocalCenter().x, 0.12));
  EXPECT_TRUE(fltcmp(body->GetLocalCenter().y, 0.0));

  delete w;
}

/**
 * As the tank drains on the simulation clock the mass should drop and the
 * center of gravity should migrate back toward the chassis center.
 */
TEST_F(VariablePayloadTest, mass_and_cog_drain_over_time) {
  world_yaml =
      this_file_dir / fs::path("variable_payload_tests/water_tank.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.5);
  World* w = World::MakeWorld(world_yaml.string());

  VariablePayload* plugin = dynamic_cast<VariablePayload*>(
      w->plugin_manager_.model_plugins_[0].get());
  ASSERT_NE(plugin, nullptr);
  b2Body* body = plugin->body_->physics_body_;

  double mass0 = body->GetMass();
  double cogx0 = body->GetLocalCenter().x;

  // One 0.5 s step drains 0.05 * 0.5 = 2.5% -> fill 0.975, payload 195 kg.
  w->Update(timekeeper);
  EXPECT_TRUE(fltcmp(body->GetMass(), 495.0));
  EXPECT_TRUE(fltcmp(body->GetLocalCenter().x, 195.0 * 0.3 / 495.0));

  // Mass and forward CoG offset both shrink as the tank empties.
  EXPECT_LT(body->GetMass(), mass0);
  EXPECT_LT(body->GetLocalCenter().x, cogx0);

  // Drain all the way: after >= 20 s of sim time the tank is empty and the body
  // is back to its 300 kg chassis with the CoG at the chassis center.
  for (int i = 0; i < 60; i++) {
    w->Update(timekeeper);
  }
  EXPECT_TRUE(fltcmp(body->GetMass(), 300.0));
  EXPECT_TRUE(fltcmp(body->GetLocalCenter().x, 0.0));

  delete w;
}

/**
 * An external fill command should override the fill level; the next step then
 * reflects the commanded mass and CoG.
 */
TEST_F(VariablePayloadTest, fill_command_overrides_level) {
  world_yaml =
      this_file_dir / fs::path("variable_payload_tests/water_tank.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.5);
  World* w = World::MakeWorld(world_yaml.string());

  VariablePayload* plugin = dynamic_cast<VariablePayload*>(
      w->plugin_manager_.model_plugins_[0].get());
  ASSERT_NE(plugin, nullptr);
  b2Body* body = plugin->body_->physics_body_;

  // Command a half-full tank.
  std_msgs::Float64 cmd;
  cmd.data = 0.5;
  plugin->FillCallback(cmd);
  w->Update(timekeeper);

  // base 300 + 0.5 * 200 = 400 kg; CoG = 100*0.3 / 400 = 0.075 m.
  // (the constant drain also runs this step, so allow a small tolerance)
  EXPECT_NEAR(body->GetMass(), 400.0, 6.0);
  EXPECT_NEAR(body->GetLocalCenter().x, 100.0 * 0.3 / 400.0, 5e-3);

  // Commands are clamped to [0, 1]. An over-unity command saturates at a full
  // tank (the next step drains it only slightly, so it stays just under 500).
  cmd.data = 5.0;
  plugin->FillCallback(cmd);
  w->Update(timekeeper);
  EXPECT_LE(body->GetMass(), 500.0 + 1e-3);
  EXPECT_GT(body->GetMass(), 490.0);

  // A negative command saturates at an empty tank: 300 kg chassis, and the
  // drain cannot push the fill below zero.
  cmd.data = -3.0;
  plugin->FillCallback(cmd);
  w->Update(timekeeper);
  EXPECT_TRUE(fltcmp(body->GetMass(), 300.0));

  delete w;
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "variable_payload_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
