/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   surface_friction_test.cpp
 * @brief  End-to-end test that the world surface_friction field scales drive
 *         traction by region (smooth, position-dependent traction loss)
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
#include <cmath>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class SurfaceFrictionTest : public ::testing::Test {
 public:
  fs::path world_yaml;
  nav_msgs::Odometry odom;
  World* w;

  void SetUp() override { w = nullptr; }
  void TearDown() override {
    if (w != nullptr) {
      delete w;
    }
  }

  void GroundTruthCB(const nav_msgs::Odometry& msg) { odom = msg; }
};

// Drive a friction-mode diff-drive robot in a straight line across a wet patch
// defined by the world's surface_friction field. The robot accelerates briskly
// on dry ground, loses traction (much smaller achievable acceleration) over the
// wet patch, and regains it on the dry recovery zone, all without any
// discontinuous pose jump at the region boundaries.
TEST_F(SurfaceFrictionTest, traction_loss_over_wet_patch) {
  world_yaml =
      fs::path(__FILE__).parent_path() / "surface_friction_tests/world.yaml";

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  w = World::MakeWorld(world_yaml.string());

  // Field must actually be enabled by the world block.
  EXPECT_TRUE(w->surface_friction_.Enabled());
  // Spot-check: dry near the origin, wet in the middle of the patch.
  EXPECT_NEAR(1.0, w->GetSurfaceFrictionFactor(b2Vec2(0.0f, 0.0f)), 1e-6);
  EXPECT_LT(w->GetSurfaceFrictionFactor(b2Vec2(5.0f, 0.0f)), 0.3);

  ros::NodeHandle nh;
  SurfaceFrictionTest* obj = this;
  ros::Subscriber sub = nh.subscribe(
      "odometry/ground_truth", 1, &SurfaceFrictionTest::GroundTruthCB, obj);

  DiffDrive* dd =
      dynamic_cast<DiffDrive*>(w->plugin_manager_.model_plugins_[0].get());
  ASSERT_NE(nullptr, dd);
  ASSERT_TRUE(dd->use_friction_drive_);

  auto vx = [&]() { return odom.twist.twist.linear.x; };
  auto px = [&]() { return odom.pose.pose.position.x; };

  // Step the sim while asserting the published pose never jumps discontinuously
  // (a step change in friction would show up as a solver explosion / teleport).
  double last_px = 0.0;
  auto step = [&](int n) {
    for (int i = 0; i < n; i++) {
      w->Update(timekeeper);
      ros::spinOnce();
      EXPECT_LT(std::fabs(px() - last_px), 0.2)
          << "discontinuous pose jump at x=" << px();
      last_px = px();
    }
  };

  geometry_msgs::Twist cmd;

  // 0) Settle at rest on dry ground near the origin (x < 2 m is dry).
  step(80);
  EXPECT_NEAR(0.0, vx(), 0.02);
  EXPECT_LT(px(), 0.5);

  // 1) DRY acceleration window: from rest, command a forward step. On dry
  // ground (factor ~1.0) the slip-limited acceleration is high.
  cmd.linear.x = 3.0;
  cmd.angular.z = 0.0;
  dd->TwistCallback(cmd);
  double v_before_dry = vx();
  step(3);
  double dry_gain = vx() - v_before_dry;
  EXPECT_GT(dry_gain, 0.05) << "robot did not accelerate on dry ground";

  // 2) Cruise forward into the wet patch (3 m < x < 7 m).
  int guard = 0;
  while (px() < 4.5 && guard++ < 5000) {
    step(1);
  }
  ASSERT_LT(guard, 5000) << "robot never reached the wet patch";

  // 3) WET acceleration window: now over the wet patch, command a higher
  // velocity. Grip is scaled down (factor ~0.15) so the achievable
  // acceleration is markedly smaller than on dry ground.
  cmd.linear.x = 6.0;
  dd->TwistCallback(cmd);
  double v_before_wet = vx();
  step(3);
  double wet_gain = vx() - v_before_wet;
  EXPECT_GT(wet_gain, 0.0) << "robot lost all traction (expected reduced grip)";
  EXPECT_LT(wet_gain, 0.5 * dry_gain)
      << "traction over the wet patch was not reduced (dry_gain=" << dry_gain
      << ", wet_gain=" << wet_gain << ")";

  // 4) Recovery: drive past the wet patch back onto dry ground (x > 9 m), then
  // command another step. Traction recovers to the dry level.
  guard = 0;
  while (px() < 9.0 && guard++ < 8000) {
    step(1);
  }
  ASSERT_LT(guard, 8000) << "robot never reached the dry recovery zone";

  cmd.linear.x = 9.0;
  dd->TwistCallback(cmd);
  double v_before_rec = vx();
  step(3);
  double rec_gain = vx() - v_before_rec;
  EXPECT_GT(rec_gain, 2.0 * wet_gain)
      << "traction did not recover on dry ground (wet_gain=" << wet_gain
      << ", rec_gain=" << rec_gain << ")";
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_friction_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
