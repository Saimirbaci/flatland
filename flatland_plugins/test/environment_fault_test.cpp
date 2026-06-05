/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   environment_fault_test.cpp
 * @brief  rostest: environment / dynamic-world faults (dynamic_obstacle,
 *         moved_furniture, spill) physically mutate the running world so the
 *         perturbation is observable only through the robot's normal sensors,
 *         the clean run is unchanged, and the ground-truth label is sealed on
 *         the reserved /_ground_truth topic.
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
#include <flatland_plugins/fault_injection_registry.h>
#include <flatland_msgs/FaultGroundTruthArray.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <algorithm>
#include <boost/filesystem.hpp>
#include <cmath>
#include <limits>
#include <string>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class EnvironmentFaultTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  World* w = nullptr;

  sensor_msgs::LaserScan scan;
  bool got_scan = false;

  nav_msgs::Odometry odom;
  bool got_odom = false;

  flatland_msgs::FaultGroundTruthArray gt;
  bool got_gt = false;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
    // The registry is a process-wide singleton; clear any prior effects.
    FaultInjectionRegistry::Get().Reset();
  }
  void TearDown() override {
    if (w != nullptr) delete w;
    w = nullptr;
  }

  void ScanCB(const sensor_msgs::LaserScan& m) {
    scan = m;
    got_scan = true;
  }
  void OdomCB(const nav_msgs::Odometry& m) {
    odom = m;
    got_odom = true;
  }
  void GtCB(const flatland_msgs::FaultGroundTruthArray& m) {
    gt = m;
    got_gt = true;
  }

  World* LoadWorld(const std::string& world_file) {
    fs::path world_yaml = this_file_dir / fs::path("environment_fault_tests") /
                          fs::path(world_file);
    w = World::MakeWorld(world_yaml.string());
    return w;
  }

  DiffDrive* FindDiffDrive() {
    for (auto& p : w->plugin_manager_.model_plugins_) {
      DiffDrive* dd = dynamic_cast<DiffDrive*>(p.get());
      if (dd != nullptr) return dd;
    }
    return nullptr;
  }

  void Step(Timekeeper& tk, unsigned int n) {
    for (unsigned int i = 0; i < n; i++) {
      w->Update(tk);
      ros::spinOnce();
    }
  }

  // Smallest finite range in the latest scan (inf if every beam is a no-hit
  // NaN, which is what an empty plane produces).
  double MinFiniteRange() const {
    double m = std::numeric_limits<double>::infinity();
    for (float r : scan.ranges) {
      if (std::isfinite(r)) m = std::min(m, static_cast<double>(r));
    }
    return m;
  }
};

// Clean-run invariant: the laser robot on an empty plane, no FaultInjector,
// sees no near return for the whole run.
TEST_F(EnvironmentFaultTest, clean_run_has_no_near_return) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe("scan", 1, &EnvironmentFaultTest::ScanCB, this);
  ASSERT_NE(nullptr, LoadWorld("world_clean.yaml"));

  Step(tk, 120);  // ~1.2 s, well past the fault onset used by the other worlds
  ASSERT_TRUE(got_scan);
  EXPECT_FALSE(std::isfinite(MinFiniteRange()))
      << "clean run produced a spurious near return at "
      << MinFiniteRange() << " m";
}

// dynamic_obstacle: before onset the scan is open; after onset a neutral
// obstacle has been spawned ~2 m ahead and the laser shows a near return.
TEST_F(EnvironmentFaultTest, dynamic_obstacle_appears_in_scan) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe("scan", 1, &EnvironmentFaultTest::ScanCB, this);
  ASSERT_NE(nullptr, LoadWorld("world_obstacle.yaml"));

  // Before onset (~0.3 s < 0.5 s): no obstacle, no finite return.
  Step(tk, 30);
  ASSERT_TRUE(got_scan);
  EXPECT_FALSE(std::isfinite(MinFiniteRange()))
      << "obstacle appeared before its onset";

  // After onset: the spawned obstacle gives a near return around 2 m.
  Step(tk, 60);  // now ~0.9 s
  double near = MinFiniteRange();
  ASSERT_TRUE(std::isfinite(near)) << "obstacle never appeared in the scan";
  EXPECT_LT(near, 2.5) << "obstacle return farther than expected";
  EXPECT_GT(near, 1.0) << "obstacle return closer than expected";

  // The spawned model carries a neutral name (no fault-specific label).
  bool found_neutral = false;
  for (Model* m : w->models_) {
    if (m->GetName() == "obstacle_1") found_neutral = true;
    EXPECT_EQ(std::string::npos, m->GetName().find("fault"))
        << "spawned model name leaks the fault: " << m->GetName();
  }
  EXPECT_TRUE(found_neutral) << "obstacle was not spawned with a neutral name";
}

// moved_furniture: an existing object behind the robot (out of view) is
// relocated in front of it on the fault onset, where the laser then sees it.
TEST_F(EnvironmentFaultTest, moved_furniture_enters_field_of_view) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe("scan", 1, &EnvironmentFaultTest::ScanCB, this);
  ASSERT_NE(nullptr, LoadWorld("world_furniture.yaml"));

  // Before onset the furniture is behind the robot -> no forward return.
  Step(tk, 30);
  ASSERT_TRUE(got_scan);
  EXPECT_FALSE(std::isfinite(MinFiniteRange()))
      << "furniture visible before it was moved";

  // After onset it has been moved ~2 m ahead and is now in the scan.
  Step(tk, 60);
  double near = MinFiniteRange();
  ASSERT_TRUE(std::isfinite(near)) << "furniture never entered the scan";
  EXPECT_LT(near, 2.6);
}

// spill: a circular low-friction patch is activated mid-run. The friction field
// reports nominal grip before the first step and a slippery factor over the
// patch afterwards, and the friction-mode robot's forward progress is choked
// relative to a clean drive.
TEST_F(EnvironmentFaultTest, spill_activates_and_chokes_traction) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub_odom =
      nh.subscribe("odometry/filtered", 1, &EnvironmentFaultTest::OdomCB, this);

  ASSERT_NE(nullptr, LoadWorld("world_spill.yaml"));
  // Before any step the spill has not been applied: nominal grip everywhere.
  EXPECT_NEAR(1.0, w->GetSurfaceFrictionFactor(b2Vec2(0.5f, 0.0f)), 1e-6);

  DiffDrive* dd = FindDiffDrive();
  ASSERT_NE(nullptr, dd);
  geometry_msgs::Twist cmd;
  cmd.linear.x = 1.0;
  dd->TwistCallback(cmd);

  Step(tk, 1);  // the FaultInjector activates the spill on the first step
  EXPECT_LT(w->GetSurfaceFrictionFactor(b2Vec2(0.5f, 0.0f)), 0.2)
      << "spill did not lower the surface friction factor";

  Step(tk, 200);  // ~2 s of commanded forward motion
  ASSERT_TRUE(got_odom);
  double spill_x = odom.pose.pose.position.x;

  // Clean drive (no spill, nominal grip) for the same command and duration.
  delete w;
  w = nullptr;
  got_odom = false;
  ASSERT_NE(nullptr, LoadWorld("world_clean_drive.yaml"));
  DiffDrive* dd_clean = FindDiffDrive();
  ASSERT_NE(nullptr, dd_clean);
  dd_clean->TwistCallback(cmd);
  Step(tk, 201);
  ASSERT_TRUE(got_odom);
  double clean_x = odom.pose.pose.position.x;

  EXPECT_GT(clean_x, 0.3) << "clean drive failed to move";
  EXPECT_LT(spill_x, clean_x)
      << "spill did not choke traction (spill_x=" << spill_x
      << " clean_x=" << clean_x << ")";
}

// The environment-fault label is published ONLY on the reserved
// /_ground_truth namespace; the in-band scan stays a plain LaserScan.
TEST_F(EnvironmentFaultTest, ground_truth_is_sealed_out_of_band) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub_scan =
      nh.subscribe("scan", 1, &EnvironmentFaultTest::ScanCB, this);
  ros::Subscriber sub_gt = nh.subscribe(
      "/_ground_truth/faults", 4, &EnvironmentFaultTest::GtCB, this);
  ASSERT_NE(nullptr, LoadWorld("world_obstacle.yaml"));

  Step(tk, 90);  // past onset, several ground-truth publish periods
  ASSERT_TRUE(got_scan);
  ASSERT_TRUE(got_gt) << "no ground truth on the reserved topic";
  ASSERT_FALSE(gt.faults.empty());
  EXPECT_EQ("dynamic_obstacle", gt.faults[0].fault_type);
  EXPECT_EQ("environment", gt.faults[0].affected_component);
  // The label identifies the (neutral) obstacle model, not a fault-named one.
  EXPECT_EQ("obstacle", gt.faults[0].affected_model);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "environment_fault_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
