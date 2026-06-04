/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   sensor_fault_test.cpp
 * @brief  rostest: laser/gps/bumper in-band fault perturbation + sealed label
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

#include <flatland_msgs/Collisions.h>
#include <flatland_plugins/diff_drive.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>

#include <cmath>
#include <regex>
#include <string>
#include <vector>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

// The exclusion the RCA bag (record_rca_bag.launch) applies. The sealing guard
// below asserts it drops the reserved ground-truth topic but keeps the in-band
// sensor topics.
static const char *kExcludeRegex = "/_ground_truth.*";

class SensorFaultTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  World *w = nullptr;
  Timekeeper timekeeper;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
    timekeeper.SetMaxStepSize(0.01);  // 100 Hz
  }
  void TearDown() override {
    if (w != nullptr) delete w;
  }

  // Step the world (and pump ROS callbacks) until sim time reaches target_sec.
  void StepTo(double target_sec) {
    while (timekeeper.GetSimTime().toSec() < target_sec) {
      w->Update(timekeeper);
      ros::spinOnce();
    }
  }

  double SimNow() const { return timekeeper.GetSimTime().toSec(); }

  // Locate the DiffDrive plugin so a test can drive the robot.
  DiffDrive *FindDiffDrive() {
    for (auto &p : w->plugin_manager_.model_plugins_) {
      DiffDrive *dd = dynamic_cast<DiffDrive *>(p.get());
      if (dd != nullptr) return dd;
    }
    return nullptr;
  }

  // Model exactly what `rosbag record -a --exclude` does: the sealed GT topic
  // is dropped, the named in-band topic is kept.
  void AssertSealing(const std::string &inband_substr) {
    std::regex exclude(kExcludeRegex);
    std::vector<ros::master::TopicInfo> topics;
    ASSERT_TRUE(ros::master::getTopics(topics));
    bool gt_present = false, gt_recorded = false, inband_recorded = false;
    for (const auto &t : topics) {
      const bool recorded = !std::regex_match(t.name, exclude);
      if (t.name == "/_ground_truth/faults") {
        gt_present = true;
        gt_recorded = recorded;
      }
      if (t.name.find(inband_substr) != std::string::npos && recorded)
        inband_recorded = true;
    }
    ASSERT_TRUE(gt_present) << "reserved GT topic was never advertised";
    EXPECT_FALSE(gt_recorded)
        << "SEALING VIOLATION: GT topic would be recorded";
    EXPECT_TRUE(inband_recorded)
        << "in-band topic " << inband_substr << " wrongly excluded";
  }
};

// --- Laser: range bias, ghost returns, latency ----------------------------
class LaserFaultTest : public SensorFaultTest {
 public:
  sensor_msgs::LaserScan scan;
  bool got_scan = false;
  void ScanCB(const sensor_msgs::LaserScan &m) {
    scan = m;
    got_scan = true;
  }
  static size_t FiniteCount(const sensor_msgs::LaserScan &s) {
    size_t n = 0;
    for (float r : s.ranges)
      if (std::isfinite(r)) n++;
    return n;
  }
};

TEST_F(LaserFaultTest, bias_ghost_latency_and_sealing) {
  fs::path world_yaml =
      this_file_dir / fs::path("sensor_fault_tests/laser_fault_world.yaml");
  w = World::MakeWorld(world_yaml.string());
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("scan", 1, &LaserFaultTest::ScanCB, this);

  // --- Clean baseline (t < 1s): record the unperturbed scan.
  StepTo(0.6);
  ASSERT_TRUE(got_scan) << "no scan published before fault onset";
  sensor_msgs::LaserScan base = scan;
  const size_t base_finite = FiniteCount(base);
  const size_t base_nan = base.ranges.size() - base_finite;
  ASSERT_GT(base_finite, 0u) << "laser sees no finite returns; check world";

  // --- Range bias active (t in [1,2]): every finite beam shifts by +0.5 m.
  StepTo(1.5);
  size_t compared = 0;
  double sum_diff = 0.0;
  for (size_t i = 0; i < base.ranges.size() && i < scan.ranges.size(); i++) {
    if (std::isfinite(base.ranges[i]) && std::isfinite(scan.ranges[i])) {
      sum_diff += scan.ranges[i] - base.ranges[i];
      compared++;
    }
  }
  ASSERT_GT(compared, 0u);
  EXPECT_NEAR(sum_diff / compared, 0.5, 0.05)
      << "expected ~0.5 m additive range bias in-band";

  // --- Bias gone (t > 2): ranges return to the clean baseline.
  StepTo(2.6);
  double sum_back = 0.0;
  size_t back_n = 0;
  for (size_t i = 0; i < base.ranges.size() && i < scan.ranges.size(); i++) {
    if (std::isfinite(base.ranges[i]) && std::isfinite(scan.ranges[i])) {
      sum_back += std::fabs(scan.ranges[i] - base.ranges[i]);
      back_n++;
    }
  }
  ASSERT_GT(back_n, 0u);
  EXPECT_LT(sum_back / back_n, 0.05) << "bias did not clear after its window";

  // --- Ghost returns active (t in [3,4]): spurious finite returns appear.
  StepTo(3.5);
  const size_t ghost_finite = FiniteCount(scan);
  EXPECT_GE(ghost_finite, base_finite)
      << "ghost returns reduced the finite-return count";
  if (base_nan > 0) {
    EXPECT_GT(ghost_finite, base_finite)
        << "ghost returns never filled the empty (NaN) beams";
    for (float r : scan.ranges) {
      if (std::isfinite(r)) {
        EXPECT_GE(r, scan.range_min);
        EXPECT_LE(r, scan.range_max + 1e-3f);
      }
    }
  }

  // --- Latency active (t >= 5): delivered scan stamp lags sim time by ~0.5 s.
  StepTo(4.8);
  ros::spinOnce();
  double lag_clean = SimNow() - scan.header.stamp.toSec();
  EXPECT_LT(lag_clean, 0.1) << "scan delivery lagged before latency onset";

  StepTo(6.2);
  ros::spinOnce();
  double lag_fault = SimNow() - scan.header.stamp.toSec();
  EXPECT_GT(lag_fault, 0.3) << "latency fault did not delay scan delivery";

  AssertSealing("scan");
}

// --- GPS: noise, freeze, latency ------------------------------------------
class GpsFaultTest : public SensorFaultTest {
 public:
  sensor_msgs::NavSatFix fix;
  bool got_fix = false;
  void FixCB(const sensor_msgs::NavSatFix &m) {
    fix = m;
    got_fix = true;
  }
};

TEST_F(GpsFaultTest, noise_freeze_latency_and_sealing) {
  fs::path world_yaml =
      this_file_dir / fs::path("sensor_fault_tests/gps_fault_world.yaml");
  w = World::MakeWorld(world_yaml.string());
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("gps/fix", 1, &GpsFaultTest::FixCB, this);

  // --- Clean baseline (t < 1s, robot stationary): record the unperturbed fix.
  StepTo(0.6);
  ASSERT_TRUE(got_fix) << "no GPS fix published before fault onset";
  const double base_lat = fix.latitude;

  // --- Noise active (t in [1,2], still stationary): the fix jitters, so two
  // samples within the window differ from each other and from the baseline.
  StepTo(1.3);
  double n1 = fix.latitude;
  StepTo(1.6);
  double n2 = fix.latitude;
  EXPECT_NE(n1, n2) << "GPS noise produced no jitter between samples";
  EXPECT_GT(std::fabs(n1 - base_lat) + std::fabs(n2 - base_lat), 0.0)
      << "GPS noise did not perturb the fix";

  // Start moving so the freeze fault is observable.
  DiffDrive *dd = FindDiffDrive();
  ASSERT_NE(nullptr, dd) << "diff_drive plugin not found";
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;
  dd->TwistCallback(cmd);

  // --- Clean moving window (t in [2,3]): consecutive fixes change.
  StepTo(2.6);
  double m1 = fix.latitude;
  StepTo(2.9);
  double m2 = fix.latitude;
  EXPECT_NE(m1, m2)
      << "moving robot produced a constant fix; cannot test freeze";

  // --- Freeze active (t in [3,4]): the fix is frozen despite the motion.
  StepTo(3.3);
  double f1_lat = fix.latitude;
  double f1_lon = fix.longitude;
  StepTo(3.7);
  EXPECT_DOUBLE_EQ(fix.latitude, f1_lat) << "freeze fault did not pin latitude";
  EXPECT_DOUBLE_EQ(fix.longitude, f1_lon)
      << "freeze fault did not pin longitude";

  // --- Latency active (t >= 5): delivered fix stamp lags sim time by ~0.4 s.
  StepTo(4.8);
  ros::spinOnce();
  double lag_clean = SimNow() - fix.header.stamp.toSec();
  EXPECT_LT(lag_clean, 0.1) << "fix delivery lagged before latency onset";

  StepTo(6.2);
  ros::spinOnce();
  double lag_fault = SimNow() - fix.header.stamp.toSec();
  EXPECT_GT(lag_fault, 0.25) << "latency fault did not delay fix delivery";

  AssertSealing("gps/fix");
}

// --- Bumper: phantom (ghost) contact, dropout, latency --------------------
class BumperFaultTest : public SensorFaultTest {
 public:
  flatland_msgs::Collisions collisions;
  bool got_collisions = false;
  int collisions_count = 0;
  void CollisionsCB(const flatland_msgs::Collisions &m) {
    collisions = m;
    got_collisions = true;
    collisions_count++;
  }
};

TEST_F(BumperFaultTest, ghost_dropout_latency_and_sealing) {
  fs::path world_yaml =
      this_file_dir / fs::path("sensor_fault_tests/bumper_fault_world.yaml");
  w = World::MakeWorld(world_yaml.string());
  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe("collisions", 5, &BumperFaultTest::CollisionsCB, this);

  // --- Clean (t < 1s): the robot touches nothing, so collisions are empty.
  StepTo(0.6);
  ASSERT_TRUE(got_collisions) << "no collisions message published";
  EXPECT_EQ(collisions.collisions.size(), 0u)
      << "clean run reported a phantom collision";

  // --- Ghost contact active (t in [1,2]): a phantom collision appears in-band.
  StepTo(1.5);
  ASSERT_GE(collisions.collisions.size(), 1u)
      << "ghost fault produced no phantom collision";
  EXPECT_EQ(collisions.collisions[0].entity_B, "phantom")
      << "phantom collision not labelled as expected";

  // --- Clean window (t in [2,3]): count the messages we receive.
  collisions_count = 0;
  StepTo(3.0);
  int clean_count = collisions_count;
  EXPECT_GT(clean_count, 0) << "no collisions delivered in the clean window";

  // --- Dropout active (t in [3,4]): messages are dropped, so far fewer arrive.
  collisions_count = 0;
  StepTo(4.0);
  int dropout_count = collisions_count;
  EXPECT_LT(dropout_count, clean_count)
      << "dropout fault did not reduce the collisions message rate";

  // --- Latency active (t >= 5): delivered stamp lags sim time by ~0.4 s.
  StepTo(4.8);
  ros::spinOnce();
  double lag_clean = SimNow() - collisions.header.stamp.toSec();
  EXPECT_LT(lag_clean, 0.2) << "collisions lagged before latency onset";

  StepTo(6.2);
  ros::spinOnce();
  double lag_fault = SimNow() - collisions.header.stamp.toSec();
  EXPECT_GT(lag_fault, 0.25) << "latency fault did not delay collisions";

  AssertSealing("collisions");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_fault_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
