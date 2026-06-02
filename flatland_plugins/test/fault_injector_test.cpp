/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   fault_injector_test.cpp
 * @brief  rostest: in-band perturbation present, label sealed out-of-band
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

#include <flatland_msgs/FaultGroundTruthArray.h>
#include <flatland_plugins/diff_drive.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <flatland_server/world_plugin.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

// The exclusion the RCA bag (record_rca_bag.launch) applies. Kept in sync with
// the launch file; the void-benchmark guard below asserts it drops the sealed
// topic and keeps every in-band one.
static const char *kExcludeRegex = "/_ground_truth.*";

class FaultInjectorTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  World *w = nullptr;

  sensor_msgs::Imu imu_filtered;
  sensor_msgs::Imu imu_truth;
  bool got_filtered = false;
  bool got_truth = false;
  flatland_msgs::FaultGroundTruthArray gt;
  bool got_gt = false;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }
  void TearDown() override {
    if (w != nullptr) delete w;
  }

  void ImuFilteredCB(const sensor_msgs::Imu &m) {
    imu_filtered = m;
    got_filtered = true;
  }
  void ImuTruthCB(const sensor_msgs::Imu &m) {
    imu_truth = m;
    got_truth = true;
  }
  void GtCB(const flatland_msgs::FaultGroundTruthArray &m) {
    gt = m;
    got_gt = true;
  }

  static double Yaw(const geometry_msgs::Quaternion &q_msg) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(q_msg, q);
    return tf::getYaw(q);
  }
};

// The FaultInjector must load as a pluginlib WorldPlugin (XML + macro present).
TEST_F(FaultInjectorTest, load_test) {
  pluginlib::ClassLoader<flatland_server::WorldPlugin> loader(
      "flatland_server", "flatland_server::WorldPlugin");
  try {
    boost::shared_ptr<flatland_server::WorldPlugin> plugin =
        loader.createInstance("flatland_plugins::FaultInjector");
  } catch (pluginlib::PluginlibException &e) {
    FAIL() << "Failed to load FaultInjector plugin. " << e.what();
  }
}

// The whole contract in one run: the fault manifests ONLY in the in-band
// signal, the label is emitted ONLY out-of-band (reserved topic + sidecar
// file), and the RCA bag exclusion drops the sealed topic but keeps in-band.
TEST_F(FaultInjectorTest, in_band_perturbation_and_sealed_label) {
  const std::string manifest_path = "/tmp/flatland_fault_ground_truth.json";
  std::remove(manifest_path.c_str());

  fs::path world_yaml =
      this_file_dir / fs::path("fault_injection_tests/fault_world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);  // 100 Hz
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_filtered =
      nh.subscribe("imu/filtered", 1, &FaultInjectorTest::ImuFilteredCB, this);
  ros::Subscriber sub_truth =
      nh.subscribe("imu/ground_truth", 1, &FaultInjectorTest::ImuTruthCB, this);
  ros::Subscriber sub_gt =
      nh.subscribe("/_ground_truth/faults", 10, &FaultInjectorTest::GtCB, this);

  // Drive forward so the condition-triggered (distance_travelled) fault fires.
  DiffDrive *dd = nullptr;
  for (auto &p : w->plugin_manager_.model_plugins_) {
    dd = dynamic_cast<DiffDrive *>(p.get());
    if (dd != nullptr) break;
  }
  ASSERT_NE(nullptr, dd) << "diff_drive plugin not found";
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;
  dd->TwistCallback(cmd);

  // --- Before onset (t < 2s): the in-band IMU is clean (matches ground truth).
  for (int i = 0; i < 150; i++) {  // ~1.5s
    w->Update(timekeeper);
    ros::spinOnce();
  }
  ASSERT_TRUE(got_filtered);
  ASSERT_TRUE(got_truth);
  double before_diff =
      std::fabs(Yaw(imu_filtered.orientation) - Yaw(imu_truth.orientation));
  EXPECT_LT(before_diff, 0.05) << "IMU perturbed before fault onset";

  // --- After onset + ramp (t ~ 4s): in-band IMU yaw is biased ~0.5 rad while
  // the clean ground-truth IMU is unchanged. The fault is visible ONLY here.
  for (int i = 0; i < 300; i++) {  // to ~4.5s
    w->Update(timekeeper);
    ros::spinOnce();
  }
  double after_diff =
      Yaw(imu_filtered.orientation) - Yaw(imu_truth.orientation);
  EXPECT_NEAR(after_diff, 0.5, 0.12) << "expected ~0.5 rad in-band IMU bias";

  // --- Sealed topic: labels are published out-of-band with correct content.
  ASSERT_TRUE(got_gt) << "no message on the reserved /_ground_truth topic";
  ASSERT_EQ(gt.faults.size(), 3u);
  bool imu_bias_active = false;
  bool torque_loss_active = false;  // condition + chaining must have fired
  for (const auto &f : gt.faults) {
    if (f.fault_id == "imu_bias_1") {
      EXPECT_EQ(f.fault_type, "sensor_bias");
      EXPECT_EQ(f.affected_component, "imu");
      if (f.current_severity > 0.0) imu_bias_active = true;
    }
    if (f.fault_id == "torque_loss_1" && f.current_severity > 0.0) {
      torque_loss_active = true;
    }
  }
  EXPECT_TRUE(imu_bias_active);
  EXPECT_TRUE(torque_loss_active)
      << "condition-triggered / chained fault never fired";

  // --- Sealed sidecar file exists and carries the labels.
  std::ifstream in(manifest_path.c_str());
  ASSERT_TRUE(in.is_open()) << "sealed manifest not written";
  std::stringstream ss;
  ss << in.rdbuf();
  std::string manifest = ss.str();
  EXPECT_NE(manifest.find("imu_bias_1"), std::string::npos);
  EXPECT_NE(manifest.find("laser_occlusion_1"), std::string::npos);
  EXPECT_NE(manifest.find("torque_loss_1"), std::string::npos);

  // --- Void-benchmark guard: the RCA bag exclusion (record -a --exclude) drops
  // the sealed topic and keeps every in-band one. Model exactly what rosbag
  // does: filter the live topic list by the exclude regex.
  std::regex exclude(kExcludeRegex);
  std::vector<ros::master::TopicInfo> topics;
  ASSERT_TRUE(ros::master::getTopics(topics));
  bool gt_present = false, gt_recorded = false;
  bool scan_recorded = false, imu_recorded = false;
  for (const auto &t : topics) {
    const bool recorded = !std::regex_match(t.name, exclude);
    if (t.name == "/_ground_truth/faults") {
      gt_present = true;
      gt_recorded = recorded;
    }
    if (t.name.find("imu/filtered") != std::string::npos && recorded)
      imu_recorded = true;
    if (t.name.find("scan") != std::string::npos && recorded)
      scan_recorded = true;
  }
  ASSERT_TRUE(gt_present) << "reserved GT topic was never advertised";
  EXPECT_FALSE(gt_recorded) << "SEALING VIOLATION: GT topic would be recorded";
  EXPECT_TRUE(imu_recorded) << "in-band imu/filtered wrongly excluded";
  EXPECT_TRUE(scan_recorded) << "in-band scan wrongly excluded";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fault_injector_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
