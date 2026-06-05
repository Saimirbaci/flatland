/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   localization_fault_test.cpp
 * @brief  rostest: amcl_divergence diverges the localization estimate + the
 *         map->odom tf, and the sealed label never leaks in-band
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

#include <flatland_plugins/localization_fault.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

#include <cmath>
#include <regex>
#include <string>
#include <vector>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

// The exclusion the RCA bag (record_rca_bag.launch) applies. The leakage guard
// below asserts it drops the sealed topic and keeps the in-band amcl_pose.
static const char* kExcludeRegex = "/_ground_truth.*";

class LocalizationFaultTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  World* w = nullptr;

  geometry_msgs::PoseWithCovarianceStamped amcl;
  nav_msgs::Odometry truth;
  bool got_amcl = false;
  bool got_truth = false;

  // Latest map->odom transform observed on /tf.
  double map_odom_x = 0.0;
  double map_odom_y = 0.0;
  bool got_map_odom = false;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }
  void TearDown() override {
    if (w != nullptr) delete w;
  }

  void AmclCB(const geometry_msgs::PoseWithCovarianceStamped& m) {
    amcl = m;
    got_amcl = true;
  }
  void TruthCB(const nav_msgs::Odometry& m) {
    truth = m;
    got_truth = true;
  }
  void TfCB(const tf2_msgs::TFMessage& m) {
    for (const auto& t : m.transforms) {
      if (t.header.frame_id == "map" && t.child_frame_id == "odom") {
        map_odom_x = t.transform.translation.x;
        map_odom_y = t.transform.translation.y;
        got_map_odom = true;
      }
    }
  }

  double MapOdomNorm() const {
    return std::sqrt(map_odom_x * map_odom_x + map_odom_y * map_odom_y);
  }
  double AmclTruthError() const {
    double dx = amcl.pose.pose.position.x - truth.pose.pose.position.x;
    double dy = amcl.pose.pose.position.y - truth.pose.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
  }
};

// The LocalizationFault must load as a pluginlib ModelPlugin (XML + macro).
TEST_F(LocalizationFaultTest, load_test) {
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
      "flatland_server", "flatland_server::ModelPlugin");
  try {
    boost::shared_ptr<flatland_server::ModelPlugin> plugin =
        loader.createInstance("flatland_plugins::LocalizationFault");
  } catch (pluginlib::PluginlibException& e) {
    FAIL() << "Failed to load LocalizationFault plugin. " << e.what();
  }
}

// Clean -> estimate == truth and map->odom == identity. Under amcl_divergence
// the estimate and map->odom diverge by the configured, severity-scaled offset.
// The sealed label is never visible in-band; amcl_pose IS.
TEST_F(LocalizationFaultTest, amcl_divergence_and_sealed_label) {
  fs::path world_yaml =
      this_file_dir /
      fs::path("localization_fault_tests/localization_world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);  // 100 Hz
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_amcl =
      nh.subscribe("amcl_pose", 1, &LocalizationFaultTest::AmclCB, this);
  ros::Subscriber sub_truth = nh.subscribe(
      "odometry/ground_truth", 1, &LocalizationFaultTest::TruthCB, this);
  ros::Subscriber sub_tf =
      nh.subscribe("/tf", 50, &LocalizationFaultTest::TfCB, this);

  // Keep the robot stationary: a pose offset is enough to demonstrate
  // divergence and keeps the clean map->odom an exact identity at the origin.

  // --- Before onset (amcl_divergence onset = 4s): clean estimate + identity tf.
  for (int i = 0; i < 200; i++) {  // ~2.0s
    w->Update(timekeeper);
    ros::spinOnce();
  }
  ASSERT_TRUE(got_amcl);
  ASSERT_TRUE(got_truth);
  ASSERT_TRUE(got_map_odom) << "no map->odom transform seen on /tf";
  EXPECT_LT(AmclTruthError(), 0.02) << "estimate perturbed before fault onset";
  EXPECT_LT(MapOdomNorm(), 0.02) << "map->odom not identity before fault onset";

  // --- After onset + ramp (to ~6s): estimate + map->odom diverge by the offset
  // (peak params x=0.4, y=0.2 at severity 1).
  for (int i = 0; i < 400; i++) {  // to ~6.0s
    w->Update(timekeeper);
    ros::spinOnce();
  }
  EXPECT_GT(AmclTruthError(), 0.2)
      << "amcl_pose did not diverge under amcl_divergence";
  EXPECT_NEAR(amcl.pose.pose.position.x - truth.pose.pose.position.x, 0.4, 0.1)
      << "x divergence does not match the configured offset";
  EXPECT_GT(MapOdomNorm(), 0.2) << "map->odom did not diverge under the fault";

  // --- Leakage guard: the RCA bag exclusion drops the sealed GT topic but keeps
  // the in-band amcl_pose. Model exactly what rosbag does.
  std::regex exclude(kExcludeRegex);
  std::vector<ros::master::TopicInfo> topics;
  ASSERT_TRUE(ros::master::getTopics(topics));
  bool gt_present = false, gt_recorded = false, amcl_recorded = false;
  for (const auto& t : topics) {
    const bool recorded = !std::regex_match(t.name, exclude);
    if (t.name == "/_ground_truth/faults") {
      gt_present = true;
      gt_recorded = recorded;
    }
    if (t.name.find("amcl_pose") != std::string::npos && recorded)
      amcl_recorded = true;
  }
  ASSERT_TRUE(gt_present) << "reserved GT topic was never advertised";
  EXPECT_FALSE(gt_recorded) << "SEALING VIOLATION: GT topic would be recorded";
  EXPECT_TRUE(amcl_recorded) << "in-band amcl_pose wrongly excluded";
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_fault_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
