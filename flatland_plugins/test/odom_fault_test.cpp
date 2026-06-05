/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   odom_fault_test.cpp
 * @brief  rostest: measurement-domain odom faults diverge odom from ground
 *         truth (diff_drive + tricycle_drive), clean run stays equal
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
#include <flatland_plugins/tricycle_drive.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cmath>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class OdomFaultTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  World* w = nullptr;

  nav_msgs::Odometry odom;        // in-band reported odom (odometry/filtered)
  nav_msgs::Odometry truth;       // clean ground truth (odometry/ground_truth)
  bool got_odom = false;
  bool got_truth = false;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }
  void TearDown() override {
    if (w != nullptr) delete w;
  }

  void OdomCB(const nav_msgs::Odometry& m) {
    odom = m;
    got_odom = true;
  }
  void TruthCB(const nav_msgs::Odometry& m) {
    truth = m;
    got_truth = true;
  }

  // Planar position error between the reported odom and the ground truth.
  double PoseError() const {
    double dx = odom.pose.pose.position.x - truth.pose.pose.position.x;
    double dy = odom.pose.pose.position.y - truth.pose.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
  }
};

// diff_drive: before the fault onset the reported odom tracks the ground truth
// (clean-run invariant); after encoder_drift / odom_slip fire the odom diverges
// measurably while the ground truth still reflects the true forward motion.
TEST_F(OdomFaultTest, diff_drive_odom_diverges_from_truth) {
  fs::path world_yaml =
      this_file_dir /
      fs::path("localization_fault_tests/localization_world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);  // 100 Hz
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_odom =
      nh.subscribe("odometry/filtered", 1, &OdomFaultTest::OdomCB, this);
  ros::Subscriber sub_truth =
      nh.subscribe("odometry/ground_truth", 1, &OdomFaultTest::TruthCB, this);

  DiffDrive* dd = nullptr;
  for (auto& p : w->plugin_manager_.model_plugins_) {
    dd = dynamic_cast<DiffDrive*>(p.get());
    if (dd != nullptr) break;
  }
  ASSERT_NE(nullptr, dd) << "diff_drive plugin not found";

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;  // m/s forward
  dd->TwistCallback(cmd);

  // --- Before onset (t < 2s): clean. odom == ground truth (zero odom noise).
  for (int i = 0; i < 150; i++) {  // ~1.5s
    w->Update(timekeeper);
    ros::spinOnce();
  }
  ASSERT_TRUE(got_odom);
  ASSERT_TRUE(got_truth);
  EXPECT_LT(PoseError(), 0.02) << "odom perturbed before any fault onset";

  // --- After onset + ramp (to ~4.5s): encoder_drift + slip diverge the odom.
  for (int i = 0; i < 300; i++) {  // to ~4.5s
    w->Update(timekeeper);
    ros::spinOnce();
  }
  // Ground truth still advanced forward under the unchanged true motion.
  EXPECT_GT(truth.pose.pose.position.x, 0.5)
      << "robot did not actually move forward";
  // The reported odom has measurably diverged from the truth.
  EXPECT_GT(PoseError(), 0.1)
      << "odom did not diverge from ground truth under encoder_drift/odom_slip";
}

// tricycle_drive parity: same measurement-domain divergence behavior.
TEST_F(OdomFaultTest, tricycle_drive_odom_diverges_from_truth) {
  fs::path world_yaml =
      this_file_dir / fs::path("localization_fault_tests/tricycle_world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_odom =
      nh.subscribe("odometry/filtered", 1, &OdomFaultTest::OdomCB, this);
  ros::Subscriber sub_truth =
      nh.subscribe("odometry/ground_truth", 1, &OdomFaultTest::TruthCB, this);

  TricycleDrive* td = nullptr;
  for (auto& p : w->plugin_manager_.model_plugins_) {
    td = dynamic_cast<TricycleDrive*>(p.get());
    if (td != nullptr) break;
  }
  ASSERT_NE(nullptr, td) << "tricycle_drive plugin not found";

  geometry_msgs::Twist cmd;
  cmd.linear.x = 1.0;    // front-wheel drive speed
  cmd.angular.z = 0.0;   // straight ahead
  td->TwistCallback(cmd);

  // --- Before onset (t < 2s): clean. odom == ground truth.
  for (int i = 0; i < 150; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }
  ASSERT_TRUE(got_odom);
  ASSERT_TRUE(got_truth);
  EXPECT_LT(PoseError(), 0.02) << "tricycle odom perturbed before fault onset";

  // --- After onset + ramp: odom diverges while truth tracks true motion.
  for (int i = 0; i < 300; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }
  EXPECT_GT(std::fabs(truth.pose.pose.position.x), 0.5)
      << "tricycle did not actually move";
  EXPECT_GT(PoseError(), 0.1)
      << "tricycle odom did not diverge under encoder_drift/odom_slip";
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_fault_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
