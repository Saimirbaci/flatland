/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   actuator_fault_test.cpp
 * @brief  rostest: actuator-stage drivetrain faults (motor_degradation,
 *         asymmetric_wheel_speed, locked_wheel, controller_latency) perturb the
 *         in-band motion/odom causally, the clean run is unchanged, and the
 *         ground-truth label is sealed on the reserved /_ground_truth topic.
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
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <boost/filesystem.hpp>
#include <cmath>
#include <string>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class ActuatorFaultTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  World* w = nullptr;

  nav_msgs::Odometry odom;   // in-band reported odom (odometry/filtered)
  nav_msgs::Odometry truth;  // clean ground truth (odometry/ground_truth)
  bool got_odom = false;
  bool got_truth = false;

  // Sealed out-of-band ground-truth label (reserved /_ground_truth topic).
  flatland_msgs::FaultGroundTruthArray gt;
  bool got_gt = false;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
    // The registry is a process-wide singleton and a clean world has no
    // FaultInjector to overwrite it, so clear any effects a previous test left.
    FaultInjectionRegistry::Get().Reset();
  }
  void TearDown() override {
    if (w != nullptr) delete w;
    w = nullptr;
  }

  void OdomCB(const nav_msgs::Odometry& m) {
    odom = m;
    got_odom = true;
  }
  void TruthCB(const nav_msgs::Odometry& m) {
    truth = m;
    got_truth = true;
  }
  void GtCB(const flatland_msgs::FaultGroundTruthArray& m) {
    gt = m;
    got_gt = true;
  }

  DiffDrive* LoadWorld(const std::string& world_file) {
    fs::path world_yaml =
        this_file_dir / fs::path("actuator_fault_tests") / fs::path(world_file);
    w = World::MakeWorld(world_yaml.string());
    DiffDrive* dd = nullptr;
    for (auto& p : w->plugin_manager_.model_plugins_) {
      dd = dynamic_cast<DiffDrive*>(p.get());
      if (dd != nullptr) break;
    }
    return dd;
  }

  void Step(Timekeeper& tk, unsigned int n) {
    for (unsigned int i = 0; i < n; i++) {
      w->Update(tk);
      ros::spinOnce();
    }
  }

  // Reported odom heading [rad].
  double Yaw() const { return tf::getYaw(odom.pose.pose.orientation); }
};

// Clean run (no FaultInjector): a straight forward command drives the robot
// forward with negligible heading change. This is the reference the fault runs
// are compared against (clean-run invariant).
TEST_F(ActuatorFaultTest, clean_run_drives_straight) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub_odom =
      nh.subscribe("odometry/filtered", 1, &ActuatorFaultTest::OdomCB, this);
  DiffDrive* dd = LoadWorld("world_clean.yaml");
  ASSERT_NE(nullptr, dd);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;
  dd->TwistCallback(cmd);

  Step(tk, 200);  // ~2 s
  ASSERT_TRUE(got_odom);
  // Reached (near) the commanded forward speed and travelled a clear distance.
  EXPECT_NEAR(0.5, odom.twist.twist.linear.x, 0.05);
  EXPECT_GT(odom.pose.pose.position.x, 0.5);
  // No yaw drift on a straight command.
  EXPECT_LT(std::fabs(Yaw()), 0.05);
}

// motor_degradation collapses the effort cap, so the robot can barely
// accelerate: far lower achieved speed and distance than the clean run.
TEST_F(ActuatorFaultTest, motor_degradation_limits_speed) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub_odom =
      nh.subscribe("odometry/filtered", 1, &ActuatorFaultTest::OdomCB, this);
  DiffDrive* dd = LoadWorld("world_motor_degradation.yaml");
  ASSERT_NE(nullptr, dd);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;
  dd->TwistCallback(cmd);

  Step(tk, 200);  // ~2 s
  ASSERT_TRUE(got_odom);
  // The clean run reaches ~0.5 m/s and travels > 0.5 m; the degraded motor is
  // an order of magnitude slower / shorter.
  EXPECT_LT(odom.twist.twist.linear.x, 0.1);
  EXPECT_LT(odom.pose.pose.position.x, 0.1);
}

// asymmetric_wheel_speed slows one wheel, coupling a forward-speed drop with a
// measurable leftward (positive) yaw drift through Box2D.
TEST_F(ActuatorFaultTest, asymmetric_wheel_speed_veers) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub_odom =
      nh.subscribe("odometry/filtered", 1, &ActuatorFaultTest::OdomCB, this);
  DiffDrive* dd = LoadWorld("world_asymmetric.yaml");
  ASSERT_NE(nullptr, dd);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;  // straight command; the fault induces the turn
  dd->TwistCallback(cmd);

  Step(tk, 200);
  ASSERT_TRUE(got_odom);
  // Slowing the left (+y) wheel turns the robot left (+yaw).
  EXPECT_GT(Yaw(), 0.2) << "asymmetric_wheel_speed did not veer left";
  // Still made forward progress (it is an imbalance, not a full stop).
  EXPECT_GT(odom.pose.pose.position.x, 0.05);
}

// locked_wheel seizes one wheel to ~0, so a straight command pivots the robot
// (large heading change) through Box2D.
TEST_F(ActuatorFaultTest, locked_wheel_pivots) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub_odom =
      nh.subscribe("odometry/filtered", 1, &ActuatorFaultTest::OdomCB, this);
  DiffDrive* dd = LoadWorld("world_locked.yaml");
  ASSERT_NE(nullptr, dd);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;
  dd->TwistCallback(cmd);

  Step(tk, 200);
  ASSERT_TRUE(got_odom);
  // The locked left wheel makes the robot pivot left (large +yaw).
  EXPECT_GT(Yaw(), 0.3) << "locked_wheel did not pivot";
}

// controller_latency injects ~0.5 s of extra transport deadtime, so the drive
// response to a cmd_vel step is delayed: no motion inside the window, motion
// after it. The command is issued AFTER the fault is active (a few primed steps)
// so the delay line is exercised, not the unprimed first step.
TEST_F(ActuatorFaultTest, controller_latency_delays_response) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub_odom =
      nh.subscribe("odometry/filtered", 1, &ActuatorFaultTest::OdomCB, this);
  DiffDrive* dd = LoadWorld("world_latency.yaml");
  ASSERT_NE(nullptr, dd);

  // Prime: a few steps with a zero command let the FaultInjector publish the
  // effect into the registry before the step command is issued.
  Step(tk, 5);
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;
  dd->TwistCallback(cmd);

  // Inside the latency window (~0.3 s after the command, < 0.5 s) there is no
  // motion yet.
  Step(tk, 30);
  ASSERT_TRUE(got_odom);
  EXPECT_NEAR(0.0, odom.twist.twist.linear.x, 0.03)
      << "moved before the controller_latency deadtime elapsed";

  // Well past the deadtime the command takes effect and the robot drives.
  Step(tk, 120);
  EXPECT_GT(odom.twist.twist.linear.x, 0.1)
      << "command never took effect after the latency window";
}

// The fault label is published ONLY on the reserved /_ground_truth namespace,
// never embedded in the in-band odom (which stays a plain Odometry message).
TEST_F(ActuatorFaultTest, ground_truth_is_sealed_out_of_band) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ros::NodeHandle nh;
  ros::Subscriber sub_odom =
      nh.subscribe("odometry/filtered", 1, &ActuatorFaultTest::OdomCB, this);
  ros::Subscriber sub_gt = nh.subscribe(
      "/_ground_truth/faults", 4, &ActuatorFaultTest::GtCB, this);
  DiffDrive* dd = LoadWorld("world_motor_degradation.yaml");
  ASSERT_NE(nullptr, dd);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;
  dd->TwistCallback(cmd);

  Step(tk, 100);
  ASSERT_TRUE(got_odom);
  // The sealed label arrived out-of-band and identifies the fault.
  ASSERT_TRUE(got_gt) << "no ground truth on the reserved topic";
  ASSERT_FALSE(gt.faults.empty());
  EXPECT_EQ("motor_degradation", gt.faults[0].fault_type);
  EXPECT_EQ("diff_drive", gt.faults[0].affected_component);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "actuator_fault_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
