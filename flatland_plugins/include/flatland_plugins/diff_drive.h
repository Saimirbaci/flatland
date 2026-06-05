/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	diff_drive.h
 * @brief   Diff drive plugin
 * @author  Mike Brousseau
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
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
#include <flatland_plugins/actuator_dynamics.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_plugins/dynamics_limits.h>
#include <flatland_plugins/wheel_friction_model.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <random>

#ifndef FLATLAND_PLUGINS_DIFFDRIVE_H
#define FLATLAND_PLUGINS_DIFFDRIVE_H

using namespace flatland_server;

namespace flatland_plugins {

class DiffDrive : public flatland_server::ModelPlugin {
 public:
  ros::Subscriber twist_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher ground_truth_pub_;
  ros::Publisher twist_pub_;
  Body* body_;
  geometry_msgs::Twist twist_msg_;
  nav_msgs::Odometry odom_msg_;
  nav_msgs::Odometry ground_truth_msg_;
  UpdateTimer update_timer_;
  tf::TransformBroadcaster tf_broadcaster;  ///< For publish ROS TF
  bool enable_odom_pub_;            ///< YAML parameter to enable odom publishing
  bool enable_odom_tf_pub_;         ///< YAML parameter to enable odom tf publishing
  bool enable_twist_pub_;           ///< YAML parameter to enable twist publishing
  bool twist_in_local_frame_;  ///< YAML parameter to publish velocity in local
                               /// frame. Original diff drive plugin publishes
                               /// local velocity wrt to odom frame
  DynamicsLimits angular_dynamics_; ///< Angular dynamics constraints
  DynamicsLimits linear_dynamics_;  ///< Linear dynamics constraints
  ActuatorDynamics linear_actuator_;   ///< Linear actuator/motor dynamics
                                       /// (latency, deadband, force cap)
  ActuatorDynamics angular_actuator_;  ///< Angular actuator/motor dynamics
                                       /// (latency, deadband, torque cap)
  double angular_velocity_ = 0.0;
  double linear_velocity_ = 0.0;

  WheelFrictionModel wheel_friction_;  ///< Anisotropic wheel-ground friction model
  bool use_friction_drive_ = false;    ///< drive_mode: friction (force-based) vs
                                       /// kinematic (default, SetLinearVelocity)
  double wheel_separation_ = 0.0;      ///< Track width between drive wheels [m]

  std::default_random_engine rng_;
  std::array<std::normal_distribution<double>, 6> noise_gen_;

  std::string fault_key_;  ///< registry component key (model/plugin)

  // Measurement-domain odometry fault state (encoder_drift / odom_slip). The
  // true Box2D motion is never touched; these only make the REPORTED odom pose
  // (odom_msg_ + odom tf) dead-reckon away from the ground truth. Until either
  // fault first goes active, odom is copied verbatim from the ground truth so a
  // clean run stays byte-for-byte identical. The divergence persists after the
  // fault window closes (a real odometry error does not heal itself).
  bool odom_diverged_ = false;     ///< latched once a meas. odom fault fires
  double odom_x_ = 0.0;            ///< dead-reckoned reported odom x [m]
  double odom_y_ = 0.0;            ///< dead-reckoned reported odom y [m]
  double odom_yaw_ = 0.0;          ///< dead-reckoned reported odom yaw [rad]
  double last_gt_x_ = 0.0;         ///< previous ground-truth x sample [m]
  double last_gt_y_ = 0.0;         ///< previous ground-truth y sample [m]
  double last_gt_angle_ = 0.0;     ///< previous ground-truth yaw sample [rad]
  bool gt_sample_valid_ = false;   ///< true once last_gt_* hold a sample

  /**
   * @name          OnInitialize
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void OnInitialize(const YAML::Node& config) override;
  /**
   * @name          BeforePhysicsStep
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void BeforePhysicsStep(const Timekeeper& timekeeper) override;
  /**
   * @name          ApplyFrictionDrive
   * @brief         Apply slip-limited traction forces at each drive wheel
   * @details       Force-based alternative to the kinematic SetLinearVelocity
   *                path. For each of the two drive wheels it computes the
   *                longitudinal/lateral slip between the commanded (limited)
   *                twist and the actual contact-patch velocity, then applies the
   *                anisotropic Coulomb friction force from wheel_friction_.
   * @param[in]     b2body       The model body to drive
   * @param[in]     dt           The physics timestep [s]
   * @param[in]     effort_scale Transient multiplier on the motor force cap
   *                             (1.0 = unchanged; motor_degradation shrinks it)
   */
  void ApplyFrictionDrive(b2Body* b2body, double dt, double effort_scale = 1.0);
  /**
   * @name        TwistCallback
   * @brief       callback to apply twist (velocity and omega)
   * @param[in]   timestep how much the physics time will increment
   */
  void TwistCallback(const geometry_msgs::Twist& msg);
};
};

#endif
