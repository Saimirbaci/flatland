/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	DiffDrive.cpp
 * @brief   DiffDrive plugin
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
#include <algorithm>
#include <cmath>
#include <flatland_plugins/diff_drive.h>
#include <flatland_plugins/fault_injection_registry.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/random.h>
#include <flatland_server/world.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace flatland_plugins {

namespace {
// Clamp the achieved per-step acceleration (new_velocity - velocity) / dt to
// +/- accel_cap, so the actuator effort limit modulates (further bounds) the
// DynamicsLimits ramp rather than adding a second independent ramp. An infinite
// cap (effort limit disabled) is a pass-through.
double CapAcceleration(double velocity, double new_velocity, double accel_cap,
                       double dt) {
  if (!std::isfinite(accel_cap) || dt <= 0.0) {
    return new_velocity;
  }
  double accel = (new_velocity - velocity) / dt;
  accel = DynamicsLimits::Saturate(accel, -accel_cap, accel_cap);
  return velocity + accel * dt;
}
}  // namespace

void DiffDrive::TwistCallback(const geometry_msgs::Twist& msg) {
  twist_msg_ = msg;
}

void DiffDrive::OnInitialize(const YAML::Node& config) {
  YamlReader reader(config);
  enable_odom_pub_ = reader.Get<bool>("enable_odom_pub", true);
  enable_odom_tf_pub_ = reader.Get<bool>("enable_odom_tf_pub", true);
  enable_twist_pub_ = reader.Get<bool>("enable_twist_pub", true);
  twist_in_local_frame_ = reader.Get<bool>("twist_in_local_frame", true);

  std::string body_name = reader.Get<std::string>("body");
  std::string odom_frame_id = reader.Get<std::string>("odom_frame_id", "odom");
  std::string ground_truth_frame_id =
      reader.Get<std::string>("ground_truth_frame_id", "map");

  std::string twist_topic = reader.Get<std::string>("twist_sub", "cmd_vel");
  std::string odom_topic =
      reader.Get<std::string>("odom_pub", "odometry/filtered");
  std::string ground_truth_topic =
      reader.Get<std::string>("ground_truth_pub", "odometry/ground_truth");
  std::string twist_pub_topic = reader.Get<std::string>("twist_pub", "twist");

  // noise are in the form of linear x, linear y, angular variances
  std::vector<double> odom_twist_noise =
      reader.GetList<double>("odom_twist_noise", {0, 0, 0}, 3, 3);
  std::vector<double> odom_pose_noise =
      reader.GetList<double>("odom_pose_noise", {0, 0, 0}, 3, 3);

  double pub_rate =
      reader.Get<double>("pub_rate", std::numeric_limits<double>::infinity());
  update_timer_.SetRate(pub_rate);

  // Angular dynamics constraints
  angular_dynamics_.Configure(reader.SubnodeOpt("angular_dynamics", YamlReader::MAP).Node());

  // Linear dynamics constraints
  linear_dynamics_.Configure(reader.SubnodeOpt("linear_dynamics", YamlReader::MAP).Node());

  // Actuator/motor dynamics (command latency, deadband, force/torque cap).
  // Opt-in: an absent or empty subnode leaves these as a pure pass-through so
  // existing worlds behave identically.
  linear_actuator_.Configure(
      reader.SubnodeOpt("linear_actuator", YamlReader::MAP).Node());
  angular_actuator_.Configure(
      reader.SubnodeOpt("angular_actuator", YamlReader::MAP).Node());

  // Drive mode: "kinematic" (default, ideal SetLinearVelocity) keeps existing
  // worlds unchanged; "friction"/"dynamic" engages the force-based traction
  // path bounded by the anisotropic wheel-ground friction model.
  std::string drive_mode = reader.Get<std::string>("drive_mode", "kinematic");
  use_friction_drive_ = (drive_mode == "friction" || drive_mode == "dynamic");
  if (!use_friction_drive_ && drive_mode != "kinematic") {
    throw YAMLException("Invalid drive_mode " + Q(drive_mode) +
                        ", supported modes are: kinematic, friction");
  }

  // Wheel-ground friction model (only used when drive_mode == friction)
  wheel_friction_.Configure(
      reader.SubnodeOpt("wheel_friction", YamlReader::MAP).Node());

  // Track width between the two drive wheels; required for friction traction
  // because diff_drive has no explicit wheel joints to infer geometry from.
  wheel_separation_ = reader.Get<double>("wheel_separation", 0.0);
  if (use_friction_drive_ && wheel_separation_ <= 0.0) {
    throw YAMLException(
        "drive_mode \"friction\" requires a positive \"wheel_separation\"");
  }

  // by default the covariance diagonal is the variance of actual noise
  // generated, non-diagonal elements are zero assuming the noises are
  // independent, we also don't care about linear z, angular x, and angular y
  std::array<double, 36> odom_pose_covar_default = {0};
  odom_pose_covar_default[0] = odom_pose_noise[0];
  odom_pose_covar_default[7] = odom_pose_noise[1];
  odom_pose_covar_default[35] = odom_pose_noise[2];

  std::array<double, 36> odom_twist_covar_default = {0};
  odom_twist_covar_default[0] = odom_twist_noise[0];
  odom_twist_covar_default[7] = odom_twist_noise[1];
  odom_twist_covar_default[35] = odom_twist_noise[2];

  auto odom_twist_covar = reader.GetArray<double, 36>("odom_twist_covariance",
                                                      odom_twist_covar_default);
  auto odom_pose_covar = reader.GetArray<double, 36>("odom_pose_covariance",
                                                     odom_pose_covar_default);

  reader.EnsureAccessedAllKeys();

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }

  // publish and subscribe to topics
  twist_sub_ = nh_.subscribe(twist_topic, 1, &DiffDrive::TwistCallback, this);
  if (enable_odom_pub_) {
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 1);
    ground_truth_pub_ =
        nh_.advertise<nav_msgs::Odometry>(ground_truth_topic, 1);
  }

  if (enable_twist_pub_) {
    twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>(
        twist_pub_topic, 1);
  }

  // init the values for the messages
  ground_truth_msg_.header.frame_id = ground_truth_frame_id;
  ground_truth_msg_.child_frame_id =
      tf::resolve("", GetModel()->NameSpaceTF(body_->name_));
  ground_truth_msg_.twist.covariance.fill(0);
  ground_truth_msg_.pose.covariance.fill(0);
  // Odometry message initially is similar to ground truth except for the
  // parent frame ID
  odom_msg_ = ground_truth_msg_;
  odom_msg_.header.frame_id = odom_frame_id;

  // copy from std::array to boost array
  for (unsigned int i = 0; i < 36; i++) {
    odom_msg_.twist.covariance[i] = odom_twist_covar[i];
    odom_msg_.pose.covariance[i] = odom_pose_covar[i];
  }

  // init the random number generators from the seeded RNG authority so noise is
  // reproducible for a given run seed (see flatland_server/random.h)
  rng_ = flatland_server::RngManager::Get().DeriveEngine(GetModel()->GetName() +
                                                         "/" + GetName());

  // Stable key the FaultInjectionRegistry uses to address this drivetrain.
  fault_key_ = ComponentKey(GetModel()->GetName(), GetName());

  for (unsigned int i = 0; i < 3; i++) {
    // variance is standard deviation squared
    noise_gen_[i] =
        std::normal_distribution<double>(0.0, sqrt(odom_pose_noise[i]));
  }

  for (unsigned int i = 0; i < 3; i++) {
    noise_gen_[i + 3] =
        std::normal_distribution<double>(0.0, sqrt(odom_twist_noise[i]));
  }

  ROS_DEBUG_NAMED("DiffDrive",
                  "Initialized with params body(%p %s) odom_frame_id(%s) "
                  "twist_sub(%s) odom_pub(%s) ground_truth_pub(%s) "
                  "odom_pose_noise({%f,%f,%f}) odom_twist_noise({%f,%f,%f}) "
                  "pub_rate(%f)\n",
                  body_, body_->name_.c_str(), odom_frame_id.c_str(),
                  twist_topic.c_str(), odom_topic.c_str(),
                  ground_truth_topic.c_str(), odom_pose_noise[0],
                  odom_pose_noise[1], odom_pose_noise[2], odom_twist_noise[0],
                  odom_twist_noise[1], odom_twist_noise[2], pub_rate);
}

void DiffDrive::BeforePhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);

  b2Body* b2body = body_->physics_body_;

  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();

  // Apply dynamics limits
  double dt = timekeeper.GetStepSize();
  const ros::Time& now = timekeeper.GetSimTime();

  // Actuator/motor dynamics pipeline, driven by a single sim-time source: push
  // the cached command into each axis' latency buffer, pull the latency-delayed
  // command, then apply the deadband before the existing acceleration ramp.
  linear_actuator_.Push(twist_msg_.linear.x, now);
  angular_actuator_.Push(twist_msg_.angular.z, now);
  double linear_cmd =
      linear_actuator_.ApplyDeadband(linear_actuator_.Pull(now));
  double angular_cmd =
      angular_actuator_.ApplyDeadband(angular_actuator_.Pull(now));

  // Acceleration ramp (existing behaviour), then modulate it with the actuator
  // effort limit: the force/torque cap bounds the achievable per-step
  // acceleration (a_max = F/m, alpha_max = tau/I). Mass and rotational inertia
  // are re-read from the Box2D body each step so the variable-mass payload
  // model is respected.
  linear_velocity_ =
      CapAcceleration(linear_velocity_,
                      linear_dynamics_.Limit(linear_velocity_, linear_cmd, dt),
                      linear_actuator_.AccelerationCap(b2body->GetMass()), dt);
  angular_velocity_ = CapAcceleration(
      angular_velocity_,
      angular_dynamics_.Limit(angular_velocity_, angular_cmd, dt),
      angular_actuator_.AccelerationCap(b2body->GetInertia()), dt);

  // Drivetrain fault perturbations: applied to the (already limited) commanded
  // velocities BEFORE they reach Box2D, so the resulting motion / odom / tf
  // reflect the fault causally rather than as a cosmetic number edit. No active
  // effect -> velocities are unchanged.
  {
    auto& reg = FaultInjectionRegistry::Get();
    double loss = 0.0;
    FaultEffect tl = reg.GetEffect(fault_key_, FaultKind::kTorqueLoss);
    if (tl.active) loss = std::max(loss, tl.severity * tl.Param("loss", 1.0));
    FaultEffect ws = reg.GetEffect(fault_key_, FaultKind::kWheelSlip);
    if (ws.active) loss = std::max(loss, ws.severity * ws.Param("loss", 1.0));
    FaultEffect sw = reg.GetEffect(fault_key_, FaultKind::kStuckWheel);
    if (sw.active) loss = std::max(loss, sw.severity);
    double factor = 1.0 - DynamicsLimits::Saturate(loss, 0.0, 1.0);
    linear_velocity_ *= factor;
    angular_velocity_ *= factor;

    FaultEffect asym = reg.GetEffect(fault_key_, FaultKind::kAsymmetricDrive);
    if (asym.active) {
      angular_velocity_ += asym.severity * asym.Param("yaw_bias");
    }

    FaultEffect db = reg.GetEffect(fault_key_, FaultKind::kDeadband);
    if (db.active) {
      double thr = db.severity * db.Param("deadband");
      if (std::fabs(linear_velocity_) < thr) linear_velocity_ = 0.0;
    }
  }

  if (use_friction_drive_) {
    // Force-based traction: let the Box2D solver integrate the body under the
    // slip-limited friction forces at each wheel instead of overwriting the
    // velocity. This lets commanded motion exceed grip and produce real slip.
    ApplyFrictionDrive(b2body, dt);
  } else {
    // we apply the twist velocities, this must be done every physics step to
    // make sure Box2D solver applies the correct velocity through out. The
    // velocity given in the twist message should be in the local frame
    b2Vec2 linear_vel_local(linear_velocity_, 0);
    b2Vec2 linear_vel = b2body->GetWorldVector(linear_vel_local);
    float angular_vel = angular_velocity_;  // angular is independent of frames

    // we want the velocity vector in the world frame at the center of mass

    // V_cm = V_o + W x r_cm/o
    // velocity at center of mass equals to the velocity at the body origin
    // plus, angular velocity cross product the displacement from the body
    // origin to the center of mass

    // r is the vector from body origin to the CM in world frame
    b2Vec2 r = b2body->GetWorldCenter() - position;
    b2Vec2 linear_vel_cm = linear_vel + angular_vel * b2Vec2(-r.y, r.x);

    b2body->SetLinearVelocity(linear_vel_cm);
    b2body->SetAngularVelocity(angular_vel);
  }

  // Update odom+ground truth messages if needed

  if (publish) {
    // get the state of the body and publish the data
    b2Vec2 linear_vel_local =
        b2body->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0));
    float angular_vel = b2body->GetAngularVelocity();

    ground_truth_msg_.header.stamp = timekeeper.GetSimTime();
    ground_truth_msg_.pose.pose.position.x = position.x;
    ground_truth_msg_.pose.pose.position.y = position.y;
    ground_truth_msg_.pose.pose.position.z = 0;
    ground_truth_msg_.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(angle);

    ground_truth_msg_.twist.twist.linear.z = 0;
    ground_truth_msg_.twist.twist.angular.x = 0;
    ground_truth_msg_.twist.twist.angular.y = 0;
    if (twist_in_local_frame_) {
      // change frame of velocity
      ground_truth_msg_.twist.twist.linear.x =
          cos(-angle) * linear_vel_local.x - sin(-angle) * linear_vel_local.y;
      ground_truth_msg_.twist.twist.linear.y =
          sin(-angle) * linear_vel_local.x + cos(-angle) * linear_vel_local.y;
      ground_truth_msg_.twist.twist.angular.z = angular_vel;
    } else {
      ground_truth_msg_.twist.twist.linear.x = linear_vel_local.x;
      ground_truth_msg_.twist.twist.linear.y = linear_vel_local.y;
      ground_truth_msg_.twist.twist.angular.z = angular_vel;
    }

    // Measurement-domain odometry faults: encoder_drift and odom_slip perturb
    // the REPORTED odom (pose, odom tf, encoder-like twist) only; the true
    // Box2D body state in ground_truth_msg_ is left untouched -- the opposite
    // of the causal drivetrain faults above. Until one of these faults first
    // goes active we copy the ground-truth pose verbatim, so a clean run is
    // byte-for-byte identical to before. Once diverged we dead-reckon the
    // integrated truth delta with slip/drift applied; the error persists after
    // the fault window closes (a real odometry error does not heal itself).
    auto& reg = FaultInjectionRegistry::Get();
    FaultEffect slip = reg.GetEffect(fault_key_, FaultKind::kOdomSlip);
    FaultEffect edrift = reg.GetEffect(fault_key_, FaultKind::kEncoderDrift);
    double slip_factor = 1.0;  // multiplies the reported translation/twist

    bool just_latched = false;
    if (!odom_diverged_ && (slip.active || edrift.active)) {
      // First onset: seed the dead-reckoned pose from the current ground truth
      // so the report is continuous at the instant the fault begins.
      odom_diverged_ = true;
      odom_x_ = position.x;
      odom_y_ = position.y;
      odom_yaw_ = angle;
      just_latched = true;
    }
    if (odom_diverged_ && !just_latched) {
      if (slip.active) {
        // >0 over-reports distance (wheel slips, encoder over-counts), <0
        // under-reports.
        slip_factor = 1.0 + slip.severity * slip.Param("slip");
      }
      double dx = gt_sample_valid_ ? (position.x - last_gt_x_) : 0.0;
      double dy = gt_sample_valid_ ? (position.y - last_gt_y_) : 0.0;
      double dyaw = gt_sample_valid_ ? (angle - last_gt_angle_) : 0.0;
      odom_x_ += slip_factor * dx;
      odom_y_ += slip_factor * dy;
      odom_yaw_ += dyaw;
      if (edrift.active) {
        // Unbounded per-axis accumulation at a per-second rate (severity 1).
        odom_x_ += edrift.severity * edrift.Param("x_rate") * dt;
        odom_y_ += edrift.severity * edrift.Param("y_rate") * dt;
        odom_yaw_ += edrift.severity * edrift.Param("yaw_rate") * dt;
      }
    }
    last_gt_x_ = position.x;
    last_gt_y_ = position.y;
    last_gt_angle_ = angle;
    gt_sample_valid_ = true;

    // Reported base pose: the dead-reckoned pose when diverged, otherwise the
    // ground truth verbatim (clean-run invariant).
    double report_x = odom_diverged_ ? odom_x_ : position.x;
    double report_y = odom_diverged_ ? odom_y_ : position.y;
    double report_yaw = odom_diverged_ ? odom_yaw_ : angle;

    // add the noise to odom messages
    odom_msg_.header.stamp = timekeeper.GetSimTime();
    odom_msg_.pose.pose = ground_truth_msg_.pose.pose;
    odom_msg_.twist.twist = ground_truth_msg_.twist.twist;
    odom_msg_.pose.pose.position.x = report_x + noise_gen_[0](rng_);
    odom_msg_.pose.pose.position.y = report_y + noise_gen_[1](rng_);
    odom_msg_.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(report_yaw + noise_gen_[2](rng_));
    odom_msg_.twist.twist.linear.x =
        slip_factor * odom_msg_.twist.twist.linear.x + noise_gen_[3](rng_);
    odom_msg_.twist.twist.linear.y =
        slip_factor * odom_msg_.twist.twist.linear.y + noise_gen_[4](rng_);
    odom_msg_.twist.twist.angular.z += noise_gen_[5](rng_);

    if (enable_odom_pub_) {
      ground_truth_pub_.publish(ground_truth_msg_);
      odom_pub_.publish(odom_msg_);
    }

    if (enable_twist_pub_) {
      // Transform global frame velocity into local frame to simulate encoder
      // readings
      geometry_msgs::TwistWithCovarianceStamped twist_pub_msg;
      twist_pub_msg.header.stamp = timekeeper.GetSimTime();
      twist_pub_msg.header.frame_id = odom_msg_.child_frame_id;

      // Forward velocity in twist.linear.x. odom_slip scales the encoder-like
      // velocity to match the over/under-reported odom pose (encoder_drift is a
      // pose accumulation only and does not appear here).
      twist_pub_msg.twist.twist.linear.x =
          slip_factor * (cos(angle) * linear_vel_local.x +
                         sin(angle) * linear_vel_local.y) +
          noise_gen_[3](rng_);

      // Angular velocity in twist.angular.z
      twist_pub_msg.twist.twist.angular.z = angular_vel + noise_gen_[5](rng_);

      twist_pub_msg.twist.covariance = odom_msg_.twist.covariance;

      twist_pub_.publish(twist_pub_msg);
    }

    if (enable_odom_tf_pub_) {
      // publish odom tf
      geometry_msgs::TransformStamped odom_tf;
      odom_tf.header = odom_msg_.header;
      odom_tf.child_frame_id = odom_msg_.child_frame_id;
      odom_tf.transform.translation.x = odom_msg_.pose.pose.position.x;
      odom_tf.transform.translation.y = odom_msg_.pose.pose.position.y;
      odom_tf.transform.translation.z = 0;
      odom_tf.transform.rotation = odom_msg_.pose.pose.orientation;
      tf_broadcaster.sendTransform(odom_tf);
    }
  }

}

void DiffDrive::ApplyFrictionDrive(b2Body* b2body, double dt) {
  // Two drive wheels straddle the body x-axis at +/- half the track width.
  double half_track = 0.5 * wheel_separation_;

  // Normal load distributed evenly across the two drive wheels (simple mass*g
  // split; z-load / CoG shifts are deferred to the 2.5D contact backlog task).
  double normal_load = 0.5 * b2body->GetMass() * WheelFrictionModel::kGravity;

  // left wheel at +y, right wheel at -y in the body frame
  const double wheel_y[2] = {half_track, -half_track};

  // World owning this model, used to look up the surface friction multiplier
  // under each wheel. Null only in detached unit setups; treat as nominal grip.
  flatland_server::World* world = GetModel()->GetWorld();

  for (int i = 0; i < 2; i++) {
    b2Vec2 wheel_local(0.0f, static_cast<float>(wheel_y[i]));

    // Commanded contact-patch velocity from the limited twist: the no-slip body
    // velocity Box2D would see at this wheel. Longitudinal only (= forward); the
    // wheel is not driven sideways so its commanded lateral velocity is zero.
    // v_x at the wheel = v - w * y_wheel (from omega x r).
    double commanded_long = linear_velocity_ - angular_velocity_ * wheel_y[i];

    // Actual body velocity at the contact point, expressed in the body/wheel
    // frame (diff-drive wheels point along body x).
    b2Vec2 vel_world = b2body->GetLinearVelocityFromLocalPoint(wheel_local);
    b2Vec2 vel_local = b2body->GetLocalVector(vel_world);

    double slip_long = commanded_long - vel_local.x;
    double slip_lat = 0.0 - vel_local.y;

    // Surface friction multiplier sampled at this wheel's world contact point,
    // so traction drops smoothly over wet patches/spills and recovers on dry
    // ground. Per-wheel (not body-centre) so a robot straddling a boundary
    // gets the correct differential grip.
    b2Vec2 wheel_world = b2body->GetWorldPoint(wheel_local);
    double surface_factor =
        world ? world->GetSurfaceFrictionFactor(wheel_world) : 1.0;

    b2Vec2 force_local = wheel_friction_.ComputeWheelForce(
        slip_long, slip_lat, normal_load, dt, surface_factor);

    // Cap the per-wheel longitudinal (motor-driven) traction to the actuator
    // force limit before applying it: the effective traction is the lesser of
    // available grip and motor force. The lateral component is passive grip
    // (not motor-driven), so it is left to the friction model. The total
    // drivetrain force is split evenly across the two drive wheels.
    double motor_cap = linear_actuator_.ForceCap();
    if (motor_cap > 0.0) {
      double per_wheel_cap = 0.5 * motor_cap;
      force_local.x = static_cast<float>(DynamicsLimits::Saturate(
          force_local.x, -per_wheel_cap, per_wheel_cap));
    }

    b2Vec2 force_world = b2body->GetWorldVector(force_local);
    b2body->ApplyForce(force_world, wheel_world, true);
  }
}
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::DiffDrive,
                       flatland_server::ModelPlugin)
