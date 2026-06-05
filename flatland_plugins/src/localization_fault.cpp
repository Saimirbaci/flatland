/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	localization_fault.cpp
 * @brief	Model plugin: synthetic AMCL pose estimate + map->odom tf, with
 *          injectable localization divergence
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

#include <Box2D/Box2D.h>
#include <flatland_plugins/fault_injection_registry.h>
#include <flatland_plugins/localization_fault.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <cmath>
#include <limits>
#include <string>

namespace flatland_plugins {

void LocalizationFault::OnInitialize(const YAML::Node& config) {
  YamlReader reader(config);

  std::string body_name = reader.Get<std::string>("body");
  map_frame_id_ = reader.Get<std::string>("map_frame_id", "map");
  odom_frame_id_ = reader.Get<std::string>("odom_frame_id", "odom");
  publish_tf_ = reader.Get<bool>("publish_tf", true);

  std::string amcl_pose_topic =
      reader.Get<std::string>("amcl_pose_pub", "amcl_pose");

  double update_rate =
      reader.Get<double>("update_rate", std::numeric_limits<double>::infinity());
  update_timer_.SetRate(update_rate);

  reader.EnsureAccessedAllKeys();

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }

  amcl_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      amcl_pose_topic, 1);

  amcl_pose_msg_.header.frame_id = map_frame_id_;
  // Modest fixed covariance: this is a synthetic estimate, not a real filter.
  amcl_pose_msg_.pose.covariance.fill(0.0);
  amcl_pose_msg_.pose.covariance[0] = 0.05;   // x
  amcl_pose_msg_.pose.covariance[7] = 0.05;   // y
  amcl_pose_msg_.pose.covariance[35] = 0.02;  // yaw

  // Stable key the FaultInjectionRegistry uses to address this component.
  fault_key_ = ComponentKey(GetModel()->GetName(), GetName());

  ROS_DEBUG_NAMED("LocalizationFault",
                  "Initialized with body(%p %s) map_frame_id(%s) "
                  "odom_frame_id(%s) amcl_pose_pub(%s) update_rate(%f)\n",
                  body_, body_->name_.c_str(), map_frame_id_.c_str(),
                  odom_frame_id_.c_str(), amcl_pose_topic.c_str(), update_rate);
}

void LocalizationFault::AfterPhysicsStep(const Timekeeper& timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  b2Body* b2body = body_->physics_body_;
  b2Vec2 position = b2body->GetPosition();
  double truth_x = position.x;
  double truth_y = position.y;
  double truth_yaw = b2body->GetAngle();

  // AMCL-divergence fault: offset the published estimate from the true pose by
  // severity-scaled params. No active effect -> estimate == truth and the
  // map->odom tf is identity (clean-run invariant). The fault perturbs only
  // this localization estimate; the drive's odom is untouched.
  double off_x = 0.0;
  double off_y = 0.0;
  double off_yaw = 0.0;
  FaultEffect div = FaultInjectionRegistry::Get().GetEffect(
      fault_key_, FaultKind::kAmclDivergence);
  if (div.active) {
    off_x = div.severity * div.Param("x");
    off_y = div.severity * div.Param("y");
    off_yaw = div.severity * div.Param("yaw");
  }

  double est_x = truth_x + off_x;
  double est_y = truth_y + off_y;
  double est_yaw = truth_yaw + off_yaw;

  ros::Time stamp = timekeeper.GetSimTime();

  // Publish the (possibly diverged) localization estimate in the map frame.
  amcl_pose_msg_.header.stamp = stamp;
  amcl_pose_msg_.pose.pose.position.x = est_x;
  amcl_pose_msg_.pose.pose.position.y = est_y;
  amcl_pose_msg_.pose.pose.position.z = 0.0;
  amcl_pose_msg_.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(est_yaw);
  amcl_pose_pub_.publish(amcl_pose_msg_);

  if (publish_tf_) {
    // map->odom = (map->base) * (odom->base)^-1, where map->base is the
    // estimate and odom->base is the truth (clean odom is anchored at the
    // world origin, so it equals the body's true world pose). When the estimate
    // equals the truth this composes to identity. As an SE(2) product:
    //   yaw_mo = off_yaw
    //   p_mo   = est - R(off_yaw) * truth
    double c = std::cos(off_yaw);
    double s = std::sin(off_yaw);
    double tx_x = est_x - (c * truth_x - s * truth_y);
    double tx_y = est_y - (s * truth_x + c * truth_y);

    geometry_msgs::TransformStamped map_odom_tf;
    map_odom_tf.header.stamp = stamp;
    map_odom_tf.header.frame_id = map_frame_id_;
    map_odom_tf.child_frame_id = odom_frame_id_;
    map_odom_tf.transform.translation.x = tx_x;
    map_odom_tf.transform.translation.y = tx_y;
    map_odom_tf.transform.translation.z = 0.0;
    map_odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(off_yaw);
    tf_broadcaster_.sendTransform(map_odom_tf);
  }
}
}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::LocalizationFault,
                       flatland_server::ModelPlugin)
