/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	trajectory_recorder.cpp
 * @brief   Record a model's actual pose into a Path and republish a planned
 *          Path for side-by-side diagnostic comparison
 * @author  Saimir Baci
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

#include <flatland_plugins/trajectory_recorder.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <cmath>
#include <limits>

using namespace flatland_server;

namespace flatland_plugins {

void TrajectoryRecorder::OnInitialize(const YAML::Node& config) {
  YamlReader reader(config);

  // Default to the first body in the model when none is named.
  std::string body_name = reader.Get<std::string>("body", "");
  if (body_name.empty()) {
    const auto& bodies = GetModel()->GetBodies();
    if (bodies.empty()) {
      throw YAMLException("TrajectoryRecorder: model has no bodies to record");
    }
    body_ = bodies.front();
  } else {
    body_ = GetModel()->GetBody(body_name);
    if (body_ == nullptr) {
      throw YAMLException("Body with name " + Q(body_name) + " does not exist");
    }
  }

  frame_id_ = reader.Get<std::string>("frame_id", "map");

  int configured_max = reader.Get<int>("max_length", 1000);
  if (configured_max < 1) {
    throw YAMLException(Q("max_length") + " must be >= 1");
  }
  max_length_ = static_cast<size_t>(configured_max);

  double update_rate = reader.Get<double>(
      "update_rate", std::numeric_limits<double>::infinity());
  update_timer_.SetRate(update_rate);

  std::string actual_topic =
      reader.Get<std::string>("actual_topic", "trajectory/actual");
  std::string planned_topic =
      reader.Get<std::string>("planned_topic", "trajectory/planned");
  // Optional external planned path (e.g. from move_base); only subscribed and
  // republished when configured.
  std::string planned_input_topic =
      reader.Get<std::string>("planned_input_topic", "");

  reader.EnsureAccessedAllKeys();

  actual_path_.header.frame_id = frame_id_;

  actual_pub_ = nh_.advertise<nav_msgs::Path>(
      GetModel()->NameSpaceTopic(actual_topic), 1);
  planned_pub_ = nh_.advertise<nav_msgs::Path>(
      GetModel()->NameSpaceTopic(planned_topic), 1, /* latch = */ true);

  if (!planned_input_topic.empty()) {
    planned_sub_ =
        nh_.subscribe(GetModel()->NameSpaceTopic(planned_input_topic), 1,
                      &TrajectoryRecorder::PlannedPathCallback, this);
  }

  ROS_DEBUG_NAMED("TrajectoryRecorder",
                  "Initialized body(%s) frame(%s) max_length(%zu) "
                  "update_rate(%f)",
                  body_->name_.c_str(), frame_id_.c_str(), max_length_,
                  update_rate);
}

void TrajectoryRecorder::BeforePhysicsStep(const Timekeeper& timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  const b2Vec2 position = body_->physics_body_->GetPosition();
  const double angle = body_->physics_body_->GetAngle();

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame_id_;
  pose.header.stamp = timekeeper.GetSimTime();
  pose.pose.position.x = position.x;
  pose.pose.position.y = position.y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.z = std::sin(angle / 2.0);
  pose.pose.orientation.w = std::cos(angle / 2.0);

  actual_path_.poses.push_back(pose);
  // Bound memory: drop the oldest pose once the cap is exceeded. max_length_ is
  // small (default 1000), so the single front-erase per step is negligible.
  if (actual_path_.poses.size() > max_length_) {
    actual_path_.poses.erase(actual_path_.poses.begin());
  }

  actual_path_.header.stamp = timekeeper.GetSimTime();
  actual_pub_.publish(actual_path_);
}

void TrajectoryRecorder::PlannedPathCallback(const nav_msgs::Path& msg) {
  // Re-stamp into the sim world frame so the planned path overlays correctly
  // even if the planner published it in its own frame. Poses are passed through
  // unchanged (no tf transform here: callers are expected to plan in the world
  // frame; the frame relabel keeps rviz from dropping the display).
  nav_msgs::Path planned = msg;
  planned.header.frame_id = frame_id_;
  for (auto& pose : planned.poses) {
    pose.header.frame_id = frame_id_;
  }
  planned_pub_.publish(planned);
}
}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::TrajectoryRecorder,
                       flatland_server::ModelPlugin)
