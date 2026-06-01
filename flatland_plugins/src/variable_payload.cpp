/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	variable_payload.cpp
 * @brief   Time-varying mass / center-of-gravity payload model plugin
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

#include <flatland_plugins/variable_payload.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

using namespace flatland_server;

namespace flatland_plugins {

void VariablePayload::OnInitialize(const YAML::Node &config) {
  YamlReader reader(config);

  std::string body_name = reader.Get<std::string>("body");
  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }

  double configured_base = reader.Get<double>("base_mass", 0.0);
  max_payload_mass_ = reader.Get<double>("max_payload_mass");
  Vec2 payload_center = reader.GetVec2("payload_center", Vec2(0, 0));
  payload_center_ = b2Vec2(payload_center.x, payload_center.y);

  double initial_fill = reader.Get<double>("initial_fill", 1.0);
  fill_ = std::max(0.0, std::min(1.0, initial_fill));
  fill_rate_ = reader.Get<double>("fill_rate", 0.0);

  std::string fill_topic = reader.Get<std::string>("fill_topic", "");
  std::string state_topic = reader.Get<std::string>("state_topic", "");

  if (max_payload_mass_ < 0.0) {
    throw YAMLException(Q("max_payload_mass") + " must be non-negative");
  }

  // Capture the fixture-derived (density * area) mass properties before we ever
  // override them. b2MassData::I is the rotational inertia about the body local
  // origin; convert it to the inertia about the empty body's own CoM so we can
  // later shift it with the parallel-axis theorem as the CoG moves.
  b2MassData fixture = body_->GetMassData();
  double fixture_center_sq = b2Dot(fixture.center, fixture.center);
  double fixture_inertia_com = fixture.I - fixture.mass * fixture_center_sq;

  if (configured_base > 0.0) {
    // Honour the explicit empty mass but keep the fixture geometry's CoG, and
    // scale its inertia by the mass ratio (same shape, different mass).
    base_mass_ = configured_base;
    base_center_ = fixture.center;
    if (fixture.mass > 0.0) {
      base_inertia_com_ = fixture_inertia_com * (base_mass_ / fixture.mass);
    } else {
      base_inertia_com_ = 0.0;
    }
  } else {
    // Derive the empty mass straight from the fixtures.
    base_mass_ = fixture.mass;
    base_center_ = fixture.center;
    base_inertia_com_ = fixture_inertia_com;
  }

  if (fill_topic != "") {
    fill_sub_ =
        nh_.subscribe(fill_topic, 1, &VariablePayload::FillCallback, this);
  }

  publish_state_ = (state_topic != "");
  if (publish_state_) {
    state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(state_topic, 1);
  }

  // Apply the initial load immediately so the body starts with the right mass.
  last_applied_fill_ = -1.0;  // force the first ApplyMass below to run
  ApplyMass(fill_);

  ROS_DEBUG_NAMED("VariablePayload",
                  "Initialized body(%s) base_mass(%f) max_payload(%f) "
                  "payload_center(%f, %f) initial_fill(%f) fill_rate(%f)",
                  body_->name_.c_str(), base_mass_, max_payload_mass_,
                  payload_center_.x, payload_center_.y, fill_, fill_rate_);

  reader.EnsureAccessedAllKeys();
}

void VariablePayload::FillCallback(const std_msgs::Float64 &msg) {
  // Only stash the commanded value here; the mass is applied in the step loop.
  fill_ = std::max(0.0, std::min(1.0, msg.data));
}

void VariablePayload::ApplyMass(double fill) {
  double payload_mass = fill * max_payload_mass_;
  double mass = base_mass_ + payload_mass;

  // A massless dynamic body is meaningless; if there is nothing to apply (no
  // base mass and an empty payload), leave Box2D's state untouched.
  if (mass <= 0.0) {
    last_applied_fill_ = fill;
    return;
  }

  // Combined center of gravity (mass-weighted) in the body local frame.
  b2Vec2 weighted = base_mass_ * base_center_ + payload_mass * payload_center_;
  b2Vec2 center(weighted.x / mass, weighted.y / mass);

  // Inertia about the combined CoM via the parallel-axis theorem. The base
  // keeps its own inertia shifted to the new CoM; the payload is treated as a
  // point mass (no intrinsic inertia) at payload_center_.
  b2Vec2 base_arm = base_center_ - center;
  b2Vec2 payload_arm = payload_center_ - center;
  double inertia_com = base_inertia_com_ +
                       base_mass_ * b2Dot(base_arm, base_arm) +
                       payload_mass * b2Dot(payload_arm, payload_arm);

  // Box2D wants the inertia about the body local origin.
  double inertia_origin = inertia_com + mass * b2Dot(center, center);

  body_->SetMassData(mass, center, inertia_origin);
  last_applied_fill_ = fill;

  if (publish_state_) {
    std_msgs::Float64MultiArray state;
    state.data = {mass, center.x, center.y};
    state_pub_.publish(state);
  }
}

void VariablePayload::BeforePhysicsStep(const Timekeeper &timekeeper) {
  // Integrate the constant fill profile on the simulation clock.
  if (fill_rate_ != 0.0) {
    fill_ += fill_rate_ * timekeeper.GetStepSize();
    fill_ = std::max(0.0, std::min(1.0, fill_));
  }

  // Skip the (cheap but non-trivial) mass recompute when nothing changed.
  if (std::fabs(fill_ - last_applied_fill_) > 1e-9) {
    ApplyMass(fill_);
  }
}
}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::VariablePayload,
                       flatland_server::ModelPlugin)
