/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	variable_payload.h
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

#ifndef FLATLAND_PLUGINS_VARIABLE_PAYLOAD_H
#define FLATLAND_PLUGINS_VARIABLE_PAYLOAD_H

#include <Box2D/Box2D.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

namespace flatland_plugins {

/**
 * VariablePayload models a body whose payload mass and center of gravity change
 * over the simulation run, so the model's handling (inertia, how it responds to
 * drive forces and external pushes) changes with its load. The canonical use
 * case is an AGV cleaning robot whose water tank drains during operation.
 *
 * The body keeps its fixture-derived (or configured) empty mass; on top of that
 * a payload of up to max_payload_mass_ is added at payload_center_ (body local
 * frame), scaled by a fill fraction in [0, 1]. The fill fraction can evolve at
 * a constant rate (e.g. a draining tank) and/or be commanded at runtime over a
 * std_msgs/Float64 topic. Each step the combined mass, center of gravity, and
 * rotational inertia are applied through the engine-level Body::SetMassData
 * hook.
 */
class VariablePayload : public flatland_server::ModelPlugin {
 public:
  flatland_server::Body *body_;  ///< body whose mass/CoG this plugin drives

  double base_mass_;         ///< empty-body mass (kg)
  b2Vec2 base_center_;       ///< empty-body CoG, body local frame (m)
  double base_inertia_com_;  ///< empty-body inertia about its own CoM (kg*m^2)

  double max_payload_mass_;  ///< payload mass at fill == 1.0 (kg)
  b2Vec2 payload_center_;    ///< payload CoG, body local frame (m)

  double fill_;       ///< current fill fraction [0, 1]
  double fill_rate_;  ///< constant fill change rate (fraction / s)

  ros::Subscriber fill_sub_;  ///< optional fill-fraction command topic
  ros::Publisher state_pub_;  ///< optional [mass, cx, cy] state topic
  bool publish_state_;        ///< whether state_pub_ is active

  double last_applied_fill_;  ///< guard so we skip redundant SetMassData

  /**
   * @name      OnInitialize
   * @brief     Parse config, resolve the target body, capture its empty mass.
   * @param[in] config The plugin YAML node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @name      BeforePhysicsStep
   * @brief     Integrate the fill profile and apply the resulting mass / CoG.
   * @param[in] timekeeper Simulation clock (provides the step size)
   */
  void BeforePhysicsStep(
      const flatland_server::Timekeeper &timekeeper) override;

  /**
   * @name      FillCallback
   * @brief     Handle an external fill-fraction command (clamped to [0, 1]).
   * @param[in] msg std_msgs/Float64 fill fraction
   */
  void FillCallback(const std_msgs::Float64 &msg);

  /**
   * @name      ApplyMass
   * @brief     Compute combined mass, CoG and inertia for the given fill and
   *            push them onto the body via the engine hook; optionally publish.
   * @param[in] fill Fill fraction in [0, 1]
   */
  void ApplyMass(double fill);
};
}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_VARIABLE_PAYLOAD_H
