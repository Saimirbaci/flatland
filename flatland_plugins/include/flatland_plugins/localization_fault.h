/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	localization_fault.h
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

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <string>

#ifndef FLATLAND_PLUGINS_LOCALIZATION_FAULT_H
#define FLATLAND_PLUGINS_LOCALIZATION_FAULT_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * @class LocalizationFault
 * @brief Stands in for an AMCL/particle-filter localization node.
 *
 * Flatland ships no localization stack, so this model plugin synthesizes the
 * two outputs an RCA agent expects to see from one: a `amcl_pose`
 * (geometry_msgs/PoseWithCovarianceStamped, in the map frame) and the
 * map->odom tf. In a clean run the estimate equals the body's true world pose
 * and map->odom is identity (flatland's diff_drive/tricycle odom is anchored at
 * the spawn/world origin, so the odom frame coincides with the map frame).
 *
 * Under an injected `amcl_divergence` fault (FaultKind::kAmclDivergence) the
 * published estimate is offset from the truth by severity-scaled `x`, `y`, and
 * `yaw` params, and the map->odom tf is recomputed as estimate * truth^-1 so
 * the localization estimate diverges from the (untouched) odom. The severity
 * ramp gives the canonical "divergence grows then holds" behavior.
 */
class LocalizationFault : public flatland_server::ModelPlugin {
 public:
  Body* body_;  ///< body whose true pose is the localization reference

  std::string map_frame_id_;   ///< parent frame of the estimate (default map)
  std::string odom_frame_id_;  ///< child frame of the map->odom tf
  bool publish_tf_;            ///< whether to broadcast the map->odom tf

  ros::Publisher amcl_pose_pub_;  ///< synthetic localization estimate
  geometry_msgs::PoseWithCovarianceStamped amcl_pose_msg_;
  tf::TransformBroadcaster tf_broadcaster_;

  UpdateTimer update_timer_;

  std::string fault_key_;  ///< registry component key (model/plugin)

  /**
   * @name      OnInitialize
   * @brief     Parse config and set up publishers/frames
   * @param[in] config The plugin YAML node
   */
  void OnInitialize(const YAML::Node& config) override;

  /**
   * @name      AfterPhysicsStep
   * @brief     Publish the (possibly diverged) estimate + map->odom tf
   * @param[in] timekeeper The sim-time source
   */
  void AfterPhysicsStep(const Timekeeper& timekeeper) override;
};
}  // namespace flatland_plugins

#endif
