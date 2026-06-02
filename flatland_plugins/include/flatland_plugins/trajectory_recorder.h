/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	trajectory_recorder.h
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

#ifndef FLATLAND_PLUGINS_TRAJECTORY_RECORDER_H
#define FLATLAND_PLUGINS_TRAJECTORY_RECORDER_H

#include <flatland_plugins/update_timer.h>
#include <flatland_server/body.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <string>

namespace flatland_plugins {

/**
 * TrajectoryRecorder accumulates a model body's actual pose over the run into a
 * nav_msgs/Path (the "actual" trajectory) and, optionally, re-stamps and
 * republishes an externally supplied planned path (e.g. from a global planner)
 * into the simulation world frame (the "planned" trajectory). The two paths are
 * designed to be rendered as two distinctly-coloured rviz Path displays in the
 * flatland_viz Diagnostic Layers panel for planned-vs-actual comparison.
 *
 * The actual path is appended on the simulation clock at a configurable rate
 * and capped at max_length_ poses (oldest dropped) so memory stays bounded over
 * long simulations. The pose is read from the body's Box2D transform, so it
 * reflects the true simulated motion (it does not depend on any odometry
 * plugin). All work happens in BeforePhysicsStep / a subscriber callback; the
 * plugin never blocks the step.
 */
class TrajectoryRecorder : public flatland_server::ModelPlugin {
 public:
  flatland_server::Body* body_;  ///< body whose pose is recorded

  std::string frame_id_;  ///< world frame both paths are published in
  size_t max_length_;     ///< cap on the actual path length (poses)

  ros::Publisher actual_pub_;    ///< actual trajectory (nav_msgs/Path)
  ros::Publisher planned_pub_;   ///< re-stamped planned trajectory
  ros::Subscriber planned_sub_;  ///< optional external planned-path input

  nav_msgs::Path actual_path_;  ///< accumulated actual trajectory

  UpdateTimer
      update_timer_;  ///< throttles pose sampling to the configured rate

  /**
   * @name      OnInitialize
   * @brief     Parse config, resolve the recorded body, advertise the paths and
   *            (optionally) subscribe to an external planned-path topic.
   * @param[in] config The plugin YAML node
   */
  void OnInitialize(const YAML::Node& config) override;

  /**
   * @name      BeforePhysicsStep
   * @brief     Sample the body pose (rate-limited), append it to the actual
   *            path (length-capped) and publish.
   * @param[in] timekeeper Simulation clock (sim time + step size)
   */
  void BeforePhysicsStep(
      const flatland_server::Timekeeper& timekeeper) override;

  /**
   * @name      PlannedPathCallback
   * @brief     Re-stamp an incoming planned path into frame_id_ and republish
   * it on the planned topic so it overlays in the sim world frame.
   * @param[in] msg The externally supplied planned path
   */
  void PlannedPathCallback(const nav_msgs::Path& msg);
};
}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_TRAJECTORY_RECORDER_H
