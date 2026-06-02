/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	simulation_manager.cpp
 * @brief	Simulation manager runs the physics+event loop
 * @author Joseph Duchesne
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

#include "flatland_server/simulation_manager.h"

#include <flatland_server/debug_visualization.h>
#include <flatland_server/layer.h>
#include <flatland_server/model.h>
#include <flatland_server/service_manager.h>
#include <flatland_server/world.h>
#include <ros/ros.h>

#include <chrono>
#include <exception>
#include <limits>
#include <string>
#include <thread>

namespace flatland_server {

SimulationManager::SimulationManager(std::string world_yaml_file,
                                     double update_rate, double step_size,
                                     bool show_viz, double viz_pub_rate,
                                     int seed, double real_time_factor)
    : world_(nullptr),
      update_rate_(update_rate),
      step_size_(step_size),
      show_viz_(show_viz),
      viz_pub_rate_(viz_pub_rate),
      world_yaml_file_(world_yaml_file),
      seed_(seed),
      real_time_factor_(real_time_factor) {
  ROS_INFO_NAMED("SimMan",
                 "Simulation params: world_yaml_file(%s) update_rate(%f), "
                 "step_size(%f) show_viz(%s), viz_pub_rate(%f) seed(%d) "
                 "real_time_factor(%f)",
                 world_yaml_file_.c_str(), update_rate_, step_size_,
                 show_viz_ ? "true" : "false", viz_pub_rate_, seed_,
                 real_time_factor_);
}

void SimulationManager::Main(bool benchmark) {
  ROS_INFO_NAMED("SimMan", "Initializing...");
  run_simulator_ = true;

  try {
    world_ = World::MakeWorld(world_yaml_file_, seed_);
    ROS_INFO_NAMED("SimMan", "World loaded");
  } catch (const std::exception& e) {
    ROS_FATAL_NAMED("SimMan", "%s", e.what());
    return;
  }

  if (show_viz_) {
    world_->DebugVisualize();
    // Latched, one-shot diagnostic overlays (occupancy grids, friction
    // regions). Only needed when visualizing; costs nothing per step.
    world_->PublishDiagnostics();
  }

  iterations_ = 0;
  double filtered_cycle_util = 0;
  double min_cycle_util = std::numeric_limits<double>::infinity();
  double max_cycle_util = 0;
  double viz_update_period = 1.0f / viz_pub_rate_;
  ServiceManager service_manager(this, world_);

  timekeeper_.SetMaxStepSize(step_size_);

  // Pacing is decoupled from physics: physics always advances by exactly one
  // fixed step_size per iteration, while wall-clock speed is governed only by
  // real_time_factor. Target wall time per step = step_size / real_time_factor.
  // real_time_factor <= 0 (or benchmark) runs unthrottled (no sleeping), so the
  // host speed never affects the simulated result or message timestamps.
  bool throttle = (real_time_factor_ > 0.0) && (benchmark == false);
  std::chrono::duration<double> expected_cycle_time(
      throttle ? step_size_ / real_time_factor_ : 0.0);

  // integrated ros::WallRate logic here to expose internals for benchmarking
  std::chrono::duration<double> start =
      std::chrono::steady_clock::now().time_since_epoch();
  std::chrono::duration<double> actual_cycle_time(0.0);
  using seconds_d = std::chrono::duration<double, std::ratio<1, 1>>;
  double seconds_taken = 0;

  // Visualization cadence is driven off simulated time (not wall time) so the
  // published output is identical regardless of how fast the host runs.
  double last_viz_sim_time = -std::numeric_limits<double>::infinity();

  ROS_INFO_NAMED("SimMan", "Simulation loop started (real_time_factor=%.3f%s)",
                 real_time_factor_, throttle ? "" : ", unthrottled");

  while (ros::ok() && run_simulator_) {
    std::chrono::duration<double> update_start =
        std::chrono::steady_clock::now().time_since_epoch();

    world_->Update(timekeeper_);  // Step physics by one fixed step_size

    double sim_time = timekeeper_.GetSimTime().toSec();
    bool update_viz =
        show_viz_ && (sim_time - last_viz_sim_time >= viz_update_period);
    if (update_viz) {
      last_viz_sim_time = sim_time;
      world_->DebugVisualize(false);  // no need to update layer
      DebugVisualization::Get().Publish(
          timekeeper_);  // publish debug visualization
    }

    ros::spinOnce();

    seconds_taken +=
        (seconds_d(std::chrono::steady_clock::now().time_since_epoch()) -
         update_start)
            .count();

    // Pace to the real-time factor, unless running unthrottled. This only
    // affects wall-clock timing, never the number of physics steps.
    {
      std::chrono::duration<double> expected_end = start + expected_cycle_time;
      std::chrono::duration<double> actual_end =
          std::chrono::steady_clock::now().time_since_epoch();
      std::chrono::duration<double> sleep_time = expected_end - actual_end;
      actual_cycle_time = actual_end - start;
      start = expected_end;  // make sure to reset our start time
      if (!throttle) {
        start = actual_end;  // no pacing target, just track elapsed time
      } else if (sleep_time.count() <= 0.0) {
        // we've taken too long; don't sleep, and resync if we're far behind
        if (actual_end > expected_end + expected_cycle_time) {
          start = actual_end;
        }
      } else {
        std::this_thread::sleep_for(sleep_time);
      }
    }

    iterations_++;

    // Utilization: wall time spent per step vs. the per-step budget. When
    // unthrottled the budget is the step itself, so factor = sim/wall speedup.
    double cycle_time = actual_cycle_time.count() * 1000;
    double budget_ms =
        (throttle ? expected_cycle_time.count() : step_size_) * 1000;
    double cycle_util = budget_ms > 0.0 ? cycle_time / budget_ms * 100 : 0.0;
    double factor = real_time_factor_;
    if (!throttle) {
      factor = cycle_time > 0.0 ? step_size_ * 1000 / cycle_time : 0.0;
    }
    min_cycle_util = std::min(cycle_util, min_cycle_util);
    if (iterations_ > 10) max_cycle_util = std::max(cycle_util, max_cycle_util);
    filtered_cycle_util = 0.99 * filtered_cycle_util + 0.01 * cycle_util;

    ROS_INFO_THROTTLE_NAMED(
        1, "SimMan",
        "utilization: min %.1f%% max %.1f%% ave %.1f%%  factor: %.1f",
        min_cycle_util, max_cycle_util, filtered_cycle_util, factor);
  }
  // std::cout << "Simulation loop ended. " << iterations_ << " iterations in "
  // << seconds_taken << " seconds, " <<  (double)iterations_/seconds_taken << "
  // iterations/sec" << std::endl;

  delete world_;
}

void SimulationManager::Shutdown() {
  ROS_INFO_NAMED("SimMan", "Shutdown called");
  run_simulator_ = false;
}

};  // namespace flatland_server
