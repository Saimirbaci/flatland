/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	actuator_dynamics.cpp
 * @brief   Per-axis actuator/motor dynamics (command latency, deadband and a
 *          torque/force -> acceleration cap) shared by the drive plugins
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

#include "flatland_plugins/actuator_dynamics.h"

#include <cmath>
#include <limits>

namespace flatland_plugins {

void ActuatorDynamics::Configure(const YAML::Node &config) {
  flatland_server::YamlReader reader(config);
  command_latency_ = reader.Get<double>("command_latency", 0.0);
  deadband_ = reader.Get<double>("deadband", 0.0);
  deadband_rescale_ = reader.Get<bool>("deadband_rescale", false);

  // The class is axis-agnostic, so accept either the linear "max_force" [N] or
  // the angular "max_torque" [N*m] key; whichever is set becomes the generic
  // effort limit. Both 0 (default) leaves the effort limit disabled.
  double max_force = reader.Get<double>("max_force", 0.0);
  double max_torque = reader.Get<double>("max_torque", 0.0);
  max_effort_ = (max_force != 0.0) ? max_force : max_torque;
}

bool ActuatorDynamics::Enabled() const {
  return command_latency_ > 0.0 || deadband_ > 0.0 || max_effort_ > 0.0;
}

double ActuatorDynamics::ApplyDeadband(double cmd) const {
  if (deadband_ <= 0.0) {
    return cmd;  // disabled: identity pass-through
  }
  if (std::fabs(cmd) < deadband_) {
    return 0.0;  // inside the deadzone: motor does not move
  }
  if (!deadband_rescale_) {
    return cmd;  // hard deadband: pass the command through unchanged
  }
  // Rescaled deadband: shift the magnitude down by the threshold so the output
  // is continuous across the deadband edge (just above threshold -> ~0).
  double sign = (cmd > 0.0) ? 1.0 : -1.0;
  return sign * (std::fabs(cmd) - deadband_);
}

void ActuatorDynamics::Push(double cmd, const ros::Time &stamp) {
  command_buffer_.emplace_back(stamp, cmd);
  // Defensive bound: drop the oldest entries if Pull is not draining the line.
  while (command_buffer_.size() > kMaxBufferSize) {
    command_buffer_.pop_front();
  }
}

double ActuatorDynamics::Pull(const ros::Time &now) {
  if (command_latency_ <= 0.0) {
    // Disabled: pass-through. Collapse the buffer to the latest command so it
    // never accumulates, and remember it for steps with no new command.
    if (!command_buffer_.empty()) {
      last_command_ = command_buffer_.back().second;
      command_buffer_.clear();
    }
    return last_command_;
  }

  // Before a full latency has elapsed (relative to sim-time zero) no command
  // can have traversed the delay line yet; return the held value. This also
  // avoids an underflow of ros::Time (which is unsigned) when now < latency.
  if (now.toSec() < command_latency_) {
    return last_command_;
  }

  // Zero-order hold delay line: advance the active command to the newest entry
  // that has already "arrived" (stamp <= now - latency), discarding the entries
  // it supersedes. The retained value persists in last_command_ until the next
  // command arrives, so a command pushed at t takes effect at t + latency.
  ros::Time cutoff = now - ros::Duration(command_latency_);
  while (!command_buffer_.empty() && command_buffer_.front().first <= cutoff) {
    last_command_ = command_buffer_.front().second;
    command_buffer_.pop_front();
  }
  return last_command_;
}

double ActuatorDynamics::AccelerationCap(double inertia) const {
  if (max_effort_ <= 0.0 || inertia <= 0.0) {
    return std::numeric_limits<double>::infinity();  // disabled / no cap
  }
  return max_effort_ / inertia;
}

}  // namespace flatland_plugins
