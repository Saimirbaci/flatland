/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	actuator_dynamics.h
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

#ifndef FLATLAND_PLUGINS_ACTUATOR_DYNAMICS_H
#define FLATLAND_PLUGINS_ACTUATOR_DYNAMICS_H

#include <flatland_server/yaml_reader.h>
#include <ros/time.h>
#include <yaml-cpp/yaml.h>

#include <deque>
#include <utility>

namespace flatland_plugins {

/**
 * @brief Composable per-axis actuator/motor dynamics for the drive plugins.
 *
 * This is a pure utility (no ROS I/O, no Box2D ownership) layered on top of the
 * existing DynamicsLimits velocity/acceleration ramp and the WheelFrictionModel
 * grip cap. One instance models a single command axis (e.g. linear or angular
 * for diff_drive, drive or steer for tricycle_drive). It captures the parts
 * of a real drivetrain that ideal velocity control ignores:
 *
 *   - command latency : transport / communication deadtime before a command
 *                       takes effect (a sim-time FIFO delay line);
 *   - deadband        : a motor / static-friction deadzone that zeroes
 *                       sub-threshold commands (optionally rescaling above it);
 *   - effort limit    : a bounded actuator force [N] or torque [N*m] that caps
 *                       the achievable per-step acceleration (a_max = F/m,
 *                       alpha_max = tau/I) and, in friction drive_mode, caps
 *                       the per-wheel motor force before the grip cap.
 *
 * Signal ordering (applied by the drive plugin each BeforePhysicsStep):
 *   (1) raw twist captured in the twist callback (cached);
 *   (2) Push() the cached command with the current Timekeeper sim time and
 *       Pull() the command delayed by command_latency seconds;
 *   (3) ApplyDeadband() on the delayed command;
 *   (4) the EXISTING DynamicsLimits::Limit() acceleration/velocity ramp;
 *   (5) AccelerationCap() modulates (further bounds) that ramp from the effort
 *       limit -- it does NOT add a second independent ramp.
 *
 * Back-compat contract: every field defaults to 0 (disabled). A plugin that
 * does not declare the actuator subnode, or declares an empty one, gets a pure
 * pass-through (Pull returns the latest command, ApplyDeadband is identity,
 * AccelerationCap is +infinity), so existing worlds are byte-for-byte the same.
 *
 * Determinism: the latency buffer is driven by Timekeeper sim time only (never
 * wall-clock), so a given run seed reproduces exactly.
 *
 * YAML keys (all optional, 0 = disabled):
 *   command_latency  : transport deadtime [s]
 *   deadband         : deadzone threshold in axis units [m/s] or [rad/s]
 *   deadband_rescale : if true, rescale above-threshold commands so the output
 *                      is continuous across the deadband edge (default false)
 *   max_force        : bounded actuator force [N]   (linear axis)
 *   max_torque       : bounded actuator torque [N*m] (angular axis)
 */
class ActuatorDynamics {
 public:
  /// Hard cap on the latency FIFO length so a Push without a matching Pull can
  /// never grow memory without bound (defensive; in normal use the buffer only
  /// ever holds the in-flight commands, i.e. ~latency / timestep entries).
  static const size_t kMaxBufferSize = 4096;

  double command_latency_ = 0.0;   ///< Command transport deadtime [s], 0 = off
  double deadband_ = 0.0;          ///< Deadzone threshold [axis units], 0 = off
  bool deadband_rescale_ = false;  ///< Rescale above-threshold for continuity
  double max_effort_ = 0.0;        ///< Actuator force[N] / torque[N*m], 0 = off

  /**
   * @brief blank constructor, defaults leave the model a no-op pass-through
   */
  ActuatorDynamics() {}

  /**
   * @name         Configure
   * @brief        Load actuator parameters from a yaml configuration node
   * @param[in]    config YAML node for the actuator block (may be empty/null)
   */
  void Configure(const YAML::Node &config);

  /**
   * @name          Enabled
   * @brief         Whether any actuator effect is active
   * @return        true if latency, deadband or effort limit is configured
   */
  bool Enabled() const;

  /**
   * @name          ApplyDeadband
   * @brief         Zero sub-threshold commands (and optionally rescale above)
   * @param[in]     cmd The commanded axis value
   * @return        0 if |cmd| < deadband, else cmd (or the rescaled command)
   */
  double ApplyDeadband(double cmd) const;

  /**
   * @name          Push
   * @brief         Enqueue a timestamped command into the latency delay line
   * @param[in]     cmd   The commanded axis value
   * @param[in]     stamp The Timekeeper sim time the command was issued at
   */
  void Push(double cmd, const ros::Time &stamp);

  /**
   * @name          Pull
   * @brief         Return the command delayed by command_latency seconds
   * @details       Zero-order hold: returns the value of the most recent
   *                command whose stamp is <= now - command_latency. With
   *                latency disabled this is a pass-through (latest command).
   * @param[in]     now The current Timekeeper sim time
   * @return        The latency-delayed command (0 until the first one arrives)
   */
  double Pull(const ros::Time &now);

  /**
   * @name          AccelerationCap
   * @brief         Convert the effort limit into a per-step acceleration cap
   * @param[in]     inertia Body mass [kg] (linear) or rotational inertia
   *                [kg*m^2] (angular), re-read from the Box2D body each step
   * @return        max_effort / inertia, or +infinity when disabled (effort 0)
   *                or inertia is non-positive (detached unit setups)
   */
  double AccelerationCap(double inertia) const;

  /**
   * @name          ForceCap
   * @brief         The raw effort limit for the friction (per-wheel) path
   * @return        max_effort_ [N] or [N*m] (0 = disabled / no cap)
   */
  double ForceCap() const { return max_effort_; }

 private:
  /// Sim-time FIFO of (stamp, command) pairs in the latency delay line.
  std::deque<std::pair<ros::Time, double>> command_buffer_;
  /// Zero-order-hold value: the most recent command that has already arrived.
  double last_command_ = 0.0;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_ACTUATOR_DYNAMICS_H
