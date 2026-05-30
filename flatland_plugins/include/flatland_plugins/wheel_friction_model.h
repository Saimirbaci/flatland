/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	wheel_friction_model.h
 * @brief   Anisotropic (longitudinal/lateral, static/kinetic) wheel-ground
 *          tangential friction model shared by the drive plugins
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

#ifndef FLATLAND_PLUGINS_WHEEL_FRICTION_MODEL_H
#define FLATLAND_PLUGINS_WHEEL_FRICTION_MODEL_H

#include <Box2D/Box2D.h>
#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>

namespace flatland_plugins {

/**
 * @brief Anisotropic Coulomb tire/ground friction model.
 *
 * For a single wheel, given the tangential slip velocity at the contact patch
 * (decomposed into the wheel-frame longitudinal and lateral axes) and the
 * normal load, this returns the friction force (in the same wheel frame) that
 * resists the slip. Each axis has independent static and kinetic friction
 * coefficients; the static coefficient is used while the slip speed stays below
 * a configurable threshold (grip), the kinetic coefficient once it exceeds it
 * (slipping). The force per axis is the impulse that would exactly null the
 * slip this step, clamped to the Coulomb ceiling mu * normal_load.
 *
 * The model is pure math (no ROS / Box2D world access beyond b2Vec2) so it is
 * directly unit-testable and shared by diff_drive and tricycle_drive.
 */
class WheelFrictionModel {
 public:
  /// Standard gravitational acceleration [m/s^2], used to recover the share of
  /// mass supported by a wheel from its normal load (m_share = N / g).
  static const double kGravity;

  bool enabled_ = false;  ///< Whether the friction path is active for the plugin
  double mu_long_static_ = 1.0;    ///< Longitudinal static friction coefficient
  double mu_long_kinetic_ = 0.8;   ///< Longitudinal kinetic friction coefficient
  double mu_lat_static_ = 1.1;     ///< Lateral static friction coefficient
  double mu_lat_kinetic_ = 0.9;    ///< Lateral kinetic friction coefficient
  double static_slip_threshold_ =
      0.05;  ///< Slip speed [m/s] separating static (grip) from kinetic (slip)

  /**
   * @brief blank constructor, defaults leave the model disabled
   */
  WheelFrictionModel() {}

  /**
   * @name         Configure
   * @brief        Load friction parameters from a yaml configuration node
   * @param[in]    config YAML node for the wheel_friction block
   */
  void Configure(const YAML::Node &config);

  /**
   * @name          ComputeWheelForce
   * @brief         Compute the wheel-frame friction force resisting the slip
   * @param[in]     longitudinal_slip_vel Commanded minus actual contact-patch
   *                velocity along the wheel longitudinal (rolling) axis [m/s]
   * @param[in]     lateral_slip_vel Commanded minus actual contact-patch
   *                velocity along the wheel lateral axis [m/s]
   * @param[in]     normal_load Normal load on the wheel [N] (= m_share * g)
   * @param[in]     dt Physics timestep [s]
   * @param[in]     surface_factor Surface friction multiplier under the wheel
   *                (1.0 = nominal/dry, < 1.0 = wet/slippery). Scales the Coulomb
   *                ceiling mu * normal_load before clamping, so a wheel over a
   *                wet patch loses traction. Sampled from the world's surface
   *                friction field at the wheel contact point; defaults to 1.0
   *                so the model is unchanged when no field is configured.
   * @return        Friction force in the wheel frame (x = longitudinal,
   *                y = lateral), each component clamped to
   *                surface_factor * mu * normal_load
   */
  b2Vec2 ComputeWheelForce(double longitudinal_slip_vel,
                           double lateral_slip_vel, double normal_load,
                           double dt, double surface_factor = 1.0) const;

 private:
  /**
   * @name          AxisForce
   * @brief         Coulomb-limited friction force along a single wheel axis
   * @param[in]     slip_vel Slip velocity along the axis [m/s]
   * @param[in]     mu_static Static friction coefficient for the axis
   * @param[in]     mu_kinetic Kinetic friction coefficient for the axis
   * @param[in]     normal_load Normal load on the wheel [N]
   * @param[in]     dt Physics timestep [s]
   * @param[in]     surface_factor Surface friction multiplier (1.0 = nominal)
   *                scaling the Coulomb ceiling
   * @return        Friction force along the axis [N], clamped to
   *                surface_factor * mu * load
   */
  double AxisForce(double slip_vel, double mu_static, double mu_kinetic,
                   double normal_load, double dt, double surface_factor) const;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_WHEEL_FRICTION_MODEL_H
