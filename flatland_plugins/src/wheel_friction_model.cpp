/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	wheel_friction_model.cpp
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

#include "flatland_plugins/wheel_friction_model.h"

#include <algorithm>
#include <cmath>

namespace flatland_plugins {

const double WheelFrictionModel::kGravity = 9.80665;

void WheelFrictionModel::Configure(const YAML::Node &config) {
  flatland_server::YamlReader reader(config);
  enabled_ = reader.Get<bool>("enabled", false);
  mu_long_static_ = reader.Get<double>("mu_long_static", 1.0);
  mu_long_kinetic_ = reader.Get<double>("mu_long_kinetic", 0.8);
  mu_lat_static_ = reader.Get<double>("mu_lat_static", 1.1);
  mu_lat_kinetic_ = reader.Get<double>("mu_lat_kinetic", 0.9);
  static_slip_threshold_ = reader.Get<double>("static_slip_threshold", 0.05);
}

double WheelFrictionModel::AxisForce(double slip_vel, double mu_static,
                                     double mu_kinetic, double normal_load,
                                     double dt) const {
  // Share of mass carried by this wheel, recovered from the normal load.
  double effective_mass = normal_load / kGravity;

  // Force that, applied over dt, exactly cancels the slip this step (F = m*a).
  double null_force = effective_mass * slip_vel / dt;

  // Coulomb ceiling: static (grip) below the slip threshold, kinetic (slipping)
  // above it. This makes the wheel hold commanded motion until the demanded
  // force exceeds mu_static * load, then drop to the lower kinetic plateau.
  double mu =
      (std::fabs(slip_vel) < static_slip_threshold_) ? mu_static : mu_kinetic;
  double max_force = mu * normal_load;

  return std::max(-max_force, std::min(null_force, max_force));
}

b2Vec2 WheelFrictionModel::ComputeWheelForce(double longitudinal_slip_vel,
                                             double lateral_slip_vel,
                                             double normal_load,
                                             double dt) const {
  // Degenerate inputs produce no force rather than a divide-by-zero.
  if (dt <= 0.0 || normal_load <= 0.0) {
    return b2Vec2(0.0f, 0.0f);
  }

  double fx = AxisForce(longitudinal_slip_vel, mu_long_static_,
                        mu_long_kinetic_, normal_load, dt);
  double fy = AxisForce(lateral_slip_vel, mu_lat_static_, mu_lat_kinetic_,
                        normal_load, dt);

  return b2Vec2(static_cast<float>(fx), static_cast<float>(fy));
}

}  // namespace flatland_plugins
