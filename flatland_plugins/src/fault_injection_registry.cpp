/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	fault_injection_registry.cpp
 * @brief	In-process store of active fault effects + pure ramp/trigger math
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

#include <flatland_plugins/fault_injection_registry.h>

#include <algorithm>
#include <cmath>

namespace flatland_plugins {

FaultInjectionRegistry &FaultInjectionRegistry::Get() {
  static FaultInjectionRegistry instance;
  return instance;
}

void FaultInjectionRegistry::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  effects_.clear();
}

void FaultInjectionRegistry::SetEffects(
    const std::map<std::string, FaultEffect> &effects) {
  std::lock_guard<std::mutex> lock(mutex_);
  effects_ = effects;
}

FaultEffect FaultInjectionRegistry::GetEffect(const std::string &component_key,
                                              FaultKind kind) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = effects_.find(MakeKey(component_key, kind));
  if (it == effects_.end()) {
    return FaultEffect();
  }
  return it->second;
}

std::string FaultInjectionRegistry::MakeKey(const std::string &component_key,
                                            FaultKind kind) {
  return component_key + "#" + std::to_string(static_cast<int>(kind));
}

// --- Pure helpers -----------------------------------------------------------

std::string ComponentKey(const std::string &model,
                         const std::string &component) {
  return model + "/" + component;
}

namespace {
// Map a normalized ramp fraction f in [0,1] to a shaped fraction in [0,1].
double Shape(RampProfile profile, double f) {
  if (f <= 0.0) return 0.0;
  if (f >= 1.0) return 1.0;
  switch (profile) {
    case RampProfile::kLinear:
      return f;
    case RampProfile::kExp: {
      const double k = 3.0;  // ease-in curvature
      return (std::exp(k * f) - 1.0) / (std::exp(k) - 1.0);
    }
    case RampProfile::kStep:
    default:
      return 1.0;
  }
}
}  // namespace

double SeverityAt(const SeverityProfile &p, double sim_time) {
  if (sim_time < p.onset_time) {
    return 0.0;
  }
  const double e = sim_time - p.onset_time;  // time since onset
  const bool indefinite = p.hold < 0.0;

  // A step profile ignores the ramps: the whole active window sits at peak.
  if (p.profile == RampProfile::kStep) {
    if (indefinite) return p.peak_severity;
    const double total = std::max(0.0, p.ramp_up) + std::max(0.0, p.hold) +
                         std::max(0.0, p.ramp_down);
    return (e < total) ? p.peak_severity : 0.0;
  }

  // Ramp-up phase.
  if (p.ramp_up > 0.0 && e < p.ramp_up) {
    return p.peak_severity * Shape(p.profile, e / p.ramp_up);
  }

  const double e_after_up = e - std::max(0.0, p.ramp_up);
  if (indefinite) {
    return p.peak_severity;
  }

  // Hold phase.
  const double hold = std::max(0.0, p.hold);
  if (e_after_up <= hold) {
    return p.peak_severity;
  }

  // Ramp-down phase.
  const double e_down = e_after_up - hold;
  if (p.ramp_down > 0.0 && e_down < p.ramp_down) {
    return p.peak_severity * Shape(p.profile, 1.0 - e_down / p.ramp_down);
  }

  return 0.0;
}

bool ConditionMet(const std::string & /*condition_type*/, double value,
                  double threshold) {
  // All supported numeric conditions (distance_travelled, x_greater, y_greater,
  // elapsed) latch when the measured quantity reaches the threshold.
  return value >= threshold;
}

FaultKind ParseFaultKind(const std::string &type) {
  if (type == "sensor_bias") return FaultKind::kSensorBias;
  if (type == "sensor_drift") return FaultKind::kSensorDrift;
  if (type == "sensor_scale") return FaultKind::kSensorScale;
  if (type == "noise_inflation") return FaultKind::kNoiseInflation;
  if (type == "dropout") return FaultKind::kDropout;
  if (type == "stuck" || type == "frozen") return FaultKind::kStuck;
  if (type == "quantization") return FaultKind::kQuantization;
  if (type == "laser_sector_occlusion") return FaultKind::kLaserSectorOcclusion;
  if (type == "torque_loss") return FaultKind::kTorqueLoss;
  if (type == "wheel_slip") return FaultKind::kWheelSlip;
  if (type == "asymmetric_drive") return FaultKind::kAsymmetricDrive;
  if (type == "deadband") return FaultKind::kDeadband;
  if (type == "stuck_wheel") return FaultKind::kStuckWheel;
  if (type == "encoder_drift") return FaultKind::kEncoderDrift;
  if (type == "odom_slip") return FaultKind::kOdomSlip;
  if (type == "amcl_divergence") return FaultKind::kAmclDivergence;
  return FaultKind::kUnknown;
}

RampProfile ParseRampProfile(const std::string &profile) {
  if (profile == "step") return RampProfile::kStep;
  if (profile == "exp") return RampProfile::kExp;
  return RampProfile::kLinear;
}

}  // namespace flatland_plugins
