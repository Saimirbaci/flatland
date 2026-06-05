/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	fault_injection_registry.h
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

#ifndef FLATLAND_PLUGINS_FAULT_INJECTION_REGISTRY_H
#define FLATLAND_PLUGINS_FAULT_INJECTION_REGISTRY_H

#include <map>
#include <mutex>
#include <string>

namespace flatland_plugins {

/**
 * @brief The taxonomy of injectable faults.
 *
 * Sensor faults perturb a published sensor message; drivetrain faults perturb
 * the commanded motion (causally, through the physics solver) and therefore the
 * resulting odom/tf. See doc/fault_injection.md for the full table.
 */
enum class FaultKind {
  kUnknown,
  // Sensor faults
  kSensorBias,
  kSensorDrift,
  kSensorScale,
  kNoiseInflation,
  kDropout,
  kStuck,
  kQuantization,
  kLaserSectorOcclusion,
  // Drivetrain faults
  kTorqueLoss,
  kWheelSlip,
  kAsymmetricDrive,
  kDeadband,
  kStuckWheel,
  // Localization / odometry faults (measurement-domain: they perturb the
  // reported odom / localization estimate while the true Box2D motion is
  // unchanged -- the opposite of the causal drivetrain faults above). Note:
  // "IMU bias" is intentionally NOT a new kind here; it is the existing
  // kSensorBias / kSensorDrift applied to the imu component.
  kEncoderDrift,
  kOdomSlip,
  kAmclDivergence,
};

/**
 * @brief Shape of the severity ramp from onset to peak (and back).
 */
enum class RampProfile { kStep, kLinear, kExp };

/**
 * @brief Time-domain description of a fault's severity envelope.
 *
 * For a time trigger, onset_time is the configured sim-time onset. For a
 * condition trigger, the FaultInjector overwrites onset_time with the latched
 * sim time at which the condition first became true, so SeverityAt() is a pure
 * function of (profile, sim_time) in both cases.
 */
struct SeverityProfile {
  double onset_time = 0.0;     ///< sim seconds since world start
  double ramp_up = 0.0;        ///< seconds to climb to peak
  double hold = -1.0;          ///< seconds at peak; < 0 means indefinite
  double ramp_down = 0.0;      ///< seconds to fall back to 0
  double peak_severity = 1.0;  ///< peak severity in [0,1]
  RampProfile profile = RampProfile::kLinear;
};

/**
 * @brief A single active fault effect, as read by a sensor/drive plugin.
 *
 * Returned by value from the registry so reads never alias the stored state.
 * `params` carries the fault-specific peak magnitudes from the world YAML;
 * callers scale them by `severity`.
 */
struct FaultEffect {
  bool active = false;                   ///< true when severity > 0 this step
  double severity = 0.0;                 ///< current severity in [0,1]
  std::map<std::string, double> params;  ///< fault-specific peak magnitudes

  /**
   * @brief Look up a param by name with a default.
   */
  double Param(const std::string &key, double default_val = 0.0) const {
    auto it = params.find(key);
    return it == params.end() ? default_val : it->second;
  }
};

/**
 * @brief Process-wide store of the active fault effects for the current step.
 *
 * Mirrors the RngManager singleton pattern so no plugin signatures change. The
 * FaultInjector world plugin is the single writer (SetEffects once per step);
 * sensor/drive model plugins are read-only consumers (GetEffect). Flatland runs
 * one world per process, so a singleton suffices; Reset() clears it between
 * unit tests.
 *
 * Thread-safety: a mutex guards the map. In practice sensor plugins read the
 * snapshot on the main thread (e.g. laser reads once in BeforePhysicsStep and
 * captures the values into locals before dispatching its ThreadPool workers),
 * so the lock is essentially uncontended.
 */
class FaultInjectionRegistry {
 public:
  /**
   * @brief Return the process-wide singleton.
   */
  static FaultInjectionRegistry &Get();

  /**
   * @brief Clear all effects. Call between tests / on world teardown.
   */
  void Reset();

  /**
   * @brief Replace the whole effect snapshot. Called once per step by the
   * FaultInjector (single writer).
   * @param[in] effects Map keyed by MakeKey(component_key, kind).
   */
  void SetEffects(const std::map<std::string, FaultEffect> &effects);

  /**
   * @brief Look up the effect for a component and fault kind.
   * @param[in] component_key Stable component id, ComponentKey(model, plugin).
   * @param[in] kind The fault kind to query.
   * @return The effect (active == false if none is configured/active).
   */
  FaultEffect GetEffect(const std::string &component_key, FaultKind kind) const;

  /**
   * @brief Build the composite map key from a component key and fault kind.
   */
  static std::string MakeKey(const std::string &component_key, FaultKind kind);

 private:
  FaultInjectionRegistry() = default;

  mutable std::mutex mutex_;                    ///< guards effects_
  std::map<std::string, FaultEffect> effects_;  ///< current per-step snapshot
};

// --- Pure helpers (no ROS / Box2D), unit-testable in isolation --------------

/**
 * @brief Compose the stable component key from a model name and plugin name.
 */
std::string ComponentKey(const std::string &model,
                         const std::string &component);

/**
 * @brief Evaluate the severity envelope at a sim time.
 * @param[in] p The severity profile (onset already resolved).
 * @param[in] sim_time Sim seconds since world start.
 * @return severity in [0, peak_severity], 0 outside the active window.
 */
double SeverityAt(const SeverityProfile &p, double sim_time);

/**
 * @brief Evaluate a numeric condition trigger (value >= threshold).
 * @param[in] condition_type The condition name (for documentation/extension).
 * @param[in] value The current measured quantity (distance, x, y, elapsed...).
 * @param[in] threshold The configured threshold.
 */
bool ConditionMet(const std::string &condition_type, double value,
                  double threshold);

/**
 * @brief Map a YAML fault type string to a FaultKind (kUnknown if unknown).
 */
FaultKind ParseFaultKind(const std::string &type);

/**
 * @brief Map a YAML profile string to a RampProfile (kLinear if unrecognized).
 */
RampProfile ParseRampProfile(const std::string &profile);

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_FAULT_INJECTION_REGISTRY_H
