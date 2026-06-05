/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	fault_injector.h
 * @brief	World plugin: orchestrates faults, emits sealed ground truth
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

#ifndef FLATLAND_PLUGINS_FAULT_INJECTOR_H
#define FLATLAND_PLUGINS_FAULT_INJECTOR_H

#include <Box2D/Box2D.h>
#include <flatland_plugins/fault_injection_registry.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <flatland_server/world_plugin.h>
#include <ros/ros.h>

#include <map>
#include <string>
#include <vector>

namespace flatland_server {
class Model;
class YamlReader;
}

namespace flatland_plugins {

/**
 * @brief World plugin that drives the fault-injection framework.
 *
 * Parses the `faults:` list from the world YAML, evaluates time/condition
 * triggers and severity ramps each step, writes the resulting effects into the
 * FaultInjectionRegistry (which sensor/drive plugins consult), and emits the
 * ground-truth label EXCLUSIVELY out-of-band: a reserved-namespace topic plus a
 * sealed sidecar manifest file. It never touches the in-band sensor/odom/tf
 * stream. See doc/fault_injection.md for the full contract.
 */
class FaultInjector : public flatland_server::WorldPlugin {
 public:
  /// Per-fault parsed config plus runtime trigger/emission state.
  struct Fault {
    // --- config ---
    std::string id;              ///< unique id
    std::string type_str;        ///< raw taxonomy type string
    FaultKind kind;              ///< resolved fault kind
    std::string model;           ///< target model name
    std::string component;       ///< target plugin name
    std::string topic;           ///< in-band topic (documentation only)
    bool condition_trigger;      ///< true=condition, false=time
    std::string condition_type;  ///< condition name (when condition_trigger)
    double threshold;            ///< condition threshold
    std::string depends_on;      ///< chained prerequisite fault id (optional)
    SeverityProfile profile;  ///< severity envelope (onset resolved per kind)
    std::map<std::string, double> params;  ///< fault-specific peak magnitudes
    std::string trigger_description;       ///< human-readable summary

    // --- environment-fault config (environment kinds only) ---
    std::string env_model_path;  ///< obstacle/furniture .model.yaml path
    flatland_server::Pose env_spawn_pose;  ///< dynamic_obstacle spawn pose
    flatland_server::Pose env_dest_pose;   ///< moved_furniture destination pose
    std::vector<b2Vec2> env_waypoints;     ///< obstacle path waypoints [m]
    double env_speed = 0.5;        ///< obstacle speed [m/s] at severity 1
    bool env_despawn_on_end = false;   ///< remove the obstacle on fault end
    bool env_spawn_if_absent = false;  ///< spawn furniture if target is absent
    double env_spill_cx = 0.0;         ///< spill centre world x [m]
    double env_spill_cy = 0.0;         ///< spill centre world y [m]
    double env_spill_radius = 1.0;     ///< spill outer radius [m]
    double env_spill_mu_min = 0.3;     ///< spill multiplier at peak severity

    // --- runtime state ---
    bool latched = false;        ///< condition has fired (onset resolved)
    double latched_onset = 0.0;  ///< sim-sec onset latched for condition faults
    bool ever_active = false;    ///< became active at least once (for chaining)
    bool was_active = false;     ///< active last step (for end detection)
    double current_severity = 0.0;  ///< severity this step (for GT emission)
    double recorded_onset = -1.0;   ///< actual onset written to manifest
    double recorded_end = -1.0;     ///< actual end written to manifest

    // --- environment-fault runtime state ---
    bool env_applied = false;      ///< onset world-mutation applied (once)
    bool env_ended = false;        ///< end-edge world-mutation applied (once)
    std::string env_spawned_name;  ///< neutral name of the spawned obstacle
    size_t env_waypoint_idx = 0;   ///< current target waypoint index
  };

  void OnInitialize(const YAML::Node &config) override;
  void BeforePhysicsStep(
      const flatland_server::Timekeeper &timekeeper) override;

 private:
  /**
   * @brief Find a loaded model by name, or nullptr.
   */
  flatland_server::Model *FindModel(const std::string &name) const;

  /**
   * @brief Measure the quantity a condition trigger compares to its threshold.
   */
  double MeasureCondition(const Fault &fault, double elapsed) const;

  /**
   * @brief Update per-model travelled-distance accumulators for this step.
   */
  void UpdateModelMotion();

  /**
   * @brief Parse the `params:` block for an environment fault (string/list
   * fields the numeric-only path cannot handle: model path, waypoints, poses).
   */
  void ParseEnvironmentParams(Fault &fault,
                              flatland_server::YamlReader &params_reader);

  /**
   * @brief Drive an environment fault's world mutation for this step.
   *
   * Spawns/moves on the active edge, ramps the spill multiplier and advances
   * obstacle motion while active, and tears down on the end edge. Never writes
   * the effect registry. Box2D/world calls are wrapped so a bad model path
   * warns rather than crashing the physics loop.
   */
  void HandleEnvironmentFault(Fault &fault, double severity, bool active,
                              const flatland_server::Timekeeper &timekeeper);

  /**
   * @brief Apply the one-shot onset mutation (spawn obstacle / move furniture).
   */
  void ApplyEnvironmentOnset(Fault &fault);

  /**
   * @brief Apply the one-shot end mutation (stop/despawn, clear spill).
   */
  void ApplyEnvironmentEnd(Fault &fault);

  /**
   * @brief Advance a live dynamic obstacle along its waypoints under sim time.
   */
  void UpdateDynamicObstacle(Fault &fault, double severity,
                             const flatland_server::Timekeeper &timekeeper);

  /**
   * @brief Publish the sealed ground-truth snapshot on the reserved topic.
   */
  void PublishGroundTruth(const ros::Time &stamp);

  /**
   * @brief (Re)write the sealed sidecar manifest file.
   */
  void WriteManifest();

  std::vector<Fault> faults_;        ///< configured faults
  ros::Publisher ground_truth_pub_;  ///< reserved-namespace GT publisher
  std::string ground_truth_topic_;   ///< reserved topic name
  std::string ground_truth_path_;    ///< sealed sidecar file path

  bool started_ = false;            ///< first step seen
  ros::Time start_time_;            ///< sim time captured at first step
  double last_gt_publish_ = -1e9;   ///< last GT publish (elapsed sec)
  double gt_publish_period_ = 0.1;  ///< GT publish throttle [s]

  std::map<std::string, b2Vec2> last_pos_;  ///< last position per model
  std::map<std::string, double> distance_;  ///< travelled distance per model

  int env_spawn_counter_ = 0;  ///< monotonic suffix for neutral obstacle names
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_FAULT_INJECTOR_H
