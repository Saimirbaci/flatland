/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	fault_injector.cpp
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

#include <flatland_msgs/FaultGroundTruth.h>
#include <flatland_msgs/FaultGroundTruthArray.h>
#include <flatland_plugins/fault_injector.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model.h>
#include <flatland_server/world.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

using namespace flatland_server;

namespace flatland_plugins {

namespace {
// Minimal JSON string escaping for the sidecar manifest (ids/types/descriptions
// are plain identifiers in practice, but escape defensively).
std::string JsonEscape(const std::string &s) {
  std::string out;
  out.reserve(s.size() + 2);
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}
}  // namespace

void FaultInjector::OnInitialize(const YAML::Node &config) {
  // Start from a clean registry so a fresh world never inherits stale effects
  // (e.g. across rostest cases in one process).
  FaultInjectionRegistry::Get().Reset();

  YamlReader reader(config);
  ground_truth_topic_ =
      reader.Get<std::string>("ground_truth_topic", "/_ground_truth/faults");
  ground_truth_path_ = reader.Get<std::string>(
      "ground_truth_path", "/tmp/flatland_fault_ground_truth.json");
  gt_publish_period_ = reader.Get<double>("ground_truth_publish_period", 0.1);

  YamlReader faults_reader = reader.SubnodeOpt("faults", YamlReader::LIST);
  if (!faults_reader.IsNodeNull()) {
    for (int i = 0; i < faults_reader.NodeSize(); i++) {
      YamlReader fr = faults_reader.Subnode(i, YamlReader::MAP);
      Fault f;
      f.id = fr.Get<std::string>("id");
      f.type_str = fr.Get<std::string>("type");
      f.kind = ParseFaultKind(f.type_str);
      if (f.kind == FaultKind::kUnknown) {
        throw YAMLException("FaultInjector: unknown fault type " +
                            Q(f.type_str) + " for fault " + Q(f.id));
      }

      YamlReader tr = fr.Subnode("target", YamlReader::MAP);
      f.model = tr.Get<std::string>("model");
      f.component = tr.Get<std::string>("component");
      f.topic = tr.Get<std::string>("topic", "");

      YamlReader trig = fr.Subnode("trigger", YamlReader::MAP);
      std::string trigger_kind = trig.Get<std::string>("kind", "time");
      f.condition_trigger = (trigger_kind == "condition");
      f.condition_type = trig.Get<std::string>("condition", "elapsed");
      f.threshold = trig.Get<double>("threshold", 0.0);
      f.depends_on = trig.Get<std::string>("depends_on", "");

      YamlReader sv = fr.Subnode("severity", YamlReader::MAP);
      f.profile.onset_time = sv.Get<double>("onset_time", 0.0);
      f.profile.ramp_up = sv.Get<double>("ramp_up", 0.0);
      f.profile.hold = sv.Get<double>("hold", -1.0);
      f.profile.ramp_down = sv.Get<double>("ramp_down", 0.0);
      f.profile.peak_severity = sv.Get<double>("peak", 1.0);
      f.profile.profile =
          ParseRampProfile(sv.Get<std::string>("profile", "linear"));

      YamlReader pr = fr.SubnodeOpt("params", YamlReader::MAP);
      if (!pr.IsNodeNull()) {
        if (IsEnvironmentKind(f.kind)) {
          // Environment faults carry string/list params (a model path,
          // waypoints, poses) the numeric-only map below cannot represent.
          ParseEnvironmentParams(f, pr);
        } else {
          YAML::Node pn = pr.Node();
          for (auto it = pn.begin(); it != pn.end(); ++it) {
            f.params[it->first.as<std::string>()] = it->second.as<double>();
          }
        }
      }

      if (f.condition_trigger) {
        f.trigger_description =
            "condition: " + f.condition_type +
            " >= " + std::to_string(f.threshold) +
            (f.depends_on.empty() ? "" : " after " + f.depends_on);
        distance_[f.model];  // ensure this model's motion is tracked
      } else {
        f.trigger_description =
            "time onset=" + std::to_string(f.profile.onset_time) + "s" +
            (f.depends_on.empty() ? "" : " after " + f.depends_on);
      }

      faults_.push_back(f);
    }
  }

  ground_truth_pub_ = nh_.advertise<flatland_msgs::FaultGroundTruthArray>(
      ground_truth_topic_, 1);

  // Create the sidecar up-front so the file always exists (onsets unresolved).
  WriteManifest();

  ROS_INFO_NAMED(
      "FaultInjector",
      "Loaded %zu fault(s); sealed ground truth -> topic %s, file %s",
      faults_.size(), ground_truth_topic_.c_str(), ground_truth_path_.c_str());
}

void FaultInjector::BeforePhysicsStep(const Timekeeper &timekeeper) {
  const ros::Time now = timekeeper.GetSimTime();
  if (!started_) {
    start_time_ = now;
    started_ = true;
  }
  const double elapsed = (now - start_time_).toSec();

  UpdateModelMotion();

  std::map<std::string, FaultEffect> snapshot;
  bool manifest_dirty = false;

  for (auto &f : faults_) {
    // Fault chaining: a fault with depends_on cannot fire until the named
    // prerequisite fault has become active at least once.
    bool prereq = true;
    if (!f.depends_on.empty()) {
      prereq = false;
      for (const auto &g : faults_) {
        if (g.id == f.depends_on) {
          prereq = g.ever_active;
          break;
        }
      }
    }

    double severity = 0.0;
    if (prereq) {
      SeverityProfile prof = f.profile;
      if (f.condition_trigger) {
        if (!f.latched) {
          bool met = false;
          if (f.condition_type == "after_fault") {
            met = true;  // prereq already satisfied above
          } else {
            double value = MeasureCondition(f, elapsed);
            met = ConditionMet(f.condition_type, value, f.threshold);
          }
          if (met) {
            f.latched = true;
            f.latched_onset = elapsed;
          }
        }
        if (f.latched) {
          prof.onset_time = f.latched_onset;
          severity = SeverityAt(prof, elapsed);
        }
      } else {
        severity = SeverityAt(prof, elapsed);
      }
    }

    f.current_severity = severity;
    const bool active = severity > 0.0;
    if (active) {
      f.ever_active = true;
    }
    if (IsEnvironmentKind(f.kind)) {
      // Environment faults mutate the world directly and are NEVER inserted
      // into the effect snapshot (no sensor/drive plugin consumes them); the
      // registry path stays byte-for-byte unchanged for every other kind.
      HandleEnvironmentFault(f, severity, active, timekeeper);
    } else if (active) {
      FaultEffect eff;
      eff.active = true;
      eff.severity = severity;
      eff.params = f.params;
      snapshot[FaultInjectionRegistry::MakeKey(
          ComponentKey(f.model, f.component), f.kind)] = eff;
    }

    // Track onset/end transitions for the sidecar manifest (write on change
    // only, never per-step, so file I/O does not block the physics loop).
    if (active && f.recorded_onset < 0.0) {
      f.recorded_onset =
          f.condition_trigger ? f.latched_onset : f.profile.onset_time;
      manifest_dirty = true;
    }
    if (!active && f.was_active && f.recorded_onset >= 0.0 &&
        f.recorded_end < 0.0) {
      f.recorded_end = elapsed;
      manifest_dirty = true;
    }
    f.was_active = active;
  }

  // Single-writer update of the in-process effect store consumed by the
  // sensor/drive plugins.
  FaultInjectionRegistry::Get().SetEffects(snapshot);

  if (manifest_dirty) {
    WriteManifest();
  }

  // Throttled out-of-band ground-truth publish on the reserved namespace.
  if (elapsed - last_gt_publish_ >= gt_publish_period_) {
    PublishGroundTruth(now);
    last_gt_publish_ = elapsed;
  }
}

Model *FaultInjector::FindModel(const std::string &name) const {
  if (world_ == nullptr) {
    return nullptr;
  }
  for (Model *m : world_->models_) {
    if (m->GetName() == name) {
      return m;
    }
  }
  return nullptr;
}

double FaultInjector::MeasureCondition(const Fault &fault,
                                       double elapsed) const {
  if (fault.condition_type == "elapsed") {
    return elapsed;
  }
  Model *m = FindModel(fault.model);
  if (m == nullptr || m->bodies_.empty()) {
    return 0.0;
  }
  b2Vec2 pos = m->bodies_[0]->physics_body_->GetPosition();
  if (fault.condition_type == "x_greater") {
    return pos.x;
  }
  if (fault.condition_type == "y_greater") {
    return pos.y;
  }
  if (fault.condition_type == "distance_travelled") {
    auto it = distance_.find(fault.model);
    return it == distance_.end() ? 0.0 : it->second;
  }
  return 0.0;
}

void FaultInjector::UpdateModelMotion() {
  for (auto &kv : distance_) {
    const std::string &name = kv.first;
    Model *m = FindModel(name);
    if (m == nullptr || m->bodies_.empty()) {
      continue;
    }
    b2Vec2 pos = m->bodies_[0]->physics_body_->GetPosition();
    auto last = last_pos_.find(name);
    if (last != last_pos_.end()) {
      b2Vec2 delta = pos - last->second;
      kv.second += delta.Length();
    }
    last_pos_[name] = pos;
  }
}

void FaultInjector::ParseEnvironmentParams(Fault &f, YamlReader &pr) {
  f.env_model_path = pr.Get<std::string>("model", "");
  f.env_spawn_pose = Pose(pr.Get<double>("x0", 0.0), pr.Get<double>("y0", 0.0),
                          pr.Get<double>("yaw0", 0.0));
  f.env_dest_pose =
      Pose(pr.Get<double>("to_x", 0.0), pr.Get<double>("to_y", 0.0),
           pr.Get<double>("to_yaw", 0.0));
  f.env_speed = pr.Get<double>("speed", 0.5);
  // Booleans are carried as 0/1 doubles to stay on the numeric YamlReader path.
  f.env_despawn_on_end = pr.Get<double>("despawn_on_end", 0.0) >= 0.5;
  f.env_spawn_if_absent = pr.Get<double>("spawn_if_absent", 0.0) >= 0.5;
  f.env_spill_cx = pr.Get<double>("center_x", 0.0);
  f.env_spill_cy = pr.Get<double>("center_y", 0.0);
  f.env_spill_radius = pr.Get<double>("radius", 1.0);
  f.env_spill_mu_min = pr.Get<double>("mu_min", 0.3);

  // Optional obstacle path: a list of [x, y] world points.
  YamlReader wp = pr.SubnodeOpt("waypoints", YamlReader::LIST);
  if (!wp.IsNodeNull()) {
    for (int i = 0; i < wp.NodeSize(); i++) {
      std::vector<double> xy =
          wp.Subnode(i, YamlReader::LIST).AsList<double>(2, 2);
      f.env_waypoints.push_back(b2Vec2(xy[0], xy[1]));
    }
  }
}

void FaultInjector::HandleEnvironmentFault(Fault &f, double severity,
                                           bool active,
                                           const Timekeeper &timekeeper) {
  if (world_ == nullptr) {
    return;
  }
  if (active) {
    if (!f.env_applied) {
      ApplyEnvironmentOnset(f);
      f.env_applied = true;
    }
    if (f.kind == FaultKind::kDynamicObstacle) {
      UpdateDynamicObstacle(f, severity, timekeeper);
    } else if (f.kind == FaultKind::kSpill) {
      // Severity scales the multiplier from 1.0 (no effect) toward mu_min at
      // peak, so the spill deepens along the ramp and recovers on ramp-down.
      const double mu = 1.0 - severity * (1.0 - f.env_spill_mu_min);
      try {
        world_->AddSpillRegion(f.id,
                               b2Vec2(f.env_spill_cx, f.env_spill_cy),
                               f.env_spill_radius, mu);
      } catch (const flatland_server::Exception &e) {
        ROS_WARN_NAMED("FaultInjector", "spill %s failed: %s", f.id.c_str(),
                       e.what());
      }
    }
  } else if (f.env_applied && !f.env_ended) {
    ApplyEnvironmentEnd(f);
    f.env_ended = true;
  }
}

void FaultInjector::ApplyEnvironmentOnset(Fault &f) {
  try {
    if (f.kind == FaultKind::kDynamicObstacle) {
      // Neutral, hand-authored-looking name so the obstacle's normal tf/markers
      // reveal nothing a world-file obstacle would not (sealing invariant).
      const std::string unique =
          f.model + "_" + std::to_string(++env_spawn_counter_);
      world_->LoadModel(f.env_model_path, "", unique, f.env_spawn_pose);
      f.env_spawned_name = unique;
      ROS_INFO_NAMED("FaultInjector", "dynamic_obstacle %s spawned model %s",
                     f.id.c_str(), unique.c_str());
    } else if (f.kind == FaultKind::kMovedFurniture) {
      if (FindModel(f.model) != nullptr) {
        world_->MoveModel(f.model, f.env_dest_pose);
      } else if (f.env_spawn_if_absent && !f.env_model_path.empty()) {
        world_->LoadModel(f.env_model_path, "", f.model, f.env_dest_pose);
        f.env_spawned_name = f.model;
      } else {
        ROS_WARN_NAMED("FaultInjector",
                       "moved_furniture %s: target model %s absent and "
                       "spawn_if_absent not set",
                       f.id.c_str(), f.model.c_str());
      }
    }
  } catch (const flatland_server::Exception &e) {
    ROS_WARN_NAMED("FaultInjector", "environment fault %s onset failed: %s",
                   f.id.c_str(), e.what());
  }
}

void FaultInjector::ApplyEnvironmentEnd(Fault &f) {
  if (f.kind == FaultKind::kSpill) {
    world_->RemoveSpillRegion(f.id);
    return;
  }
  if (f.kind == FaultKind::kDynamicObstacle && !f.env_spawned_name.empty()) {
    Model *m = FindModel(f.env_spawned_name);
    if (m != nullptr && !m->bodies_.empty()) {
      m->bodies_[0]->physics_body_->SetLinearVelocity(b2Vec2(0.0f, 0.0f));
    }
    if (f.env_despawn_on_end) {
      try {
        world_->DeleteModel(f.env_spawned_name);
      } catch (const flatland_server::Exception &e) {
        ROS_WARN_NAMED("FaultInjector", "despawn of %s failed: %s",
                       f.env_spawned_name.c_str(), e.what());
      }
      f.env_spawned_name.clear();
    }
  }
}

void FaultInjector::UpdateDynamicObstacle(Fault &f, double severity,
                                          const Timekeeper & /*timekeeper*/) {
  if (f.env_spawned_name.empty()) {
    return;
  }
  Model *m = FindModel(f.env_spawned_name);
  if (m == nullptr || m->bodies_.empty()) {
    return;
  }
  b2Body *body = m->bodies_[0]->physics_body_;

  // A pathless obstacle just sits where it spawned (still a real obstacle).
  if (f.env_waypoints.empty()) {
    body->SetLinearVelocity(b2Vec2(0.0f, 0.0f));
    return;
  }

  const b2Vec2 pos = body->GetPosition();
  b2Vec2 to = f.env_waypoints[f.env_waypoint_idx] - pos;
  double dist = to.Length();

  // Reached the current waypoint -> advance, looping for a continuous patrol so
  // the obstacle keeps crossing the robot's path for the fault duration.
  const double kReachThreshold = 0.1;  // m
  if (dist < kReachThreshold) {
    f.env_waypoint_idx = (f.env_waypoint_idx + 1) % f.env_waypoints.size();
    to = f.env_waypoints[f.env_waypoint_idx] - pos;
    dist = to.Length();
  }

  // Velocity-driven motion (not SetPose) so Box2D resolves contact and the
  // obstacle can nudge/block the robot. Speed scales with severity and is
  // re-asserted each step, so collision impulses are corrected next step rather
  // than knocking the obstacle off its path. Driven from sim time only.
  const double speed = std::max(0.0, f.env_speed) *
                       std::max(0.0, std::min(1.0, severity));
  if (dist > 1e-6 && speed > 0.0) {
    to *= static_cast<float>(1.0 / dist);  // normalize
    body->SetLinearVelocity(
        b2Vec2(static_cast<float>(speed) * to.x,
               static_cast<float>(speed) * to.y));
  } else {
    body->SetLinearVelocity(b2Vec2(0.0f, 0.0f));
  }
}

void FaultInjector::PublishGroundTruth(const ros::Time &stamp) {
  flatland_msgs::FaultGroundTruthArray arr;
  arr.header.stamp = stamp;
  arr.faults.reserve(faults_.size());
  const double base = start_time_.toSec();
  for (const auto &f : faults_) {
    flatland_msgs::FaultGroundTruth g;
    g.fault_id = f.id;
    g.fault_type = f.type_str;
    g.affected_model = f.model;
    g.affected_component = f.component;
    g.affected_topic = f.topic;
    g.onset_time = f.recorded_onset >= 0.0 ? ros::Time(base + f.recorded_onset)
                                           : ros::Time(0);
    g.end_time =
        f.recorded_end >= 0.0 ? ros::Time(base + f.recorded_end) : ros::Time(0);
    g.peak_severity = f.profile.peak_severity;
    g.current_severity = f.current_severity;
    g.trigger_description = f.trigger_description;
    arr.faults.push_back(g);
  }
  ground_truth_pub_.publish(arr);
}

void FaultInjector::WriteManifest() {
  std::ofstream out(ground_truth_path_.c_str(),
                    std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    ROS_WARN_NAMED("FaultInjector",
                   "Could not open sealed ground-truth manifest %s for writing",
                   ground_truth_path_.c_str());
    return;
  }
  out << "{\n  \"faults\": [\n";
  for (size_t i = 0; i < faults_.size(); i++) {
    const Fault &f = faults_[i];
    out << "    {\n";
    out << "      \"fault_id\": \"" << JsonEscape(f.id) << "\",\n";
    out << "      \"fault_type\": \"" << JsonEscape(f.type_str) << "\",\n";
    out << "      \"affected_model\": \"" << JsonEscape(f.model) << "\",\n";
    out << "      \"affected_component\": \"" << JsonEscape(f.component)
        << "\",\n";
    out << "      \"affected_topic\": \"" << JsonEscape(f.topic) << "\",\n";
    out << "      \"onset_time\": " << f.recorded_onset << ",\n";
    out << "      \"end_time\": " << f.recorded_end << ",\n";
    out << "      \"peak_severity\": " << f.profile.peak_severity << ",\n";
    out << "      \"trigger_description\": \""
        << JsonEscape(f.trigger_description) << "\"\n";
    out << "    }" << (i + 1 < faults_.size() ? "," : "") << "\n";
  }
  out << "  ]\n}\n";
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::FaultInjector,
                       flatland_server::WorldPlugin)
