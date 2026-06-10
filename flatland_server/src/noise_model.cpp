/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	noise_model.cpp
 * @brief	Calibrated, context-conditioned per-channel sensor/actuator
 * noise model
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

#include "flatland_server/noise_model.h"

#include <flatland_server/exceptions.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>

namespace flatland_server {

namespace {

// Read an optional double sub-key (default 0.0). yaml-cpp parses JSON numbers.
double OptDouble(const YAML::Node &node, const std::string &key,
                 double def = 0.0) {
  if (!node || !node.IsMap() || !node[key]) {
    return def;
  }
  try {
    return node[key].as<double>();
  } catch (const YAML::Exception &) {
    throw YAMLException("noise_model: key '" + key + "' is not a number");
  }
}

// Parse a {surface_id_string: offset} map into {int: double}.
std::map<int, double> ParseSurfaceOffsets(const YAML::Node &node) {
  std::map<int, double> out;
  if (!node || !node.IsMap()) {
    return out;
  }
  for (const auto &kv : node) {
    int surface_id = 0;
    try {
      surface_id = kv.first.as<int>();
    } catch (const YAML::Exception &) {
      throw YAMLException(
          "noise_model: surface offset key must be an integer id");
    }
    out[surface_id] = kv.second.as<double>();
  }
  return out;
}

}  // namespace

std::shared_ptr<NoiseModel> NoiseModel::Legacy() {
  // Private ctor; make_shared cannot reach it, so construct via new.
  return std::shared_ptr<NoiseModel>(new NoiseModel());
}

std::shared_ptr<NoiseModel> NoiseModel::FromFile(const std::string &path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception &e) {
    throw YAMLException("noise_model: failed to load '" + path +
                        "': " + e.what());
  }

  if (!root || !root.IsMap()) {
    throw YAMLException("noise_model: '" + path +
                        "' is not a JSON/YAML object");
  }

  if (!root["schema_version"]) {
    throw YAMLException("noise_model: '" + path + "' missing schema_version");
  }
  const int version = root["schema_version"].as<int>();
  if (version != kSchemaVersion) {
    throw YAMLException("noise_model: unsupported schema_version " +
                        std::to_string(version) + " (expected " +
                        std::to_string(kSchemaVersion) + ") in '" + path + "'");
  }

  const std::string model_type =
      root["model_type"] ? root["model_type"].as<std::string>() : "";
  if (model_type != "parametric_linear") {
    throw YAMLException("noise_model: unsupported model_type '" + model_type +
                        "' in '" + path + "' (expected parametric_linear)");
  }

  std::shared_ptr<NoiseModel> model(new NoiseModel());
  model->legacy_ = false;

  const YAML::Node channels = root["channels"];
  if (channels && channels.IsMap()) {
    for (const auto &kv : channels) {
      const std::string name = kv.first.as<std::string>();
      const YAML::Node cn = kv.second;
      ChannelModel cm;

      cm.base_std = OptDouble(cn, "base_std");
      const YAML::Node std_coef = cn["std_coef"];
      cm.std_speed = OptDouble(std_coef, "speed");
      cm.std_age = OptDouble(std_coef, "age");
      cm.std_darkness = OptDouble(std_coef, "darkness");
      cm.surface_std_offset = ParseSurfaceOffsets(cn["surface_std_offset"]);

      cm.base_mean = OptDouble(cn, "base_mean");
      const YAML::Node mean_coef = cn["mean_coef"];
      cm.mean_speed = OptDouble(mean_coef, "speed");
      cm.mean_age = OptDouble(mean_coef, "age");
      cm.mean_darkness = OptDouble(mean_coef, "darkness");
      cm.surface_mean_offset = ParseSurfaceOffsets(cn["surface_mean_offset"]);

      model->channels_[name] = cm;
    }
  }

  return model;
}

std::shared_ptr<NoiseModel> NoiseModel::LoadOrLegacy(
    const std::string &path, const std::string &component_key) {
  if (path.empty()) {
    return Legacy();
  }
  try {
    auto model = FromFile(path);
    ROS_INFO_NAMED("NoiseModel", "%s: loaded calibrated noise model from '%s'",
                   component_key.c_str(), path.c_str());
    return model;
  } catch (const YAMLException &e) {
    ROS_WARN_NAMED("NoiseModel",
                   "%s: failed to load noise_model '%s' (%s); falling back to "
                   "legacy constant-std noise",
                   component_key.c_str(), path.c_str(), e.what());
    return Legacy();
  }
}

NoiseParams NoiseModel::GetParams(const std::string &channel,
                                  const NoiseContext &ctx,
                                  double fallback_std) const {
  NoiseParams params;
  auto it = channels_.find(channel);
  if (it == channels_.end()) {
    // Legacy / unmodelled channel: zero-mean Gaussian at the fallback std.
    params.mean = 0.0;
    params.std = std::max(0.0, fallback_std);
    return params;
  }

  const ChannelModel &cm = it->second;
  const double darkness = 1.0 - ctx.lighting;

  double surface_std = 0.0;
  auto so = cm.surface_std_offset.find(ctx.surface_id);
  if (so != cm.surface_std_offset.end()) surface_std = so->second;

  double surface_mean = 0.0;
  auto mo = cm.surface_mean_offset.find(ctx.surface_id);
  if (mo != cm.surface_mean_offset.end()) surface_mean = mo->second;

  params.std = cm.base_std + cm.std_speed * ctx.speed +
               cm.std_age * ctx.sensor_age + cm.std_darkness * darkness +
               surface_std;
  params.std = std::max(0.0, params.std);

  params.mean = cm.base_mean + cm.mean_speed * ctx.speed +
                cm.mean_age * ctx.sensor_age + cm.mean_darkness * darkness +
                surface_mean;
  return params;
}

double NoiseModel::Sample(const std::string &channel, const NoiseContext &ctx,
                          std::default_random_engine &rng,
                          double fallback_std) const {
  const NoiseParams p = GetParams(channel, ctx, fallback_std);
  if (p.std <= 0.0) {
    return p.mean;
  }
  std::normal_distribution<double> dist(p.mean, p.std);
  return dist(rng);
}

}  // namespace flatland_server
