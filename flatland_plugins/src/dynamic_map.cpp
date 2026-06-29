/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	dynamic_map.cpp
 * @brief	World plugin: scripted mid-episode static-map geometry changes
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

#include <flatland_plugins/dynamic_map.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/layer.h>
#include <flatland_server/random.h>
#include <flatland_server/world.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>

#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace flatland_plugins {

namespace {

// Minimal JSON string escaping for the sealed sidecar.
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

DynamicMap::Op ParseOp(const std::string &s) {
  if (s == "fill") return DynamicMap::Op::kFill;
  if (s == "clear") return DynamicMap::Op::kClear;
  if (s == "translate") return DynamicMap::Op::kTranslate;
  throw flatland_server::YAMLException(
      "DynamicMap: unknown op \"" + s +
      "\" (expected one of: fill, clear, translate)");
}

}  // namespace

void DynamicMap::OnInitialize(const YAML::Node &config) {
  flatland_server::YamlReader reader(config);

  target_layer_ = reader.Get<std::string>("target_layer");
  manifest_path_ = reader.Get<std::string>(
      "manifest_path", "/tmp/flatland_dynamic_map_ground_truth.json");
  seed_key_ = reader.Get<std::string>("seed_key", "dynamic_map");
  seed_ = flatland_server::RngManager::Get().GetSeed();

  flatland_server::YamlReader events_reader =
      reader.SubnodeOpt("events", flatland_server::YamlReader::LIST);
  if (!events_reader.IsNodeNull()) {
    for (int i = 0; i < events_reader.NodeSize(); i++) {
      flatland_server::YamlReader er =
          events_reader.Subnode(i, flatland_server::YamlReader::MAP);
      Event e;
      e.id = er.Get<std::string>("id", "event_" + std::to_string(i));
      e.op_str = er.Get<std::string>("op");
      e.op = ParseOp(e.op_str);

      flatland_server::YamlReader tr =
          er.Subnode("trigger", flatland_server::YamlReader::MAP);
      e.time_s = tr.Get<double>("time_s");
      tr.EnsureAccessedAllKeys();

      std::array<double, 4> region = er.GetArray<double, 4>("region");
      e.rx = region[0];
      e.ry = region[1];
      e.rw = region[2];
      e.rh = region[3];
      if (e.rw <= 0.0 || e.rh <= 0.0) {
        throw flatland_server::YAMLException(
            "DynamicMap: event \"" + e.id +
            "\" region width/height must be positive");
      }

      if (e.op == Op::kTranslate) {
        std::array<double, 2> dest = er.GetArray<double, 2>("dest");
        e.dx = dest[0];
        e.dy = dest[1];
      }

      er.EnsureAccessedAllKeys();
      events_.push_back(e);
    }
  }

  reader.EnsureAccessedAllKeys();

  // Resolve the target layer once; geometry edits are no-ops if it is missing
  // or carries no image bitmap (line-segment / empty layers).
  layer_ = world_->GetLayer(target_layer_);
  if (layer_ == nullptr) {
    ROS_WARN_NAMED("DynamicMap",
                   "target_layer \"%s\" not found; no map edits will apply",
                   target_layer_.c_str());
  } else if (layer_->GetBitmap().empty()) {
    ROS_WARN_NAMED("DynamicMap",
                   "target_layer \"%s\" has no occupancy bitmap "
                   "(non-image layer); no map edits will apply",
                   target_layer_.c_str());
    layer_ = nullptr;
  }

  // Seal the manifest up-front so the file always exists (onsets unresolved).
  WriteManifest();

  ROS_INFO_NAMED("DynamicMap",
                 "Loaded %zu scripted map event(s) for layer \"%s\"; sealed "
                 "manifest -> %s",
                 events_.size(), target_layer_.c_str(), manifest_path_.c_str());
}

void DynamicMap::BeforePhysicsStep(
    const flatland_server::Timekeeper &timekeeper) {
  const ros::Time now = timekeeper.GetSimTime();
  if (!started_) {
    start_time_ = now;
    started_ = true;
  }
  const double elapsed = (now - start_time_).toSec();

  if (layer_ == nullptr) {
    return;
  }

  bool fired = false;
  for (Event &e : events_) {
    if (e.applied || elapsed < e.time_s) {
      continue;
    }

    // Read-modify-write the layer's current bitmap, then atomically swap the
    // collision geometry. Done here (between steps), never in a contact
    // callback, and only on the fire edge so there is no per-step cost.
    cv::Mat bitmap = layer_->GetBitmap().clone();
    ApplyEvent(e, bitmap);
    layer_->RebuildCollisionFromBitmap(bitmap, layer_->GetOccupiedThresh(),
                                       layer_->GetResolution());

    e.applied = true;
    e.applied_at = elapsed;
    fired = true;
    ROS_INFO_NAMED(
        "DynamicMap", "Applied event \"%s\" (op=%s) on layer \"%s\" at t=%.3fs",
        e.id.c_str(), e.op_str.c_str(), target_layer_.c_str(), elapsed);
  }

  if (fired) {
    // Refresh the latched occupancy overlay so rviz / diagnostics reflect the
    // new geometry, and re-seal the manifest with the resolved onset times.
    world_->RepublishLayerOccupancy();
    WriteManifest();
  }
}

bool DynamicMap::WorldRectToPixels(double wx, double wy, double ww, double wh,
                                   cv::Rect &out) const {
  const cv::Mat &bitmap = layer_->GetBitmap();
  const double res = layer_->GetResolution();
  const flatland_server::Pose &origin = layer_->GetOrigin();
  const int rows = bitmap.rows;
  const int cols = bitmap.cols;
  if (res <= 0.0 || rows < 1 || cols < 1) {
    out = cv::Rect();
    return false;
  }

  // Inverse of the Layer transform: col = (wx - ox) / res, and world y grows
  // upward while image rows grow downward, so row = rows - (wy - oy) / res.
  const double col_lo = (wx - origin.x) / res;
  const double col_hi = (wx + ww - origin.x) / res;
  const double row_bottom = rows - (wy - origin.y) / res;    // larger row
  const double row_top = rows - (wy + wh - origin.y) / res;  // smaller row

  cv::Rect rect(static_cast<int>(std::lround(col_lo)),
                static_cast<int>(std::lround(row_top)),
                static_cast<int>(std::lround(col_hi - col_lo)),
                static_cast<int>(std::lround(row_bottom - row_top)));

  // Clamp to the image so an edit near a border never reads/writes out of
  // range.
  out = rect & cv::Rect(0, 0, cols, rows);
  return out.width > 0 && out.height > 0;
}

void DynamicMap::ApplyEvent(const Event &e, cv::Mat &bitmap) const {
  // Flatland occupancy convention: a pixel value >= occupied_thresh is an
  // obstacle, so 1.0 == occupied and 0.0 == free.
  cv::Rect src;
  if (!WorldRectToPixels(e.rx, e.ry, e.rw, e.rh, src)) {
    ROS_WARN_NAMED("DynamicMap",
                   "event \"%s\" region falls outside the map; skipped",
                   e.id.c_str());
    return;
  }

  switch (e.op) {
    case Op::kFill:
      bitmap(src).setTo(1.0);
      break;
    case Op::kClear:
      bitmap(src).setTo(0.0);
      break;
    case Op::kTranslate: {
      cv::Rect dst;
      if (!WorldRectToPixels(e.dx, e.dy, e.rw, e.rh, dst)) {
        ROS_WARN_NAMED("DynamicMap",
                       "event \"%s\" destination falls outside the map; "
                       "source cleared only",
                       e.id.c_str());
        bitmap(src).setTo(0.0);
        break;
      }
      // Move the obstacle content: snapshot the source, clear it (now free),
      // then stamp the snapshot at the destination (resized on rounding skew).
      cv::Mat patch = bitmap(src).clone();
      bitmap(src).setTo(0.0);
      if (patch.size() != dst.size()) {
        cv::resize(patch, patch, dst.size(), 0, 0, cv::INTER_NEAREST);
      }
      patch.copyTo(bitmap(dst));
      break;
    }
  }
}

void DynamicMap::WriteManifest() const {
  if (manifest_path_.empty()) {
    return;
  }
  std::ofstream out(manifest_path_.c_str(), std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    ROS_WARN_NAMED("DynamicMap",
                   "Could not open sealed manifest %s for writing",
                   manifest_path_.c_str());
    return;
  }
  out << "{\n";
  out << "  \"global_seed\": " << seed_ << ",\n";
  out << "  \"seed_key\": \"" << JsonEscape(seed_key_) << "\",\n";
  out << "  \"target_layer\": \"" << JsonEscape(target_layer_) << "\",\n";
  out << "  \"events\": [\n";
  for (size_t i = 0; i < events_.size(); i++) {
    const Event &e = events_[i];
    out << "    {\n";
    out << "      \"id\": \"" << JsonEscape(e.id) << "\",\n";
    out << "      \"op\": \"" << JsonEscape(e.op_str) << "\",\n";
    out << "      \"trigger_time_s\": " << e.time_s << ",\n";
    out << "      \"applied_at_s\": " << e.applied_at << ",\n";
    out << "      \"region_m\": [" << e.rx << ", " << e.ry << ", " << e.rw
        << ", " << e.rh << "],\n";
    out << "      \"dest_m\": [" << e.dx << ", " << e.dy << "]\n";
    out << "    }" << (i + 1 < events_.size() ? "," : "") << "\n";
  }
  out << "  ]\n}\n";
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::DynamicMap,
                       flatland_server::WorldPlugin)
