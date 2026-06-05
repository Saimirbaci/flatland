/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	map_mutator.cpp
 * @brief	Deterministic procedural perturbation of layer occupancy bitmaps
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

#include <flatland_server/exceptions.h>
#include <flatland_server/map_mutator.h>
#include <ros/ros.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace flatland_server {

namespace {

// Minimal JSON string escaping for the sealed sidecar (paths/keys are plain
// identifiers in practice, but escape defensively).
std::string JsonEscape(const std::string& s) {
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

// Parse one [min, max] range block, validating min <= max. Returns false (and
// leaves out_min/out_max untouched) when the key is absent.
bool ReadRange(YamlReader& reader, const std::string& key, double& out_min,
               double& out_max) {
  YamlReader sub = reader.SubnodeOpt(key, YamlReader::LIST);
  if (sub.IsNodeNull()) {
    return false;
  }
  std::array<double, 2> range = sub.AsArray<double, 2>();
  if (range[0] > range[1]) {
    throw YAMLException("mutation \"" + key + "\" range min (" +
                        std::to_string(range[0]) + ") exceeds max (" +
                        std::to_string(range[1]) + ")");
  }
  out_min = range[0];
  out_max = range[1];
  return true;
}

double SampleUniform(std::default_random_engine& rng, double lo, double hi) {
  if (hi <= lo) {
    return lo;
  }
  std::uniform_real_distribution<double> dist(lo, hi);
  return dist(rng);
}

}  // namespace

MutationConfig MapMutator::FromConfig(YamlReader& reader) {
  MutationConfig cfg;
  if (reader.IsNodeNull()) {
    return cfg;  // disabled, byte-for-byte identical behavior
  }

  cfg.enabled = reader.Get<bool>("enabled", false);
  cfg.seed_key = reader.Get<std::string>("seed_key", "");
  cfg.manifest_path = reader.Get<std::string>("manifest_path", "");
  cfg.dump_path = reader.Get<std::string>("dump_path", "");

  // --- wall_jitter -----------------------------------------------------------
  YamlReader wj = reader.SubnodeOpt("wall_jitter", YamlReader::MAP);
  if (!wj.IsNodeNull()) {
    cfg.wall_jitter.enabled = true;
    cfg.wall_jitter.max_translation_m =
        wj.Get<double>("max_translation_m", 0.0);
    cfg.wall_jitter.max_rotation_deg = wj.Get<double>("max_rotation_deg", 0.0);
    if (cfg.wall_jitter.max_translation_m < 0.0 ||
        cfg.wall_jitter.max_rotation_deg < 0.0) {
      throw YAMLException(
          "mutation \"wall_jitter\" max_translation_m/max_rotation_deg must be "
          "non-negative");
    }
    wj.EnsureAccessedAllKeys();
  }

  // --- aisle_width -----------------------------------------------------------
  YamlReader aw = reader.SubnodeOpt("aisle_width", YamlReader::MAP);
  if (!aw.IsNodeNull()) {
    cfg.aisle_width.enabled = true;
    if (!ReadRange(aw, "dilate_erode_range_m", cfg.aisle_width.min_delta_m,
                   cfg.aisle_width.max_delta_m)) {
      throw YAMLException(
          "mutation \"aisle_width\" requires a \"dilate_erode_range_m\" "
          "[min, max] list");
    }
    aw.EnsureAccessedAllKeys();
  }

  // --- clutter ---------------------------------------------------------------
  YamlReader cl = reader.SubnodeOpt("clutter", YamlReader::MAP);
  if (!cl.IsNodeNull()) {
    cfg.clutter.enabled = true;
    std::array<int, 2> add = cl.GetArray<int, 2>("add_count_range", {{0, 0}});
    if (add[0] < 0 || add[1] < add[0]) {
      throw YAMLException(
          "mutation \"clutter\" add_count_range must be non-negative and "
          "ascending");
    }
    cfg.clutter.add_count_min = add[0];
    cfg.clutter.add_count_max = add[1];

    cfg.clutter.remove_fraction = cl.Get<double>("remove_fraction", 0.0);
    if (cfg.clutter.remove_fraction < 0.0 ||
        cfg.clutter.remove_fraction > 1.0) {
      throw YAMLException(
          "mutation \"clutter\" remove_fraction must be within [0, 1]");
    }

    double blob_min = 0.0;
    double blob_max = 0.0;
    if (ReadRange(cl, "blob_size_m_range", blob_min, blob_max)) {
      if (blob_min < 0.0) {
        throw YAMLException(
            "mutation \"clutter\" blob_size_m_range must be non-negative");
      }
      cfg.clutter.blob_size_min_m = blob_min;
      cfg.clutter.blob_size_max_m = blob_max;
    }
    cfg.clutter.max_component_size_m =
        cl.Get<double>("max_component_size_m", 0.5);
    if (cfg.clutter.max_component_size_m < 0.0) {
      throw YAMLException(
          "mutation \"clutter\" max_component_size_m must be non-negative");
    }
    cl.EnsureAccessedAllKeys();
  }

  // --- obstacle_density ------------------------------------------------------
  YamlReader od = reader.SubnodeOpt("obstacle_density", YamlReader::MAP);
  if (!od.IsNodeNull()) {
    cfg.obstacle_density.enabled = true;
    cfg.obstacle_density.target_scale = od.Get<double>("target_scale", 1.0);
    if (cfg.obstacle_density.target_scale < 0.0) {
      throw YAMLException(
          "mutation \"obstacle_density\" target_scale must be non-negative");
    }
    od.EnsureAccessedAllKeys();
  }

  reader.EnsureAccessedAllKeys();
  return cfg;
}

cv::Mat MapMutator::Apply(const cv::Mat& bitmap, double resolution,
                          double occupied_thresh, const MutationConfig& cfg,
                          std::default_random_engine& rng,
                          MutationManifest* out_manifest) {
  cv::Mat work = bitmap.clone();
  if (!cfg.enabled || resolution <= 0.0) {
    return work;
  }

  std::vector<MutationOpRecord> ops;

  // (1) wall_jitter: bounded rigid shift + rotation of the whole map.
  if (cfg.wall_jitter.enabled && (cfg.wall_jitter.max_translation_m > 0.0 ||
                                  cfg.wall_jitter.max_rotation_deg > 0.0)) {
    const double max_t_px = cfg.wall_jitter.max_translation_m / resolution;
    const double tx = SampleUniform(rng, -max_t_px, max_t_px);
    const double ty = SampleUniform(rng, -max_t_px, max_t_px);
    const double rot_deg = SampleUniform(rng, -cfg.wall_jitter.max_rotation_deg,
                                         cfg.wall_jitter.max_rotation_deg);

    cv::Point2f center(static_cast<float>(work.cols) / 2.0f,
                       static_cast<float>(work.rows) / 2.0f);
    cv::Mat rot = cv::getRotationMatrix2D(center, rot_deg, 1.0);
    rot.at<double>(0, 2) += tx;
    rot.at<double>(1, 2) += ty;
    // Free space (0.0) fills any exposed border so jitter never invents walls.
    cv::warpAffine(work, work, rot, work.size(), cv::INTER_NEAREST,
                   cv::BORDER_CONSTANT, cv::Scalar(0.0));

    MutationOpRecord rec;
    rec.type = "wall_jitter";
    rec.magnitude = std::hypot(tx, ty) * resolution;
    rec.detail = "tx_m=" + std::to_string(tx * resolution) +
                 " ty_m=" + std::to_string(ty * resolution) +
                 " rot_deg=" + std::to_string(rot_deg);
    ops.push_back(rec);
  }

  // (2) aisle_width: dilate (narrow) or erode (widen) corridors.
  if (cfg.aisle_width.enabled) {
    const double delta_m = SampleUniform(rng, cfg.aisle_width.min_delta_m,
                                         cfg.aisle_width.max_delta_m);
    const int radius_px =
        static_cast<int>(std::round(std::abs(delta_m) / resolution));
    if (radius_px >= 1) {
      cv::Mat kernel = cv::getStructuringElement(
          cv::MORPH_ELLIPSE, {radius_px * 2 + 1, radius_px * 2 + 1});
      if (delta_m > 0.0) {
        cv::dilate(work, work, kernel);  // grow obstacles -> narrower aisles
      } else {
        cv::erode(work, work, kernel);  // shrink obstacles -> wider aisles
      }
      MutationOpRecord rec;
      rec.type = "aisle_width";
      rec.magnitude = delta_m;
      rec.detail = (delta_m > 0.0 ? "narrow_m=" : "widen_m=") +
                   std::to_string(std::abs(delta_m));
      ops.push_back(rec);
    }
  }

  // (3) clutter remove: clear small (non-structural) obstacle components.
  if (cfg.clutter.enabled && cfg.clutter.remove_fraction > 0.0) {
    cv::Mat mask = (work >= occupied_thresh);  // CV_8U 0/255
    cv::Mat labels, stats, centroids;
    const int num = cv::connectedComponentsWithStats(mask, labels, stats,
                                                     centroids, 8, CV_32S);
    const double max_dim_px = cfg.clutter.max_component_size_m / resolution;
    for (int label = 1; label < num; label++) {
      const int w = stats.at<int>(label, cv::CC_STAT_WIDTH);
      const int h = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      // Only small blobs are removable; large structural walls are preserved.
      if (w > max_dim_px || h > max_dim_px) {
        continue;
      }
      if (SampleUniform(rng, 0.0, 1.0) >= cfg.clutter.remove_fraction) {
        continue;
      }
      work.setTo(0.0, labels == label);
      MutationOpRecord rec;
      rec.type = "clutter_remove";
      rec.magnitude = static_cast<double>(w) * h;
      rec.location_x = centroids.at<double>(label, 0);
      rec.location_y = centroids.at<double>(label, 1);
      ops.push_back(rec);
    }
  }

  // (4) clutter add: drop random obstacle blobs in free space.
  if (cfg.clutter.enabled && cfg.clutter.add_count_max > 0) {
    std::uniform_int_distribution<int> count_dist(cfg.clutter.add_count_min,
                                                  cfg.clutter.add_count_max);
    int count = count_dist(rng);
    if (cfg.obstacle_density.enabled) {
      count = static_cast<int>(
          std::round(count * cfg.obstacle_density.target_scale));
    }

    // Collect free cells once; sampling indices keeps draws deterministic.
    std::vector<cv::Point> free_cells;
    for (int r = 0; r < work.rows; r++) {
      const float* row = work.ptr<float>(r);
      for (int c = 0; c < work.cols; c++) {
        if (row[c] < occupied_thresh) {
          free_cells.emplace_back(c, r);
        }
      }
    }

    if (!free_cells.empty()) {
      std::uniform_int_distribution<size_t> idx_dist(0, free_cells.size() - 1);
      for (int i = 0; i < count; i++) {
        const cv::Point center = free_cells[idx_dist(rng)];
        const double radius_m = SampleUniform(rng, cfg.clutter.blob_size_min_m,
                                              cfg.clutter.blob_size_max_m);
        const int radius_px =
            std::max(1, static_cast<int>(std::round(radius_m / resolution)));
        cv::circle(work, center, radius_px, cv::Scalar(1.0), cv::FILLED);
        MutationOpRecord rec;
        rec.type = "clutter_add";
        rec.magnitude = radius_m;
        rec.location_x = center.x;
        rec.location_y = center.y;
        ops.push_back(rec);
      }
    }
  }

  // Keep the bitmap a valid occupancy image in [0, 1].
  cv::threshold(work, work, 1.0, 1.0, cv::THRESH_TRUNC);
  cv::threshold(work, work, 0.0, 0.0, cv::THRESH_TOZERO);

  ROS_INFO_NAMED("MapMutator", "applied %zu mutation op(s) (seed_key=\"%s\")",
                 ops.size(), cfg.seed_key.c_str());

  if (out_manifest != nullptr) {
    out_manifest->seed_key = cfg.seed_key;
    out_manifest->ops = ops;
  }

  if (!cfg.dump_path.empty()) {
    cv::Mat dump;
    work.convertTo(dump, CV_8UC1, 255.0);
    if (!cv::imwrite(cfg.dump_path, dump)) {
      ROS_WARN_NAMED("MapMutator", "Could not write mutated map dump to %s",
                     cfg.dump_path.c_str());
    }
  }

  return work;
}

void MapMutator::WriteManifest(const MutationManifest& manifest,
                               const std::string& path) {
  if (path.empty()) {
    return;
  }
  std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    ROS_WARN_NAMED("MapMutator",
                   "Could not open sealed mutation manifest %s for writing",
                   path.c_str());
    return;
  }
  out << "{\n";
  out << "  \"global_seed\": " << manifest.global_seed << ",\n";
  out << "  \"derived_seed\": " << manifest.derived_seed << ",\n";
  out << "  \"seed_key\": \"" << JsonEscape(manifest.seed_key) << "\",\n";
  out << "  \"layer_name\": \"" << JsonEscape(manifest.layer_name) << "\",\n";
  out << "  \"operations\": [\n";
  for (size_t i = 0; i < manifest.ops.size(); i++) {
    const MutationOpRecord& op = manifest.ops[i];
    out << "    {\n";
    out << "      \"type\": \"" << JsonEscape(op.type) << "\",\n";
    out << "      \"magnitude\": " << op.magnitude << ",\n";
    out << "      \"location_x\": " << op.location_x << ",\n";
    out << "      \"location_y\": " << op.location_y << ",\n";
    out << "      \"detail\": \"" << JsonEscape(op.detail) << "\"\n";
    out << "    }" << (i + 1 < manifest.ops.size() ? "," : "") << "\n";
  }
  out << "  ]\n}\n";
}

}  // namespace flatland_server
