/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	map_mutator.h
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

#ifndef FLATLAND_SERVER_MAP_MUTATOR_H
#define FLATLAND_SERVER_MAP_MUTATOR_H

#include <flatland_server/yaml_reader.h>

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include <vector>

namespace flatland_server {

/**
 * @brief Bounded rigid jitter of the whole map: shift/rotate walls within
 * tolerance so the geometry stays recognizable but is no longer
 * pixel-identical.
 */
struct WallJitterConfig {
  bool enabled = false;
  double max_translation_m = 0.0;  ///< max |dx|,|dy| translation, meters
  double max_rotation_deg = 0.0;   ///< max |rotation| about image center, deg
};

/**
 * @brief Morphological widening/narrowing of corridors. A delta sampled in
 * [min_delta_m, max_delta_m] dilates obstacles (narrows aisles) when positive
 * and erodes them (widens aisles) when negative.
 */
struct AisleWidthConfig {
  bool enabled = false;
  double min_delta_m = 0.0;  ///< most-widening delta (may be negative)
  double max_delta_m = 0.0;  ///< most-narrowing delta (may be positive)
};

/**
 * @brief Add random obstacle blobs in free space and/or remove existing small
 * (non-structural) clutter components.
 */
struct ClutterConfig {
  bool enabled = false;
  int add_count_min = 0;         ///< min blobs to add
  int add_count_max = 0;         ///< max blobs to add
  double remove_fraction = 0.0;  ///< fraction of removable components to clear
  double blob_size_min_m = 0.0;  ///< min blob radius, meters
  double blob_size_max_m = 0.0;  ///< max blob radius, meters
  /// Components with bbox area at/below this (m^2) are eligible for removal and
  /// are never created larger; keeps structural walls untouched.
  double max_component_size_m = 0.5;
};

/**
 * @brief Global scale on clutter activity. target_scale > 1 multiplies the
 * sampled add-count; < 1 thins it. A convenience knob over ClutterConfig.
 */
struct ObstacleDensityConfig {
  bool enabled = false;
  double target_scale = 1.0;
};

/**
 * @brief Parsed `mutation:` block for one layer.
 *
 * Absent or `enabled: false` => Apply() returns the input bitmap untouched, so
 * existing worlds are byte-for-byte identical.
 */
struct MutationConfig {
  bool enabled = false;
  std::string seed_key;  ///< RNG key; defaults to the layer name if empty
  std::string
      manifest_path;      ///< optional sealed JSON sidecar path (out-of-band)
  std::string dump_path;  ///< optional mutated PNG dump path (for inspection)

  WallJitterConfig wall_jitter;
  AisleWidthConfig aisle_width;
  ClutterConfig clutter;
  ObstacleDensityConfig obstacle_density;
};

/**
 * @brief One applied mutation operation, recorded for the sealed manifest.
 */
struct MutationOpRecord {
  std::string type;        ///< "wall_jitter", "aisle_width", "clutter_add", ...
  double magnitude = 0.0;  ///< op-specific magnitude (m, deg, or count)
  double location_x = -1.0;  ///< pixel x of the op, or -1 if global
  double location_y = -1.0;  ///< pixel y of the op, or -1 if global
  std::string detail;        ///< free-form human-readable detail
};

/**
 * @brief Ordered record of everything Apply() did to one layer bitmap. Written
 * out-of-band (never published in-band) so mapping/localization eval knows
 * exactly which novel map a run used.
 */
struct MutationManifest {
  uint32_t global_seed = 0;
  uint32_t derived_seed = 0;
  std::string seed_key;
  std::string layer_name;
  std::vector<MutationOpRecord> ops;
};

/**
 * @brief Deterministic, bitmap-in/bitmap-out procedural map mutation engine.
 *
 * All randomness flows through a caller-supplied std::default_random_engine
 * (from RngManager::DeriveEngine) so a run is fully reproducible for a given
 * global seed + seed_key. Pure functions: no ROS graph, no Box2D, unit-testable
 * in isolation.
 */
class MapMutator {
 public:
  /**
   * @brief Parse a `mutation:` YAML block into a MutationConfig.
   * @param[in] reader YamlReader positioned at the `mutation:` map (may be null
   * node, in which case a disabled config is returned).
   * @return Parsed, range-validated config. Throws YAMLException on bad input.
   */
  static MutationConfig FromConfig(YamlReader& reader);

  /**
   * @brief Apply the configured mutations to an occupancy bitmap.
   * @param[in] bitmap CV_32FC1 occupancy image, values in [0, 1] (as built by
   * Layer::MakeLayer after convertTo).
   * @param[in] resolution Meters per pixel, for converting metric tolerances.
   * @param[in] occupied_thresh Threshold at/above which a pixel is an obstacle.
   * @param[in] cfg Parsed mutation config.
   * @param[in,out] rng Injected engine; all draws come from here.
   * @param[out] out_manifest Optional; populated with the applied op records.
   * @return The mutated bitmap (a clone of the input when disabled).
   */
  static cv::Mat Apply(const cv::Mat& bitmap, double resolution,
                       double occupied_thresh, const MutationConfig& cfg,
                       std::default_random_engine& rng,
                       MutationManifest* out_manifest);

  /**
   * @brief Write a sealed manifest to a JSON sidecar. No-op when path empty.
   * @param[in] manifest The manifest to serialize.
   * @param[in] path Destination path. A write failure warns, never throws.
   */
  static void WriteManifest(const MutationManifest& manifest,
                            const std::string& path);
};

}  // namespace flatland_server

#endif  // FLATLAND_SERVER_MAP_MUTATOR_H
