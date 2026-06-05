/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	map_mutator_test.cpp
 * @brief	Unit tests for the deterministic procedural map mutation engine
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

#include <flatland_server/map_mutator.h>
#include <gtest/gtest.h>

#include <cmath>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>

using flatland_server::MapMutator;
using flatland_server::MutationConfig;
using flatland_server::MutationManifest;
using flatland_server::MutationOpRecord;

namespace {

constexpr double kResolution = 0.05;     // m / pixel
constexpr double kOccupiedThresh = 0.5;  // [0, 1]

// A blank free-space bitmap (CV_32FC1, all 0.0).
cv::Mat FreeMap(int rows, int cols) {
  return cv::Mat::zeros(rows, cols, CV_32FC1);
}

int CountOccupied(const cv::Mat& m) {
  return cv::countNonZero(m >= kOccupiedThresh);
}

int CountFree(const cv::Mat& m) {
  return cv::countNonZero(m < kOccupiedThresh);
}

bool MatsEqual(const cv::Mat& a, const cv::Mat& b) {
  if (a.size() != b.size() || a.type() != b.type()) {
    return false;
  }
  cv::Mat diff;
  cv::absdiff(a, b, diff);
  return cv::countNonZero(diff.reshape(1)) == 0;
}

int CountOps(const MutationManifest& m, const std::string& type) {
  int n = 0;
  for (const auto& op : m.ops) {
    if (op.type == type) {
      n++;
    }
  }
  return n;
}

}  // namespace

/**
 * A disabled config returns the input bitmap untouched, so worlds without a
 * `mutation:` block (or with enabled: false) load byte-for-byte as before.
 */
TEST(MapMutatorTest, disabled_returns_input_unchanged) {
  cv::Mat in = FreeMap(50, 50);
  cv::circle(in, {25, 25}, 5, cv::Scalar(1.0), cv::FILLED);

  MutationConfig cfg;          // enabled defaults to false
  cfg.clutter.enabled = true;  // would otherwise add clutter
  cfg.clutter.add_count_min = 5;
  cfg.clutter.add_count_max = 5;
  cfg.clutter.blob_size_min_m = 0.1;
  cfg.clutter.blob_size_max_m = 0.1;

  std::default_random_engine rng(7);
  cv::Mat out =
      MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng, nullptr);

  EXPECT_TRUE(MatsEqual(in, out));
}

/**
 * Determinism: the same (seed, config, input) yields a byte-identical output.
 */
TEST(MapMutatorTest, same_seed_is_deterministic) {
  cv::Mat in = FreeMap(60, 60);

  MutationConfig cfg;
  cfg.enabled = true;
  cfg.clutter.enabled = true;
  cfg.clutter.add_count_min = 4;
  cfg.clutter.add_count_max = 8;
  cfg.clutter.blob_size_min_m = 0.05;
  cfg.clutter.blob_size_max_m = 0.2;

  std::default_random_engine rng_a(12345);
  std::default_random_engine rng_b(12345);
  cv::Mat out_a =
      MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng_a, nullptr);
  cv::Mat out_b =
      MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng_b, nullptr);

  EXPECT_TRUE(MatsEqual(out_a, out_b));
}

/**
 * Different seeds produce different mutated maps (otherwise we would not be
 * generating novel maps at all).
 */
TEST(MapMutatorTest, different_seed_differs) {
  cv::Mat in = FreeMap(60, 60);

  MutationConfig cfg;
  cfg.enabled = true;
  cfg.clutter.enabled = true;
  cfg.clutter.add_count_min = 6;
  cfg.clutter.add_count_max = 6;
  cfg.clutter.blob_size_min_m = 0.1;
  cfg.clutter.blob_size_max_m = 0.1;

  std::default_random_engine rng_a(1);
  std::default_random_engine rng_b(2);
  cv::Mat out_a =
      MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng_a, nullptr);
  cv::Mat out_b =
      MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng_b, nullptr);

  EXPECT_FALSE(MatsEqual(out_a, out_b));
}

/**
 * wall_jitter translation stays within the configured metric tolerance, as
 * recorded in the manifest.
 */
TEST(MapMutatorTest, wall_jitter_respects_translation_bound) {
  cv::Mat in = FreeMap(80, 80);
  cv::rectangle(in, {20, 20}, {60, 60}, cv::Scalar(1.0), 2);

  MutationConfig cfg;
  cfg.enabled = true;
  cfg.wall_jitter.enabled = true;
  cfg.wall_jitter.max_translation_m = 0.1;
  cfg.wall_jitter.max_rotation_deg = 3.0;

  const double bound = cfg.wall_jitter.max_translation_m * std::sqrt(2.0);
  for (unsigned seed = 0; seed < 25; seed++) {
    std::default_random_engine rng(seed);
    MutationManifest manifest;
    MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng, &manifest);
    ASSERT_EQ(1, CountOps(manifest, "wall_jitter"));
    for (const auto& op : manifest.ops) {
      if (op.type == "wall_jitter") {
        EXPECT_LE(op.magnitude, bound + 1e-9);
      }
    }
  }
}

/**
 * Widening a corridor (negative aisle delta -> erode obstacles) never removes
 * free space, so a corridor can never be disconnected by widening; a bounded
 * narrowing leaves the corridor traversable (free cells remain).
 */
TEST(MapMutatorTest, aisle_width_keeps_corridor_traversable) {
  // Two horizontal walls with a 6px free corridor between them.
  cv::Mat in = FreeMap(40, 40);
  cv::rectangle(in, {0, 10}, {39, 16}, cv::Scalar(1.0), cv::FILLED);
  cv::rectangle(in, {0, 23}, {39, 29}, cv::Scalar(1.0), cv::FILLED);
  const int free_before = CountFree(in);

  // Widen (erode walls): free space must not shrink.
  {
    MutationConfig cfg;
    cfg.enabled = true;
    cfg.aisle_width.enabled = true;
    cfg.aisle_width.min_delta_m = -0.05;  // widen by ~1px
    cfg.aisle_width.max_delta_m = -0.05;
    std::default_random_engine rng(3);
    cv::Mat out =
        MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng, nullptr);
    EXPECT_GE(CountFree(out), free_before);
  }

  // Narrow (dilate walls) by a bounded amount: corridor still has free cells.
  {
    MutationConfig cfg;
    cfg.enabled = true;
    cfg.aisle_width.enabled = true;
    cfg.aisle_width.min_delta_m = 0.05;  // narrow by ~1px each side
    cfg.aisle_width.max_delta_m = 0.05;
    std::default_random_engine rng(3);
    cv::Mat out =
        MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng, nullptr);
    EXPECT_GT(CountFree(out), 0);
  }
}

/**
 * clutter add: a fixed count range adds exactly that many blobs (manifest +
 * occupied-cell growth), and the density scale multiplies the count.
 */
TEST(MapMutatorTest, clutter_add_respects_count_and_density) {
  cv::Mat in = FreeMap(100, 100);

  MutationConfig cfg;
  cfg.enabled = true;
  cfg.clutter.enabled = true;
  cfg.clutter.add_count_min = 3;
  cfg.clutter.add_count_max = 3;
  cfg.clutter.blob_size_min_m = 0.1;
  cfg.clutter.blob_size_max_m = 0.1;

  {
    std::default_random_engine rng(11);
    MutationManifest manifest;
    cv::Mat out = MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng,
                                    &manifest);
    EXPECT_EQ(3, CountOps(manifest, "clutter_add"));
    EXPECT_GT(CountOccupied(out), 0);
  }

  // Density scale x2 -> 6 blobs.
  {
    MutationConfig scaled = cfg;
    scaled.obstacle_density.enabled = true;
    scaled.obstacle_density.target_scale = 2.0;
    std::default_random_engine rng(11);
    MutationManifest manifest;
    MapMutator::Apply(in, kResolution, kOccupiedThresh, scaled, rng, &manifest);
    EXPECT_EQ(6, CountOps(manifest, "clutter_add"));
  }
}

/**
 * clutter remove clears small clutter components but preserves large structural
 * walls (bounded by max_component_size_m).
 */
TEST(MapMutatorTest, clutter_remove_spares_structural_walls) {
  cv::Mat in = FreeMap(120, 120);
  // A large structural wall (well above the size threshold).
  cv::rectangle(in, {0, 0}, {119, 6}, cv::Scalar(1.0), cv::FILLED);
  // A small clutter blob.
  cv::circle(in, {60, 60}, 2, cv::Scalar(1.0), cv::FILLED);
  const int wall_cells_before =
      cv::countNonZero(in.rowRange(0, 7) >= kOccupiedThresh);

  MutationConfig cfg;
  cfg.enabled = true;
  cfg.clutter.enabled = true;
  cfg.clutter.remove_fraction = 1.0;       // remove every eligible component
  cfg.clutter.max_component_size_m = 0.5;  // 10px at 0.05 m/px

  std::default_random_engine rng(5);
  MutationManifest manifest;
  cv::Mat out =
      MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng, &manifest);

  // The small blob is gone...
  EXPECT_LT(out.at<float>(60, 60), kOccupiedThresh);
  EXPECT_GE(CountOps(manifest, "clutter_remove"), 1);
  // ...but the structural wall survives untouched.
  const int wall_cells_after =
      cv::countNonZero(out.rowRange(0, 7) >= kOccupiedThresh);
  EXPECT_EQ(wall_cells_before, wall_cells_after);
}

/**
 * The manifest records one entry per applied op and carries the seed metadata.
 */
TEST(MapMutatorTest, manifest_records_applied_ops) {
  cv::Mat in = FreeMap(80, 80);

  MutationConfig cfg;
  cfg.enabled = true;
  cfg.seed_key = "layer_2d";
  cfg.wall_jitter.enabled = true;
  cfg.wall_jitter.max_translation_m = 0.1;
  cfg.wall_jitter.max_rotation_deg = 2.0;
  cfg.clutter.enabled = true;
  cfg.clutter.add_count_min = 2;
  cfg.clutter.add_count_max = 2;
  cfg.clutter.blob_size_min_m = 0.1;
  cfg.clutter.blob_size_max_m = 0.1;

  std::default_random_engine rng(9);
  MutationManifest manifest;
  manifest.seed_key = cfg.seed_key;
  MapMutator::Apply(in, kResolution, kOccupiedThresh, cfg, rng, &manifest);

  EXPECT_EQ("layer_2d", manifest.seed_key);
  EXPECT_EQ(1, CountOps(manifest, "wall_jitter"));
  EXPECT_EQ(2, CountOps(manifest, "clutter_add"));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
