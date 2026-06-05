/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   surface_friction_field_test.cpp
 * @brief  Unit tests for the smoothly-sampled surface friction field
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

#include <flatland_server/surface_friction_field.h>
#include <gtest/gtest.h>

#include <vector>

using flatland_server::SurfaceFrictionField;

/**
 * A default-constructed (empty) field is disabled and returns the nominal 1.0
 * everywhere, so worlds without a surface_friction block are unaffected.
 */
TEST(SurfaceFrictionFieldTest, empty_field_is_nominal) {
  SurfaceFrictionField field;
  EXPECT_FALSE(field.Enabled());
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(0.0f, 0.0f)), 1e-9);
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(123.0f, -45.0f)), 1e-9);
}

/**
 * Each region's configured multiplier is reproduced exactly at the cell centre.
 * Grid (1 row, 4 cols, 1 m cells, origin (0,0)): dry, dry, wet, wet.
 */
TEST(SurfaceFrictionFieldTest, reproduces_value_at_region_center) {
  std::vector<double> grid = {1.0, 1.0, 0.2, 0.2};
  SurfaceFrictionField field(grid, 4, 1, 1.0, 0.0, 0.0);
  EXPECT_TRUE(field.Enabled());

  // Cell centres are at x = col + 0.5.
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(0.5f, 0.5f)), 1e-9);  // dry
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(1.5f, 0.5f)), 1e-9);  // dry
  EXPECT_NEAR(0.2, field.GetFrictionFactor(b2Vec2(2.5f, 0.5f)), 1e-9);  // wet
  EXPECT_NEAR(0.2, field.GetFrictionFactor(b2Vec2(3.5f, 0.5f)), 1e-9);  // wet
}

/**
 * The anti-step-change guarantee: sampling a dense line of points across the
 * dry->wet boundary, no two adjacent samples jump by more than a small epsilon,
 * and the multiplier decreases monotonically through the transition.
 */
TEST(SurfaceFrictionFieldTest, transition_is_continuous_and_monotonic) {
  std::vector<double> grid = {1.0, 1.0, 0.2, 0.2};
  SurfaceFrictionField field(grid, 4, 1, 1.0, 0.0, 0.0);

  const double step = 0.01;
  double prev = field.GetFrictionFactor(b2Vec2(0.5f, 0.5f));
  for (double x = 0.5 + step; x <= 3.5; x += step) {
    double cur = field.GetFrictionFactor(b2Vec2(static_cast<float>(x), 0.5f));
    // Continuity: adjacent samples never step by more than the per-step slope
    // can produce (max gradient 0.8 / cell over a 1 m cell -> 0.008 per step).
    EXPECT_LT(std::fabs(cur - prev), 0.05) << "discontinuity near x=" << x;
    // Monotonic non-increasing from dry to wet.
    EXPECT_LE(cur, prev + 1e-9) << "non-monotonic near x=" << x;
    prev = cur;
  }
}

/**
 * Bilinear interpolation matches a hand-computed value at the centre of a 2x2
 * grid. Rows are stored top-down (row 0 = max y), matching the occupancy map
 * convention.
 */
TEST(SurfaceFrictionFieldTest, bilinear_matches_hand_value) {
  // row 0 (top, high y): 0.2, 0.4 ; row 1 (bottom, low y): 0.6, 0.8
  std::vector<double> grid = {0.2, 0.4, 0.6, 0.8};
  SurfaceFrictionField field(grid, 2, 2, 1.0, 0.0, 0.0);

  // Centre of the grid (x=1, y=1): equal blend of all four corners.
  // top = 0.3, bottom = 0.7, value = 0.5.
  EXPECT_NEAR(0.5, field.GetFrictionFactor(b2Vec2(1.0f, 1.0f)), 1e-9);

  // Top-left cell centre (x=0.5, y=1.5) -> exactly the stored 0.2.
  EXPECT_NEAR(0.2, field.GetFrictionFactor(b2Vec2(0.5f, 1.5f)), 1e-9);
  // Bottom-right cell centre (x=1.5, y=0.5) -> exactly the stored 0.8.
  EXPECT_NEAR(0.8, field.GetFrictionFactor(b2Vec2(1.5f, 0.5f)), 1e-9);
}

/**
 * Queries outside the raster extent are ambient (1.0), in every direction.
 */
TEST(SurfaceFrictionFieldTest, outside_extent_is_nominal) {
  std::vector<double> grid = {0.2, 0.2, 0.2, 0.2};
  SurfaceFrictionField field(grid, 2, 2, 1.0, 0.0, 0.0);  // extent [0,2]x[0,2]

  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(-0.5f, 1.0f)), 1e-9);
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(2.5f, 1.0f)), 1e-9);
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(1.0f, -0.5f)), 1e-9);
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(1.0f, 2.5f)), 1e-9);
  // Inside, the wet value is returned.
  EXPECT_NEAR(0.2, field.GetFrictionFactor(b2Vec2(1.0f, 1.0f)), 1e-9);
}

/**
 * The sampled multiplier is clamped to a positive floor, so a region can never
 * produce zero (frictionless) traction even if its configured value is lower.
 */
TEST(SurfaceFrictionFieldTest, clamps_to_positive_floor) {
  std::vector<double> grid = {0.001, 0.001, 0.001, 0.001};
  const double min_factor = 0.05;
  SurfaceFrictionField field(grid, 2, 2, 1.0, 0.0, 0.0, min_factor);

  double f = field.GetFrictionFactor(b2Vec2(1.0f, 1.0f));
  EXPECT_NEAR(min_factor, f, 1e-9);
  EXPECT_GT(f, 0.0);
}

/**
 * Origin offset and resolution are honoured: a non-unit grid maps world points
 * to the correct cells.
 */
TEST(SurfaceFrictionFieldTest, honors_origin_and_resolution) {
  // 2x1 grid, 0.5 m cells, origin (-1, -1). Extent x[-1, 0], y[-1, -0.5].
  std::vector<double> grid = {0.3, 0.9};
  SurfaceFrictionField field(grid, 2, 1, 0.5, -1.0, -1.0);

  // Cell centres at x = -1 + (col + 0.5)*0.5 = -0.75 and -0.25.
  EXPECT_NEAR(0.3, field.GetFrictionFactor(b2Vec2(-0.75f, -0.75f)), 1e-9);
  EXPECT_NEAR(0.9, field.GetFrictionFactor(b2Vec2(-0.25f, -0.75f)), 1e-9);
  // Halfway between the two cell centres -> mean.
  EXPECT_NEAR(0.6, field.GetFrictionFactor(b2Vec2(-0.5f, -0.75f)), 1e-9);
}

/**
 * A disabled field produces only a DELETEALL marker, so the diagnostic overlay
 * clears cleanly and never draws geometry for worlds without surface friction.
 */
TEST(SurfaceFrictionFieldTest, marker_array_disabled_is_delete_all) {
  SurfaceFrictionField field;
  visualization_msgs::MarkerArray array = field.ToMarkerArray("map");

  ASSERT_EQ(1u, array.markers.size());
  EXPECT_EQ(visualization_msgs::Marker::DELETEALL, array.markers[0].action);
}

/**
 * Only the sub-nominal (slippery) cells become points in the CUBE_LIST overlay;
 * the dry baseline cells are skipped. The two wet cells of a dry/dry/wet/wet
 * row yield two points at their cell centres, coloured toward red.
 */
TEST(SurfaceFrictionFieldTest, marker_array_emits_only_slippery_cells) {
  std::vector<double> grid = {1.0, 1.0, 0.2, 0.2};
  SurfaceFrictionField field(grid, 4, 1, 1.0, 0.0, 0.0);

  visualization_msgs::MarkerArray array = field.ToMarkerArray("map");

  // DELETEALL marker followed by the CUBE_LIST.
  ASSERT_EQ(2u, array.markers.size());
  const visualization_msgs::Marker& cubes = array.markers[1];
  EXPECT_EQ("map", cubes.header.frame_id);
  EXPECT_EQ(visualization_msgs::Marker::CUBE_LIST, cubes.type);

  // Two wet cells -> two points (+ matching per-point colours).
  ASSERT_EQ(2u, cubes.points.size());
  ASSERT_EQ(2u, cubes.colors.size());

  // Cell centres at x = 2.5 and 3.5, y = 0.5 (single row).
  EXPECT_NEAR(2.5, cubes.points[0].x, 1e-6);
  EXPECT_NEAR(0.5, cubes.points[0].y, 1e-6);
  EXPECT_NEAR(3.5, cubes.points[1].x, 1e-6);

  // Slippery -> more red than green.
  EXPECT_GT(cubes.colors[0].r, cubes.colors[0].g);
}

// --- Runtime spill overlays -------------------------------------------------

/**
 * A spill activated on an otherwise-empty field makes it Enabled() and slippery
 * inside the radius while staying nominal outside; removing it restores the
 * clean-run 1.0-everywhere invariant.
 */
TEST(SurfaceFrictionFieldTest, spill_overlay_activates_and_clears) {
  SurfaceFrictionField field;
  EXPECT_FALSE(field.Enabled());

  field.AddCircularRegion("spill_1", b2Vec2(0.0f, 0.0f), 2.0, 0.1);
  EXPECT_TRUE(field.Enabled());

  // Core (flat) is the configured multiplier.
  EXPECT_NEAR(0.1, field.GetFrictionFactor(b2Vec2(0.0f, 0.0f)), 1e-9);
  EXPECT_NEAR(0.1, field.GetFrictionFactor(b2Vec2(0.5f, 0.0f)), 1e-9);  // core
  // Outside the radius is ambient.
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(2.5f, 0.0f)), 1e-9);
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(0.0f, 3.0f)), 1e-9);

  field.RemoveCircularRegion("spill_1");
  EXPECT_FALSE(field.Enabled());
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(0.0f, 0.0f)), 1e-9);
}

/**
 * The spill edge feathers continuously (C0) from the core multiplier back to
 * 1.0 at the radius, so the traction solver never sees a step change crossing
 * the boundary, and the multiplier increases monotonically from core to edge.
 */
TEST(SurfaceFrictionFieldTest, spill_edge_is_continuous_and_monotonic) {
  SurfaceFrictionField field;
  field.AddCircularRegion("spill_1", b2Vec2(0.0f, 0.0f), 2.0, 0.2);

  const double dstep = 0.005;
  double prev = field.GetFrictionFactor(b2Vec2(0.0f, 0.0f));
  for (double d = dstep; d <= 2.5; d += dstep) {
    double cur = field.GetFrictionFactor(b2Vec2(static_cast<float>(d), 0.0f));
    EXPECT_LT(std::fabs(cur - prev), 0.05) << "spill discontinuity at d=" << d;
    EXPECT_GE(cur, prev - 1e-9) << "spill non-monotonic at d=" << d;
    prev = cur;
  }
  // Exactly at the edge the overlay has feathered fully back to nominal.
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(2.0f, 0.0f)), 1e-9);
}

/**
 * Re-adding a spill with the same id updates it in place (severity ramp), and a
 * spill composes with the static raster by taking the slipperier of the two.
 */
TEST(SurfaceFrictionFieldTest, spill_updates_in_place_and_composes) {
  // Static raster: uniformly dry (1.0) 4x4 grid over [0,4]x[0,4].
  std::vector<double> grid(16, 1.0);
  SurfaceFrictionField field(grid, 4, 4, 1.0, 0.0, 0.0);
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(2.0f, 2.0f)), 1e-9);

  field.AddCircularRegion("spill_1", b2Vec2(2.0f, 2.0f), 1.0, 0.5);
  EXPECT_NEAR(0.5, field.GetFrictionFactor(b2Vec2(2.0f, 2.0f)), 1e-9);

  // Same id -> replaced, not duplicated; the slipperier value now applies.
  field.AddCircularRegion("spill_1", b2Vec2(2.0f, 2.0f), 1.0, 0.2);
  EXPECT_NEAR(0.2, field.GetFrictionFactor(b2Vec2(2.0f, 2.0f)), 1e-9);

  field.RemoveCircularRegion("spill_1");
  // Raster remains; back to the dry baseline.
  EXPECT_NEAR(1.0, field.GetFrictionFactor(b2Vec2(2.0f, 2.0f)), 1e-9);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
