/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   surface_friction_field.h
 * @brief  World-owned, smoothly-sampled per-region surface friction multiplier
 *         field (wet patches, spills, ramps) loaded from the world YAML
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

#ifndef FLATLAND_SERVER_SURFACE_FRICTION_FIELD_H
#define FLATLAND_SERVER_SURFACE_FRICTION_FIELD_H

#include <Box2D/Box2D.h>
#include <flatland_server/yaml_reader.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/filesystem.hpp>
#include <string>
#include <vector>

namespace flatland_server {

/**
 * @brief Smoothly-sampled per-region surface friction multiplier field.
 *
 * Models wet patches, spills, and ramps as a raster of friction *multipliers*
 * over the world plane. A grayscale image is mapped linearly so that white
 * (255) is the dry multiplier @p mu_max and black (0) is the wet multiplier
 * @p mu_min; the grid is sampled with bilinear interpolation so that the factor
 * varies continuously (C0) as a body moves across a region boundary. This
 * smoothness is deliberate: feeding step-change friction into the contact /
 * traction solver produces force discontinuities and chatter, which the
 * interpolation avoids.
 *
 * The returned value is a unitless multiplier (clamped to a small positive
 * floor) intended to scale a configured friction coefficient. An empty /
 * default-constructed field, or any query outside the raster's extent, returns
 * 1.0 (no effect), so worlds without a `surface_friction` block behave exactly
 * as before.
 *
 * The sampling math depends only on std and Box2D's b2Vec2, so the field is
 * directly unit-testable from an in-memory grid without a running world or
 * OpenCV. Image loading lives in the FromConfig factory.
 */
class SurfaceFrictionField {
 public:
  /// Lower bound the returned multiplier is clamped to, so a region can never
  /// drive traction to exactly zero (degenerate, frictionless contact).
  static constexpr double kDefaultMinFactor = 0.05;

  /**
   * @brief Construct an empty (disabled) field that returns 1.0 everywhere.
   */
  SurfaceFrictionField() = default;

  /**
   * @brief Construct from an in-memory raster of friction multipliers.
   * @param[in] factors Row-major grid of size width*height; each cell is a
   *            friction multiplier already mapped into the desired range.
   * @param[in] width Number of columns (must be >= 1)
   * @param[in] height Number of rows (must be >= 1)
   * @param[in] resolution Metres per cell (must be > 0)
   * @param[in] origin_x World x of the lower-left corner of the grid [m]
   * @param[in] origin_y World y of the lower-left corner of the grid [m]
   * @param[in] min_factor Lower clamp applied to every sampled value
   */
  SurfaceFrictionField(const std::vector<double>& factors, int width,
                       int height, double resolution, double origin_x,
                       double origin_y, double min_factor = kDefaultMinFactor);

  /**
   * @brief Whether the field carries any region data. A disabled field always
   *        returns 1.0.
   *
   * Becomes true once a runtime spill overlay is activated, even if the static
   * raster is empty, so a clean world that injects a spill mid-run starts
   * altering traction. With no static raster and no spills it stays false and
   * GetFrictionFactor returns 1.0 everywhere (clean-run invariant).
   */
  bool Enabled() const { return enabled_ || !spills_.empty(); }

  /**
   * @brief Sample the friction multiplier at a world point.
   * @param[in] world_point Query point in world coordinates [m]
   * @return Continuous, bounded friction multiplier; 1.0 if the field is
   *         disabled or the point lies outside the raster extent.
   */
  double GetFrictionFactor(const b2Vec2& world_point) const;

  /**
   * @brief Add or update a circular low-friction spill overlay, keyed by id.
   *
   * A runtime overlay that the static raster knows nothing about: it models a
   * spill that appears mid-run. Inside a flat core the returned multiplier is
   * @p mu; from the core out to @p radius it feathers (C0) back to 1.0 so the
   * traction solver never sees a step change at the edge (mirroring the
   * raster's bilinear smoothing). Calling again with the same @p id replaces
   * the region, so the FaultInjector can ramp @p mu with severity each step.
   *
   * @param[in] id Stable region id (e.g. the fault id); reused id updates it
   * @param[in] center Spill centre in world coordinates [m]
   * @param[in] radius Spill outer radius [m] (> 0); no effect beyond it
   * @param[in] mu Slipperiest multiplier at the core, clamped to >= min_factor
   */
  void AddCircularRegion(const std::string& id, const b2Vec2& center,
                         double radius, double mu);

  /**
   * @brief Remove a circular spill overlay previously added with @p id. A
   *        no-op if the id is absent. Once the last spill is removed and there
   *        is no static raster, the field reports Enabled() == false again.
   */
  void RemoveCircularRegion(const std::string& id);

  /**
   * @brief Build a field from the optional world-YAML `surface_friction` block.
   *
   * Accepts (all parsed through YamlReader, throwing YAMLException on bad
   * config):
   *   map:        grayscale image path (relative to @p world_dir or absolute)
   *   resolution: metres per pixel (> 0)
   *   origin:     [x, y] world coordinate of the image lower-left corner
   *   mu_min:     multiplier for fully-wet (black) pixels   (default 0.3)
   *   mu_max:     multiplier for fully-dry (white) pixels   (default 1.0)
   *   min_factor: lower clamp on the sampled value          (default 0.05)
   *
   * A null reader (no block in the world) yields a disabled field.
   *
   * @param[in] reader YamlReader for the `surface_friction` subnode
   * @param[in] world_dir Directory of the world YAML, used to resolve `map`
   * @return A populated or disabled SurfaceFrictionField
   */
  static SurfaceFrictionField FromConfig(
      YamlReader& reader, const boost::filesystem::path& world_dir);

  /**
   * @brief Build a MarkerArray visualizing the low-traction regions.
   *
   * Emits a single CUBE_LIST marker whose points are the centres of the cells
   * whose friction multiplier is below the field maximum (the nominal/dry
   * baseline), each colour-mapped from green (full grip) to red (most
   * slippery). A leading DELETEALL marker clears any previously published
   * regions. A disabled field yields an empty (DELETEALL-only) array, so the
   * overlay simply disappears for worlds without a surface_friction block.
   *
   * Cells are emitted at a stride that bounds the point count to @p max_points;
   * if the field is larger than that the stride is logged so the downsampling
   * is never silent. This is a one-shot, latched-publish helper — it does no
   * physics work and is never called from the step loop.
   *
   * @param[in] frame_id tf frame the markers are stamped in (e.g. "map")
   * @param[in] ns Marker namespace
   * @param[in] max_points Upper bound on emitted cell points (downsampled
   * above)
   * @return MarkerArray ready to publish
   */
  visualization_msgs::MarkerArray ToMarkerArray(
      const std::string& frame_id, const std::string& ns = "friction_regions",
      size_t max_points = 200000) const;

 private:
  /// A runtime circular low-friction spill overlay (analytic, not rastered).
  struct CircularRegion {
    std::string id;  ///< stable key for update-in-place / removal
    double cx = 0.0;     ///< centre world x [m]
    double cy = 0.0;     ///< centre world y [m]
    double radius = 0.0;  ///< outer radius [m]; no effect beyond it
    double mu = 1.0;      ///< slipperiest multiplier at the core
  };

  /**
   * @brief Sample the analytic spill overlays at a point, taking the minimum
   *        multiplier over all active spills (1.0 if none affect the point).
   */
  double SampleSpills(const b2Vec2& world_point) const;

  bool enabled_ = false;      ///< false -> static raster returns 1.0
  std::vector<CircularRegion> spills_;  ///< active runtime spill overlays
  std::vector<double> grid_;  ///< row-major width_*height_ friction multipliers
  int width_ = 0;             ///< grid columns
  int height_ = 0;            ///< grid rows
  double resolution_ = 1.0;   ///< metres per cell
  double origin_x_ = 0.0;     ///< world x of grid lower-left corner [m]
  double origin_y_ = 0.0;     ///< world y of grid lower-left corner [m]
  double min_factor_ = kDefaultMinFactor;  ///< lower clamp on sampled value

  /**
   * @brief Read a grid cell, clamping the column/row to the valid range so the
   *        edge value extends outward (used by the bilinear interpolation).
   */
  double CellClamped(int col, int row) const;
};

}  // namespace flatland_server

#endif  // FLATLAND_SERVER_SURFACE_FRICTION_FIELD_H
