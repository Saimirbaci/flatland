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
  SurfaceFrictionField(const std::vector<double> &factors, int width,
                       int height, double resolution, double origin_x,
                       double origin_y,
                       double min_factor = kDefaultMinFactor);

  /**
   * @brief Whether the field carries any region data. A disabled field always
   *        returns 1.0.
   */
  bool Enabled() const { return enabled_; }

  /**
   * @brief Sample the friction multiplier at a world point.
   * @param[in] world_point Query point in world coordinates [m]
   * @return Continuous, bounded friction multiplier; 1.0 if the field is
   *         disabled or the point lies outside the raster extent.
   */
  double GetFrictionFactor(const b2Vec2 &world_point) const;

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
      YamlReader &reader, const boost::filesystem::path &world_dir);

 private:
  bool enabled_ = false;     ///< false -> GetFrictionFactor always returns 1.0
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
