/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   surface_friction_field.cpp
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

#include <flatland_server/exceptions.h>
#include <flatland_server/surface_friction_field.h>
#include <flatland_server/types.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#if CV_MAJOR_VERSION < 3
#define FLATLAND_FRICTION_GREYSCALE CV_LOAD_IMAGE_GRAYSCALE
#else
#include <opencv2/imgcodecs.hpp>
#define FLATLAND_FRICTION_GREYSCALE cv::ImreadModes::IMREAD_GRAYSCALE
#endif

namespace flatland_server {

constexpr double SurfaceFrictionField::kDefaultMinFactor;

SurfaceFrictionField::SurfaceFrictionField(const std::vector<double> &factors,
                                           int width, int height,
                                           double resolution, double origin_x,
                                           double origin_y, double min_factor)
    : enabled_(true),
      grid_(factors),
      width_(width),
      height_(height),
      resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      min_factor_(min_factor) {
  if (width_ < 1 || height_ < 1) {
    throw Exception(
        "SurfaceFrictionField: grid dimensions must be >= 1 (got " +
        std::to_string(width_) + "x" + std::to_string(height_) + ")");
  }
  if (resolution_ <= 0.0) {
    throw Exception(
        "SurfaceFrictionField: resolution must be > 0 (got " +
        std::to_string(resolution_) + ")");
  }
  if (grid_.size() != static_cast<size_t>(width_) * height_) {
    throw Exception(
        "SurfaceFrictionField: grid size " + std::to_string(grid_.size()) +
        " does not match width*height " +
        std::to_string(static_cast<size_t>(width_) * height_));
  }
}

double SurfaceFrictionField::CellClamped(int col, int row) const {
  col = std::max(0, std::min(width_ - 1, col));
  row = std::max(0, std::min(height_ - 1, row));
  return grid_[static_cast<size_t>(row) * width_ + col];
}

double SurfaceFrictionField::GetFrictionFactor(
    const b2Vec2 &world_point) const {
  // A disabled field (no surface_friction block) never alters traction.
  if (!enabled_) {
    return 1.0;
  }

  const double x = world_point.x;
  const double y = world_point.y;

  // Points outside the raster extent are ambient: no wet/dry region there.
  const double max_x = origin_x_ + width_ * resolution_;
  const double max_y = origin_y_ + height_ * resolution_;
  if (x < origin_x_ || x > max_x || y < origin_y_ || y > max_y) {
    return 1.0;
  }

  // Continuous pixel-centre coordinates. Columns increase with +x. Rows are
  // stored top-down (row 0 = max y) to match the occupancy-map convention in
  // layer.cpp, so +y maps to decreasing row.
  const double cx = (x - origin_x_) / resolution_ - 0.5;
  const double ry = (height_ - 0.5) - (y - origin_y_) / resolution_;

  const int c0 = static_cast<int>(std::floor(cx));
  const int r0 = static_cast<int>(std::floor(ry));
  const double fx = cx - c0;
  const double fy = ry - r0;

  // Bilinear blend of the four surrounding cells (edge-clamped). This makes the
  // sampled multiplier C0-continuous across cell and region boundaries, so the
  // traction solver never sees a friction step change.
  const double v00 = CellClamped(c0, r0);
  const double v10 = CellClamped(c0 + 1, r0);
  const double v01 = CellClamped(c0, r0 + 1);
  const double v11 = CellClamped(c0 + 1, r0 + 1);

  const double top = v00 + fx * (v10 - v00);
  const double bottom = v01 + fx * (v11 - v01);
  const double value = top + fy * (bottom - top);

  return std::max(min_factor_, value);
}

SurfaceFrictionField SurfaceFrictionField::FromConfig(
    YamlReader &reader, const boost::filesystem::path &world_dir) {
  // No surface_friction block -> disabled field (factor 1.0 everywhere).
  if (reader.IsNodeNull()) {
    return SurfaceFrictionField();
  }

  const double resolution = reader.Get<double>("resolution");
  if (resolution <= 0.0) {
    throw YAMLException("surface_friction \"resolution\" must be > 0 (got " +
                        std::to_string(resolution) + ")");
  }
  Vec2 origin = reader.GetVec2("origin", Vec2(0, 0));
  const double mu_min = reader.Get<double>("mu_min", 0.3);
  const double mu_max = reader.Get<double>("mu_max", 1.0);
  const double min_factor =
      reader.Get<double>("min_factor", kDefaultMinFactor);

  boost::filesystem::path image_path(reader.Get<std::string>("map"));
  reader.EnsureAccessedAllKeys();

  if (image_path.string().empty()) {
    throw YAMLException("surface_friction requires a \"map\" image path");
  }
  if (image_path.string().front() != '/') {
    image_path = world_dir / image_path;
  }

  ROS_INFO_NAMED("SurfaceFrictionField",
                 "Loading surface friction map from path=\"%s\" "
                 "(mu_min=%.3f mu_max=%.3f res=%.4f)",
                 image_path.string().c_str(), mu_min, mu_max, resolution);

  cv::Mat image =
      cv::imread(image_path.string(), FLATLAND_FRICTION_GREYSCALE);
  if (image.empty()) {
    throw YAMLException("Failed to load surface_friction map " +
                        Q(image_path.string()));
  }

  const int width = image.cols;
  const int height = image.rows;

  // Map pixel intensity [0,255] linearly into [mu_min, mu_max]: white (dry) ->
  // mu_max, black (wet/spill) -> mu_min. The grid stores the resulting
  // multipliers directly so sampling stays pure arithmetic.
  std::vector<double> factors;
  factors.reserve(static_cast<size_t>(width) * height);
  for (int r = 0; r < height; r++) {
    for (int c = 0; c < width; c++) {
      const double intensity = image.at<uint8_t>(r, c) / 255.0;
      factors.push_back(mu_min + intensity * (mu_max - mu_min));
    }
  }

  return SurfaceFrictionField(factors, width, height, resolution, origin.x,
                              origin.y, min_factor);
}

}  // namespace flatland_server
