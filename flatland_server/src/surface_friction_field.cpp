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

SurfaceFrictionField::SurfaceFrictionField(const std::vector<double>& factors,
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
    throw Exception("SurfaceFrictionField: grid dimensions must be >= 1 (got " +
                    std::to_string(width_) + "x" + std::to_string(height_) +
                    ")");
  }
  if (resolution_ <= 0.0) {
    throw Exception("SurfaceFrictionField: resolution must be > 0 (got " +
                    std::to_string(resolution_) + ")");
  }
  if (grid_.size() != static_cast<size_t>(width_) * height_) {
    throw Exception("SurfaceFrictionField: grid size " +
                    std::to_string(grid_.size()) +
                    " does not match width*height " +
                    std::to_string(static_cast<size_t>(width_) * height_));
  }
}

double SurfaceFrictionField::CellClamped(int col, int row) const {
  col = std::max(0, std::min(width_ - 1, col));
  row = std::max(0, std::min(height_ - 1, row));
  return grid_[static_cast<size_t>(row) * width_ + col];
}

double SurfaceFrictionField::SampleSpills(const b2Vec2& world_point) const {
  // No spill affects this point -> ambient multiplier.
  double factor = 1.0;
  for (const auto& s : spills_) {
    if (s.radius <= 0.0) {
      continue;
    }
    const double dx = world_point.x - s.cx;
    const double dy = world_point.y - s.cy;
    const double d = std::sqrt(dx * dx + dy * dy);
    if (d >= s.radius) {
      continue;  // outside the spill: no effect
    }
    // Flat low-friction core out to half the radius, then a C0 linear feather
    // back to 1.0 at the edge, so a body entering the spill loses traction
    // smoothly rather than through a force step (mirrors the raster's bilinear
    // smoothing and avoids contact-solver chatter).
    const double core = 0.5 * s.radius;
    double region;
    if (d <= core) {
      region = s.mu;
    } else {
      const double t = (d - core) / (s.radius - core);  // 0 at core, 1 at edge
      region = s.mu + t * (1.0 - s.mu);
    }
    factor = std::min(factor, region);
  }
  return factor;
}

double SurfaceFrictionField::GetFrictionFactor(
    const b2Vec2& world_point) const {
  // Start from the static raster sample (1.0 if there is no raster), then take
  // the minimum against any active runtime spill overlay. A clean field with no
  // raster and no spills returns exactly 1.0 (clean-run invariant).
  double value = 1.0;

  if (enabled_) {
    const double x = world_point.x;
    const double y = world_point.y;

    // Points outside the raster extent are ambient: no wet/dry region there.
    const double max_x = origin_x_ + width_ * resolution_;
    const double max_y = origin_y_ + height_ * resolution_;
    if (x >= origin_x_ && x <= max_x && y >= origin_y_ && y <= max_y) {
      // Continuous pixel-centre coordinates. Columns increase with +x. Rows are
      // stored top-down (row 0 = max y) to match the occupancy-map convention
      // in layer.cpp, so +y maps to decreasing row.
      const double cx = (x - origin_x_) / resolution_ - 0.5;
      const double ry = (height_ - 0.5) - (y - origin_y_) / resolution_;

      const int c0 = static_cast<int>(std::floor(cx));
      const int r0 = static_cast<int>(std::floor(ry));
      const double fx = cx - c0;
      const double fy = ry - r0;

      // Bilinear blend of the four surrounding cells (edge-clamped). This makes
      // the sampled multiplier C0-continuous across cell and region boundaries,
      // so the traction solver never sees a friction step change.
      const double v00 = CellClamped(c0, r0);
      const double v10 = CellClamped(c0 + 1, r0);
      const double v01 = CellClamped(c0, r0 + 1);
      const double v11 = CellClamped(c0 + 1, r0 + 1);

      const double top = v00 + fx * (v10 - v00);
      const double bottom = v01 + fx * (v11 - v01);
      value = top + fy * (bottom - top);
    }
  }

  if (!spills_.empty()) {
    value = std::min(value, SampleSpills(world_point));
  }

  return std::max(min_factor_, value);
}

void SurfaceFrictionField::AddCircularRegion(const std::string& id,
                                             const b2Vec2& center,
                                             double radius, double mu) {
  CircularRegion region;
  region.id = id;
  region.cx = center.x;
  region.cy = center.y;
  region.radius = radius;
  region.mu = std::max(min_factor_, mu);

  for (auto& s : spills_) {
    if (s.id == id) {
      s = region;  // update in place so severity can ramp the multiplier
      return;
    }
  }
  spills_.push_back(region);
}

void SurfaceFrictionField::RemoveCircularRegion(const std::string& id) {
  spills_.erase(std::remove_if(spills_.begin(), spills_.end(),
                               [&](const CircularRegion& s) {
                                 return s.id == id;
                               }),
                spills_.end());
}

SurfaceFrictionField SurfaceFrictionField::FromConfig(
    YamlReader& reader, const boost::filesystem::path& world_dir) {
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
  const double min_factor = reader.Get<double>("min_factor", kDefaultMinFactor);

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

  cv::Mat image = cv::imread(image_path.string(), FLATLAND_FRICTION_GREYSCALE);
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

visualization_msgs::MarkerArray SurfaceFrictionField::ToMarkerArray(
    const std::string& frame_id, const std::string& ns,
    size_t max_points) const {
  visualization_msgs::MarkerArray array;

  // Always lead with a DELETEALL so re-publishing replaces the previous set and
  // a disabled field clears the overlay entirely.
  visualization_msgs::Marker clear;
  clear.header.frame_id = frame_id;
  clear.ns = ns;
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(clear);

  if (!enabled_ || grid_.empty()) {
    return array;
  }

  // The nominal (driest) multiplier present is the baseline; cells at that
  // value are ordinary floor and not worth drawing. Anything below it is a
  // wet/spill region we want to highlight.
  const double max_factor = *std::max_element(grid_.begin(), grid_.end());
  const double span = std::max(1e-9, max_factor - min_factor_);

  // Choose a cell stride so the emitted point count stays under max_points.
  const size_t total_cells = grid_.size();
  size_t stride = 1;
  if (max_points > 0 && total_cells > max_points) {
    // ceil(sqrt(total/max)) keeps the 2D thinning roughly uniform in x and y.
    const double ratio = static_cast<double>(total_cells) / max_points;
    stride = static_cast<size_t>(std::ceil(std::sqrt(ratio)));
    ROS_WARN_NAMED("SurfaceFrictionField",
                   "friction region overlay downsampled by stride %zu "
                   "(%zu cells > %zu point budget)",
                   stride, total_cells, max_points);
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = resolution_ * stride;
  marker.scale.y = resolution_ * stride;
  marker.scale.z = 0.02;

  for (int row = 0; row < height_; row += static_cast<int>(stride)) {
    for (int col = 0; col < width_; col += static_cast<int>(stride)) {
      const double factor = grid_[static_cast<size_t>(row) * width_ + col];
      // Skip nominal/dry cells; only render the slippery patches.
      if (factor >= max_factor - 1e-6) {
        continue;
      }

      geometry_msgs::Point p;
      p.x = origin_x_ + (col + 0.5) * resolution_;
      // Rows are stored top-down (row 0 = max y), matching GetFrictionFactor.
      p.y = origin_y_ + (height_ - row - 0.5) * resolution_;
      p.z = 0.0;
      marker.points.push_back(p);

      // green (grip) -> red (slip) as the multiplier drops toward min_factor_.
      const double t =
          std::max(0.0, std::min(1.0, (factor - min_factor_) / span));
      std_msgs::ColorRGBA color;
      color.r = static_cast<float>(1.0 - t);
      color.g = static_cast<float>(t);
      color.b = 0.0f;
      color.a = 0.5f;
      marker.colors.push_back(color);
    }
  }

  array.markers.push_back(marker);
  return array;
}

}  // namespace flatland_server
