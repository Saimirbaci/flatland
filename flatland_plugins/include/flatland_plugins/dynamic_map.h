/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	dynamic_map.h
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

#ifndef FLATLAND_PLUGINS_DYNAMIC_MAP_H
#define FLATLAND_PLUGINS_DYNAMIC_MAP_H

#include <flatland_server/timekeeper.h>
#include <flatland_server/world_plugin.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace flatland_server {
class Layer;
class YamlReader;
}  // namespace flatland_server

namespace flatland_plugins {

/**
 * @brief World plugin that mutates a named static layer's collision geometry
 * mid-episode on a scripted timeline (a shelf moves, a corridor closes).
 *
 * Mirrors the FaultInjector contract: it parses an ordered `events:` list keyed
 * to a `target_layer`, evaluates a sim-time trigger each step, and on each
 * event's onset edge read-modify-writes the layer's in-memory occupancy bitmap
 * (OpenCV rectangle ops in world-metric coordinates), then atomically rebuilds
 * the layer's Box2D chain-loop fixtures + occupancy grid and re-publishes the
 * latched overlay. All edits happen in BeforePhysicsStep (between steps), never
 * inside a contact callback, and only on the fire edge -- never per step -- so
 * there is zero per-step cost once all events have fired.
 *
 * The applied timeline is recorded out-of-band in a sealed JSON manifest (the
 * run seed plus every op with its sim onset time and world/pixel region) so
 * mapping/localization eval knows exactly how the map evolved.
 */
class DynamicMap : public flatland_server::WorldPlugin {
 public:
  /// What an event does to the occupancy bitmap.
  enum class Op {
    kFill,       ///< mark a rectangular region occupied (obstacle appears)
    kClear,      ///< mark a rectangular region free (corridor opens)
    kTranslate,  ///< clear a source rect and fill a same-size rect at dest
  };

  /// Per-event parsed config plus runtime fire state.
  struct Event {
    std::string id;       ///< unique event id (for the manifest/logs)
    double time_s = 0.0;  ///< sim-time onset threshold (elapsed seconds)
    Op op = Op::kFill;    ///< edit operation
    std::string op_str;   ///< raw op string (for the manifest)

    // Region in world coordinates [m]: lower-left x, lower-left y, width,
    // height. For kTranslate this is the SOURCE rectangle.
    double rx = 0.0;
    double ry = 0.0;
    double rw = 0.0;
    double rh = 0.0;

    // Destination lower-left [m] for kTranslate (width/height reused from the
    // source rect).
    double dx = 0.0;
    double dy = 0.0;

    bool applied = false;      ///< fire-edge mutation already applied (once)
    double applied_at = -1.0;  ///< sim-sec the event actually fired
  };

  void OnInitialize(const YAML::Node &config) override;
  void BeforePhysicsStep(
      const flatland_server::Timekeeper &timekeeper) override;

 private:
  /**
   * @brief Apply one event's edit to @p bitmap (read-modify-write, in place).
   */
  void ApplyEvent(const Event &event, cv::Mat &bitmap) const;

  /**
   * @brief Convert a world-metric rectangle to an integer pixel rect on the
   * target layer's bitmap, clamped to the image bounds. Returns false (empty
   * rect) when the region is fully outside the bitmap.
   *
   * Uses the exact transform Layer::LoadFromBitmap / BuildOccupancyGrid assume:
   * col = (wx - origin.x) / res, row = rows - (wy - origin.y) / res.
   */
  bool WorldRectToPixels(double wx, double wy, double ww, double wh,
                         cv::Rect &out) const;

  /**
   * @brief (Re)write the sealed JSON manifest of the scripted edit timeline.
   */
  void WriteManifest() const;

  std::string target_layer_;                 ///< name of the layer to mutate
  flatland_server::Layer *layer_ = nullptr;  ///< resolved target layer
  std::vector<Event> events_;  ///< scripted edits, in declaration order
  std::string manifest_path_;  ///< sealed sidecar path (empty = disabled)
  std::string seed_key_;       ///< provenance label for the manifest
  uint32_t seed_ = 0;          ///< run seed recorded for reproducibility

  bool started_ = false;  ///< first step seen
  ros::Time start_time_;  ///< sim time captured at first step
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_DYNAMIC_MAP_H
