/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	scenario_scorer.cpp
 * @brief	Pure stress-score accumulator for the AI scenario generator
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

#include "flatland_server/scenario_scorer.h"

#include <algorithm>
#include <cmath>

namespace flatland_server {

ScenarioScorer::ScenarioScorer(const ScoreWeights &weights,
                               double goal_radius_m, double timeout_s)
    : weights_(weights),
      goal_radius_m_(goal_radius_m),
      timeout_s_(timeout_s) {}

void ScenarioScorer::SetPath(double start_x, double start_y, double goal_x,
                             double goal_y) {
  sx_ = start_x;
  sy_ = start_y;
  gx_ = goal_x;
  gy_ = goal_y;
  start_goal_dist_ = std::hypot(gx_ - sx_, gy_ - sy_);
  have_path_ = true;
}

void ScenarioScorer::RecordLocalization(double true_x, double true_y,
                                        double est_x, double est_y) {
  double err = std::hypot(true_x - est_x, true_y - est_y);
  loc_err_sum_ += err;
  loc_err_max_ = std::max(loc_err_max_, err);
  loc_samples_++;
}

void ScenarioScorer::RecordCollisions(int n) {
  if (n > 0) collisions_ += n;
}

void ScenarioScorer::RecordTruePose(double x, double y, double sim_time_s) {
  if (!have_path_) return;

  double dist_to_goal = std::hypot(gx_ - x, gy_ - y);
  last_goal_dist_ = dist_to_goal;
  if (!goal_reached_ && dist_to_goal <= goal_radius_m_) {
    goal_reached_ = true;
    time_to_goal_ = sim_time_s;
  }

  // Path-tracking error: perpendicular distance to the start->goal segment.
  track_err_sum_ += PointSegmentDistance(x, y, sx_, sy_, gx_, gy_);
  track_samples_++;
}

ScenarioMetrics ScenarioScorer::Finalize(double sim_time_s) const {
  ScenarioMetrics m;
  m.sim_time_s = sim_time_s;
  m.invalid = invalid_;
  m.mean_localization_error_m =
      loc_samples_ > 0 ? loc_err_sum_ / static_cast<double>(loc_samples_) : 0.0;
  m.max_localization_error_m = loc_err_max_;
  m.collision_count = collisions_;
  m.goal_reached = goal_reached_;
  m.time_to_goal_s = time_to_goal_;
  m.goal_distance_m =
      last_goal_dist_ >= 0.0 ? last_goal_dist_ : start_goal_dist_;
  m.mean_tracking_error_m =
      track_samples_ > 0 ? track_err_sum_ / static_cast<double>(track_samples_)
                         : 0.0;
  return m;
}

double ScenarioScorer::CompositeScore(const ScenarioMetrics &m) const {
  if (m.invalid) return 1.0;  // a broken run is maximally "stressful" by fiat

  double w_sum = weights_.localization_error + weights_.collisions +
                 weights_.goal_failure + weights_.tracking_error;
  if (w_sum <= 0.0) return 0.0;

  double s = 0.0;
  s += weights_.localization_error *
       NormLocalization(m.mean_localization_error_m);
  s += weights_.collisions * NormCollisions(m.collision_count);
  s += weights_.goal_failure *
       NormGoalFailure(m.goal_reached, m.goal_distance_m, start_goal_dist_);
  s += weights_.tracking_error * NormTracking(m.mean_tracking_error_m);
  return s / w_sum;
}

// --- normalizers -----------------------------------------------------------
// Each maps a raw metric to ~[0,1] with a soft saturation so no single sample
// dominates. Scales are deliberately conservative ground-robot values.

double ScenarioScorer::NormLocalization(double mean_err_m) {
  if (mean_err_m <= 0.0) return 0.0;
  return 1.0 - std::exp(-mean_err_m / 0.5);  // ~0.86 at 1 m mean error
}

double ScenarioScorer::NormCollisions(int count) {
  if (count <= 0) return 0.0;
  return 1.0 - std::exp(-static_cast<double>(count) / 3.0);  // ~0.63 at 3
}

double ScenarioScorer::NormGoalFailure(bool reached, double goal_distance_m,
                                       double start_goal_dist_m) {
  if (reached) return 0.0;
  if (start_goal_dist_m <= 1e-6) return 1.0;
  // Fraction of the journey left unfinished, clamped to [0,1].
  double frac = goal_distance_m / start_goal_dist_m;
  return std::max(0.0, std::min(1.0, frac));
}

double ScenarioScorer::NormTracking(double mean_track_m) {
  if (mean_track_m <= 0.0) return 0.0;
  return 1.0 - std::exp(-mean_track_m / 0.5);
}

double ScenarioScorer::PointSegmentDistance(double px, double py, double ax,
                                            double ay, double bx, double by) {
  double dx = bx - ax;
  double dy = by - ay;
  double len2 = dx * dx + dy * dy;
  if (len2 <= 1e-12) {
    return std::hypot(px - ax, py - ay);  // degenerate segment == point
  }
  double t = ((px - ax) * dx + (py - ay) * dy) / len2;
  t = std::max(0.0, std::min(1.0, t));
  double cx = ax + t * dx;
  double cy = ay + t * dy;
  return std::hypot(px - cx, py - cy);
}

}  // namespace flatland_server
