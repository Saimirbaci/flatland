/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	scenario_scorer.h
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

#ifndef FLATLAND_SERVER_SCENARIO_SCORER_H
#define FLATLAND_SERVER_SCENARIO_SCORER_H

#include <string>

namespace flatland_server {

/**
 * @brief Relative weights of the stress-score components. A higher composite
 * score means the algorithm-under-test performed WORSE (more stress), so the
 * adversarial proposer maximizes it.
 */
struct ScoreWeights {
  double localization_error = 1.0;  ///< est-vs-true pose error
  double collisions = 1.0;          ///< contact count
  double goal_failure = 1.0;        ///< failed/incomplete navigation
  double tracking_error = 1.0;      ///< deviation from the straight path
};

/**
 * @brief Component metrics measured over one scenario run.
 */
struct ScenarioMetrics {
  double mean_localization_error_m = 0.0;
  double max_localization_error_m = 0.0;
  int collision_count = 0;
  bool goal_reached = false;
  double goal_distance_m = 0.0;       ///< final true-pose distance to goal
  double mean_tracking_error_m = 0.0;
  double sim_time_s = 0.0;
  double time_to_goal_s = -1.0;       ///< sim-sec goal first reached, else -1
  bool invalid = false;              ///< run crashed/produced no usable data
};

/**
 * @brief Pure (no ROS, no Box2D) accumulator + score math for one scenario.
 *
 * The runner node feeds it samples from live topics + the sealed out-of-band
 * true pose; this class only does arithmetic, so it is unit-testable against
 * fixtures. Component normalizers are static and saturating so a single extreme
 * sample cannot dominate the composite score.
 */
class ScenarioScorer {
 public:
  /**
   * @param[in] weights   component weights (need not sum to 1).
   * @param[in] goal_radius_m  true-pose distance at/below which the goal counts
   *   as reached.
   * @param[in] timeout_s  scenario sim-time budget (for context/metadata).
   */
  ScenarioScorer(const ScoreWeights &weights, double goal_radius_m,
                 double timeout_s);

  /// Set the straight-line reference path (start->goal) for tracking error.
  void SetPath(double start_x, double start_y, double goal_x, double goal_y);

  /// Record one localization sample: true vs estimated 2D position.
  void RecordLocalization(double true_x, double true_y, double est_x,
                          double est_y);

  /// Add @p n contacts observed this step (n >= 0).
  void RecordCollisions(int n);

  /// Update goal-reach + tracking error from the latest TRUE pose at @p t.
  void RecordTruePose(double x, double y, double sim_time_s);

  /// Mark the run as crashed/unusable (scored as max stress).
  void MarkInvalid() { invalid_ = true; }

  /// Compute the accumulated metrics, stamping the final sim time.
  ScenarioMetrics Finalize(double sim_time_s) const;

  /// Weighted, saturating composite stress score in [0, 1] (higher = worse).
  double CompositeScore(const ScenarioMetrics &m) const;

  // --- pure, static component normalizers (each maps to ~[0,1]) ---
  static double NormLocalization(double mean_err_m);
  static double NormCollisions(int count);
  static double NormGoalFailure(bool reached, double goal_distance_m,
                                double start_goal_dist_m);
  static double NormTracking(double mean_track_m);

  /// Perpendicular distance from (px,py) to the segment (ax,ay)->(bx,by).
  static double PointSegmentDistance(double px, double py, double ax, double ay,
                                     double bx, double by);

  const ScoreWeights &weights() const { return weights_; }
  double start_goal_dist() const { return start_goal_dist_; }

 private:
  ScoreWeights weights_;
  double goal_radius_m_;
  double timeout_s_;

  // localization accumulators
  double loc_err_sum_ = 0.0;
  double loc_err_max_ = 0.0;
  long loc_samples_ = 0;

  int collisions_ = 0;

  // goal / tracking
  bool have_path_ = false;
  double sx_ = 0.0, sy_ = 0.0, gx_ = 0.0, gy_ = 0.0;
  double start_goal_dist_ = 0.0;
  bool goal_reached_ = false;
  double time_to_goal_ = -1.0;
  double last_goal_dist_ = -1.0;
  double track_err_sum_ = 0.0;
  long track_samples_ = 0;

  bool invalid_ = false;
};

}  // namespace flatland_server

#endif  // FLATLAND_SERVER_SCENARIO_SCORER_H
