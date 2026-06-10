/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	scenario_scorer_test.cpp
 * @brief	Unit tests for the pure ScenarioScorer stress-score math
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

#include <gtest/gtest.h>

#include "flatland_server/scenario_scorer.h"

using flatland_server::ScenarioMetrics;
using flatland_server::ScenarioScorer;
using flatland_server::ScoreWeights;

// --- normalizers are saturating and monotonic --------------------------------
TEST(ScenarioScorer, NormalizersMonotoneAndBounded) {
  EXPECT_DOUBLE_EQ(ScenarioScorer::NormLocalization(0.0), 0.0);
  EXPECT_GT(ScenarioScorer::NormLocalization(1.0),
            ScenarioScorer::NormLocalization(0.2));
  EXPECT_LT(ScenarioScorer::NormLocalization(100.0), 1.0);

  EXPECT_DOUBLE_EQ(ScenarioScorer::NormCollisions(0), 0.0);
  EXPECT_GT(ScenarioScorer::NormCollisions(10),
            ScenarioScorer::NormCollisions(1));
  EXPECT_LT(ScenarioScorer::NormCollisions(1000), 1.0);

  // Goal failure: reached -> 0; unfinished -> fraction of distance left.
  EXPECT_DOUBLE_EQ(ScenarioScorer::NormGoalFailure(true, 5.0, 10.0), 0.0);
  EXPECT_DOUBLE_EQ(ScenarioScorer::NormGoalFailure(false, 5.0, 10.0), 0.5);
  EXPECT_DOUBLE_EQ(ScenarioScorer::NormGoalFailure(false, 20.0, 10.0), 1.0);
  EXPECT_DOUBLE_EQ(ScenarioScorer::NormGoalFailure(false, 1.0, 0.0), 1.0);
}

// --- point-to-segment distance ----------------------------------------------
TEST(ScenarioScorer, PointSegmentDistance) {
  // Perpendicular from (0,2) to the x-axis segment (0,0)->(10,0) == 2.
  EXPECT_NEAR(ScenarioScorer::PointSegmentDistance(0, 2, 0, 0, 10, 0), 2.0,
              1e-9);
  // Beyond the segment end clamps to the endpoint distance.
  EXPECT_NEAR(ScenarioScorer::PointSegmentDistance(13, 0, 0, 0, 10, 0), 3.0,
              1e-9);
  // Degenerate segment == point distance.
  EXPECT_NEAR(ScenarioScorer::PointSegmentDistance(3, 4, 0, 0, 0, 0), 5.0,
              1e-9);
}

// --- localization accumulation ----------------------------------------------
TEST(ScenarioScorer, LocalizationMeanAndMax) {
  ScoreWeights w;
  ScenarioScorer s(w, 0.5, 60.0);
  s.RecordLocalization(0, 0, 0, 0);  // err 0
  s.RecordLocalization(0, 0, 3, 4);  // err 5
  s.RecordLocalization(0, 0, 1, 0);  // err 1
  ScenarioMetrics m = s.Finalize(10.0);
  EXPECT_NEAR(m.mean_localization_error_m, (0.0 + 5.0 + 1.0) / 3.0, 1e-9);
  EXPECT_NEAR(m.max_localization_error_m, 5.0, 1e-9);
  EXPECT_EQ(m.sim_time_s, 10.0);
}

// --- goal reach + tracking ---------------------------------------------------
TEST(ScenarioScorer, GoalReachAndTracking) {
  ScoreWeights w;
  ScenarioScorer s(w, 0.5, 60.0);
  s.SetPath(0, 0, 10, 0);
  EXPECT_NEAR(s.start_goal_dist(), 10.0, 1e-9);

  s.RecordTruePose(2, 1, 1.0);   // 1 m off the path, far from goal
  s.RecordTruePose(9.9, 0.1, 5.0);  // within 0.5 m of goal -> reached at t=5
  ScenarioMetrics m = s.Finalize(6.0);
  EXPECT_TRUE(m.goal_reached);
  EXPECT_NEAR(m.time_to_goal_s, 5.0, 1e-9);
  EXPECT_GT(m.mean_tracking_error_m, 0.0);
}

// --- collisions + composite is weighted mean of components -------------------
TEST(ScenarioScorer, CompositeWeightedMean) {
  ScoreWeights w;  // all 1.0
  ScenarioScorer s(w, 0.5, 60.0);
  s.SetPath(0, 0, 10, 0);
  s.RecordCollisions(3);
  s.RecordLocalization(0, 0, 0.5, 0);  // 0.5 m
  s.RecordTruePose(0, 0, 1.0);         // far from goal, never reached

  ScenarioMetrics m = s.Finalize(60.0);
  EXPECT_EQ(m.collision_count, 3);
  EXPECT_FALSE(m.goal_reached);

  double expected =
      (ScenarioScorer::NormLocalization(m.mean_localization_error_m) +
       ScenarioScorer::NormCollisions(m.collision_count) +
       ScenarioScorer::NormGoalFailure(false, m.goal_distance_m,
                                       s.start_goal_dist()) +
       ScenarioScorer::NormTracking(m.mean_tracking_error_m)) /
      4.0;
  EXPECT_NEAR(s.CompositeScore(m), expected, 1e-9);
  EXPECT_GE(s.CompositeScore(m), 0.0);
  EXPECT_LE(s.CompositeScore(m), 1.0);
}

// --- invalid run scores as maximal stress -----------------------------------
TEST(ScenarioScorer, InvalidRunIsMaxStress) {
  ScoreWeights w;
  ScenarioScorer s(w, 0.5, 60.0);
  s.MarkInvalid();
  ScenarioMetrics m = s.Finalize(1.0);
  EXPECT_TRUE(m.invalid);
  EXPECT_DOUBLE_EQ(s.CompositeScore(m), 1.0);
}

// --- determinism: identical inputs -> identical score ------------------------
TEST(ScenarioScorer, Deterministic) {
  ScoreWeights w;
  auto build = []() {
    ScenarioScorer s(ScoreWeights{}, 0.5, 60.0);
    s.SetPath(0, 0, 5, 5);
    s.RecordCollisions(2);
    s.RecordLocalization(0, 0, 0.3, 0.4);
    s.RecordTruePose(1, 1, 2.0);
    ScenarioMetrics m = s.Finalize(30.0);
    return s.CompositeScore(m);
  };
  EXPECT_DOUBLE_EQ(build(), build());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
