/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	flatland_scenario_runner.cpp
 * @brief	Headless observer that scores an algorithm-under-test run
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

// A pure ROS observer node. It does NOT run the simulation; scenario_run.launch
// starts flatland_server (which steps physics + publishes /clock, /tf,
// collisions) and the algorithm-under-test alongside it. This node watches the
// sealed out-of-band TRUE pose (tf global_frame->robot_base_frame, the same
// transform model_tf_publisher emits) plus the in-band estimate + collisions,
// feeds a pure ScenarioScorer, and writes scenario_result.json on the terminal
// condition (goal reached or sim-time timeout). A wall-clock watchdog scores a
// hung/crashed run as max stress + invalid so the outer loop never hangs.

#include <flatland_msgs/Collisions.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include "flatland_server/scenario_scorer.h"

namespace {

double g_est_x = 0.0;
double g_est_y = 0.0;
bool g_have_est = false;

void EstPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  g_est_x = msg->pose.pose.position.x;
  g_est_y = msg->pose.pose.position.y;
  g_have_est = true;
}

std::string JsonEscape(const std::string &s) {
  std::string out;
  for (char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          // Other control characters -> \u00XX so output stays valid JSON.
          char buf[7];
          snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
        break;
    }
  }
  return out;
}

void WriteResult(const std::string &path,
                 const flatland_server::ScenarioMetrics &m, double composite,
                 const flatland_server::ScoreWeights &w, int seed,
                 const std::string &genome_hash, const std::string &terminal) {
  std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    ROS_ERROR_NAMED("ScenarioRunner", "Cannot open result file %s",
                    path.c_str());
    return;
  }
  out << "{\n";
  out << "  \"seed\": " << seed << ",\n";
  out << "  \"genome_hash\": \"" << JsonEscape(genome_hash) << "\",\n";
  out << "  \"composite_score\": " << composite << ",\n";
  out << "  \"terminal\": \"" << JsonEscape(terminal) << "\",\n";
  out << "  \"invalid\": " << (m.invalid ? "true" : "false") << ",\n";
  out << "  \"metrics\": {\n";
  out << "    \"mean_localization_error_m\": " << m.mean_localization_error_m
      << ",\n";
  out << "    \"max_localization_error_m\": " << m.max_localization_error_m
      << ",\n";
  out << "    \"collision_count\": " << m.collision_count << ",\n";
  out << "    \"goal_reached\": " << (m.goal_reached ? "true" : "false")
      << ",\n";
  out << "    \"goal_distance_m\": " << m.goal_distance_m << ",\n";
  out << "    \"mean_tracking_error_m\": " << m.mean_tracking_error_m << ",\n";
  out << "    \"sim_time_s\": " << m.sim_time_s << ",\n";
  out << "    \"time_to_goal_s\": " << m.time_to_goal_s << "\n";
  out << "  },\n";
  out << "  \"weights\": {\n";
  out << "    \"localization_error\": " << w.localization_error << ",\n";
  out << "    \"collisions\": " << w.collisions << ",\n";
  out << "    \"goal_failure\": " << w.goal_failure << ",\n";
  out << "    \"tracking_error\": " << w.tracking_error << "\n";
  out << "  }\n";
  out << "}\n";
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "flatland_scenario_runner");
  ros::NodeHandle pnh("~");

  // --- parameters ---
  std::string result_path, genome_hash, global_frame, base_frame;
  std::string est_topic, collisions_topic;
  pnh.param<std::string>("result_path", result_path,
                         "/tmp/scenario_result.json");
  pnh.param<std::string>("genome_hash", genome_hash, "");
  pnh.param<std::string>("global_frame", global_frame, "map");
  pnh.param<std::string>("robot_base_frame", base_frame, "robot_base");
  pnh.param<std::string>("est_pose_topic", est_topic, "robot/amcl_pose");
  pnh.param<std::string>("collisions_topic", collisions_topic,
                         "robot/collisions");

  int seed = -1;
  pnh.param<int>("seed", seed, -1);

  double timeout_s, goal_radius, wall_timeout_s, rate_hz;
  double goal_x, goal_y, start_x, start_y;
  pnh.param<double>("scenario_duration", timeout_s, 60.0);
  pnh.param<double>("goal_radius", goal_radius, 0.5);
  pnh.param<double>("goal_x", goal_x, 0.0);
  pnh.param<double>("goal_y", goal_y, 0.0);
  pnh.param<double>("start_x", start_x, 0.0);
  pnh.param<double>("start_y", start_y, 0.0);
  pnh.param<double>("wall_timeout", wall_timeout_s, 300.0);
  pnh.param<double>("sample_rate", rate_hz, 50.0);

  flatland_server::ScoreWeights w;
  pnh.param<double>("w_localization", w.localization_error, 1.0);
  pnh.param<double>("w_collisions", w.collisions, 1.0);
  pnh.param<double>("w_goal", w.goal_failure, 1.0);
  pnh.param<double>("w_tracking", w.tracking_error, 1.0);

  flatland_server::ScenarioScorer scorer(w, goal_radius, timeout_s);
  scorer.SetPath(start_x, start_y, goal_x, goal_y);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::NodeHandle nh;
  ros::Subscriber est_sub = nh.subscribe(est_topic, 10, EstPoseCb);
  ros::Subscriber col_sub = nh.subscribe<flatland_msgs::Collisions>(
      collisions_topic, 50,
      [&scorer](const flatland_msgs::Collisions::ConstPtr &msg) {
        scorer.RecordCollisions(static_cast<int>(msg->collisions.size()));
      });

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_INFO_NAMED("ScenarioRunner",
                 "Observing: tf %s->%s, est=%s, collisions=%s; timeout=%.1fs, "
                 "goal=(%.2f,%.2f) r=%.2f -> %s",
                 global_frame.c_str(), base_frame.c_str(), est_topic.c_str(),
                 collisions_topic.c_str(), timeout_s, goal_x, goal_y,
                 goal_radius, result_path.c_str());

  // Wait for sim time to start advancing (use_sim_time + /clock).
  ros::WallTime wall_start = ros::WallTime::now();
  while (ros::ok() && ros::Time::now().toSec() <= 0.0) {
    if ((ros::WallTime::now() - wall_start).toSec() > wall_timeout_s) {
      ROS_ERROR_NAMED("ScenarioRunner", "sim time never started; invalid");
      scorer.MarkInvalid();
      auto m = scorer.Finalize(0.0);
      WriteResult(result_path, m, scorer.CompositeScore(m), w, seed,
                  genome_hash, "no_clock");
      ros::shutdown();
      return 0;
    }
    ros::WallDuration(0.05).sleep();
  }

  double sim_start = ros::Time::now().toSec();
  std::string terminal = "timeout";
  ros::Rate rate(rate_hz);

  while (ros::ok()) {
    double now = ros::Time::now().toSec();
    double elapsed = now - sim_start;

    // True pose from the sealed out-of-band transform.
    geometry_msgs::TransformStamped tf;
    bool have_true = false;
    double true_x = 0.0, true_y = 0.0;
    try {
      tf = tf_buffer.lookupTransform(global_frame, base_frame, ros::Time(0));
      true_x = tf.transform.translation.x;
      true_y = tf.transform.translation.y;
      have_true = true;
    } catch (const tf2::TransformException &) {
      // not available yet; keep waiting
    }

    if (have_true) {
      scorer.RecordTruePose(true_x, true_y, elapsed);
      if (g_have_est) {
        scorer.RecordLocalization(true_x, true_y, g_est_x, g_est_y);
      }
      if (std::hypot(goal_x - true_x, goal_y - true_y) <= goal_radius) {
        terminal = "goal_reached";
        break;
      }
    }

    if (elapsed >= timeout_s) {
      terminal = "timeout";
      break;
    }
    if ((ros::WallTime::now() - wall_start).toSec() > wall_timeout_s) {
      ROS_WARN_NAMED("ScenarioRunner", "wall watchdog tripped; invalid");
      scorer.MarkInvalid();
      terminal = "wall_timeout";
      break;
    }
    rate.sleep();
  }

  double elapsed = ros::Time::now().toSec() - sim_start;
  flatland_server::ScenarioMetrics m = scorer.Finalize(elapsed);
  double composite = scorer.CompositeScore(m);
  WriteResult(result_path, m, composite, w, seed, genome_hash, terminal);

  ROS_INFO_NAMED("ScenarioRunner",
                 "Scenario done (%s): composite=%.4f loc=%.3fm coll=%d "
                 "goal_reached=%d track=%.3fm -> %s",
                 terminal.c_str(), composite, m.mean_localization_error_m,
                 m.collision_count, m.goal_reached, m.mean_tracking_error_m,
                 result_path.c_str());

  spinner.stop();
  ros::shutdown();
  return 0;
}
