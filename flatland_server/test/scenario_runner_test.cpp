/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	scenario_runner_test.cpp
 * @brief	Rostest: flatland_scenario_runner emits a sealed result file
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
#include <ros/ros.h>

#include <fstream>
#include <sstream>
#include <string>

// Wait (wall time) for the runner to write its sealed result file, then assert
// it is well formed: not invalid, carries the genome hash + seed echoed back,
// and a goal-failure terminal (the static robot never reaches the far goal).
TEST(ScenarioRunner, EmitsWellFormedResult) {
  ros::NodeHandle pnh("~");
  std::string result_path;
  ASSERT_TRUE(pnh.getParam("result_path", result_path));

  std::string contents;
  ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(90.0);
  while (ros::WallTime::now() < deadline) {
    std::ifstream f(result_path.c_str());
    if (f.good()) {
      std::stringstream ss;
      ss << f.rdbuf();
      contents = ss.str();
      if (contents.find("composite_score") != std::string::npos &&
          contents.find('}') != std::string::npos) {
        break;
      }
    }
    ros::WallDuration(0.5).sleep();
  }

  ASSERT_FALSE(contents.empty()) << "runner never wrote " << result_path;
  EXPECT_NE(contents.find("\"composite_score\""), std::string::npos);
  EXPECT_NE(contents.find("\"genome_hash\": \"testhash\""), std::string::npos);
  EXPECT_NE(contents.find("\"invalid\": false"), std::string::npos)
      << "sim time started, so the run must be valid; got:\n"
      << contents;
  EXPECT_NE(contents.find("\"goal_reached\": false"), std::string::npos)
      << "static robot must not reach the far goal; got:\n"
      << contents;
  EXPECT_NE(contents.find("\"metrics\""), std::string::npos);
  EXPECT_NE(contents.find("\"weights\""), std::string::npos);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "scenario_runner_test");
  return RUN_ALL_TESTS();
}
