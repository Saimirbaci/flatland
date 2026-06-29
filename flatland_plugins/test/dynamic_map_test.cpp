/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   dynamic_map_test.cpp
 * @brief  rostest: the DynamicMap world plugin mutates a static layer's Box2D
 *         collision geometry mid-episode on a scripted timeline (a corridor
 *         closes, a shelf moves), the change is observable via ray casts
 * against the rebuilt static body, the timeline is deterministic across runs,
 *         and the applied ops are sealed to an out-of-band JSON manifest.
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

#include <Box2D/Box2D.h>
#include <flatland_server/layer.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <string>

namespace fs = boost::filesystem;
using namespace flatland_server;

// Records whether a ray cast against the world hit any fixture.
class RayHitCallback : public b2RayCastCallback {
 public:
  bool hit = false;
  float32 ReportFixture(b2Fixture* /*fixture*/, const b2Vec2& /*point*/,
                        const b2Vec2& /*normal*/,
                        float32 /*fraction*/) override {
    hit = true;
    return 0.0f;  // terminate: we only need to know whether anything was hit
  }
};

class DynamicMapTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  World* w = nullptr;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }
  void TearDown() override {
    if (w != nullptr) delete w;
    w = nullptr;
  }

  World* LoadWorld() {
    fs::path world_yaml =
        this_file_dir / fs::path("dynamic_map_tests") / fs::path("world.yaml");
    w = World::MakeWorld(world_yaml.string());
    return w;
  }

  void Step(Timekeeper& tk, unsigned int n) {
    for (unsigned int i = 0; i < n; i++) {
      w->Update(tk);
      ros::spinOnce();
    }
  }

  // True if a ray from p1 to p2 crosses any collision fixture in the world.
  bool RayHits(double x1, double y1, double x2, double y2) const {
    RayHitCallback cb;
    w->physics_world_->RayCast(&cb, b2Vec2(x1, y1), b2Vec2(x2, y2));
    return cb.hit;
  }

  // Number of fixtures currently on the named layer's static body.
  int LayerFixtureCount(const std::string& layer_name) const {
    Layer* layer = w->GetLayer(layer_name);
    if (layer == nullptr || layer->GetBody() == nullptr) return -1;
    int count = 0;
    for (b2Fixture* f = layer->GetBody()->physics_body_->GetFixtureList();
         f != nullptr; f = f->GetNext()) {
      count++;
    }
    return count;
  }

  std::string ReadFile(const std::string& path) const {
    std::ifstream in(path.c_str());
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
  }
};

// The target layer resolves and starts with exactly the one shelf obstacle.
TEST_F(DynamicMapTest, initial_geometry_has_only_the_shelf) {
  ASSERT_NE(nullptr, LoadWorld());
  Layer* layer = w->GetLayer("static");
  ASSERT_NE(nullptr, layer) << "target layer not resolvable by name";
  EXPECT_EQ(1, LayerFixtureCount("static"))
      << "expected exactly one initial obstacle (the shelf)";

  // The shelf at world [1,2]x[1,2] is hit; the empty corridor and far side are
  // clear before any event fires.
  EXPECT_TRUE(RayHits(0.5, 1.5, 3.0, 1.5)) << "shelf missing at start";
  EXPECT_FALSE(RayHits(2.0, 5.0, 6.0, 5.0)) << "corridor blocked at start";
  EXPECT_FALSE(RayHits(6.0, 7.5, 9.0, 7.5)) << "far side occupied at start";
}

// "A corridor closes": at t=1 s a wall fills x[4,5], so a previously-open ray
// across the middle of the map now hits.
TEST_F(DynamicMapTest, corridor_closes_at_its_scheduled_time) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ASSERT_NE(nullptr, LoadWorld());

  Step(tk, 50);  // ~0.49 s, before the t=1 onset
  EXPECT_FALSE(RayHits(2.0, 5.0, 6.0, 5.0))
      << "corridor closed before its scheduled time";

  Step(tk, 110);  // now ~1.59 s, past the t=1 onset (still before t=2)
  EXPECT_TRUE(RayHits(2.0, 5.0, 6.0, 5.0))
      << "corridor did not close after its scheduled time";
  EXPECT_EQ(2, LayerFixtureCount("static"))
      << "expected shelf + new wall after the corridor closes";
  // The shelf has not moved yet (t=2 not reached).
  EXPECT_TRUE(RayHits(0.5, 1.5, 3.0, 1.5)) << "shelf moved too early";
}

// "A shelf moves": at t=2 s the corner shelf is translated to the far side, so
// the source ray clears and a far-side ray begins to hit.
TEST_F(DynamicMapTest, shelf_moves_at_its_scheduled_time) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ASSERT_NE(nullptr, LoadWorld());

  Step(tk, 160);  // ~1.59 s: corridor closed, shelf not yet moved
  ASSERT_TRUE(RayHits(0.5, 1.5, 3.0, 1.5)) << "shelf gone before its move";
  ASSERT_FALSE(RayHits(6.0, 7.5, 9.0, 7.5)) << "shelf at destination too early";

  Step(tk, 100);  // ~2.59 s: past the t=2 onset
  EXPECT_FALSE(RayHits(0.5, 1.5, 3.0, 1.5))
      << "shelf still present at its old location after the move";
  EXPECT_TRUE(RayHits(6.0, 7.5, 9.0, 7.5))
      << "shelf did not appear at its new location";
  // Geometry count is unchanged by a translate: shelf + wall.
  EXPECT_EQ(2, LayerFixtureCount("static"));
}

// The scripted timeline is sealed out-of-band to a JSON manifest with the
// resolved onset times for both ops.
TEST_F(DynamicMapTest, manifest_records_both_ops_out_of_band) {
  Timekeeper tk;
  tk.SetMaxStepSize(0.01);
  ASSERT_NE(nullptr, LoadWorld());
  Step(tk, 260);  // past both onsets

  const std::string manifest =
      ReadFile("/tmp/flatland_dynamic_map_ground_truth.json");
  ASSERT_FALSE(manifest.empty()) << "manifest file was not written";
  EXPECT_NE(std::string::npos, manifest.find("close_corridor"));
  EXPECT_NE(std::string::npos, manifest.find("move_shelf"));
  EXPECT_NE(std::string::npos, manifest.find("\"target_layer\": \"static\""));
  // Both onsets must be resolved (no longer the unfired -1 sentinel).
  EXPECT_EQ(std::string::npos, manifest.find("\"applied_at_s\": -1"))
      << "an event onset was never resolved in the manifest";
}

// Same seed, same scripted world => identical geometry and timeline.
TEST_F(DynamicMapTest, edits_are_deterministic_across_runs) {
  Timekeeper tk1;
  tk1.SetMaxStepSize(0.01);
  ASSERT_NE(nullptr, LoadWorld());
  Step(tk1, 260);
  const int count_run1 = LayerFixtureCount("static");
  const bool source_run1 = RayHits(0.5, 1.5, 3.0, 1.5);
  const bool dest_run1 = RayHits(6.0, 7.5, 9.0, 7.5);
  const bool corridor_run1 = RayHits(2.0, 5.0, 6.0, 5.0);

  delete w;
  w = nullptr;

  Timekeeper tk2;
  tk2.SetMaxStepSize(0.01);
  ASSERT_NE(nullptr, LoadWorld());
  Step(tk2, 260);
  EXPECT_EQ(count_run1, LayerFixtureCount("static"));
  EXPECT_EQ(source_run1, RayHits(0.5, 1.5, 3.0, 1.5));
  EXPECT_EQ(dest_run1, RayHits(6.0, 7.5, 9.0, 7.5));
  EXPECT_EQ(corridor_run1, RayHits(2.0, 5.0, 6.0, 5.0));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_map_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
