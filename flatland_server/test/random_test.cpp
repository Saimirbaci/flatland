/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	random_test.cpp
 * @brief	Unit tests for the seeded RNG authority (RngManager)
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

#include <flatland_server/random.h>
#include <gtest/gtest.h>
#include <cstdint>
#include <random>
#include <vector>

using flatland_server::RngManager;

// Same global seed + same key must reproduce the same engine sequence.
TEST(RngManagerTest, same_seed_same_key_is_reproducible) {
  RngManager::Get().Seed(12345);
  std::default_random_engine e1 = RngManager::Get().DeriveEngine("modelA/laser");
  RngManager::Get().Seed(12345);
  std::default_random_engine e2 = RngManager::Get().DeriveEngine("modelA/laser");
  for (int i = 0; i < 100; i++) {
    EXPECT_EQ(e1(), e2());
  }
}

// Distinct keys under the same seed yield independent streams.
TEST(RngManagerTest, different_key_differs) {
  RngManager::Get().Seed(12345);
  std::default_random_engine a = RngManager::Get().DeriveEngine("modelA/laser");
  std::default_random_engine b = RngManager::Get().DeriveEngine("modelB/laser");
  bool differ = false;
  for (int i = 0; i < 100; i++) {
    if (a() != b()) {
      differ = true;
      break;
    }
  }
  EXPECT_TRUE(differ);
}

// A different seed under the same key yields a different stream.
TEST(RngManagerTest, different_seed_differs) {
  RngManager::Get().Seed(1);
  std::default_random_engine a = RngManager::Get().DeriveEngine("k");
  RngManager::Get().Seed(2);
  std::default_random_engine b = RngManager::Get().DeriveEngine("k");
  bool differ = false;
  for (int i = 0; i < 100; i++) {
    if (a() != b()) {
      differ = true;
      break;
    }
  }
  EXPECT_TRUE(differ);
}

// DeriveSeed is stable for a key and distinct across keys; accessors report
// the active seed.
TEST(RngManagerTest, derive_seed_stable_and_distinct) {
  RngManager::Get().Seed(777);
  uint32_t s1 = RngManager::Get().DeriveSeed("foo");
  uint32_t s1b = RngManager::Get().DeriveSeed("foo");
  uint32_t s2 = RngManager::Get().DeriveSeed("bar");
  EXPECT_EQ(s1, s1b);
  EXPECT_NE(s1, s2);
  EXPECT_TRUE(RngManager::Get().IsSeeded());
  EXPECT_EQ(RngManager::Get().GetSeed(), 777u);
}

// Mirrors how plugins draw gaussian noise: a normal_distribution fed by a
// derived engine must reproduce byte-for-byte for the same seed.
TEST(RngManagerTest, reproduces_normal_distribution_sequence) {
  RngManager::Get().Seed(42);
  std::default_random_engine rng1 = RngManager::Get().DeriveEngine("robot/imu");
  std::normal_distribution<double> n1(0.0, 1.0);
  std::vector<double> seq1;
  for (int i = 0; i < 50; i++) {
    seq1.push_back(n1(rng1));
  }

  RngManager::Get().Seed(42);
  std::default_random_engine rng2 = RngManager::Get().DeriveEngine("robot/imu");
  std::normal_distribution<double> n2(0.0, 1.0);
  for (int i = 0; i < 50; i++) {
    EXPECT_DOUBLE_EQ(seq1[i], n2(rng2));
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
