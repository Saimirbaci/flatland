/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	random.cpp
 * @brief	Central seeded RNG authority for deterministic/reproducible runs
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

#include "flatland_server/random.h"
#include <ros/ros.h>

namespace flatland_server {

namespace {
// FNV-1a hash of the key bytes. Used instead of std::hash because std::hash is
// not specified to be stable across compilers/runs, which would defeat
// cross-build reproducibility.
uint32_t Fnv1aHash(const std::string &key) {
  uint32_t hash = 2166136261u;  // FNV offset basis
  for (char c : key) {
    hash ^= static_cast<uint32_t>(static_cast<unsigned char>(c));
    hash *= 16777619u;  // FNV prime
  }
  return hash;
}
}  // namespace

RngManager &RngManager::Get() {
  static RngManager instance;
  return instance;
}

void RngManager::Seed(uint32_t global_seed) {
  global_seed_ = global_seed;
  seeded_ = true;
}

uint32_t RngManager::DeriveSeed(const std::string &key) const {
  if (!seeded_) {
    ROS_WARN_NAMED("Rng",
                   "DeriveSeed(\"%s\") called before RngManager::Seed(); "
                   "using global seed 0",
                   key.c_str());
  }

  // Mix the global seed with a stable hash of the key through a seed_seq so
  // that every (seed, key) pair maps to a well-distributed, reproducible value.
  std::seed_seq seq{global_seed_, Fnv1aHash(key)};
  uint32_t derived = 0;
  seq.generate(&derived, &derived + 1);
  return derived;
}

std::default_random_engine RngManager::DeriveEngine(
    const std::string &key) const {
  return std::default_random_engine(DeriveSeed(key));
}

}  // namespace flatland_server
