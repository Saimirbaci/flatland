/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	random.h
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

#ifndef FLATLAND_SERVER_RANDOM_H
#define FLATLAND_SERVER_RANDOM_H

#include <cstdint>
#include <random>
#include <string>

namespace flatland_server {

/**
 * @brief Process-wide seeded RNG authority.
 *
 * A single global seed makes every stochastic source in the simulator
 * reproducible. Rather than hand a shared engine to each consumer (which would
 * couple their draw counts and make determinism sensitive to plugin add/remove
 * order), callers ask for a private engine derived from a stable string key
 * (e.g. "model_name/plugin_name"). Identical (seed, key) pairs always yield the
 * same sequence; distinct keys yield independent sequences.
 *
 * Mirrors the DebugVisualization singleton pattern so no plugin signatures
 * change. Seed() is called once at world startup before any plugin initializes.
 */
class RngManager {
 public:
  /**
   * @brief Return the process-wide singleton.
   */
  static RngManager &Get();

  /**
   * @brief Set the global seed. Call once at world startup, before plugins init.
   * @param[in] global_seed The authoritative run seed.
   */
  void Seed(uint32_t global_seed);

  /**
   * @brief Whether Seed() has been called.
   */
  bool IsSeeded() const { return seeded_; }

  /**
   * @brief The effective global seed (0 if never seeded).
   */
  uint32_t GetSeed() const { return global_seed_; }

  /**
   * @brief Derive a deterministic 32-bit seed from the global seed and a key.
   * @param[in] key Stable identifier for the consumer (e.g. model/plugin name).
   * @return A reproducible seed value, identical across runs for (seed, key).
   */
  uint32_t DeriveSeed(const std::string &key) const;

  /**
   * @brief Derive a private, deterministically-seeded engine from a key.
   * @param[in] key Stable identifier for the consumer (e.g. model/plugin name).
   * @return A std::default_random_engine seeded via DeriveSeed(key).
   */
  std::default_random_engine DeriveEngine(const std::string &key) const;

 private:
  RngManager() : global_seed_(0), seeded_(false) {}

  uint32_t global_seed_;  ///< The authoritative run seed
  bool seeded_;           ///< True once Seed() has been called
};

}  // namespace flatland_server

#endif  // FLATLAND_SERVER_RANDOM_H
