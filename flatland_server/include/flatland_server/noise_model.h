/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	noise_model.h
 * @brief	Calibrated, context-conditioned per-channel sensor/actuator
 * noise model
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

#ifndef FLATLAND_SERVER_NOISE_MODEL_H
#define FLATLAND_SERVER_NOISE_MODEL_H

#include <flatland_server/noise_context.h>

#include <map>
#include <memory>
#include <random>
#include <string>

namespace flatland_server {

/**
 * @brief Resolved per-channel noise parameters for one context.
 */
struct NoiseParams {
  double mean = 0.0;  ///< additive bias for the channel
  double std = 0.0;   ///< standard deviation (>= 0) for the channel
};

/**
 * @brief Calibrated, context-conditioned noise model (schema_version 1).
 *
 * Implements the `parametric_linear` representation from
 * docs/noise_model_format.md: each channel's std and mean are an affine
 * function of (speed, sensor_age, darkness = 1 - lighting) plus a per-surface
 * offset. Evaluation is closed-form, allocation-free, and deterministic, so it
 * is safe in the physics step loop and stays seed-reproducible through the
 * caller's RngManager-derived engine.
 *
 * A model built via Legacy() (or for a channel absent from a loaded file) is a
 * pass-through that yields NoiseParams{0, fallback_std}, letting plugins keep
 * their exact pre-existing constant-std behaviour. IsLegacy() lets a plugin
 * skip the model entirely and preserve byte-for-byte draw order when no
 * calibrated file is configured.
 */
class NoiseModel {
 public:
  /// Schema version this loader understands.
  static constexpr int kSchemaVersion = 1;

  /**
   * @brief Build the built-in legacy_gaussian (pass-through) model.
   */
  static std::shared_ptr<NoiseModel> Legacy();

  /**
   * @brief Load and validate a calibrated model from a JSON file.
   * @param[in] path Filesystem path to the model JSON.
   * @return A populated NoiseModel.
   * @throws YAMLException on a missing/unparsable file, wrong schema_version,
   *         or unsupported model_type.
   */
  static std::shared_ptr<NoiseModel> FromFile(const std::string &path);

  /**
   * @brief Convenience loader for plugins: empty path -> Legacy(); otherwise
   *        FromFile(path), degrading to Legacy() with a ROS_WARN if the file is
   *        missing/invalid. Never throws, so a bad config falls back to the
   *        legacy constant-std behaviour rather than aborting world load.
   * @param[in] path Model file path (may be empty).
   * @param[in] component_key Component id used only for the warning message.
   */
  static std::shared_ptr<NoiseModel> LoadOrLegacy(
      const std::string &path, const std::string &component_key);

  /**
   * @brief True for the built-in pass-through model (no calibrated channels).
   */
  bool IsLegacy() const { return legacy_; }

  /**
   * @brief Whether the model carries an entry for a channel.
   */
  bool HasChannel(const std::string &channel) const {
    return channels_.find(channel) != channels_.end();
  }

  /**
   * @brief Resolve the noise parameters for a channel in a context.
   * @param[in] channel Channel name (see docs/noise_model_format.md).
   * @param[in] ctx The per-step context.
   * @param[in] fallback_std Std to use when the channel is absent (legacy std).
   * @return Resolved {mean, std}; std is clamped to >= 0.
   */
  NoiseParams GetParams(const std::string &channel, const NoiseContext &ctx,
                        double fallback_std = 0.0) const;

  /**
   * @brief Draw one context-conditioned noise sample for a channel.
   * @param[in] channel Channel name.
   * @param[in] ctx The per-step context.
   * @param[in,out] rng The caller's seeded engine (draw order preserved).
   * @param[in] fallback_std Std when the channel is absent (legacy std).
   * @return mean + std * N(0,1) when std > 0, drawing one value from `rng`;
   *         exactly `mean` (no draw) when std <= 0.
   */
  double Sample(const std::string &channel, const NoiseContext &ctx,
                std::default_random_engine &rng,
                double fallback_std = 0.0) const;

 private:
  NoiseModel() = default;

  /// Affine context model for a single channel.
  struct ChannelModel {
    double base_std = 0.0;
    double std_speed = 0.0;
    double std_age = 0.0;
    double std_darkness = 0.0;
    std::map<int, double> surface_std_offset;

    double base_mean = 0.0;
    double mean_speed = 0.0;
    double mean_age = 0.0;
    double mean_darkness = 0.0;
    std::map<int, double> surface_mean_offset;
  };

  bool legacy_ = true;
  std::map<std::string, ChannelModel> channels_;
};

}  // namespace flatland_server

#endif  // FLATLAND_SERVER_NOISE_MODEL_H
