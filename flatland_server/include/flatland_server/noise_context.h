/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	noise_context.h
 * @brief	Per-step sensing context (surface/speed/lighting/age) for noise
 * models
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

#ifndef FLATLAND_SERVER_NOISE_CONTEXT_H
#define FLATLAND_SERVER_NOISE_CONTEXT_H

#include <Box2D/Box2D.h>

#include <functional>

namespace flatland_server {

/**
 * @brief Cheap per-step sensing context that conditions the noise model.
 *
 * Every field is a scalar the engine can produce without allocating in the hot
 * path. See docs/noise_model_format.md for the contract with the Python
 * calibrator. The model derives a "darkness" feature as (1 - lighting).
 */
struct NoiseContext {
  int surface_id = 0;       ///< discrete surface bucket (0 = nominal/dry)
  double speed = 0.0;       ///< body linear speed [m/s], >= 0
  double lighting = 1.0;    ///< ambient lighting scalar, 0 (dark) .. 1 (bright)
  double sensor_age = 0.0;  ///< sensor age [hours], >= 0 (per-plugin config)
};

/**
 * @brief Process-wide provider of the world-level pieces of a NoiseContext.
 *
 * Mirrors the RngManager / FaultInjectionRegistry singleton pattern so plugin
 * signatures do not change. The World is the single writer: at load it sets the
 * ambient lighting scalar and a surface sampler (a closure over the world's
 * SurfaceFrictionField). Sensor/drive plugins are read-only consumers that call
 * Build(body, age) once per step. With nothing configured the provider returns
 * a default context (surface 0, lighting 1.0), so legacy worlds are unaffected.
 *
 * Thread-safety: the world writes once at load (single-threaded); plugins read
 * on the main thread before dispatching any workers. No locking is needed and
 * Build() is const / non-allocating.
 */
class NoiseContextProvider {
 public:
  /// Friction factor at/above which the surface counts as nominal/dry.
  static constexpr double kSurfaceDryThreshold = 0.85;
  /// Friction factor below which the surface counts as the slipperiest bucket.
  static constexpr double kSurfaceWetThreshold = 0.55;

  /**
   * @brief Return the process-wide singleton.
   */
  static NoiseContextProvider &Get();

  /**
   * @brief Restore defaults (lighting 1.0, no surface sampler). Call between
   *        tests / on world teardown.
   */
  void Reset();

  /**
   * @brief Set the ambient lighting scalar, clamped to [0, 1].
   */
  void SetLighting(double lighting);

  /**
   * @brief The ambient lighting scalar (default 1.0).
   */
  double Lighting() const { return lighting_; }

  /**
   * @brief Install a surface friction sampler: world (x, y) -> friction factor
   *        (1.0 = full grip). The World passes a closure over its
   *        SurfaceFrictionField. A null/empty sampler means every point is
   *        nominal (surface_id 0).
   */
  void SetSurfaceSampler(std::function<double(double, double)> sampler);

  /**
   * @brief Bucket the surface under a world point into a surface_id.
   * @return 0 (dry) when no sampler is installed or factor is high; 1/2 for
   *         progressively slipperier surfaces.
   */
  int SurfaceIdAt(double world_x, double world_y) const;

  /**
   * @brief Build a NoiseContext for a body: linear speed + surface from the
   *        body position, plus the world lighting and the supplied sensor age.
   * @param[in] body Non-owning Box2D body the sensor/drive is attached to.
   * @param[in] sensor_age_hours Per-plugin configured sensor age in hours.
   */
  NoiseContext Build(const b2Body *body, double sensor_age_hours) const;

 private:
  NoiseContextProvider() = default;

  double lighting_ = 1.0;  ///< ambient lighting scalar in [0,1]
  std::function<double(double, double)>
      surface_sampler_;  ///< world (x,y) -> friction factor; empty -> dry
};

}  // namespace flatland_server

#endif  // FLATLAND_SERVER_NOISE_CONTEXT_H
