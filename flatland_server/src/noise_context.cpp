/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name	noise_context.cpp
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

#include "flatland_server/noise_context.h"

#include <algorithm>
#include <utility>

namespace flatland_server {

NoiseContextProvider &NoiseContextProvider::Get() {
  static NoiseContextProvider instance;
  return instance;
}

void NoiseContextProvider::Reset() {
  lighting_ = 1.0;
  surface_sampler_ = nullptr;
}

void NoiseContextProvider::SetLighting(double lighting) {
  lighting_ = std::max(0.0, std::min(1.0, lighting));
}

void NoiseContextProvider::SetSurfaceSampler(
    std::function<double(double, double)> sampler) {
  surface_sampler_ = std::move(sampler);
}

int NoiseContextProvider::SurfaceIdAt(double world_x, double world_y) const {
  if (!surface_sampler_) {
    return 0;
  }
  const double factor = surface_sampler_(world_x, world_y);
  if (factor >= kSurfaceDryThreshold) {
    return 0;
  }
  if (factor >= kSurfaceWetThreshold) {
    return 1;
  }
  return 2;
}

NoiseContext NoiseContextProvider::Build(const b2Body *body,
                                         double sensor_age_hours) const {
  NoiseContext ctx;
  ctx.lighting = lighting_;
  ctx.sensor_age = std::max(0.0, sensor_age_hours);

  if (body != nullptr) {
    const b2Vec2 v = body->GetLinearVelocity();
    ctx.speed = v.Length();
    const b2Vec2 p = body->GetPosition();
    ctx.surface_id = SurfaceIdAt(p.x, p.y);
  }
  return ctx;
}

}  // namespace flatland_server
