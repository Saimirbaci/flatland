# Context-Conditioned Noise Model Format

This document is the **contract** between the C++ consumer
(`flatland_server/noise_model.{h,cpp}`) and the Python producer
(`noise_cal/`). Both sides must agree on the schema version and field names
below. Treat it like `scenario_gen/param_space.json`: versioned, and any change
forces a coordinated edit on both sides.

## 1. Motivation

Flatland sensors/drives historically add **static zero-mean Gaussian** noise
with a single configured `std` (e.g. `noise_std_dev`, `odom_pose_noise`). Real
hardware noise is not static: it grows with vehicle **speed**, depends on the
**surface** under the robot, degrades in poor **lighting** (vision/range), and
worsens as a sensor **ages**. This format describes a small, calibrated model
that maps a cheap per-step *context* to per-channel noise parameters, so the
baseline noise is realistic rather than arbitrary. The existing
fault-injection framework then layers faults *on top* of this realistic
baseline.

The model is intentionally **dependency-free and deterministic**: a parametric
(piecewise-linear) form that a C++11 build can evaluate in the step loop with no
ML runtime, and that stays seed-reproducible through the existing `RngManager`.

## 2. NoiseContext

The engine produces a `NoiseContext` cheaply each step
(`flatland_server/noise_context.h`). Fields:

| Field        | Type     | Units / range        | Source |
|--------------|----------|----------------------|--------|
| `surface_id` | int      | discrete bucket ≥ 0  | Surface under the body, bucketed from the world's `SurfaceFrictionField` factor (0 = nominal/dry, higher = more slippery/rough). |
| `speed`      | double   | m/s, ≥ 0             | Body linear speed from Box2D (`b2Body::GetLinearVelocity().Length()`). |
| `lighting`   | double   | 0 (dark) .. 1 (bright) | World-level scalar; default 1.0, optionally set from the world YAML `noise_context: {lighting: x}`. |
| `sensor_age` | double   | hours, ≥ 0           | Per-plugin config (`sensor_age_hours`), constant per run. |

The model's *darkness* feature is derived as `1 - lighting`, so a brighter world
(higher `lighting`) reduces light-driven noise.

Surface bucketing (`NoiseContextProvider::SurfaceIdAt`): the world friction
multiplier `f` (1.0 = full grip) maps to `surface_id`:
`f >= 0.85 -> 0`, `0.55 <= f < 0.85 -> 1`, `f < 0.55 -> 2`. Worlds without a
`surface_friction` block always report `surface_id = 0`.

## 3. Channels

A *channel* is one scalar measurement stream a plugin perturbs. Names are
free-form strings agreed here:

| Plugin          | Channels |
|-----------------|----------|
| laser           | `range` |
| imu             | `orientation_z`, `angular_velocity_z`, `linear_acceleration_x`, `linear_acceleration_y` (others `*_x/_y/_z` reserved) |
| gps             | `gps_x`, `gps_y` (metres of horizontal position error, applied before the ECEF→lat/lon conversion is *not* re-run; error is added in lat/lon-degrees via the per-channel value) |
| diff_drive      | `odom_pose_x`, `odom_pose_y`, `odom_pose_yaw`, `odom_twist_x`, `odom_twist_y`, `odom_twist_yaw` |
| tricycle_drive  | same six `odom_*` channels as diff_drive |

A channel absent from the model file falls back to the plugin's legacy constant
`std` (so partial models are valid and safe).

## 4. File format (schema_version 1)

A single JSON object (parsed in C++ via yaml-cpp, which is a JSON superset):

```json
{
  "schema_version": 1,
  "model_type": "parametric_linear",
  "metadata": {
    "fit_source": "synthetic|rosbag",
    "created": "2026-06-10",
    "notes": "free text"
  },
  "channels": {
    "range": {
      "base_std": 0.012,
      "std_coef": { "speed": 0.004, "age": 0.0009, "darkness": 0.020 },
      "surface_std_offset": { "0": 0.0, "1": 0.010, "2": 0.030 },
      "base_mean": 0.0,
      "mean_coef": { "speed": 0.0, "age": 0.0, "darkness": 0.0 },
      "surface_mean_offset": { "0": 0.0 }
    }
  }
}
```

### Evaluation (closed form, what `noise_model.cpp` implements)

For a channel `c` and context `ctx`, with `darkness = 1 - ctx.lighting`:

```
std(c)  = max(0, base_std
                 + std_coef.speed    * ctx.speed
                 + std_coef.age      * ctx.sensor_age
                 + std_coef.darkness * darkness
                 + surface_std_offset[ctx.surface_id])

mean(c) =        base_mean
                 + mean_coef.speed    * ctx.speed
                 + mean_coef.age      * ctx.sensor_age
                 + mean_coef.darkness * darkness
                 + surface_mean_offset[ctx.surface_id]
```

The sample is `mean(c) + std(c) * N(0,1)` drawn from the plugin's seeded engine.
A missing surface key contributes `0`. All coefficient sub-keys are optional and
default to `0`; `base_std` defaults to `0`.

### Required vs optional

* Required: `schema_version` (must equal `1`), `model_type`
  (`"parametric_linear"`), `channels` (a map, possibly empty).
* Everything else is optional with the defaults above.

A file with the wrong `schema_version` or `model_type`, or that is unparsable,
makes the loader throw a `YAMLException`; the plugin then logs and falls back to
**legacy** constant-`std` behaviour (no crash, no silent change of meaning).

## 5. Backward compatibility

When a plugin has no `noise_model:` key, or the file fails to load, the engine
uses the built-in `legacy_gaussian` model: the exact pre-existing constant-`std`
draw path is preserved **byte-for-byte** (same distribution objects, same RNG
draw order), so existing worlds and determinism tests are unaffected.

## 6. Calibrate → export → load round trip

1. `noise_cal/fit.py` ingests labeled CSV (`surface_id,speed,lighting,age,
   channel,residual`) — from rosbags or the synthetic generator — and fits the
   per-channel coefficients above.
2. `noise_cal/exporter.py` writes a schema-valid JSON matching this document.
3. A world/model YAML references the file via `noise_model: path/to/model.json`;
   `noise_model.cpp` loads and validates it against `schema_version 1`.
