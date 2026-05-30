# Surface-Dependent Friction & Traction Loss

Flatland can vary ground friction **by region** so a robot loses traction over
wet patches, spills, or ramps and recovers on dry ground — the realistic
behaviour a cleaning robot laying down water needs. Transitions are **smooth**
(bilinearly interpolated), not step changes, so the force-based traction /
contact solver never sees a friction discontinuity (which would cause chatter).

## Concept

A world owns one `SurfaceFrictionField`
(`flatland_server/surface_friction_field.h`): a raster of friction
**multipliers** over the world plane. The multiplier is unitless — `1.0` is
nominal/dry, `< 1.0` is reduced grip (wet/slippery). It does **not** replace the
friction coefficients; it *scales* them.

The friction-mode drive plugins (`DiffDrive`, `TricycleDrive` with
`drive_mode: friction`) sample the field once per wheel per physics step at the
wheel's world contact point and pass the result into
`WheelFrictionModel::ComputeWheelForce`, which scales the Coulomb ceiling
`μ · N → surface_factor · μ · N`. See
[`../../flatland_plugins/doc/wheel_friction.md`](../../flatland_plugins/doc/wheel_friction.md).

A world **without** a `surface_friction` block, or any query outside the raster
extent, returns `1.0` — existing worlds are completely unchanged.

## World YAML schema

Add an optional top-level `surface_friction` block (sibling of `layers` /
`models` / `properties`):

```yaml
surface_friction:
  map: "friction_map.png"   # grayscale image; relative to the world dir or absolute
  resolution: 0.05          # metres per pixel (> 0)
  origin: [-1.0, -1.0]      # world (x, y) of the image lower-left corner [m]
  mu_min: 0.15              # multiplier for fully-wet (black, 0) pixels
  mu_max: 1.0               # multiplier for fully-dry (white, 255) pixels
  min_factor: 0.05          # optional lower clamp on the sampled multiplier
```

| Field        | Required | Default | Meaning |
|--------------|----------|---------|---------|
| `map`        | yes      | —       | Grayscale image; **white (255) = dry**, **black (0) = wet**. |
| `resolution` | yes      | —       | Metres per pixel. Must be `> 0`. |
| `origin`     | no       | `[0,0]` | World coordinate of the image's lower-left corner. |
| `mu_min`     | no       | `0.3`   | Multiplier mapped to black pixels (wettest). |
| `mu_max`     | no       | `1.0`   | Multiplier mapped to white pixels (driest). |
| `min_factor` | no       | `0.05`  | Hard lower clamp so grip never reaches exactly zero. |

Pixel intensity `p ∈ [0,255]` maps linearly to
`mu_min + (p/255) · (mu_max − mu_min)`, then is bilinearly interpolated between
neighbouring cells and clamped to `min_factor`. Make the image border the
ambient (dry/white) value so the raster edge blends into the surrounding
`1.0` field without a step.

`μ` values may be `$eval` Lua expressions (the world YAML is preprocessed before
parsing), so spill multipliers can be parameterised like any other field. Bad
config (missing `map`, non-positive `resolution`, unreadable image) throws a
`YAMLException` with file/field context.

## Smoothing & continuity

Sampling is bilinear over the raster, so the multiplier is **C0-continuous**
across cell and region boundaries. A body crossing a wet boundary sees the grip
ramp over the image's transition band rather than flipping instantly — this is
the explicit anti-step-change guarantee covered by
`flatland_server/test/surface_friction_field_test.cpp`
(`transition_is_continuous_and_monotonic`) and exercised end-to-end by
`flatland_plugins/test/surface_friction_test.cpp`.

## Scope (v1) and future work

v1 applies surface friction through the **wheel friction model only** (the
friction-mode drives). Flatland is top-down with zero gravity, so the floor is
not a Box2D collision surface — there is no normal force pressing a body onto
the floor, hence no floor *contact* whose friction a `PreSolve`
`b2Contact::SetFriction` override could scale. The wheel model is therefore the
physically meaningful hook for wet-floor traction loss.

Deferred to later tasks:

- A `World::PreSolve` contact-friction override to also affect non-friction
  models sliding against **walls/obstacles** (niche in 2D; adds per-contact cost
  and a no-double-count rule vs the wheel model).
- Dynamic mutation of the field (a cleaning robot "laying down water" wetting
  cells at runtime), which the raster representation was chosen to support.

## Security

The friction map image and `μ` values come from the world YAML, which is already
arbitrary-code-capable via the Lua preprocessor. This feature grants **no new**
filesystem/process capability — only load worlds from trusted sources, as with
any Flatland world.
