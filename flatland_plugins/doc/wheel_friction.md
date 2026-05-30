# Wheel–Ground Contact & Anisotropic Friction Model

This document specifies the high-fidelity tangential-slip traction model added to
the `DiffDrive` and `TricycleDrive` plugins, the YAML it accepts, and the
calibrated default parameter set (calibration against measured robot data is
**pending** — the defaults below are physically reasonable placeholders, not
measured values).

## Why

The original drive plugins move the body *kinematically*: every physics step they
overwrite the body velocity with the commanded twist via
`b2Body::SetLinearVelocity()`. The robot therefore reaches any commanded velocity
instantly, never slips, and never loses traction. This is the `kinematic` drive
mode and remains the **default** for backward compatibility.

The new `friction` drive mode replaces that ideal path with force-based traction:
each wheel applies a Coulomb-limited friction force at its contact patch, so
commanded motion that exceeds available grip produces real longitudinal (wheel
spin / lock) and lateral (side-slip) slip.

## Model

For each wheel, per physics step of size `dt`:

1. **Wheel frame.** The longitudinal axis is the wheel's rolling direction; the
   lateral axis is perpendicular to it. For a diff-drive wheel this is the body
   frame; for the tricycle front wheel it is the body frame rotated by the
   steering angle `theta_f`.

2. **Commanded surface velocity.** The no-slip contact-patch velocity the wheel
   rotation imparts — longitudinal = drive speed, lateral = 0 (wheels are not
   driven sideways). For a diff-drive wheel at body-frame `y = y_w`, the
   longitudinal command is `v - w * y_w` (from `omega × r`).

3. **Actual contact velocity.** `b2Body::GetLinearVelocityFromLocalPoint(wheel)`
   projected onto the wheel axes.

4. **Slip velocity** = commanded − actual, per axis.

5. **Coulomb-limited force.** Per axis, the force that would exactly null the
   slip this step is `F_null = m_share · slip / dt`, where the share of mass on
   the wheel is recovered from its normal load `m_share = N / g`. This is clamped
   to the friction ceiling:

   ```
   F_axis = clamp( m_share · slip / dt , −μ·N , +μ·N )
   ```

   `μ` is the **static** coefficient while `|slip| < static_slip_threshold`
   (grip) and the **kinetic** coefficient above it (slipping). Longitudinal and
   lateral use independent coefficients (anisotropy). Because the unclamped term
   is exactly the nulling impulse, the integrator never overshoots — the path is
   numerically stable at the existing AGV-tuned contact-solver settings (see
   [`../../flatland_server/doc/contact_solver.md`](../../flatland_server/doc/contact_solver.md)).

6. **Apply.** The wheel-frame force is rotated to the world frame and applied via
   `b2Body::ApplyForce()` at the wheel's world contact point. No velocity is
   overwritten, so the Box2D solver integrates the body normally.

### Normal load

`N` is the static weight `m·g` split evenly across the drive wheels: two for
diff-drive, three (front + two rear) for the tricycle. Dynamic z-load transfer
and centre-of-gravity shifts are intentionally **out of scope** here and are the
subject of the *2.5D / richer-contact extension (z-load, normal-force friction)*
backlog task — that work should replace the even split in
`DiffDrive::ApplyFrictionDrive` / `TricycleDrive::ApplyFrictionDrive` with a
load-transfer model and feed the per-wheel `N` into the same
`WheelFrictionModel::ComputeWheelForce`.

### Per-wheel roles (tricycle)

- **Front wheel** — steered and driven at `v_f` along its heading; resists both
  longitudinal and lateral slip.
- **Rear wheels** — passive rolling: longitudinally free (`slip_long = 0`),
  laterally gripped, so they enforce the rolling constraint that turns steering
  into rotation.

## YAML schema

```yaml
plugins:
  - type: DiffDrive          # or TricycleDrive
    # ...existing keys...
    drive_mode: friction      # "kinematic" (default) | "friction"
    wheel_separation: 0.4     # DiffDrive only: track width [m] (required, > 0)
    wheel_friction:
      enabled: true
      mu_long_static: 1.0     # longitudinal static friction coefficient
      mu_long_kinetic: 0.8    # longitudinal kinetic friction coefficient
      mu_lat_static: 1.2      # lateral static friction coefficient
      mu_lat_kinetic: 1.0     # lateral kinetic friction coefficient
      static_slip_threshold: 0.05  # static→kinetic transition slip speed [m/s]
```

Omitting `drive_mode` (or setting `kinematic`) leaves the plugin behaving exactly
as before — existing worlds are unchanged. `wheel_separation` is only consumed in
friction mode and `DiffDrive` throws a `YAMLException` if it is missing or
non-positive there.

## Calibrated default parameter set (calibration pending)

| Parameter               | Default | Units | Notes |
|-------------------------|---------|-------|-------|
| `mu_long_static`        | 1.0     | –     | Rubber-on-concrete-ish dry grip |
| `mu_long_kinetic`       | 0.8     | –     | ~20% drop once spinning |
| `mu_lat_static`         | 1.2     | –     | Lateral grip > longitudinal (tires resist side-slip) |
| `mu_lat_kinetic`        | 1.0     | –     | |
| `static_slip_threshold` | 0.05    | m/s   | Below this the wheel is treated as gripping |

### Calibration methodology (to apply once measured data is available)

1. **Longitudinal kinetic (`mu_long_kinetic`).** Command a step velocity well
   above the grip limit on the real robot, measure the achieved acceleration
   `a`; then `mu_long_kinetic ≈ a / g` while the wheels are slipping.
2. **Longitudinal static (`mu_long_static`).** Find the largest commanded
   acceleration the robot can follow *without* wheel slip; `mu_long_static ≈
   a_max / g`.
3. **Lateral coefficients.** Drive a constant-radius arc at increasing speed;
   the lateral acceleration at which the robot begins to under-steer (side-slip)
   gives `mu_lat_static`; the sustained lateral acceleration while sliding gives
   `mu_lat_kinetic`.
4. **`static_slip_threshold`.** Set to the slip speed at which measured traction
   transitions from the static plateau to the kinetic plateau (typically a few
   cm/s for hard tires).

## Related / adjacent tasks

- **Surface-dependent friction and traction loss** (*in progress*) — that task
  should add a per-surface `μ` override (e.g. keyed off the layer/material under
  the wheel) that scales the coefficients here. The integration hook is
  `WheelFrictionModel::ComputeWheelForce`: multiply the configured `μ` by a
  surface factor before the Coulomb clamp.
- **2.5D / richer-contact extension (z-load, normal-force friction)**
  (*backlog*) — replaces the even `N = m·g / n_wheels` split with dynamic load
  transfer, as noted above.
- **Variable mass / center-of-gravity payload model** (*in progress*) — feeds
  the per-wheel normal load once CoG-aware load distribution lands.
