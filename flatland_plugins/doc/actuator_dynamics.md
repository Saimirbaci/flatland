# Actuator / Motor Dynamics

This document specifies the actuator/motor dynamics layered onto the `DiffDrive`
and `TricycleDrive` plugins, the YAML it accepts, and how it composes with the
existing acceleration ramps (`*_dynamics`) and the wheel-ground friction model.

## Why

Even with the `*_dynamics` acceleration ramps and the `friction` drive mode, the
drive plugins still respond to a velocity command *too cleanly*: the command
takes effect the instant it is received, the smallest command produces motion,
and the only thing bounding acceleration is the configured ramp (not the motor).
Real drivetrains add three effects that matter for sim-to-real transfer and are
the prerequisite knobs for **actuator fault injection**:

- **Command latency** — transport / communication deadtime between issuing a
  command and the motor acting on it.
- **Deadband** — a motor / static-friction deadzone: commands below a threshold
  produce no motion at all.
- **Torque / force limit** — a bounded actuator effort that caps the achievable
  acceleration (`a_max = F/m`, `alpha_max = tau/I`) and, in `friction` mode, caps
  the per-wheel motor force before the grip cap (`effective = min(grip, motor)`).

These are implemented in the small, pure-utility `ActuatorDynamics` class
(`actuator_dynamics.{h,cpp}`), one instance per command axis, **composed** with
`DynamicsLimits` and `WheelFrictionModel` — no ramp logic is duplicated.

## Signal ordering

Every physics step, for each axis, the drive plugin runs:

1. **Raw twist** captured in the twist callback (cached on the plugin).
2. **Command latency** — the cached command is pushed into a FIFO stamped with
   the **Timekeeper sim time** (never wall-clock, preserving determinism) and the
   command delayed by `command_latency` seconds is pulled back out (zero-order
   hold: a command pushed at `t` takes effect at `t + command_latency`).
3. **Deadband** — `|cmd| < deadband` is forced to `0`; above the threshold the
   command passes through (optionally rescaled so the output is continuous across
   the deadband edge).
4. **Acceleration ramp** — the **existing** `DynamicsLimits::Limit()` for that
   axis (`acceleration_limit` / `deceleration_limit` / `velocity_limit`).
5. **Torque/force cap** — the effort limit `max_force`/`max_torque` is converted
   to a per-step acceleration cap `a_max = effort / inertia` and used to clamp the
   acceleration achieved by step 4. The inertia (body mass for linear, rotational
   inertia for angular) is **re-read from the Box2D body each step**, so the
   variable-mass / payload model is respected. In `friction` drive mode the same
   `max_force` additionally clamps the per-wheel longitudinal traction before it
   is applied, so the effective force is `min(grip, motor)`.

The torque limit therefore *modulates* the ramp (a second, tighter acceleration
bound) rather than adding an independent ramp.

## YAML schema

The feature is **opt-in via per-axis subnodes**. Omitting them (or leaving them
empty) is a pure pass-through, so existing worlds are byte-for-byte unchanged.

```yaml
plugins:
  - type: DiffDrive
    # ...existing keys...
    linear_actuator:           # forward (linear.x) command axis
      command_latency: 0.0     # [s]     transport deadtime          (0 = off)
      deadband: 0.0            # [m/s]   deadzone threshold          (0 = off)
      deadband_rescale: false  #         rescale above threshold for continuity
      max_force: 0.0           # [N]     actuator force => a_max=F/m  (0 = off)
    angular_actuator:          # yaw (angular.z) command axis
      command_latency: 0.0     # [s]
      deadband: 0.0            # [rad/s]
      deadband_rescale: false
      max_torque: 0.0          # [N*m]   actuator torque => alpha_max=tau/I

  - type: TricycleDrive
    # ...existing keys...
    drive_actuator:            # front-wheel drive speed (linear.x) axis
      command_latency: 0.0     # [s]
      deadband: 0.0            # [m/s]
      max_force: 0.0           # [N]
    steer_actuator:            # steering angle (angular.z) axis
      command_latency: 0.0     # [s]
      deadband: 0.0            # [rad]  applied to the steering ANGLE command
      max_torque: 0.0          # [N*m]  caps the steering angular acceleration
```

Notes:

- `max_force` (linear axes) and `max_torque` (angular axes) map onto the same
  internal effort limit; use whichever fits the axis.
- For `DiffDrive` in `friction` mode the linear `max_force` is the **total**
  drivetrain force and is split evenly across the two drive wheels; for
  `TricycleDrive` the front wheel is the sole drive wheel and carries the full
  limit.
- **Tricycle steering** is an angle command (not a velocity), so latency and
  deadband are applied to the steering **angle**. The steering `max_torque` caps
  the steering angular acceleration; because the 2nd-order kinematic steering
  model has no dedicated steering-column inertia, the body rotational inertia is
  used as a coarse proxy.

## Interaction with `drive_mode` and `*_dynamics`

| Layer            | Kinematic mode                        | Friction mode                                   |
|------------------|---------------------------------------|-------------------------------------------------|
| `*_dynamics`     | ramps the applied velocity            | ramps the *commanded* velocity fed to traction  |
| `max_force/torque` accel cap | clamps the ramped velocity's accel | clamps the ramped commanded velocity's accel |
| `max_force` (friction only) | n/a                       | clamps per-wheel longitudinal force `min(grip, motor)` |
| latency, deadband | applied to the command pre-ramp      | applied to the command pre-ramp                 |

All caps are disabled at their `0` default, so any subset can be enabled
independently.

## Determinism

The latency FIFO is keyed solely on `Timekeeper` sim time, and its length is
bounded (it only ever holds the in-flight commands, i.e. `~latency / timestep`
entries, with a hard safety cap). Given the same run seed the actuator response
is reproducible — consistent with the deterministic-simulation work.

## Handoff: actuator fault injection

This is the scaffolding for the **Actuator faults** task. A fault layer can
perturb exactly these parameters at runtime — e.g. inflate `command_latency`
(comm dropout), widen `deadband` (sticky motor), or shrink `max_force`/
`max_torque` (degraded/failing actuator) — without touching the drive plugins,
because the effects are already isolated in `ActuatorDynamics`.

## Related / adjacent tasks

- **Wheel-ground contact & anisotropic friction model** — see
  [`wheel_friction.md`](wheel_friction.md); the actuator force cap composes with
  its grip cap as `min(grip, motor)`.
- **Variable mass / center-of-gravity payload model** — the per-step mass /
  inertia read means the acceleration cap tracks payload changes automatically.
- **Actuator faults** (*in progress*) — the consumer of these parameters.
