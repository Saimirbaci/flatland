# Variable Mass / Center-of-Gravity Payload Model

This document specifies the `VariablePayload` model plugin and the generic
engine hook it is built on, which together let a body's **mass and center of
gravity change over a simulation run**. The motivating use case is an AGV
cleaning robot whose water tank drains during operation: as the tank empties the
robot gets lighter and its center of gravity migrates, so its handling (inertia,
response to drive forces and external pushes) changes the way a real robot's
does.

## Why

Out of the box a Flatland body's mass is fixed: Box2D derives it once from
`density × footprint area` for every fixture and never revisits it. Nothing in
the engine ever calls `b2Body::SetMassData`, so payload that is taken on or shed
during a run (water, parcels, a swept-up load) has no effect on the dynamics.

`VariablePayload` closes that gap. It keeps the body's empty (chassis) mass and
adds a payload of up to `max_payload_mass` on top of it, scaled by a fill
fraction in `[0, 1]`. The fill fraction can drain/fill at a constant rate on the
simulation clock and/or be commanded at runtime over a ROS topic. Each physics
step the plugin recomputes the combined mass, center of gravity, and rotational
inertia and applies them through the engine-level `Body::SetMassData` hook.

## The generic hook (`flatland_server::Body`)

Two reusable, plugin-agnostic methods on the core `Body` class wrap the Box2D
mass API so any plugin can drive mass without hand-rolling Box2D calls:

```cpp
b2MassData GetMassData() const;
void SetMassData(double mass, const b2Vec2 &local_center, double inertia);
```

- `GetMassData()` returns the body's current mass (kg), local-frame center of
  gravity, and rotational inertia **about the body local origin** (`b2MassData`
  convention).
- `SetMassData(mass, local_center, inertia)` overrides them. `mass` must be a
  positive finite value (throws `std::invalid_argument` otherwise). `inertia` is
  the rotational inertia about the body local origin; pass a value `<= 0` to keep
  the body's current rotational inertia. The hook deliberately **never** calls
  `b2Body::ResetMassData` afterward — doing so would discard the override and
  recompute the mass from the fixture densities.

## Mass model

Let `f ∈ [0, 1]` be the current fill fraction. With empty mass `m_base` at
local-frame center `c_base`, and a payload of `m_pay = f · max_payload_mass` at
local-frame center `payload_center`:

```
m   = m_base + m_pay                                   # total mass
c   = (m_base · c_base + m_pay · payload_center) / m   # combined CoG (local frame)
```

Inertia is handled by the **parallel-axis theorem**, so the rotational behavior
stays physical as the CoG moves:

- The empty body keeps its own inertia `I_base_com` (about its own CoM), captured
  once at load time from the fixtures.
- The payload is treated as a **point mass** (no intrinsic inertia) at
  `payload_center`.
- Both are shifted to the new combined CoM and summed:

```
I_com    = I_base_com + m_base·|c_base − c|² + m_pay·|payload_center − c|²
I_origin = I_com + m·|c|²        # Box2D wants inertia about the local origin
```

`SetMassData(m, c, I_origin)` is then applied. When the fill fraction has not
changed since the last step the recompute is skipped, so a static or fully
drained tank costs nothing.

### Empty (base) mass

`base_mass` sets the empty chassis mass explicitly. If it is omitted (or `0`),
the empty mass is derived straight from the body's fixtures (the usual
`density × area`). When given, the fixture geometry's CoG is kept and its inertia
is scaled by the mass ratio (same shape, different mass).

## YAML schema

```yaml
plugins:
  - type: VariablePayload
    name: water_tank
    body: base                  # target body name (required)
    base_mass: 300.0            # empty-body mass [kg]; 0 (default) => derive from fixtures
    max_payload_mass: 200.0     # payload mass at fill == 1.0 [kg] (required, >= 0)
    payload_center: [0.3, 0.0]  # payload CoG in the body local frame [m] (default [0, 0])
    initial_fill: 1.0           # starting fill fraction [0, 1] (default 1.0)
    fill_rate: -0.05            # constant fill change rate [fraction/s] (default 0 = constant)
    fill_topic: "fill_level"    # std_msgs/Float64 fill-fraction command (default "" = none)
    state_topic: "payload_state"  # std_msgs/Float64MultiArray [mass, cx, cy] (default "" = none)
```

## Runtime interface

- **`fill_topic`** (`std_msgs/Float64`, optional) — sets the current fill
  fraction, clamped to `[0, 1]`. The callback only stores the value; the mass is
  applied in the next physics step (so commands never mutate Box2D state from a
  ROS callback thread). A non-zero `fill_rate` continues to act on top of the
  commanded level.
- **`state_topic`** (`std_msgs/Float64MultiArray`, optional) — publishes
  `[mass, cx, cy]` (total mass and the combined local-frame CoG) whenever the
  mass is reapplied.

## Example: draining AGV cleaning water tank

`test/variable_payload_tests/water_tank_agv.model.yaml` is a 0.5 m circular AGV
chassis (300 kg) carrying a 200 kg water tank whose CoG sits 0.3 m ahead of the
body origin, draining at 5 % of capacity per second:

- Full: `m = 500 kg`, CoG at `x = 0.12 m` (forward of center).
- Empty: `m = 300 kg`, CoG back at the chassis center (`x = 0`).

`test/variable_payload_tests/water_tank.world.yaml` loads it on a small map.

## Performance

Applying `SetMassData` is a handful of floating-point operations plus a Box2D
mass/velocity update — negligible per step. The plugin additionally skips the
call entirely on any step where the fill fraction is unchanged, so an idle or
fully drained tank adds no per-step cost.
