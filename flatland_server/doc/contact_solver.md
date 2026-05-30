# Contact-Solver Configuration (AGV-scale dynamics)

Flatland integrates rigid-body physics with Box2D on a fixed timestep. The
contact solver's accuracy is governed by four knobs, all set in the world YAML
`properties:` block. The defaults reproduce Flatland's historical behavior; the
extra knobs (`substeps`, `continuous_physics`) exist to keep contacts stable at
**AGV mass and speed ranges** (heavy and/or fast bodies), where the stock
settings let bodies sink into or tunnel through walls.

> Box2D version: the engine currently vendors **Box2D 2.3.2**
> (`flatland_server/thirdparty/Box2D`). All knobs below are supported by that
> version. A version bump (2.4.x re-vendor and/or the 3.x TGS solver) is tracked
> as a separate, build-gated follow-up — see [Limits](#limits-of-box2ds-2d-model).

## Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `velocity_iterations` | `10` | Box2D constraint-solver velocity iterations per (sub-)step. More iterations → more accurate contact/joint velocities, higher CPU cost. |
| `position_iterations` | `10` | Box2D position-correction iterations per (sub-)step. More iterations → less residual penetration. |
| `substeps` | `1` | Number of equal sub-steps Box2D integrates per outer step. The outer `step_size` is split into `step_size / substeps`; finer dt sharply improves stability for heavy/fast bodies. Must be `>= 1`. |
| `continuous_physics` | `true` | Enables Box2D continuous collision detection (CCD). Catches fast bodies that would otherwise pass through thin static geometry in a single step. Disable only to trade realism for throughput. |

Example (the recommended AGV-scale configuration):

```yaml
properties:
  velocity_iterations: 12
  position_iterations: 8
  substeps: 4
  continuous_physics: true
```

## How sub-stepping works

`World::Update` (`flatland_server/src/world.cpp`) integrates physics in
`substeps` equal sub-steps:

```cpp
const double sub_dt = timekeeper.GetStepSize() / substeps;
for (int i = 0; i < substeps; i++)
  physics_world_->Step(sub_dt, velocity_iterations, position_iterations);
```

Key properties:

- The **published clock is unchanged**: `Timekeeper::StepTime()` still advances
  by exactly one `step_size` per outer step, and `/clock` ticks at the same
  rate.
- **Plugin cadence is unchanged**: `BeforePhysicsStep` / `AfterPhysicsStep` fire
  once per *outer* step, so sensor/drive plugins see the same timing they always
  did. Only the internal Box2D integration is finer.
- `substeps: 1` is **bit-for-bit identical** to the previous single
  `Step(step_size, …)` call, so existing worlds and the determinism baseline are
  untouched until you opt in.

Why it helps: Box2D's semi-implicit Euler integration and its
penetration-recovery (Baumgarte) term degrade as `dt` grows relative to body
speed and contact stiffness. A 1500 kg body at 2 m/s moving 10 mm/step pushes
deep into a wall before the position solver can react. Halving `dt` (doubling
`substeps`) roughly halves the per-step displacement and the peak penetration,
at a near-linear CPU cost (`substeps` Box2D steps per frame).

## Continuous physics (CCD) and tunnelling

With `continuous_physics: true`, Box2D computes the time-of-impact for
fast-moving bodies against **static** geometry and clips their motion at the
contact, instead of letting them teleport past a thin wall when
`speed * dt > wall_thickness`. This is the cheap, always-on guard for
tunnelling; `substeps` is the complementary guard that also improves resting
contact quality. For fast dynamic-vs-dynamic tunnelling, Box2D additionally
requires the bullet flag, which Flatland does not currently expose (noted under
[Limits](#limits-of-box2ds-2d-model)).

## Recommended AGV-scale values

These are **analytical** recommendations derived from the integration math and
the geometry of the stress worlds (`flatland_server/test/agv_stress_tests/`).
They have **not** yet been confirmed by an empirical benchmark sweep on a built
workspace — that is a tracked follow-up. Run the sweep with:

```bash
scripts/solver_tuning_sweep.sh        # sweeps substeps × iters × CCD, reports iter/sec
```

| AGV envelope | `substeps` | `velocity_iterations` | `position_iterations` | `continuous_physics` |
|--------------|-----------:|----------------------:|----------------------:|:--------------------:|
| Light & slow (≤200 kg, ≤1 m/s) | 1 | 10 | 8 | true |
| Typical (≤800 kg, ≤2 m/s)      | 2 | 10 | 8 | true |
| Heavy / fast (≤1500 kg, ≤3 m/s)| 4 | 12 | 8 | true |

Rationale and trade-offs:

- **`position_iterations` 8 vs 10**: lowering it slightly recovers throughput;
  penetration is dominated far more by `substeps` than by the last few position
  iterations, so the headroom is better spent on `substeps`.
- **`velocity_iterations` 12 for heavy bodies**: heavy bodies generate large
  contact impulses; a couple more velocity iterations reduce jitter/bounce.
- **Throughput cost**: `substeps: 4` costs ≈4× the Box2D step time (not 4× the
  whole frame — plugins and viz are unaffected). On the benchmark world the
  Box2D step is a fraction of the frame, so the end-to-end cost is well under
  4×, but confirm with the sweep before committing project-wide defaults. Do not
  globally raise the defaults: it would erode the perf headroom from the
  `simplify_map` work and shift the determinism baseline (see below).

## Determinism implications

Box2D is single-threaded and integrates deterministically for a fixed
`step_size` and iteration count, so a fixed solver configuration is reproducible
(see `flatland_server/doc/determinism.md`). Two caveats:

- Changing `substeps`, `velocity_iterations`, `position_iterations`, or
  `continuous_physics` **changes the physics trajectory** and therefore the
  byte-level output. Pin these values (alongside `seed` and `step_size`) when
  comparing runs. `substeps: 1` + the historical iteration counts reproduce the
  pre-existing baseline exactly.
- These are world-level `properties`, so a reproducible run is defined by the
  world file plus the seed, with no hidden node-level state.

## Limits of Box2D's 2D model

Box2D is a strictly planar (x, y, yaw) solver. Several AGV-realism gaps are
inherent to that model and are **not** fixable by solver tuning; they are
tracked as follow-ups and overlap the in-flight friction/payload epics:

- **No z-axis load transfer or tip-over.** Mass is area density in a plane;
  there is no normal load, no weight shift under acceleration, no rollover. A
  "resting stack" has no gravity to settle (Flatland gravity is always `(0,0)`),
  so stack-compression stability is not representable. Couples to the
  *Variable mass / center-of-gravity payload* epic.
- **Single-coefficient Coulomb friction.** Box2D mixes one friction coefficient
  per contact with no dependence on normal force, no static-vs-kinetic split,
  and no slip/traction curve — inadequate for wheel-ground modeling. Couples to
  the *High-fidelity wheel-ground contact and friction* and *Surface-dependent
  friction / traction loss* epics.
- **No out-of-plane contact.** Ramps, curbs, and uneven floors cannot be
  represented; everything is a top-down silhouette.
- **No exposed bullet flag** for dynamic-vs-dynamic CCD (only dynamic-vs-static
  is guarded by `continuous_physics`).

A **2.5D extension** (per-body height/clearance bands and normal-load-aware
friction) and a **Box2D 3.x TGS-solver port** are proposed follow-ups; the
latter would improve stiff/heavy-contact stability beyond what iteration and
sub-step tuning can reach on 2.3.2.
