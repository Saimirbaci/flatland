# Deterministic / Reproducible Simulation Runs

Flatland can run **byte-reproducibly**: given the same `seed`, the same world
YAML, and the same `step_size`, every run produces identical published output
(sensor noise, odometry, model trajectories). This is the foundation for
regression replay and root-cause analysis against a known ground truth.

## What makes a run reproducible

Two things were previously nondeterministic and are now seed-driven:

1. **Plugin / world RNG.** A single seeded authority,
   `flatland_server::RngManager` (`flatland_server/random.h`), is seeded once at
   world startup. Each consumer asks for a private engine derived from a stable
   key (`model_name/plugin_name`), so a plugin's stream is independent of how
   many other plugins exist or what order they load in. The shipped plugins that
   carry noise — `laser`, `imu`, `diff_drive`, `tricycle_drive`, and
   `world_random_wall` — all draw from it.
2. **The Lua preprocessor.** `$eval` expressions that call `math.random` are
   seeded from the same run seed, so YAML preprocessing is reproducible.

Box2D itself is single-threaded and integrates on a fixed timestep, so the
physics is already deterministic for a fixed `step_size` and iteration count.
The contact-solver knobs (`velocity_iterations`, `position_iterations`,
`substeps`, `continuous_physics`) are part of that fixed configuration: changing
any of them changes the physics trajectory and therefore the byte-level output,
so pin them alongside `seed` and `step_size` when comparing runs. `substeps: 1`
with the default iteration counts reproduces the historical baseline exactly.
See `flatland_server/doc/contact_solver.md` for the full solver configuration
and AGV-scale tuning.

## Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `seed` | `-1` | Run seed. `-1` (or any value `<0`) means **nondeterministic** — a fresh seed is drawn from `std::random_device` and logged so the run can still be replayed. Any value `>=0` makes the run reproducible. |
| `real_time_factor` | `1.0` | Simulated seconds per real second. The loop advances physics by exactly one `step_size` per iteration and paces wall time to `step_size / real_time_factor`. `<=0` runs unthrottled (as fast as possible). The benchmark node always runs unthrottled. |

### Seed precedence

`seed` is resolved in this order (first match wins):

1. The node parameter `seed` (if `>= 0`).
2. The world YAML `properties.seed` (if present).
3. Nondeterministic — a `std::random_device` value, logged at startup.

The effective seed is always logged at `INFO` (`Using deterministic seed N` /
`using nondeterministic seed N`) so a run is replayable by passing that value
back as `seed`.

## Examples

Reproducible run from the launch file:

```bash
roslaunch flatland_server server.launch seed:=42 real_time_factor:=1.0
```

Pin the seed inside a world file instead (node param still overrides it):

```yaml
properties:
  velocity_iterations: 10
  position_iterations: 10
  seed: 42
```

Run as fast as possible (e.g. for data generation), still reproducible:

```bash
roslaunch flatland_server server.launch seed:=42 real_time_factor:=0
```

To restore the old behavior of fresh random noise on every run, leave `seed` at
its default of `-1`.

## Notes / limitations

- Reproducibility is guaranteed for a given build on a given platform. The key
  hashing uses FNV-1a (not `std::hash`) to be stable across compilers, but the
  underlying `std::default_random_engine` / `std::normal_distribution`
  implementations are standard-library-defined, so cross-toolchain byte equality
  is not guaranteed.
- World-file *top-level* `$eval math.random` (as opposed to model files) is only
  seeded reproducibly when `seed` is supplied as a node parameter, because the
  world file is preprocessed before `properties.seed` is parsed.
- The Lua sandbox was not broadened; only `math.randomseed` is called. Worlds
  are still arbitrary-code-capable — only load worlds from trusted sources.
