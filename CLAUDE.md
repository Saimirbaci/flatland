# CLAUDE.md — Flatland

## What This Is
Flatland is a performance-centric 2D robot simulator for ROS 1. It loads a world
from YAML (layers + models), simulates rigid-body physics with a vendored copy of
Box2D, runs robot behaviour through `pluginlib`-loaded C++ plugins (laser, diff
drive, IMU, GPS, bumper, …), and publishes/visualizes everything over ROS topics
and RViz. It is a C++11 catkin metapackage split across five packages. Map layers
are converted from occupancy images to Box2D collision geometry via OpenCV
`findContours` + chain loops, with an optional `simplify_map` step for speed.

## Quick Start
This is a catkin package set — it is built inside a catkin workspace, not standalone.
```bash
# from your catkin workspace root, with this repo cloned into src/
rosdep install --from-paths src --ignore-src        # install ROS deps
catkin build                                        # build all 5 packages
source devel/setup.bash

# run the simulator (loads a world yaml, starts the ROS node)
roslaunch flatland_server server.launch

# run the headless performance benchmark
roslaunch flatland_server benchmark.launch

# run tests for one package + read results
catkin run_tests flatland_server --no-deps
catkin_test_results build/flatland_server
```
CI (`.github/workflows/industrial_ci_action.yml`) runs `ros-industrial/industrial_ci`
with `ROS_DISTRO: noetic`. Before pushing, code must pass `clang-format --style=file`
and `clang-tidy` — see `scripts/ci_prebuild.sh`.

## Architecture
Five catkin packages at the repo root:
- **`flatland/`** — metapackage; aggregates the others, holds no code.
- **`flatland_msgs/`** — ROS `.msg`/`.srv` definitions (e.g. spawn/move model services, collisions, debug topics).
- **`flatland_server/`** — the core engine. World/model/layer loading, the Box2D
  physics step, the plugin manager, ROS service interface, timekeeping, and the
  `flatland_server_node` entrypoint. ~58 files; this is where most engine work happens.
- **`flatland_plugins/`** — the `pluginlib` plugin library: sensors and actuators
  that attach to models (laser, diff_drive, tricycle_drive, imu, gps, bumper,
  bool_sensor, tween, model_tf_publisher, update_timer, world_modifier, …).
- **`flatland_viz/`** — RViz/Qt tools for spawning and inspecting models interactively.

Vendored third-party code lives under `flatland_server/thirdparty/` (Box2D,
ThreadPool, Tweeny) — treat as read-only.

## Key Files
- `flatland_server/src/flatland_server_node.cpp` — process entrypoint.
- `flatland_server/src/simulation_manager.cpp` — top-level sim loop; drives the world + ROS spin.
- `flatland_server/src/world.cpp` — loads a world YAML; owns the Box2D `b2World`, layers, and models.
- `flatland_server/src/model.cpp`, `model_body.cpp`, `body.cpp`, `joint.cpp` — model/body/joint construction from YAML into Box2D entities.
- `flatland_server/src/layer.cpp` — occupancy-image → Box2D collision geometry (OpenCV `findContours`, chain loops, `simplify_map`).
- `flatland_server/src/plugin_manager.cpp` — discovers/instantiates plugins and dispatches lifecycle + contact callbacks.
- `flatland_server/src/yaml_reader.cpp` + `yaml_preprocessor.cpp` — YAML parsing helpers; the preprocessor evaluates embedded **Lua** expressions in world files.
- `flatland_server/src/timekeeper.cpp` — simulation timestep / sim-time clock.
- `flatland_server/include/flatland_server/flatland_plugin.h` — base plugin class + lifecycle hooks (see Conventions).
- `flatland_server/include/flatland_server/model_plugin.h` / `world_plugin.h` — the two plugin flavours plugins subclass.
- `flatland_server/include/flatland_server/exceptions.h` — `YAMLException` and friends thrown on bad config.
- `flatland_server/include/flatland_server/debug_visualization.h` — publishes Box2D bodies as RViz markers.
- `flatland_plugins/flatland_plugins.xml` — `pluginlib` manifest; every plugin class MUST be listed here.
- `flatland_server/src/flatland_benchmark.cpp` + `launch/benchmark.launch` + `test/benchmark_world/` — perf benchmark.

## Conventions
- **Per-file license header.** Every `.h`/`.cpp` starts with the Avidbots ASCII-art
  BSD-3-clause header (see top of `flatland_server/src/world.cpp`). Copy an existing
  file's header and update `@name`/`@brief`/`@author`/`@copyright`. Never omit it.
- **clang-format `--style=file`.** Format every C/C++ file before committing; CI fails
  otherwise. Files under `thirdparty/` are excluded (CI greps `-v thirdparty/`).
- **Plugin registration is two-sided.** A new plugin needs both
  `PLUGINLIB_EXPORT_CLASS(flatland_plugins::Foo, flatland_server::ModelPlugin)` at the
  bottom of its `.cpp` AND a matching `<class>` entry in `flatland_plugins/flatland_plugins.xml`.
  Missing either means pluginlib silently can't find it at runtime.
- **Plugin lifecycle.** Plugins subclass `ModelPlugin`/`WorldPlugin` and override
  `OnInitialize(const YAML::Node&)` (required), and optionally `BeforePhysicsStep` /
  `AfterPhysicsStep` (take `const Timekeeper&`), and Box2D contact hooks
  `BeginContact` / `EndContact` / `PreSolve` / `PostSolve`.
- **YAML parsing goes through `YamlReader`.** Don't hand-roll `YAML::Node` traversal in
  engine/plugin code; use `flatland_server/include/flatland_server/yaml_reader.h` so
  errors surface as `YAMLException` with file/key context.
- **Box2D ownership.** The `b2World` (owned by `World`) owns all `b2Body`/`b2Joint`/
  `b2Fixture`; create/destroy them through the world, never `delete` them yourself.
- **Coordinate frames.** Physics is in metres/radians; map layers carry a resolution +
  origin. Convert at the boundary; don't mix pixel and world units.

## Never Do
- **Never edit `flatland_server/thirdparty/`** (Box2D, ThreadPool, Tweeny) — vendored upstream.
- **Never push C++ that isn't clang-format `--style=file` clean** or that fails `clang-tidy` (`scripts/ci_prebuild.sh` gates CI).
- **Never add a plugin class without a matching entry in `flatland_plugins/flatland_plugins.xml`** — it won't load.
- **Never drop the per-file Avidbots BSD license header** when creating or moving a file.
- **Never treat world YAML as trusted** — it can embed Lua (`yaml_preprocessor`) that executes during load. See `.claude/rules/common/security.md`.
- **Never block the physics step** with slow I/O inside `BeforePhysicsStep`/`AfterPhysicsStep`; it stalls the whole sim.
