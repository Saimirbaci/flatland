# CLAUDE.md — Flatland

## What This Is
Flatland is a performance-centric **2D physics simulator for ground robots**, built as a ROS 1
(Noetic; legacy Kinetic) catkin metapackage in C++11. It uses Box2D for rigid-body physics, loads
worlds/models/layers from YAML (with embedded Lua preprocessing), and exposes a pluginlib-based
plugin system so sensors and drives (laser, IMU, GPS, diff-drive, bumper, …) can be attached to
models at load time. It is the lighter-weight 2D counterpart to Gazebo, optimized for fast
headless simulation and RViz-based visualization. Current version: **1.5.0**.

## Quick Start
Flatland is a set of catkin packages — it builds inside a catkin workspace, not standalone.
```bash
# 1. Clone into a catkin workspace's src/ folder, then from the workspace root:
rosdep install --from-paths src --ignore-src    # install missing ROS deps
catkin build                                     # build all 5 packages (catkin_tools)

# 2. Run the simulator (loads test/conestogo_office_test/world.yaml by default):
source devel/setup.bash
roslaunch flatland_server server.launch          # add use_rviz:=true for visualization

# 3. Run the full test suite (gtest + rostest):
catkin run_tests flatland_server flatland_plugins --no-deps
catkin_test_results                              # aggregate pass/fail across packages

# 4. Benchmark (matches the simplify_map perf work in recent commits):
roslaunch flatland_server benchmark.launch       # runs flatland_benchmark on test/benchmark_world
```
Pre-push CI gate: `bash scripts/ci_prebuild.sh` (clang-format `--style=file` + clang-tidy). CI
runs via `ros-industrial/industrial_ci` — see `.github/workflows/industrial_ci_action.yml`.

## Architecture
Five catkin packages under the repo root:

| Package | Role |
|---------|------|
| `flatland/` | Metapackage — only ties the others together via `package.xml` deps. No code. |
| `flatland_msgs/` | Custom ROS messages & services (e.g. spawn/delete model, move model). |
| `flatland_server/` | **Core engine.** World/model/layer loading, Box2D stepping, the plugin manager, services, and the `flatland_server` + `flatland_benchmark` nodes. Vendors Box2D under `thirdparty/`. |
| `flatland_plugins/` | The shipped model/world plugins (laser, imu, gps, diff_drive, tricycle_drive, bumper, bool_sensor, tween, model_tf_publisher, world_random_wall, …). Registered via `flatland_plugins.xml`. |
| `flatland_viz/` | Standalone Qt/OGRE visualizer and RViz tools (spawn-model tool, model dialogs, pause-sim tool). |

Data flow: a **world YAML** references **layer** images (the static map) and **model** YAMLs; each
model lists **plugins**. `world.cpp` owns the Box2D `b2World` and drives `BeforePhysicsStep` /
`AfterPhysicsStep` on every plugin each timestep via `plugin_manager.cpp`.

## Key Files
- `flatland_server/src/simulation_manager.cpp` — top-level sim loop: build world, step physics on the `Timekeeper` clock, publish viz.
- `flatland_server/src/world.cpp` — owns the `b2World`; loads layers + models; iterates plugins per step.
- `flatland_server/src/plugin_manager.cpp` — loads plugins via `pluginlib::ClassLoader` and dispatches lifecycle/contact callbacks.
- `flatland_server/src/yaml_reader.cpp` + `include/.../yaml_reader.h` — typed YAML accessors that throw `YAMLException` on bad config. Use these, never raw `YAML::Node`.
- `flatland_server/src/yaml_preprocessor.cpp` — runs embedded **Lua** over world YAML before parsing (`$eval` expressions, env-var substitution).
- `flatland_server/src/layer.cpp` — map loading: OpenCV `findContours` + `simplify_map` → Box2D chain-loop edges (recent perf path).
- `flatland_server/include/flatland_server/flatland_plugin.h` — base plugin class; defines `OnInitialize`, `BeforePhysicsStep`, `AfterPhysicsStep`, `BeginContact`/`EndContact`/`PreSolve`/`PostSolve`.
- `flatland_server/include/flatland_server/model_plugin.h` — `ModelPlugin` base (holds `Model *model_`, `GetModel()`); most plugins extend this. `world_plugin.h` is the world-scoped counterpart.
- `flatland_server/include/flatland_server/exceptions.h` — `YAMLException`, `PluginException`, etc.
- `flatland_server/src/timekeeper.cpp` — sim-time clock and fixed timestep.
- `flatland_server/src/debug_visualization.cpp` + `debug_visualization.h` — publishes Box2D bodies as RViz markers.
- `flatland_plugins/flatland_plugins.xml` — pluginlib manifest. **Every plugin class must be listed here.**
- `flatland_plugins/src/imu.cpp`, `laser.cpp`, `diff_drive.cpp` — representative model-plugin implementations to copy from.
- `flatland_server/src/flatland_benchmark.cpp` + `launch/benchmark.launch` — headless perf harness.

## Conventions
- **License header on every source file.** Each `.cpp`/`.h` opens with the Avidbots ASCII-art BSD header (see top of `flatland_server/src/body.cpp`), including `@name`, `@brief`, `@author`. New files must carry it.
- **clang-format `--style=file`** governs all formatting (the repo `.clang-format`). `scripts/ci_prebuild.sh` rejects any unformatted, non-`thirdparty/` file. clang-tidy also runs.
- **Plugin registration is two-step:** `PLUGINLIB_EXPORT_CLASS(flatland_plugins::Foo, flatland_server::ModelPlugin)` at the bottom of the `.cpp` **and** a `<class>` entry in `flatland_plugins/flatland_plugins.xml`. Missing either → plugin won't load at runtime.
- **Parse YAML only through `yaml_reader.h`** helpers (`Get<T>`, subnode accessors). They throw `YAMLException` with file/field context — do not hand-roll `YAML::Node` access.
- **Box2D ownership:** the `b2World` owns all bodies/fixtures/joints. Create via the world, never `new b2Body`/`delete`; destroy through `b2World::DestroyBody`. Plugins must not outlive their model.
- **Coordinate frames:** Box2D and ROS are both right-handed, meters + radians; flatland is 2D so only x, y, yaw are physical. Respect `tf` frame ids configured per model.
- **Plugin work goes in `BeforePhysicsStep`/`AfterPhysicsStep`/contact callbacks**, never in a busy loop — the engine drives them on the `Timekeeper` clock.
- **Tests are paired:** a `*_test.cpp` (gtest) plus a `*.test` rostest launch file; wire them with `add_rostest_gtest` / `catkin_add_gtest` in the package `CMakeLists.txt`.

## Never Do
- **Never edit `flatland_server/thirdparty/Box2D`** (or anything under `thirdparty/`). It's vendored upstream and excluded from clang-format/clang-tidy.
- **Never push without `bash scripts/ci_prebuild.sh` passing** — unformatted code fails CI immediately.
- **Never add a plugin class without registering it** in `flatland_plugins.xml` *and* `PLUGINLIB_EXPORT_CLASS`.
- **Never drop or alter the per-file Avidbots BSD license header.**
- **Never trust world YAML as safe input** — `yaml_preprocessor.cpp` evaluates embedded Lua, so a world file is arbitrary-code-capable. Only load worlds from trusted sources.
- **Never block the physics step** with sleeps or synchronous network calls inside plugin callbacks.
