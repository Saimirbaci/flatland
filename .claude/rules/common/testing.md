# Testing — Flatland

Tests are gtest + rostest, registered per package in `CMakeLists.txt` and living in
each package's `test/` directory (e.g. `flatland_server/test/`).

## Running tests (verbatim)
```bash
# build first, then run a package's tests and read results
catkin build
catkin run_tests flatland_server --no-deps
catkin_test_results build/flatland_server
```
CI runs the full suite via `ros-industrial/industrial_ci` (`ROS_DISTRO: noetic`,
see `.github/workflows/industrial_ci_action.yml`).

## Two test flavours
- **Pure unit tests** → `catkin_add_gtest(<name> test/<name>.cpp)`. No ROS master needed.
  Example: `geometry_test`, `collision_filter_registry_test`, `null_test`.
- **ROS-integration tests** → `add_rostest_gtest(<name> test/<name>.test test/<name>.cpp)`.
  Needs a `.test` launch file paired with the `.cpp`. Example: `load_world_test`,
  `model_test`, `plugin_manager_test`, `service_manager_test`, `dummy_model_plugin_test`.

## Conventions
- A rostest gets a matching `test/<name>.test` (roslaunch XML) plus `test/<name>.cpp`
  (the gtest body). Add both, and register in `CMakeLists.txt` inside the
  `if(CATKIN_ENABLE_TESTING)` block.
- World/model fixtures live under `flatland_server/test/` as subdirectories of YAML
  (e.g. `load_world_tests/`, `conestogo_office_test/`, `benchmark_world/`,
  `yaml_preprocessor/`). Reuse these rather than inventing new worlds where possible.
- Plugins use the `dummy_model_plugin` / `dummy_world_plugin` pattern (see
  `flatland_server/src/dummy_*_plugin.cpp`) as a minimal test plugin scaffold.
