---
name: flatland-testing
description: Use when adding or running tests in Flatland. Explains the gtest vs rostest split, the .cpp/.test pairing, where fixture worlds live, and the exact catkin commands — so you pick the right harness and register it correctly.
---

# Testing Flatland

gtest + rostest, registered in each package's `CMakeLists.txt`, with tests under
`flatland_server/test/` (and the other packages' `test/` dirs).

## Run
```bash
catkin build
catkin run_tests flatland_server --no-deps
catkin_test_results build/flatland_server
```
CI runs everything via `ros-industrial/industrial_ci` (`ROS_DISTRO: noetic`).

## Pick the harness
- **gtest** (no ROS master) for pure logic — geometry, parsing, registries:
  `catkin_add_gtest(<name> test/<name>.cpp)`.
  Examples: `geometry_test.cpp`, `collision_filter_registry_test.cpp`, `null.cpp`.
- **rostest** (needs a node / world load / services):
  `add_rostest_gtest(<name> test/<name>.test test/<name>.cpp)` — write BOTH the `.cpp`
  gtest body and the `.test` roslaunch file.
  Examples: `load_world_test`, `model_test`, `plugin_manager_test`,
  `service_manager_test`, `debug_visualization_test`, `dummy_model_plugin_test`,
  `dummy_world_plugin_test`.

## Fixtures
Reuse the YAML worlds already under `flatland_server/test/`:
`load_world_tests/`, `conestogo_office_test/`, `benchmark_world/`, `yaml_preprocessor/`.
For plugin tests, copy the `dummy_model_plugin` / `dummy_world_plugin` pattern
(`flatland_server/src/dummy_*_plugin.cpp`).

## Checklist for a new test
1. Add `test/<name>.cpp` (+ `test/<name>.test` if rostest).
2. Register it in `CMakeLists.txt` inside `if(CATKIN_ENABLE_TESTING)`.
3. Give the file the Avidbots BSD license header; keep it clang-format clean.
4. Make sure it fails before the fix / passes after.
