# Testing — Flatland

## How CI runs tests
Tests build and run inside the catkin workspace via `ros-industrial/industrial_ci`
(`.github/workflows/industrial_ci_action.yml`, `ROS_DISTRO=noetic`). Locally, from the workspace root:
```bash
catkin run_tests flatland_server flatland_plugins --no-deps
catkin_test_results                 # non-zero exit if any test failed
```
Run a single package's tests with `catkin run_tests flatland_plugins --no-deps`.

## Two test flavors, always paired
1. **gtest** (pure C++ unit tests) — registered with `catkin_add_gtest` in the package
   `CMakeLists.txt`. Example: `flatland_server/test/geometry_test.cpp`.
2. **rostest** (needs a running ROS node graph) — a `*_test.cpp` **plus** a matching `*.test`
   launch file, registered with `add_rostest_gtest`. Example pair:
   `flatland_plugins/test/imu_test.cpp` + `flatland_plugins/test/imu_test.test`.

When adding a plugin, add **both** the `_test.cpp` and the `.test` file and wire them in
`CMakeLists.txt` — every shipped plugin already follows this (`laser_test`, `diff_drive_test`,
`gps_test`, `tween_test`, …).

## Test fixtures / worlds
- Sample worlds live under `flatland_server/test/` — e.g. `conestogo_office_test/`,
  `benchmark_world/`, `load_world_tests/`, `plugin_manager_tests/`, `yaml_preprocessor/`.
- Plugin tests load small purpose-built worlds from `flatland_plugins/test/<plugin>_tests/`.
- Reuse an existing test world before authoring a new one.

## Conventions
- Name tests `<unit>_test.cpp` / `<unit>_test.test` to match the file under test.
- Assert on observable behavior (published topics, model pose, contact callbacks), not internals.
