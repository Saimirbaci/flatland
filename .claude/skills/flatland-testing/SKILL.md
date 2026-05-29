---
name: flatland-testing
description: Write and run Flatland tests — gtest unit tests and rostest (.cpp + .test) integration tests, with the right CMake wiring and test worlds. Invoke when adding coverage, debugging a failing test, or doing TDD.
---

# Testing in Flatland

Two flavors, both run via catkin. CI executes them through `ros-industrial/industrial_ci`.

## Run
```bash
catkin run_tests flatland_server flatland_plugins --no-deps
catkin_test_results          # aggregate; non-zero exit on any failure
catkin run_tests flatland_plugins --no-deps    # single package
```

## gtest (pure C++ unit test)
- One `.cpp` under the package `test/` dir, registered with `catkin_add_gtest(...)` in
  `CMakeLists.txt`.
- Examples: `flatland_server/test/geometry_test.cpp`,
  `flatland_server/test/collision_filter_registry_test.cpp`.

## rostest (needs the ROS node graph)
- A **pair**: `<unit>_test.cpp` + `<unit>_test.test` (a roslaunch file), registered with
  `add_rostest_gtest(...)`.
- Examples: `flatland_plugins/test/imu_test.cpp` + `imu_test.test`;
  `flatland_server/test/load_world_test.cpp` + `load_world_test.test`;
  `plugin_manager_test.*`, `service_manager_test.*`, `model_test.*`.
- The `.test` file launches `flatland_server` with a small world, then runs the gtest binary as a
  test node.

## Test worlds / fixtures
- Engine worlds: `flatland_server/test/` — `conestogo_office_test/`, `load_world_tests/`,
  `plugin_manager_tests/`, `yaml_preprocessor/`, `benchmark_world/`.
- Plugin worlds: `flatland_plugins/test/<plugin>_tests/` (e.g. `laser_tests/`, `bumper_tests/`,
  `tween_tests/`, `tricycle_drive_tests/`).
- Reuse an existing world before writing a new one.

## Conventions
- Name tests `<unit>_test.cpp` / `<unit>_test.test` matching the unit under test.
- Assert on observable behavior: published topics, model pose, contact callbacks, thrown
  `YAMLException`.
- Deterministic timing: drive on the sim `Timekeeper` / fixed step size, never wall-clock.
- License header + clang-format `--style=file` on new test files too.
