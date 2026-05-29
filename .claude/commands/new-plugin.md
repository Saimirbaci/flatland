Scaffold a new Flatland plugin end-to-end. Plugin name/purpose: $ARGUMENTS

Follow the `flatland-plugin-authoring` skill. Do every step — a plugin that compiles but isn't
registered will silently fail to load.

1. Pick the closest existing plugin in `flatland_plugins/src/` to copy from (sensor → `imu.cpp`/
   `gps.cpp`/`laser.cpp`; drive → `diff_drive.cpp`/`tricycle_drive.cpp`; collision →
   `bumper.cpp`/`bool_sensor.cpp`; world plugin → `world_random_wall.cpp`). Read it first.
2. Create the header `flatland_plugins/include/flatland_plugins/<name>.h` deriving from
   `flatland_server::ModelPlugin` (or `WorldPlugin`), with the Avidbots license header.
3. Create `flatland_plugins/src/<name>.cpp`:
   - `OnInitialize(config)` parsing via a `YamlReader` (throw `YAMLException` on bad config),
   - per-step logic in `BeforePhysicsStep`/`AfterPhysicsStep` (or `BeginContact`/`EndContact`),
   - publish via the correct `sensor_msgs`/`std_msgs` type using the model's node handle,
   - end with `PLUGINLIB_EXPORT_CLASS(flatland_plugins::<Name>, flatland_server::ModelPlugin)`.
4. Add a `<class>` entry to `flatland_plugins/flatland_plugins.xml`.
5. Wire `src/<name>.cpp` and the test target into `flatland_plugins/CMakeLists.txt`.
6. Add `flatland_plugins/test/<name>_test.cpp` (+ `<name>_test.test` and a small world under
   `test/<name>_tests/` if it needs the node graph), modeled on `imu_test.*`.
7. Run `bash scripts/ci_prebuild.sh`, then
   `catkin run_tests flatland_plugins --no-deps && catkin_test_results`.

Report which files you created and the test result.
