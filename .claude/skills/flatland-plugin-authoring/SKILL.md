---
name: flatland-plugin-authoring
description: Scaffold a new Flatland model or world plugin end-to-end (header, impl, pluginlib export, XML manifest, CMake, tests). Invoke when adding a sensor, drive, or world behavior — the most common change in this repo (recent: imu, gps, diff-drive tf, variable_payload).
---

# Authoring a Flatland plugin

A plugin is only "done" when all six pieces exist. Copy the closest existing plugin rather than
starting blank: `imu.cpp` (sensor publishing), `gps.cpp`, `laser.cpp` (sensor + raycast),
`diff_drive.cpp` (drive + tf), `bumper.cpp`/`bool_sensor.cpp` (collision), `world_random_wall.cpp`
(world plugin), `variable_payload.cpp` (drives a body's mass/CoG via `Body::SetMassData`, optional
command + state topics — see the `flatland-physics-box2d` skill for the mass hook).

## Steps
1. **Header** — `flatland_plugins/include/flatland_plugins/<name>.h`
   ```cpp
   namespace flatland_plugins {
   class MyThing : public flatland_server::ModelPlugin {  // or WorldPlugin
    public:
     void OnInitialize(const YAML::Node &config) override;
     void BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) override;
   };
   }  // namespace flatland_plugins
   ```
   Include the Avidbots license header (copy from any existing header).

2. **Impl** — `flatland_plugins/src/<name>.cpp`
   - `OnInitialize`: read config with a `YamlReader` (from
     `flatland_server/include/flatland_server/yaml_reader.h`); throw `YAMLException` on bad input.
   - Get the node handle / model via `GetModel()`; set up publishers (`sensor_msgs/...`) / subscribers.
   - Do per-step work in `BeforePhysicsStep` / `AfterPhysicsStep`; collision sensors use
     `BeginContact`/`EndContact`.
   - Last line: `PLUGINLIB_EXPORT_CLASS(flatland_plugins::MyThing, flatland_server::ModelPlugin)`.

3. **Manifest** — add to `flatland_plugins/flatland_plugins.xml`:
   ```xml
   <class type="flatland_plugins::MyThing" base_class_type="flatland_server::ModelPlugin">
     <description>...</description>
   </class>
   ```

4. **CMake** — add `src/<name>.cpp` to the plugin library target and add the test target in
   `flatland_plugins/CMakeLists.txt`.

5. **Tests** — `flatland_plugins/test/<name>_test.cpp` + `<name>_test.test`; mirror `imu_test.*`.
   Add a minimal world under `flatland_plugins/test/<name>_tests/` if needed.

6. **Verify** — `bash scripts/ci_prebuild.sh` then
   `catkin run_tests flatland_plugins --no-deps && catkin_test_results`.

## Gotchas
- Forgetting the `flatland_plugins.xml` entry → pluginlib can't find the class at runtime (compiles fine).
- Sensor/drive plugins consult the fault-injection registry just before publishing/commanding
  (`imu`/`laser`/`gps`/`bumper`/`diff_drive`/`tricycle_drive`). If your plugin emits sensor data or
  drives a body, see the `flatland-fault-injection` skill for the hook pattern and the
  no-active-effect-is-a-no-op invariant.
- Never `new`/`delete` Box2D bodies; hold non-owning pointers into the model.
- Use the model's `ros::NodeHandle`, not a fresh global one.
