# C++ Patterns — Flatland

C++11 throughout. ROS 1 roscpp. Box2D physics. `pluginlib` for loadable behaviour.
This file captures the repo-specific idioms; for formatting/headers see
`common/coding-style.md`.

## ROS 1 (roscpp)
- The node entrypoint is `flatland_server/src/flatland_server_node.cpp`; the sim loop
  is `simulation_manager.cpp`. Use `ros::NodeHandle` for params/pubs/subs.
- ROS interfaces (services to spawn/move/delete models, debug topics) are wired in
  `flatland_server/src/service_manager.cpp` using messages from `flatland_msgs`.
- Use `tf`/`tf2` for frame broadcasts (see `flatland_plugins/src/model_tf_publisher.cpp`,
  `diff_drive.cpp`).

## Plugin pattern (`pluginlib`)
Plugins subclass `flatland_server::ModelPlugin` or `WorldPlugin` (both extend
`FlatlandPlugin`, see `flatland_server/include/flatland_server/flatland_plugin.h`).
Lifecycle hooks to override:
- `virtual void OnInitialize(const YAML::Node &config)` — **required**; parse config here.
- `BeforePhysicsStep(const Timekeeper&)` / `AfterPhysicsStep(const Timekeeper&)` — per-step work.
- `BeginContact` / `EndContact` / `PreSolve` / `PostSolve` — Box2D collision callbacks (take `b2Contact*`).

Register at the bottom of the `.cpp`:
```cpp
PLUGINLIB_EXPORT_CLASS(flatland_plugins::Foo, flatland_server::ModelPlugin)
```
and add a `<class type="flatland_plugins::Foo" base_class_type="flatland_server::ModelPlugin">`
entry to `flatland_plugins/flatland_plugins.xml`. Study `flatland_plugins/src/imu.cpp`
and `gps.cpp` as recent, clean examples.

## Box2D ownership & coordinates
- The `b2World` is owned by `flatland_server::World` (`src/world.cpp`). All
  `b2Body`/`b2Joint`/`b2Fixture` are created and destroyed through Box2D world calls —
  never `delete` them. Models/bodies/joints wrap them (`model.cpp`, `body.cpp`, `joint.cpp`).
- Collision filtering goes through `collision_filter_registry.h`/`.cpp` (named categories),
  not raw bitmasks.
- Physics units are metres/radians; the timestep comes from `Timekeeper`
  (`timekeeper.h`). Don't read wall-clock time in the step — use the timekeeper's sim time.

## Config parsing & errors
- Parse YAML via `flatland_server/include/flatland_server/yaml_reader.h` (typed getters
  with key/file context), not raw `YAML::Node` access.
- On bad config, throw the exceptions in
  `flatland_server/include/flatland_server/exceptions.h` (e.g. `YAMLException`) so the
  loader reports a useful message.

## Debug visualization
- To draw bodies/markers in RViz, use
  `flatland_server/include/flatland_server/debug_visualization.h` rather than publishing
  `visualization_msgs` ad hoc.
