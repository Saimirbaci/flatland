# C++ Patterns — Flatland

The only language rule that applies: this is a **C++11**, ROS 1 (roscpp) catkin codebase. No
C++14/17 features. Match the idioms already in the tree.

## ROS / roscpp
- Node handles, publishers, subscribers, and `tf2` are the I/O surface. Plugins receive their
  `ros::NodeHandle` through the engine — get it from the model, don't create a global one.
- Use `flatland_msgs` for custom messages/services; standard `sensor_msgs`/`nav_msgs`/`std_srvs`
  for the rest (e.g. laser → `sensor_msgs/LaserScan`, gps → `sensor_msgs/NavSatFix`).
- Logging: `ROS_INFO/ROS_WARN/ROS_ERROR(_STREAM)` — not `std::cout`.

## Plugin lifecycle (`flatland_server/include/flatland_server/flatland_plugin.h`)
All plugins derive from `FlatlandPlugin`; model plugins from `ModelPlugin`
(`model_plugin.h`, holds `Model *model_`, `GetModel()`), world plugins from `WorldPlugin`.
Override only what you need:
- `OnInitialize(const YAML::Node &config)` — **pure virtual**, parse your config here.
- `BeforePhysicsStep(const Timekeeper &)` / `AfterPhysicsStep(const Timekeeper &)` — per-step work.
- `BeginContact`/`EndContact`/`PreSolve`/`PostSolve(b2Contact*…)` — Box2D collision callbacks.
Register at the bottom of the `.cpp`:
`PLUGINLIB_EXPORT_CLASS(flatland_plugins::Foo, flatland_server::ModelPlugin)` — and add the
matching `<class>` entry to `flatland_plugins/flatland_plugins.xml`.

## Box2D ownership
- The `b2World` (owned by `world.cpp`) owns all bodies, fixtures, and joints. Create them through
  the world / model body, never `new b2Body`; release via `b2World::DestroyBody`.
- A plugin holds *non-owning* pointers into its model's bodies. Never delete them; never let the
  plugin outlive the model.
- Collision categories/masks are managed via `collision_filter_registry.h` — register through it
  rather than hand-setting `b2Filter` bits.

## YAML parsing & errors
- Parse config through `yaml_reader.h` (`YamlReader`, `Get<T>`, subnode helpers) — it carries file
  + field context and throws `YAMLException` (`flatland_server/include/flatland_server/exceptions.h`).
- Don't index raw `YAML::Node` directly; let `YamlReader` produce typed values and clear errors.

## Visualization & timing
- Emit debug geometry through `debug_visualization.h` (publishes RViz markers); don't roll your own
  marker publishing.
- Use the passed `Timekeeper` for sim time/timestep — never wall-clock (`ros::WallTime`) inside the
  step loop, and never `sleep` in a plugin callback.
