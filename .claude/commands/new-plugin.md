Scaffold a new Flatland plugin end-to-end (matches the recent imu/gps plugin additions).

Ask me for: the plugin class name (CamelCase, e.g. `Sonar`) and whether it is a
**model** plugin (attaches to a model — the common case) or a **world** plugin.

Then do all of the following, citing real files as you go:

1. Create `flatland_plugins/include/flatland_plugins/<snake_name>.h` with the Avidbots
   BSD license header (copy from `flatland_plugins/include/flatland_plugins/imu.h`),
   declaring `class <Name> : public flatland_server::ModelPlugin` (or `WorldPlugin`).
2. Create `flatland_plugins/src/<snake_name>.cpp` with the header, an `OnInitialize`
   that parses config via `flatland_server::YamlReader`, and any needed
   `BeforePhysicsStep`/`AfterPhysicsStep`/contact hooks. Use `imu.cpp`/`gps.cpp` as templates.
3. End the `.cpp` with
   `PLUGINLIB_EXPORT_CLASS(flatland_plugins::<Name>, flatland_server::ModelPlugin)`.
4. Add a `<class type="flatland_plugins::<Name>" base_class_type="flatland_server::ModelPlugin">`
   entry to `flatland_plugins/flatland_plugins.xml`.
5. Add the new source to the plugins library target in `flatland_plugins/CMakeLists.txt`
   and any new ROS dependency to `flatland_plugins/package.xml`.
6. Add a test following the `.claude/skills/flatland-testing` conventions.

Reference `.claude/skills/flatland-plugin-authoring/SKILL.md`. Remind me that steps 3
AND 4 are both required or pluginlib won't find the class. Do not commit.
