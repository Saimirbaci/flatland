---
name: flatland-plugin-authoring
description: Use when creating a new Flatland model or world plugin (a sensor or actuator that attaches to a model, like laser/diff_drive/imu/gps), or modifying an existing one. Covers the full scaffold — header, impl, pluginlib registration, xml manifest, CMake target, and test — so you don't rediscover the two-sided registration requirement.
---

# Authoring a Flatland plugin

Plugins are C++ classes loaded at runtime by `pluginlib`, living in `flatland_plugins/`.
Reference implementations: `flatland_plugins/src/imu.cpp` and `gps.cpp` (recent, clean).

## Steps
1. **Header** — `flatland_plugins/include/flatland_plugins/foo.h`. Start with the Avidbots
   BSD license header (copy from `imu.h`). Declare `class Foo : public flatland_server::ModelPlugin`
   (use `WorldPlugin` for world-scope plugins). Include `<flatland_server/model_plugin.h>`,
   `<flatland_server/timekeeper.h>`, `<Box2D/Box2D.h>` as needed.

2. **Implementation** — `flatland_plugins/src/foo.cpp`. Override:
   - `void OnInitialize(const YAML::Node &config)` — **required**; parse params here.
   - `void BeforePhysicsStep(const Timekeeper&)` / `AfterPhysicsStep(const Timekeeper&)` — per-step.
   - `BeginContact`/`EndContact`/`PreSolve`/`PostSolve(b2Contact*…)` — collision hooks if needed.
   Parse config via `flatland_server::YamlReader`; throw `YAMLException` (`flatland_server/exceptions.h`).

3. **Register (BOTH are mandatory):**
   - Bottom of `foo.cpp`:
     `PLUGINLIB_EXPORT_CLASS(flatland_plugins::Foo, flatland_server::ModelPlugin)`
   - Add to `flatland_plugins/flatland_plugins.xml`:
     `<class type="flatland_plugins::Foo" base_class_type="flatland_server::ModelPlugin"><description>…</description></class>`
   Missing either → `pluginlib` "class not found" at runtime.

4. **Build** — add the source to the plugins library target in
   `flatland_plugins/CMakeLists.txt`; add any new ROS dep to `flatland_plugins/package.xml`.

5. **Test** — add a gtest/rostest (see `flatland-testing` skill). Model on
   `flatland_server/src/dummy_model_plugin.cpp` for a minimal case.

6. **Docs** — recent plugin PRs (imu, gps) added documentation; follow suit.

## Gotchas
- Don't block the physics step with slow I/O. Drive timing from `Timekeeper`, not wall clock.
- Use `collision_filter_registry.h` for collision categories; `debug_visualization.h` for RViz markers.
- Plugins are referenced by `type:` in a model YAML's `plugins:` list — the type must match the xml entry.
