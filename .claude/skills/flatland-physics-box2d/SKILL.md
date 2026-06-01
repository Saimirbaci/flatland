---
name: flatland-physics-box2d
description: Work with Flatland's Box2D physics — creating bodies/joints/fixtures, collision filtering, and the fixed-timestep Timekeeper clock. Invoke for physics behavior, contact handling, coordinate frames, or stepping questions.
---

# Box2D physics in Flatland

Flatland wraps a single Box2D `b2World` (owned by `flatland_server/src/world.cpp`) and steps it on a
fixed-timestep clock. Everything physical is planar: meters + radians, x / y / yaw only.

## Ownership model (do not break)
- The `b2World` owns all `b2Body` / `b2Fixture` / `b2Joint`. Create them through the world / model
  body wrappers (`body.cpp`, `model_body.cpp`, `joint.cpp`), destroy via `b2World::DestroyBody`.
- **Never** `new b2Body` / `delete` directly. Plugins hold *non-owning* raw pointers into the
  model's bodies and must not outlive the model.
- Flatland's wrappers: `Body`/`ModelBody` (`body.h`, `model_body.h`), `Joint` (`joint.h`),
  `Entity` (`entity.h`).

## Timestep & sim time
- `flatland_server/src/timekeeper.cpp` (`timekeeper.h`) holds the fixed `step_size` and sim time.
  `world.cpp` calls `b2World::Step` with it and dispatches `BeforePhysicsStep`/`AfterPhysicsStep`.
- Inside the step loop use the passed `Timekeeper` — never `ros::WallTime`, never `sleep`.
- `step_size` / `update_rate` are launch args (see `launch/server.launch`).

## Collision filtering
- Use `collision_filter_registry.cpp` / `.h` to allocate categories and layer bits — don't set raw
  `b2Filter` category/mask bits by hand. Layers and model bodies register through it.
- Contact callbacks on plugins: `BeginContact`, `EndContact`, `PreSolve`, `PostSolve`
  (`flatland_plugin.h`). `bumper.cpp` / `bool_sensor.cpp` are working examples.

## Mass & center of gravity
- Box2D derives a body's mass once from `density × area` per fixture and never revisits it. To make
  mass/CoG/inertia vary at runtime, use the generic hooks on `Body` (`body.h`/`body.cpp`):
  `b2MassData GetMassData()` and `SetMassData(double mass, const b2Vec2 &local_center, double inertia)`.
- `SetMassData` wraps `b2Body::SetMassData` and **never** calls `ResetMassData` (which would discard
  the override and recompute from fixture densities). `mass` must be positive + finite (throws
  `std::invalid_argument`); `inertia` is about the body local origin, pass `<= 0` to keep the current
  value. Adding a non-zero-density fixture also implicitly resets mass, so an override only persists
  while the fixture set is unchanged.
- When the CoG moves, shift inertia with the parallel-axis theorem (`I_origin = I_com + m·|c|²`).
  `flatland_plugins/src/variable_payload.cpp` is the worked example (draining payload mass + CoG).

## Geometry / transforms
- `geometry.cpp` (`geometry.h`) has the pose/transform helpers. Keep tf frame ids consistent with
  the model config.

## Visualizing
- `debug_visualization.cpp` / `.h` publishes the Box2D bodies as RViz markers — use it to inspect
  fixture shapes rather than logging coordinates.

## Performance
- Fewer fixtures/edges = faster. The map path uses `simplify_map` to reduce edge count — see the
  `flatland-map-layers` skill and benchmark with `roslaunch flatland_server benchmark.launch`.
