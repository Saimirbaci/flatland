---
name: flatland-physics-box2d
description: Use when working with Flatland's Box2D physics — creating/destroying bodies, joints, fixtures, setting up collision filtering, the simulation timestep, or coordinate-frame conversions. Explains ownership and unit conventions so you don't introduce leaks or pixel/metre bugs.
---

# Box2D physics in Flatland

Flatland wraps a **vendored** Box2D (`flatland_server/thirdparty/Box2D`, read-only).

## Ownership model (important)
- `flatland_server::World` (`src/world.cpp`) owns the single `b2World`.
- All `b2Body`, `b2Joint`, `b2Fixture` are created and destroyed through the `b2World`
  (`CreateBody`/`DestroyBody`, etc.) — **never `new`/`delete` them yourself**.
- Flatland's `Body` (`body.cpp`, `model_body.cpp`) and `Joint` (`joint.cpp`) wrap the Box2D
  objects and are built from YAML by `model.cpp`.

## Timestep
- `flatland_server::Timekeeper` (`timekeeper.cpp`, `include/.../timekeeper.h`) owns the sim
  clock and step size. `BeforePhysicsStep`/`AfterPhysicsStep` receive `const Timekeeper&`.
- Use sim time from `Timekeeper`, never wall-clock time, so runs stay deterministic.
- Keep per-step work cheap — the step is the hot loop in a "performance centric" simulator.

## Collision filtering
- Use named categories via `flatland_server::CollisionFilterRegistry`
  (`collision_filter_registry.cpp`, `.h`) rather than hand-rolled Box2D category/mask bits.
- Contact events reach plugins through `BeginContact`/`EndContact`/`PreSolve`/`PostSolve`.

## Coordinates & units
- Physics is in **metres and radians**. Map layers carry a resolution + origin (pixels).
- Convert at the image/world boundary (in `layer.cpp`/`geometry.cpp`); never mix pixel and
  world units in body coordinates.

## Visual debugging
- `debug_visualization.h`/`.cpp` publishes Box2D bodies as RViz markers — use it instead of
  ad-hoc `visualization_msgs`.

For map-image → geometry specifics and perf, see the `flatland-map-layers` skill.
