# Diagnostic Overlays — Topic Contract

The flatland_viz "Diagnostic Layers" panel renders five toggleable overlays plus
the existing dynamic-obstacle debug markers. Each overlay is backed by a ROS
topic in the shared world frame `map`. This table is the contract the publishers
(server / plugins) and the viewer agree on.

| Overlay | Topic | Type | Source | Status |
|---------|-------|------|--------|--------|
| Occupancy / costmap | `/flatland_server/layers/<layer>/occupancy` (latched) | `nav_msgs/OccupancyGrid` | `Layer::BuildOccupancyGrid` (from the loaded bitmap), published once by `World::PublishDiagnostics` | **new** |
| Laser scan | `<model_ns>/scan` (configurable per-plugin) | `sensor_msgs/LaserScan` | `flatland_plugins/laser` | existing |
| Planned trajectory | `<model_ns>/trajectory/planned` | `nav_msgs/Path` | `flatland_plugins/trajectory_recorder` (re-stamps an externally supplied planned path into `map`) | **new** |
| Actual trajectory | `<model_ns>/trajectory/actual` | `nav_msgs/Path` | `flatland_plugins/trajectory_recorder` (accumulates the body pose each step, length-capped) | **new** |
| Friction regions | `/flatland_server/debug/friction_regions` (latched) | `visualization_msgs/MarkerArray` | `SurfaceFrictionField::ToMarkerArray`, published once by `World::PublishDiagnostics` | **new** |
| Dynamic obstacles | `/flatland_server/debug/*` (auto-discovered) | `visualization_msgs/MarkerArray` | `flatland_server/debug_visualization` + `DebugTopicList` | existing |

## Conventions

- **Frame:** every overlay is emitted in the `map` frame, matching
  `debug_visualization.cpp` and the top-down ortho view in flatland_viz. Mixing
  frames would misalign overlays.
- **Latching:** the static overlays (occupancy, friction regions) are published
  **once** on latched topics at world-load time, never inside the physics step,
  so they cost nothing per step and late subscribers (rviz, the panel) still get
  them.
- **Trajectories** publish on the simulation clock (`Timekeeper`) at a
  configurable rate; the actual path is capped at `max_length` poses to bound
  memory over long runs.

## Gaps this feature fills

The occupancy grid, friction-region markers, and the two trajectory paths did
not exist as published topics before this feature: layers only became Box2D
collision geometry, the surface-friction field was sample-only, and there was no
recorder for planned-vs-actual comparison. The laser scan and dynamic-obstacle
markers were reused as-is.
