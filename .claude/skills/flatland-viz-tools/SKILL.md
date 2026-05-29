---
name: flatland-viz-tools
description: Use when working on flatland_viz — the RViz/Qt visualization and interaction tools (spawning models, model dialogs, pause-sim). Maps the viz package files so you don't hunt for the right tool/dialog.
---

# flatland_viz — RViz/Qt tools

`flatland_viz` is the interactive front end: an RViz-based window plus custom tools that
talk to the running `flatland_server` over its ROS services (`service_manager.cpp`) and
debug topics (`debug_visualization.cpp`).

## Files (`flatland_viz/`)
- `src/flatland_viz_node.cpp` — the viz process entrypoint.
- `src/flatland_viz.cpp` / `include/.../flatland_viz.h` — the main RViz render panel/widget.
- `src/flatland_window.cpp` / `flatland_window.h` — the top-level Qt window.
- `src/spawn_model_tool.cpp` / `spawn_model_tool.h` — RViz tool to place a model in the world
  (calls the spawn service with a chosen model YAML + pose).
- `src/load_model_dialog.cpp` / `load_model_dialog.h` — dialog to pick a model file to spawn.
- `src/model_dialog.cpp` / `model_dialog.h` — model properties dialog.
- `src/pause_sim_tool.cpp` / `pause_sim_tool.h` — RViz tool to pause/resume the simulation.

## Conventions
- Tools are RViz plugins — they're exported via the package's plugin description xml and
  built in `flatland_viz/CMakeLists.txt`. A new tool needs the class, the export entry, and
  the CMake target (same two-sided pattern as server plugins).
- Keep the Avidbots BSD license header on every file; Qt/OGRE/RViz headers are heavy — match
  the include style of the existing tool files.
- Viz talks to the server through `flatland_msgs` services — don't reach into engine
  internals; go through the ROS interface.

> Note: per `git log`, viz has seen little recent change (active work is in
> `flatland_server`/`flatland_plugins`). Use this skill mainly to orient quickly.
