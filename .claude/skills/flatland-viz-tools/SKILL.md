---
name: flatland-viz-tools
description: Work on the flatland_viz package — the standalone Qt/OGRE visualizer and its RViz-style interactive tools (spawn model, model dialogs, pause sim). Invoke when changing visualization, the viz window, or interactive spawning UI.
---

# Flatland visualization (flatland_viz)

`flatland_viz` is a standalone viewer built on RViz's rendering stack (Qt + OGRE). It subscribes to
the markers Flatland publishes (`debug_visualization` in `flatland_server`) and adds interactive
tools for spawning/posing models. It is **separate** from running the sim headless — the engine
works without it (`server.launch show_viz:=true` for markers, `use_rviz:=true` to use RViz).

## Files (`flatland_viz/src/`)
- `flatland_viz_node.cpp` — the executable entrypoint.
- `flatland_viz.cpp` — the main viz display / render panel wiring.
- `flatland_window.cpp` — the top-level Qt window.
- `spawn_model_tool.cpp` — RViz tool to place a model in the world interactively.
- `load_model_dialog.cpp` / `model_dialog.cpp` — Qt dialogs for choosing/configuring a model to spawn.
- `pause_sim_tool.cpp` — RViz tool that toggles simulation pause.

## How it talks to the engine
- Spawning/deleting models goes through the **services/messages in `flatland_msgs`** handled by
  `flatland_server/src/service_manager.cpp`. The viz tools are clients of those services — don't
  duplicate spawn logic in the viz, call the service.
- Geometry comes from the engine's markers; the viz does not own physics state.

## When you change a tool
- RViz tools subclass `rviz::Tool` and register via `PLUGINLIB_EXPORT_CLASS`; keep the plugin
  manifest/CMake wiring in `flatland_viz` consistent (same registration discipline as engine plugins).
- Qt code: keep signal/slot connections in the dialog/window classes; don't block the GUI thread on
  service calls.
- License header + clang-format `--style=file` apply here too.

Note: per recent git history, active development is on the engine and plugins, not the viz — touch
this only for explicit visualization work.
