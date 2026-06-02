^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flatland_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Layered diagnostic overlays. A new dockable "Diagnostic Layers" panel
  (``DiagnosticLayerPanel``) adds per-overlay show/hide checkboxes wired to
  rviz Displays created in ``FlatlandViz``: an occupancy/costmap Map, a
  LaserScan, planned vs actual trajectory Paths (distinctly coloured), a
  friction-region MarkerArray, and an aggregate toggle over the auto-discovered
  dynamic-obstacle debug markers. Toggling a box only flips a display's enabled
  state; no data plumbing is duplicated.

1.4.1 (2023-11-24)
------------------
* CMake version required bump to fix ros build farm warning

1.4.0 (2023-11-22)
------------------
* Version Bump

1.3.3 (2023-02-06)
------------------
* Merge pull request `#95 <https://github.com/avidbots/flatland/issues/95>`_ from avidbots/per-package-licenses
  Per package licenses
* Contributors: Joseph Duchesne

* Merge pull request `#95 <https://github.com/avidbots/flatland/issues/95>`_ from avidbots/per-package-licenses
  Per package licenses
* Contributors: Joseph Duchesne

* Merge pull request `#95 <https://github.com/avidbots/flatland/issues/95>`_ from avidbots/per-package-licenses
  Per package licenses
* Contributors: Joseph Duchesne
