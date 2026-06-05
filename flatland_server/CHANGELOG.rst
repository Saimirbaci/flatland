^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flatland_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Procedural map mutation engine: an optional per-layer ``mutation:`` block
  deterministically perturbs the static occupancy map at load time — bounded
  wall shift/rotate (``wall_jitter``), corridor widening/narrowing
  (``aisle_width``), and clutter add/remove with density scaling (``clutter``,
  ``obstacle_density``) — so mapping/localization is tested against novel maps
  instead of one fixed map. Seeded from the run ``seed`` via ``RngManager``
  (per-layer ``seed_key``) for byte-reproducible runs, with an optional sealed
  out-of-band JSON manifest (``manifest_path``) recording exactly what changed.
  Absent / ``enabled: false`` leaves existing worlds byte-for-byte identical.
  See ``doc/map_mutation.md``.
* Diagnostic overlay publishers for the modernized flatland_viz. Each
  image-based layer now also publishes a latched ``nav_msgs/OccupancyGrid``
  (``/flatland_server/layers/<name>/occupancy`` plus the stable
  ``/flatland_server/occupancy``) so the static map renders as an rviz Map
  display, and a configured ``surface_friction`` field publishes a latched
  colour-mapped ``visualization_msgs/MarkerArray`` of its low-traction regions
  (``/flatland_server/debug/friction_regions``). Both are built once at
  world-load time via ``World::PublishDiagnostics`` (gated on ``use_rviz``) and
  cost nothing per physics step. See ``doc/diagnostic_overlays.md``.
* Deterministic / reproducible runs: added a central seeded RNG authority
  (``flatland_server/random.h`` ``RngManager``) and a ``seed`` parameter. A run
  is now byte-reproducible given the same seed, world YAML and step size.
* ``seed`` resolution precedence: node param ``seed`` > world YAML
  ``properties.seed`` > nondeterministic (``std::random_device``). The effective
  seed is logged at startup so any run can be replayed. ``seed`` defaults to
  ``-1`` (nondeterministic), preserving prior fresh-random behavior.
* The Lua preprocessor (``$eval`` ``math.random``) is now seeded from the run
  seed for reproducible YAML preprocessing.
* Added an explicit ``real_time_factor`` parameter (simulated seconds per real
  second; ``<=0`` runs unthrottled). The sim loop is now a fixed-step loop paced
  solely by ``real_time_factor``; visualization cadence is driven off simulated
  time so output no longer depends on host speed. The benchmark always runs
  unthrottled.
* Added determinism tests (``random_test`` gtest, ``determinism_test`` rostest).

1.4.1 (2023-11-24)
------------------
* CMake version required bump to fix ros build farm warning

1.4.0 (2023-11-22)
------------------
* Version Bump

1.3.3 (2023-02-06)
------------------
* added missing std_srvs dep
* Merge pull request `#95 <https://github.com/avidbots/flatland/issues/95>`_ from avidbots/per-package-licenses
  Per package licenses
* Contributors: Joseph Duchesne

* added missing std_srvs dep
* Merge pull request `#95 <https://github.com/avidbots/flatland/issues/95>`_ from avidbots/per-package-licenses
  Per package licenses
* Contributors: Joseph Duchesne

* added missing std_srvs dep
* Merge pull request `#95 <https://github.com/avidbots/flatland/issues/95>`_ from avidbots/per-package-licenses
  Per package licenses
* Contributors: Joseph Duchesne
