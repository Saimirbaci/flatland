^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flatland_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* New ``DynamicMap`` world plugin for mid-episode dynamic map changes — the
  static environment itself evolves within a run (a shelf moves, a corridor
  closes) to test mapping/localization robustness to non-static worlds. It
  parses an ordered ``events`` list keyed to a ``target_layer`` and, on each
  event's sim-time trigger, read-modify-writes that layer's occupancy bitmap in
  world-metric coordinates (``fill`` = obstacle appears / corridor closes,
  ``clear`` = corridor opens, ``translate`` = a rectangular obstacle moves), then
  atomically rebuilds the layer's Box2D chain-loop geometry and re-publishes the
  latched occupancy overlay. Geometry is swapped only in ``BeforePhysicsStep``
  (between steps, never in a contact callback) and only on the fire edge, so
  there is no per-step cost once events have fired. The applied timeline is
  sealed out-of-band to a JSON manifest (run seed plus each op with its resolved
  sim onset time and world/pixel region) for ground truth. Backed by a new
  ``flatland_server::Layer::RebuildCollisionFromBitmap`` runtime-rebuild API and
  ``flatland_server::World::GetLayer`` lookup. See ``doc/dynamic_map.md``.
* New ``TrajectoryRecorder`` model plugin for the diagnostic overlays. It
  accumulates a body's actual pose each step into a length-capped
  ``nav_msgs/Path`` (``trajectory/actual``) read straight from the Box2D
  transform, and optionally re-stamps an externally supplied planned path into
  the world frame and republishes it (``trajectory/planned``) for
  planned-vs-actual comparison in rviz. Sampling is rate-limited via the shared
  ``UpdateTimer`` and the actual path is bounded to ``max_length`` poses so long
  runs stay memory-safe.
* Actuator / motor dynamics for the drive plugins. ``diff_drive`` and
  ``tricycle_drive`` gain an opt-in, per-axis ``ActuatorDynamics`` layer
  (``linear_actuator`` / ``angular_actuator`` for diff drive, ``drive_actuator``
  / ``steer_actuator`` for tricycle) adding command latency (a sim-time FIFO),
  a motor deadband, and a torque/force limit that bounds the achievable
  acceleration (``a_max = F/m`` / ``alpha_max = tau/I``) and, in ``friction``
  drive mode, caps the per-wheel motor force before the grip cap
  (``min(grip, motor)``). It composes with the existing ``*_dynamics`` ramps and
  the wheel-ground friction model — no ramp logic is duplicated. Body mass and
  inertia are re-read each step so the variable-mass payload model is respected.
  The feature is disabled by default (empty/absent subnode => byte-for-byte
  unchanged behaviour) and is the prerequisite scaffolding for actuator fault
  injection. See ``doc/actuator_dynamics.md``.
* Sensor/drive noise is now seed-driven for reproducible runs. ``laser``,
  ``imu``, ``diff_drive`` and ``tricycle_drive`` draw their noise generators from
  the seeded ``flatland_server::RngManager`` (keyed by model/plugin name) instead
  of ``std::random_device``; ``world_random_wall`` shuffles via a seeded engine
  instead of ``srand(time(0))``. With the same run seed, noise sequences are
  identical across runs. Use ``seed:=-1`` (the default) for nondeterministic
  noise as before.

1.4.1 (2023-11-24)
------------------
* CMake version required bump to fix ros build farm warning

1.4.0 (2023-11-22)
------------------
* Merge pull request `#103 <https://github.com/avidbots/flatland/pull/103>`_ adding 
* Contributors: Marc Gallant, Marc Bosch-Jorge

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
