^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flatland_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
