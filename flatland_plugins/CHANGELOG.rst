^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flatland_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
