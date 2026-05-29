^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flatland_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
