Build and run the Flatland test suite, then report results.

From the catkin workspace root (the dir whose `src/` contains this repo):

```bash
catkin build                                              # ensure up to date
source devel/setup.bash
catkin run_tests flatland_server flatland_plugins --no-deps
catkin_test_results                                       # aggregate pass/fail
```

If `$ARGUMENTS` names a single package or test, scope it:
`catkin run_tests $ARGUMENTS --no-deps && catkin_test_results`.

Notes:
- gtest targets are registered via `catkin_add_gtest`; rostest pairs (`*_test.cpp` + `*.test`) via
  `add_rostest_gtest` — both run under `catkin run_tests`.
- If a build error blocks the tests, hand off to the build-error-resolver flow rather than guessing.
- On failure, surface the failing test name and the assertion output; do not just report the exit code.
