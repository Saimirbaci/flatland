Build and run the Flatland test suite, then report results.

Run, from the catkin workspace root:

```bash
catkin build
catkin run_tests flatland_server --no-deps
catkin_test_results build/flatland_server
```

If I name a specific package (e.g. `flatland_plugins`), substitute it for
`flatland_server` in the `run_tests` and `catkin_test_results` commands. To run the
whole workspace, drop `--no-deps` and run `catkin run_tests` then
`catkin_test_results build`.

Summarize: which gtest/rostest targets ran, how many passed/failed, and the failing
assertions with `file:line`. If a target failed to build, hand off to the build-error
analysis (see `.claude/agents/staff-build-error-resolver.md`). Do not commit.
