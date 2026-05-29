Run the Flatland performance benchmark and report timing (matches the simplify_map
benchmark workflow).

The benchmark is `flatland_server/src/flatland_benchmark.cpp`, launched via
`flatland_server/launch/benchmark.launch`, exercising the world under
`flatland_server/test/benchmark_world/`.

Steps:
1. `catkin build flatland_server` then `source devel/setup.bash`.
2. Run `roslaunch flatland_server benchmark.launch` and capture the timing output.
3. If I'm comparing a change: run the benchmark on the baseline first, then on the
   change, and report the delta (this is how the simplify_map / findContours speedups
   were validated).
4. If asked, sweep the `simplify_map` rosparam (0 / 1 / 2 — see
   `flatland_server/src/layer.cpp:254`) and report timing per level, noting that higher
   levels reduce collision-geometry detail.

Report wall-clock / step timings clearly. Do not commit.
