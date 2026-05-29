Run the Flatland performance benchmark and report the result. Args (optional): $ARGUMENTS

This wraps `flatland_benchmark` (`flatland_server/src/flatland_benchmark.cpp`) via
`flatland_server/launch/benchmark.launch`, which runs against
`flatland_server/test/benchmark_world/world.yaml`.

```bash
catkin build flatland_server
source devel/setup.bash
roslaunch flatland_server benchmark.launch          # default simplify_map:=2
```

Useful overrides (pass any in $ARGUMENTS):
- `simplify_map:=<0|1|2>` — 0=none, 1=moderate, 2=significant map simplification (the perf lever).
- `update_rate:=<hz>` `step_size:=<s>` `benchmark_duration:=<s>` — sim timing.
- `use_perf:=true` — wrap in `perf record` (writes `perf.out.flatland_benchmark.data`).

When measuring a change:
1. Run the benchmark on the baseline (e.g. `simplify_map:=0`) and record the reported rate.
2. Run with your change / higher simplify_map and record again.
3. Report the delta (the simplify_map work targets ~10–20% gains). See the `flatland-map-layers`
   skill for what `simplify_map` actually does.
