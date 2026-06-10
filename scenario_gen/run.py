# Copyright 2026 Avidbots Corp.
# Single-scenario headless run harness: genome -> render -> launch sim + AUT ->
# collect scenario_result.json -> stress score. Also provides a deterministic
# offline stub evaluator so the proposer/curriculum loop is testable without ROS.
"""Evaluate one genome and return its stress score.

`evaluate()` renders the genome to a world, launches `scenario_run.launch`
(simulator + observer/scorer) plus the algorithm-under-test, waits for the
terminal condition, and reads the sealed `scenario_result.json`. A crashed/hung
run is reported as `composite_score=1.0, invalid=True` so the outer loop never
hangs or rewards a broken simulator.

`stub_score()` is a pure, deterministic function of the genome used when no ROS
is available (offline curriculum smoke tests, proposer convergence demos): it
rewards harder + *combined* scenarios so optimizers have a real signal to climb.
"""

import argparse
import hashlib
import json
import os
import shutil
import signal
import subprocess
import tempfile
import time
from typing import Any, Dict, List, Optional

from .genome import Genome, load_param_space
from . import render as render_mod


# --------------------------------------------------------------- stub evaluator
def stub_score(g: Genome) -> Dict[str, Any]:
    """Deterministic offline stress score in [0,1] (no ROS needed).

    Built so that (a) harder genomes score higher (difficulty term) and
    (b) genomes that *combine* a map mutation with a fault score extra
    ("interaction" term) — mirroring the real insight that compounded stress
    breaks navigation worse than either alone. A tiny hash-derived jitter adds
    realism without breaking reproducibility (same genome+seed => same score).
    """
    v = g.values
    diff = g.difficulty()
    map_active = any(v[k] >= 0.5 for k in (
        "mut_wall_jitter_enable", "mut_aisle_enable",
        "mut_clutter_enable", "mut_density_enable"))
    fault_active = any(v["fault%d_enable" % s] >= 0.5
                       for s in range(g.ps.get("n_fault_slots", 3)))
    interaction = 0.2 if (map_active and fault_active) else 0.0
    h = int(hashlib.sha1(g.hash().encode()).hexdigest()[:8], 16)
    jitter = ((h % 1000) / 1000.0 - 0.5) * 0.06  # +/- 0.03, deterministic
    composite = max(0.0, min(1.0, 0.75 * diff + interaction + jitter))
    return {
        "composite_score": composite,
        "genome_hash": g.hash(),
        "seed": g.seed,
        "invalid": False,
        "terminal": "stub",
        "metrics": {"difficulty": round(diff, 4),
                    "map_active": map_active, "fault_active": fault_active},
        "weights": {},
    }


# --------------------------------------------------------------- real evaluator
def _launch_cmd(world_path: str, meta: Dict[str, Any], result_path: str,
                seed: int, weights: Dict[str, float], scenario_duration: float,
                wall_timeout: float, real_time_factor: float) -> List[str]:
    a = lambda k, v: "%s:=%s" % (k, v)  # noqa: E731
    start = meta["start_pose"]
    goal = meta["goal"]
    return [
        "roslaunch", "flatland_server", "scenario_run.launch",
        a("world_path", world_path),
        a("result_path", result_path),
        a("genome_hash", meta["genome_hash"]),
        a("seed", seed),
        a("scenario_duration", scenario_duration),
        a("wall_timeout", wall_timeout),
        a("real_time_factor", real_time_factor),
        a("global_frame", meta["global_frame"]),
        a("robot_base_frame", meta["robot_base_frame"]),
        a("est_pose_topic", "%s/amcl_pose" % meta["robot_name"]),
        a("collisions_topic", "%s/collisions" % meta["robot_name"]),
        a("start_x", start[0]), a("start_y", start[1]),
        a("goal_x", goal[0]), a("goal_y", goal[1]),
        a("w_localization", weights.get("localization_error", 1.0)),
        a("w_collisions", weights.get("collisions", 1.0)),
        a("w_goal", weights.get("goal_failure", 1.0)),
        a("w_tracking", weights.get("tracking_error", 1.0)),
    ]


def _invalid_result(g: Genome, terminal: str) -> Dict[str, Any]:
    return {"composite_score": 1.0, "genome_hash": g.hash(), "seed": g.seed,
            "invalid": True, "terminal": terminal, "metrics": {}, "weights": {}}


def evaluate(g: Genome, template: str = "conestogo",
             out_dir: Optional[str] = None, aut_cmd: Optional[str] = None,
             weights: Optional[Dict[str, float]] = None,
             scenario_duration: float = 60.0, wall_timeout: float = 300.0,
             real_time_factor: float = 0.0, stub: bool = False,
             aut_warmup_s: float = 3.0) -> Dict[str, Any]:
    """Render + run one genome; return its result dict (with composite_score)."""
    weights = weights or {}
    if stub or shutil.which("roslaunch") is None:
        return stub_score(g)

    if out_dir is None:
        # Default scratch dir under the user's private tmp tree. Use a 0700
        # per-user base so a co-tenant on a shared host can't pre-create or
        # symlink the predictable <genome_hash> path to clobber/redirect our
        # world.yaml / scenario_result.json writes.
        base = os.path.join(tempfile.gettempdir(),
                            "scenario_gen-%d" % os.getuid())
        os.makedirs(base, mode=0o700, exist_ok=True)
        out_dir = os.path.join(base, g.hash())
    os.makedirs(out_dir, exist_ok=True)
    meta = render_mod.render(g, template, out_dir)
    world_path = meta["world_path"]
    result_path = os.path.join(out_dir, "scenario_result.json")
    if os.path.exists(result_path):
        os.remove(result_path)

    cmd = _launch_cmd(world_path, meta, result_path, g.seed, weights,
                      scenario_duration, wall_timeout, real_time_factor)

    aut_proc = None
    launch_proc = None
    try:
        launch_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        if aut_cmd:
            time.sleep(aut_warmup_s)  # let the sim publish /clock + tf first
            aut_proc = subprocess.Popen(aut_cmd, shell=True, preexec_fn=os.setsid)
        # roslaunch exits when the required observer node finishes.
        launch_proc.wait(timeout=wall_timeout + 30.0)
    except subprocess.TimeoutExpired:
        return _finish(g, result_path, "wall_timeout", launch_proc, aut_proc)
    except Exception as e:  # pragma: no cover - defensive
        print("scenario_gen.run: launch error: %s" % e)
        return _finish(g, result_path, "launch_error", launch_proc, aut_proc)
    return _finish(g, result_path, "ok", launch_proc, aut_proc)


def _kill(proc) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        for _ in range(20):
            if proc.poll() is not None:
                return
            time.sleep(0.25)
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        pass


def _finish(g: Genome, result_path: str, terminal: str, launch_proc,
            aut_proc) -> Dict[str, Any]:
    _kill(aut_proc)
    _kill(launch_proc)
    if not os.path.exists(result_path):
        return _invalid_result(g, terminal if terminal != "ok" else "no_result")
    try:
        with open(result_path) as f:
            res = json.load(f)
    except Exception:
        return _invalid_result(g, "bad_result")
    res.setdefault("composite_score", 1.0)
    res.setdefault("genome_hash", g.hash())
    res.setdefault("seed", g.seed)
    return res


# ------------------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description="Evaluate one scenario genome")
    ap.add_argument("--template", default="conestogo")
    ap.add_argument("--out", default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--genome", help="genome.json to evaluate")
    ap.add_argument("--random", action="store_true")
    ap.add_argument("--aut", default=None, help="algorithm-under-test launch cmd")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--rtf", type=float, default=0.0, help="real_time_factor")
    ap.add_argument("--stub", action="store_true", help="offline stub score")
    args = ap.parse_args(argv)

    ps = load_param_space()
    if args.genome:
        with open(args.genome) as f:
            g = Genome.from_dict(json.load(f), ps)
        g.seed = args.seed
    elif args.random:
        import random
        g = Genome.random(random.Random(args.seed), ps, seed=args.seed)
    else:
        g = Genome(param_space=ps, seed=args.seed)

    res = evaluate(g, template=args.template, out_dir=args.out, aut_cmd=args.aut,
                   scenario_duration=args.duration, real_time_factor=args.rtf,
                   stub=args.stub)
    print(json.dumps(res, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
