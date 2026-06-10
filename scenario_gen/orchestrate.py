# Copyright 2026 Avidbots Corp.
# Outer-loop orchestrator: ties the curriculum, proposer and headless harness
# into a generation loop with a reproducible run ledger + checkpoint/resume.
"""The generation loop.

Each generation:

1. ask the :class:`CurriculumController` for a target difficulty,
2. get candidate genome(s) from the configured :class:`Proposer`
   (plus a few archive **replays** to fight forgetting),
3. render + run each genome via the headless harness (``run.evaluate``),
4. feed ``(genome, stress score)`` back to the proposer + curriculum,
5. append every scenario to a **run ledger** (genome hash, seed, score,
   ground-truth manifest path, world path) for exact standalone replay,
6. checkpoint curriculum state so a long run can resume after a crash.

The ledger is append-only JSONL; on restart the orchestrator replays it into the
proposer + curriculum and continues from the next generation.
"""

import argparse
import json
import os
from typing import Any, Dict, List, Optional

from .genome import Genome, load_param_space
from .curriculum import CurriculumController
from .proposers import make_proposer
from . import run as run_mod
from . import failure_boundary as fb_mod


class RunLedger:
    """Append-only JSONL ledger of every evaluated scenario."""

    def __init__(self, ledger_dir: str):
        self.dir = ledger_dir
        os.makedirs(ledger_dir, exist_ok=True)
        self.path = os.path.join(ledger_dir, "ledger.jsonl")

    def append(self, record: Dict[str, Any]) -> None:
        with open(self.path, "a") as f:
            f.write(json.dumps(record, sort_keys=True) + "\n")

    def load(self) -> List[Dict[str, Any]]:
        if not os.path.exists(self.path):
            return []
        out = []
        with open(self.path) as f:
            for line in f:
                line = line.strip()
                if line:
                    out.append(json.loads(line))
        return out


def _weights_from_arg(arg: Optional[str]) -> Dict[str, float]:
    """Parse ``loc=1,coll=2,goal=1,track=0.5`` style weight strings."""
    weights = {"localization_error": 1.0, "collisions": 1.0,
               "goal_failure": 1.0, "tracking_error": 1.0}
    alias = {"loc": "localization_error", "coll": "collisions",
             "goal": "goal_failure", "track": "tracking_error"}
    if not arg:
        return weights
    for tok in arg.split(","):
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        k = alias.get(k.strip(), k.strip())
        if k in weights:
            weights[k] = float(v)
    return weights


def run_curriculum(template: str = "conestogo", proposer_strategy: str = "rl",
                   generations: int = 10, per_generation: int = 5,
                   replay_per_generation: int = 1,
                   target_success=(0.5, 0.7), ledger_dir: str = "./scenario_runs",
                   base_seed: int = 0, aut_cmd: Optional[str] = None,
                   weights: Optional[Dict[str, float]] = None,
                   scenario_duration: float = 60.0, real_time_factor: float = 0.0,
                   stub: bool = False, llm_model: Optional[str] = None,
                   fail_threshold: float = 0.6,
                   resume: bool = True) -> Dict[str, Any]:
    """Run the adversarial/curriculum loop. Returns a summary dict."""
    ps = load_param_space()
    weights = weights or _weights_from_arg(None)
    ledger = RunLedger(ledger_dir)
    ckpt = os.path.join(ledger_dir, "curriculum_state.json")
    dr_ckpt = os.path.join(ledger_dir, "dr_distribution.json")

    prop_kwargs = {}
    if llm_model and proposer_strategy in ("llm", "hybrid"):
        prop_kwargs["model"] = llm_model
    if proposer_strategy == "dr":
        prop_kwargs["fail_threshold"] = fail_threshold
    proposer = make_proposer(proposer_strategy, ps, seed=base_seed, **prop_kwargs)
    curriculum = CurriculumController(target_success=target_success, seed=base_seed)

    history: List[Dict[str, Any]] = []
    scenario_counter = 0
    start_gen = 0

    # --- resume: replay prior ledger into proposer + curriculum ---
    if resume:
        prior = ledger.load()
        if prior:
            if os.path.exists(ckpt):
                curriculum.load(ckpt)
            for rec in prior:
                g = Genome.from_dict(rec["genome"], ps)
                proposer.update(g, rec["score"])
                history.append({"genome": rec["genome"], "genome_hash": rec["genome_hash"],
                                "score": rec["score"], "terminal": rec.get("terminal", ""),
                                "invalid": bool(rec.get("invalid", False))})
            scenario_counter = len(prior)
            start_gen = max((rec.get("generation", 0) for rec in prior), default=-1) + 1
            print("Resumed: %d prior scenarios, continuing at generation %d"
                  % (scenario_counter, start_gen))

    gen_scores: List[float] = []
    for gen in range(start_gen, start_gen + generations):
        target = curriculum.target()
        proposals = proposer.propose(history, k=per_generation,
                                     target_difficulty=target,
                                     base_seed=base_seed + scenario_counter)
        # Anti-forgetting: replay a few archived elites.
        replays = curriculum.sample_replay(replay_per_generation, ps)
        batch = [(p.genome, p.rationale) for p in proposals] + \
                [(g, "archive replay") for g in replays]

        gen_dir = os.path.join(ledger_dir, "gen%03d" % gen)
        os.makedirs(gen_dir, exist_ok=True)
        batch_scores = []
        for j, (g, rationale) in enumerate(batch):
            g.seed = base_seed + scenario_counter  # deterministic per-scenario seed
            scn_dir = os.path.join(gen_dir, "scn%02d" % j)
            os.makedirs(scn_dir, exist_ok=True)
            result = run_mod.evaluate(
                g, template=template, out_dir=scn_dir, aut_cmd=aut_cmd,
                weights=weights, scenario_duration=scenario_duration,
                real_time_factor=real_time_factor, stub=stub)
            score = float(result.get("composite_score", 1.0))

            proposer.update(g, score)
            status = curriculum.observe(g, result)

            with open(os.path.join(scn_dir, "genome.json"), "w") as f:
                json.dump(g.to_dict(), f, indent=2, sort_keys=True)
            record = {
                "generation": gen, "scenario": scenario_counter,
                "genome_hash": g.hash(), "seed": g.seed, "score": score,
                "difficulty": round(g.difficulty(), 4),
                "rationale": rationale, "terminal": result.get("terminal", ""),
                "invalid": bool(result.get("invalid", False)),
                "target_difficulty": status["target_difficulty"],
                "success_rate": status["success_rate"],
                "genome": g.to_dict(),
                "result_path": os.path.join(scn_dir, "scenario_result.json"),
                "world_path": os.path.join(scn_dir, "world.yaml"),
            }
            ledger.append(record)
            history.append({"genome": g.to_dict(), "genome_hash": g.hash(),
                            "score": score, "terminal": result.get("terminal", ""),
                            "invalid": bool(result.get("invalid", False))})
            batch_scores.append(score)
            scenario_counter += 1

        curriculum.save(ckpt)
        # Persist the DR distribution checkpoint alongside curriculum state so a
        # run is inspectable mid-flight (resume itself replays the ledger).
        if hasattr(proposer, "dist"):
            with open(dr_ckpt, "w") as f:
                json.dump(proposer.dist.to_dict(), f, indent=2, sort_keys=True)
        mean = sum(batch_scores) / len(batch_scores) if batch_scores else 0.0
        gen_scores.append(mean)
        print("gen %03d: mean_stress=%.3f target_diff=%.2f success_rate=%s archive=%d"
              % (gen, mean, curriculum.target(),
                 ("%.2f" % curriculum.recent_success_rate()
                  if curriculum.recent_success_rate() is not None else "n/a"),
                 len(curriculum.archive)))

    # --- estimate + report the AUT's failure boundary from the full history ---
    boundary = fb_mod.estimate_boundary(history, fail_threshold=fail_threshold,
                                        param_space=ps)
    boundary_paths = fb_mod.write_report(boundary, ledger_dir)

    return {
        "generations": generations, "scenarios": scenario_counter,
        "gen_mean_scores": gen_scores,
        "final_target_difficulty": curriculum.target(),
        "archive_size": len(curriculum.archive),
        "frontier": curriculum.frontier()[:10],
        "ledger": ledger.path,
        "fail_threshold": fail_threshold,
        "failure_boundary_report": boundary_paths["json"],
        "failure_boundary_markdown": boundary_paths["markdown"],
        "n_failures": boundary["n_failures"],
        "fail_rate": boundary["fail_rate"],
        "top_failure_dimensions": boundary["ranked_dimensions"][:5],
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description="AI adversarial/curriculum scenario loop")
    ap.add_argument("--template", default="conestogo")
    ap.add_argument("--proposer", default="rl",
                    choices=["random", "rl", "llm", "hybrid", "dr"])
    ap.add_argument("--aut", default=None, help="algorithm-under-test launch cmd")
    ap.add_argument("--generations", type=int, default=10)
    ap.add_argument("--per-generation", type=int, default=5)
    ap.add_argument("--replay-per-gen", type=int, default=1)
    ap.add_argument("--target-success", type=float, nargs=2, default=[0.5, 0.7])
    ap.add_argument("--ledger", default="./scenario_runs")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--weights", default=None, help="loc=1,coll=1,goal=1,track=1")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--rtf", type=float, default=0.0)
    ap.add_argument("--llm-model", default=None)
    ap.add_argument("--fail-threshold", type=float, default=0.6,
                    help="composite-score fail criterion for the boundary report "
                         "(and the DR proposer's failure-region target)")
    ap.add_argument("--stub", action="store_true", help="offline stub evaluator")
    ap.add_argument("--no-resume", action="store_true")
    args = ap.parse_args(argv)

    summary = run_curriculum(
        template=args.template, proposer_strategy=args.proposer,
        generations=args.generations, per_generation=args.per_generation,
        replay_per_generation=args.replay_per_gen,
        target_success=tuple(args.target_success), ledger_dir=args.ledger,
        base_seed=args.seed, aut_cmd=args.aut,
        weights=_weights_from_arg(args.weights), scenario_duration=args.duration,
        real_time_factor=args.rtf, stub=args.stub, llm_model=args.llm_model,
        fail_threshold=args.fail_threshold, resume=not args.no_resume)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
