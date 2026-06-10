# Copyright 2026 Avidbots Corp.
# Difficulty curriculum controller + scenario archive for the AI scenario
# generator. Shapes difficulty toward the AUT's competence frontier instead of
# always maximizing failure, and preserves diverse discovered failures.
"""Curriculum controller.

Rather than always maximizing failure (which drifts toward *unsolvable*
scenarios), the controller targets a **success band** (default 50-70%): it nudges
a `target_difficulty` signal up when the algorithm-under-test is coping and down
when it is drowning, so the proposer is biased toward the frontier of the AUT's
competence.

It also keeps a **MAP-Elites-style archive** keyed by behavior descriptors
(difficulty bin × dominant-stressor family), storing the highest-stress *solvable*
genome per cell. The archive (a) enforces novelty/diversity, (b) supports replay
of past failures to prevent catastrophic forgetting, and (c) applies a
**solvability regularizer**: invalid or degenerate-impossible scenarios are kept
out of the elite frontier so the loop never rewards a broken simulator.

State is JSON-persisted for checkpoint/resume of long runs.
"""

import json
import os
import random
from typing import Any, Dict, List, Optional, Tuple

from .genome import Genome


# A scenario counts as "solved" by the AUT (i.e. easy for it) when its stress
# score is at/below this, or it explicitly reached the goal.
DEFAULT_SOLVE_THRESHOLD = 0.5
# A scenario is treated as (likely) unsolvable / degenerate above this stress
# AND when the goal was never reached — excluded from the elite frontier.
DEFAULT_UNSOLVABLE_STRESS = 0.97


def dominant_family(g: Genome) -> str:
    """Behavior descriptor: which stressor family dominates this genome."""
    v = g.values
    map_on = any(v[k] >= 0.5 for k in (
        "mut_wall_jitter_enable", "mut_aisle_enable",
        "mut_clutter_enable", "mut_density_enable"))
    fault_on = any(v["fault%d_enable" % s] >= 0.5
                   for s in range(g.ps.get("n_fault_slots", 3)))
    if map_on and fault_on:
        return "combined"
    if fault_on:
        return "fault"
    if map_on:
        return "map"
    return "nominal"


def is_solved(result: Dict[str, Any],
              solve_threshold: float = DEFAULT_SOLVE_THRESHOLD) -> bool:
    """Did the AUT cope with this scenario?"""
    if result.get("invalid"):
        return False
    if result.get("metrics", {}).get("goal_reached"):
        return True
    return float(result.get("composite_score", 1.0)) <= solve_threshold


def is_solvable(result: Dict[str, Any],
                unsolvable_stress: float = DEFAULT_UNSOLVABLE_STRESS) -> bool:
    """Solvability regularizer: keep degenerate-impossible runs off the frontier."""
    if result.get("invalid"):
        return False
    reached = result.get("metrics", {}).get("goal_reached", False)
    return reached or float(result.get("composite_score", 1.0)) < unsolvable_stress


class CurriculumController:
    def __init__(self, target_success: Tuple[float, float] = (0.5, 0.7),
                 n_difficulty_bins: int = 5, window: int = 20,
                 step: float = 0.05, init_difficulty: float = 0.25,
                 solve_threshold: float = DEFAULT_SOLVE_THRESHOLD,
                 seed: int = 0):
        self.lo, self.hi = target_success
        self.mid = 0.5 * (self.lo + self.hi)
        self.n_bins = n_difficulty_bins
        self.window = window
        self.step = step
        self.solve_threshold = solve_threshold
        self.target_difficulty = init_difficulty
        self._recent_solved: List[bool] = []
        # archive[cell_key] = {"genome": dict, "score": float, "family": str,
        #                       "difficulty": float}
        self.archive: Dict[str, Dict[str, Any]] = {}
        self.rng = random.Random(seed)

    # ----------------------------------------------------------------- frontier
    def target(self) -> float:
        """The difficulty the proposer should aim for this generation."""
        return self.target_difficulty

    def recent_success_rate(self) -> Optional[float]:
        if not self._recent_solved:
            return None
        return sum(self._recent_solved) / len(self._recent_solved)

    def _bin(self, difficulty: float) -> int:
        b = int(difficulty * self.n_bins)
        return max(0, min(self.n_bins - 1, b))

    def _cell_key(self, g: Genome) -> str:
        return "%s/d%d" % (dominant_family(g), self._bin(g.difficulty()))

    # ------------------------------------------------------------------ observe
    def observe(self, g: Genome, result: Dict[str, Any]) -> Dict[str, Any]:
        """Feed one evaluated (genome, result). Returns a short status dict."""
        solved = is_solved(result, self.solve_threshold)
        self._recent_solved.append(solved)
        if len(self._recent_solved) > self.window:
            self._recent_solved.pop(0)

        # Adjust target difficulty toward the success band.
        rate = self.recent_success_rate()
        if rate is not None and len(self._recent_solved) >= max(4, self.window // 4):
            if rate > self.hi:        # AUT coping too well -> harder
                self.target_difficulty = min(1.0, self.target_difficulty + self.step)
            elif rate < self.lo:      # AUT drowning -> easier
                self.target_difficulty = max(0.0, self.target_difficulty - self.step)

        # Archive elite (solvable, novel-or-better) genomes.
        archived = False
        if is_solvable(result):
            key = self._cell_key(g)
            score = float(result.get("composite_score", 0.0))
            cur = self.archive.get(key)
            if cur is None or score > cur["score"]:
                self.archive[key] = {"genome": g.to_dict(), "score": score,
                                     "family": dominant_family(g),
                                     "difficulty": round(g.difficulty(), 4)}
                archived = True
        return {"solved": solved, "success_rate": rate,
                "target_difficulty": round(self.target_difficulty, 3),
                "archived": archived, "archive_size": len(self.archive)}

    # ------------------------------------------------------------------- replay
    def sample_replay(self, n: int, param_space: Dict[str, Any]) -> List[Genome]:
        """Sample up to ``n`` archived genomes to replay (anti-forgetting)."""
        if not self.archive:
            return []
        cells = list(self.archive.values())
        self.rng.shuffle(cells)
        return [Genome.from_dict(c["genome"], param_space) for c in cells[:n]]

    def frontier(self) -> List[Dict[str, Any]]:
        """Archived elites sorted hardest-first (for reporting/seeding)."""
        return sorted(self.archive.values(), key=lambda c: c["difficulty"],
                      reverse=True)

    # -------------------------------------------------------------- persistence
    def save(self, path: str) -> None:
        state = {
            "target_difficulty": self.target_difficulty,
            "recent_solved": self._recent_solved,
            "archive": self.archive,
            "config": {"target_success": [self.lo, self.hi],
                       "n_bins": self.n_bins, "window": self.window,
                       "step": self.step, "solve_threshold": self.solve_threshold},
        }
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            json.dump(state, f, indent=2, sort_keys=True)

    def load(self, path: str) -> None:
        with open(path) as f:
            state = json.load(f)
        self.target_difficulty = state.get("target_difficulty", self.target_difficulty)
        self._recent_solved = state.get("recent_solved", [])
        self.archive = state.get("archive", {})
