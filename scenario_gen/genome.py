# Copyright 2026 Avidbots Corp.
# AI-driven scenario generator: the scenario "genome" — the bounded search
# space an AI policy controls, plus encode/decode/validate/difficulty helpers.
#
# Pure, dependency-free (stdlib only) so it is shared by the renderer, every
# proposer, the curriculum controller and the test suite without pulling ML
# deps into code that just needs the contract.
"""Scenario genome: the parameter vector an AI policy proposes.

A :class:`Genome` is a thin, validated wrapper over a flat ``{knob: value}``
dict whose keys/bounds are defined by ``param_space.json``. It can be encoded to
and decoded from a fixed-length numeric vector (for RL / black-box optimizers),
emitted as / parsed from plain JSON (for the LLM proposer), randomly sampled,
clamped back into bounds, hashed for the reproducibility ledger, and scored for
difficulty by the curriculum controller.

The genome is deliberately *structure-free*: faults live in a fixed number of
"slots" and map mutations behind 0/1 gates, so the vector length never changes.
``render.py`` is the only component that turns a genome into Flatland YAML.
"""

import hashlib
import json
import os
import random
from typing import Any, Dict, List, Optional

PARAM_SPACE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "param_space.json")

_GATE_ON = 0.5  # a "gate" knob is interpreted as on when value >= this


def load_param_space(path: Optional[str] = None) -> Dict[str, Any]:
    """Load and return the param_space.json contract as a dict."""
    with open(path or PARAM_SPACE_PATH, "r") as f:
        return json.load(f)


class GenomeError(ValueError):
    """Raised when a genome violates the param_space contract."""


class Genome:
    """A validated point in the scenario parameter space.

    Args:
        values: ``{knob_name: value}``. Missing knobs fall back to their
            param_space default; unknown knobs raise :class:`GenomeError`.
        param_space: parsed param_space (loaded from disk when omitted).
        seed: world RNG seed threaded to ``properties.seed`` at render time.
    """

    def __init__(self, values: Optional[Dict[str, Any]] = None,
                 param_space: Optional[Dict[str, Any]] = None,
                 seed: int = 0):
        self.ps = param_space or load_param_space()
        self.dims = self.ps["dimensions"]
        self._by_name = {d["name"]: d for d in self.dims}
        self.seed = int(seed)
        self.values: Dict[str, Any] = {d["name"]: d["default"] for d in self.dims}
        if values:
            for k, v in values.items():
                if k not in self._by_name:
                    raise GenomeError("unknown genome knob: %r" % k)
                self.values[k] = v
        self.clamp()

    # ------------------------------------------------------------------ bounds
    def _n_categories(self, dim: Dict[str, Any]) -> int:
        ref = dim.get("categories_ref")
        if ref == "fault_types":
            return len(self.ps["fault_types"])
        # template-backed categoricals (start/goal) have an unknown count here;
        # the renderer wraps the index modulo the template's candidate list, so
        # any non-negative int is valid. Cap at a generous bound for sampling.
        return dim.get("n_categories", 8)

    def clamp(self) -> "Genome":
        """Force every knob back inside its declared bounds. Mutates in place."""
        for d in self.dims:
            name, t = d["name"], d["type"]
            v = self.values[name]
            if t in ("float", "gate"):
                v = float(v)
                v = max(d["low"], min(d["high"], v))
            elif t == "int":
                v = int(round(float(v)))
                v = max(int(d["low"]), min(int(d["high"]), v))
            elif t == "categorical":
                n = self._n_categories(d)
                v = int(round(float(v))) % max(1, n)
            self.values[name] = v
        return self

    # ------------------------------------------------------------------ vector
    def to_vector(self) -> List[float]:
        """Encode to a fixed-length float vector (knob order == dims order)."""
        return [float(self.values[d["name"]]) for d in self.dims]

    @classmethod
    def from_vector(cls, vec: List[float], param_space: Optional[Dict] = None,
                    seed: int = 0) -> "Genome":
        ps = param_space or load_param_space()
        dims = ps["dimensions"]
        if len(vec) != len(dims):
            raise GenomeError("vector length %d != %d dims" % (len(vec), len(dims)))
        values = {d["name"]: vec[i] for i, d in enumerate(dims)}
        return cls(values, ps, seed=seed)

    @staticmethod
    def vector_bounds(param_space: Optional[Dict] = None):
        """Return ``(lows, highs)`` lists for the flat vector (optimizer init)."""
        ps = param_space or load_param_space()
        lows, highs = [], []
        for d in ps["dimensions"]:
            if d["type"] == "categorical":
                ref = d.get("categories_ref")
                n = len(ps["fault_types"]) if ref == "fault_types" else d.get("n_categories", 8)
                lows.append(0.0)
                highs.append(float(max(0, n - 1)))
            else:
                lows.append(float(d["low"]))
                highs.append(float(d["high"]))
        return lows, highs

    # ------------------------------------------------------------------ random
    @classmethod
    def random(cls, rng: random.Random, param_space: Optional[Dict] = None,
               seed: int = 0) -> "Genome":
        """Uniformly sample a valid genome using the given seeded RNG."""
        ps = param_space or load_param_space()
        values: Dict[str, Any] = {}
        for d in ps["dimensions"]:
            t = d["type"]
            if t in ("float", "gate"):
                values[d["name"]] = rng.uniform(d["low"], d["high"])
            elif t == "int":
                values[d["name"]] = rng.randint(int(d["low"]), int(d["high"]))
            elif t == "categorical":
                ref = d.get("categories_ref")
                n = len(ps["fault_types"]) if ref == "fault_types" else d.get("n_categories", 8)
                values[d["name"]] = rng.randrange(max(1, n))
        return cls(values, ps, seed=seed)

    # -------------------------------------------------------------- difficulty
    def difficulty(self) -> float:
        """Scalar difficulty descriptor in roughly [0, 1].

        A weighted average of each *active* knob's normalized magnitude using
        the param_space ``difficulty_weight``. Map/fault structure behind a
        disabled gate contributes nothing. Used by the curriculum controller to
        bin scenarios and target a competence frontier.
        """
        total_w = 0.0
        acc = 0.0
        # Which fault slots / map groups are active (gated)?
        active_gates = {
            "map": self._gate("mut_wall_jitter_enable") or self._gate("mut_aisle_enable")
                   or self._gate("mut_clutter_enable") or self._gate("mut_density_enable"),
        }
        for d in self.dims:
            w = float(d.get("difficulty_weight", 0.0))
            if w <= 0.0:
                continue
            name = d["name"]
            # Gate knobs contribute their on/off directly.
            if d["type"] == "gate":
                contrib = 1.0 if self._gate(name) else 0.0
            else:
                # Skip params whose owning gate/slot is inactive.
                if not self._knob_active(d):
                    continue
                contrib = self._normalized(d)
            acc += w * contrib
            total_w += w
            _ = active_gates  # documented intent; not used beyond clarity
        return acc / total_w if total_w > 0 else 0.0

    def _gate(self, name: str) -> bool:
        return float(self.values.get(name, 0.0)) >= _GATE_ON

    def _knob_active(self, d: Dict[str, Any]) -> bool:
        group = d.get("group")
        if group == "fault":
            slot = d.get("slot")
            return self._gate("fault%d_enable" % slot) if slot is not None else True
        if group == "map":
            # map magnitude knobs ride on their op's gate
            name = d["name"]
            if name.startswith("mut_wall"):
                return self._gate("mut_wall_jitter_enable")
            if name.startswith("mut_aisle"):
                return self._gate("mut_aisle_enable")
            if name.startswith("mut_clutter"):
                return self._gate("mut_clutter_enable")
            if name.startswith("mut_density"):
                return self._gate("mut_density_enable")
        return True

    def _normalized(self, d: Dict[str, Any]) -> float:
        t = d["type"]
        v = float(self.values[d["name"]])
        if t == "categorical":
            return 0.0
        lo, hi = float(d["low"]), float(d["high"])
        if hi == lo:
            return 0.0
        # symmetric knobs (e.g. aisle delta where negative eases) -> use |.|
        if lo < 0 < hi:
            return min(1.0, abs(v) / max(abs(lo), abs(hi)))
        return max(0.0, min(1.0, (v - lo) / (hi - lo)))

    # --------------------------------------------------------------- dict/json
    def to_dict(self) -> Dict[str, Any]:
        """Plain-JSON representation: ``{seed, values}``."""
        return {"seed": self.seed, "values": dict(self.values)}

    @classmethod
    def from_dict(cls, d: Dict[str, Any],
                  param_space: Optional[Dict] = None) -> "Genome":
        return cls(d.get("values", {}), param_space, seed=int(d.get("seed", 0)))

    def hash(self) -> str:
        """Stable short hash of (values, seed) for the reproducibility ledger."""
        blob = json.dumps(self.to_dict(), sort_keys=True).encode("utf-8")
        return hashlib.sha1(blob).hexdigest()[:12]

    def __repr__(self) -> str:
        return "Genome(seed=%d, hash=%s, difficulty=%.3f)" % (
            self.seed, self.hash(), self.difficulty())
