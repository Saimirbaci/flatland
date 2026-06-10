# Copyright 2026 Avidbots Corp.
# Adaptive domain-randomization distribution for the closed-loop DR controller:
# a per-dimension parametric distribution over the scenario genome that shifts
# its mass each generation toward high-stress (failure) regions.
"""Adaptive randomization distribution over the genome parameter space.

This is the heart of the *closed-loop domain-randomization controller*. Where a
plain domain-randomization sweep samples from a fixed distribution, this one is
**active**: after each generation it refits itself toward the genomes that
stressed the algorithm-under-test the most, so successive generations
concentrate samples near the AUT's failure boundary instead of wasting budget on
scenarios it already handles.

Adaptation algorithm (decision)
-------------------------------
The adaptation rule is the **Cross-Entropy Method (CEM) with score-weighted
elite reweighting** — per-dimension moment matching over the existing genome
space. It was chosen over a Bayesian/surrogate optimizer (Gaussian-process or
random-forest acquisition) because:

* it is **pure stdlib** (``random``/``math``), matching ``genome.py``'s
  deliberate "no ML deps" constraint, so it runs anywhere the genome contract
  does;
* it is **deterministic** under a seeded RNG (reproducible runs / ledgers);
* it maps cleanly onto the existing ``Proposer.update(genome, score)`` feedback
  contract — buffer scored samples, refit at generation boundaries.

A surrogate strategy can be added later without touching the
:class:`~scenario_gen.proposers.base.Proposer` interface: :meth:`refit` is the
single pluggable adaptation hook, and the distribution serializes to/from JSON
so an alternative learner could consume the same checkpoint.

Per-dimension distribution families are fixed by the knob type:

* ``float`` / ``int`` knobs -> **truncated Gaussian** over the normalized
  ``[0, 1]`` range (clamped back into ``[low, high]`` at sample time);
* ``gate`` knobs -> **Bernoulli** (probability the structure is toggled on);
* ``categorical`` knobs -> a **categorical weight vector** over the knob's
  candidate indices.

CEM can collapse prematurely (entropy -> 0) and stop exploring; every family
therefore carries an **entropy/variance floor** so the distribution keeps
sampling broadly even after many refits.
"""

import math
import random
from typing import Any, Dict, List, Optional, Tuple

from .genome import Genome, load_param_space


def _n_categories(ps: Dict[str, Any], d: Dict[str, Any]) -> int:
    """Category count for a categorical knob (mirrors Genome._n_categories)."""
    ref = d.get("categories_ref")
    if ref == "fault_types":
        return len(ps["fault_types"])
    return d.get("n_categories", 8)


class DRDistribution:
    """Per-dimension adaptive randomization distribution over the genome vector.

    The distribution is built from ``load_param_space()`` and keeps one
    parametric family per knob, in the genome's fixed knob order. Sampling
    produces valid (clamped) :class:`Genome` objects; :meth:`refit` performs the
    CEM elite-reweighting update; :meth:`entropy`/:meth:`spread` track
    convergence; :meth:`to_dict`/:meth:`from_dict` checkpoint the state.

    Args:
        param_space: parsed param_space (loaded from disk when omitted).
        seed: RNG seed for any internal stochastic helpers (kept for
            reproducibility; :meth:`sample` takes its own RNG).
        elite_frac: fraction of scored samples treated as elite each refit.
        init_sigma: initial per-dim std for float/int knobs (normalized space).
        sigma_floor: minimum float/int std so the search never fully collapses.
        gate_floor: clamp Bernoulli ``p`` to ``[gate_floor, 1 - gate_floor]``.
        cat_smoothing: additive (Laplace) smoothing on categorical weights.
        lr: refit learning rate — blend factor from old params toward the new
            elite moments (1.0 == replace, the classic CEM update).
    """

    def __init__(self, param_space: Optional[Dict[str, Any]] = None,
                 seed: int = 0, elite_frac: float = 0.3,
                 init_sigma: float = 0.30, sigma_floor: float = 0.06,
                 gate_floor: float = 0.05, cat_smoothing: float = 0.5,
                 lr: float = 1.0):
        self.ps = param_space or load_param_space()
        self.dims = self.ps["dimensions"]
        self.seed = int(seed)
        self.rng = random.Random(seed)
        self.elite_frac = float(elite_frac)
        self.init_sigma = float(init_sigma)
        self.sigma_floor = float(sigma_floor)
        self.gate_floor = float(gate_floor)
        self.cat_smoothing = float(cat_smoothing)
        self.lr = float(lr)
        self.n_refits = 0
        self.params: List[Dict[str, Any]] = [self._init_param(d) for d in self.dims]

    # --------------------------------------------------------------- init / bounds
    def _init_param(self, d: Dict[str, Any]) -> Dict[str, Any]:
        t = d["type"]
        if t == "gate":
            # Start from the param_space default's on/off bias.
            p = 1.0 if float(d.get("default", 0.0)) >= 0.5 else 0.5
            return {"type": "gate", "p": self._clamp_p(p)}
        if t == "categorical":
            n = max(1, _n_categories(self.ps, d))
            return {"type": "categorical", "weights": [1.0 / n] * n}
        # float / int -> normalized truncated Gaussian. Center on the default.
        lo, hi = self._dim_bounds(d)
        default = float(d.get("default", lo))
        mean = 0.5 if hi <= lo else max(0.0, min(1.0, (default - lo) / (hi - lo)))
        return {"type": "float", "mean": mean, "sigma": self.init_sigma}

    def _dim_bounds(self, d: Dict[str, Any]) -> Tuple[float, float]:
        """``(low, high)`` for a float/int knob in genome units."""
        return float(d["low"]), float(d["high"])

    def _clamp_p(self, p: float) -> float:
        return max(self.gate_floor, min(1.0 - self.gate_floor, float(p)))

    # ------------------------------------------------------------------- sampling
    def sample(self, rng: random.Random, seed: int = 0) -> Genome:
        """Draw one valid :class:`Genome` from the current distribution."""
        values: Dict[str, Any] = {}
        for d, p in zip(self.dims, self.params):
            name, t = d["name"], d["type"]
            if t == "gate":
                values[name] = 1.0 if rng.random() < p["p"] else 0.0
            elif t == "categorical":
                values[name] = self._sample_categorical(rng, p["weights"])
            else:  # float / int
                lo, hi = self._dim_bounds(d)
                nval = max(0.0, min(1.0, rng.gauss(p["mean"], p["sigma"])))
                values[name] = lo + nval * (hi - lo)
        # Genome() clamps int rounding / categorical wrap / float bounds for us.
        return Genome(values, self.ps, seed=seed)

    @staticmethod
    def _sample_categorical(rng: random.Random, weights: List[float]) -> int:
        total = sum(weights)
        if total <= 0.0:
            return rng.randrange(len(weights))
        r = rng.random() * total
        acc = 0.0
        for i, w in enumerate(weights):
            acc += w
            if r <= acc:
                return i
        return len(weights) - 1

    # ----------------------------------------------------------------- adaptation
    def refit(self, scored_samples: List[Tuple[Genome, float]],
              fail_threshold: Optional[float] = None) -> bool:
        """CEM elite-reweighting update toward high-score (failure) samples.

        Args:
            scored_samples: ``[(genome, stress_score), ...]`` observed this
                generation (or the whole buffered history).
            fail_threshold: if given, samples scoring ``>= fail_threshold`` are
                always treated as elite (the failure region we want to chase),
                in addition to the top ``elite_frac`` by score.

        Returns ``True`` if the distribution was updated (enough elites), else
        ``False`` (too few samples — distribution left unchanged so it keeps
        exploring).
        """
        if len(scored_samples) < 4:
            return False
        ranked = sorted(scored_samples, key=lambda t: t[1], reverse=True)
        n_elite = max(2, int(round(self.elite_frac * len(ranked))))
        elite = [g for g, _ in ranked[:n_elite]]
        if fail_threshold is not None:
            for g, s in ranked[n_elite:]:
                if s >= fail_threshold:
                    elite.append(g)
        if len(elite) < 2:
            return False

        for i, d in enumerate(self.dims):
            self.params[i] = self._refit_dim(d, self.params[i], elite)
        self.n_refits += 1
        return True

    def _refit_dim(self, d: Dict[str, Any], prev: Dict[str, Any],
                   elite: List[Genome]) -> Dict[str, Any]:
        name, t = d["name"], d["type"]
        if t == "gate":
            on = sum(1 for g in elite if float(g.values[name]) >= 0.5)
            p_new = on / float(len(elite))
            p = (1.0 - self.lr) * prev["p"] + self.lr * p_new
            return {"type": "gate", "p": self._clamp_p(p)}
        if t == "categorical":
            n = len(prev["weights"])
            counts = [self.cat_smoothing] * n
            for g in elite:
                idx = int(g.values[name]) % n
                counts[idx] += 1.0
            tot = sum(counts)
            w_new = [c / tot for c in counts]
            w = [(1.0 - self.lr) * o + self.lr * nw
                 for o, nw in zip(prev["weights"], w_new)]
            s = sum(w)
            return {"type": "categorical", "weights": [x / s for x in w]}
        # float / int: moment-match in normalized space with a variance floor.
        lo, hi = self._dim_bounds(d)
        col = []
        for g in elite:
            v = float(g.values[name])
            col.append(0.0 if hi <= lo else max(0.0, min(1.0, (v - lo) / (hi - lo))))
        mu = sum(col) / len(col)
        var = sum((c - mu) ** 2 for c in col) / len(col)
        mean = (1.0 - self.lr) * prev["mean"] + self.lr * mu
        sigma = max(self.sigma_floor, math.sqrt(var))
        return {"type": "float", "mean": max(0.0, min(1.0, mean)), "sigma": sigma}

    # ----------------------------------------------------------------- convergence
    def entropy(self) -> float:
        """Mean per-dimension entropy proxy in roughly ``[0, 1]`` (1 == broad).

        float/int -> sigma scaled against ``init_sigma``; gate -> normalized
        Bernoulli entropy; categorical -> normalized Shannon entropy. Useful to
        watch the distribution converge (and to confirm the entropy floor keeps
        it from collapsing to 0).
        """
        if not self.params:
            return 0.0
        acc = 0.0
        for p in self.params:
            if p["type"] == "gate":
                acc += _bernoulli_entropy(p["p"])
            elif p["type"] == "categorical":
                acc += _categorical_entropy(p["weights"])
            else:
                acc += max(0.0, min(1.0, p["sigma"] / max(1e-9, self.init_sigma)))
        return acc / len(self.params)

    def spread(self) -> float:
        """Alias for :meth:`entropy` (convergence-tracking convenience)."""
        return self.entropy()

    # --------------------------------------------------------------- checkpointing
    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": 1,
            "seed": self.seed,
            "n_refits": self.n_refits,
            "config": {
                "elite_frac": self.elite_frac, "init_sigma": self.init_sigma,
                "sigma_floor": self.sigma_floor, "gate_floor": self.gate_floor,
                "cat_smoothing": self.cat_smoothing, "lr": self.lr,
            },
            "params": self.params,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any],
                  param_space: Optional[Dict[str, Any]] = None) -> "DRDistribution":
        cfg = d.get("config", {})
        obj = cls(param_space=param_space, seed=int(d.get("seed", 0)),
                  elite_frac=cfg.get("elite_frac", 0.3),
                  init_sigma=cfg.get("init_sigma", 0.30),
                  sigma_floor=cfg.get("sigma_floor", 0.06),
                  gate_floor=cfg.get("gate_floor", 0.05),
                  cat_smoothing=cfg.get("cat_smoothing", 0.5),
                  lr=cfg.get("lr", 1.0))
        obj.n_refits = int(d.get("n_refits", 0))
        params = d.get("params")
        if params and len(params) == len(obj.params):
            obj.params = params
        return obj

    def __repr__(self) -> str:
        return "DRDistribution(dims=%d, refits=%d, entropy=%.3f)" % (
            len(self.params), self.n_refits, self.entropy())


def _bernoulli_entropy(p: float) -> float:
    """Binary entropy normalized to ``[0, 1]`` (1 at p=0.5)."""
    if p <= 0.0 or p >= 1.0:
        return 0.0
    return -(p * math.log(p) + (1.0 - p) * math.log(1.0 - p)) / math.log(2.0)


def _categorical_entropy(weights: List[float]) -> float:
    """Shannon entropy of a weight vector normalized to ``[0, 1]``."""
    n = len(weights)
    if n <= 1:
        return 0.0
    tot = sum(weights)
    if tot <= 0.0:
        return 1.0
    h = 0.0
    for w in weights:
        pw = w / tot
        if pw > 0.0:
            h -= pw * math.log(pw)
    return h / math.log(n)
