# Copyright 2026 Avidbots Corp.
# RL / black-box optimizer proposer for the AI scenario generator.
"""Failure-seeking proposers driven by black-box optimization.

* :class:`RandomProposer` — uniform sampling baseline (exploration / control).
* :class:`RLProposer` — a dependency-free Cross-Entropy-Method (a.k.a. Gaussian
  evolution strategy) over the flat genome vector in normalized [0,1] space. It
  treats the stress score as reward and refits a search distribution toward the
  elite (highest-scoring) genomes each generation, so the average proposed
  stress score climbs over generations. Optional NumPy accelerates nothing here;
  the implementation is pure stdlib so it runs anywhere the genome contract does.

Both honor a ``target_difficulty`` hint by over-sampling candidates and keeping
the one whose :meth:`Genome.difficulty` is closest to the requested band — a
light coupling to the curriculum without abandoning the reward objective.
"""

import random
from typing import Any, Dict, List, Optional

from ..genome import Genome
from .base import Proposal, Proposer


def _normalize(vec: List[float], lows: List[float], highs: List[float]):
    out = []
    for v, lo, hi in zip(vec, lows, highs):
        out.append(0.0 if hi <= lo else max(0.0, min(1.0, (v - lo) / (hi - lo))))
    return out


def _denormalize(nvec: List[float], lows: List[float], highs: List[float]):
    return [lo + n * (hi - lo) for n, lo, hi in zip(nvec, lows, highs)]


class RandomProposer(Proposer):
    """Uniform random genomes — exploration baseline."""

    def __init__(self, param_space: Dict[str, Any], seed: int = 0):
        super().__init__(param_space, seed)
        self.rng = random.Random(seed)

    def propose(self, history, k=1, target_difficulty=None, base_seed=0):
        out = []
        for i in range(k):
            g = self._sample_biased(target_difficulty, base_seed + i)
            out.append(Proposal(g, rationale="uniform random sample"))
        return out

    def _sample_biased(self, target, seed):
        cands = [Genome.random(self.rng, self.ps, seed=seed)
                 for _ in range(4 if target is not None else 1)]
        if target is None:
            return cands[0]
        return min(cands, key=lambda g: abs(g.difficulty() - target))


class RLProposer(Proposer):
    """Cross-Entropy-Method optimizer over the normalized genome vector.

    Args:
        elite_frac: fraction of evaluated genomes refit toward each generation.
        init_sigma: initial per-dim std (normalized space).
        sigma_floor: minimum std so the search never fully collapses.
        sigma_decay: multiplicative sigma decay per refit (annealing).
    """

    def __init__(self, param_space: Dict[str, Any], seed: int = 0,
                 elite_frac: float = 0.3, init_sigma: float = 0.35,
                 sigma_floor: float = 0.05, sigma_decay: float = 0.92):
        super().__init__(param_space, seed)
        self.rng = random.Random(seed)
        self.lows, self.highs = Genome.vector_bounds(param_space)
        n = len(self.lows)
        self.mean = [0.5] * n
        self.sigma = [init_sigma] * n
        self.elite_frac = elite_frac
        self.sigma_floor = sigma_floor
        self.sigma_decay = sigma_decay
        self._buffer = []          # (normalized_vec, score)
        self._fitted = False

    # ------------------------------------------------------------- optimization
    def update(self, genome: Genome, score: float) -> None:
        nvec = _normalize(genome.to_vector(), self.lows, self.highs)
        self._buffer.append((nvec, float(score)))

    def _refit(self) -> None:
        if len(self._buffer) < 4:
            return
        ranked = sorted(self._buffer, key=lambda t: t[1], reverse=True)
        n_elite = max(2, int(round(self.elite_frac * len(ranked))))
        elite = [vec for vec, _ in ranked[:n_elite]]
        dim = len(self.mean)
        for j in range(dim):
            col = [vec[j] for vec in elite]
            mu = sum(col) / len(col)
            var = sum((c - mu) ** 2 for c in col) / len(col)
            self.mean[j] = mu
            self.sigma[j] = max(self.sigma_floor,
                                (var ** 0.5) * 1.0, self.sigma[j] * self.sigma_decay
                                if self._fitted else var ** 0.5)
        self._fitted = True

    # ------------------------------------------------------------------ propose
    def propose(self, history, k=1, target_difficulty=None, base_seed=0):
        # Refit from everything observed so far before proposing this generation.
        self._refit()
        out = []
        for i in range(k):
            g = self._sample_biased(target_difficulty, base_seed + i)
            rationale = ("CEM sample (sigma~%.2f, %d obs)"
                         % (sum(self.sigma) / len(self.sigma), len(self._buffer)))
            out.append(Proposal(g, rationale=rationale))
        return out

    def _sample_once(self, seed: int) -> Genome:
        if not self._fitted and not self._buffer:
            # First generation: explore uniformly for diversity.
            return Genome.random(self.rng, self.ps, seed=seed)
        nvec = [max(0.0, min(1.0, self.rng.gauss(m, s)))
                for m, s in zip(self.mean, self.sigma)]
        vec = _denormalize(nvec, self.lows, self.highs)
        return Genome.from_vector(vec, self.ps, seed=seed)

    def _sample_biased(self, target, seed):
        oversample = 6 if target is not None else 1
        cands = [self._sample_once(seed) for _ in range(oversample)]
        if target is None:
            return cands[0]
        return min(cands, key=lambda g: abs(g.difficulty() - target))
