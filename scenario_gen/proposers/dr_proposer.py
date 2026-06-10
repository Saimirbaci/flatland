# Copyright 2026 Avidbots Corp.
# Closed-loop domain-randomization proposer for the AI scenario generator: an
# adaptive randomization distribution that shifts toward failure regions.
"""Domain-randomization proposer — active failure-finding.

:class:`DomainRandomizationProposer` wraps a
:class:`~scenario_gen.dr_distribution.DRDistribution` in the standard
:class:`~scenario_gen.proposers.base.Proposer` contract. Each generation it
draws ``k`` genomes from the *current* randomization distribution; as the
orchestrator feeds back ``(genome, stress score)`` via :meth:`update`, it buffers
them and refits the distribution at generation boundaries so its mass shifts
toward the regions where the algorithm-under-test fails. That closed loop turns a
passive domain-randomization sweep into an active failure-finder.

The proposer is deliberately thin: all of the adaptation math lives in
``DRDistribution`` (the pluggable hook), so a future surrogate learner can be
dropped in without changing this class or the ``Proposer`` interface.
"""

import random
from typing import Any, Dict, List, Optional

from ..dr_distribution import DRDistribution
from ..genome import Genome
from .base import Proposal, Proposer


class DomainRandomizationProposer(Proposer):
    """Closed-loop DR controller behind the :class:`Proposer` interface.

    Args:
        param_space: parsed param_space contract.
        seed: RNG seed (reproducible sampling + distribution init).
        fail_threshold: stress score at/above which a sample is always treated
            as elite during a refit (the failure region we steer toward).
        refit_every: refit the distribution once this many *new* scored samples
            have buffered since the last refit (a generation boundary). The
            ``Proposer.update`` contract is per-scenario, so we batch here.
        **dist_kwargs: forwarded to :class:`DRDistribution` (elite_frac,
            init_sigma, sigma_floor, gate_floor, cat_smoothing, lr).
    """

    def __init__(self, param_space: Dict[str, Any], seed: int = 0,
                 fail_threshold: float = 0.6, refit_every: int = 4,
                 **dist_kwargs):
        super().__init__(param_space, seed)
        self.rng = random.Random(seed)
        self.dist = DRDistribution(param_space, seed=seed, **dist_kwargs)
        self.fail_threshold = float(fail_threshold)
        self.refit_every = max(1, int(refit_every))
        self._buffer = []            # (Genome, score) — all observed
        self._since_refit = 0        # new samples since the last refit

    # ------------------------------------------------------------------- feedback
    def update(self, genome: Genome, score: float) -> None:
        """Buffer one scored sample; refit at the generation boundary."""
        self._buffer.append((genome, float(score)))
        self._since_refit += 1
        if self._since_refit >= self.refit_every:
            if self.dist.refit(self._buffer, fail_threshold=self.fail_threshold):
                self._since_refit = 0

    # ------------------------------------------------------------------- proposing
    def propose(self, history, k=1, target_difficulty=None, base_seed=0):
        out = []
        for i in range(k):
            g = self._sample_biased(target_difficulty, base_seed + i)
            rationale = ("DR sample @ entropy=%.3f (%d refits, %d obs)"
                         % (self.dist.entropy(), self.dist.n_refits,
                            len(self._buffer)))
            out.append(Proposal(g, rationale=rationale))
        return out

    def _sample_biased(self, target: Optional[float], seed: int) -> Genome:
        """Sample once, or oversample and keep the closest to ``target``."""
        oversample = 6 if target is not None else 1
        cands = [self.dist.sample(self.rng, seed=seed) for _ in range(oversample)]
        if target is None:
            return cands[0]
        return min(cands, key=lambda g: abs(g.difficulty() - target))
