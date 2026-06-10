# Copyright 2026 Avidbots Corp.
# Common Proposer interface for the AI scenario generator.
"""The :class:`Proposer` contract every policy implements.

A proposer is the failure-seeking "brain": given the run history, it proposes
the next genome(s) and is told each genome's stress score. Concrete strategies
(RL black-box optimizer, LLM, hybrid) live alongside this module and are built
via :func:`make_proposer`.
"""

import abc
from typing import Any, Dict, List, Optional

from ..genome import Genome


class Proposal:
    """A proposed genome plus optional human-readable rationale."""

    def __init__(self, genome: Genome, rationale: str = ""):
        self.genome = genome
        self.rationale = rationale

    def __repr__(self) -> str:
        return "Proposal(%r, rationale=%r)" % (self.genome, self.rationale[:60])


class Proposer(abc.ABC):
    """Failure-seeking scenario proposer.

    Implementations maximize the stress score (subject to the curriculum's
    target difficulty / solvability constraints, which are passed as hints).
    """

    def __init__(self, param_space: Dict[str, Any], seed: int = 0):
        self.ps = param_space
        self.seed = seed

    @abc.abstractmethod
    def propose(self, history: List[Dict[str, Any]], k: int = 1,
                target_difficulty: Optional[float] = None,
                base_seed: int = 0) -> List[Proposal]:
        """Return up to ``k`` next genomes to evaluate.

        Args:
            history: list of ``{"genome": {...}, "score": float, ...}`` records,
                most-recent last.
            k: number of genomes to propose.
            target_difficulty: optional difficulty the curriculum wants (~[0,1]);
                a hint, proposers should bias toward it when they can.
            base_seed: world seed to stamp on proposed genomes.
        """

    def update(self, genome: Genome, score: float) -> None:
        """Feed back an evaluated (genome, stress score). Default: no-op."""


def make_proposer(strategy: str, param_space: Dict[str, Any], seed: int = 0,
                  **kwargs) -> Proposer:
    """Factory: ``strategy in {"random", "rl", "llm", "hybrid"}``."""
    strategy = (strategy or "rl").lower()
    if strategy == "random":
        from .rl_proposer import RandomProposer
        return RandomProposer(param_space, seed=seed, **kwargs)
    if strategy == "rl":
        from .rl_proposer import RLProposer
        return RLProposer(param_space, seed=seed, **kwargs)
    if strategy == "llm":
        from .llm_proposer import LLMProposer
        return LLMProposer(param_space, seed=seed, **kwargs)
    if strategy == "hybrid":
        from .llm_proposer import HybridProposer
        return HybridProposer(param_space, seed=seed, **kwargs)
    raise ValueError("unknown proposer strategy: %r" % strategy)
