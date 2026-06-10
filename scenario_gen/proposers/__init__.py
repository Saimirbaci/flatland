# Copyright 2026 Avidbots Corp.
# AI proposers for the scenario generator: pluggable failure-seeking policies
# behind a common Proposer interface (RL black-box optimizer, LLM, hybrid).
"""Proposer implementations. See base.Proposer for the contract."""

from .base import Proposer, Proposal, make_proposer  # noqa: F401
from .dr_proposer import DomainRandomizationProposer  # noqa: F401
