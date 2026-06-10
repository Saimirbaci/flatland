# Copyright 2026 Avidbots Corp.
# LLM-driven proposer for the AI scenario generator.
"""Claude-driven failure-seeking scenario proposer.

Given the param_space spec and a history of (genome, score, failure summary)
tuples, the LLM proposes the next genome(s) as structured JSON, reasoning about
*why* prior scenarios stressed the algorithm-under-test and how to escalate
(e.g. "localization failed under aisle narrowing + encoder drift -> combine with
a dynamic obstacle at the chokepoint"). It excels at the structured/categorical
composition (which faults, where) that a numeric optimizer struggles with.

Implementation notes (follows the claude-api skill conventions):
* **Prompt caching** of the large, static param_space system block
  (``cache_control: ephemeral``) so only the short rolling history is re-sent.
* **Structured output** via a forced tool call whose ``input_schema`` is built
  from the param_space, so the model returns schema-shaped data, not prose.
* **Validation + repair**: every returned genome is clamped to bounds via
  :class:`Genome`; anything unparseable falls back to the RL proposer so the
  outer loop never stalls on an API hiccup.

Degrades gracefully: if the ``anthropic`` SDK or an API key is unavailable, it
transparently falls back to :class:`RLProposer` (logged once).
"""

import json
import os
from typing import Any, Dict, List, Optional

from ..genome import Genome
from .base import Proposal, Proposer
from .rl_proposer import RLProposer

DEFAULT_MODEL = "claude-opus-4-8"


def _genome_tool_schema(ps: Dict[str, Any]) -> Dict[str, Any]:
    """Build the structured-output tool input_schema from param_space dims."""
    props = {}
    for d in ps["dimensions"]:
        prop = {"type": "number", "description": d.get("description", "")}
        if d["type"] == "categorical":
            ref = d.get("categories_ref")
            n = len(ps["fault_types"]) if ref == "fault_types" else d.get("n_categories", 8)
            prop["description"] += " (categorical index 0..%d)" % (n - 1)
        else:
            prop["minimum"] = d["low"]
            prop["maximum"] = d["high"]
        props[d["name"]] = prop
    values_schema = {"type": "object", "properties": props,
                     "description": "All genome knob values (see param_space)."}
    return {
        "type": "object",
        "properties": {
            "scenarios": {
                "type": "array",
                "description": "One entry per proposed scenario genome.",
                "items": {
                    "type": "object",
                    "properties": {
                        "rationale": {"type": "string",
                                      "description": "Why this stresses the AUT / how it escalates."},
                        "values": values_schema,
                    },
                    "required": ["rationale", "values"],
                },
            }
        },
        "required": ["scenarios"],
    }


def _fault_type_names(ps: Dict[str, Any]) -> List[str]:
    return [t["type"] for t in ps["fault_types"]]


class LLMProposer(Proposer):
    def __init__(self, param_space: Dict[str, Any], seed: int = 0,
                 model: str = DEFAULT_MODEL, max_history: int = 12,
                 api_key: Optional[str] = None, fallback: bool = True,
                 max_tokens: int = 4096):
        super().__init__(param_space, seed)
        self.model = model
        self.max_history = max_history
        self.api_key = api_key or os.environ.get("ANTHROPIC_API_KEY")
        self.max_tokens = max_tokens
        self._fallback = RLProposer(param_space, seed=seed) if fallback else None
        self._client = None
        self._warned = False
        self._tool_schema = _genome_tool_schema(param_space)

    # --------------------------------------------------------------- SDK client
    def _get_client(self):
        if self._client is not None:
            return self._client
        if not self.api_key:
            return None
        try:
            import anthropic
            self._client = anthropic.Anthropic(api_key=self.api_key)
            return self._client
        except Exception:
            return None

    def _warn_fallback(self, why: str) -> None:
        if not self._warned:
            print("scenario_gen.llm_proposer: %s; falling back to RL proposer." % why)
            self._warned = True

    # ------------------------------------------------------------------ prompts
    def _system_blocks(self) -> List[Dict[str, Any]]:
        spec = json.dumps(self.ps, indent=2)
        guide = (
            "You are an adversarial test designer for a 2D ground-robot navigation "
            "stack simulated in Flatland. You propose SCENARIO GENOMES that make the "
            "algorithm-under-test FAIL (high localization error, collisions, missed "
            "goals, path deviation) while staying SOLVABLE (a competent robot could "
            "still occasionally succeed). Reason about why past scenarios stressed "
            "the stack and escalate by COMBINING stressors (e.g. narrow an aisle AND "
            "add an encoder drift fault AND place a dynamic obstacle at the chokepoint). "
            "Each genome is a flat set of knobs defined by the param_space below. "
            "Gates (>=0.5 = on) toggle map ops and fault slots; fault*_type is a "
            "categorical index into fault_types. Always return values within bounds."
        )
        return [
            {"type": "text", "text": guide},
            {"type": "text", "text": "PARAM_SPACE:\n" + spec,
             "cache_control": {"type": "ephemeral"}},
        ]

    def _history_message(self, history, k, target) -> str:
        ftypes = _fault_type_names(self.ps)
        lines = ["fault_types index legend: " +
                 ", ".join("%d=%s" % (i, n) for i, n in enumerate(ftypes))]
        if history:
            lines.append("\nRecent scenarios (genome hash, stress score, notes):")
            for rec in history[-self.max_history:]:
                gh = rec.get("genome_hash", rec.get("genome", {}).get("seed", "?"))
                sc = rec.get("score", rec.get("composite_score"))
                term = rec.get("terminal", "")
                lines.append("- %s score=%.3f %s" % (gh, sc if sc is not None else -1, term))
        else:
            lines.append("\nNo history yet — propose a diverse opening batch.")
        if target is not None:
            lines.append("\nTarget difficulty ~%.2f (0=trivial,1=extreme). Aim near it "
                         "so the AUT succeeds ~50-70%% of the time." % target)
        lines.append("\nPropose %d NEW genome(s) that escalate failure. Call the tool." % k)
        return "\n".join(lines)

    # ------------------------------------------------------------------ propose
    def propose(self, history, k=1, target_difficulty=None, base_seed=0):
        client = self._get_client()
        if client is None:
            self._warn_fallback("anthropic SDK/API key unavailable")
            return self._fallback_propose(history, k, target_difficulty, base_seed)
        try:
            resp = client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                system=self._system_blocks(),
                tools=[{"name": "propose_scenarios",
                        "description": "Return the proposed scenario genomes.",
                        "input_schema": self._tool_schema}],
                tool_choice={"type": "tool", "name": "propose_scenarios"},
                messages=[{"role": "user",
                           "content": self._history_message(history, k, target_difficulty)}],
            )
            scenarios = self._extract(resp)
        except Exception as e:
            self._warn_fallback("API/parse error: %s" % e)
            return self._fallback_propose(history, k, target_difficulty, base_seed)

        out = []
        for i, sc in enumerate(scenarios[:k]):
            try:
                g = Genome(sc.get("values", {}), self.ps, seed=base_seed + i)
            except Exception:
                # repair: drop unknown keys, keep valid ones
                vals = {kk: vv for kk, vv in sc.get("values", {}).items()
                        if kk in {d["name"] for d in self.ps["dimensions"]}}
                g = Genome(vals, self.ps, seed=base_seed + i)
            out.append(Proposal(g, rationale=sc.get("rationale", "")))
        if not out:  # model returned nothing usable
            return self._fallback_propose(history, k, target_difficulty, base_seed)
        return out

    def _extract(self, resp) -> List[Dict[str, Any]]:
        for block in resp.content:
            if getattr(block, "type", None) == "tool_use":
                return block.input.get("scenarios", [])
        return []

    def _fallback_propose(self, history, k, target, base_seed):
        if self._fallback is None:
            raise RuntimeError("LLM proposer unavailable and fallback disabled")
        return self._fallback.propose(history, k, target, base_seed)

    def update(self, genome: Genome, score: float) -> None:
        if self._fallback is not None:
            self._fallback.update(genome, score)


class HybridProposer(Proposer):
    """LLM proposes structure; RL tunes magnitudes.

    Concretely: half the batch comes from the LLM (semantic, combinatorial
    composition) and half from the RL optimizer (precise numeric tuning around
    the elite). Both proposers are fed every evaluated score, so the RL half
    sharpens the magnitudes of whatever structures the LLM keeps discovering.
    """

    def __init__(self, param_space: Dict[str, Any], seed: int = 0, **kwargs):
        super().__init__(param_space, seed)
        self.llm = LLMProposer(param_space, seed=seed, **kwargs)
        self.rl = RLProposer(param_space, seed=seed)

    def propose(self, history, k=1, target_difficulty=None, base_seed=0):
        n_llm = (k + 1) // 2
        n_rl = k - n_llm
        out = self.llm.propose(history, n_llm, target_difficulty, base_seed)
        if n_rl > 0:
            out += self.rl.propose(history, n_rl, target_difficulty,
                                   base_seed + n_llm)
        return out

    def update(self, genome: Genome, score: float) -> None:
        self.llm.update(genome, score)
        self.rl.update(genome, score)
