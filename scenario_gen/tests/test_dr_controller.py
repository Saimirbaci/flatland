# Copyright 2026 Avidbots Corp.
# Unit + integration tests for the closed-loop domain-randomization controller:
# the adaptive DRDistribution, the DR proposer, the failure-boundary estimator,
# and the end-to-end orchestrate loop (all offline via the stub evaluator).
"""Run with ``python -m pytest scenario_gen/tests`` or ``python -m unittest``."""

import os
import random
import shutil
import tempfile
import unittest

from scenario_gen.genome import Genome, load_param_space
from scenario_gen.dr_distribution import DRDistribution
from scenario_gen.proposers import make_proposer
from scenario_gen.run import stub_score
from scenario_gen.failure_boundary import estimate_boundary, write_report, is_failure
from scenario_gen import orchestrate

PS = load_param_space()


class DRDistributionTest(unittest.TestCase):
    def test_sample_is_valid_and_bounded(self):
        dist = DRDistribution(PS, seed=1)
        rng = random.Random(0)
        for i in range(20):
            g = dist.sample(rng, seed=i)
            for d in PS["dimensions"]:
                v = g.values[d["name"]]
                if d["type"] in ("float", "gate"):
                    self.assertGreaterEqual(v, d["low"])
                    self.assertLessEqual(v, d["high"])

    def test_refit_moves_mass_toward_high_score_region(self):
        # Synthetic objective: score rewards a high fault0_peak (with fault0 on).
        dist = DRDistribution(PS, seed=2)
        rng = random.Random(2)
        peak_idx = [i for i, d in enumerate(PS["dimensions"])
                    if d["name"] == "fault0_peak"][0]
        before = dist.params[peak_idx]["mean"]
        for _ in range(6):
            samples = []
            for i in range(20):
                g = dist.sample(rng, seed=i)
                score = g.values["fault0_peak"] if g.values["fault0_enable"] >= 0.5 else 0.0
                samples.append((g, score))
            dist.refit(samples, fail_threshold=0.6)
        after = dist.params[peak_idx]["mean"]
        # Mass for fault0_peak should shift upward (normalized mean rises).
        self.assertGreater(after, before)
        # And the gate should be biased on.
        gate_idx = [i for i, d in enumerate(PS["dimensions"])
                    if d["name"] == "fault0_enable"][0]
        self.assertGreater(dist.params[gate_idx]["p"], 0.5)

    def test_entropy_floor_prevents_collapse(self):
        # Even when every elite is identical, entropy must stay above 0.
        dist = DRDistribution(PS, seed=3)
        fixed = Genome({"fault0_enable": 1.0, "fault0_peak": 1.0}, PS, seed=0)
        samples = [(fixed, 1.0)] * 20
        for _ in range(10):
            dist.refit(samples, fail_threshold=0.6)
        self.assertGreater(dist.entropy(), 0.0)
        # float sigmas respect the floor
        for p in dist.params:
            if p["type"] == "float":
                self.assertGreaterEqual(p["sigma"], dist.sigma_floor - 1e-9)
            if p["type"] == "gate":
                self.assertLessEqual(p["p"], 1.0 - dist.gate_floor + 1e-9)

    def test_checkpoint_roundtrip(self):
        dist = DRDistribution(PS, seed=4)
        rng = random.Random(4)
        samples = [(dist.sample(rng, seed=i), random.Random(i).random())
                   for i in range(20)]
        dist.refit(samples, fail_threshold=0.5)
        d = dist.to_dict()
        restored = DRDistribution.from_dict(d, PS)
        self.assertEqual(restored.to_dict(), d)
        # Restored distribution samples identically given the same RNG.
        a = dist.sample(random.Random(99), seed=7).to_vector()
        b = restored.sample(random.Random(99), seed=7).to_vector()
        self.assertEqual(a, b)


class DRProposerTest(unittest.TestCase):
    def test_dr_climbs_stub_score(self):
        p = make_proposer("dr", PS, seed=1)
        means = []
        for gen in range(12):
            props = p.propose([], k=8, base_seed=gen * 100)
            scores = []
            for pr in props:
                s = stub_score(pr.genome)["composite_score"]
                p.update(pr.genome, s)
                scores.append(s)
            means.append(sum(scores) / len(scores))
        self.assertGreater(means[-1], means[0] + 0.05,
                           "DR proposer should drive stress upward")

    def test_dr_emits_valid_bounded_genomes(self):
        p = make_proposer("dr", PS, seed=2)
        for pr in p.propose([], k=4, target_difficulty=0.5, base_seed=0):
            for d in PS["dimensions"]:
                if d["type"] in ("float", "gate"):
                    v = pr.genome.values[d["name"]]
                    self.assertGreaterEqual(v, d["low"])
                    self.assertLessEqual(v, d["high"])

    def test_rationale_reports_entropy(self):
        p = make_proposer("dr", PS, seed=3)
        pr = p.propose([], k=1, base_seed=0)[0]
        self.assertIn("entropy", pr.rationale)


class FailureBoundaryTest(unittest.TestCase):
    def _history(self, rule, n=200, seed=0):
        rng = random.Random(seed)
        hist = []
        for i in range(n):
            g = Genome.random(rng, PS, seed=i)
            fail = rule(g)
            hist.append({"genome": g.to_dict(),
                         "score": 0.9 if fail else 0.2,
                         "terminal": "stub", "invalid": False})
        return hist

    def test_fail_criterion(self):
        self.assertTrue(is_failure(0.7, "stub", 0.6))
        self.assertFalse(is_failure(0.4, "stub", 0.6))
        self.assertTrue(is_failure(0.0, "collision", 0.6))
        self.assertTrue(is_failure(0.1, "timeout", 0.6))

    def test_recovers_known_threshold(self):
        # fail iff knob fault0_peak > 0.5 (and fault0 enabled)
        rule = lambda g: g.values["fault0_enable"] >= 0.5 and g.values["fault0_peak"] > 0.5
        report = estimate_boundary(self._history(rule), fail_threshold=0.6,
                                   param_space=PS)
        self.assertGreater(report["n_failures"], 0)
        # The driving dimension should rank first (surrogate or sensitivity).
        self.assertIn(report["ranked_dimensions"][0],
                      ("fault0_peak", "fault0_enable"))
        peak = [e for e in report["per_dim"] if e["name"] == "fault0_peak"][0]
        self.assertEqual(peak["direction"], "higher")
        self.assertIsNotNone(peak["crossing"])
        # Crossing should be near the true 0.5 boundary.
        self.assertLess(abs(peak["crossing"] - 0.5), 0.2)

    def test_no_failures_reports_coverage(self):
        report = estimate_boundary(self._history(lambda g: False), fail_threshold=0.6,
                                   param_space=PS)
        self.assertEqual(report["n_failures"], 0)
        self.assertIn("No failures", report["coverage_note"])

    def test_write_report_artifacts(self):
        tmp = tempfile.mkdtemp(prefix="scn_fb_")
        try:
            rule = lambda g: g.values["fault0_peak"] > 0.5
            report = estimate_boundary(self._history(rule), 0.6, PS)
            paths = write_report(report, tmp)
            self.assertTrue(os.path.exists(paths["json"]))
            self.assertTrue(os.path.exists(paths["markdown"]))
            with open(paths["markdown"]) as f:
                self.assertIn("Failure boundary report", f.read())
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class DROrchestrateSmokeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="scn_dr_orch_")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_end_to_end_produces_boundary(self):
        summary = orchestrate.run_curriculum(
            proposer_strategy="dr", generations=3, per_generation=6,
            replay_per_generation=0, ledger_dir=self.tmp, base_seed=1,
            stub=True, fail_threshold=0.5, resume=False)
        self.assertEqual(len(summary["gen_mean_scores"]), 3)
        # Boundary artifacts exist and are referenced from the summary.
        self.assertTrue(os.path.exists(summary["failure_boundary_report"]))
        self.assertTrue(os.path.exists(summary["failure_boundary_markdown"]))
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "failure_boundary.json")))
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "dr_distribution.json")))
        self.assertIn("top_failure_dimensions", summary)
        self.assertIn("fail_rate", summary)


if __name__ == "__main__":
    unittest.main()
