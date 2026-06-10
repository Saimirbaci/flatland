# Copyright 2026 Avidbots Corp.
# Unit tests for the AI scenario generator's Python orchestrator: genome
# contract, renderer, proposers, curriculum, determinism, and the loop smoke
# test (all offline via the deterministic stub evaluator -- no ROS needed).
"""Run with ``python -m pytest scenario_gen/tests`` or ``python -m unittest``."""

import json
import os
import random
import shutil
import tempfile
import unittest

import yaml

from scenario_gen.genome import Genome, load_param_space, GenomeError
from scenario_gen import render as render_mod
from scenario_gen.proposers import make_proposer
from scenario_gen.curriculum import CurriculumController, is_solved, is_solvable
from scenario_gen.run import stub_score
from scenario_gen import orchestrate

PS = load_param_space()


class GenomeTest(unittest.TestCase):
    def test_vector_roundtrip(self):
        g = Genome.random(random.Random(1), PS, seed=5)
        g2 = Genome.from_vector(g.to_vector(), PS, seed=5)
        self.assertEqual(g.to_vector(), g2.to_vector())

    def test_clamp_bounds(self):
        g = Genome({"fault0_peak": 99.0, "mut_density_scale": -10.0}, PS)
        self.assertLessEqual(g.values["fault0_peak"], 1.0)
        self.assertGreaterEqual(g.values["mut_density_scale"], 0.5)

    def test_unknown_knob_rejected(self):
        with self.assertRaises(GenomeError):
            Genome({"not_a_knob": 1.0}, PS)

    def test_difficulty_monotonic(self):
        easy = Genome({"fault0_enable": 0.0}, PS)
        hard = Genome({"fault0_enable": 1.0, "fault0_peak": 1.0,
                       "fault0_magnitude": 1.0, "fault0_type": 0}, PS)
        self.assertGreater(hard.difficulty(), easy.difficulty())

    def test_hash_deterministic(self):
        g1 = Genome({"fault0_enable": 1.0}, PS, seed=3)
        g2 = Genome({"fault0_enable": 1.0}, PS, seed=3)
        self.assertEqual(g1.hash(), g2.hash())
        g3 = Genome({"fault0_enable": 1.0}, PS, seed=4)
        self.assertNotEqual(g1.hash(), g3.hash())

    def test_random_reproducible(self):
        a = Genome.random(random.Random(7), PS, seed=0).to_vector()
        b = Genome.random(random.Random(7), PS, seed=0).to_vector()
        self.assertEqual(a, b)


class RenderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="scn_render_")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _render(self, g, sub="o"):
        out = os.path.join(self.tmp, sub)
        render_mod.render(g, "conestogo", out)
        with open(os.path.join(out, "world.yaml")) as f:
            return yaml.safe_load(f), out

    def test_renders_valid_world(self):
        g = Genome.random(random.Random(2), PS, seed=9)
        world, _ = self._render(g)
        self.assertIn("properties", world)
        self.assertEqual(world["properties"]["seed"], 9)
        self.assertEqual(len(world["models"]), 1)
        self.assertTrue(os.path.isabs(world["layers"][0]["map"]))

    def test_fault_entries_match_schema(self):
        g = Genome({"fault0_enable": 1.0, "fault0_type": 9,  # torque_loss
                    "fault0_peak": 0.7, "fault0_magnitude": 0.5}, PS, seed=1)
        world, _ = self._render(g)
        faults = world["plugins"][0]["faults"]
        self.assertGreaterEqual(len(faults), 1)
        f = faults[0]
        for key in ("id", "type", "target", "trigger", "severity"):
            self.assertIn(key, f)
        for key in ("model", "component", "topic"):
            self.assertIn(key, f["target"])
        for key in ("onset_time", "ramp_up", "hold", "ramp_down", "peak", "profile"):
            self.assertIn(key, f["severity"])
        # drive fault component override applied for this template
        self.assertEqual(f["target"]["component"], "cleaner_drive")

    def test_mutation_keys_valid(self):
        g = Genome({"mut_aisle_enable": 1.0, "mut_aisle_delta_m": 0.1,
                    "mut_clutter_enable": 1.0, "mut_clutter_add_min": 2,
                    "mut_clutter_add_span": 3}, PS, seed=1)
        world, _ = self._render(g)
        mut = world["layers"][0]["mutation"]
        self.assertTrue(mut["enabled"])
        self.assertEqual(len(mut["aisle_width"]["dilate_erode_range_m"]), 2)
        self.assertEqual(mut["clutter"]["add_count_range"], [2, 5])

    def test_render_deterministic(self):
        # Same genome+seed -> identical world content (modulo the out_dir-derived
        # sealed-manifest paths, which are deliberately per-run).
        g = Genome.random(random.Random(11), PS, seed=42)
        render_mod.render(g, "conestogo", os.path.join(self.tmp, "a"))
        render_mod.render(g, "conestogo", os.path.join(self.tmp, "b"))

        def _strip_paths(w):
            for layer in w.get("layers", []):
                layer.get("mutation", {}).pop("manifest_path", None)
            for plug in w.get("plugins", []):
                plug.pop("ground_truth_path", None)
            return w

        with open(os.path.join(self.tmp, "a", "world.yaml")) as f:
            a = _strip_paths(yaml.safe_load(f))
        with open(os.path.join(self.tmp, "b", "world.yaml")) as f:
            b = _strip_paths(yaml.safe_load(f))
        self.assertEqual(a, b)
        # And the genome hash (the reproducibility key) is identical.
        with open(os.path.join(self.tmp, "a", "genome.json")) as f:
            ga = json.load(f)
        with open(os.path.join(self.tmp, "b", "genome.json")) as f:
            gb = json.load(f)
        self.assertEqual(ga, gb)

    def test_asset_path_escape_rejected(self):
        bad = os.path.join(self.tmp, "badtpl")
        os.makedirs(bad)
        with open(os.path.join(bad, "template.json"), "w") as f:
            json.dump({"name": "bad", "map": "../../../nope.yaml",
                       "robot_model": "../../../nope.model.yaml",
                       "start_candidates": [[0, 0, 0]],
                       "goal_candidates": [[1, 1]]}, f)
        with self.assertRaises(render_mod.RenderError):
            render_mod.load_template(bad)


class ProposerTest(unittest.TestCase):
    def test_rl_climbs_stub_score(self):
        p = make_proposer("rl", PS, seed=1)
        means = []
        for gen in range(10):
            props = p.propose([], k=8, base_seed=gen * 100)
            scores = []
            for pr in props:
                s = stub_score(pr.genome)["composite_score"]
                p.update(pr.genome, s)
                scores.append(s)
            means.append(sum(scores) / len(scores))
        self.assertGreater(means[-1], means[0] + 0.05,
                           "RL proposer should drive stress upward")

    def test_proposers_emit_valid_bounded_genomes(self):
        for strat in ("random", "rl", "llm", "hybrid"):
            p = make_proposer(strat, PS, seed=2)
            for pr in p.propose([], k=3, target_difficulty=0.5, base_seed=0):
                v = pr.genome.values
                for d in PS["dimensions"]:
                    if d["type"] in ("float", "gate"):
                        self.assertGreaterEqual(v[d["name"]], d["low"])
                        self.assertLessEqual(v[d["name"]], d["high"])

    def test_target_difficulty_bias(self):
        p = make_proposer("random", PS, seed=3)
        props = p.propose([], k=6, target_difficulty=0.4, base_seed=0)
        for pr in props:
            self.assertLess(abs(pr.genome.difficulty() - 0.4), 0.45)


class CurriculumTest(unittest.TestCase):
    def test_target_rises_on_success(self):
        c = CurriculumController(init_difficulty=0.3, window=10, step=0.05)
        rng = random.Random(0)
        for i in range(30):
            c.observe(Genome.random(rng, PS, seed=i),
                      {"composite_score": 0.1,
                       "metrics": {"goal_reached": True}, "invalid": False})
        self.assertGreater(c.target(), 0.3)

    def test_target_falls_on_failure(self):
        c = CurriculumController(init_difficulty=0.7, window=10, step=0.05)
        rng = random.Random(0)
        for i in range(30):
            c.observe(Genome.random(rng, PS, seed=i),
                      {"composite_score": 0.95,
                       "metrics": {"goal_reached": False}, "invalid": False})
        self.assertLess(c.target(), 0.7)

    def test_solvability_regularizer(self):
        self.assertFalse(is_solvable({"invalid": True, "composite_score": 0.2}))
        self.assertFalse(is_solvable({"invalid": False, "composite_score": 0.99,
                                      "metrics": {"goal_reached": False}}))
        self.assertTrue(is_solvable({"invalid": False, "composite_score": 0.6,
                                     "metrics": {"goal_reached": False}}))
        self.assertTrue(is_solved({"composite_score": 0.3,
                                   "metrics": {"goal_reached": False}}))

    def test_archive_and_save_load(self):
        c = CurriculumController(seed=0)
        rng = random.Random(1)
        for i in range(10):
            c.observe(Genome.random(rng, PS, seed=i),
                      {"composite_score": 0.6,
                       "metrics": {"goal_reached": False}, "invalid": False})
        self.assertGreater(len(c.archive), 0)
        self.assertGreaterEqual(len(c.sample_replay(2, PS)), 1)
        tmp = tempfile.mkdtemp(prefix="scn_ckpt_")
        try:
            path = os.path.join(tmp, "state.json")
            c.save(path)
            c2 = CurriculumController(seed=0)
            c2.load(path)
            self.assertEqual(len(c2.archive), len(c.archive))
            self.assertEqual(c2.target(), c.target())
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class DeterminismTest(unittest.TestCase):
    def test_stub_score_deterministic(self):
        g = Genome.random(random.Random(5), PS, seed=21)
        self.assertEqual(stub_score(g)["composite_score"],
                         stub_score(g)["composite_score"])


class OrchestrateSmokeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="scn_orch_")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_loop_runs_and_writes_ledger(self):
        summary = orchestrate.run_curriculum(
            proposer_strategy="rl", generations=3, per_generation=6,
            replay_per_generation=0, ledger_dir=self.tmp, base_seed=1,
            stub=True, resume=False)
        self.assertEqual(len(summary["gen_mean_scores"]), 3)
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "ledger.jsonl")))
        with open(os.path.join(self.tmp, "ledger.jsonl")) as f:
            lines = [l for l in f if l.strip()]
        self.assertEqual(len(lines), 18)
        # stress should not collapse; later generation >= first (loose bound)
        self.assertGreaterEqual(max(summary["gen_mean_scores"]),
                                summary["gen_mean_scores"][0])

    def test_resume(self):
        orchestrate.run_curriculum(
            proposer_strategy="rl", generations=2, per_generation=4,
            replay_per_generation=0, ledger_dir=self.tmp, base_seed=1,
            stub=True, resume=False)
        orchestrate.run_curriculum(
            proposer_strategy="rl", generations=2, per_generation=4,
            replay_per_generation=0, ledger_dir=self.tmp, base_seed=1,
            stub=True, resume=True)
        with open(os.path.join(self.tmp, "ledger.jsonl")) as f:
            lines = [l for l in f if l.strip()]
        self.assertEqual(len(lines), 16)  # 2+2 generations * 4


if __name__ == "__main__":
    unittest.main()
