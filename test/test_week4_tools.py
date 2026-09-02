#!/usr/bin/env python3

import argparse
import csv
import json
import pathlib
import tempfile
import types
import unittest

import numpy as np

from analyze_detector_roc import block_bootstrap_pfa
from run_imu_bias_monte_carlo import analyze as analyze_imu
from run_week4_validation import Runner
from summarize_snapshot_sweep import read_and_validate
from week4_common import (INVALID, InvalidArtifact, clopper_pearson,
                          condition_tolerance, holm, load_protocol,
                          sequence_rolling)

ROOT = pathlib.Path(__file__).resolve().parents[1]
PROTOCOL = ROOT / "config/week4_validation.yaml"


def consume(generator):
    rows = []
    try:
        while True:
            rows.append(next(generator))
    except StopIteration as stop:
        return rows, stop.value


class StatisticsTest(unittest.TestCase):
    def test_exact_interval_and_holm_fixture(self):
        low, high = clopper_pearson(5, 100)
        self.assertLess(low, .05)
        self.assertGreater(high, .05)
        corrected = holm([.001, .02, .2], .05)
        self.assertEqual([item["rejected"] for item in corrected],
                         [True, True, False])

    def test_condition_aware_tolerance(self):
        self.assertEqual(condition_tolerance(1), 1e-9)
        self.assertLessEqual(condition_tolerance(1e12), 1e-3)
        with self.assertRaises(InvalidArtifact):
            condition_tolerance(float("inf"))


class SnapshotCompletenessTest(unittest.TestCase):
    fields = ["scenario_id", "trajectory", "geometry", "anchor_count",
              "geometry_scale", "sigma", "fault_anchor", "fault_m", "p_fa",
              "p_md", "seed", "derived_seed", "epoch", "axis_covered",
              "operational_hmi", "availability", "detector_passed", "model_valid",
              "signed_pe_x", "signed_pe_y", "signed_pe_z", "pred_var_x",
              "pred_var_y", "pred_var_z"]

    def spec(self):
        return {"seeds": {"start": 17, "stop": 17},
                "trajectories": ["straight"], "anchor_counts": [1],
                "geometries": {"regular": {"geometry_scales": [1.0]}},
                "noise_sigmas_m": [0.1],
                "fault": {"magnitudes_m": [0.5]}, "epochs_per_job": 2,
                "expected_rows": 4}

    def write(self, path, duplicate=False, omit=False):
        data = []
        for anchor, magnitude in ((-1, 0.0), (0, .5)):
            for epoch in range(2):
                data.append({"scenario_id": "x", "trajectory": "straight",
                    "geometry": "regular", "anchor_count": 1,
                    "geometry_scale": 1.0, "sigma": .1,
                    "fault_anchor": anchor, "fault_m": magnitude,
                    "p_fa": 1e-5, "p_md": 1e-3, "seed": 17,
                    "derived_seed": 1, "epoch": epoch, "axis_covered": 1,
                    "operational_hmi": 0, "availability": "Available",
                    "detector_passed": 1, "model_valid": 1,
                    "signed_pe_x": 0, "signed_pe_y": 0, "signed_pe_z": 0,
                    "pred_var_x": 1, "pred_var_y": 1, "pred_var_z": 1})
        if omit: data.pop()
        if duplicate: data.append(dict(data[0]))
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.fields)
            writer.writeheader(); writer.writerows(data)

    def test_complete_missing_and_duplicate(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = pathlib.Path(temporary)/"shard.csv"
            self.write(path)
            _, result = consume(read_and_validate([path], self.spec()))
            self.assertEqual(result["status"], "PASS")
            self.write(path, omit=True)
            _, result = consume(read_and_validate([path], self.spec()))
            self.assertEqual(result["status"], INVALID)
            self.write(path, duplicate=True)
            _, result = consume(read_and_validate([path], self.spec()))
            self.assertEqual(result["status"], INVALID)


class RocIsolationTest(unittest.TestCase):
    def test_window_never_crosses_seed_trajectory(self):
        rows = [
            {"seed": 1, "trajectory": "a", "epoch": 0, "x": 1},
            {"seed": 1, "trajectory": "a", "epoch": 1, "x": 2},
            {"seed": 2, "trajectory": "a", "epoch": 0, "x": 100},
            {"seed": 2, "trajectory": "a", "epoch": 1, "x": 200},
        ]
        values = [value for _, value in sequence_rolling(rows, "x", 2)]
        self.assertTrue(np.isnan(values[0]))
        self.assertEqual(values[1], 3)
        self.assertTrue(np.isnan(values[2]))
        self.assertEqual(values[3], 300)

    def test_block_bootstrap_is_deterministic(self):
        blocks = [[False]*99+[True], [False]*100, [False]*98+[True, True]]
        left = block_bootstrap_pfa(blocks, .01, 100,
                                   np.random.default_rng(23))
        right = block_bootstrap_pfa(blocks, .01, 100,
                                    np.random.default_rng(23))
        self.assertEqual(left, right)

    def test_protocol_seed_sets_are_disjoint(self):
        protocol = load_protocol(PROTOCOL)["roc"]
        sets = []
        for name in ("calibration_seeds", "nominal_evaluation_seeds",
                     "fault_evaluation_seeds"):
            item = protocol[name]
            sets.append(set(range(item["start"], item["start"]+item["count"])))
        self.assertFalse(sets[0] & sets[1] or sets[0] & sets[2] or sets[1] & sets[2])


class ImuAndResumeTest(unittest.TestCase):
    def test_small_imu_fixture_is_deterministic(self):
        protocol = load_protocol(PROTOCOL)
        left = analyze_imu(ROOT/"config/realtime_uwb_imu_pl_research.yaml",
                           protocol, 500)
        right = analyze_imu(ROOT/"config/realtime_uwb_imu_pl_research.yaml",
                            protocol, 500)
        self.assertEqual(left, right)
        self.assertTrue(left["checks"]["jacobian"])

    def test_invalid_attempt_resumes_same_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            args = types.SimpleNamespace(protocol=PROTOCOL,
                results_root=pathlib.Path(temporary), formal=False,
                bin_dir=None, threads=1)
            runner = Runner(args)
            runner.prepare()
            directory, record = runner.attempt("noncentral", False)
            (directory/"gate_result.json").write_text('{"status":"INVALID"}\n')
            runner.finish_attempt(record, {"status": "INVALID"})
            resumed, resumed_record = runner.attempt("noncentral", True)
            self.assertEqual(directory, resumed)
            self.assertEqual(record["attempt"], resumed_record["attempt"])


if __name__ == "__main__":
    unittest.main()
