#!/usr/bin/env python3
"""Deterministic fixtures for the advisor-report statistical contract."""

import pathlib
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import pandas as pd

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from advisor_report_common import load_protocol
from analyze_advisor_report import (block_bootstrap, clopper_pearson_upper, holm,
                                    isolated_pl_summary, longest_true_run,
                                    persistent_fault_summary,
                                    representative_trajectory_rows, validate)


class AdvisorReportContract(unittest.TestCase):
    def test_seed_families_are_disjoint(self):
        protocol = load_protocol(ROOT / "config/advisor_report_experiments.yaml")
        roc = protocol["roc"]
        families = []
        for name in ("calibration_seeds", "nominal_evaluation_seeds",
                     "fault_evaluation_seeds"):
            spec = roc[name]
            families.append(set(range(spec["start"], spec["start"]+spec["count"])))
        for name in ("nominal", "window_ablation", "isolated_pl", "persistent_fault"):
            spec = protocol[name]["seeds"]
            families.append(set(range(spec["start"], spec["start"]+spec["count"])))
        families.extend([{protocol["performance"]["seed"]},
                         {protocol["representative_fault"]["seed"]},
                         {protocol["representative_trajectory"]["seed"]}])
        for left in range(len(families)):
            for right in range(left + 1, len(families)):
                self.assertFalse(families[left] & families[right])

    def test_balanced_isolated_cell_assignment(self):
        protocol = load_protocol(ROOT / "config/advisor_report_experiments.yaml")
        spec = protocol["isolated_pl"]
        anchors = spec["fault"]["anchor_ids"]
        magnitudes = spec["fault"]["magnitudes_m"]
        counts = {(anchor, magnitude): 0 for anchor in anchors for magnitude in magnitudes}
        for offset in range(spec["seeds"]["count"]):
            counts[(anchors[offset % len(anchors)],
                    magnitudes[(offset // len(anchors)) % len(magnitudes)])] += 1
        self.assertEqual(set(counts.values()),
                         {spec["fault"]["seeds_per_anchor_magnitude_cell"]})

    def test_block_bootstrap_is_deterministic(self):
        values = np.asarray([-.02, .01, .03, -.01])
        self.assertEqual(block_bootstrap(values, 1000, 9),
                         block_bootstrap(values, 1000, 9))

    def test_holm_fixture(self):
        result = holm([.001, .02, .8], .05)
        self.assertTrue(result[0]["rejected"])
        self.assertTrue(result[1]["rejected"])
        self.assertFalse(result[2]["rejected"])
        self.assertEqual([item["rank"] for item in result], [1, 2, 3])

    def test_clopper_pearson_upper_fixture(self):
        self.assertAlmostEqual(clopper_pearson_upper(0, 10),
                               1.0 - .05 ** (1.0 / 10.0), places=12)
        self.assertEqual(clopper_pearson_upper(10, 10), 1.0)

    @staticmethod
    def rows() -> pd.DataFrame:
        rows = []
        for history in (0, 200):
            for seed, ratio, availability, h_error, hpl in (
                    (1, .9, "AVAILABLE", .2, .3),
                    (2, 1.1, "AVAILABLE", .4, .3),
                    (3, 1.4, "ALERT", .5, np.nan)):
                rows.append({
                    "seed": seed, "trajectory": "figure_eight",
                    "graph_history": history, "epoch": 300,
                    "timestamp_ns": 1, "input_digest": f"d{seed}",
                    "fault_active": 1, "fault_m": .5, "anchor_id": 1,
                    "fault_scenario": "isolated_single_epoch",
                    "truth_px": 0., "truth_py": 0., "truth_pz": 0.,
                    "est_px": h_error, "est_py": 0., "est_pz": .1,
                    "position_error_m": np.hypot(h_error, .1),
                    "velocity_error_mps": 0., "orientation_error_rad": 0.,
                    "horizontal_error_m": h_error, "vertical_error_m": .1,
                    "cov_xx": .1, "cov_yy": .1, "cov_zz": .1,
                    "position_nees": 1., "conditional_statistic": ratio * 10,
                    "conditional_threshold": 10.,
                    "conditional_passed": availability == "AVAILABLE",
                    "batch_committed": availability == "AVAILABLE",
                    "hpl_m": hpl, "vpl_m": .2 if availability == "AVAILABLE" else np.nan,
                    "finite_pl": availability == "AVAILABLE",
                    "availability": availability, "formal_eligible": True,
                    "measurement_model_valid": True, "core_ms": 1.,
                    "active_values": 1, "active_factors": 1,
                })
        return pd.DataFrame(rows)

    def test_containment_hmi_and_near_boundary_denominators(self):
        protocol = load_protocol(ROOT / "config/advisor_report_experiments.yaml")
        protocol["isolated_pl"]["seeds"]["count"] = 3
        protocol["isolated_pl"]["fault"]["seeds_per_anchor_magnitude_cell"] = 3
        summary, cells = isolated_pl_summary(self.rows(), protocol)
        self.assertEqual(summary["overall"]["finite_pl_denominator"], 4)
        self.assertEqual(summary["overall"]["contained"], 2)
        self.assertEqual(summary["overall"]["hmi_count"], 2)
        self.assertEqual(summary["near_boundary"]["samples"], 4)
        self.assertEqual(set(cells.samples), {3})

    def test_persistent_policy_summary(self):
        frame = self.rows().copy()
        frame["fault_scenario"] = "persistent_to_sequence_end"
        frame["fault_active"] = 1
        summary, sequences = persistent_fault_summary(frame)
        self.assertEqual(summary["rejected_with_finite_pl_violations"], 0)
        self.assertEqual(summary["maximum_continuous_rejection_epochs"], 1)
        self.assertEqual(sorted(sequences.longest_rejection_epochs.unique().tolist()), [0, 1])
        self.assertEqual(longest_true_run([False, True, True, False, True]), 2)

    def test_validation_rejects_available_without_finite_pl(self):
        frame = self.rows().iloc[[0]].copy()
        frame.loc[:, "finite_pl"] = False
        frame.loc[:, "hpl_m"] = np.nan
        with self.assertRaisesRegex(ValueError, "AVAILABLE"):
            validate(frame, "fixture", (300, 300))

    def test_representative_filter_is_sorted_and_isolated(self):
        frame = pd.DataFrame({
            "trajectory": ["circle", "figure_eight", "figure_eight"],
            "graph_history": [0, 0, 0], "epoch": [9, 2, 1]})
        selected = representative_trajectory_rows(frame)
        self.assertEqual(selected.trajectory.unique().tolist(), ["figure_eight"])
        self.assertEqual(selected.epoch.tolist(), [1, 2])

    def test_benchmark_fault_duration_output_range_digest_and_resume(self):
        binary = ROOT.parents[1] / "devel/.private/uwb_imu_pl/lib/uwb_imu_pl/advisor_paired_benchmark"
        if not binary.is_file():
            self.skipTest("advisor_paired_benchmark has not been built")
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "sample.csv"
            command = [str(binary), str(ROOT/"config/realtime_uwb_imu_pl_research.yaml"),
                       str(output), "fault", "20269901", "1", "--epochs", "8",
                       "--histories", "0,2", "--trajectories", "straight",
                       "--fault-onset", "3", "--fault-duration-epochs", "1",
                       "--fault-anchor", "1", "--fault-magnitude", "1",
                       "--output-start-epoch", "2", "--output-end-epoch", "5",
                       "--threads", "1", "--resume"]
            subprocess.run(command, cwd=ROOT, check=True)
            first = pd.read_csv(output)
            subprocess.run(command, cwd=ROOT, check=True)
            second = pd.read_csv(output)
            self.assertEqual(len(first), len(second))
            self.assertEqual(sorted(first.epoch.unique()), [2, 3, 4, 5])
            self.assertEqual(first[first.fault_active == 1].epoch.unique().tolist(), [3])
            self.assertTrue((first.groupby(["seed", "trajectory"])
                             .input_digest.nunique() == 1).all())


if __name__ == "__main__":
    unittest.main()
