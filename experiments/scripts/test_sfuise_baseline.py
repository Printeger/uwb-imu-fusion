#!/usr/bin/env python3
"""Engineering tests for the GT-isolated SFUISE ToA adapter."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[2]
SFUISE = ROOT / "experiments/compare_algorithm/SFUISE"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SfuiseBaselineTest(unittest.TestCase):
    def test_all_frozen_configs_are_absolute_toa(self):
        for sequence in (1, 2, 3):
            path = SFUISE / f"sfuise/config/config_test_isas-walk{sequence}.yaml"
            config = yaml.safe_load(path.read_text(encoding="utf-8"))
            self.assertIs(config["if_tdoa"], False)
            self.assertEqual(config["topic_uwb"], "/rtls_flares")

    def test_adapter_has_no_gt_subscription_or_oracle_input(self):
        path = ROOT / "experiments/compare_algorithm/sfuise_adapter_ros/src/sfuise_trajectory_adapter.cpp"
        source = path.read_text(encoding="utf-8")
        self.assertNotIn("/vive/", source)
        self.assertNotIn("ground_truth", source.lower())
        self.assertNotIn("detector", source.lower())
        self.assertNotIn("recovery", source.lower())
        self.assertIn('subscribe("/EstimationInterface/toa_ds"', source)
        self.assertIn('subscribe("/SplineFusion/est_window"', source)

    def test_runner_never_plays_gt_and_blocks_native_gt_export_path(self):
        path = ROOT / "experiments/scripts/run_sfuise_toa.py"
        source = path.read_text(encoding="utf-8")
        self.assertIn('"gt_topic_played": False', source)
        self.assertIn("interface_calib_blocked", source)
        self.assertIn('cfg["topic_imu"], cfg["topic_uwb"], cfg["topic_anchor_list"]', source)
        self.assertNotIn('cfg["topic_ground_truth"]', source)

    def test_unified_trajectory_validation_and_interval_clip(self):
        evaluator = load("sfuise_comparison_test", ROOT / "experiments/scripts/evaluate_sfuise_baseline.py")
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "input.tum"
            target = Path(directory) / "output" / "trajectory.tum"
            source.write_text(
                "1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n3 2 0 0 0 0 0 1\n",
                encoding="utf-8")
            self.assertEqual(evaluator.validate_and_clip(source, target, 1.5, 2.5), 1)
            self.assertEqual(target.read_text(encoding="utf-8").split()[0], "2")

    def test_tdoa_trajectory_is_not_a_supported_method(self):
        evaluator = load("sfuise_comparison_methods", ROOT / "experiments/scripts/evaluate_sfuise_baseline.py")
        self.assertNotIn("tdoa", {entry[1].lower() for entry in evaluator.METHODS})


if __name__ == "__main__":
    unittest.main()
