#!/usr/bin/env python3

import csv
import hashlib
import json
import os
import pathlib
import subprocess
import tempfile
import unittest

from round2_common import (CounterRng, boundary_amplitude, enumerate_cells,
                           load_protocol, metric_classification, scenario_id,
                           seed_for)

ROOT = pathlib.Path(__file__).resolve().parents[1]
PROTOCOL = ROOT / "config/integrity_round2_protocol.json"
TEST_BIN_DIR = pathlib.Path(os.environ.get(
    "UWB_IMU_PL_TEST_BIN_DIR",
    ROOT.parent.parent /
    "devel/.private/uwb_imu_pl/lib/uwb_imu_pl"))


class RoundTwoProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.protocol, cls.digest = load_protocol(PROTOCOL)

    def test_inventory_is_canonical_complete_and_unique(self):
        cells = list(enumerate_cells(self.protocol))
        self.assertEqual(len(cells), 5543)
        self.assertEqual(len(cells), len({cell["id"] for cell in cells}))
        self.assertEqual(scenario_id("E", {"ratio": 2, "geometry": "normal8"}),
                         scenario_id("E", {"geometry": "normal8", "ratio": 2}))

    def test_counter_rng_and_seed_domains_replay(self):
        left = CounterRng(11, "test", "cell")
        right = CounterRng(11, "test", "cell")
        self.assertEqual([left.uint64(i) for i in range(20)],
                         [right.uint64(i) for i in range(20)])
        values = {name: seed_for(self.protocol, name, "cell", 7)
                  for name in self.protocol["seed_domains"]}
        self.assertEqual(len(values), len(set(values.values())))
        per_cell = [seed_for(self.protocol, "test", "cell", ordinal)
                    for ordinal in range(4096)]
        self.assertEqual(len(per_cell), len(set(per_cell)))

    def test_boundary_uses_directional_gram_and_ramp_direction(self):
        self.assertAlmostEqual(boundary_amplitude(
            9.0, [[100.0, 0.0], [0.0, 4.0]], [0.0, 1.0], 2.0), 3.0)
        self.assertNotEqual(boundary_amplitude(
            9.0, [[100.0, 0.0], [0.0, 4.0]], [1.0, 0.0], 2.0), 3.0)

    def test_metric_categories_are_mutually_exclusive(self):
        cases = {
            "nominal_keep": (set(), set(), set(), True),
            "nominal_false_exclusion": (set(), {"x"}, set(), True),
            "wrong_exclusion": ({"x"}, {"y"}, {"x"}, False),
            "correct_exclusion": ({"x"}, {"x"}, {"x"}, False),
            "ambiguous_exclusion": ({"x"}, {"x", "y"}, {"y"}, False),
            "union_exclusion": ({"x", "z"}, {"x", "z"}, {"y"}, False),
        }
        for expected, arguments in cases.items():
            self.assertEqual(metric_classification(*arguments), expected)

    def test_prepare_is_resumable_and_never_claims_pass(self):
        with tempfile.TemporaryDirectory() as temporary:
            command = ["python3", str(ROOT / "tools/run_integrity_round2.py"),
                       "prepare", "--root", temporary, "--limit-cells", "1"]
            subprocess.run(command, check=True, stdout=subprocess.PIPE, text=True)
            directories = list(pathlib.Path(temporary).iterdir())
            self.assertEqual(len(directories), 1)
            state = json.loads((directories[0] / "campaign_state.json").read_text())
            self.assertEqual(state["profile"], "SMOKE")
            self.assertEqual(state["status"], "INVALID")
            subprocess.run(command, check=True, stdout=subprocess.PIPE, text=True)
            out_of_order = ["python3", str(ROOT / "tools/run_integrity_round2.py"),
                            "calibrate", "--root", temporary, "--limit-cells", "1"]
            completed = subprocess.run(out_of_order, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
            self.assertEqual(completed.returncode, 2)
            self.assertIn("cannot follow build=PENDING", completed.stderr)

    def test_cpp_raw_stream_runner_replays_short_fixture(self):
        executable = TEST_BIN_DIR / "integrity_round2_scenario"
        if not executable.is_file():
            self.skipTest("C++ scenario runner is not built")
        cell = json.dumps({"gate": "E", "id": "unit-replay",
                           "geometry": "normal8", "fault_mode": "nominal",
                           "ratio": 0.0, "estimator_mode": "full_history",
                           "onset_epoch": 2, "epochs": 3}, sort_keys=True,
                          separators=(",", ":"))
        command = [str(executable),
                   str(ROOT / "config/realtime_uwb_imu_pl_research.yaml"),
                   cell, "5000007"]
        left = subprocess.run(command, check=True, text=True,
                              capture_output=True).stdout
        right = subprocess.run(command, check=True, text=True,
                               capture_output=True).stdout
        self.assertEqual(left, right)
        self.assertEqual(json.loads(left)["status"], "PASS")

    def test_v6_run_validator_checks_inventory_and_protocol_digest(self):
        executable = TEST_BIN_DIR / "realtime_performance_benchmark"
        if not executable.is_file():
            self.skipTest("performance runner is not built")
        with tempfile.TemporaryDirectory() as temporary:
            run = pathlib.Path(temporary) / "run"
            environment = os.environ.copy()
            library_paths = [TEST_BIN_DIR.parent,
                             ROOT.parent.parent / "devel/.private/uwb_imu_pl/lib",
                             ROOT.parent.parent / "devel/lib",
                             pathlib.Path("/opt/ros/noetic/lib")]
            environment["LD_LIBRARY_PATH"] = ":".join(map(str, library_paths)) + (
                ":" + environment["LD_LIBRARY_PATH"]
                if environment.get("LD_LIBRARY_PATH") else "")
            subprocess.run([str(executable),
                            str(ROOT / "config/fde_joint_order2.yaml"),
                            str(run), "2"], check=True, capture_output=True,
                           text=True, env=environment)
            manifest_path = run / "run_manifest.json"
            manifest = json.loads(manifest_path.read_text())
            protocol_copy = run / "protocol.json"
            protocol_copy.write_bytes(PROTOCOL.read_bytes())
            manifest["protocol_path"] = protocol_copy.name
            manifest["protocol_sha256"] = hashlib.sha256(
                protocol_copy.read_bytes()).hexdigest()
            manifest["failure_catalog_path"] = "failure_catalog.json"
            (run / "failure_catalog.json").write_text("{}\n")
            artifact = run / "states.csv"
            with artifact.open(newline="") as stream:
                row_count = sum(1 for _ in csv.DictReader(stream))
            raw_inventory = {"artifacts": [{"path": artifact.name,
                "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
                "rows": row_count}]}
            inventory_path = run / "raw_inventory.json"
            inventory_path.write_text(json.dumps(raw_inventory) + "\n")
            manifest["raw_inventory_path"] = inventory_path.name
            manifest["raw_inventory_sha256"] = hashlib.sha256(
                inventory_path.read_bytes()).hexdigest()
            manifest["artifact_checksum_path"] = "checksums.sha256"
            checksum = hashlib.sha256(protocol_copy.read_bytes()).hexdigest()
            (run / "checksums.sha256").write_text(
                f"{checksum}  {protocol_copy.name}\n")
            manifest_path.write_text(json.dumps(manifest) + "\n")
            validator = ["python3", str(ROOT / "tools/validate_run_schema.py"),
                         str(run)]
            subprocess.run(validator, check=True, capture_output=True, text=True)
            protocol_copy.write_text("{}\n")
            failed = subprocess.run(validator, capture_output=True, text=True)
            self.assertEqual(failed.returncode, 1)
            self.assertIn("protocol artifact", failed.stderr)


if __name__ == "__main__":
    unittest.main()
