#!/usr/bin/env python3
import csv
import pathlib
import tempfile
import unittest
from unittest.mock import patch
from gate_d_diagnostics import SCHEMA, STAGES, validate_attachments
from run_round2_performance import summarize_run, one_run

class DiagnosticsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = pathlib.Path(self.temp.name)
        self.data = {}
        attempts, stages, candidates, legacy, transactions, timing = [], [], [], [], [], []
        for attempt in range(1, 102):
            identity = dict(schema_version=SCHEMA, input_attempt_id=str(attempt),
                            input_timestamp_ns=str(attempt * 100), transaction_id=str(attempt),
                            window_id=str(attempt), graph_version="1", ordering_version="2",
                            noise_model_version="3", linpoint_version="4", output_timestamp_ns="100")
            attempts.append(dict(identity, backend_epoch_before="1", backend_epoch_after="1",
                                 pending_duration_s=".05", raw_imu_samples="10", consecutive_rejections="1",
                                 marginalization_count="0", status="EXECUTED", reason=""))
            for stage in STAGES:
                stages.append(dict(identity, stage=stage, status="EXECUTED", wall_ms="1", reason=""))
            for action in range(128 if attempt == 101 else 1):
                candidates.append(dict(identity, action_id=str(action), kernel_evaluated="1",
                    numerical_valid="0", post_passed="0", pl_evaluated="0", coverage_rejected="1",
                    selected="0", slow_path="0", near_gate="0", recovered_replacement="0",
                    numerical_path="LOW_RANK", fallback_reason="", skip_reason="numerical rejection",
                    cache_hits="0", kernel_ms="1", post_ms=".1", bridge_ms="", fault_map_ms="", pl_ms=""))
                legacy.append(dict(timestamp_ns="100", window_id=str(attempt),
                                   action_id=str(action), removed_group_ids="",
                                   evaluation_wall_ms="1"))
            transactions.append(dict(timestamp_ns="100", transaction_id=str(attempt), window_id=str(attempt), base_graph_version="1", linearization_version="4"))
            timing.append(dict(timestamp_ns="100", epoch="1", stage="core_total", wall_ms="1", cold_warm="cold" if attempt <= 100 else "warm", success="1"))
        for name, rows in (("diagnostic_attempts", attempts), ("diagnostic_stages", stages),
                           ("diagnostic_candidates", candidates), ("candidates", legacy),
                           ("transactions", transactions), ("timing", timing)):
            self.data[name] = rows
        self.save()

    def save(self):
        for name, rows in self.data.items():
            with (self.path / (name + ".csv")).open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader(); writer.writerows(rows)

    def verdict(self):
        self.save()
        return summarize_run(self.path, 101, 4, 128)

    def test_repeated_state_timestamps_and_normal_rejection_are_valid(self):
        self.assertEqual(self.verdict()["status"], "PASS")

    def test_v2_exact_timing_links(self):
        for name in ("diagnostic_attempts", "diagnostic_stages", "diagnostic_candidates"):
            for row in self.data[name]: row["schema_version"] = "uwb-imu-pl/gate-d-diagnostics/v2"
        timing, links = [], []
        for row in self.data["diagnostic_stages"]:
            attempt = int(row["input_attempt_id"])
            timing.append(dict(timestamp_ns="100", epoch="1", stage=row["stage"], wall_ms="1",
                               cold_warm="cold" if attempt <= 100 else "warm", success="1"))
            links.append(dict(timing_row=str(len(timing)), input_attempt_id=str(attempt),
                              transaction_id=str(attempt), window_id=str(attempt), stage=row["stage"]))
        self.data["timing"] = timing
        self.data["diagnostic_timing_links"] = links
        self.assertEqual(self.verdict()["status"], "PASS")
        links[-1]["input_attempt_id"] = "100"
        self.assertEqual(self.verdict()["status"], "INVALID")
        links[-1]["input_attempt_id"] = "101"
        links.pop()
        self.assertEqual(self.verdict()["status"], "INVALID")

    def test_v3_pending_group_must_be_in_actual_frozen_window(self):
        for name in ("diagnostic_attempts", "diagnostic_stages", "diagnostic_candidates"):
            for row in self.data[name]: row["schema_version"] = "uwb-imu-pl/gate-d-diagnostics/v3"
        for row in self.data["diagnostic_attempts"]: row["frozen_group_ids"] = "1;2;3"
        for row in self.data["candidates"]: row["removed_group_ids"] = "3"
        self.save()
        validate_attachments(self.path, 101, expected_candidates=128, require_legacy_timing=False)
        self.data["candidates"][-1]["removed_group_ids"] = "4"
        self.save()
        with self.assertRaisesRegex(ValueError, "outside actual frozen window"):
            validate_attachments(self.path, 101, expected_candidates=128, require_legacy_timing=False)

    def test_v9_and_v10_schema_validation_use_actual_frozen_window(self):
        for version in ("v9", "v10"):
            for name in ("diagnostic_attempts", "diagnostic_stages",
                         "diagnostic_candidates"):
                for row in self.data[name]:
                    row["schema_version"] = (
                        "uwb-imu-pl/gate-d-diagnostics/" + version)
            for row in self.data["diagnostic_attempts"]:
                row["frozen_group_ids"] = "1;2;3"
            for row in self.data["candidates"]:
                row["removed_group_ids"] = "3"
            self.save()
            validate_attachments(self.path, 101, expected_candidates=128,
                                 require_legacy_timing=False)
            self.data["candidates"][-1]["removed_group_ids"] = "4"
            self.save()
            with self.assertRaisesRegex(ValueError,
                                        "outside actual frozen window"):
                validate_attachments(self.path, 101,
                                     expected_candidates=128,
                                     require_legacy_timing=False)
            self.data["candidates"][-1]["removed_group_ids"] = "3"

    def test_reinit_may_reuse_transaction_and_window_ids(self):
        for name in ("diagnostic_attempts", "diagnostic_stages", "diagnostic_candidates"):
            for row in self.data[name]:
                if row["input_attempt_id"] == "101":
                    row["transaction_id"] = row["window_id"] = "1"
        self.data["transactions"][-1]["transaction_id"] = "1"
        self.data["transactions"][-1]["window_id"] = "1"
        for row in self.data["candidates"]:
            if row["window_id"] == "101": row["window_id"] = "1"
        self.assertEqual(self.verdict()["status"], "PASS")

    def test_missing_rows(self):
        for name in self.data:
            with self.subTest(name=name):
                row = self.data[name].pop()
                self.assertEqual(self.verdict()["status"], "INVALID")
                self.data[name].append(row)

    def test_duplicate_rows(self):
        for name in self.data:
            with self.subTest(name=name):
                self.data[name].append(self.data[name][-1].copy())
                self.assertEqual(self.verdict()["status"], "INVALID")
                self.data[name].pop()

    def test_127_and_129_unique_kernels(self):
        removed = [self.data[name].pop() for name in ("diagnostic_candidates", "candidates")]
        self.assertEqual(self.verdict()["status"], "INVALID")
        for name, row in zip(("diagnostic_candidates", "candidates"), removed):
            self.data[name].append(row)
            self.data[name].append(dict(row, action_id="128"))
        self.assertEqual(self.verdict()["status"], "INVALID")

    def test_nonfinite_and_negative_durations(self):
        row = self.data["diagnostic_candidates"][-1]
        for value in ("nan", "inf", "-1"):
            row["kernel_ms"] = value
            self.assertEqual(self.verdict()["status"], "INVALID")

    def test_wrong_version_and_input_identity(self):
        row = self.data["diagnostic_candidates"][-1]
        for field in ("graph_version", "ordering_version", "noise_model_version", "linpoint_version", "window_id", "transaction_id", "input_timestamp_ns"):
            original = row[field]; row[field] = "999"
            self.assertEqual(self.verdict()["status"], "INVALID")
            row[field] = original

    def test_kernel_not_executed(self):
        self.data["diagnostic_candidates"][-1]["kernel_evaluated"] = "0"
        self.assertEqual(self.verdict()["status"], "INVALID")

    def test_v8_and_v9_exact_eligibility_skip_have_no_fabricated_numerics(self):
        for version in ("v8", "v9"):
            with self.subTest(version=version):
                row = self.data["diagnostic_candidates"][-1]
                for name in ("diagnostic_attempts", "diagnostic_stages",
                             "diagnostic_candidates"):
                    for item in self.data[name]:
                        item["schema_version"] = (
                            "uwb-imu-pl/gate-d-diagnostics/" + version)
                for item in self.data["diagnostic_attempts"]:
                    item["frozen_group_ids"] = ""
                row.update(kernel_evaluated="0", numerical_valid="0",
                           pl_evaluated="0", kernel_ms="", post_ms="",
                           bridge_ms="", fault_map_ms="", pl_ms="",
                           skip_reason="SKIPPED_INELIGIBLE: coverage gap")
                self.assertEqual(self.verdict()["status"], "PASS")
                row["kernel_ms"] = "1"
                self.assertEqual(self.verdict()["status"], "INVALID")
                row["kernel_ms"] = ""

    def test_process_failure_and_empty_output_have_complete_summary(self):
        result = summarize_run(self.path, 101, 4, 128, 2)
        self.assertEqual(result["status"], "INVALID")
        self.assertIsNone(result["core_total"]["p99_ms"])
        with tempfile.TemporaryDirectory() as empty:
            result = summarize_run(pathlib.Path(empty), 101, 4, 128)
            self.assertEqual(result["status"], "INVALID")
            self.assertFalse(result["candidate_count"]["contract_met"])
        with patch("run_round2_performance.subprocess.run", side_effect=OSError("missing executable")):
            self.assertEqual(one_run(self.path, 101, 4, "0", 128)["status"], "INVALID")

if __name__ == "__main__":
    unittest.main()
