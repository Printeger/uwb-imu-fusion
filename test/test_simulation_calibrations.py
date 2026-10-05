#!/usr/bin/env python3

import json
import csv
import hashlib
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import yaml
import summarize_p1_06_simulation_acceptance as acceptance_summary
from validate_simulation_calibrations import ROOT, validate
from analyze_fde_profile_runs import analyze_run, rows
from compare_p1_04_outputs import compare_directories
from summarize_p1_06_simulation_acceptance import (
    acceptance_path, distribution, expected_run_identity, fgo_gates, identity_failures,
    indexed, proof_partition,
    publication_failures, revised_nonrealtime_pass, validate_gate24)
from run_fde_profile_validation import (
    GATE24_GOLDEN_ANCHOR, _resolve_golden_reference_manifest,
    canonical_golden_manifest_sha256, golden_reference_manifest,
    immutable_identity, resume_reusable, terminal_artifacts_match)


MANIFEST = ROOT / "config/p1_06_simulation_calibrations.json"


class SimulationCalibrationTest(unittest.TestCase):
    def _gate24_fixture(self, root):
        gate = root / "gate24"
        gate.mkdir(parents=True)
        golden = (ROOT / "docs/evidence/p1-05-deterministic-concurrency/"
                  "archived-raw/candidate-3x45")
        runs = []
        for index in (1, 2, 3):
            before = golden / f"run{index}"
            after = gate / f"run{index}"
            after.mkdir()
            files = {}
            for name in (
                    "bridge.csv", "candidates.csv", "diagnostic_attempts.csv",
                    "diagnostic_candidates.csv", "diagnostic_coverage.csv",
                    "diagnostic_history_summary.csv",
                    "diagnostic_snapshot_identity.csv",
                    "diagnostic_square_root.csv", "diagnostic_state_steps.csv",
                    "hypotheses.csv", "integrity.csv", "states.csv",
                    "transactions.csv"):
                shutil.copy2(before / name, after / name)
                with (after / name).open(newline="", encoding="utf-8") as stream:
                    row_count = sum(1 for _ in csv.DictReader(stream))
                files[name] = {
                    "status": "PASS", "mismatches": [], "rows": row_count,
                    "before_sha256": hashlib.sha256(
                        (before / name).read_bytes()).hexdigest(),
                    "after_sha256": hashlib.sha256(
                        (after / name).read_bytes()).hexdigest()}
            with (after / "diagnostic_attempts.csv").open(newline="") as stream:
                attempts = list(csv.DictReader(stream))
            with (after / "integrity.csv").open(newline="") as stream:
                integrity = list(csv.DictReader(stream))
            core = [float(value) for value in range(1, 46)]
            analysis = [value + .1 for value in core]
            arrival = [value + .2 for value in core]
            with (after / "timing.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(("timestamp_ns", "epoch", "stage", "wall_ms",
                                 "problem_size", "hypothesis_count",
                                 "factor_count", "cold_warm", "success"))
                for attempt_id in range(1, 46):
                    for stage, value in (("core_total", core[attempt_id - 1]),
                                         ("analysis_completion",
                                          analysis[attempt_id - 1]),
                                         ("arrival_to_publish",
                                          arrival[attempt_id - 1]),
                                         ("outer_epoch",
                                          arrival[attempt_id - 1] + .1)):
                        writer.writerow((attempt_id, attempt_id, stage, value,
                                         1, 1, 1, "warm", 1))
            (gate / f"run{index}.time").write_text(
                "Maximum resident set size (kbytes): 100\n")
            outcomes = []
            for attempt_id, (diag, integ) in enumerate(zip(attempts, integrity), 1):
                outcomes.append({
                    "attempt": attempt_id, "complete_work": True,
                    "deadline_missed": integ["deadline_missed"] == "1",
                    "timeout": diag["watchdog_wall_timeout"] == "1",
                    "sensor_stale": diag["watchdog_sensor_stale"] == "1",
                    "watchdog_reason": diag["watchdog_reason"],
                    "status": diag["status"], "terminal_reason": diag["reason"],
                    "transaction_opened": diag["transaction_opened"] == "1"})
            deadline = sum(item["deadline_missed"] for item in outcomes)
            runs.append({
                "run_directory": str(after), "attempted": 45, "excluded": 0,
                "complete_work_attempts": 45, "complete_work_rate": 1.0,
                "deadline_misses": deadline,
                "deadline_miss_rate": deadline / 45, "rss_peak_kib": 100,
                "core_compute": distribution(core),
                "analysis_completion": distribution(analysis),
                "arrival_to_publish": distribution(arrival),
                "raw_core_ms": core, "raw_analysis_completion_ms": analysis,
                "raw_arrival_to_publish_ms": arrival,
                "raw_attempt_outcomes": outcomes})
            (gate / f"equivalence-run{index}.json").write_text(json.dumps(
                compare_directories(before, after, gate24_audit=True)))
        all_core = [x for run in runs for x in run["raw_core_ms"]]
        all_analysis = [x for run in runs
                        for x in run["raw_analysis_completion_ms"]]
        all_arrival = [x for run in runs
                       for x in run["raw_arrival_to_publish_ms"]]
        deadline = sum(run["deadline_misses"] for run in runs)
        aggregate = {"attempted": 135, "excluded": 0,
                     "complete_work_attempts": 135, "complete_work_rate": 1.0,
                     "deadline_misses": deadline,
                     "deadline_miss_rate": deadline / 135,
                     "rss_peak_kib": 100,
                     "core_compute": distribution(all_core),
                     "analysis_completion": distribution(all_analysis),
                     "arrival_to_publish": distribution(all_arrival)}
        performance = gate / "all-attempt-performance.json"
        performance.write_text(json.dumps({"groups": [{"runs": runs,
                                                        "aggregate": aggregate}]}))
        anchor = GATE24_GOLDEN_ANCHOR
        (gate / "gate24_run_identity.json").write_text(json.dumps({
            "golden_reference": anchor["reference"],
            "golden_annotated_tag_object": anchor["annotated_tag_object"],
            "golden_peeled_commit": anchor["peeled_commit"],
            "golden_tree": anchor["tree"],
            "golden_manifest_sha256": anchor["manifest_sha256"],
            "golden_anchor_schema": anchor["schema"],
        }))
        (gate / "golden_reference_manifest.json").write_text(json.dumps(
            golden_reference_manifest("golden-p1-05-deterministic-concurrency")))
        return gate, performance

    def _fgo_fixture(self, root):
        protocol = yaml.safe_load(
            (ROOT / "config/fde_profiles_validation.yaml").read_text())
        calibrations = validate(MANIFEST)
        calibration = next(item for item in calibrations["profiles"]
                           if item["profile"] == "off")
        epochs = int(protocol["epochs"]["fgo"])
        seed = int(protocol["input"]["seeds"][0])
        runs = []
        for scenario in protocol["scenarios"]["fgo"]:
            run_id = f"off__{scenario}__e{epochs}__s{seed}__r1"
            run = root / run_id
            run.mkdir(parents=True)
            identity = expected_run_identity(
                run_id, "off", scenario, epochs, seed, protocol, calibration)
            identity.update({"gate": "fgo", "scale": "full", "exit_code": 0,
                             "runner_wall_s": 1.0, "terminal_artifacts": {}})
            (run / "validation_run.json").write_text(json.dumps(identity))
            state_fields = ("timestamp_ns", "state_id", "px", "py", "pz",
                            "qw", "qx", "qy", "qz", "vx", "vy", "vz",
                            "bax", "bay", "baz", "bgx", "bgy", "bgz")
            truth_fields = ("timestamp_ns", "px", "py", "pz", "qw", "qx",
                            "qy", "qz")
            integrity_fields = ("timestamp_ns", "attempted_timestamp_ns",
                                "state_timestamp_ns", "batch_committed",
                                "state_valid", "fresh", "deadline_missed",
                                "pl_status", "within_alert_limits", "hpl_m",
                                "vpl_m", "pl_x", "pl_y", "pl_z", "fde_status",
                                "selected_action_type")
            attempt_fields = ("input_attempt_id", "input_timestamp_ns",
                              "output_timestamp_ns", "marginalization_count",
                              "single_uwb_hypotheses",
                              "single_accel_hypotheses",
                              "single_gyro_hypotheses",
                              "double_uwb_accel_hypotheses",
                              "double_uwb_gyro_hypotheses", "generated_actions")
            terminal_fields = ("input_attempt_id",)
            for name, fields in (("states.csv", state_fields),
                                 ("ground_truth.csv", truth_fields),
                                 ("integrity.csv", integrity_fields),
                                 ("diagnostic_attempts.csv", attempt_fields),
                                 ("terminal_packets.csv", terminal_fields)):
                with (run / name).open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader()
                    for attempt in range(1, epochs + 1):
                        timestamp = attempt * 50_000_000
                        if name == "states.csv":
                            row = dict.fromkeys(fields, 0)
                            row.update(timestamp_ns=timestamp, state_id=attempt,
                                       qw=1)
                        elif name == "ground_truth.csv":
                            row = dict.fromkeys(fields, 0)
                            row.update(timestamp_ns=timestamp, qw=1)
                        elif name == "integrity.csv":
                            row = dict.fromkeys(fields, 0)
                            row.update(timestamp_ns=timestamp,
                                       attempted_timestamp_ns=timestamp,
                                       state_timestamp_ns=timestamp,
                                       batch_committed=1, state_valid=1, fresh=1,
                                       pl_status="NOT_COMPUTED",
                                       fde_status="FDE_DISABLED",
                                       selected_action_type="KEEP_ALL")
                        elif name == "diagnostic_attempts.csv":
                            row = dict.fromkeys(fields, 0)
                            row.update(input_attempt_id=attempt,
                                       input_timestamp_ns=timestamp,
                                       output_timestamp_ns=timestamp)
                        else:
                            row = {"input_attempt_id": attempt}
                        writer.writerow(row)
            item = analyze_run(run)
            item.update(identity)
            runs.append(item)
        return {"runs": runs}, protocol, calibrations

    def test_five_profile_bindings_are_current_and_synthetic_only(self):
        result = validate(MANIFEST)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(len(result["profiles"]), 5)
        self.assertEqual(result["artifact_class"], "simulation/synthetic")
        self.assertEqual(result["hardware_formal_qualification"],
                         "CLOSED/NOT_CLAIMED")

    def test_digest_mutation_fails_closed(self):
        payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
        payload["profiles"][0]["runtime_config"]["sha256"] = "0" * 64
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mutated.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "stale runtime_config sha256"):
                validate(path)

    def test_hardware_claim_mutation_fails_closed(self):
        payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
        payload["qualification_boundary"]["production_formal_eligible_expected"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mutated.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "qualification boundary changed"):
                validate(path)

    def test_simulation_model_version_mutation_fails_closed(self):
        payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
        payload["simulation_model"]["version"] = "unreviewed-model"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mutated.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unexpected simulation_model version"):
                validate(path)

    def test_simulation_model_source_mutation_fails_closed(self):
        payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
        payload["simulation_model"]["source"]["sha256"] = "0" * 64
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mutated.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "stale simulation_model source sha256"):
                validate(path)

    def test_complete_proof_identity_csv_field_is_not_truncated(self):
        proof_identity = "x" * 200_000
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "proof.csv"
            path.write_text("proof_identity\n" + proof_identity + "\n",
                            encoding="utf-8")
            self.assertEqual(rows(path), [{"proof_identity": proof_identity}])

    def test_acceptance_path_uses_attempt_work_not_scenario_label(self):
        complete = {"window_id": "7", "generated_actions": "1",
                    "kernel_evaluated_actions": "1"}
        keep_all = [{"window_id": "7", "action_id": "3", "cardinality": "0",
                     "action_type": "ACTION_SEARCH_OCCURRENCE_V1",
                     "removed_group_ids": "", "added_group_ids": "",
                     "physical_source_ids": "KEEP_ALL",
                     "model_error_record":
                         "operation_identity=ACTION_OPERATION_V1|0;0;0;0;0;0;0;0:"}]
        self.assertEqual(acceptance_path({}, []), "unclassified")
        self.assertEqual(acceptance_path({"generated_actions": "0"}, []),
                         "unclassified")
        self.assertEqual(acceptance_path(
            {"generated_actions": "0", "kernel_evaluated_actions": "0"},
            [], {"fde_status": "FDE_DISABLED"}), "fde_disabled")
        self.assertEqual(acceptance_path(complete, keep_all), "normal")
        # Post/PL failure must stay in the normal realtime denominator.
        incomplete = dict(complete, post_passed_actions="0",
                          pl_evaluated_actions="0")
        self.assertEqual(acceptance_path(incomplete, keep_all), "normal")
        self.assertEqual(acceptance_path({"generated_actions": "2"}, []),
                         "unclassified")

    def test_only_exact_partition_is_accepted(self):
        exact = {"coverage_status": "COMPLETE", "coverage_exact_leaves": "100",
                 "coverage_enveloped_leaves": "0",
                 "coverage_uncovered_leaves": "0",
                 "coverage_proof_count": "0", "coverage_envelope_count": "0"}
        self.assertTrue(proof_partition(exact)["valid"])
        for key, value in (("coverage_uncovered_leaves", "1"),
                           ("coverage_enveloped_leaves", "1"),
                           ("coverage_proof_count", "1")):
            mutated = dict(exact, **{key: value})
            self.assertFalse(proof_partition(mutated)["valid"])
        self.assertFalse(proof_partition("reason", 2, 50, 2)["valid"])

    def test_metadata_failure_blocks_revised_acceptance(self):
        profiles = {"failed_runs": 0, "missing_attempts": 0,
                    "complete_work_rate_over_planned": 1.0,
                    "metadata_failures": ["mutated"]}
        passed = {"status": "PASS"}
        self.assertFalse(revised_nonrealtime_pass(
            profiles, True, True, passed, passed, passed))

    def test_identity_binding_rejects_override_and_digest_mutations(self):
        expected = {"profile": "off", "fault_overrides": {},
                    "simulation_calibration_id": "simcal",
                    "git_sha": "c" * 40, "source_tree_sha256": "d" * 64,
                    "benchmark_source_sha256": "e" * 64,
                    "benchmark_sha256": "f" * 64,
                    "loaded_dso_sha256": {"lib": "1" * 64}}
        identity = dict(expected, protocol_hash="a" * 64,
                        config_hash="b" * 64, exit_code=0,
                        runner_wall_s=1.0)
        self.assertEqual(identity_failures(identity, expected), [])
        for key, value in (("fault_overrides", {"UWB_IMU_PL_UWB_BIAS_M": "2"}),
                           ("simulation_calibration_id", "other"),
                           ("source_tree_sha256", "0" * 64),
                           ("benchmark_sha256", "0" * 64),
                           ("loaded_dso_sha256", {"lib": "0" * 64})):
            mutated = dict(identity, **{key: value})
            self.assertTrue(identity_failures(mutated, expected), key)

    def test_resume_ignores_only_completion_fields(self):
        before = {"run_id": "r", "profile": "off", "exit_code": 0,
                  "runner_wall_s": 1.5, "terminal_artifacts": {"a": 1}}
        self.assertEqual(immutable_identity(before),
                         {"run_id": "r", "profile": "off"})
        mutated = dict(before, profile="joint_order2")
        self.assertNotEqual(immutable_identity(before),
                            immutable_identity(mutated))

    def test_resume_terminal_manifest_rejects_missing_or_mutated_raw(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertFalse(terminal_artifacts_match(root, {}))

    def test_resume_rejects_identity_change_and_old_nonzero_exit(self):
        expected = {"run_id": "r", "source_tree_sha256": "a" * 64}
        old = dict(expected, exit_code=0, runner_wall_s=1.0,
                   terminal_artifacts={})
        frozen = {"benchmark_sha256": "b" * 64}
        with tempfile.TemporaryDirectory() as directory, patch(
                "run_fde_profile_validation.terminal_artifacts_match",
                return_value=True):
            run = Path(directory)
            self.assertTrue(resume_reusable(old, expected, run, frozen, frozen))
            self.assertFalse(resume_reusable(
                dict(old, exit_code=1), expected, run, frozen, frozen))
            self.assertFalse(resume_reusable(
                old, expected, run, frozen, {"benchmark_sha256": "c" * 64}))
            self.assertFalse(resume_reusable(
                dict(old, source_tree_sha256="d" * 64), expected, run,
                frozen, frozen))

    def test_duplicate_terminal_and_stage_rows_fail_closed(self):
        failures = []
        indexed([{"input_attempt_id": "1"}, {"input_attempt_id": "1"}],
                "input_attempt_id", failures, "attempts")
        self.assertTrue(failures)

    def test_publication_contract_rejects_missing_packet_and_late_protected(self):
        diag = {"input_attempt_id": "1", "transaction_id": "1", "window_id": "2",
                "publication_snapshot_id": "3",
                "publication_state_solution_id": "4",
                "publication_history_summary_id": "5",
                "publication_manifest_digest": "6",
                "publication_health_state": "7",
                "publication_risk_proof_id": "8",
                "publication_identity_check": "ACCEPTED",
                "publication_missing_identity_fields": "",
                "transaction_opened": "1", "publication_gate_executed": "1",
                "publication_transition_accepted": "1", "status": "EXECUTED",
                "backend_epoch_before": "0", "backend_epoch_after": "1",
                "publication_protected": "0", "selected_actions": "0",
                "publication_unprotected": "1", "publication_refusal": "closed",
                "risk_total": "0.1", "risk_upper_bound": "0.2",
                "risk_margin": "0.1", "risk_ledger_charged_total": "0.1",
                "risk_ledger_declared_total": "0.1",
                "risk_ledger_closes": "1", "risk_ledger_all_validated": "1",
                "risk_ledger_unvalidated_terms": "0",
                "risk_ledger_not_implemented_terms": "0",
                "selection_risk_proof_id": "8",
                "risk_ledger_terms": "nominal", "publication_certificate_id": "0"}
        integrity = {"transaction_id": "1", "window_id": "2",
                     "batch_committed": "1", "publication_protected": "0",
                     "formal_eligible": "0", "final_packet_protocol_version": "2",
                     "final_packet_digest": "9", "final_packet_authoritative": "0",
                     "selected_action_id": "0", "selected_action_type": "",
                     "backend_updates": "1", "fde_status": "RISK_BUDGET_INVALID",
                     "deadline_missed": "0"}
        transaction = {"transaction_id": "1", "window_id": "2",
                       "selected_action_id": "0", "backend_updates": "1",
                       "fde_status": "RISK_BUDGET_INVALID"}
        terminal = {"input_attempt_id": "1", "transaction_id": "1",
                    "window_id": "2", "selected_action_id": "0",
                    "backend_epoch_before": "0", "backend_epoch_after": "1",
                    "batch_committed": "1", "risk_ledger_closes": "1",
                    "risk_ledger_all_validated": "1",
                    "selection_risk_proof_id": "8",
                    "publication_risk_proof_id": "8",
                    "publication_certificate_id": "0",
                    "publication_protected": "0", "formal_eligible": "0",
                    "deadline_missed": "0", "publish_call_steady_ns": "100",
                    "publish_return_steady_ns": "101",
                    "arrival_to_publish_ns": "50", "final_packet_digest": "9"}
        self.assertEqual(publication_failures(
            diag, integrity, [], False, True, transaction, terminal, 0.00005), [])
        mutations = (
            (dict(diag, risk_ledger_closes="0"), integrity, terminal),
            (dict(diag, risk_ledger_charged_total="0.2"), integrity, terminal),
            (dict(diag, risk_ledger_declared_total="0.2"), integrity, terminal),
            (dict(diag, selection_risk_proof_id="7"), integrity, terminal),
            (diag, dict(integrity, final_packet_digest="10"), terminal),
            (diag, dict(integrity, selected_action_id="4"), terminal),
            (diag, integrity, dict(terminal, backend_epoch_after="2")),
            (diag, integrity, dict(terminal, final_packet_digest="10")),
        )
        for mutated_diag, mutated_integrity, mutated_terminal in mutations:
            self.assertTrue(publication_failures(
                mutated_diag, mutated_integrity, [], False, True,
                transaction, mutated_terminal, 0.00005))
        self.assertTrue(publication_failures(
            dict(diag, publication_protected="1"), integrity, [], True, True,
            transaction, terminal, 0.00005))

        selected_diag = dict(diag, selected_actions="1")
        selected_integrity = dict(integrity, selected_action_id="4",
                                  selected_action_type="REMOVE")
        selected_transaction = dict(transaction, selected_action_id="4")
        selected_terminal = dict(terminal, selected_action_id="4")
        winner = {"selected": "1", "action_id": "4",
                  "action_type": "REMOVE", "valid": "1",
                  "post_detector_passed": "1", "covers_plausible_set": "1",
                  "model_error_validated": "1"}
        self.assertEqual(publication_failures(
            selected_diag, selected_integrity, [winner], False, True,
            selected_transaction, selected_terminal, 0.00005), [])
        for key in ("valid", "post_detector_passed", "covers_plausible_set",
                    "model_error_validated"):
            self.assertTrue(publication_failures(
                selected_diag, selected_integrity,
                [dict(winner, **{key: "0"})], False, True,
                selected_transaction, selected_terminal, 0.00005), key)
        self.assertTrue(publication_failures(
            selected_diag, dict(selected_integrity, selected_action_type="OTHER"),
            [winner], False, True, selected_transaction, selected_terminal,
            0.00005))

    def test_gate24_rejects_top_level_only_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "all-attempt-performance.json").write_text(
                json.dumps({"groups": [{"aggregate": {"attempted": 135,
                    "complete_work_attempts": 135, "excluded": 0,
                    "complete_work_rate": 1.0, "rss_peak_kib": 100},
                    "runs": []}]}), encoding="utf-8")
            result = validate_gate24(root)
            self.assertEqual(result["status"], "FAIL")

    def test_gate24_raw_and_equivalence_mutations_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            gate, performance = self._gate24_fixture(Path(directory))
            self.assertEqual(validate_gate24(gate)["status"], "PASS")
            original = json.loads(performance.read_text())
            payload = json.loads(json.dumps(original))
            payload["groups"][0]["runs"][0]["raw_attempt_outcomes"][0][
                "attempt"] = 2
            performance.write_text(json.dumps(payload))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")
            payload = json.loads(json.dumps(original))
            payload["groups"][0]["runs"][0]["raw_attempt_outcomes"][0][
                "complete_work"] = False
            performance.write_text(json.dumps(payload))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")
            payload = json.loads(json.dumps(original))
            payload["groups"][0]["runs"][0]["rss_peak_kib"] = 101
            performance.write_text(json.dumps(payload))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")
            gate, performance = self._gate24_fixture(Path(directory) / "second")
            payload = json.loads(performance.read_text())
            payload["groups"][0]["runs"][0]["raw_core_ms"][0] += 1
            performance.write_text(json.dumps(payload))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")
            gate, performance = self._gate24_fixture(Path(directory) / "third")
            payload = json.loads(performance.read_text())
            payload["groups"][0]["aggregate"]["attempted"] = 134
            performance.write_text(json.dumps(payload))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")
            gate, _ = self._gate24_fixture(Path(directory) / "fourth")
            equivalence = gate / "equivalence-run1.json"
            item = json.loads(equivalence.read_text())
            item["before"] = str(Path(directory) / "contains-p1-05/run1")
            equivalence.write_text(json.dumps(item))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")
            gate, _ = self._gate24_fixture(Path(directory) / "fifth")
            equivalence = gate / "equivalence-run1.json"
            item = json.loads(equivalence.read_text())
            item["files"]["states.csv"]["after_sha256"] = "0" * 64
            equivalence.write_text(json.dumps(item))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")
            gate, _ = self._gate24_fixture(Path(directory) / "sixth")
            equivalence = gate / "equivalence-run1.json"
            item = json.loads(equivalence.read_text())
            item["files"]["states.csv"]["rows"] += 1
            equivalence.write_text(json.dumps(item))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")
            gate, _ = self._gate24_fixture(Path(directory) / "seventh")
            frozen = {"source_tree_sha256": "a" * 64,
                      "benchmark_sha256": "b" * 64,
                      "loaded_dso_sha256": {"lib": "c" * 64}}
            identity_path = gate / "gate24_run_identity.json"
            identity = json.loads(identity_path.read_text())
            identity.update(frozen)
            identity_path.write_text(json.dumps(identity))
            (gate / "campaign_identity.json").write_text(json.dumps(frozen))
            raw_validator = acceptance_summary.validate_gate24_run
            with patch(
                    "summarize_p1_06_simulation_acceptance.validate_gate24_run",
                    side_effect=lambda root, run, index, unused:
                        raw_validator(root, run, index, None)):
                self.assertEqual(validate_gate24(gate, frozen)["status"], "PASS")
                identity["source_tree_sha256"] = "0" * 64
                identity_path.write_text(json.dumps(identity))
            self.assertEqual(validate_gate24(gate, frozen)["status"], "FAIL")

    def test_gate24_recomputes_raw_semantics_despite_forged_pass_json(self):
        with tempfile.TemporaryDirectory() as directory:
            gate, performance = self._gate24_fixture(Path(directory))
            states = gate / "run1/states.csv"
            with states.open(newline="") as stream:
                rows_payload = list(csv.DictReader(stream))
            fields = list(rows_payload[0])
            rows_payload[0]["px"] = str(float(rows_payload[0]["px"]) + 1.0)
            with states.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader(); writer.writerows(rows_payload)
            equivalence = gate / "equivalence-run1.json"
            forged = json.loads(equivalence.read_text())
            forged["status"] = "PASS"
            forged["files"]["states.csv"].update({
                "status": "PASS", "mismatches": [],
                "after_sha256": hashlib.sha256(states.read_bytes()).hexdigest()})
            equivalence.write_text(json.dumps(forged))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")

            gate, performance = self._gate24_fixture(Path(directory) / "discrete")
            attempts = gate / "run1/diagnostic_attempts.csv"
            with attempts.open(newline="") as stream:
                rows_payload = list(csv.DictReader(stream))
            fields = list(rows_payload[0])
            rows_payload[0]["status"] = "DISCARDED"
            with attempts.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader(); writer.writerows(rows_payload)
            summary = json.loads(performance.read_text())
            summary["groups"][0]["runs"][0]["raw_attempt_outcomes"][0][
                "status"] = "DISCARDED"
            performance.write_text(json.dumps(summary))
            equivalence = gate / "equivalence-run1.json"
            forged = json.loads(equivalence.read_text())
            forged["status"] = "PASS"
            forged["files"]["diagnostic_attempts.csv"].update({
                "status": "PASS", "mismatches": [],
                "after_sha256": hashlib.sha256(attempts.read_bytes()).hexdigest()})
            equivalence.write_text(json.dumps(forged))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")

    def test_gate24_raw_terminal_outcome_fields_cannot_be_aggregated_away(self):
        for key, value in (("sensor_stale", True),
                           ("terminal_reason", "forged"),
                           ("transaction_opened", False)):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as directory:
                gate, performance = self._gate24_fixture(Path(directory))
                payload = json.loads(performance.read_text())
                payload["groups"][0]["runs"][0]["raw_attempt_outcomes"][0][
                    key] = value
                performance.write_text(json.dumps(payload))
                self.assertEqual(validate_gate24(gate)["status"], "FAIL")

    def test_gate24_synchronized_sensor_audit_forgery_fails(self):
        for field, value, outcome_field, outcome_value in (
                ("watchdog_sensor_stale", "1", "sensor_stale", True),
                ("watchdog_reason", "forged", "watchdog_reason", "forged")):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                gate, performance = self._gate24_fixture(Path(directory))
                attempts = gate / "run1/diagnostic_attempts.csv"
                with attempts.open(newline="") as stream:
                    rows_payload = list(csv.DictReader(stream))
                fields = list(rows_payload[0])
                rows_payload[0][field] = value
                with attempts.open("w", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader(); writer.writerows(rows_payload)
                summary = json.loads(performance.read_text())
                summary["groups"][0]["runs"][0]["raw_attempt_outcomes"][0][
                    outcome_field] = outcome_value
                performance.write_text(json.dumps(summary))
                evidence = gate / "equivalence-run1.json"
                forged = json.loads(evidence.read_text())
                forged["status"] = "PASS"
                forged["files"]["diagnostic_attempts.csv"].update({
                    "status": "PASS", "mismatches": [],
                    "after_sha256": hashlib.sha256(
                        attempts.read_bytes()).hexdigest()})
                evidence.write_text(json.dumps(forged))
                self.assertEqual(validate_gate24(gate)["status"], "FAIL")

    def test_gate24_rejects_path_escape_and_golden_manifest_forgery(self):
        with tempfile.TemporaryDirectory() as directory:
            gate, performance = self._gate24_fixture(Path(directory))
            payload = json.loads(performance.read_text())
            payload["groups"][0]["runs"][0]["run_directory"] = "../run1"
            performance.write_text(json.dumps(payload))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")

            gate, performance = self._gate24_fixture(
                Path(directory) / "absolute-external")
            external = Path(directory) / "outside-run1"
            shutil.copytree(gate / "run1", external)
            payload = json.loads(performance.read_text())
            payload["groups"][0]["runs"][0]["run_directory"] = str(external)
            performance.write_text(json.dumps(payload))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")

            gate, _ = self._gate24_fixture(Path(directory) / "manifest")
            path = gate / "golden_reference_manifest.json"
            manifest = json.loads(path.read_text())
            manifest["runs"]["run1"]["states.csv"]["blob"] = "0" * 40
            path.write_text(json.dumps(manifest))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")

    def test_gate24_rejects_each_pinned_anchor_difference(self):
        manifest_fields = ("tag_object", "commit", "tree")
        for field in manifest_fields:
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                gate, _ = self._gate24_fixture(Path(directory))
                path = gate / "golden_reference_manifest.json"
                payload = json.loads(path.read_text())
                payload[field] = "0" * 40
                path.write_text(json.dumps(payload))
                self.assertEqual(validate_gate24(gate)["status"], "FAIL")
        with tempfile.TemporaryDirectory() as directory:
            gate, _ = self._gate24_fixture(Path(directory))
            path = gate / "gate24_run_identity.json"
            payload = json.loads(path.read_text())
            payload["golden_manifest_sha256"] = "0" * 64
            path.write_text(json.dumps(payload))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")

    def test_gate24_rejects_protocol_pin_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            gate, _ = self._gate24_fixture(root)
            protocol = yaml.safe_load(
                (ROOT / "config/fde_profiles_validation.yaml").read_text())
            protocol["golden_reference_anchor"]["manifest_sha256"] = "0" * 64
            protocol_path = root / "replaced-protocol.yaml"
            protocol_path.write_text(yaml.safe_dump(protocol, sort_keys=False))
            self.assertEqual(
                validate_gate24(gate, protocol_path=protocol_path)["status"],
                "FAIL")

    def test_force_moved_annotated_tag_fails_after_synchronized_replacement(self):
        """A self-consistent moved tag is not the independently pinned anchor."""
        reference = GATE24_GOLDEN_ANCHOR["reference"]
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "repo"
            subprocess.run(
                ["git", "clone", "--quiet", "--no-checkout", "--shared",
                 str(ROOT), str(fixture)], check=True)
            # The fixture starts with the exact production object graph.
            golden_reference_manifest(reference, root=fixture)
            subprocess.run(["git", "-C", str(fixture), "config",
                            "user.name", "negative fixture"], check=True)
            subprocess.run(["git", "-C", str(fixture), "config",
                            "user.email", "fixture@example.invalid"], check=True)
            subprocess.run(["git", "-C", str(fixture), "tag", "-f", "-a",
                            reference, "HEAD", "-m", "synchronized replacement"],
                           check=True, stdout=subprocess.DEVNULL)
            replacement = _resolve_golden_reference_manifest(reference, fixture)
            replacement_digest = canonical_golden_manifest_sha256(replacement)
            # Model an attacker synchronizing the manifest, identity and all
            # local copies to the moved tag.  They agree with one another.
            synchronized = Path(directory) / "synchronized"
            synchronized.mkdir()
            (synchronized / "golden_reference_manifest.json").write_text(
                json.dumps(replacement, sort_keys=True))
            (synchronized / "gate24_run_identity.json").write_text(json.dumps({
                "golden_reference": reference,
                "golden_annotated_tag_object": replacement["tag_object"],
                "golden_peeled_commit": replacement["commit"],
                "golden_tree": replacement["tree"],
                "golden_manifest_sha256": replacement_digest,
            }, sort_keys=True))
            local = synchronized / "local"
            for run, files in replacement["runs"].items():
                for name, metadata in files.items():
                    path = local / run / name
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_bytes(subprocess.check_output(
                        ["git", "-C", str(fixture), "cat-file", "blob",
                         metadata["blob"]]))
                    self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                                     metadata["sha256"])
            self.assertNotEqual(replacement["tag_object"],
                                GATE24_GOLDEN_ANCHOR["annotated_tag_object"])
            with self.assertRaisesRegex(ValueError, "differs from pinned anchor"):
                golden_reference_manifest(reference, root=fixture)

    def test_gate24_rejects_mutated_golden_copy_and_symlink_run(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            gate, _ = self._gate24_fixture(root)
            source = (ROOT / "docs/evidence/p1-05-deterministic-concurrency/"
                      "archived-raw/candidate-3x45")
            golden_copy = root / "golden-copy"
            shutil.copytree(source, golden_copy)
            states = golden_copy / "run1/states.csv"
            states.write_bytes(states.read_bytes() + b"\n")
            self.assertEqual(
                validate_gate24(gate, golden_copy_root=golden_copy)["status"],
                "FAIL")

            gate, performance = self._gate24_fixture(root / "symlink")
            run1 = gate / "run1"
            moved = root / "external-run1"
            run1.rename(moved)
            run1.symlink_to(moved, target_is_directory=True)
            payload = json.loads(performance.read_text())
            payload["groups"][0]["runs"][0]["run_directory"] = str(run1)
            performance.write_text(json.dumps(payload))
            self.assertEqual(validate_gate24(gate)["status"], "FAIL")

    def test_fgo_rejects_199_201_even_when_total_is_800(self):
        with tempfile.TemporaryDirectory() as directory, patch(
                "summarize_p1_06_simulation_acceptance.terminal_artifacts_match",
                return_value=True):
            root = Path(directory)
            analyzed, protocol, calibrations = self._fgo_fixture(root)
            for run_index, delta in ((0, -1), (1, 1)):
                path = (Path(analyzed["runs"][run_index]["artifact_directory"]) /
                        "integrity.csv")
                with path.open(newline="") as stream:
                    rows_payload = list(csv.DictReader(stream))
                fields = list(rows_payload[0])
                rows_payload = (rows_payload[:-1] if delta < 0 else
                                rows_payload + [dict(rows_payload[-1])])
                with path.open("w", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader(); writer.writerows(rows_payload)
            result = fgo_gates(analyzed, protocol["gates"], protocol,
                               calibrations, None, root)
            self.assertEqual(result["status"], "FAIL")
            self.assertTrue(any("FGO run is not 200/200" in value
                                for value in result["failures"]))

    def test_fgo_recomputes_raw_terminal_count_and_accuracy(self):
        with tempfile.TemporaryDirectory() as directory, patch(
                "summarize_p1_06_simulation_acceptance.terminal_artifacts_match",
                return_value=True):
            analyzed, protocol, calibrations = self._fgo_fixture(Path(directory))
            result = fgo_gates(analyzed, protocol["gates"], protocol,
                               calibrations, None, Path(directory))
            self.assertEqual(result["status"], "PASS", result["failures"])

            analyzed["runs"][0]["counts"]["attempted"] = 199
            result = fgo_gates(analyzed, protocol["gates"], protocol,
                               calibrations, None, Path(directory))
            self.assertEqual(result["status"], "FAIL")

            analyzed, protocol, calibrations = self._fgo_fixture(
                Path(directory) / "raw-199-summary-200")
            integrity = (Path(analyzed["runs"][0]["artifact_directory"]) /
                         "integrity.csv")
            with integrity.open(newline="") as stream:
                integrity_rows = list(csv.DictReader(stream))
            fields = list(integrity_rows[0])
            with integrity.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader(); writer.writerows(integrity_rows[:-1])
            # The analyzed summary intentionally remains the original 200.
            self.assertEqual(analyzed["runs"][0]["counts"]["attempted"], 200)
            result = fgo_gates(analyzed, protocol["gates"], protocol,
                               calibrations, None,
                               Path(directory) / "raw-199-summary-200")
            self.assertEqual(result["status"], "FAIL")

            analyzed, protocol, calibrations = self._fgo_fixture(
                Path(directory) / "summary-accuracy")
            analyzed["runs"][0]["accuracy"][
                "truth_at_state_position_rmse_m"] = 0.25
            result = fgo_gates(analyzed, protocol["gates"], protocol,
                               calibrations, None,
                               Path(directory) / "summary-accuracy")
            self.assertEqual(result["status"], "FAIL")

            analyzed, protocol, calibrations = self._fgo_fixture(
                Path(directory) / "raw-value")
            states = Path(analyzed["runs"][0]["artifact_directory"]) / "states.csv"
            with states.open(newline="") as stream:
                state_rows = list(csv.DictReader(stream))
            fields = list(state_rows[0])
            state_rows[0]["px"] = "2.0"
            with states.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader(); writer.writerows(state_rows)
            result = fgo_gates(analyzed, protocol["gates"], protocol,
                               calibrations, None, Path(directory) / "raw-value")
            self.assertEqual(result["status"], "FAIL")

    def test_fgo_rejects_nonfinite_timestamp_order_and_path_mutations(self):
        mutations = (
            ("states.csv", "px", "nan"),
            ("states.csv", "vx", "inf"),
            ("states.csv", "qw", "0"),
            ("ground_truth.csv", "timestamp_ns", ""),
        )
        for filename, field, value in mutations:
            with self.subTest(filename=filename, field=field), \
                    tempfile.TemporaryDirectory() as directory, patch(
                        "summarize_p1_06_simulation_acceptance.terminal_artifacts_match",
                        return_value=True):
                root = Path(directory)
                analyzed, protocol, calibrations = self._fgo_fixture(root)
                path = Path(analyzed["runs"][0]["artifact_directory"]) / filename
                with path.open(newline="") as stream:
                    rows_payload = list(csv.DictReader(stream))
                fields = list(rows_payload[0])
                if field == "qw":
                    for key in ("qw", "qx", "qy", "qz"):
                        rows_payload[0][key] = "0"
                else:
                    rows_payload[0][field] = value
                with path.open("w", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader(); writer.writerows(rows_payload)
                result = fgo_gates(analyzed, protocol["gates"], protocol,
                                   calibrations, None, root)
                self.assertEqual(result["status"], "FAIL")

        with tempfile.TemporaryDirectory() as directory, patch(
                "summarize_p1_06_simulation_acceptance.terminal_artifacts_match",
                return_value=True):
            root = Path(directory)
            analyzed, protocol, calibrations = self._fgo_fixture(root)
            states = Path(analyzed["runs"][0]["artifact_directory"]) / "states.csv"
            with states.open(newline="") as stream:
                rows_payload = list(csv.DictReader(stream))
            fields = list(rows_payload[0])
            rows_payload[0], rows_payload[1] = rows_payload[1], rows_payload[0]
            with states.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader(); writer.writerows(rows_payload)
            result = fgo_gates(analyzed, protocol["gates"], protocol,
                               calibrations, None, root)
            self.assertEqual(result["status"], "FAIL")

            analyzed, protocol, calibrations = self._fgo_fixture(root / "path")
            analyzed["runs"][0]["artifact_directory"] = "../outside"
            result = fgo_gates(analyzed, protocol["gates"], protocol,
                               calibrations, None, root / "path")
            self.assertEqual(result["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
