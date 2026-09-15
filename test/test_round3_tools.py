#!/usr/bin/env python3

import inspect
import json
import os
import pathlib
import stat
import tempfile
import unittest

import run_integrity_round3 as round3
import validate_run_schema as run_schema
from round2_common import InvalidCampaign, canonical_json, enumerate_cells, load_protocol

ROOT = pathlib.Path(__file__).resolve().parents[1]
PROTOCOL_PATH = ROOT / "config/integrity_round3_protocol.json"


class RoundThreeProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.protocol, cls.digest = load_protocol(PROTOCOL_PATH)
        round3.exact_frozen_inheritance(cls.protocol)
        cls.cells = list(enumerate_cells(cls.protocol))

    def test_frozen_matrix_counts_are_exact(self):
        self.assertEqual(len(self.cells), 5543)
        self.assertEqual(sum(1 for _ in round3.iter_tasks(
            self.protocol, self.cells, "pilot")), 88688)
        self.assertEqual(sum(1 for _ in round3.iter_tasks(
            self.protocol, self.cells, "formal")), 1347332)

    def test_gate_bc_verdicts_do_not_reference_analytic_mc(self):
        source = inspect.getsource(round3.unit_gate_verdicts)
        self.assertNotIn("analytic", source.lower())
        self.assertIn("PrepareAndDiscardNeverMutateBackend", source)
        self.assertIn("HistoricalUwbReplacementCommitsAtomically", source)

    def test_gate_bc_reject_stale_workspace_xml_without_campaign_build(self):
        with tempfile.TemporaryDirectory() as temporary:
            campaign = pathlib.Path(temporary)
            (campaign / "evidence").mkdir()
            state = {"tested_code_sha": "frozen",
                     "protocol_sha256": "protocol", "config_sha256": "config"}
            verdicts = round3.unit_gate_verdicts(campaign, state)
            self.assertEqual(verdicts["B"]["status"], "INVALID")
            self.assertEqual(verdicts["C"]["status"], "INVALID")
            self.assertFalse(verdicts["B"]["build_identity_valid"])

    def test_threshold_invariance_is_not_hardcoded(self):
        source = (ROOT / "apps/integrity_round2_scenario.cpp").read_text()
        self.assertNotIn('"local_threshold_invariant\\":true', source)
        self.assertIn("sameDecisionSignature", source)
        self.assertIn("diagnostic_low", source)
        self.assertIn("diagnostic_high", source)

    def test_deterministic_gate_i_expectations_fail_closed(self):
        source = (ROOT / "apps/integrity_round2_scenario.cpp").read_text()
        self.assertIn("LATE_HISTORY_REPLACEMENT_NOT_OBSERVED", source)
        self.assertIn("HISTORY_PRIOR_CONTAMINATED_NOT_OBSERVED", source)
        self.assertIn("BRIDGE_TIMEOUT_REINIT_NOT_VERIFIED", source)
        self.assertIn("!expectation_failure.empty()", source)

    def test_run_schema_v1_through_v5_accept_minimal_valid_runs(self):
        for number in range(1, 6):
            version = f"uwb-imu-pl/v{number}"
            with self.subTest(version=version), tempfile.TemporaryDirectory() as temporary:
                run = pathlib.Path(temporary)
                resolved = run / "resolved_config.yaml"
                resolved.write_text("fixed_lag_epochs: 0\n")
                manifest = {"schema_version": version, "git_sha": "frozen",
                            "config_hash": run_schema.fnv1a64(resolved.read_bytes()),
                            "seed": 1, "git_dirty": False}
                if number >= 2:
                    headers = (run_schema.V2_HEADERS if number <= 3 else
                               run_schema.V4_HEADERS if number == 4 else
                               run_schema.V5_HEADERS)
                    for name, header in headers.items():
                        (run / name).write_text(header + "\n")
                    (run / "summary.json").write_text(json.dumps({
                        "processed": 0, "committed": 0, "rejected": 0,
                        "errors": 0, "metrics": {}}) + "\n")
                if number == 3:
                    manifest.update({"fixed_lag_epochs": 0,
                                     "execution_command": "fixture"})
                if number == 4:
                    manifest.update({"fixed_lag_epochs": 0, "scope": {},
                                     "gate_j_evidence": {}, "formal_eligible": False})
                if number == 5:
                    manifest.update({"fixed_lag_epochs": 0,
                        "protocol_sha256": "0" * 64,
                        "raw_inventory_sha256": "0" * 64,
                        "protocol_path": "UNAVAILABLE",
                        "raw_inventory_path": "UNAVAILABLE",
                        "artifact_checksum_path": "UNAVAILABLE",
                        "seed_domain": "development",
                        "failure_catalog_path": "UNAVAILABLE",
                        "execution_command": "fixture", "attempt": 1})
                (run / "run_manifest.json").write_text(json.dumps(manifest) + "\n")
                self.assertEqual(run_schema.validate(run), version)

    def test_production_integrity_path_has_no_explicit_inverse(self):
        paths = [
            ROOT / "src/uwb_imu_pl/integrity/hypothesis_generator.cpp",
            ROOT / "src/uwb_imu_pl/integrity/integrity_monitor.cpp",
            ROOT / "src/uwb_imu_pl/integrity/protection_level_v2.cpp",
            ROOT / "src/uwb_imu_pl/estimation/incremental_estimator.cpp",
        ]
        for path in paths:
            self.assertNotIn(".inverse()", path.read_text(), path.name)

    def test_jsonl_resume_rejects_changed_identity(self):
        task = {field: value for field, value in zip(round3.IDENTITY_FIELDS,
            ("a", "b", "cell", "development", 0, 3000000, 0))}
        payload = {**task, "status": "PASS"}
        payload["outcome_sha256"] = round3.sha256_bytes(canonical_json(payload))
        with tempfile.TemporaryDirectory() as temporary:
            directory = pathlib.Path(temporary)
            task_path = directory / "task.jsonl"
            output_path = directory / "output.jsonl"
            task_path.write_bytes(canonical_json(task))
            output_path.write_bytes(canonical_json(payload))
            self.assertEqual(round3.valid_output_prefix(task_path, output_path), 1)
            payload["ordinal"] = 1
            body = dict(payload)
            body.pop("outcome_sha256")
            payload["outcome_sha256"] = round3.sha256_bytes(canonical_json(body))
            output_path.write_bytes(canonical_json(payload))
            with self.assertRaisesRegex(InvalidCampaign, "IDENTITY_MISMATCH"):
                round3.valid_output_prefix(task_path, output_path)

    def test_bulk_crash_resume_uses_exact_prefix(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = pathlib.Path(temporary)
            task_path = directory / "tasks.jsonl"
            root = directory / "campaign"
            tasks = []
            for ordinal in range(3):
                task = {"gate": "E", "id": "cell", "cell_id": "cell",
                        "seed_domain": "development", "ordinal": ordinal,
                        "seed": 3000000 + ordinal, "git_sha": "a",
                        "protocol_sha256": "b", "shard": 0}
                tasks.append(task)
            task_path.write_bytes(b"".join(canonical_json(task) for task in tasks))
            worker = directory / "worker.py"
            worker.write_text(
                "#!/usr/bin/env python3\nimport json,sys\n"
                "for i,line in enumerate(sys.stdin):\n"
                " print(json.dumps({'status':'PASS'}), flush=True)\n"
                " if i == 0: sys.exit(9)\n")
            worker.chmod(worker.stat().st_mode | stat.S_IXUSR)
            old_executable, old_config = round3.EXECUTABLE, round3.CONFIG
            round3.EXECUTABLE, round3.CONFIG = worker, PROTOCOL_PATH
            try:
                with self.assertRaisesRegex(InvalidCampaign, "WORKER_CRASH"):
                    round3.run_shard(root, "pilot", 0, task_path, 1)
                output = root / "raw/pilot/shard-0.jsonl"
                self.assertEqual(round3.valid_output_prefix(task_path, output), 1)
                worker.write_text(
                    "#!/usr/bin/env python3\nimport json,sys\n"
                    "for line in sys.stdin:\n"
                    " print(json.dumps({'status':'PASS'}), flush=True)\n")
                worker.chmod(worker.stat().st_mode | stat.S_IXUSR)
                summary = round3.run_shard(root, "pilot", 0, task_path, 1)
                self.assertEqual(summary["rows"], 3)
                self.assertEqual(summary["resumed_rows"], 1)
            finally:
                round3.EXECUTABLE, round3.CONFIG = old_executable, old_config

    def test_streaming_completeness_rejects_missing_tail(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            for shard in range(4):
                task_path = root / f"tasks/pilot/shard-{shard}.jsonl"
                output_path = root / f"raw/pilot/shard-{shard}.jsonl"
                task_path.parent.mkdir(parents=True, exist_ok=True)
                output_path.parent.mkdir(parents=True, exist_ok=True)
                task = {"gate": "E", "cell_id": f"cell-{shard}",
                        "seed_domain": "development", "ordinal": 0,
                        "seed": 3000000 + shard, "git_sha": "a",
                        "protocol_sha256": "b", "shard": shard}
                task_path.write_bytes(canonical_json(task))
                if shard != 3:
                    payload = {**task, "status": "PASS"}
                    payload["outcome_sha256"] = round3.sha256_bytes(
                        canonical_json(payload))
                    output_path.write_bytes(canonical_json(payload))
                else:
                    output_path.write_bytes(b"")
            with self.assertRaisesRegex(InvalidCampaign, "MISSING_CELL"):
                round3.stream_profile(root, "pilot")


if __name__ == "__main__":
    unittest.main()
