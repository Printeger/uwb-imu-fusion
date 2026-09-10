#!/usr/bin/env python3
"""R05 engineering/startup gate. It never enters estimator optimization."""

import hashlib
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

from a19_r05_launch import prepare_attempt, write_json_atomic


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    root = Path.cwd().resolve()
    evidence = Path(sys.argv[1]).resolve()
    runner = evidence / "a19_r05_pipeline_final"
    config = root / "doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/P1_step_seed10101_conditional.yaml"
    raw = root / "doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/inputs/raw/step"
    prepare = evidence / "engineering/prepare_output_final/prepare_status.json"
    prepare_command = evidence / "engineering/prepare_run_final/command.json"
    handshake = evidence / "engineering/direct_handshake_attempt2/handshake.json"
    handshake_eval = evidence / "engineering/direct_handshake_evaluation.json"
    tests = [
        evidence / "engineering/identity_contract_tests.exitcode",
        evidence / "engineering/direct_handshake_attempt2_test.exitcode",
        evidence / "engineering/direct_handshake_evaluator.exitcode",
    ]
    runtime = [
        Path("/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so"),
        Path("/usr/local/lib/libgtsam.so.4.2.0"),
        Path("/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2"),
        Path("/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0"),
    ]
    required = [runner, config, raw / "input_manifest.json", raw / "imu.csv",
                raw / "uwb_observations.csv", prepare, prepare_command,
                handshake, handshake_eval, *tests, *runtime]
    checks = {"required_paths": all(path.is_file() for path in required)}
    identities = [{"path": str(path), "sha256": digest(path)} for path in required]

    prepared = json.loads(prepare.read_text())
    checks["real_request_contract_check"] = (
        prepared.get("status") == "PREPARED_AND_CONTRACT_CHECKED_BEFORE_STAGE1"
        and prepared.get("contract_accepted") is True
        and prepared.get("contract_reason") == "ACCEPTED"
        and prepared.get("request_schema") ==
            "A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY"
        and prepared.get("request_provider") == "development_nonconsumable"
        and prepared.get("request_policy") == "PAPER_CERTIFIED_PAIR_REDUCTION_V1"
        and prepared.get("request_role") == "development"
        and prepared.get("request_identity") == prepared.get("context_solver_identity")
        and prepared.get("stage1_calls") == 0
        and prepared.get("conditional_optimizer_calls") == 0
        and all(prepared.get(key) is True for key in
                ("raw_load", "plan", "materialize", "initialize",
                 "graph_values", "content_identity"))
    )
    command = json.loads(prepare_command.read_text())
    checks["prepare_command_exit_and_no_truth"] = (
        command.get("exit_code") == 0
        and command.get("forbidden_input_open_lines") == []
        and command.get("maps_captured") is True
    )
    direct = json.loads(handshake.read_text())
    direct_eval = json.loads(handshake_eval.read_text())
    checks["direct_stage1_to_evaluator_handshake"] = (
        direct.get("development_schema") == prepared.get("request_schema")
        and direct.get("provider") == prepared.get("request_provider")
        and direct.get("partition_hash") == direct.get("final_parent_partition_hash")
        and direct.get("solver_identity") == direct.get("final_solver_identity")
        and direct.get("stage1_callbacks", 0) > 0
        and direct.get("stage2_callbacks", 0) > 0
        and direct.get("score_groups", 0) > 0
        and direct.get("final_valid") is True
        and direct_eval.get("status") == "PASS"
        and direct_eval.get("parent_relations_ok") is True
    )
    checks["focused_tests_nonzero"] = all(
        path.read_text().strip() == "0" for path in tests)

    ldd = subprocess.run(["ldd", str(runner)], text=True, capture_output=True,
                         check=True).stdout
    resolved = {}
    for line in ldd.splitlines():
        fields = line.strip().split()
        if len(fields) >= 3 and fields[1] == "=>" and fields[2].startswith("/"):
            resolved[fields[0]] = str(Path(fields[2]).resolve())
    expected = {"libuwb_imu_fgo.so": str(runtime[0].resolve()),
                "libgtsam.so.4": str(runtime[1].resolve()),
                "libmpfr.so.6": str(runtime[2].resolve()),
                "libgmp.so.10": str(runtime[3].resolve())}
    checks["dynamic_library_resolution"] = all(
        resolved.get(name) == path for name, path in expected.items())

    # Independent temporary launch preparation: no runner or estimator call.
    with tempfile.TemporaryDirectory(prefix="a19-r05-launch-") as name:
        temporary = Path(name)
        launched = prepare_attempt(temporary, "probe")
        launch_status = json.loads(launched["status"].read_text())
        checks["isolated_parent_writable_fresh_leaf_atomic_status"] = (
            launch_status.get("parent_exists") is True
            and launch_status.get("parent_writable") is True
            and launch_status.get("output_leaf_exists") is False
            and not launched["output_leaf"].exists()
            and not launched["status"].with_name(
                launched["status"].name + ".tmp").exists()
        )

    argv = [str(runner), "--policy",
            "PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1",
            "development-prepare-only", str(config),
            "ORIGINAL_RAW_NO_CHECKPOINT",
            str(evidence / "engineering/prepare_output_final")]
    overwrite = subprocess.run(argv, text=True, capture_output=True)
    (evidence / "engineering/overwrite_rejection.stdout").write_text(overwrite.stdout)
    (evidence / "engineering/overwrite_rejection.stderr").write_text(overwrite.stderr)
    checks["existing_leaf_rejected_before_initialization"] = (
        overwrite.returncode == 2 and "OUTPUT_EXISTS" in overwrite.stderr)
    wrong = subprocess.run([str(runner), "--wrong"], text=True,
                           capture_output=True)
    (evidence / "engineering/wrong_argv.stdout").write_text(wrong.stdout)
    (evidence / "engineering/wrong_argv.stderr").write_text(wrong.stderr)
    checks["wrong_argv_rejected"] = wrong.returncode == 2
    checks["formal_reader_rejected_development"] = (
        prepared.get("consumable") is False
        and (evidence / "engineering/prepare_output_final/diagnostic_manifest.json").is_file()
    )

    result = {
        "schema": "T10_A19_R05_ENGINEERING_GATE_V1",
        "passed": all(checks.values()),
        "estimator": "NOT_RUN",
        "checks": checks,
        "identities": identities,
        "ldd": ldd.splitlines(),
        "ldd_realpaths": resolved,
        "expected_runtime": expected,
    }
    write_json_atomic(evidence / "ENGINEERING_GATE.json", result)
    print(json.dumps({"passed": result["passed"], "checks": checks}, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
