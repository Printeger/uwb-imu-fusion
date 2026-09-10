#!/usr/bin/env python3
"""Audit the frozen A19-R01 engineering gate without running an estimator."""
import csv
import datetime
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

root = Path.cwd()
evidence = Path(sys.argv[1]).resolve()


def command(relative):
    return json.loads((evidence / relative / "command.json").read_text())


def loaded(relative):
    return json.loads((evidence / relative).read_text())


mixed = loaded("engineering/mixed_output_gate2/summary.json")
constructor = loaded("engineering/constructor_output_gate2/pipeline_status.json")
constructor_stage = loaded(
    "engineering/constructor_output_gate2/stage2/status.json")
partial = loaded("engineering/partial_output_gate2/pipeline_status.json")
partial_stage = loaded("engineering/partial_output_gate2/stage2/status.json")
manifest = loaded("engineering/mixed_output_gate2/diagnostic_manifest.json")
xml_root = ET.parse(evidence / "engineering/refit_gtest_final.xml").getroot()
partition_rows = list(csv.DictReader((
    evidence / "engineering/constructor_output_gate2/stage1/partition.csv"
).open()))
snapshot_rows = list(csv.DictReader((
    evidence / "engineering/constructor_output_gate2/stage1/support_snapshot.csv"
).open()))

checks = {
    "final_core_and_refit_build": command("commands/build_refit_test_5")["exit_code"] == 0,
    "final_runner_compile_link":
        command("commands/compile_runner_2")["exit_code"] == 0 and
        command("commands/link_runner_3")["exit_code"] == 0,
    "final_integration_compile_link":
        command("commands/compile_integration_4")["exit_code"] == 0 and
        command("commands/link_integration_5")["exit_code"] == 0,
    "mixed_real_certified_exit": command("engineering/mixed_certified_gate2")["exit_code"] == 0,
    "mixed_all_ranges_and_live_c":
        mixed["converged"] and mixed["calls"] > 0 and mixed["trials"] > 0 and
        mixed["accepted"] > 0 and mixed["candidate_factors"] == 2 and
        mixed["reference_factors"] == 2 and mixed["live_c_m"] > 0,
    "mixed_existing_scoring_eta_s_gamma":
        mixed["eta"] > 0 and mixed["s_m"] >= 0 and
        isinstance(mixed["gamma"], (int, float)) and
        bool(mixed["linearization_id"]),
    "formal_reader_rejects_nonconsumable_diagnostic":
        manifest["schema"] == "A19_STAGE1_STAGE2_SCORE_DIAGNOSTIC_ONLY" and
        manifest["role"] == "development" and manifest["consumable"] is False,
    "range_contract_negative_tests":
        int(xml_root.attrib["tests"]) == 25 and int(xml_root.attrib["failures"]) == 0 and
        int(xml_root.attrib["errors"]) == 0 and
        any(case.attrib.get("name") ==
            "A19DevelopmentRejectsRangeContractMismatchesBeforeCallback"
            for case in xml_root.iter("testcase")),
    "stage1_checkpoint_before_constructor":
        len(snapshot_rows) == 4 and len(partition_rows) == 1 and
        command("engineering/constructor_failure_gate2")["exit_code"] == 0 and
        (evidence / "engineering/constructor_output_gate2/stage1/partition_identity.json").exists(),
    "constructor_zero_exact_counts":
        constructor["Stage1"] == "CONVERGED" and
        constructor["Stage2"] == "FAILED_CONSTRUCTOR_BEFORE_ITERATE" and
        constructor["scoring"] == "NOT_RUN" and
        constructor_stage["calls"] == 0 and constructor_stage["trials"] == 0 and
        constructor_stage["counter_state"] == "COMPLETE" and
        constructor_stage["segments"] is None and constructor_stage["eligible"] is None,
    "partial_failure_lower_bound_counts":
        command("engineering/partial_failure_gate2")["exit_code"] == 0 and
        partial["Stage2"] == "FAILED_PARTIAL_EXECUTION" and
        partial["scoring"] == "NOT_RUN" and partial_stage["calls"] > 0 and
        partial_stage["trials"] > 0 and
        partial_stage["counter_state"] ==
            "CONFIRMED_LOWER_BOUND_CURRENT_OPERATION_UNKNOWN",
    "regression_build_and_suite":
        command("commands/build_regression_targets_3")["exit_code"] == 0 and
        command("engineering/regression_suite")["exit_code"] == 0,
    "default_off": command("engineering/default_off")["exit_code"] == 2,
    "a18_43_pair_compatibility":
        command("engineering/static_43")["exit_code"] == 0 and
        command("engineering/static_43_check")["exit_code"] == 0 and
        loaded("engineering/static_batch_checks.json")["passed"],
    "no_truth_gt_in_final_engineering_commands": all(
        not command(path)["forbidden_input_open_lines"] for path in (
            "engineering/mixed_certified_gate2",
            "engineering/constructor_failure_gate2",
            "engineering/partial_failure_gate2",
            "engineering/refit_tests_final",
            "engineering/regression_suite",
            "engineering/static_43",
            "engineering/static_43_check",
        )),
    "frozen_config": hashlib.sha256((root /
        "doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/"
        "P1_step_seed10101_conditional.yaml").read_bytes()).hexdigest() ==
        "479df3daae2096bb665582f60611c66841ed87edc496ddcbaf7cc9e180ed6c4f",
    "protocol_preregistered": (evidence / "before/T10_A19_R01_PROTOCOL.md").exists(),
}

gate = {
    "schema": "A19_R01_ENGINEERING_GATE_V1",
    "passed": all(checks.values()),
    "checks": checks,
    "mixed_fixture": mixed,
    "retained_failures": [
        "wrong build working directory",
        "initial integration fixture compile error",
        "two malformed manual link paths",
        "ASSERT used in a non-void callback",
        "reference certificate validation initially leaked into legacy Run",
        "bare gtsam_unstable selected system GTSAM 4.0",
        "full regression build exceeded its first 300 second record",
        "one malformed evidence-logger invocation",
    ],
    "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}
(evidence / "ENGINEERING_GATE.json").write_text(json.dumps(gate, indent=2) + "\n")
if not gate["passed"]:
    raise SystemExit("A19-R01 engineering gate failed")

config = root / ("doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/"
                 "P1_step_seed10101_conditional.yaml")
raw = root / ("doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/"
              "inputs/raw/step")
paths = [
    root / "include/uifgo/nlos_refit.h",
    root / "src/nlos_refit.cpp",
    root / "test/test_nlos_refit.cpp",
    root / "tools/paper/a18_live_graph.h",
    root / "tools/paper/a18_optimizer.h",
    root / "tools/paper/a19_r01_status.h",
    root / "tools/paper/a19_r01_integration.cpp",
    root / "tools/paper/a19_stage2_score.cpp",
    root / "tools/paper/a19_r01_gate.py",
    root / "tools/paper/a19_r01_authorize.py",
    root / "doc/ie_sprint/T10_A19_R01_PROTOCOL.md",
    root / "doc/ie_sprint/METHOD_CONTRACT.md",
    root / "doc/ie_sprint/EXPERIMENT_CONTRACT.md",
    config,
    evidence / "a19_stage2_score",
    evidence / "a19_r01_integration",
    Path("/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so"),
    Path("/usr/local/lib/libgtsam.so.4.2.0"),
    Path("/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2"),
    Path("/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0"),
    raw / "input_manifest.json",
    raw / "imu.csv",
    raw / "uwb_observations.csv",
]
identities = []
for path in paths:
    path = path.resolve()
    if not path.exists():
        raise SystemExit(f"missing identity input: {path}")
    copy = evidence / "pre_pilot_source" / str(path).lstrip("/")
    copy.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, copy)
    identities.append({
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "bytes": path.stat().st_size,
        "copy": str(copy.relative_to(evidence)),
    })
(evidence / "PRE_PILOT_IDENTITIES.json").write_text(
    json.dumps(identities, indent=2) + "\n")
print(json.dumps({"passed": True, "checks": len(checks),
                  "identities": len(identities)}))
