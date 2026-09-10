#!/usr/bin/env python3
"""Freeze the A19 gate and execute the only authorized fresh pipeline."""
import datetime
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

root = Path.cwd()
evidence = Path(sys.argv[1]).resolve()
config = root / "doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/P1_step_seed10101_conditional.yaml"

def exit_code(relative):
    return json.loads((evidence / relative / "command.json").read_text())["exit_code"]

checks = {
    "core_build": exit_code("commands/build_core_1") == 0,
    "a19_compile": exit_code("commands/compile_a19_1") == 0,
    "a19_link": exit_code("commands/link_a19_1") == 0,
    "regression_targets_build": exit_code("commands/build_regression_targets") == 0,
    "regression_suite": exit_code("engineering/regression_suite") == 0,
    "refit_scoring_24": exit_code("engineering/refit_tests_2") == 0,
    "a18_cpp_104": exit_code("engineering/a18_cpp_tests") == 0,
    "discovery_39": exit_code("engineering/discovery_tests") == 0,
    "default_off": exit_code("engineering/default_off") == 2,
    "static_43_run": exit_code("engineering/static_43") == 0,
    "static_43_check": exit_code("engineering/static_43_check") == 0,
    "static_43_compatible": json.loads(
        (evidence / "engineering/static_batch_checks.json").read_text())["passed"],
    "static_batch_le_10_seconds": json.loads(
        (evidence / "engineering/static_43/command.json").read_text()
    )["finished_utc"] > json.loads(
        (evidence / "engineering/static_43/command.json").read_text()
    )["started_utc"],
    "resolved_fixture_failure_retained":
        exit_code("engineering/refit_tests_1") == 1 and
        exit_code("commands/build_refit_test_1") == 2,
    "missing_path_failure_retained": exit_code("engineering/refit_tests") == 127,
    "frozen_config": hashlib.sha256(config.read_bytes()).hexdigest() ==
        "479df3daae2096bb665582f60611c66841ed87edc496ddcbaf7cc9e180ed6c4f",
    "protocol_preregistered": (evidence / "before/T10_A19_AMENDMENT_PROTOCOL.md").exists(),
}
static_meta = json.loads((evidence / "engineering/static_43/command.json").read_text())
start = datetime.datetime.fromisoformat(static_meta["started_utc"])
finish = datetime.datetime.fromisoformat(static_meta["finished_utc"])
checks["static_batch_le_10_seconds"] = (finish - start).total_seconds() <= 10.0
gate = {
    "schema": "A19_ENGINEERING_GATE_V1",
    "passed": all(checks.values()),
    "checks": checks,
    "static_batch_external_seconds": (finish - start).total_seconds(),
    "resolved_failures": [
        "initial refit test executable path did not exist (exit 127)",
        "first A19 fixture used strict V2 LM and exhausted lambda before outer2",
        "first test compile used ASSERT in a non-void callback lambda",
    ],
    "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}
(evidence / "ENGINEERING_GATE.json").write_text(json.dumps(gate, indent=2) + "\n")
if not gate["passed"]:
    raise SystemExit("A19 engineering gate failed")

paths = [
    root / "include/uifgo/nlos_discovery.h",
    root / "include/uifgo/nlos_refit.h",
    root / "src/nlos_discovery.cpp",
    root / "src/nlos_refit.cpp",
    root / "src/nlos_scoring.cpp",
    root / "test/test_nlos_refit.cpp",
    root / "tools/paper/a18_optimizer.h",
    root / "tools/paper/a19_stage2_score.cpp",
    root / "tools/paper/a19_authorize.py",
    root / "doc/ie_sprint/T10_A19_AMENDMENT_PROTOCOL.md",
    root / "doc/ie_sprint/METHOD_CONTRACT.md",
    root / "doc/ie_sprint/EXPERIMENT_CONTRACT.md",
    config,
    evidence / "a19_stage2_score",
    Path("/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so"),
    Path("/usr/local/lib/libgtsam.so.4.2.0"),
    Path("/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2"),
    Path("/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0"),
]
raw = root / "doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/inputs/raw/step"
paths += [raw / name for name in
          ("input_manifest.json", "imu.csv", "uwb_observations.csv")]
identities = []
for path in paths:
    if not path.exists():
        raise SystemExit(f"missing frozen input {path}")
    destination = evidence / "source" / str(path).lstrip("/")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, destination)
    identities.append({
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "bytes": path.stat().st_size,
        "copy": str(destination.relative_to(evidence)),
    })
(evidence / "PRE_PILOT_IDENTITIES.json").write_text(
    json.dumps(identities, indent=2) + "\n")

ticket = {
    "schema": "A19_PILOT_ONCE_TICKET_V1",
    "gate_sha256": hashlib.sha256(
        (evidence / "ENGINEERING_GATE.json").read_bytes()).hexdigest(),
    "scenario": "P1 step seed10101",
    "initialization": "ORIGINAL_RAW_NO_CHECKPOINT",
    "stage1_outer_max": 500,
    "conditional_call_max": 50,
    "stage2_outer_max": 200,
    "process_tree_seconds": 900,
    "max_processes": 1,
    "retry": "FORBIDDEN",
    "truth_gt": "FORBIDDEN",
    "after_scoring": "STOP",
    "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}
with (evidence / "PILOT_ONCE_TICKET.json").open("x") as stream:
    json.dump(ticket, stream, indent=2)
    stream.write("\n")

command = [
    sys.executable, "tools/paper/a14_run_logged.py",
    "--record", str(evidence / "pilot_run"), "--seconds", "900", "--",
    str(evidence / "a19_stage2_score"), "--policy",
    "PAPER_CERTIFIED_PAIR_REDUCTION_V1",
    "development-stage1-stage2-score", str(config),
    "ORIGINAL_RAW_NO_CHECKPOINT", str(evidence / "pilot/output"),
]
(evidence / "PILOT_COMMAND.json").write_text(json.dumps(command, indent=2) + "\n")
completed = subprocess.run(command)
actual = json.loads((evidence / "pilot_run/command.json").read_text())
(evidence / "PILOT_COMPLETION.json").write_text(json.dumps({
    "wrapper_exit": completed.returncode,
    "actual_exit": actual["exit_code"],
    "external_wall_s": actual["external_wall_s"],
    "forbidden_input_open_lines": actual["forbidden_input_open_lines"],
    "retry": "NOT_RUN",
}, indent=2) + "\n")
print("A19_PILOT_FINISHED", completed.returncode)
raise SystemExit(completed.returncode)
