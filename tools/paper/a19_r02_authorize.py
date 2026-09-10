#!/usr/bin/env python3
"""Consume one A19-R02 ticket and run the bounded automatic pipeline."""
import datetime
import hashlib
import json
import subprocess
import sys
from pathlib import Path

from a19_r02_launch import prepare_attempt, write_json_atomic

root = Path.cwd().resolve()
evidence = Path(sys.argv[1]).resolve()
attempt = sys.argv[2]
preflight = json.loads((evidence / "STARTUP_PREFLIGHT.json").read_text())
if not preflight.get("passed") or preflight.get("estimator") != "NOT_RUN":
    raise SystemExit("STARTUP_PREFLIGHT_NOT_PASSED")
if attempt not in ("attempt1", "attempt2"):
    raise SystemExit("ATTEMPT_ID_REJECTED")
existing = sorted(evidence.glob("attempts/*/TICKET.json"))
if len(existing) >= 2:
    raise SystemExit("INFRASTRUCTURE_ATTEMPT_BUDGET_EXHAUSTED")

prepared = prepare_attempt(evidence, attempt)
runner = evidence / "a19_stage2_score"
config = root / ("doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/"
                 "P1_step_seed10101_conditional.yaml")
for item in preflight["identities"]:
    path = Path(item["path"])
    if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
        raise SystemExit("IDENTITY_CHANGED_AFTER_PREFLIGHT:" + str(path))

ticket = {
    "schema": "A19_R02_AUTO_SCORE_TICKET_V1",
    "attempt": attempt,
    "preflight_sha256": hashlib.sha256(
        (evidence / "STARTUP_PREFLIGHT.json").read_bytes()).hexdigest(),
    "scenario": "A10_FROZEN_P1_STEP_SEED10101",
    "initialization": "ORIGINAL_RAW_NO_CHECKPOINT",
    "stage1_outer_max": 500,
    "conditional_block_iteration_max": 50,
    "stage2_outer_max": 200,
    "process_tree_hard_timeout_seconds": 900,
    "truth_gt": "FORBIDDEN",
    "retry_class": "ONLY_CONFIRMED_PRE_INITIALIZATION_INFRASTRUCTURE_MAX_TWO",
    "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}
write_json_atomic(prepared["root"] / "TICKET.json", ticket)
command = [
    sys.executable, "tools/paper/a14_run_logged.py",
    "--record", str(prepared["root"] / "estimator_run"),
    "--seconds", "900", "--", str(runner), "--policy",
    "PAPER_CERTIFIED_PAIR_REDUCTION_V1",
    "development-stage1-stage2-score", str(config),
    "ORIGINAL_RAW_NO_CHECKPOINT", str(prepared["output_leaf"]),
]
write_json_atomic(prepared["root"] / "COMMAND.json", {"argv": command})
completed = subprocess.run(command)
actual = json.loads((prepared["root"] / "estimator_run/command.json").read_text())
write_json_atomic(prepared["root"] / "COMPLETION.json", {
    "schema": "A19_R02_AUTO_SCORE_COMPLETION_V1",
    "attempt": attempt,
    "wrapper_exit": completed.returncode,
    "actual_exit": actual["exit_code"],
    "external_wall_s": actual["external_wall_s"],
    "forbidden_input_open_lines": actual["forbidden_input_open_lines"],
})
raise SystemExit(completed.returncode)
