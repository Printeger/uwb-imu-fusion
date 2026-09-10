#!/usr/bin/env python3
"""Consume the sole R05 complete-development ticket after exact gates pass."""

import datetime
import hashlib
import json
import subprocess
import sys
from pathlib import Path

from a19_r05_launch import prepare_attempt, write_json_atomic


def main() -> int:
    root = Path.cwd().resolve()
    evidence = Path(sys.argv[1]).resolve()
    gate = json.loads((evidence / "ENGINEERING_GATE.json").read_text())
    if not gate.get("passed") or gate.get("estimator") != "NOT_RUN":
        raise SystemExit("R05_ENGINEERING_GATE_NOT_PASSED")
    if list(evidence.glob("attempts/*/TICKET.json")):
        raise SystemExit("R05_SCIENCE_TICKET_ALREADY_CONSUMED")
    for row in gate["identities"]:
        path = Path(row["path"])
        if hashlib.sha256(path.read_bytes()).hexdigest() != row["sha256"]:
            raise SystemExit("IDENTITY_CHANGED_AFTER_GATE:" + str(path))

    attempt = prepare_attempt(evidence, "attempt1")
    ticket = {
        "schema": "T10_A19_R05_AUTO_SCORE_FINAL_TICKET_V1",
        "attempt": "attempt1",
        "role": "development",
        "scenario": "A10_FROZEN_P1_STEP_SEED10101",
        "initialization": "ORIGINAL_RAW_NO_CHECKPOINT",
        "truth_gt": "FORBIDDEN_FROM_ESTIMATOR",
        "stage1_outer_max": 500,
        "conditional_block_iteration_max": 50,
        "stage2_outer_max": 200,
        "automatic_hard_timeout_seconds": 900,
        "per_final_hard_timeout_seconds": 900,
        "final_total_hard_budget_seconds": 4500,
        "science_total_hard_budget_seconds": 5400,
        "actual_outer_process_tree_timeout_seconds": 5400,
        "automatic_enforcement": "RUNNER_SIGALRM_900",
        "final_enforcement": "RUNNER_INDEPENDENT_PROCESS_GROUP_WATCHDOG_900_EACH",
        "retry": "FORBIDDEN_AFTER_ALGORITHM_START",
        "identity_schema": "A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY",
        "tau_eta": 0.10,
        "tau_s_m": 0.10,
        "tau_gamma": 1.0,
        "epsilon_bad_m": 0.20,
        "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    }
    write_json_atomic(attempt["root"] / "TICKET.json", ticket)
    runner = evidence / "a19_r05_pipeline_final"
    config = root / "doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/P1_step_seed10101_conditional.yaml"
    command = [
        sys.executable, "tools/paper/a14_run_logged.py", "--record",
        str(attempt["root"] / "estimator_run"), "--seconds", "5400", "--",
        str(runner), "--policy",
        "PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1",
        "development-stage1-stage2-score", str(config),
        "ORIGINAL_RAW_NO_CHECKPOINT", str(attempt["output_leaf"]),
    ]
    write_json_atomic(attempt["root"] / "COMMAND.json", {"argv": command})
    completed = subprocess.run(command)
    actual = json.loads((attempt["root"] / "estimator_run/command.json").read_text())
    write_json_atomic(attempt["root"] / "COMPLETION.json", {
        "schema": "A19_R05_COMPLETION_V1",
        "attempt": "attempt1",
        "wrapper_exit": completed.returncode,
        "actual_exit": actual["exit_code"],
        "external_wall_s": actual["external_wall_s"],
        "forbidden_input_open_lines": actual["forbidden_input_open_lines"],
    })
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
