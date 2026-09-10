#!/usr/bin/env python3
"""Consume the only R03 ticket and run its bounded science process tree."""
import datetime, hashlib, json, subprocess, sys
from pathlib import Path
from a19_r03_launch import prepare_attempt, write_json_atomic

root=Path.cwd().resolve(); evidence=Path(sys.argv[1]).resolve(); attempt=sys.argv[2]
preflight=json.loads((evidence/"STARTUP_PREFLIGHT.json").read_text())
if not preflight.get("passed") or preflight.get("estimator") != "NOT_RUN": raise SystemExit("STARTUP_PREFLIGHT_NOT_PASSED")
existing=list(evidence.glob("attempts/*/TICKET.json"))
if attempt == "attempt1":
    if existing: raise SystemExit("R03_ATTEMPT1_ALREADY_CONSUMED")
elif attempt == "attempt2":
    prior=evidence/"attempts/attempt1"
    completion=json.loads((prior/"COMPLETION.json").read_text())
    stderr=(prior/"estimator_run/stderr.log").read_text()
    if (len(existing) != 1 or completion["external_wall_s"] >= 1.0 or
            (prior/"pilot/output").exists() or
            "A19_DEFAULT_OFF_OR_INVALID_POLICY" not in stderr):
        raise SystemExit("ATTEMPT2_REQUIRES_CONFIRMED_PREINITIALIZATION_INFRA_FAILURE")
else:
    raise SystemExit("R03_ATTEMPT_ID_REJECTED")
for row in preflight["identities"]:
    p=Path(row["path"])
    if hashlib.sha256(p.read_bytes()).hexdigest()!=row["sha256"]: raise SystemExit("IDENTITY_CHANGED_AFTER_PREFLIGHT:"+str(p))
prepared=prepare_attempt(evidence,attempt)
ticket={"schema":"A19_R03_AUTO_SCORE_AND_FINAL_TICKET_V1","attempt":attempt,
        "scenario":"A10_FROZEN_P1_STEP_SEED10101","initialization":"ORIGINAL_RAW_NO_CHECKPOINT",
        "stage1_outer_max":500,"conditional_block_iteration_max":50,"stage2_outer_max":200,
        "automatic_hard_timeout_seconds":900,"per_final_hard_timeout_seconds":900,
        "final_total_hard_budget_seconds":4500,"science_total_hard_budget_seconds":5400,
        "actual_shared_process_tree_timeout_seconds":900,"truth_gt":"FORBIDDEN",
        "retry":"FORBIDDEN_AFTER_ALGORITHM_START","utc":datetime.datetime.now(datetime.timezone.utc).isoformat()}
write_json_atomic(prepared["root"]/"TICKET.json",ticket)
runner=evidence/"a19_r03_pipeline"; config=root/"doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/P1_step_seed10101_conditional.yaml"
command=[sys.executable,"tools/paper/a14_run_logged.py","--record",str(prepared["root"]/"estimator_run"),"--seconds","900","--",str(runner),"--policy","PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1","development-stage1-stage2-score",str(config),"ORIGINAL_RAW_NO_CHECKPOINT",str(prepared["output_leaf"])]
write_json_atomic(prepared["root"]/"COMMAND.json",{"argv":command})
completed=subprocess.run(command); actual=json.loads((prepared["root"]/"estimator_run/command.json").read_text())
write_json_atomic(prepared["root"]/"COMPLETION.json",{"schema":"A19_R03_COMPLETION_V1","attempt":attempt,"wrapper_exit":completed.returncode,"actual_exit":actual["exit_code"],"external_wall_s":actual["external_wall_s"],"forbidden_input_open_lines":actual["forbidden_input_open_lines"]})
raise SystemExit(completed.returncode)
