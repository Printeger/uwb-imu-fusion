#!/usr/bin/env python3
"""Consume the A19-R01 one-shot ticket and run one bounded estimator tree."""
import datetime
import hashlib
import json
import subprocess
import sys
from pathlib import Path

root = Path.cwd()
evidence = Path(sys.argv[1]).resolve()
gate_path = evidence / "ENGINEERING_GATE.json"
gate = json.loads(gate_path.read_text())
if gate.get("schema") != "A19_R01_ENGINEERING_GATE_V1" or not gate.get("passed"):
    raise SystemExit("A19-R01 gate is not passed")
for item in json.loads((evidence / "PRE_PILOT_IDENTITIES.json").read_text()):
    path = Path(item["path"])
    if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
        raise SystemExit(f"identity changed after gate: {path}")

config = root / ("doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/"
                 "P1_step_seed10101_conditional.yaml")
config_text = config.read_text()
for frozen in ("lm_max_iter: 50", "discovery_max_outer_iterations: 500",
               "max_refit_iterations: 200"):
    if frozen not in config_text:
        raise SystemExit(f"frozen budget missing: {frozen}")
pilot_parent = evidence / "pilot"
if pilot_parent.exists() or (evidence / "pilot_run").exists():
    raise SystemExit("pilot output or command record already exists")
# The runner intentionally requires its leaf output directory to be new, but
# std::filesystem::create_directory cannot create a missing parent.  Prepare
# only the isolated parent before consuming a future one-shot ticket.
pilot_parent.mkdir(exist_ok=False)

ticket = {
    "schema": "A19_R01_PILOT_ONCE_TICKET_V1",
    "gate_sha256": hashlib.sha256(gate_path.read_bytes()).hexdigest(),
    "identity_sha256": hashlib.sha256(
        (evidence / "PRE_PILOT_IDENTITIES.json").read_bytes()).hexdigest(),
    "scenario": "A10_FROZEN_P1_STEP_SEED10101",
    "initialization": "ORIGINAL_RAW_NO_CHECKPOINT",
    "stage1_outer_max": 500,
    "conditional_block_iteration_max": 50,
    "stage2_outer_max": 200,
    "estimator_process_tree_hard_timeout_seconds": 900,
    "authorized_estimator_process_trees": 1,
    "retry": "FORBIDDEN",
    "truth_gt": "FORBIDDEN",
    "after_first_failure_or_scoring": "STOP",
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
    "estimator_process_trees": 1,
    "retry": "NOT_RUN",
}, indent=2) + "\n")
raise SystemExit(completed.returncode)
