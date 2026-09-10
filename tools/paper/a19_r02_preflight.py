#!/usr/bin/env python3
"""Run the A19-R02 startup gate without invoking the estimator."""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import uuid
from pathlib import Path

from a19_r02_launch import prepare_attempt, write_json_atomic

root = Path.cwd().resolve()
evidence = Path(sys.argv[1]).resolve()
r01 = root / "doc/ie_sprint/evidence/t10_a19_r01_stage2_integration_20260910T024847Z"
runner = evidence / "a19_stage2_score"
config = root / ("doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/"
                 "P1_step_seed10101_conditional.yaml")
raw = root / ("doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/"
              "inputs/raw/step")
required = [runner, config, raw / "input_manifest.json", raw / "imu.csv",
            raw / "uwb_observations.csv",
            Path("/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so"),
            Path("/usr/local/lib/libgtsam.so.4.2.0"),
            Path("/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2"),
            Path("/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0")]
checks = {"required_paths": all(path.is_file() for path in required)}

r01_ids = {item["path"]: item for item in json.loads(
    (r01 / "PRE_PILOT_IDENTITIES.json").read_text())}
identity_pairs = {
    runner: r01 / "a19_stage2_score",
    required[5]: required[5], required[6]: required[6],
    required[7]: required[7], required[8]: required[8], config: config,
    raw / "input_manifest.json": raw / "input_manifest.json",
    raw / "imu.csv": raw / "imu.csv",
    raw / "uwb_observations.csv": raw / "uwb_observations.csv",
}
identity_rows = []
for current, r01_path in identity_pairs.items():
    current_hash = hashlib.sha256(current.read_bytes()).hexdigest()
    if current == runner:
        expected_hash = hashlib.sha256(r01_path.read_bytes()).hexdigest()
    else:
        expected_hash = r01_ids[str(r01_path.resolve())]["sha256"]
    identity_rows.append({"path": str(current), "sha256": current_hash,
                          "expected_sha256": expected_hash,
                          "matches_r01": current_hash == expected_hash})
checks["r01_unchanged_identities"] = all(row["matches_r01"] for row in identity_rows)

ldd = subprocess.run(["ldd", str(runner)], text=True, capture_output=True,
                     check=True).stdout
ldd_realpaths = {}
for line in ldd.splitlines():
    fields = line.strip().split()
    if len(fields) >= 3 and fields[1] == "=>" and fields[2].startswith("/"):
        ldd_realpaths[fields[0]] = str(Path(fields[2]).resolve())
expected_runtime = {
    "libuwb_imu_fgo.so": str(required[5].resolve()),
    "libgtsam.so.4": str(required[6].resolve()),
    "libmpfr.so.6": str(required[7].resolve()),
    "libgmp.so.10": str(required[8].resolve()),
}
checks["dynamic_library_resolution"] = all(
    ldd_realpaths.get(name) == path for name, path in expected_runtime.items())

with tempfile.TemporaryDirectory(prefix="a19-r02-preflight-") as temporary:
    temporary = Path(temporary)
    sandbox = temporary / "evidence"
    sandbox.mkdir()
    prepared = prepare_attempt(sandbox, "attempt1")
    checks["isolated_parent_writable"] = (
        prepared["pilot_parent"].is_dir() and
        os.access(prepared["pilot_parent"], os.W_OK))
    payload = json.loads(prepared["status"].read_text())
    checks["atomic_status_strict_json"] = (
        payload["estimator"] == "NOT_RUN" and
        not prepared["status"].with_name("launch_status.json.tmp").exists())
    log = prepared["logs"] / "probe.log"
    log.write_text("preflight-log-ok\n")
    checks["log_creation"] = log.read_text() == "preflight-log-ok\n"
    prepared["output_leaf"].mkdir()
    sentinel = prepared["output_leaf"] / "sentinel"
    sentinel.write_text("preserve")
    duplicate_rejected = False
    try:
        prepared["output_leaf"].mkdir()
    except FileExistsError:
        duplicate_rejected = True
    attempt_rejected = False
    try:
        prepare_attempt(sandbox, "attempt1")
    except FileExistsError:
        attempt_rejected = True
    checks["fresh_leaf_no_overwrite"] = (
        duplicate_rejected and attempt_rejected and
        sentinel.read_text() == "preserve")

    logger = root / "tools/paper/a14_run_logged.py"
    exit_record = temporary / "exit_record"
    exit_probe = subprocess.run([
        sys.executable, str(logger), "--record", str(exit_record),
        "--seconds", "5", "--", "bash", "-c", "exit 7"])
    exit_meta = json.loads((exit_record / "command.json").read_text())
    checks["exit_code_accounting"] = (
        exit_probe.returncode == 7 and exit_meta["exit_code"] == 7 and
        exit_meta["maps_captured"])

    marker = "a19-r02-tree-" + uuid.uuid4().hex
    timeout_record = temporary / "timeout_record"
    timeout_probe = subprocess.run([
        sys.executable, str(logger), "--record", str(timeout_record),
        "--seconds", "1", "--", "bash", "-c",
        f"python3 -c 'import time; time.sleep(30)' {marker} & wait"])
    timeout_meta = json.loads((timeout_record / "command.json").read_text())
    live_marker = False
    for proc in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            if marker.encode() in proc.read_bytes():
                live_marker = True
                break
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    checks["timeout_and_process_tree_accounting"] = (
        timeout_probe.returncode == 137 and timeout_meta["exit_code"] == 137 and
        not live_marker and timeout_meta["maps_captured"])

result = {
    "schema": "A19_R02_STARTUP_PREFLIGHT_V1",
    "estimator": "NOT_RUN",
    "passed": all(checks.values()),
    "checks": checks,
    "identities": identity_rows,
    "ldd": ldd.splitlines(),
    "ldd_realpaths": ldd_realpaths,
    "expected_runtime": expected_runtime,
}
write_json_atomic(evidence / "STARTUP_PREFLIGHT.json", result)
print(json.dumps({"passed": result["passed"], "checks": checks}))
raise SystemExit(0 if result["passed"] else 1)
