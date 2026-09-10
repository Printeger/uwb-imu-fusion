#!/usr/bin/env python3
"""R03 startup gate; this script never invokes the estimator."""
import hashlib, json, os, subprocess, sys, tempfile, uuid
from pathlib import Path
from a19_r03_launch import prepare_attempt, write_json_atomic

root = Path.cwd().resolve(); evidence = Path(sys.argv[1]).resolve()
runner = evidence / "a19_r03_pipeline"
config = root / "doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/P1_step_seed10101_conditional.yaml"
raw = root / "doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/inputs/raw/step"
runtime = [Path("/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so"),
           Path("/usr/local/lib/libgtsam.so.4.2.0"),
           Path("/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2"),
           Path("/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0")]
required = [runner, config, raw/"input_manifest.json", raw/"imu.csv",
            raw/"uwb_observations.csv", *runtime]
checks = {"required_paths": all(p.is_file() for p in required)}
rows = [{"path": str(p), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
        for p in required]
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
checks["dynamic_library_resolution"] = all(resolved.get(k) == v for k,v in expected.items())
with tempfile.TemporaryDirectory(prefix="a19-r03-preflight-") as td:
    td=Path(td); sandbox=td/"evidence"; sandbox.mkdir()
    prepared=prepare_attempt(sandbox,"attempt1")
    checks["isolated_parent_writable"] = prepared["pilot_parent"].is_dir() and os.access(prepared["pilot_parent"],os.W_OK)
    checks["atomic_status_strict_json"] = json.loads(prepared["status"].read_text())["estimator"] == "NOT_RUN" and not prepared["status"].with_name("launch_status.json.tmp").exists()
    log=prepared["logs"]/"probe.log"; log.write_text("ok\n"); checks["log_creation"] = log.read_text()=="ok\n"
    prepared["output_leaf"].mkdir(); sentinel=prepared["output_leaf"]/"sentinel"; sentinel.write_text("preserve")
    try: prepared["output_leaf"].mkdir(); duplicate=False
    except FileExistsError: duplicate=True
    try: prepare_attempt(sandbox,"attempt1"); attempt_duplicate=False
    except FileExistsError: attempt_duplicate=True
    checks["fresh_leaf_no_overwrite"] = duplicate and attempt_duplicate and sentinel.read_text()=="preserve"
    logger=root/"tools/paper/a14_run_logged.py"
    record=td/"exit"; proc=subprocess.run([sys.executable,str(logger),"--record",str(record),"--seconds","5","--","bash","-c","exit 7"])
    meta=json.loads((record/"command.json").read_text()); checks["exit_code_accounting"] = proc.returncode==7 and meta["exit_code"]==7 and meta["maps_captured"]
    marker="a19-r03-tree-"+uuid.uuid4().hex; record=td/"timeout"
    proc=subprocess.run([sys.executable,str(logger),"--record",str(record),"--seconds","1","--","bash","-c",f"python3 -c 'import time; time.sleep(30)' {marker} & wait"])
    meta=json.loads((record/"command.json").read_text()); live=False
    for cmdline in Path("/proc").glob("[0-9]*/cmdline"):
        try: live = live or marker.encode() in cmdline.read_bytes()
        except (OSError, ProcessLookupError): pass
    checks["timeout_and_process_tree_accounting"] = proc.returncode==137 and meta["exit_code"]==137 and not live and meta["maps_captured"]
result={"schema":"A19_R03_STARTUP_PREFLIGHT_V1","estimator":"NOT_RUN",
        "passed":all(checks.values()),"checks":checks,"identities":rows,
        "ldd":ldd.splitlines(),"ldd_realpaths":resolved,"expected_runtime":expected}
write_json_atomic(evidence/"STARTUP_PREFLIGHT.json",result)
print(json.dumps({"passed":result["passed"],"checks":checks}))
raise SystemExit(0 if result["passed"] else 1)
