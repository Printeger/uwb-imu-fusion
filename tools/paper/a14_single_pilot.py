#!/usr/bin/env python3
"""Only A14's predeclared one-process development pilot, no retries."""
import hashlib,json,subprocess,sys
from pathlib import Path
r=Path(sys.argv[1]).resolve();gate=json.loads((r/'ENGINEERING_GATE.json').read_text());assert gate['passed']
p=json.loads((r/'PILOT_PREREGISTRATION.json').read_text());cfg=Path(p['run_config']).resolve();assert hashlib.sha256(cfg.read_bytes()).hexdigest()==p['run_config_sha256']
runner=Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner');assert hashlib.sha256(runner.read_bytes()).hexdigest()==gate['runner_sha256']
assert hashlib.sha256(Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so').read_bytes()).hexdigest()==gate['core_sha256']
# Exclusive marker makes accidental rerun fail before launching a process.
with (r/'PILOT_STARTED.json').open('x') as f:json.dump({'attempt':1,'retry_allowed':False,'protocol':'PILOT_PREREGISTRATION.json'},f)
cmd=[sys.executable,'tools/paper/a14_run_logged.py','--record',str(r/'commands/pilot_P1_step_seed10101'),'--seconds','120','--',str(runner),'--config',str(cfg),'--output-root',str(r/'runs'),'--run-id','P1_step_seed10101_conditional']
raise SystemExit(subprocess.run(cmd).returncode)
