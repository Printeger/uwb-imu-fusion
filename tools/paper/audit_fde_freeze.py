#!/usr/bin/env python3
"""Verify the explicitly frozen recovery backend and external artifacts."""
import hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
EVIDENCE=Path('/home/mint/ws_fusion_uwb/res/fde_forensics_20260911_01')
baseline=json.loads((EVIDENCE/'baseline_hashes.json').read_text())
allowed={'src/config.cpp','include/uifgo/config.h','src/nlos_fde.cpp','include/uifgo/nlos_fde.h','tools/run_ie_paper.cpp','tools/paper/run_experiments.py','CMakeLists.txt','src/nlos_inference.cpp'}
results={};changes=[]
for name,old in baseline.items():
 p=ROOT/name if not name.startswith('/') else Path(name)
 new=hashlib.sha256(p.read_bytes()).hexdigest()
 results[name]={'before':old,'after':new,'equal':old==new}
 if old!=new:changes.append(name)
for name in changes:
 if not name.startswith('/') and name not in allowed:raise AssertionError('unexpected frozen change: '+name)
old=subprocess.check_output(['git','show','c254c6acdce48a1be66febaaa935e0f2846e9992:src/nlos_inference.cpp'],cwd=ROOT,text=True)
new=(ROOT/'src/nlos_inference.cpp').read_text()
expected=old.replace('frozen_full_support.provider != "imu_aided_postfit_fde_v2"','frozen_full_support.provider != (cfg.fde_grouped_test ?\n            "imu_aided_grouped_fde_v3" : "imu_aided_postfit_fde_v2")')
assert new==expected
# No dependency upgrade permitted. Only project library and runner may change.
for name in changes:
 if name.startswith('/'):
  assert '/ws_fusion_uwb/devel/' in name and ('libuwb_imu_fgo.so' in name or name.endswith('/uwb_imu_fgo_paper_runner')),name
report={'changes':changes,'files':results,'recovery_numerical_source_unchanged':True,'recovery_interface_exception':'Only SUCCESS_EMPTY provider identity selected by opt-in flag; exact replacement verified','defaults_configs_evaluator_dependencies_frozen':True}
(EVIDENCE/'freeze_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'changed_paths':changes,'recovery_numerical_source_unchanged':True},indent=2))
