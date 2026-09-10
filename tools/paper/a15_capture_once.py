#!/usr/bin/env python3
"""Exclusive A15 single-attempt capture; never replays or retries."""
import hashlib,json,subprocess,sys,datetime,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
E=ROOT/'doc/ie_sprint/evidence/t10_a15_terminal_numeric_20260909T152633Z'
A14=ROOT/'doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z'
C=A14/'P1_step_seed10101_conditional.yaml'
RUNNER=Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner')
CORE=RUNNER.parents[1]/'libuwb_imu_fgo.so'
GTSAM=Path('/usr/local/lib/libgtsam.so.4.2.0')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 assert sha(C)=='479df3daae2096bb665582f60611c66841ed87edc496ddcbaf7cc9e180ed6c4f'
 assert sha(GTSAM)=='00d83aa618194aefc4b2011e1d29bd9aba107a8b5e0ee8e946eb2455351219da'
 assert json.loads((E/'build_final_sequential/command.json').read_text())['exit_code']==0
 pre={}
 for p in [C,RUNNER,CORE,GTSAM,ROOT/'tools/run_ie_paper.cpp',ROOT/'src/nlos_solver_utils.cpp',ROOT/'src/nlos_discovery.cpp',ROOT/'include/uifgo/nlos_solver_utils.h',ROOT/'doc/ie_sprint/T10_A15_PROTOCOL.md']:
  pre[str(p)]={'sha256':sha(p),'bytes':p.stat().st_size}
 raw=ROOT/'doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/inputs/raw/step'
 for p in sorted(raw.iterdir()):
  if not p.is_file():continue
  assert p.name in ['input_manifest.json','imu.csv','uwb_observations.csv'],p
  pre[str(p)]={'sha256':sha(p),'bytes':p.stat().st_size}
  d=E/'inputs'/p.name;d.parent.mkdir(exist_ok=True);shutil.copy2(p,d)
 shutil.copy2(C,E/'inputs/config_original.yaml')
 (E/'capture_preflight_identity.json').write_text(json.dumps(pre,indent=2)+'\n')
 # Mark consumed BEFORE spawning the single estimator. Failure never releases it.
 with (E/'CAPTURE_STARTED.json').open('x') as f:json.dump({'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'estimator_process_limit':1,'hard_limit_s':120,'retry':False},f)
 cmd=[sys.executable,str(ROOT/'tools/paper/a14_run_logged.py'),'--record',str(E/'capture_command'),'--seconds','120','--',str(RUNNER),'--config',str(C),'--output-root',str(E/'runs'),'--run-id','P1_step_seed10101_terminal_capture','--diagnostic-conditional-lm-outer','1','--diagnostic-passive-terminal-capture']
 (E/'capture_wrapper_argv.json').write_text(json.dumps(cmd,indent=2)+'\n')
 result=subprocess.run(cmd,cwd=ROOT)
 (E/'CAPTURE_FINISHED.json').write_text(json.dumps({'wrapper_exit_code':result.returncode,'retry':False})+'\n')
 return result.returncode
if __name__=='__main__':sys.exit(main())
