#!/usr/bin/env python3
"""Evaluate the frozen A14 engineering gate before the sole pilot invocation."""
import datetime,hashlib,json,sys,xml.etree.ElementTree as E
from pathlib import Path
r=Path(sys.argv[1]);checks=[]
def check(name,passed,**detail):checks.append(dict(name=name,passed=bool(passed),**detail))
static=json.loads((r/'static_final/ENGINEERING_STATIC_CHECKS.json').read_text());check('static_all_frozen_checks',static['pass'],count=static['count'])
reg=json.loads((r/'REGRESSION_EXITS.json').read_text())
for x in reg:check(x['name'],x['exit_code']==0,exit_code=x['exit_code'])
count=0
for p in r.glob('test_*.xml'):
 root=E.parse(p).getroot();count+=int(root.attrib['tests']);check(p.name,int(root.attrib['failures'])==0 and int(root.attrib['errors'])==0)
for name in ['build_core_runner','build_probe_object','link_probe','build_tests','build_probe_object_final','link_probe_final','build_affected_runners']:
 p=r/'commands'/name/'command.json';v=json.loads(p.read_text()) if p.exists() else {};check(name,v.get('exit_code')==0,exit_code=v.get('exit_code'))
core=Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so');h=hashlib.sha256(core.read_bytes()).hexdigest()
for name in ['static_probe_final']+[x['name'] for x in reg if x['name']!='test_t09_runner_contract']:
 v=json.loads((r/'commands'/name/'command.json').read_text());mapped=[x for x in v['elf_identity'] if x['mapped'] and Path(x['realpath']).name=='libuwb_imu_fgo.so'];check(name+'_actual_final_core',bool(mapped) and all(x['sha256']==h for x in mapped))
for p in (r/'commands').glob('*/command.json'):
 v=json.loads(p.read_text());check(str(p.parent.name)+'_truth_isolation',not v.get('forbidden_input_open_lines',[]))
for row in json.loads((r/'PRE_CHANGE_IDENTITY.json').read_text()):
 s=row['path']
 if '/inputs/raw/step/' in s or 'P1_step_seed10101.yaml' in s or s=='/usr/local/lib/libgtsam.so.4.2.0':
  check('unchanged_'+s,hashlib.sha256(Path(s).read_bytes()).hexdigest()==row['sha256'])
runner=Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner')
result={'schema':'a14_engineering_gate_v1','passed':all(x['passed'] for x in checks),'registered_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'gtest_count':count,'static_check_count':static['count'],'core_sha256':h,'runner_sha256':hashlib.sha256(runner.read_bytes()).hexdigest(),'checks':checks,'failed':[x for x in checks if not x['passed']]}
with (r/'ENGINEERING_GATE.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in result.items() if k!='checks'},indent=2))
raise SystemExit(0 if result['passed'] else 1)
