#!/usr/bin/env python3
"""Frozen gate + one fresh Stage1 process tree. No retry or adaptive budget."""
import sys,json,datetime,hashlib,shutil,subprocess
from pathlib import Path
E=Path(sys.argv[1]).resolve();root=Path.cwd();cfg=root/'doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/P1_step_seed10101_conditional.yaml';checks={}
for name in ['build_core_runner_tests','final_stage1_compile','final_stage1_link','tests_final_compile','tests_final_link','cpp_tests_final_run','static_native_exact_run','static_checks_2_run','nonzero_old_check_run','legacy_discovery_tests','static_batch_2_run']:
 p=E/name/'command.json';checks[name]=p.exists() and json.loads(p.read_text())['exit_code']==0
for name in ['negative_default_off','negative_illegal_policy','negative_scope','negative_warm_start']:
 checks[name]=json.loads((E/name/'command.json').read_text())['exit_code']==2
checks['43_pairs']=json.loads((E/'static_batch_2_checks.json').read_text())['passed']
checks['nonzero']=json.loads((E/'NONZERO_GATE.json').read_text())['passed']
v=json.loads((E/'static_batch_2_run/command.json').read_text());secs=(datetime.datetime.fromisoformat(v['finished_utc'])-datetime.datetime.fromisoformat(v['started_utc'])).total_seconds();checks['complete_batch_le_10_seconds']=secs<=10
checks['cpp_final_tests']='ALL_PASS' in (E/'cpp_tests_final_run/stdout.log').read_text()
checks['source_frozen_protocol']=hashlib.sha256((E/'PREREGISTERED_PROTOCOL.md').read_bytes()).digest()==hashlib.sha256((root/'doc/ie_sprint/T10_A18_AMENDMENT_PROTOCOL.md').read_bytes()).digest()
checks['truth_isolation']=all(not json.loads(p.read_text()).get('forbidden_input_open_lines',[]) for p in E.glob('*/command.json'))
gate={'passed':all(checks.values()),'checks':checks,'batch_external_seconds':secs,'UTC':datetime.datetime.now(datetime.timezone.utc).isoformat()};(E/'ENGINEERING_GATE.json').write_text(json.dumps(gate,indent=2)+'\n');assert gate['passed'],gate
paths=[root/'CMakeLists.txt',root/'package.xml',root/'AGENTS.md',cfg]+list((root/'src').glob('*.cpp'))+list((root/'include/uifgo').glob('*.h'))+list((root/'tools/paper').glob('a18*'))+[root/'tools/paper'/n for n in ['a17_graph_io.h','a17_first_block.cpp','a17_certificate_worker.py','a16_reference.py','a16_mpfr.py','a14_run_logged.py','t07_run_logged_command.py']]+[root/'tools/run_ie_paper.cpp',root/'test/test_nlos_discovery.cpp']+[root/'doc/ie_sprint'/n for n in ['METHOD_CONTRACT.md','EXPERIMENT_CONTRACT.md','T10_A18_AMENDMENT_PROTOCOL.md','T10_A17_AMENDMENT_PROTOCOL.md','T10_A16_SOLVER_AMENDMENT_DRAFT.md']]
paths+=list((root/'include/uifgo').glob('*.hpp'))
raw=root/'doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/inputs/raw/step';paths+=[raw/n for n in ['input_manifest.json','imu.csv','uwb_observations.csv']]
paths += [E/'a18_stage1',E/'a18_tests',E/'a18_static',Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so'),Path('/usr/local/lib/libgtsam.so.4.2.0'),Path('/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2'),Path('/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0')]
ids=[]
for p in paths:
 assert p.exists(),p
 h=hashlib.sha256(p.read_bytes()).hexdigest();dest=E/'pre_pilot_source'/str(p).lstrip('/');dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest);ids.append({'path':str(p),'sha256':h,'bytes':p.stat().st_size,'copy':str(dest.relative_to(E))})
(E/'PRE_PILOT_IDENTITIES.json').write_text(json.dumps(ids,indent=2)+'\n')
with (E/'PILOT_ONCE_TICKET.json').open('x') as f:json.dump({'gate':gate,'UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scenario':'P1 step seed10101','initialization':'ORIGINAL_RAW_NO_CHECKPOINT','outer_max':500,'conditional_max':50,'process_tree_seconds':900,'max_processes':1,'Stage2':'NOT_RUN'},f,indent=2)
cmd=[sys.executable,'tools/paper/a14_run_logged.py','--record',str(E/'pilot_run'),'--seconds','900','--',str(E/'a18_stage1'),'--policy','PAPER_CERTIFIED_PAIR_REDUCTION_V1','development-stage1',str(cfg),'NO_CHECKPOINT',str(E/'pilot')]
(E/'PILOT_COMMAND.json').write_text(json.dumps(cmd,indent=2)+'\n');result=subprocess.run(cmd)
(E/'PILOT_COMPLETION.json').write_text(json.dumps({'wrapper_exit':result.returncode,'actual':json.loads((E/'pilot_run/command.json').read_text())['exit_code'],'retry':'NOT_RUN'},indent=2)+'\n')
# A scientific failure is an outcome, never an instruction to rerun.
print('PILOT_FINISHED',result.returncode)
