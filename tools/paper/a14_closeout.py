#!/usr/bin/env python3
"""Archive and summarize the completed A14 gate and sole pilot, never rerun."""
import csv,datetime,difflib,hashlib,json,re,shutil,subprocess,sys
from pathlib import Path
r=Path(sys.argv[1]);run=r/'runs/P1_step_seed10101_conditional';cmd=json.loads((r/'commands/pilot_P1_step_seed10101/command.json').read_text());st=json.loads((run/'run_status.json').read_text());diag=json.loads((run/'discovery_failure_diagnostics.json').read_text());lm=diag['last_conditional_lm'];gate=json.loads((r/'ENGINEERING_GATE.json').read_text());common=json.loads((run/'common_preparation.json').read_text());inp=json.loads((run/'input_manifest.json').read_text());checks=[]
def check(name,ok,**detail):checks.append(dict(name=name,passed=bool(ok),**detail))
check('engineering_gate_preceded_pilot',gate['passed'] and gate['registered_utc']<cmd['started_utc'])
check('one_scientific_attempt',len(list((r/'commands').glob('pilot_*')))==1 and len(list((r/'runs').iterdir()))==1)
check('recorded_exit',cmd['exit_code']==st['exit_code']==1)
check('bounded_run',cmd['external_wall_s']<120 and cmd['timeout_s']==120)
check('original_four_AND_failure_not_overridden',not lm['converged'] and not lm['last_qualification_stationarity']['stationary'] and not st['stage2_refit_run'] and not st['gate_or_fallback_run'])
check('complete_trial_accounting',lm['lambda_trial_accounting_status']=='COMPLETE' and lm['accepted_update_count']+lm['rejected_lambda_trial_count']==lm['lambda_trial_count'] and lm['accepted_update_count']+lm['no_update_return_count']==lm['iterate_call_count'])
check('no_following_chain',diag['outer_trace_rows']==0 and diag['chain_results_retained']==0)
check('new_initial_graph_is_engineering_checked',common['graph_linearization_sha256']=='t08graphlin-sha256:99cdfbc7032cd3e2f6a2488b12d87b1219d75e60ba0c4f7ca80d56aff3c12710')
check('raw_initial_values_unchanged',common['values_sha256']=='t08values-sha256:7115e76d406123d3a30079dc115c6d3501ceca157d270eaced2d8be492b7cd3f')
check('raw_cache_identity_unchanged',inp['cache_id']=='sha256:d0dca8d096f06bc9022408fde98e41f7c8bff02b9428e866d63bf4a4409e26c0')
for name,key in [('libuwb_imu_fgo.so','core_sha256'),('uwb_imu_fgo_paper_runner','runner_sha256')]:
 rows=[x for x in cmd['elf_identity'] if x['mapped'] and Path(x['realpath']).name==name];check(name+'_actual_map_matches_gate',bool(rows) and all(x['sha256']==gate[key] for x in rows))
for p in (r/'commands').glob('*/command.json'):
 v=json.loads(p.read_text());check(p.parent.name+'_truth_isolation',not v.get('forbidden_input_open_lines',[]))
# All pre-existing raw files and the dependency stay unchanged. Do not read GT.
for row in json.loads((r/'PRE_CHANGE_IDENTITY.json').read_text()):
 s=row['path']
 if '/inputs/raw/step/' in s or 'P1_step_seed10101.yaml' in s or s=='/usr/local/lib/libgtsam.so.4.2.0':check('unchanged_'+s,hashlib.sha256(Path(s).read_bytes()).hexdigest()==row['sha256'])
resources=(r/'commands/pilot_P1_step_seed10101/resources.txt').read_text();rss=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',resources)[1])
summary={'schema':'a14_pilot_summary_v1','role':'development','scientific_run_count':1,'exit_code':1,'status':st['status'],'first_conditional_block':lm,'Stage1':{'status':st['discovery_status'],'reason':st['reason'],'completed_outer_iterations':diag['outer_trace_rows'],'chain_solve_count':diag['chain_results_retained']},'Stage2':{'status':'NOT_RUN','reason':'STAGE1_CONDITIONAL_LM_FAILED'},'candidate_observation_count':None,'segment_count':None,'group_count':None,'eligible_count':None,'score_unavailable_group_count':None,'unevaluated_count_reason':'NOT_EVALUATED_STAGE1_FAILED; serialized empty partition is a failure artifact, not a valid zero-candidate result','score_status':'NOT_RUN','Stage2_cache_status':'NOT_PRODUCED_STAGE1_FAILED','raw_cache_id':inp['cache_id'],'common_preparation_id':common['common_preparation_id'],'initial_values_sha256':common['values_sha256'],'model_identity':(run/'imu_covariance_model.txt').read_text().splitlines()[0],'external_wall_s':cmd['external_wall_s'],'runner_wall_s':st['elapsed_seconds'],'Stage1_wall_s':st['stage1_seconds'],'max_rss_kib':rss,'gate_final_validation_test_T11':'NOT_RUN','truth_GT':'NOT_READ','retry':'NOT_RUN','new_model_warm_start':'NONE_FROM_RAW','next_minimal_action_proposed_not_run':'At a separately authorized single new-model first-block checkpoint, capture actual last accepted Values/delta and audit native-retract directional derivatives plus roundoff and lambda-exhaustion branches. Do not change tolerance or damping based only on this terminal summary.'}
(r/'PILOT_SUMMARY.json').write_text(json.dumps(summary,indent=2)+'\n')
with (r/'PILOT_SUMMARY.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['run','role','exit','first_block_calls','accepted','rejected_trials','stage1','stage2','candidates','segments','groups','eligible','unavailable_groups','score','stage2_cache','wall_s','rss_kib']);w.writerow(['P1_step_seed10101','development',1,lm['iterate_call_count'],lm['accepted_update_count'],lm['rejected_lambda_trial_count'],st['discovery_status'],'NOT_RUN','NA','NA','NA','NA','NA','NOT_RUN','NOT_PRODUCED',cmd['external_wall_s'],rss])
(r/'CLOSEOUT_CHECKS.json').write_text(json.dumps({'passed':all(x['passed'] for x in checks),'checks':checks},indent=2)+'\n')
# Snapshot final source and affected binaries without collecting unrelated data.
paths=list(Path('include').rglob('*.h'))+list(Path('src').glob('*.cpp'))+list(Path('test').glob('*.cpp'))
paths += [Path('CMakeLists.txt'),Path('package.xml'),Path('AGENTS.md'),Path('tools/run_ie_paper.cpp'),Path('tools/run_offline.cpp'),Path('tools/paper/run_experiments.py'),Path('tools/paper/t07_run_logged_command.py')]
paths += list(Path('tools/paper').glob('*a14*.py'))+list(Path('tools/paper').glob('a14*.cpp'))
source_rows=[]
for p in paths:
 d=r/'after'/p;d.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,d);source_rows.append({'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(r/'FINAL_SOURCE_IDENTITY.json').write_text(json.dumps(source_rows,indent=2)+'\n')
bins=['/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so']
binroot=Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo')
bins += [str(binroot/n) for n in ['uwb_imu_fgo_paper_runner','uwb_imu_fgo_node','test_config','test_imu_preint','test_graph_builder','test_paper_input','test_paper_methods','test_paper_stage2_cache','test_nlos_discovery','test_nlos_refit','test_nlos_inference','test_nlos_recoverability']]
rows=[]
for b in bins:
 p=Path(b);d=r/'after/binaries'/p.name;d.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,d);notes=subprocess.check_output(['readelf','-n',str(p)],text=True);build=re.search(r'Build ID: (\S+)',notes)
 rows.append({'path':b,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'build_id':build[1] if build else None,'bytes':p.stat().st_size})
(r/'FINAL_BINARY_IDENTITY.json').write_text(json.dumps(rows,indent=2)+'\n')
# Minimal dependency source used in the model audit, read-only.
for s in ['/usr/local/include/gtsam/navigation/CombinedImuFactor.h','/usr/local/include/gtsam/navigation/PreintegrationParams.h','/usr/local/include/gtsam/config.h','/home/mint/dep/gtsam/gtsam/navigation/CombinedImuFactor.cpp','/home/mint/dep/gtsam/gtsam/navigation/TangentPreintegration.cpp','/home/mint/dep/gtsam/gtsam/navigation/PreintegrationBase.cpp','/home/mint/dep/gtsam/gtsam/linear/NoiseModel.cpp']:
 p=Path(s);d=r/'after/dependencies'/p.name;d.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,d)
failures=[]
for p in (r/'commands').glob('*/command.json'):
 v=json.loads(p.read_text())
 if v.get('exit_code')!=0:failures.append({'command_record':str(p),'exit_code':v.get('exit_code'),'reason':st['reason'] if p.parent.name=='pilot_P1_step_seed10101' else 'SEE_COMMAND_LOG'})
(r/'FAILURES.json').write_text(json.dumps(failures,indent=2)+'\n')
print(json.dumps({'closeout_passed':all(x['passed'] for x in checks),'pilot':summary,'failed_commands':failures},indent=2))
raise SystemExit(0 if all(x['passed'] for x in checks) else 1)
