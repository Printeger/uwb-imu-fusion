#!/usr/bin/env python3
"""Read-only A12 boundaries, budgets, artifacts and actual runtime identity checks."""
import csv,hashlib,json,re,subprocess,sys
from pathlib import Path

def rows(p):
 with Path(p).open() as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main(root):
 r=Path(root);failed=[];checks={}
 def ck(name,test):
  checks[name]=bool(test)
  if not test:failed.append(name)
 # Original source/config/raw and key runtime binaries only. No truth file reads/hashes.
 pre=json.loads((r/'PRE_RUN_IDENTITY.json').read_text());ck('frozen_source_config_raw_runtime_unchanged',all(sha(x['source'])==x['sha256'] for x in pre))
 ck('restoration_pass',json.loads((r/'RESTORATION.json').read_text())['passed'])
 ck('static_conditional_gate',json.loads((r/'STATIC_AUDIT.json').read_text())['supports_single_comparison'])
 arm=json.loads((r/'ARM_AUDIT.json').read_text());ck('A_reference_exact_reproduction',arm['A_reproduces_A11'])
 dif=json.loads((r/'ARM_A_REPRODUCTION_DIFFERENCES.json').read_text());ck('all_A_differences_exact_zero',all(v==0 for row in dif for k,v in row.items() if k!='call'))
 ck('A_B_comparable',arm['comparable'])
 ck('negative_comparison_retained',not arm['B_local_fixed_budget_improvement'] and not arm['A']['converged_generic_AND_stationarity'] and not arm['B']['converged_generic_AND_stationarity'])
 def mapping(p):return {x['name']:x['value'] for x in rows(p)}
 am,bm=mapping(r/'A/lm_parameters.csv'),mapping(r/'B/lm_parameters.csv');ck('only_diagonalDamping_parameter_differs',set(am)==set(bm) and {k for k in am if am[k]!=bm[k]}=={'mode','diagonalDamping'} and am['diagonalDamping']=='0' and bm['diagonalDamping']=='1')
 ia,ib,st=[mapping(r/p/'restoration_identity.csv') for p in ['A','B','static']];ck('same_checkpoint_graph_values_lambda',all(ia[k]==ib[k]==st[k] for k in ['checkpoint_graph_sha','checkpoint_values_sha','checkpoint_error','initial_graph_sha','initial_values_sha']) and am['lambdaInitial']==bm['lambdaInitial']=='0.10000000000000006')
 command_records=[json.loads(p.read_text()) for p in sorted((r/'commands').glob('*/command.json'))];run_records=[x for x in command_records if '/usr/bin/time' in x['argv'] and any(Path(y).name=='a12_checkpoint_probe' for y in x['argv'])];ck('exactly_two_static_setup_and_two_arm_processes',len(run_records)==4)
 for mode in ['A','B']:
  trace=rows(r/mode/'trace.csv');rec=json.loads((r/'commands'/('arm_'+mode)/'command.json').read_text());ck(mode+'_one_run_bounded',rec['exit_code']==0 and len(trace)==150 and float(trace[-1]['elapsed_seconds'])<30 and '30s' in rec['argv'])
  ck(mode+'_complete_finite_trace',all(all(v not in ('nan','inf','-inf','') for v in row.values()) for row in trace))
  ck(mode+'_generic_AND_stationarity_not_met',not (trace[-1]['generic']=='1' and trace[-1]['stationary']=='1'))
 ck('B_authorized_after_A_audit',json.loads((r/'commands/audit_A/command.json').read_text())['finished_utc']<json.loads((r/'commands/arm_B/command.json').read_text())['started_utc'])
 opens={};mapped={};identities=[]
 forbidden=re.compile(r'(^|/)(evaluation|truth|gt)(/|\.|$)',re.I)
 for mode in ['failed_setup/static','static','A','B']:
  lines=(r/mode/'file_access.trace').read_text().splitlines();paths=[]
  for line in lines:
   if 'openat(' in line or 'open(' in line:
    matches=re.findall(r'"([^"\n]*)"',line)
    if matches:paths.append(matches[0])
  opens[mode]=sorted(set(paths));ck(mode+'_no_truth_or_other_scene_open',not any(forbidden.search(p) or '/raw/los/' in p or '/raw/ramp/' in p or 'P2_' in p or 'P3_' in p for p in paths))
  libs=sorted(set(line.split()[5] for line in (r/mode/'process_maps.txt').read_text().splitlines() if len(line.split())>=6 and line.split()[5].startswith('/') and '.so' in line.split()[5]));mapped[mode]=libs
 ck('same_actual_mapped_library_paths',mapped['static']==mapped['A']==mapped['B']==mapped['failed_setup/static'])
 for path in mapped['A']:
  notes=subprocess.check_output(['readelf','-n',path],text=True);identities.append({'realpath':str(Path(path).resolve()),'sha256':sha(path),'bytes':Path(path).stat().st_size,'build_id':[line.strip() for line in notes.splitlines() if 'Build ID:' in line]})
 expected={'libgtsam.so.4.2.0':'00d83aa618194aefc4b2011e1d29bd9aba107a8b5e0ee8e946eb2455351219da','libuwb_imu_fgo.so':'7b7ac49187c0f61cd739a22a30291830a5ce5e32af36ede61c8031e157ff96a6'}
 for name,h in expected.items():ck(name+'_same_as_A11',any(Path(x['realpath']).name==name and x['sha256']==h for x in identities))
 for path in r.glob('*PRERUN_SHA256.json'):
  if path.name=='STATIC_PRERUN_SHA256.json':continue # v1 intentionally retained separately.
  ck(path.name+'_sealed_match',all(sha(r/p)==h for p,h in json.loads(path.read_text()).items()))
 ck('failed_setup_retained',json.loads((r/'commands/static_probe/command.json').read_text())['exit_code']==1 and (r/'failed_setup/a12_checkpoint_probe_v1.cpp').is_file())
 result={'passed':not failed,'checks':checks,'failures':failed,'static_iterate_calls':0,'arm_iterate_calls':300,'estimator_arm_processes':2,'truth_read_or_hash':'NOT_PERFORMED','production_strategy_change':'NONE','chain':'NOT_RUN','Stage2':'NOT_RUN','gate':'NOT_RUN','held_out':'NOT_RUN','scheduler':'NOT_IMPLEMENTED','C1_C2_C3':'NOT_UPGRADED','original_stationarity_failures':['A','B'],'unavailable_counts':'NOT_EVALUATED_NO_STAGE2'}
 for name,obj in [('CLOSEOUT_CHECK.json',result),('OPENED_FILES.json',opens),('FINAL_MAPPED_LIBRARIES.json',identities),('COMMAND_INDEX.json',command_records)]: (r/name).write_text(json.dumps(obj,indent=2)+'\n')
 print(json.dumps(result,indent=2));return 0 if not failed else 1
if __name__=='__main__':sys.exit(main(sys.argv[1]))
