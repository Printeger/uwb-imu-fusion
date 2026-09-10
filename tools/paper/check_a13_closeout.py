#!/usr/bin/env python3
"""Verify A13 scope, provenance and frozen numerical evidence; no estimator run."""
import csv,hashlib,json,re,subprocess,sys
from pathlib import Path

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main(root):
 r=Path(root);checks={};fail=[]
 def ck(n,v):
  checks[n]=bool(v)
  if not v:fail.append(n)
 pre=json.loads((r/'PRE_RUN_IDENTITY.json').read_text());ck('original_source_raw_config_contracts_runtime_unchanged',all(sha(x['source'])==x['sha256'] for x in pre))
 sealed=json.loads((r/'PRERUN_SHA256.json').read_text());ck('preregistered_code_reference_protocol_sealed',all(sha(r/p)==h for p,h in sealed.items()))
 av2=json.loads((r/'ANALYSIS_V2_PRERUN.json').read_text());ck('analysis_v2_sealed',sha(r/av2['path'])==av2['sha256'] and sha('tools/paper/audit_a13_covariance.py')==av2['sha256'])
 with (r/'static/restoration_identity.csv').open() as f:identity={x['name']:x['value'] for x in csv.DictReader(f)}
 ck('original_graph_and_values_gate',identity['restoration_gate']=='PASS' and identity['checkpoint_graph_sha']==identity['after_graph_sha'] and identity['checkpoint_values_sha']==identity['after_values_sha'])
 ck('zero_optimizer_construction_and_iterate',identity['iterate_calls']=='0' and identity['optimizer_constructions']=='0')
 source=Path('tools/paper/a13_imu_covariance_probe.cpp').read_text();ck('no_conditional_optimizer_or_new_navigation_solver_in_probe',not re.search(r'LevenbergMarquardtOptimizer|\.iterate\(|\.optimize\(|\.solve\(',source))
 numeric=json.loads((r/'NUMERICAL_CHECKS.json').read_text());ck('all_static_numerical_checks',numeric['passed'] and all(numeric['checks'].values()))
 commands=[json.loads(p.read_text()) for p in sorted((r/'commands').glob('*/command.json'))];runtime=[c for c in commands if any(Path(a).name=='a13_imu_covariance_probe' for a in c['argv']) and '/usr/bin/strace' in c['argv']];ck('one_bounded_checkpoint_process',len(runtime)==1 and runtime[0]['exit_code']==0 and '120s' in runtime[0]['argv'] and float(identity['wall_seconds'])<120)
 trace=(r/'static/file_access.trace').read_text().splitlines();paths=[]
 for line in trace:
  if 'openat(' in line or 'open(' in line:
   x=re.findall(r'"([^"\n]*)"',line)
   if x:paths.append(x[0])
 ck('no_truth_GT_other_scene_file_access',not any(re.search(r'(^|/)(evaluation|truth|gt)(/|\.|$)',p,re.I) or '/raw/los/' in p or '/raw/ramp/' in p or 'P2_' in p or 'P3_' in p for p in paths))
 libs=sorted(set(line.split()[5] for line in (r/'static/process_maps.txt').read_text().splitlines() if len(line.split())>=6 and line.split()[5].startswith('/') and '.so' in line.split()[5]));bound=[]
 for p in libs:
  result=subprocess.run(['readelf','-n',p],capture_output=True,text=True);bound.append({'realpath':str(Path(p).resolve()),'sha256':sha(p),'bytes':Path(p).stat().st_size,'build_id':[line.strip() for line in result.stdout.splitlines() if 'Build ID:' in line]})
 a12=json.loads(Path('doc/ie_sprint/evidence/t10_a12_checkpoint_scale_20260909T131349Z/FINAL_MAPPED_LIBRARIES.json').read_text());prev={x['realpath']:x['sha256'] for x in a12};ck('mapped_libraries_match_A12',len(bound)==len(a12) and all(prev.get(x['realpath'])==x['sha256'] for x in bound))
 notes={}
 for name,args in [('undefined_symbols',['nm','-D','-C','--undefined-only',str(r/'a13_imu_covariance_probe')]),('probe_ldd',['ldd',str(r/'a13_imu_covariance_probe')]),('probe_readelf',['readelf','-n',str(r/'a13_imu_covariance_probe')])]:
  c=subprocess.run(args,capture_output=True,text=True);(r/(name+'.txt')).write_text(c.stdout+c.stderr);notes[name]={'argv':args,'exit_code':c.returncode}
 ck('no_optimizer_imports_in_probe',not re.search(r'LevenbergMarquardtOptimizer|NonlinearOptimizer::optimize|::iterate\(', (r/'undefined_symbols.txt').read_text()))
 ck('all_failures_preserved',(r/'failed_build/a13_imu_covariance_probe_v1.cpp').exists() and (r/'failed_analysis/audit_a13_covariance_v1.py').exists() and json.loads((r/'commands/build_probe/command.json').read_text())['exit_code']==1 and json.loads((r/'commands/static_audit/command.json').read_text())['exit_code']==1)
 # The logged wrapper returns g++ exit 1 in this environment; no runtime retry.
 result={'passed':not fail,'check_count':len(checks),'checks':checks,'failures':fail,'static_numerical_checks':len(numeric['checks']),'actual_checkpoint_processes':len(runtime),'optimizer_iterate':0,'truth_read_or_hash':'NOT_PERFORMED','source_production_change':'NONE','restoration_initializer':'Original raw trilateration and its 3x3 LDLT retained; conditional LM NOT_RUN','proposed_model_and_regressions':'NOT_IMPLEMENTED_NOT_RUN','chain_Stage2_validation_test_T11':'NOT_RUN','gate_and_claim':'UNCHANGED','actual_mapped_library_count':len(bound),'probe_sha256':sha(r/'a13_imu_covariance_probe')}
 for name,obj in [('CLOSEOUT_CHECK.json',result),('FINAL_MAPPED_LIBRARIES.json',bound),('OPENED_FILES.json',sorted(set(paths))),('IDENTITY_COMMANDS.json',notes),('COMMAND_INDEX.json',commands)]: (r/name).write_text(json.dumps(obj,indent=2)+'\n')
 print(json.dumps(result,indent=2));return 0 if not fail else 1
if __name__=='__main__':sys.exit(main(sys.argv[1]))
