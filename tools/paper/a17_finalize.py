#!/usr/bin/env python3
"""A17 evidence-only closure; whitelisted raw and actual mapped ELFs."""
import hashlib,json,shutil,re,csv,datetime,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'doc/ie_sprint/evidence/t10_a17_certified_prototype_20260910T002211Z';A16=ROOT/'doc/ie_sprint/evidence/t10_a16_endpoint_precision_20260909T160252Z'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for v in iter(lambda:f.read(1024*1024),b''):h.update(v)
 return h.hexdigest()
def main():
 entries=[]
 def cp(p,q,role):
  p=Path(p);q=E/q;q.parent.mkdir(parents=True,exist_ok=True);assert not q.exists();shutil.copy2(p,q);entries.append({'source':str(p),'path':str(q.relative_to(E)),'sha256':sha(q),'bytes':q.stat().st_size,'role':role})
 for p in sorted((ROOT/'tools/paper').glob('a17_*')):
  if p.is_file():cp(p,Path('source/repo/tools/paper')/p.name,'A17_current_source')
 names=['tools/paper/a16_export_endpoints.cpp','tools/paper/a16_mpfr.py','tools/paper/a16_reference.py','tools/paper/a14_run_logged.py','tools/paper/t07_run_logged_command.py','src/nlos_solver_utils.cpp','include/uifgo/nlos_solver_utils.h','src/graph_builder.cpp','src/imu_preint.cpp','src/initializer.cpp','src/uwb_factor.cpp','src/config.cpp','src/paper_input.cpp','src/paper_stage2_cache.cpp','src/nlos_inference.cpp','include/uifgo/paper_pose_prior_factor.h','include/uifgo/config.h','include/uifgo/imu_preint.h','include/uifgo/paper_stage2_cache.h','test/test_nlos_discovery.cpp','CMakeLists.txt','AGENTS.md','doc/ie_sprint/STATUS.md','doc/ie_sprint/T10_READINESS.md','doc/ie_sprint/METHOD_CONTRACT.md','doc/ie_sprint/EXPERIMENT_CONTRACT.md','doc/ie_sprint/T10_A17_AMENDMENT_PROTOCOL.md','doc/ie_sprint/T10_A16_PROTOCOL.md','doc/ie_sprint/T10_A16_SOLVER_AMENDMENT_DRAFT.md','paper/CLAIM_EVIDENCE.md','doc/v2/paper_structure.tex','doc/v2/v2_roadmap.md']
 for s in names:cp(ROOT/s,Path('source/repo')/s,'reused_source_or_contract')
 # A16's source set already bounds the residual implementation dependencies.
 for p in sorted((A16/'source/gtsam').rglob('*')):
  if p.is_file():cp(p,Path('source/gtsam')/p.relative_to(A16/'source/gtsam'),'reviewed_GTSAM_residual_source')
 for n in ['nonlinear/internal/LevenbergMarquardtState.h','nonlinear/internal/NonlinearOptimizerState.h','nonlinear/NonlinearOptimizer.h','nonlinear/LevenbergMarquardtOptimizer.h','nonlinear/LevenbergMarquardtParams.h']:
  cp(Path('/usr/local/include/gtsam')/n,Path('source/installed_gtsam')/n,'actual_installed_LM_interface')
 for n in ['nonlinear/NonlinearOptimizer.cpp','nonlinear/LevenbergMarquardtOptimizer.cpp']:
  cp(Path('/home/mint/dep/gtsam/gtsam')/n,Path('source/linked_gtsam')/n,'linked_solver_source_correspondence')
 cfg=ROOT/'doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/P1_step_seed10101_conditional.yaml';cp(cfg,Path('inputs')/cfg.name,'A14_frozen_config')
 raw=ROOT/'doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/inputs/raw/step'
 for n in ['input_manifest.json','imu.csv','uwb_observations.csv']:cp(raw/n,Path('inputs/raw')/n,'raw_only_no_truth')
 for n in ['VERIFICATION.md','FORMULAS_AND_ERROR_BOUND.md','endpoints/exact_binary64.csv','endpoints/factors.csv','endpoints/identity.csv','reference/dps100_interval/summary.json','inputs/A15_selected/conditional_lm_direction_base_values.csv','inputs/A15_selected/conditional_lm_trial_deltas.csv']:
  cp(A16/n,Path('inputs/A16_regression_only')/n,'frozen_engineering_checkpoint_not_pilot_warm_start')
 runtime=json.loads((E/'ALL_MAPPED_IDENTITIES.json').read_text());seen={}
 for r in runtime:
  if r['sha256'] not in seen:
   p=Path(r['realpath']);assert sha(p)==r['sha256'];q=Path('runtime')/(r['sha256'][:16]+'_'+p.name);cp(p,q,'actual_mapped_ELF');seen[r['sha256']]=str(q)
  r['payload']=seen[r['sha256']]
 (E/'RUNTIME_IDENTITY.json').write_text(json.dumps(runtime,indent=2)+'\n')
 # Replay-free context identity: binds prototype policy and the actually prepared
 # frozen input/initial graph; deliberately not a production cache schema.
 m=json.loads((E/'pilot/diagnostic_manifest.json').read_text());i=list(csv.DictReader((E/'pilot/initial_identity.csv').open()))[0];rawhashes={n:sha(raw/n) for n in ['input_manifest.json','imu.csv','uwb_observations.csv']};binding={'schema':'A17_DIAGNOSTIC_CONTEXT_V1','consumable':False,'policy_identity':m['strategy_identity'],'initial_identity':i,'raw_hashes':rawhashes,'config_sha256':sha(cfg)};binding['context_sha256']=hashlib.sha256(json.dumps(binding,sort_keys=True,separators=(',',':')).encode()).hexdigest();(E/'DIAGNOSTIC_CONTEXT.json').write_text(json.dumps(binding,indent=2)+'\n')
 checks={'result_audit_pass':json.loads((E/'RESULT_AUDIT.json').read_text())['pass'],'engineering_gate_pass':json.loads((E/'ENGINEERING_GATE.json').read_text())['pass'],'pilot_once_ticket_present':(E/'PILOT_ONCE_TICKET.json').exists(),'source_binary_frozen_unchanged':all(sha(Path(p))==h for p,h in json.loads((E/'PILOT_ONCE_TICKET.json').read_text())['source_binary_config'].items()),'original_protocol_unchanged':sha(E/'PREREGISTERED_PROTOCOL.md')==sha(ROOT/'doc/ie_sprint/T10_A17_AMENDMENT_PROTOCOL.md')}
 missing=[]
 for p in [E/'VERIFICATION.md',ROOT/'doc/ie_sprint/T10_A17_AMENDMENT_PROTOCOL.md']:
  for link in re.findall(r'\]\(([^)]+)\)',p.read_text()):
   if not link.startswith(('http','#')) and not (p.parent/link.split('#')[0]).exists():missing.append([str(p),link])
 checks['new_doc_links_valid']=not missing
 cmds=[]
 for p in sorted(E.glob('*/command.json')):
  j=json.loads(p.read_text());cmds.append({'record':str(p.relative_to(E)),'argv':j['argv'],'exit_code':j.get('exit_code'),'started_utc':j.get('started_utc'),'finished_utc':j.get('finished_utc')})
 (E/'COMMAND_INDEX.json').write_text(json.dumps(cmds,indent=2)+'\n')
 (E/'SOURCE_AND_INPUT_INVENTORY.json').write_text(json.dumps({'entries':entries},indent=2)+'\n')
 result={'pass':all(checks.values()),'checks':checks,'missing_links':missing,'source_input_runtime_snapshots':len(entries),'runtime_unique_ELF':len(seen),'production_solver_change':False,'formal_or_Stage2_integration':'NOT_RUN','verified_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()};(E/'FINAL_CHECKS.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));assert result['pass']
if __name__=='__main__':main()
