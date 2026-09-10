#!/usr/bin/env python3
"""A16 evidence closure only. Whitelisted raw inputs; no truth traversal."""
from pathlib import Path
import csv,datetime,hashlib,json,shutil,subprocess
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'doc/ie_sprint/evidence/t10_a16_endpoint_precision_20260909T160252Z'
A15=ROOT/'doc/ie_sprint/evidence/t10_a15_terminal_numeric_20260909T152633Z'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as s:
  for b in iter(lambda:s.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def main():
 entries=[]
 def cp(p,q,role):
  p=Path(p);q=E/q;q.parent.mkdir(parents=True,exist_ok=True)
  if q.exists():raise ValueError('snapshot exists '+str(q))
  shutil.copy2(p,q);entries.append({'source':str(p),'payload':str(q.relative_to(E)),'sha256':sha(q),'bytes':q.stat().st_size,'role':role})
 source=['tools/paper/a16_export_endpoints.cpp','tools/paper/a16_mpfr.py','tools/paper/a16_reference.py','tools/paper/a16_summarize.py','tools/paper/a16_finalize.py','tools/paper/a16_archive.py','tools/paper/a14_run_logged.py','tools/paper/t07_run_logged_command.py','src/nlos_solver_utils.cpp','src/imu_preint.cpp','src/uwb_factor.cpp','src/graph_builder.cpp','src/initializer.cpp','src/config.cpp','src/paper_input.cpp','src/paper_stage2_cache.cpp','src/nlos_inference.cpp','include/uifgo/paper_pose_prior_factor.h','include/uifgo/imu_preint.h','include/uifgo/config.h','AGENTS.md','doc/ie_sprint/METHOD_CONTRACT.md','doc/ie_sprint/EXPERIMENT_CONTRACT.md','doc/ie_sprint/T10_A16_PROTOCOL.md','doc/ie_sprint/T10_A16_SOLVER_AMENDMENT_DRAFT.md','doc/ie_sprint/STATUS.md','doc/ie_sprint/T10_READINESS.md','paper/CLAIM_EVIDENCE.md','doc/v2/paper_structure.tex','doc/v2/v2_roadmap.md']
 for p in source:cp(ROOT/p,Path('source/repo')/p,'source_or_contract')
 gs=['geometry/SO3.cpp','geometry/SO3.h','geometry/Rot3M.cpp','geometry/Rot3.h','geometry/Pose3.cpp','geometry/Pose3.h','navigation/NavState.cpp','navigation/NavState.h','navigation/PreintegrationBase.cpp','navigation/PreintegrationBase.h','navigation/TangentPreintegration.cpp','navigation/TangentPreintegration.h','navigation/CombinedImuFactor.cpp','navigation/CombinedImuFactor.h','navigation/ImuBias.cpp','navigation/ImuBias.h','nonlinear/ExpressionFactor.h','nonlinear/PriorFactor.h','nonlinear/NonlinearFactor.cpp','nonlinear/LevenbergMarquardtOptimizer.cpp','nonlinear/LevenbergMarquardtParams.h','linear/NoiseModel.cpp','linear/NoiseModel.h','config.h']
 for p in gs:
  s=Path('/home/mint/dep/gtsam/gtsam')/p
  if p=='config.h':s=Path('/usr/local/include/gtsam/config.h')
  cp(s,Path('source/gtsam')/p,'linked_GTSAM_corresponding_source_or_installed_config')
  installed=Path('/usr/local/include/gtsam')/p
  if p.endswith('.h') and installed.exists():cp(installed,Path('source/installed_gtsam_headers')/p,'actual_compiled_header')
 cfg=ROOT/'doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/P1_step_seed10101_conditional.yaml';cp(cfg,Path('inputs')/cfg.name,'unchanged_A14_config')
 raw=ROOT/'doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/inputs/raw/step'
 for name in ['input_manifest.json','imu.csv','uwb_observations.csv']:cp(raw/name,Path('inputs/raw')/name,'raw_only_no_truth')
 # Copy source evidence solely for identity/provenance; only requested rows kept.
 cap=A15/'runs/P1_step_seed10101_terminal_capture';sourceids=[]
 for name,select in [('conditional_lm_direction_base_values.csv',lambda r:r['phase']=='CALL_17_BASE'),('conditional_lm_trial_deltas.csv',lambda r:r['call_index']=='17' and r['trial_index_within_call']=='1')]:
  p=cap/name;reader=csv.DictReader(p.open());rows=[r for r in reader if select(r)];q=E/'inputs/A15_selected'/name;q.parent.mkdir(parents=True,exist_ok=True)
  with q.open('w') as f:w=csv.DictWriter(f,fieldnames=reader.fieldnames);w.writeheader();w.writerows(rows)
  sourceids.append({'original':str(p),'original_sha256':sha(p),'selected_rows':len(rows),'selected_sha256':sha(q),'payload':str(q.relative_to(E))})
 for p in [cap/'conditional_lm_factor_audit.csv',A15/'static/identity.csv',A15/'VERIFICATION.md',ROOT/'doc/ie_sprint/T10_A15_NEXT_ACTION_PROPOSAL.md']:
  cp(p,Path('inputs/A15_provenance')/p.name,'frozen_provenance_not_new_trial_evaluation')
 (E/'A15_SELECTED_PROVENANCE.json').write_text(json.dumps(sourceids,indent=2)+'\n')
 # Actual successful mapped ELFs, de-duplicated by content; not an inferred ldd set.
 runtimes=[];seen={}
 for rec in ['export_run','reference_run']:
  j=json.loads((E/rec/'command.json').read_text());assert j['exit_code']==0 and not j['forbidden_input_open_lines']
  for lib in j['elf_identity']:
   if not lib['mapped']:continue
   p=Path(lib['realpath']);assert sha(p)==lib['sha256']
   if lib['sha256'] not in seen:
    q=Path('runtime')/(lib['sha256'][:16]+'_'+p.name);cp(p,q,'actual_mapped_ELF');seen[lib['sha256']]=str(q)
   runtimes.append({'process_record':rec,**lib,'payload':seen[lib['sha256']]})
 (E/'RUNTIME_IDENTITY.json').write_text(json.dumps(runtimes,indent=2)+'\n')
 # Checked source snippets prove no invocation of an optimizer in the exporter.
 exporter=(ROOT/'tools/paper/a16_export_endpoints.cpp').read_text()
 forbidden=['.iterate(','.optimize(','InspectFirstLinkedLmTry(','LevenbergMarquardtOptimizer lm','GaussNewtonOptimizer ']
 checks={'endpoint_seal_unchanged':json.loads((E/'UNCHANGED_CHECKS.json').read_text())['pass'],'reference_verdict':json.loads((E/'reference/VERDICT.json').read_text()),'no_optimizer_invocation_in_exporter':not any(s in exporter for s in forbidden),'one_endpoint_retract_expression':exporter.count('base.retract(delta)')==1,'native_residual_restoration_pass':all(r['pass']=='1' for r in csv.DictReader((E/'endpoints/restoration_checks.csv').open())),'protocol_unchanged':sha(E/'PREREGISTERED_PROTOCOL.md')==sha(ROOT/'doc/ie_sprint/T10_A16_PROTOCOL.md'),'all_evidence_process_truth_open_count':sum(len(json.loads((E/rec/'command.json').read_text())['forbidden_input_open_lines']) for rec in ['export_run','reference_run']),'optimizer_iterates':0,'new_estimator_processes':0,'other_trial_evaluation':'NOT_RUN','source_AND_runtime_snapshots':len(entries)}
 checks['pass']=checks['endpoint_seal_unchanged'] and checks['no_optimizer_invocation_in_exporter'] and checks['one_endpoint_retract_expression'] and checks['native_residual_restoration_pass'] and checks['protocol_unchanged'] and checks['all_evidence_process_truth_open_count']==0 and all(checks['reference_verdict'][k] for k in ['two_precision_pass','interval_pass','strict_descent_both_intervals','formula_sanity_pass'])
 (E/'SOURCE_AND_INPUT_INVENTORY.json').write_text(json.dumps({'entries':entries},indent=2)+'\n');(E/'FINAL_CHECKS.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(checks,indent=2));assert checks['pass']
if __name__=='__main__':main()
