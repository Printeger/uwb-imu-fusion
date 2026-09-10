#!/usr/bin/env python3
"""Freeze A15 payload and assert evidence consistency, without estimator replay."""
import csv,json,hashlib,shutil,collections,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'doc/ie_sprint/evidence/t10_a15_terminal_numeric_20260909T152633Z'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(p):return list(csv.DictReader(p.open()))
def main():
 checks={}
 pre=json.loads((E/'capture_preflight_identity.json').read_text())
 for name,v in pre.items():
  if name.endswith('T10_A15_PROTOCOL.md') or '/usr/local/' in name or '/inputs/raw/' in name or name.endswith(('.yaml','.cpp','.h','.so','paper_runner')):
   checks['unchanged:'+name]=sha(Path(name))==v['sha256']
 for n in ['capture_command','static_command']:
  d=json.loads((E/n/'command.json').read_text());checks[n+':no_truth']=not d['forbidden_input_open_lines'];checks[n+':maps']=d['maps_captured'];checks[n+':exit']=d['exit_code']==(1 if n=='capture_command' else 0)
  for v in d['elf_identity']:
   if not v['mapped']:continue
   p=Path(v['realpath']);checks[n+':elf:'+str(p)]=sha(p)==v['sha256']
   dest=E/'runtime_payload'/v['sha256']/p.name;dest.parent.mkdir(parents=True,exist_ok=True)
   if not dest.exists():shutil.copy2(p,dest)
 gate=json.loads((E/'capture_reproduction_gate.json').read_text());checks.update({'capture_gate:'+k:v for k,v in gate.items()})
 ds=rows(E/'selected_linked_static_comparison.csv');checks['all_selected_linked_linear_drops_exact']=all(x['linear_drop_reproduced']=='True' for x in ds)
 checks['accepted_linked_objective_exact']=ds[0]['objective_drop_reproduced']=='True'
 checks['rejected_did_not_evaluate_objective']=all(x['linked_objective_evaluated']=='False' for x in ds[1:])
 checks['white_residual_equals_negative_rhs']=all(float(x['linear_residual_difference'])==0 for x in rows(E/'static/factor_contributions.csv'))
 checks['accepted_retracts_16_pass']=len(rows(E/'static/accepted_retract_checks.csv'))==16 and all(x['pass']=='1' for x in rows(E/'static/accepted_retract_checks.csv'))
 fd=json.loads((E/'fd_summary.json').read_text());checks['all_four_directions_preregistered_consecutive_gate']=len(fd)==4 and all(min(x[k+'_max_consecutive'] for k in ['residual_pass','direct_pass','identity_pass'])>=3 for x in fd.values())
 expanded=rows(E/'factor_fd_expanded.csv');checks['all_factor_fd_retained']=len(expanded)==13356;checks['124_factor_fd_failures_retained']=sum(x['residual_pass']=='0' for x in expanded)==124
 d=rows(E/'runs/P1_step_seed10101_terminal_capture/conditional_lm_trial_deltas.csv');checks['all_delta_coordinates']=len(d)==42*615
 counts=collections.Counter(x['observed_branch'] for x in rows(E/'linked_trial_accounting.csv') if x['call']=='17');checks['all19_branches']=counts=={'REJECT_LINEAR_NEGATIVE':18,'REJECT_RESOLUTION':1}
 for n in ['build','build_final_sequential','static_compile_gradient','static_link_final','aggregation_final_command']:
  checks['command_success:'+n]=json.loads((E/n/'command.json').read_text())['exit_code']==0
 checks['failed_compile_preserved']=json.loads((E/'static_compile/command.json').read_text())['exit_code']==1
 checks['interrupted_build_preserved']=json.loads((E/'build_final/command.json').read_text())['exit_code']!=0
 (E/'FINAL_CHECKS.json').write_text(json.dumps({'pass':all(checks.values()),'checks':checks},indent=2)+'\n')
 assert all(checks.values()),[k for k,v in checks.items() if not v]
 # Source/config/docs archive; evaluation/GT directories are never traversed.
 files=list((ROOT/'src').glob('*.cpp'))+list((ROOT/'include/uifgo').glob('*.h'))
 files += [ROOT/'CMakeLists.txt',ROOT/'package.xml',ROOT/'AGENTS.md',ROOT/'tools/run_ie_paper.cpp',ROOT/'test/test_nlos_discovery.cpp',ROOT/'test/test_nlos_refit.cpp']
 files += list((ROOT/'tools/paper').glob('a15_*.py'))+[ROOT/'tools/paper/a15_static_recompute.cpp',ROOT/'tools/paper/a14_run_logged.py',ROOT/'tools/paper/t07_run_logged_command.py']
 files += [ROOT/'doc/ie_sprint'/n for n in ['STATUS.md','T10_READINESS.md','METHOD_CONTRACT.md','EXPERIMENT_CONTRACT.md','T10_A15_PROTOCOL.md','T10_A15_NEXT_ACTION_PROPOSAL.md']]
 files += [ROOT/'paper/CLAIM_EVIDENCE.md',ROOT/'doc/v2/paper_structure.tex',ROOT/'doc/v2/v2_roadmap.md']
 for p in files:
  dest=E/'after'/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
 # Bind test binaries as rebuilt but NOT_RUN, separately from actual mapped ELF.
 tests=[]
 for name in ['test_nlos_discovery','test_nlos_refit']:
  p=Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo')/name
  tests.append({'path':str(p),'sha256':sha(p),'status':'REBUILT_NOT_RUN'})
 (E/'test_binary_identity.json').write_text(json.dumps(tests,indent=2)+'\n')
 (E/'git_identity.json').write_text(json.dumps({'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'branch':subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),'protected_dirty_worktree':True},indent=2)+'\n')
 print(json.dumps({'checks':len(checks),'pass':all(checks.values()),'estimator_processes':1,'static_processes':1,'fd_failures_retained':124}))
if __name__=='__main__':main()
