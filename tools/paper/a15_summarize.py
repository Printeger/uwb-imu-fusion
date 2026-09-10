#!/usr/bin/env python3
"""Read only A15 capture/static evidence; no estimator or truth input."""
import csv,json,re,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
E=ROOT/'doc/ie_sprint/evidence/t10_a15_terminal_numeric_20260909T152633Z'
C=E/'runs/P1_step_seed10101_terminal_capture'
A=ROOT/'doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/runs/P1_step_seed10101_conditional'
def rows(p):return list(csv.DictReader(p.open()))
def writecsv(p,data):
 with p.open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
def main():
 calls=rows(C/'conditional_lm_calls.csv');st=rows(C/'conditional_lm_stationarity.csv')
 old=json.loads((A/'common_preparation.json').read_text());new=json.loads((C/'common_preparation.json').read_text())
 checks={k:old[k]==new[k] for k in old}
 checks.update(calls_17=len(calls)==17,accepted_16=sum(int(c['accepted_state_update']) for c in calls)==16,
 trials_42=int(calls[-1]['inner_iterations_after'])==42,terminal_error=float(calls[-1]['error_after'])==2325.3170707993281,
 lambda_exact=float(calls[-1]['lambda_after'])==100000.00000000007,
 no_extra_solve=all(c['first_try_reason']=='NOT_RUN_A15_NO_EXTRA_SOLVE' and c['linear_system_solved']=='0' for c in calls),
 no_stationarity=st[-1]['stationary']=='0',gradient=abs(float(st[-1]['max_scaled_gradient'])-1.2054936689764872e-5)<=2e-12)
 (E/'capture_reproduction_gate.json').write_text(json.dumps(checks,indent=2)+'\n')
 if not all(checks.values()):raise RuntimeError('A14 reproduction gate failed; static NOT_RUN')
 text=(C/'conditional_lm_linked_trydelta.log').read_text();trialrows=[]
 for c in calls:
  idx=int(c['call_index']);body=re.search(r'ACTUAL_LINKED_TRYDELTA_CALL_BEGIN call_index='+str(idx)+r'\n(.*?)ACTUAL_LINKED_TRYDELTA_CALL_END call_index='+str(idx)+r'\n',text,re.S).group(1)
  ts=re.split(r'trying lambda = ',body)[1:]
  for i,t in enumerate(ts,1):
   def val(pattern):
    m=re.search(pattern,t);return float(m.group(1)) if m else None
   lam=float(t.splitlines()[0]);lc=val(r'linearizedCostChange = (\S+)');oe=val(r'old error \(([^)]+)\)');ne=val(r'new \(tentative\) error \(([^)]+)\)');f=val(r'modelFidelity: (\S+)');inc='increasing lambda' in t;accepted=c['accepted_state_update']=='1' and i==len(ts)
   branch='ACCEPT_FIDELITY' if accepted else ('REJECT_LINEAR_NEGATIVE' if lc is not None and lc<0 else ('REJECT_FIDELITY' if f is not None else ('REJECT_RESOLUTION' if lc is not None and val(r'linear delta norm = (\S+)') is not None else 'UNSOLVED')))
   trialrows.append(dict(call=idx,trial=i,lambda_value=lam,delta_norm=val(r'linear delta norm = (\S+)'),new_linear_error=val(r'newlinearizedError = (\S+)'),linked_linear_drop=lc,old_error=oe,new_error=ne,linked_objective_drop=None if oe is None else oe-ne,model_fidelity=f,fidelity_branch_executed=f is not None,increase_lambda=inc,accepted=accepted,observed_branch=branch))
 assert len(trialrows)==42 and sum(t['accepted'] for t in trialrows)==16
 writecsv(E/'linked_trial_accounting.csv',trialrows)
 if not (E/'static/RESTORATION_GATE.txt').exists():print('CAPTURE_GATE_PASS; static not yet run');return
 ds=rows(E/'static/direction_summary.csv');fds=rows(E/'static/fd_summary.csv');fs=rows(E/'static/factor_fd.csv')
 summary={}
 def maxconsecutive(items):
  a=m=0
  for b in items:a=a+1 if b else 0;m=max(m,a)
  return m
 for name in dict.fromkeys(f['selection'] for f in fds):
  rr=[f for f in fds if f['selection']==name]
  summary[name]={k:sum(f[k]=='1' for f in rr) for k in ['direct_pass','identity_pass','residual_pass']}
  summary[name].update({k+'_max_consecutive':maxconsecutive(f[k]=='1' for f in rr) for k in ['direct_pass','identity_pass','residual_pass']})
  summary[name]['min_direct_abs_error']=min(float(f['direct_abs_error']) for f in rr)
  summary[name]['min_identity_abs_error']=min(float(f['identity_abs_error']) for f in rr)
  summary[name]['analytic']=float(rr[0]['analytic_ld'])
  summary[name]['residual_failed_factor_step_count']=sum(f['residual_pass']=='0' for f in fs if f['selection']==name)
 for r in ds:
  t=next(t for t in trialrows if t['call']==int(r['call']) and t['trial']==int(r['trial']))
  # Linked omitted objective evaluation when linearized subtraction was negative.
  r['linked_objective_evaluated']=t['old_error'] is not None
  r['linked_recorded_objective_drop']=t['linked_objective_drop']
  r['linked_recorded_linear_drop']=t['linked_linear_drop']
  r['linear_drop_reproduced']=float(r['linked_linear_drop'])==t['linked_linear_drop']
  r['objective_drop_reproduced']=None if t['old_error'] is None else float(r['direct_drop'])==t['linked_objective_drop']
  r['observed_branch']=t['observed_branch']
  r['linked_recorded_model_fidelity']=t['model_fidelity']
 writecsv(E/'selected_linked_static_comparison.csv',ds)
 (E/'fd_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 # Each factor's objective test is reported separately, never substituted for graph derivative.
 ff=[]
 for r in fs:
  q=dict(r);a=float(r['analytic_ld']);tol=1e-7+1e-5*abs(a)
  q.update(objective_tolerance=tol,objective_direct_abs_error=abs(float(r['objective_fd_direct'])-a),objective_identity_abs_error=abs(float(r['objective_fd_identity_ld'])-a))
  q['objective_direct_pass']=q['objective_direct_abs_error']<=tol;q['objective_identity_pass']=q['objective_identity_abs_error']<=tol;ff.append(q)
 writecsv(E/'factor_fd_expanded.csv',ff)
 print(json.dumps({'reproduction':checks,'selected_trials':ds,'derivatives':summary},indent=2))
if __name__=='__main__':main()
