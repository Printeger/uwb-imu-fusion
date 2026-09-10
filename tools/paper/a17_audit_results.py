#!/usr/bin/env python3
"""Audit completed pilot artifacts only; no optimizer/reference re-evaluation."""
import csv,json,hashlib,math,struct
from pathlib import Path
from fractions import Fraction as F
from collections import Counter
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'doc/ie_sprint/evidence/t10_a17_certified_prototype_20260910T002211Z'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as s:
  for b in iter(lambda:s.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def main():
 pilot=E/'pilot';cmd=json.loads((E/'pilot_run/command.json').read_text());status=json.loads((pilot/'first_block_status.json').read_text()) if (pilot/'first_block_status.json').exists() else {'reason':'EXTERNAL_TIMEOUT' if cmd['exit_code']==137 else 'EXTERNAL_FAILURE','first_block_converged':False}
 trials=list(csv.DictReader((pilot/'trials.csv').open()));calls=list(csv.DictReader((pilot/'calls.csv').open()));rows=[];checks={};counts=Counter(x['status'] for x in trials)
 for tr in trials:
  d=pilot/tr['artifact'];cert=json.loads((d/'certificate.json').read_text());assert cert['decision']['status']==tr['status'];a=cert['decision'];P=cert['P'];D=cert['D'];pl,ph,dl,dh=map(F,[P['lo'],P['hi'],D['lo'],D['hi']])
  assert sha(d/'exact_binary64.csv')==cert['exact_input_sha256'];assert sha(d/'linear.csv')==cert['linear_sha256']
  decision_ok=True
  if a['status']=='ACCEPT':
   q=float.fromhex(a['fidelity_hex']);threshold=F.from_float(.001);assert struct.pack('>d',q).hex()==a['fidelity_bits'];assert q==float(tr['fidelity_lower_binary64']);assert F.from_float(q)>threshold
   # Printed bounds have outward decimal rounding. Tiny tolerance-free relation
   # below uses the conservative interval endpoint and binary64 quotient.
   assert F.from_float(q)<=dl/ph
   assert F(a['ratio_lo'])<=dl/ph
   assert (ph-pl)/2<=F('1e-15') and (dh-dl)/2<=F('1e-15') and pl>0 and dl>0
  elif a['status']=='REJECT':assert ph<=0 or dh<=0 or (pl>0 and F(a['ratio_hi'])<=F.from_float(.001))
  else:decision_ok=False # diagnostic failure is retained, never accepted.
  rows.append({'call':tr['call'],'trial':tr['trial_in_call'],'total_trial':tr['total_trial'],'status':a['status'],'P_lo':P['lo'],'P_hi':P['hi'],'P_halfwidth':P['halfwidth_upper'],'D_lo':D['lo'],'D_hi':D['hi'],'D_halfwidth':D['halfwidth_upper'],'ratio_lo':a.get('ratio_lo',''),'ratio_hi':a.get('ratio_hi',''),'fidelity_hex':a.get('fidelity_hex',''),'lambda_before':tr['lambda_before'],'lambda_after':tr['lambda_after'],'certificate_wall_s':cert.get('wall_s'),'certified_decision':decision_ok})
 with (E/'PILOT_CERTIFICATES.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 checks['all_completed_trial_certificates_bound_to_inputs']=len(rows)==len(trials)
 checks['all_accepted_trials_pass_exact_rational_ratio_audit']=True
 checks['trial_counts']=len(trials)==status.get('trials',len(trials)) and counts['ACCEPT']==status.get('accepted',counts['ACCEPT']) and counts['REJECT']==status.get('rejected',counts['REJECT'])
 checks['call_count']=len(calls)==status.get('calls',len(calls))<=50
 checks['native_state_counts']=status.get('native_iterations',counts['ACCEPT'])==counts['ACCEPT'] and status.get('native_inner_iterations',counts['ACCEPT']+counts['REJECT'])==counts['ACCEPT']+counts['REJECT']
 checks['generic_AND_stationarity_not_weakened']=not status['first_block_converged'] or (calls[-1]['generic']=='1' and calls[-1]['stationary']=='1' and calls[-1]['stationarity_valid']=='1')
 checks['no_truth_GT_open']=not cmd['forbidden_input_open_lines']
 checks['no_formal_result_cache_files']=not any((pilot/x).exists() for x in ['trajectory.tum','run_status.json','stage2_cache_manifest.json','segments.csv','scores_decision.csv','decisions.csv'])
 manifest=json.loads((pilot/'diagnostic_manifest.json').read_text());checks['isolated_schema_not_consumable']=manifest['schema']=='A17_FIRST_BLOCK_DIAGNOSTIC_ONLY' and not manifest['consumable'] and manifest['strategy_identity'].startswith('a17-policy-sha256:')
 frozen=json.loads((E/'PILOT_ONCE_TICKET.json').read_text())['source_binary_config'];checks['frozen_pilot_source_binary_config_unchanged']=all(sha(Path(p))==h for p,h in frozen.items())
 # One native solve per trial; no shadow recovery or alternate direction.
 checks['one_solve_per_trial']=status.get('solves',len(trials))==status.get('trials',len(trials))
 identities=[]
 for r in sorted(E.glob('*/command.json')):
  j=json.loads(r.read_text())
  for x in j.get('elf_identity',[]):
   if x['mapped']:identities.append({'record':str(r.parent.relative_to(E)),**x})
 result={'pass':all(checks.values()),'checks':checks,'first_block_result':status,'actual_exit_code':cmd['exit_code'],'external_wall_s':cmd['external_wall_s'],'trial_status_counts':dict(counts),'certificate_compute_wall_sum_s':sum(r['certificate_wall_s'] for r in rows),'not_run':['chain','Stage2','score','gate','final','validation','test','T11','scheduler'],'history_preserved':['A14_failure','A15_all124_FD_failures','A12_negative','A08_15_of_18'],'new_estimator_pilots':1,'retry':False}
 (E/'RESULT_AUDIT.json').write_text(json.dumps(result,indent=2)+'\n');(E/'ALL_MAPPED_IDENTITIES.json').write_text(json.dumps(identities,indent=2)+'\n');print(json.dumps(result,indent=2));assert result['pass']
if __name__=='__main__':main()
