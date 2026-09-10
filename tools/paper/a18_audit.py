#!/usr/bin/env python3
import sys,json,csv,hashlib,re
from pathlib import Path
from fractions import Fraction as F
E=Path(sys.argv[1]);pilot=E/'pilot';checks={};blocks=[];certrows=[]
def rb(s):n,e=s.split('@');return F(int(n,2))*F(2)**int(e)
for block in sorted(pilot.glob('outer[0-9]*'),key=lambda p:int(p.name[5:])):
 status=json.loads((block/'block_status.json').read_text()) if (block/'block_status.json').exists() else {'reason':'INCOMPLETE_TIMEOUT','converged':False}
 trials=list(csv.DictReader((block/'trials.csv').open()));calls=list(csv.DictReader((block/'calls.csv').open()));accepted=rejected=unresolved=0
 for i,t in enumerate(trials,1):
  d=block/t['artifact'];cpath=d/'certificate.json';c=json.loads(cpath.read_text()) if cpath.exists() else None
  assert int(t['total_trial'])==i
  if c:
   Plo,Phi,Dlo,Dhi=[rb(c[n+'_binary']) for n in ['P_lo','P_hi','D_lo','D_hi']]
   assert (Phi-Plo)/2<=F('1e-15') and (Dhi-Dlo)/2<=F('1e-15')
   assert c['status']==t['status'] and c['factor_count']==371 and c['branch_count']==164
   if c['status']=='ACCEPT':
    flo,fhi=rb(c['fidelity_lo_binary']),rb(c['fidelity_hi_binary']);q=F.from_float(c['fidelity_binary64']);assert Plo>0 and Dlo>0 and F.from_float(.001)<q<=flo<=Dlo/Phi and Dhi/Plo<=fhi
   elif c['status']=='REJECT':assert Phi<=0 or Dhi<=0 or (Plo>0 and Dlo>0 and rb(c['fidelity_hi_binary'])<=F.from_float(.001))
   row={'outer':int(block.name[5:]),'call':t['call'],'trial':t['trial_in_call'],'status':c['status'],'P_lo':c['P_lo'],'P_hi':c['P_hi'],'D_lo':c['D_lo'],'D_hi':c['D_hi'],'fidelity_binary64':c['fidelity_binary64'],'lambda_before':t['lambda_before'],'lambda_after':t['lambda_after']};certrows.append(row)
  if t['status']=='ACCEPT':accepted+=1
  elif t['status'] in ['REJECT','LINEAR_SOLVE_FAILED']:rejected+=1
  else:unresolved+=1
  assert (accepted,rejected,unresolved)==tuple(int(t[k]) for k in ['accepted_total','rejected_total','unresolved_total'])
 if status.get('converged'):assert calls[-1]['generic']=='1' and calls[-1]['stationary']=='1' and calls[-1]['stationarity_valid']=='1'
 if 'calls' in status:assert status['calls']==len(calls) and status['trials']==len(trials) and status['accepted']==accepted and status['rejected']==rejected and status['unresolved']==unresolved
 blocks.append(dict(**dict(status,outer=int(block.name[5:])),observed_calls=len(calls),observed_trials=len(trials),observed_accepted=accepted,observed_rejected=rejected,observed_unresolved=unresolved,last_call=calls[-1] if calls else None))
status=json.loads((pilot/'stage1_status.json').read_text());outers=list(csv.DictReader((pilot/'outers.csv').open()));checks['all_certificates_and_counters']=True
checks['AND']=bool(outers) and (not status['converged'] or all(outers[-1][k]=='1' for k in ['objective_ok','step_ok','KKT_ok','navigation_ok']))
checks['no_after_first_failure']=all(b['converged'] for b in blocks[:-1])
checks['one_pilot']=len(list(E.glob('PILOT_ONCE_TICKET.json')))==1 and json.loads((E/'PILOT_ONCE_TICKET.json').read_text())['max_processes']==1
checks['unchanged_source_and_input']=all(hashlib.sha256(Path(x['path']).read_bytes()).hexdigest()==x['sha256'] for x in json.loads((E/'PRE_PILOT_IDENTITIES.json').read_text()))
commands=[json.loads(p.read_text()) for p in E.glob('*/command.json')];checks['no_truth_open']=all(not c.get('forbidden_input_open_lines',[]) for c in commands)
checks['no_formal_artifacts']=not any(p.name in ['run_status.json','trajectory.tum','stage2_cache_manifest.json','scores_decision.csv','scores_final.csv'] for p in pilot.rglob('*'))
first=blocks[0];old=Path('doc/ie_sprint/evidence/t10_a17_certified_prototype_20260910T002211Z/pilot');a=list(csv.DictReader((old/'calls.csv').open()));b=list(csv.DictReader((pilot/'outer1/calls.csv').open()));checks['A17_first_block_reproduced']=len(a)==len(b)==20 and all(all(x[k]==y[k] for k in ['call','error','lambda','trials','accepted','rejected','unresolved','generic','stationary','max_scaled_gradient','roundoff']) for x,y in zip(a,b))
with (E/'PILOT_CERTIFICATES.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(certrows[0]));w.writeheader();w.writerows(certrows)
resource=(E/'pilot_run/resources.txt').read_text();runtime=json.loads((E/'pilot_run/command.json').read_text());summary={'status':status,'completed_outers':len(outers),'attempted_outers':len(blocks),'total_calls':sum(b['observed_calls'] for b in blocks),'total_trials':sum(b['observed_trials'] for b in blocks),'accepted':sum(b['observed_accepted'] for b in blocks),'rejected':sum(b['observed_rejected'] for b in blocks),'unresolved':sum(b['observed_unresolved'] for b in blocks),'certificate_seconds':sum(b.get('certificate_s',0) for b in blocks),'external_wall_s':runtime['external_wall_s'],'exit_code':runtime['exit_code'],'peak_RSS_KiB':int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',resource)[1]),'first_block':first,'last_block':blocks[-1],'last_outer':outers[-1] if outers else None}
if status['converged']:
 partition=list(csv.DictReader((pilot/'partition.csv').open()));summary['partition']={'segments':len(partition),'candidate_observations':sum(int(p['obs_count']) for p in partition),'short':sum(p['short_support']=='1' for p in partition),'boundary':None,'eligible':None,'groups':None}
else:summary['partition']={'segments':None,'candidate_observations':None,'short':None,'boundary':None,'eligible':None,'groups':None}
result={'passed':all(checks.values()),'checks':checks,'summary':summary,'blocks':blocks};(E/'RESULT_AUDIT.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'passed':result['passed'],'checks':checks,'summary':summary},indent=2));sys.exit(0 if result['passed'] else 1)
