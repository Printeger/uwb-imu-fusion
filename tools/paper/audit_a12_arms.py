#!/usr/bin/env python3
"""Read-only comparison of fixed-budget C++ A12 traces."""
import csv,json,math,sys,statistics
from pathlib import Path

def rows(p):
 with Path(p).open() as f:return list(csv.DictReader(f))
def close(a,b,at,rt):return math.isclose(float(a),float(b),abs_tol=at,rel_tol=rt)
def main(root):
 r=Path(root);ref=r/'reference';a=rows(r/'A/trace.csv');calls=rows(ref/'conditional_lm_calls.csv')[50:];gs=rows(ref/'first_block_stationarity.csv')[50:];fail=[];diffs=[]
 if len(a)!=150:fail.append('A_did_not_complete_reference_150_calls')
 for row,c,g in zip(a,calls,gs):
  dif={}
  for field,other,at,rt in [('error_before','error_before',2e-10,2e-12),('error_after','error_after',2e-10,2e-12),('step_norm','accepted_values_delta_norm',2e-9,2e-8),('step_max','accepted_values_max_abs_delta',2e-9,2e-8),('lambda_before','lambda_before',0,1e-14),('lambda_after','lambda_after',0,1e-14),('accepted','accepted_state_update',0,0),('rejected','rejected_lambda_trials_before_acceptance',0,0)]:
   dif[field]=abs(float(row[field])-float(c[other]))
   if not close(row[field],c[other],at,rt):fail.append(f"A_call{row['reference_call']}_{field}")
  for field in ['rotation','translation','velocity','accel_bias','gyro_bias','max_scaled_gradient']:
   dif[field]=abs(float(row[field])-float(g[field]))
   if not close(row[field],g[field],2e-9,2e-8):fail.append(f"A_call{row['reference_call']}_{field}")
  diffs.append({'call':int(row['reference_call']),**dif})
 def summary(trace):
  last=trace[-1];grad=[float(t['max_scaled_gradient']) for t in trace[-20:]]
  return {'calls':len(trace),'initial_error':float(trace[0]['error_before']),'final_error':float(last['error_after']),'final_scaled_gradient':float(last['max_scaled_gradient']),'generic':last['generic']=='1','stationary':last['stationary']=='1','converged_generic_AND_stationarity':last['generic']=='1' and last['stationary']=='1','accepted':sum(int(t['accepted']) for t in trace),'rejected':sum(int(t['rejected']) for t in trace),'lambda_final':float(last['lambda_after']),'iterate_seconds':sum(float(t['iterate_seconds']) for t in trace),'elapsed_seconds':float(last['elapsed_seconds']),'last20_gradient':{'min':min(grad),'median':statistics.median(grad),'max':max(grad)},'clip_low_total':sum(int(t['clip_low']) for t in trace),'clip_high_total':sum(int(t['clip_high']) for t in trace)}
 result={'A_reproduces_A11':not fail,'failures':fail,'A':summary(a) if a else {'status':'NO_COMPLETED_CALL'},'B':{'status':'NOT_RUN_PENDING_A_REPRODUCTION' if not fail else 'NOT_RUN_A_NOT_REPRODUCED'}}
 if (r/'B/trace.csv').exists():
  b=rows(r/'B/trace.csv');result['B']=summary(b)
  result['comparable']=not fail
  if not fail:
   sa,sb=result['A'],result['B'];result['B_local_fixed_budget_improvement']=(sb['converged_generic_AND_stationarity'] and not sa['converged_generic_AND_stationarity']) or (sb['final_error']<=sa['final_error']+2e-10+2e-12*abs(sa['final_error']) and sb['final_scaled_gradient']<=.5*sa['final_scaled_gradient'])
 for name,obj in [('ARM_A_REPRODUCTION_DIFFERENCES.json',diffs),('ARM_AUDIT.json',result)]: (r/name).write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
 print(json.dumps(result,indent=2));return 0 if not fail else 1
if __name__=='__main__':sys.exit(main(sys.argv[1]))
