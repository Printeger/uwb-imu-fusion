#!/usr/bin/env python3
"""Independent pairing/statistics only; never estimates a trajectory."""
import argparse,csv,json,hashlib
from pathlib import Path
import numpy as np
from scipy.stats import chi2

def read(p):
 with p.open() as f:return list(csv.DictReader(f))
def matrix(p):return np.loadtxt(p,delimiter=',',ndmin=2)
def stats(x):
 x=np.asarray(x,dtype=float)
 return dict(zip(['min','p10','p50','p90','max'],map(float,np.quantile(x,[0,.1,.5,.9,1]))))
def test(e,C):
 w,V=np.linalg.eigh((C+C.T)/2);tol=max(1e-12,1e-10*max(abs(w)));keep=w>tol
 if min(w)<-tol or not any(keep):return {'valid':False,'spectrum':w.tolist(),'reason':'NON_PSD_OR_ZERO_RANK'}
 q=V.T@e;T=float(np.sum(q[keep]**2/w[keep]));nu=int(sum(keep));threshold=float(chi2.ppf(.99,nu))
 return {'valid':True,'statistic':T,'rank':nu,'threshold':threshold,'rejected':T>threshold,'spectrum':w.tolist(),'rank_threshold':tol,'null_component_norm':float(np.linalg.norm(q[~keep]))}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);ap.add_argument('truth',type=Path);ap.add_argument('output',type=Path);a=ap.parse_args()
 if (a.output/'diagnosis_seal.json').exists():raise RuntimeError('sealed diagnosis cannot be overwritten')
 a.output.mkdir(parents=True,exist_ok=True)
 truth=json.loads(a.truth.read_text());ids=list(map(str,truth['affected_obs_ids']));physical=read(a.run/'physical_pair.csv');physical={x['obs_id']:x for x in physical}
 c={x['obs_id']:x for x in read(a.run/'clean/fde_observations.csv')};i={x['obs_id']:x for x in read(a.run/'injected/fde_observations.csv')}
 ledger={arm:{x['obs_id']:x for x in read(a.run/arm/'observations.csv')} for arm in ['clean','injected']}
 rows=[]
 for oid in ids:
  row={'obs_id':oid,'status':'PLANNED' if oid in physical else 'NOT_PLANNED'}
  for arm,table in [('clean',c),('injected',i)]:
   row.update({arm+'_'+k:v for k,v in table[oid].items() if k!='obs_id'})
   row.update({arm+'_ledger_'+k:v for k,v in ledger[arm][oid].items() if k!='obs_id'})
  if oid in physical:row.update({'physical_'+k:v for k,v in physical[oid].items() if k!='obs_id'})
  rows.append(row)
 fields=list(dict.fromkeys(k for row in rows for k in row))
 with (a.output/'fde_injected_observation_forensics.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
 summary={'pair_audit':json.loads((a.run/'pair_audit.json').read_text()),'truth_sha256':hashlib.sha256(a.truth.read_bytes()).hexdigest(),'run':str(a.run),'subset':json.loads((a.run/'subset_status.json').read_text())}
 planned=[x for x in physical.values() if x['held']=='1'];oids=[x['obs_id'] for x in planned]
 # Artifact field names follow the production writer, not inference from labels.
 print('FDE fields',list(c[oids[0]]))
 summary['planned_ids']=oids
 rc=np.array([float(c[x]['residual_m']) for x in oids]);ri=np.array([i[x]['residual_m'] for x in oids],dtype=float)
 vc=np.array([c[x]['residual_variance_m2'] for x in oids],dtype=float);vi=np.array([i[x]['residual_variance_m2'] for x in oids],dtype=float)
 summary['residual_diagnostics']={'clean_m':stats(rc),'injected_m':stats(ri),'shift_m':stats(ri-rc),'attenuation':stats((ri-rc)/-.5),'injected_negative_count':int(sum(ri<0)),'clean_variance_m2':stats(vc),'injected_variance_m2':stats(vi),'variance_ratio':stats(vi/vc),'clean_numerator_squared_m2':stats(rc**2),'injected_numerator_squared_m2':stats(ri**2),'clean_statistic':stats(rc**2/vc),'injected_statistic':stats(ri**2/vi),'clean_redundancy':stats(vc/np.array([float(c[x]['factor_sigma_m']) for x in oids])**2),'injected_redundancy':stats(vi/np.array([float(i[x]['factor_sigma_m']) for x in oids])**2)}
 for arm,tab in [('clean',c),('injected',i)]:
  tested=[x for x in tab.values() if x['tested']=='1'];positive=[x for x in tested if x['nlos_candidate']=='1'];tp=[x for x in positive if x['obs_id'] in oids];fp=[x for x in positive if x['obs_id'] not in oids] if arm=='injected' else positive
  summary[arm+'_detection']={'tested':len(tested),'point_positive':len(positive),'point_TP':len(tp) if arm=='injected' else 0,'point_FP':len(fp),'point_FPR':len(fp)/(len(tested)-len(oids) if arm=='injected' else len(tested)),'retained_support':sum(bool(x['segment_id']) for x in tested),'filter_reasons':{r:sum(x['candidate_filter_reason']==r for x in tested) for r in sorted({x['candidate_filter_reason'] for x in tested})}}
 # The complete input ledger comparison includes all raw records, not just factors.
 lc=ledger['clean'];li=ledger['injected'];assert lc.keys()==li.keys()
 identity_cols=['source_message','source_range','source_observation','raw_time','raw_tag_id','anchor_id','valid','planned','keyframe_id','nominal_sigma_m']
 assert all(all(lc[k][col]==li[k][col] for col in identity_cols) for k in lc)
 summary['raw_plan_identity_noise_equal']=True
 summary['diagnostic_truth_access']=True
 summary['production_truth_parameter_added']=False
 for arm,tab in [('clean',c),('injected',i)]:
  summary[arm]={'linear':json.loads((a.run/arm/'linear/statistics.json').read_text())}
 if summary['subset']['converged']:
  held=read(a.run/'held_out.csv');assert [x['obs_id'] for x in held]==oids
  hc=np.array([float(x['clean_residual']) for x in held]);hi=np.array([float(x['injected_residual']) for x in held]);sigma=np.array([float(x['sigma']) for x in held]);P=matrix(a.run/'subset_prediction_linear/Pww.csv');Cp=np.linalg.solve(P,np.eye(len(P)))
  summary['heldout_shift_max_error']=float(np.max(abs(hi-hc+.5)))
  summary['heldout_clean_m']=stats(hc);summary['heldout_injected_m']=stats(hi)
  summary['heldout_clean_test']=test(hc/sigma,Cp);summary['heldout_injected_test']=test(hi/sigma,Cp)
  np.savetxt(a.output/'heldout_covariance_m2.csv',sigma[:,None]*Cp*sigma[None,:],delimiter=',')
  for arm in ['clean','injected']:
   P0=matrix(a.run/arm/'linear/Pww.csv');K=matrix(a.run/arm/'linear/response.csv');ew=matrix(a.run/arm/'linear/window_raw_whitened.csv').ravel();pw=matrix(a.run/arm/'linear/window_projected.csv').ravel();delta=matrix(a.run/arm/'subset_delta.csv').ravel()
   # Correlated deletion covariance: D D^T=K0W PWW^-1 K0W^T.
   # Thin QR reduces the rank-30 support without a full n-by-n covariance.
   Q,R=np.linalg.qr(K,mode='reduced');Cr=R@np.linalg.solve(P0,R.T);dr=Q.T@delta;pred=K@np.linalg.solve(P0,pw)
   summary[arm]['separation']=test(dr,Cr)
   summary[arm]['separation'].update({'actual_tangent_norm':float(np.linalg.norm(delta)),'linear_predicted_norm':float(np.linalg.norm(pred)),'linear_nonlinear_difference_norm':float(np.linalg.norm(delta-pred)),'outside_linear_covariance_range_norm':float(np.linalg.norm(delta-Q@dr)),'linear_test':test(Q.T@pred,Cr),'interpretation':'LOCAL_LINEAR_DIAGNOSTIC; nonlinear discrepancy separately reported; no calibrated significance claim'})
   np.savetxt(a.output/(arm+'_separation_covariance_reduced.csv'),Cr,delimiter=',')
   np.savetxt(a.output/(arm+'_linear_predicted_delta.csv'),pred,delimiter=',')
   tr=read(a.run/arm/'trajectory.csv');sub=read(a.run/'subset_trajectory.csv');assert len(tr)==len(sub)
   sep=np.array([[float(s[k])-float(t[k]) for k in ['x','y','z']] for s,t in zip(sub,tr)])
   summary[arm]['same_frame_position_separation_m']=stats(np.linalg.norm(sep,axis=1));np.savetxt(a.output/(arm+'_position_separation.csv'),sep,delimiter=',')
 (a.output/'fde_forensic_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
if __name__=='__main__':main()
