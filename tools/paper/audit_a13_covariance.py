#!/usr/bin/env python3
"""Static PIM covariance/source attribution. No navigation solve or truth reads."""
import csv,json,sys,math
from pathlib import Path
import numpy as np

def rows(p):
 with Path(p).open() as f:return list(csv.DictReader(f))
def write(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def maxabs(x):return float(np.max(np.abs(x)))
def matches(a,b,at=1e-12,rt=1e-10):return maxabs(a-b)<=at+rt*maxabs(b)

def main(root):
 r=Path(root);o=r/'static';m={};seen=set()
 for x in rows(o/'matrices.csv'):
  key=(int(x['factor_index']),x['name']);i,j=int(x['row']),int(x['col']);assert (key,i,j) not in seen;seen.add((key,i,j))
  if key not in m:m[key]=np.full((int(x['nrows']),int(x['ncols'])),np.nan)
  m[key][i,j]=float(x['value'])
 assert all(np.isfinite(v).all() for v in m.values())
 failures=[];checks={}
 def ck(name,ok):
  checks[name]=bool(ok)
  if not ok:failures.append(name)
 factors=rows(o/'factor_curvature.csv');pims=rows(o/'pim_summary.csv');steps=rows(o/'propagation_steps.csv');orig=json.loads((r/'reference/STATIC_AUDIT.json').read_text());frozen=orig['physical_spectra']['stationarity_physical']['weak_modes'][0]
 ck('counts',len(factors)==371 and len(pims)==40 and len(steps)==1600)
 kappa=math.fsum(float(x['kappa']) for x in factors);ck('same_frozen_eigen_direction_curvature',abs(kappa-frozen['physical_curvature'])<=1e-10+1e-8*abs(frozen['physical_curvature']))
 cross=rows(o/'navigation_cross_curvature.csv');ck('all_navigation_cross_terms',len(cross)==371*25)
 for f in factors:
  fi=int(f['factor_index']);ks=math.fsum(float(x['value']) for x in cross if int(x['factor_index'])==fi)
  ck(f'factor_{fi}_cross_sum',abs(ks-float(f['kappa']))<=1e-10+1e-8*abs(float(f['kappa'])))
 sources=['measurement_acc','measurement_gyro','integration','bias_RW_acc','bias_RW_gyro','biasAccOmegaInt_acc','biasAccOmegaInt_gyro','biasAccOmegaInt_cross'];parts=[];blockrows=[];precisionblocks=[];pim_metrics=[];aggregate={s:0.0 for s in sources};resblocks=['rotation','position','velocity','accel_bias','gyro_bias']
 for pim in pims:
  fi=int(pim['factor_index']);get=lambda name:m[(fi,name)];Sigma=get('actual_PIM_covariance');R=get('actual_noise_R');J=get('actual_whitened_J');H=get('unwhitened_H');u=get('local_frozen_direction').ravel();z=get('unwhitened_z').ravel();white=get('actual_white_z').ravel();K=get('biasAccOmegaInt');
  ck(f'{fi}_K_actual_I6',np.array_equal(K,np.eye(6)))
  expected={'accelerometerCovariance':.002**2,'gyroscopeCovariance':.0002**2,'integrationCovariance':1e-9,'biasAccCovariance':.01**2,'biasOmegaCovariance':.00002**2}
  ck(f'{fi}_declared_other_parameters',all(matches(get(n),np.eye(3)*v,1e-25,1e-14) for n,v in expected.items()))
  ck(f'{fi}_reset_semantics',float(pim['reset_cov_max'])==0 and float(pim['reset_biasAccOmegaInt_difference'])==0)
  ck(f'{fi}_replay_native_matches_PIM',float(pim['final_covariance_replay_max_diff'])<=1e-12+1e-10*maxabs(Sigma) and float(pim['mean_replay_max_diff'])<=1e-12+1e-10*maxabs(get('preintegrated')))
  ck(f'{fi}_full_noise_covariance_matches_PIM',matches(get('actual_noise_covariance'),Sigma,1e-10,1e-8))
  ck(f'{fi}_whitening_full_cross_covariance',matches(R@Sigma@R.T,np.eye(15),1e-10,1e-8) and matches(R@H,J,1e-10,1e-8) and matches(R@z,white,1e-10,1e-8) and matches(H@u,z,1e-10,1e-8))
  S={s:np.zeros((15,15)) for s in sources};localsteps=[x for x in steps if int(x['factor_index'])==fi];Qa=get('accelerometerCovariance');Qg=get('gyroscopeCovariance')
  for x in localsteps:
   dt=float(x['dt']);F=np.array([[float(x[f'F_{i}_{j}']) for j in range(15)] for i in range(15)]);V=F[6:9,9:12];T=F[:3,12:15];Q={s:np.zeros((15,15)) for s in sources}
   Q['measurement_acc'][6:9,6:9]=V@Qa@V.T/dt;Q['measurement_gyro'][:3,:3]=T@Qg@T.T/dt;Q['integration'][3:6,3:6]=dt*get('integrationCovariance');Q['bias_RW_acc'][9:12,9:12]=dt*get('biasAccCovariance');Q['bias_RW_gyro'][12:15,12:15]=dt*get('biasOmegaCovariance');Q['biasAccOmegaInt_acc'][6:9,6:9]=V@K[:3,:3]@V.T/dt;Q['biasAccOmegaInt_gyro'][:3,:3]=T@K[3:,3:]@T.T/dt;Q['biasAccOmegaInt_cross'][6:9,:3]=V@K[3:,:3]@T.T;Q['biasAccOmegaInt_cross'][:3,6:9]=Q['biasAccOmegaInt_cross'][6:9,:3].T
   for s in sources:S[s]=F@S[s]@F.T+Q[s]
  ck(f'{fi}_independent_propagation_decomposition',all(matches(S[s],get('component_'+s)) for s in sources) and matches(sum(S.values()),Sigma))
  ck(f'{fi}_cross_parameter_zero_but_covariance_cross_retained',np.array_equal(get('component_biasAccOmegaInt_cross'),np.zeros((15,15))) and maxabs(Sigma-np.diag(np.diag(Sigma)))>0)
  v=np.linalg.solve(Sigma,z);k=float(z@v);native=float(white@white);ck(f'{fi}_full_precision_direction_curvature',abs(k-native)<=1e-10+1e-8*abs(native))
  # Fixed full covariance sensitivity, not the sum of inverse source covariances.
  vals=[]
  for s in sources:
   term=float(v@get('component_'+s)@v);aggregate[s]+=term;vals.append(term);parts.append({'factor_index':fi,'source':s,'attribution_v_Sigma_s_v':term,'derivative_kappa_wrt_source_multiplier':-term,'fraction_of_kappa':term/k if k else None})
   C=get('component_'+s)
   for i in range(5):
    for j in range(5):blockrows.append([fi,s,resblocks[i],resblocks[j],float(v[i*3:i*3+3]@C[i*3:i*3+3,j*3:j*3+3]@v[j*3:j*3+3])])
  ck(f'{fi}_source_attribution_sum',abs(math.fsum(vals)-k)<=1e-10+1e-8*abs(k))
  precision=R.T@R
  for i in range(5):
   for j in range(5):precisionblocks.append([fi,resblocks[i],resblocks[j],float(z[i*3:i*3+3]@precision[i*3:i*3+3,j*3:j*3+3]@z[j*3:j*3+3])])
  ck(f'{fi}_density_ratio_identity',matches(get('component_biasAccOmegaInt_acc'),get('component_measurement_acc')*(1/.002**2)) and matches(get('component_biasAccOmegaInt_gyro'),get('component_measurement_gyro')*(1/.0002**2)))
  navtraces={block:float(np.trace(Sigma[3*i:3*i+3,3*i:3*i+3])) for i,block in enumerate(resblocks)}
  componenttraces={s:{block:float(np.trace(get('component_'+s)[3*i:3*i+3,3*i:3*i+3])) for i,block in enumerate(resblocks)} for s in sources}
  pim_metrics.append({'factor_index':fi,'native_kappa':native,'kappa_from_covariance_solve':k,'K_attribution_fraction':sum(parts[-8+i]['attribution_v_Sigma_s_v'] for i in (5,6,7))/k,'whitening_identity_max_diff':maxabs(R@Sigma@R.T-np.eye(15)),'covariance_cross_max':maxabs(Sigma-np.diag(np.diag(Sigma))),'trace_by_residual_block':navtraces,'source_trace_by_residual_block':componenttraces,'biasHat':get('biasHat').ravel().tolist(),'bias_i_at_checkpoint':get('bias_i_at_checkpoint').ravel().tolist()})
 for file,header,data in [('covariance_source_block_attribution.csv',['factor_index','source','block_row','block_col','value'],blockrows),('full_precision_residual_cross_terms.csv',['factor_index','block_row','block_col','value'],precisionblocks)]:
  with (o/file).open('w') as f:w=csv.writer(f);w.writerow(header);w.writerows(data)
 with (o/'covariance_source_attribution.csv').open('w') as f:w=csv.DictWriter(f,list(parts[0]));w.writeheader();w.writerows(parts)
 counts={};curv={}
 for f in factors:
  name='CombinedImuFactor' if 'CombinedImuFactor' in f['dynamic_type'] else ('prior' if int(f['factor_index'])<3 else 'UWB');counts[name]=counts.get(name,0)+1;curv[name]=curv.get(name,0)+float(f['kappa'])
 k_total=curv['CombinedImuFactor'];a={s:{'attribution':v,'fraction_of_IMU_kappa':v/k_total} for s,v in aggregate.items()}
 result={'status':'PASS_STATIC_AUDIT' if not failures else 'FAILED_STATIC_AUDIT','checks_passed':sum(checks.values()),'check_count':len(checks),'failures':failures,'counts':counts,'frozen_direction_kappa':kappa,'A12_frozen_direction_kappa':frozen['physical_curvature'],'curvature_by_factor_class':curv,'IMU_fraction_of_total_curvature':k_total/kappa,'aggregate_covariance_source_attribution':a,'biasAccOmegaInt_is_I6_all40':all(checks[f"{int(p['factor_index'])}_K_actual_I6"] for p in pims),'source_attribution_interpretation':'v=Sigma_full^-1 z; a_s=v^T Sigma_s v; sum_s a_s=kappa; d(kappa)/d alpha_s=-a_s at alpha=1. Not additive inverse covariance information.','measurement_K_density_ratio_acc':1/.002**2,'measurement_K_density_ratio_gyro':1/.0002**2,'optimizer_iterate_calls':0,'production_covariance_changes':0,'PIM_metrics':pim_metrics}
 write(r/'COVARIANCE_AUDIT.json',result);write(r/'NUMERICAL_CHECKS.json',{'passed':not failures,'checks':checks,'failures':failures})
 print(json.dumps({k:v for k,v in result.items() if k!='PIM_metrics'},indent=2));return 0 if not failures else 1
if __name__=='__main__':sys.exit(main(sys.argv[1]))
