#!/usr/bin/env python3
"""Independently check exported fixed-offset values against raw ranges/poses."""
import csv,json,sys
from pathlib import Path
import numpy as np
from evaluate_runs import load_tum,quaternion_matrix
partial,full,output=map(Path,sys.argv[1:])
def rows(p):return list(csv.DictReader(p.open()))
reports=[]
for root in [partial,full]:
 final=root/'normal/final'
 compensation={r['segment_id']:r for r in rows(final/'fixed_compensations.csv')}
 poses=load_tum(final/'trajectory.tum')
 inputs={r['obs_id']:r for r in rows(root/'input_ranges.csv')}
 residuals=rows(final/'residuals.csv')
 ids=[r['obs_id'] for r in residuals]
 assert len(ids)==len(set(ids))==len(inputs)
 metadata=rows(final/'final_factor_metadata.csv')
 assert all(not any(key.startswith('c') for key in r['keys'].split(';')) for r in metadata)
 assert not rows(final/'segment_bias.csv')
 errors=[]
 for r in residuals:
  raw=inputs[r['obs_id']];pose=poses[int(raw['keyframe_id'])]
  lever=np.array([float(raw[k]) for k in ['lx','ly','lz']])
  anchor=np.array([float(raw[k]) for k in ['ax','ay','az']])
  delta=float(compensation[r['segment_id']]['delta_c_fixed_m']) if r['segment_id'] else 0.
  expected=np.linalg.norm(pose[1]+quaternion_matrix(pose[2])@lever-anchor)+float(raw['beta_m'])+delta-float(raw['raw_z_m'])
  errors.append(abs(expected-float(r['residual_value'])))
 assert max(errors)<1e-10,max(errors)
 for c in compensation.values():
  assert c['final_use']=='1' and float(c['delta_c_fixed_m'])>0
 reports.append(dict(root=str(root),residual_count=len(errors),max_residual_difference_m=max(errors),compensations=list(compensation.values())))
p,f=reports
assert {r['segment_id'] for r in p['compensations']}=={r['segment_id'] for r in f['compensations']}
for a,b in zip(p['compensations'],f['compensations']):
 assert a['c_hat_stage2_m']==b['c_hat_stage2_m'] and a['sigma_c_local_m']==b['sigma_c_local_m']
 assert float(a['delta_c_fixed_m'])==max(0.,float(a['c_hat_stage2_m'])-2*float(a['sigma_c_local_m']))
 assert b['delta_c_fixed_m']==b['c_hat_stage2_m']
output.write_text(json.dumps(dict(status='PASS',same_lcb_restoration_set=True,runs=reports),indent=2))
print('PASS: 16 raw-range residuals, no C, same recovery set, exact frozen offsets')
