"""Replay old-admission local windows through existing native ImuPreintegrator."""
import sys,subprocess
from pathlib import Path
import numpy as np,pandas as pd
import diagnose_starloc_signals as d
import run_recover_vs_reject as r
out=Path(sys.argv[1]);binary=Path(sys.argv[2]);geo=r.read(r.ROOT/'experiments/recover_vs_reject_gt_geometry.json');records=[];meta=[]
for name in r.RECORDINGS:
 src=r.ROOT/'data/starloc/data'/name;im=pd.read_csv(src/'imu.csv');u=pd.read_csv(src/'uwb.csv');gt=d.GroundTruth(u,np.array(geo['common_geometry']['T_rig_imu'])[:3,3]);off=geo['recordings'][name]['imu']['time_offset_s'];t=im.time_s.to_numpy()+off
 acc=d.Signal(t,im[['linear_acceleration_'+k for k in 'xyz']].to_numpy()@d.FLIP.T);gyro=d.Signal(t,im[['angular_velocity_'+k for k in 'xyz']].to_numpy()@d.FLIP.T)
 windows=pd.read_csv(out/(name+'_closure.csv'));windows=windows[windows.variant=='old']
 for w in windows.itertuples():
  t0=w.time_s;t1=t0+w.duration_s;ts=np.r_[t0,t[(t>t0)&(t<t1)],t1];head=np.r_[len(ts),t0,t1,gt.pos([t0])[0],gt.velocity(t0),gt.sl(t0).as_matrix().ravel(),gt.pos([t1])[0],gt.velocity(t1),gt.sl(t1).as_matrix().ravel()]
  records.append(' '.join(format(v,'.17g') for v in head)+'\n'+'\n'.join(' '.join(format(v,'.17g') for v in row) for row in np.column_stack([ts,acc.at(ts),gyro.at(ts)])))
  meta.append(dict(recording=name,time_s=t0,duration_s=w.duration_s,python_rotation_deg=w.rotation_error_deg,python_position_m=w.position_error_m))
f=out/'native_input.txt';f.write_text(str(len(records))+'\n'+'\n'.join(records));cmd=[str(binary),str(f),str(out/'native_output.csv')];p=subprocess.run(cmd);assert p.returncode==0
z=pd.read_csv(out/'native_output.csv',header=None,names=['index','native_rotation_deg','native_position_m','native_velocity_mps']);a=pd.concat([pd.DataFrame(meta),z],axis=1);a.to_csv(out/'native_closure.csv',index=False);rows=[]
for (name,T),g in a.groupby(['recording','duration_s']):rows.append(dict(recording=name,duration_s=T,n=len(g),rotation_deg=d.stats(g.native_rotation_deg),position_m=d.stats(g.native_position_m),python_native_rotation_difference_deg=d.stats(abs(g.native_rotation_deg-g.python_rotation_deg)),python_native_position_difference_m=d.stats(abs(g.native_position_m-g.python_position_m))))
r.write(out/'native_summary.json',rows);r.write(out/'native_execution.json',dict(command=cmd,exit_code=0,binary_sha256=r.sha(binary),source_sha256=r.sha(r.ROOT/'experiments/scripts/starloc_preintegration_check.cpp'),backend_sha256=r.sha(r.WS/'devel/lib/libuwb_imu_fgo.so'),semantics='EXISTING_IntegrateBetween_RIGHT_ENDPOINT_GTSAM_COMBINED_ZERO_BIAS_GT_START_NO_GRAPH_SOLVE'));print(rows)
