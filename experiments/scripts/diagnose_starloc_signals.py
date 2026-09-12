#!/usr/bin/env python3
"""GT-assisted local signal diagnostics, never a navigation estimator or input calibration."""
import argparse,json
from pathlib import Path
import numpy as np,pandas as pd
from scipy.spatial.transform import Rotation,Slerp
from scipy.optimize import minimize_scalar
from scipy.integrate import cumulative_trapezoid
import run_recover_vs_reject as rr
from diagnose_starloc_imu import proper_axes

FLIP=np.diag([1.,-1.,-1.]);GRAV=np.array([0.,0.,-9.81])

def rms(x):return float(np.sqrt(np.mean(np.square(x)))) if len(x) else None

def stats(x):
 x=np.asarray(x);return dict(n=len(x),rmse=rms(x),median=float(np.median(x)) if len(x) else None,p95=float(np.quantile(x,.95)) if len(x) else None)

def intervals_ok(t,a,b,gap=.05):
 t=np.asarray(t);a=np.asarray(a);b=np.asarray(b);bad=np.r_[0,np.cumsum(np.diff(t)>gap)]
 lo=np.searchsorted(t,a,side='right')-1;hi=np.searchsorted(t,b,side='left');valid=(lo>=0)&(hi<len(t));lo=np.clip(lo,0,len(t)-1);hi=np.clip(hi,0,len(t)-1)
 return valid & ((bad[hi]-bad[lo])==0)

class Signal:
 def __init__(self,t,x):
  self.t=np.asarray(t);self.x=np.asarray(x)
  assert np.isfinite(self.t).all() and np.all(np.diff(self.t)>0) and np.isfinite(self.x).all()
  self.integral=cumulative_trapezoid(self.x,self.t,axis=0,initial=0)
 def at(self,q):return np.column_stack([np.interp(q,self.t,self.x[:,k]) for k in range(3)])
 def primitive(self,q):
  q=np.asarray(q);j=np.clip(np.searchsorted(self.t,q,side='right')-1,0,len(self.t)-2);dt=q-self.t[j]
  return self.integral[j]+self.x[j]*dt[:,None]+.5*(self.x[j+1]-self.x[j])/(self.t[j+1]-self.t[j])[:,None]*dt[:,None]**2
 def mean(self,q,width):return (self.primitive(q+width/2)-self.primitive(q-width/2))/width

class GroundTruth:
 def __init__(self,u,lever):
  g=u.drop_duplicates('time_s').sort_values('time_s');self.t=g.time_s.to_numpy();self.r=Rotation.from_quat(g[['rot_x','rot_y','rot_z','w']]);self.sl=Slerp(self.t,self.r)
  self.rig=g[['x','y','z']].to_numpy();self.p=self.rig+self.r.apply(lever)
 def pos(self,q):return np.column_stack([np.interp(q,self.t,self.p[:,k]) for k in range(3)])
 def velocity(self,q):
  i=np.searchsorted(self.t,q-.25);j=np.searchsorted(self.t,q+.25);x=self.t[i:j]-q
  if len(x)<5 or not intervals_ok(self.t,[q-.25],[q+.25])[0]:return None
  return np.linalg.lstsq(np.column_stack([x**k for k in range(4)]),self.p[i:j],rcond=None)[0][1]

def propagate(t,acc,gyro,R0,gtrot=None):
 """Return relative displacement/velocity excluding v0 and gravity; SO3 midpoint."""
 rot=R0;dp=np.zeros(3);dv=np.zeros(3)
 for j,dt in enumerate(np.diff(t)):
  w=(gyro[j]+gyro[j+1])*.5;f=(acc[j]+acc[j+1])*.5
  mid=rot*Rotation.from_rotvec(w*dt*.5) if gtrot is None else gtrot[j]
  world=mid.apply(f);dp+=dv*dt+.5*world*dt*dt;dv+=world*dt
  rot=rot*Rotation.from_rotvec(w*dt)
 return rot,dp,dv

def run(out):
 out.mkdir(parents=True,exist_ok=False);geo=rr.read(rr.ROOT/'experiments/recover_vs_reject_gt_geometry.json');result=[];axes=proper_axes();tri=np.array(geo['common_geometry']['T_rig_imu']);axes += [tri[:3,:3],tri[:3,:3].T]
 for name in rr.RECORDINGS:
  src=rr.ROOT/'data/starloc/data'/name;u=pd.read_csv(src/'uwb.csv');im=pd.read_csv(src/'imu.csv',usecols=['time_s']+['linear_acceleration_'+k for k in 'xyz']+['angular_velocity_'+k for k in 'xyz'])
  gyro=Signal(im.time_s,im[['angular_velocity_'+k for k in 'xyz']]);acc=Signal(im.time_s,im[['linear_acceleration_'+k for k in 'xyz']]);gt=GroundTruth(u,tri[:3,3]);old=geo['recordings'][name]['imu']['time_offset_s']
  # Every tested shift has the same time support; timing fit uses physical gyro only.
  lo=max(gt.t[0]+.3,gyro.t[0]+2.1);hi=min(gt.t[-1]-.3,gyro.t[-1]-2.1);q=np.arange(lo,hi,.1)
  valid=intervals_ok(gt.t,q-.25,q+.25)
  # Reject windows intersecting an IMU gap under ANY shift, so loss counts are fixed.
  for shift in np.arange(-2,2.001,.01):valid &= intervals_ok(gyro.t,q-shift-.05,q-shift+.05)
  q=q[valid];wref=(gt.sl(q-.05).inv()*gt.sl(q+.05)).as_rotvec()/.1
  train=q<(lo+hi)/2;test=~train;moving=np.linalg.norm(wref,axis=1)>.1
  def loss(shift,mask=train):
   err=np.linalg.norm(gyro.mean(q-shift,.1),axis=1)-np.linalg.norm(wref,axis=1);a=np.abs(err[mask & moving]);return float(np.mean(np.where(a<=.2,.5*a*a,.2*(a-.1)))) if len(a) else float('inf')
  shifts=np.arange(-2,2.001,.01);cost=np.array([loss(s) for s in shifts]);seed=shifts[np.argmin(cost)];opt=minimize_scalar(loss,bounds=(max(-2,seed-.01),min(2,seed+.01)),method='bounded');best=float(opt.x)
  second=np.array([loss(s,test) for s in shifts]);test_best=float(shifts[np.argmin(second)])
  wm=gyro.mean(q-best,.1);axis_scores=[rms(np.linalg.norm(wm[train]@a.T-wref[train],axis=1)) for a in axes];rg=axes[int(np.argmin(axis_scores))]
  sync=[]
  for label,s in [('zero',0.),('old',old),('signal_train',best)]:
   for split,mask in [('train',train & moving),('test',test & moving)]:
    m=gyro.mean(q-s,.1);sync.append(dict(offset_name=label,offset_s=s,split=split,n=int(mask.sum()),norm_rmse_radps=rms((np.linalg.norm(m,axis=1)-np.linalg.norm(wref,axis=1))[mask]),vector_old_axes_rmse_radps=rms(np.linalg.norm((m@FLIP.T-wref)[mask],axis=1))))
  unit=[]
  for label,scale in [('rad/s',1.),('deg/s_to_rad/s',np.pi/180),('rad/s_misread_x180/pi',180/np.pi)]:
   unit.append(dict(unit=label,scale=scale,train=stats(np.linalg.norm(wm[train]@rg.T*scale-wref[train],axis=1)),test=stats(np.linalg.norm(wm[test]@rg.T*scale-wref[test],axis=1))))
  axis_rows=[dict(index=i,R=a.tolist(),train_rmse=axis_scores[i],test_rmse=rms(np.linalg.norm(wm[test]@a.T-wref[test],axis=1))) for i,a in enumerate(axes)]
  # Low motion GT-only selection, independent acc and gyro rotation checks.
  low=np.ones(len(q),bool);center=gt.pos(q);rc=gt.sl(q)
  for dt in np.linspace(-.25,.25,11):low &= (np.linalg.norm(gt.pos(q+dt)-center,axis=1)<=.02)&((rc.inv()*gt.sl(q+dt)).magnitude()<=.02)
  am=acc.mean(q-best,.2);gravity=rc.inv().apply(np.tile(-GRAV,(len(q),1)))
  lowtrain=low&train;lowtest=low&test
  ascores=[rms(np.linalg.norm((am@a.T-gravity)[lowtrain],axis=1)) for a in axes]
  ra=axes[int(np.argmin([x if x is not None else np.inf for x in ascores]))] if lowtrain.any() else FLIP
  gravity_rows=[]
  for label,a in [('identity',np.eye(3)),('old_flip',FLIP),('manufacturer_chain',tri[:3,:3]),('diagnostic_train',ra)]:
   for split,mask in [('train',lowtrain),('test',lowtest)]:
    z=am[mask]@a.T;ref=gravity[mask];angle=np.rad2deg(np.arccos(np.clip(np.sum(z*ref,axis=1)/(np.linalg.norm(z,axis=1)*9.81),-1,1)))
    gravity_rows.append(dict(axis=label,split=split,angle_deg=stats(angle),vector_error_mps2=stats(np.linalg.norm(z-ref,axis=1)),magnitude_mps2=stats(np.linalg.norm(z,axis=1))))
  # Local GT-initialized closure. These outputs cannot be interpreted as estimator ATE.
  closures=[];details=[]
  variants=[('zero_old_axes',0.,FLIP,FLIP,1.),('old',old,FLIP,FLIP,1.),('signal_old_axes',best,FLIP,FLIP,1.),('signal_diagnostic_axes',best,ra,rg,1.),('signal_deg_hypothesis',best,ra,rg,np.pi/180)]
  for duration in [.1,.5,1.]:
   starts=np.arange(lo,hi-1.,1.);allvalid=np.array([gt.velocity(float(t)) is not None for t in starts]) & intervals_ok(gt.t,starts-.25,starts+duration+.25)
   for _,shift,_,_,_ in variants:allvalid &= intervals_ok(gyro.t,starts-shift,starts+duration-shift)
   starts=starts[allvalid]
   for label,shift,A,W,scale in variants:
    local=[]
    for t0 in starts:
     t1=t0+duration;it=gyro.t[(gyro.t>t0-shift)&(gyro.t<t1-shift)]+shift;ts=np.r_[t0,it,t1];aa=acc.at(ts-shift)@A.T;ww=gyro.at(ts-shift)@W.T*scale;R0=gt.sl(t0);v0=gt.velocity(t0)
     rot,dp,dv=propagate(ts,aa,ww,R0);_,dp_gt,dv_gt=propagate(ts,aa,ww,R0,gt.sl((ts[:-1]+ts[1:])/2))
     disp=(gt.pos([t1])-gt.pos([t0]))[0];e=dp+v0*duration+.5*GRAV*duration**2-disp;eg=dp_gt+v0*duration+.5*GRAV*duration**2-disp
     row=dict(variant=label,duration_s=duration,time_s=float(t0),rotation_error_deg=float(np.rad2deg((rot.inv()*gt.sl(t1)).magnitude())),position_error_m=float(np.linalg.norm(e)),position_GT_rotation_error_m=float(np.linalg.norm(eg)),velocity_error_mps=float(np.linalg.norm(dv+GRAV*duration+v0-gt.velocity(t1))))
     local.append(row);details.append(row)
    closures.append(dict(variant=label,duration_s=duration,n=len(local),excluded_windows=int(len(allvalid)-sum(allvalid)),**{k:stats([v[k] for v in local]) for k in ['rotation_error_deg','position_error_m','position_GT_rotation_error_m','velocity_error_mps']}))
  selected=u[u.from_id==1];start=max(float(selected.time_s.min()),float(gyro.t[0]+old));end=min(float(selected.time_s.max()),float(gyro.t[-1]+old));kept=selected[(selected.time_s>=start)&(selected.time_s<=end)];planned=kept.iloc[::4]
  crop=dict(start_s=start,end_s=end,uwb_removed_start=int(sum(selected.time_s<start)),uwb_removed_end=int(sum(selected.time_s>end)),uwb_retained=len(kept),imu_corrected_first_s=float(gyro.t[0]+old),original_uwb_first_s=float(selected.time_s.min()),first5_new_plan_times_s=planned.head(5).time_s.tolist(),first5_new_plan_anchor_ids=planned.head(5).to_id.tolist(),crop_requires_reinitialization=True,accuracy_effect='NOT_RUN_NOT_ESTABLISHED')
  row=dict(recording=name,old_offset_s=old,signal_train_offset_s=best,signal_test_grid_minimum_s=test_best,sync=sync,gyro_units=unit,gyro_axis_candidates=axis_rows,selected_gyro_R=rg.tolist(),selected_acc_R=ra.tolist(),gravity=gravity_rows,closures=closures,crop=crop,raw_acc_norm=stats(np.linalg.norm(acc.x,axis=1)),source_hashes={f:rr.sha(src/f) for f in ['imu.csv','uwb.csv','calib.json']})
  result.append(row);rr.write(out/(name+'.json'),row);pd.DataFrame(details).to_csv(out/(name+'_closure.csv'),index=False);pd.DataFrame(dict(offset_s=shifts,train_loss=cost,test_loss=second)).to_csv(out/(name+'_time_loss.csv'),index=False)
  print(name,'old/signal/test offsets',old,best,test_best,'crop',crop,flush=True)
 rr.write(out/'results.json',result);rr.write(out/'manifest.json',dict(role='GT_ASSISTED_LOCAL_DIAGNOSTIC_NOT_ESTIMATOR',script_sha256=rr.sha(__file__),protocol_sha256=rr.sha(rr.ROOT/'experiments/STARLOC_SIGNAL_DIAGNOSTIC_PROTOCOL.md'),gt_initialization='LOCAL_CLOSURE_ONLY',bias='ZERO_NOT_FITTED',estimator_runs=0))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);run(p.parse_args().output.resolve())
