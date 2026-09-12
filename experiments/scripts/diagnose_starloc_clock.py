"""Estimate one constant clock shift from published GT-to-GT pose only."""
import argparse
import run_recover_vs_reject as rr
import diagnose_starloc_imu as diag
import numpy as np,pandas as pd
from scipy.spatial import cKDTree
from scipy.optimize import minimize_scalar
from pathlib import Path
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();out=args.output
out.mkdir(parents=True,exist_ok=False)
results=[]
for n in rr.RECORDINGS:
 p=rr.ROOT/'data/starloc/data'/n;u=pd.read_csv(p/'uwb.csv');im=pd.read_csv(p/'imu.csv');cols=['x','y','z','rot_x','rot_y','rot_z','w']
 ux=u[cols].to_numpy();ix=im[cols].to_numpy();dist,j=cKDTree(ux).query(ix);good=dist<.001
 initial=float(np.median(u.time_s.values[j[good]]-im.time_s.values[good]))
 index=np.arange(0,len(im),10);ti=im.time_s.values[index];xi=ix[index]
 mask=(ti+initial-.05>u.time_s.min())&(ti+initial+.05<u.time_s.max());ti=ti[mask];xi=xi[mask]
 def errors(offset):
  pred=np.column_stack([np.interp(ti+offset,u.time_s,ux[:,k]) for k in range(7)])
  return pred-xi
 def obj(offset,mask=slice(None)):return np.mean(np.sum(errors(offset)[mask]**2,axis=1))
 opt=minimize_scalar(obj,bounds=(initial-.05,initial+.05),method='bounded',options={'xatol':1e-10})
 halves=[minimize_scalar(lambda x:obj(x,slice(a,b)),bounds=(initial-.05,initial+.05),method='bounded').x for a,b in [(0,len(ti)//2),(len(ti)//2,len(ti))]]
 r=dict(recording=n,imu_to_uwb_offset_s=float(opt.x),scale=1.,gt_pose_rmse_before=float(np.sqrt(obj(0))),gt_pose_rmse_after=float(np.sqrt(opt.fun)),half_offsets_s=halves,matched_count=int(good.sum()),source='GT_TO_GT_PUBLISHED_POSE_CORRESPONDENCE_NOT_ESTIMATOR_ERROR')
 results.append(r);print(r,flush=True)
rr.write(out/'clock_diagnosis.json',results)
