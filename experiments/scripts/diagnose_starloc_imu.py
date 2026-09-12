#!/usr/bin/env python3
"""GT-assisted finite frame/sign diagnostic, never estimator preprocessing by GT."""
import argparse,itertools,json
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp
import run_recover_vs_reject as rr


def proper_axes():
    out=[]
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([-1,1],repeat=3):
            a=np.eye(3)[list(perm)]*np.array(signs)[:,None]
            if np.linalg.det(a)>.5:out.append(a)
    return out


def transform(doc):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat([doc[k] for k in ['rot_x','rot_y','rot_z','w']]).as_matrix()
    t[:3,3]=[doc[k] for k in 'xyz'];return t


def common_geometry():
    cams=[];imus=[]
    for name in rr.RECORDINGS:
        c=rr.read(rr.ROOT/'data/starloc/data'/name/'calib.json')
        cams.append(transform(c['tf_cam_rig']));imus.append(transform(c['tf_cam_imu']))
    tcr=np.eye(4);tcr[:3,:3]=Rotation.from_matrix(np.array([t[:3,:3] for t in cams])).mean().as_matrix()
    tcr[:3,3]=np.mean([t[:3,3] for t in cams],0)
    # Names and translation are consistent with camera-coordinate rig origin.
    # Manufacturer camera-IMU transform represents IMU origin in camera coordinates.
    tci=imus[0]
    tri=np.linalg.inv(tcr)@tci
    tag=np.array([-.186,-.113,.098])
    return dict(T_camera_rig=tcr.tolist(),T_camera_imu=tci.tolist(),T_rig_imu=tri.tolist(),
                tag_in_rig_m=tag.tolist(),lever_imu_m=(tri[:3,:3].T@(tag-tri[:3,3])).tolist())


def samples(name,geo,clock_offset=0.):
    p=rr.ROOT/'data/starloc/data'/name
    u=pd.read_csv(p/'uwb.csv');im=pd.read_csv(p/'imu.csv')
    im['time_s'] += clock_offset
    g=u[['time_s','x','y','z','rot_x','rot_y','rot_z','w']].drop_duplicates('time_s').sort_values('time_s')
    t=g.time_s.to_numpy();xyz=g[['x','y','z']].to_numpy();rot=Rotation.from_quat(g[['rot_x','rot_y','rot_z','w']].to_numpy())
    tmin=max(t[0]+.25,im.time_s.min()+.25);tmax=min(t[-1]-.25,im.time_s.max()-.25)
    query=np.arange(tmin,tmax,.1);sl=Slerp(t,rot)
    tri=np.array(geo['T_rig_imu']);lever=tri[:3,3]
    out=[]
    for q in query:
        lo=np.searchsorted(t,q-.25);hi=np.searchsorted(t,q+.25)
        if hi-lo<5 or np.max(np.diff(t[lo:hi]))>.05:continue
        mi=im[(im.time_s>=q-.1)&(im.time_s<=q+.1)]
        if len(mi)<5 or mi.time_s.diff().max()>.05:continue
        # Smooth measurements using same central 0.2s interval as GT gyro.
        acc=mi[['linear_acceleration_'+a for a in 'xyz']].mean().to_numpy()
        gyro=mi[['angular_velocity_'+a for a in 'xyz']].mean().to_numpy()
        pose=sl(q);before=sl(q-.1);after=sl(q+.1)
        w=(before.inv()*after).as_rotvec()/.2
        x=t[lo:hi]-q;design=np.column_stack([np.ones(len(x)),x,x*x,x*x*x])
        a_rig=2*np.linalg.lstsq(design,xyz[lo:hi],rcond=None)[0][2]
        imu_xyz=xyz[lo:hi]+rot[lo:hi].apply(lever)
        a_imu=2*np.linalg.lstsq(design,imu_xyz,rcond=None)[0][2]
        f_rig=pose.inv().apply(a_rig-[0,0,-9.81]);f_imu=pose.inv().apply(a_imu-[0,0,-9.81])
        out.append([q,*acc,*gyro,*w,*f_rig,*f_imu])
    return np.array(out)


def metrics(measured,reference):
    err=measured-reference
    return dict(rmse=float(np.sqrt(np.mean(np.sum(err*err,axis=1)))),
                median_norm=float(np.median(np.linalg.norm(err,axis=1))),mean_error=err.mean(0).tolist())


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--clock',type=Path);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);geo=common_geometry();results=[]
    clocks={v['recording']:v['imu_to_uwb_offset_s'] for v in rr.read(a.clock)} if a.clock else {}
    for name in rr.RECORDINGS:
        d=samples(name,geo,clocks.get(name,0.));np.save(a.output/(name+'.npy'),d)
        acc=[];gyro=[]
        for mat in proper_axes():
            for sign in [-1,1]:
                acc.append(dict(R_rig_acc=mat.tolist(),sign=sign,**metrics(sign*d[:,1:4]@mat.T,d[:,13:16])))
            gyro.append(dict(R_rig_gyro=mat.tolist(),**metrics(d[:,4:7]@mat.T,d[:,7:10])))
        acc.sort(key=lambda x:x['rmse']);gyro.sort(key=lambda x:x['rmse'])
        best=acc[0];corrected=best['sign']*d[:,1:4]@np.array(best['R_rig_acc']).T
        results.append(dict(recording=name,samples=len(d),acceleration_candidates=acc,gyro_candidates=gyro,
            origin_rig=metrics(corrected,d[:,10:13]),origin_imu=metrics(corrected,d[:,13:16]),
            origin_signal=metrics(d[:,10:13],d[:,13:16])))
        print(name,'acc',acc[:2],'gyro',gyro[:2],flush=True)
    rr.write(a.output/'diagnosis.json',dict(schema='GT_ASSISTED_IMU_DIAG_V1',geometry=geo,recordings=results,
        source_sha256={name:{f:rr.sha(rr.ROOT/'data/starloc/data'/name/f) for f in ['uwb.csv','imu.csv','calib.json']} for name in rr.RECORDINGS}))

if __name__=='__main__':main()
