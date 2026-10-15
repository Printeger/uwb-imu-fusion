#!/usr/bin/env python3
"""SC8 independent timestamp/physical-point metrics. GT is evaluator-only."""
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def summary(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return dict(n=0, rmse=None, mean=None, median=None, p95=None, p99=None, max=None)
    b = np.sort(a)
    return dict(n=len(a), rmse=float(np.sqrt(np.mean(a*a))), mean=float(a.mean()),
                median=float(np.median(a)), p95=float(b[math.ceil(.95*len(b))-1]),
                p99=float(b[math.ceil(.99*len(b))-1]), max=float(b[-1]))


def validate(t, p, q):
    t, p, q = np.asarray(t), np.asarray(p), np.asarray(q)
    if t.ndim != 1 or p.shape != (len(t), 3) or q.shape != (len(t), 4):
        raise ValueError('pose shape')
    if not all(np.isfinite(x).all() for x in (t,p,q)) or np.any(np.diff(t) <= 0):
        raise ValueError('nonfinite, duplicate or unordered pose')
    if np.any(abs(np.linalg.norm(q,axis=1)-1) > 1e-6):
        raise ValueError('nonunit quaternion')
    return t.astype(float), p.astype(float), q.astype(float)


def sample(t, p, q, query, max_gap=.1):
    t,p,q = validate(t,p,q)
    query = np.asarray(query, float)
    idx = np.searchsorted(t, query)
    r = np.clip(idx,0,len(t)-1)
    l = np.maximum(r-1,0)
    exact = abs(t[r]-query) <= 1e-8
    ok = exact | ((idx > 0) & (idx < len(t)) & (t[r]-t[l] <= max_gap+1e-9))
    pp = np.full((len(query),3),np.nan)
    qq = np.full((len(query),4),np.nan)
    if ok.any():
        qq[ok] = Slerp(t,Rotation.from_quat(q))(np.clip(query[ok],t[0],t[-1])).as_quat()
        for j in range(3): pp[ok,j] = np.interp(query[ok],t,p[:,j])
    return ok,pp,qq


def evaluate(estimate, truth, lever=(0,0,0), max_gap=.1, times=None):
    """Poses contain TAG position and BODY orientation in the common world.

    Truth times define the common RPE pair grid. No alignment/time fitting.
    EST and GT interpolation never spans a gap larger than max_gap.
    """
    et,ep,eq=validate(*estimate); gt,gp,gq=validate(*truth)
    grid = et if times is None else np.asarray(times,float)
    ev,pe,qe=sample(et,ep,eq,grid,max_gap)
    gv,pt,qt=sample(gt,gp,gq,grid,max_gap)
    ok=ev & gv
    d=pe[ok]-pt[ok]
    re,rt=Rotation.from_quat(qe[ok]),Rotation.from_quat(qt[ok])
    result={'ape_m':summary(np.linalg.norm(d,axis=1)),
            'hpe_m':summary(np.linalg.norm(d[:,:2],axis=1)),
            'vpe_m':summary(abs(d[:,2])),
            'are_rad':summary((rt.inv()*re).magnitude()),
            'are_deg':summary(np.degrees((rt.inv()*re).magnitude())),
            'planned_grid':len(grid),'matched':int(ok.sum()),
            'coverage':float(ok.mean()) if len(ok) else None,
            'alignment':'NONE','true_bias':'NOT_EVALUATED_BY_POSE_ONLY_SCORER'}
    for dt in [.5,1.]:
        # Pairs start at the shared GT grid; same physical dt for every method.
        start = gt[(gt+dt <= gt[-1]+1e-9)]
        queries=np.r_[start,start+dt]
        a,es,qs=sample(et,ep,eq,queries,max_gap)
        b,ts,us=sample(gt,gp,gq,queries,max_gap)
        n=len(start); valid=a[:n]&a[n:]&b[:n]&b[n:]
        ii=np.where(valid)[0]; jj=ii+n
        v=(es[jj]-es[ii])-(ts[jj]-ts[ii])
        tag_err=np.linalg.norm(v,axis=1)
        # Gaps anywhere INSIDE the pair invalidate it, even with exact ends.
        for tt in [et,gt]:
            gap_ends=np.where(np.diff(tt)>max_gap+1e-9)[0]
            if len(gap_ends):
                bad=np.any((start[ii,None]<tt[gap_ends+1]) &
                           ((start[ii]+dt)[:,None]>tt[gap_ends]),axis=1)
                ii,jj,tag_err=ii[~bad],jj[~bad],tag_err[~bad]
        if not len(ii):
            for key in [f'rpe_tag_{dt:g}s_m',f'rpe_body_{dt:g}s_m',f'rpe_body_{dt:g}s_rad']:
                result[key]=summary([])
            continue
        er0,er1=Rotation.from_quat(qs[ii]),Rotation.from_quat(qs[jj])
        tr0,tr1=Rotation.from_quat(us[ii]),Rotation.from_quat(us[jj])
        eb0,eb1=es[ii]-er0.apply(lever),es[jj]-er1.apply(lever)
        tb0,tb1=ts[ii]-tr0.apply(lever),ts[jj]-tr1.apply(lever)
        de=er0.inv().apply(eb1-eb0); dg=tr0.inv().apply(tb1-tb0)
        body_err=np.linalg.norm(de-dg,axis=1)
        rot_err=((tr0.inv()*tr1).inv()*(er0.inv()*er1)).magnitude()
        assert np.all(tag_err <= np.linalg.norm(es[ii]-ts[ii],axis=1)+np.linalg.norm(es[jj]-ts[jj],axis=1)+1e-9)
        result[f'rpe_tag_{dt:g}s_m']=summary(tag_err)
        result[f'rpe_body_{dt:g}s_m']=summary(body_err)
        result[f'rpe_body_{dt:g}s_rad']=summary(rot_err)
    arrays=dict(t=grid[ok],estimate=pe[ok],truth=pt[ok],ape=np.linalg.norm(d,axis=1),
                hpe=np.linalg.norm(d[:,:2],axis=1),vpe=abs(d[:,2]),are=(rt.inv()*re).magnitude())
    return result,arrays
