"""Independent evaluator for the added component only. Never imported by estimator."""
import math
from pathlib import Path
import numpy as np
import evaluate_runs as ev
from nlos_injection import rows, sha


def ratio(a,b):return {'value':a/b if b else None,'numerator':a,'denominator':b,'status':'AVAILABLE' if b else 'UNDEFINED_ZERO_DENOMINATOR'}

def detection(records,truth_ids,start,success=True,planned_ids=None):
    planned=[r for r in records if r['valid']=='1' and r['planned']=='1']; universe={r['obs_id'] for r in planned} if planned_ids is None else set(planned_ids);truth=set(truth_ids)&universe
    tested={r['obs_id'] for r in planned if r['tested']=='1'}
    complete=success and tested==universe and bool(universe)
    out={'status':'AVAILABLE' if complete else 'UNAVAILABLE_FDE_INCOMPLETE','planned_count':len(universe),'tested_count':len(tested),'truth_count':len(truth),'detection_coverage':ratio(len(tested),len(universe)),'fp_semantics':'RELATIVE_TO_ADDED_COMPONENT_ONLY','delay_semantics':'OFFLINE_SUPPORT_LOCALIZATION_SENSOR_TIME_DELAY'}
    for name,field in [('two_sided_fault','fault_detected'),('positive_excess_candidate','nlos_candidate'),('temporal_support','segment_id')]:
        if not complete:out[name]={'status':'UNAVAILABLE_FDE_INCOMPLETE','tp':None,'fp':None,'fn':None};continue
        pred={r['obs_id'] for r in planned if (bool(r[field]) if field=='segment_id' else r[field]=='1')}
        tp=len(pred&truth);fp=len(pred-truth);fn=len(truth-pred)
        hits=[float(r['sensor_time']) for r in planned if r['obs_id'] in pred&truth]
        out[name]={'status':'AVAILABLE','tp':tp,'fp':fp,'fn':fn,'precision':ratio(tp,tp+fp),'recall':ratio(tp,tp+fn),'f1':ratio(2*tp,2*tp+fp+fn),'first_tp_delay_s':min(hits)-start if hits else None,'first_tp_delay_status':'AVAILABLE' if hits else 'UNAVAILABLE_NO_TRUE_POSITIVE','first_tp_delay_truth_denominator':len(truth),'obs_set_iou':ratio(tp,len(pred|truth))}
    return out

def correction(delta):
    if delta is None or not math.isfinite(delta):return {'status':'UNAVAILABLE'}
    error=delta-.5
    return {'status':'AVAILABLE','delta_m':delta,'error_m':error,'relation':'equal' if abs(error)<=1e-9 else ('under' if error<0 else 'over'),
            'bin':'zero' if delta==0 else ('partial' if 0<delta<.45 else ('near-full' if .45<=delta<=.5 else 'over-full'))}

def correction_delivery(delta,decision_use,final_use,fallback,success):
    corrected=bool(decision_use and final_use and not fallback and success)
    return dict(proposed=correction(delta),actual=correction(delta) if corrected else {'status':'UNAVAILABLE_NOT_CORRECTED'},actual_offset_m=delta if corrected else None,corrected=corrected)

def overlap(s,e,start,end):return max(0.,min(e,end)-max(s,start))

def segment_metric(records,truth_ids,start,end,c_hat=None):
    ids={r['obs_id'] for r in records};affected=ids&set(truth_ids);times=[float(r['sensor_time']) for r in records]
    fraction=len(affected)/len(ids) if ids else None
    weights=[1/float(r.get('ledger_nominal_sigma_m',1.))**2 for r in records]
    weighted=sum(w for r,w in zip(records,weights) if r['obs_id'] in affected)/sum(weights) if weights else None
    return dict(obs_count=len(ids),injected_obs_count=len(affected),injected_fraction=ratio(len(affected),len(ids)),
                obs_set_iou=ratio(len(affected),len(ids|set(truth_ids))),truth_observation_coverage=ratio(len(affected),len(set(truth_ids))),
                mixed=bool(affected and len(affected)<len(ids)),contains_uninjected_observations=len(affected)<len(ids),
                interval_overlap_s=overlap(min(times),max(times),start,end) if times else None,
                weighted_injected_component_m=.5*weighted if weighted is not None else None,
                c_hat_stage2_m=c_hat,signed_error_vs_added_half_m=c_hat-.5 if c_hat is not None else None,
                absolute_error_vs_added_half_m=abs(c_hat-.5) if c_hat is not None else None,
                signed_error_vs_support_mean_m=c_hat-.5*weighted if c_hat is not None and weighted is not None else None)

def localization(run,protocol,window=None):
    run=Path(run);out=ev.trajectory_metrics(run,protocol)
    count=out.get('matched_count',{}).get('value')
    for key,v in out.items():
        if isinstance(v,dict) and v.get('status')=='AVAILABLE' and any(term in key for term in ('ATE','horizontal','height','axis','RPE')):
            v['denominator']=max(0,count-1) if key.startswith('RPE') and count is not None else count
    if not (run/'trajectory.tum').is_file():return out
    est=ev.load_tum(run/'trajectory.tum');planned=rows(run/'observations.csv')
    times={float(r['raw_time']) for r in planned if r['planned']=='1'}
    # Keyframe count is authoritative; source packet times need not equal keyframe timestamps.
    k={r['keyframe_id'] for r in planned if r['planned']=='1'}
    out['trajectory_coverage']=ratio(len({x[0] for x in est}&times),len(k));out['complete_planned_trajectory']=len(est)==len(k) and {x[0] for x in est}==times;out['trajectory_sha256']=sha((run/'trajectory.tum').read_bytes())
    if window is None:return out
    matches,_=ev.match_trajectories(est,ev.load_tum(Path(protocol['ground_truth'])),protocol['time_association'])
    if len(matches)<3:out['window']={'status':'UNAVAILABLE_INSUFFICIENT_MATCHES'};return out
    x=np.array([e[1] for e,g in matches]);y=np.array([g[1] for e,g in matches]);xc=x-x.mean(0);yc=y-y.mean(0)
    if np.linalg.matrix_rank(xc)<2:out['window']={'status':'UNAVAILABLE_ALIGNMENT_DEGENERATE'};return out
    u,_,vt=np.linalg.svd(xc.T@yc);d=np.eye(3);d[2,2]=np.linalg.det(vt.T@u.T);r=vt.T@d@u.T
    errors=(r@x.T).T+y.mean(0)-r@x.mean(0)-y
    mask=np.array([window[0]<=e[0]<=window[1] for e,g in matches]);z=errors[mask]
    out['window']={'status':'AVAILABLE' if len(z) else 'UNAVAILABLE_ZERO_MATCHES','count':len(z),'alignment':'FULL_TRAJECTORY_SCALE1_SE3','rmse_m':float(np.sqrt(np.mean(np.sum(z*z,axis=1)))) if len(z) else None,'p95_m':float(np.percentile(np.linalg.norm(z,axis=1),95)) if len(z) else None,'horizontal_rmse_m':float(np.sqrt(np.mean(np.sum(z[:,:2]**2,axis=1)))) if len(z) else None,'vertical_rmse_m':float(np.sqrt(np.mean(z[:,2]**2))) if len(z) else None}
    return out
