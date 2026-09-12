#!/usr/bin/env python3
"""Offline evaluator only. Never launches, configures, or feeds an estimator."""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import uuid
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
PRIVATE=ROOT.parents[1]/'evaluator_private/icra'
ASSOCIATION={'policy':'linear_position_slerp_orientation','max_bracket_gap_s':0.05,
             'exact_timestamp':'direct','extrapolation':'forbidden'}
FIELDS=['run_id','method','timestamp','obs_id','tag_id','anchor_id','measured_range','static_range_bias',
        'gt_range','raw_range_error','detector_candidate','segment_id','estimated_bias',
        'recovery_accepted','applied_correction','corrected_range','corrected_range_error',
        'planned','valid','evaluation_status','gt_left_timestamp','gt_right_timestamp',
        'gt_bracket_gap_s','detector_status','correction_status','reference_status','assumed_fields']


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def read_csv(p):
    with Path(p).open(newline='') as f:
        reader=csv.DictReader(f)
        if reader.fieldnames is None or len(set(reader.fieldnames))!=len(reader.fieldnames):
            raise ValueError('missing/duplicate CSV header')
        rows=list(reader)
        if any(None in r or any(v is None for v in r.values()) for r in rows):
            raise ValueError('malformed CSV row')
        return rows


def unique(rows,key):
    result={}
    for r in rows:
        if not r[key] or r[key] in result:raise ValueError('duplicate/empty '+key)
        result[r[key]]=r
    return result


def flag(s):
    if s not in ('0','1'):raise ValueError('expected explicit Boolean 0/1')
    return s=='1'


def number(x):
    x=float(x)
    if not math.isfinite(x):raise ValueError('nonfinite numeric value')
    return x


def rigid(x):
    T=np.asarray(x,dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-9,rtol=0):
        raise ValueError('invalid homogeneous transform')
    if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-7,rtol=0) or abs(np.linalg.det(T[:3,:3])-1)>1e-7:
        raise ValueError('transform rotation must be SO(3)')
    return T


def quaternion(q):
    q=np.asarray(q,dtype=float);n=np.linalg.norm(q)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(n-1)>1e-3:raise ValueError('invalid GT quaternion')
    return q/n


def rotation(q):
    x,y,z,w=quaternion(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def slerp(a,b,f):
    a=quaternion(a);b=quaternion(b);dot=float(a@b)
    if dot<0:b=-b;dot=-dot
    if dot>0.9995:return quaternion((a+f*(b-a))/np.linalg.norm(a+f*(b-a)))
    theta=math.acos(np.clip(dot,-1,1))
    return (math.sin((1-f)*theta)*a+math.sin(f*theta)*b)/math.sin(theta)


def load_gt(path):
    rows=[]
    for line in Path(path).read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#'):continue
        v=[number(x) for x in line.split()]
        if len(v)!=8:raise ValueError('GT TUM requires eight columns')
        if rows and v[0]<=rows[-1][0]:raise ValueError('GT times not strictly increasing')
        rows.append((v[0],np.array(v[1:4]),quaternion(v[4:8])))
    if not rows:raise ValueError('GT empty')
    return rows


def associate(t,gt,times,max_gap=0.05):
    j=bisect.bisect_left(times,t)
    if j<len(times) and times[j]==t:return gt[j][1],gt[j][2],t,t,'AVAILABLE'
    if j==0 or j==len(times):return None,None,None,None,'OUTSIDE_GT_SUPPORT'
    left,right=gt[j-1],gt[j];gap=right[0]-left[0]
    if gap>max_gap:return None,None,left[0],right[0],'GT_GAP_EXCEEDED'
    f=(t-left[0])/gap
    return left[1]+f*(right[1]-left[1]),slerp(left[2],right[2],f),left[0],right[0],'AVAILABLE'


def calibration(path, allow_identity_assumptions=False):
    d=json.loads(Path(path).read_text())
    if d.get('schema')!='isas_walk1_range_calibration_v1' or d.get('recording_id')!='ISAS-Walk1':
        raise ValueError('only ISAS-Walk1 calibration schema is supported')
    if d.get('uwb_measurement_type') not in ['ABSOLUTE_TOA','ABSOLUTE_TWR','ABSOLUTE_RANGE']:
        raise ValueError('TDoA/unknown measurements forbidden')
    if d.get('association')!=ASSOCIATION:raise ValueError('association protocol must be predeclared 0.05s rule')
    required=['T_anchor_GT','T_marker_IMU','lever_IMU_tag_m','anchors_m','static_bias_m','GT_time_to_sensor']
    assumed=[]
    missing=[k for k in required if d.get(k) is None]
    for k in required:
        if k in missing:continue
        evidence=d.get('sources',{}).get(k)
        if evidence and evidence.get('origin')=='USER_AUTHORIZED_ASSUMPTION':
            if not allow_identity_assumptions:raise ValueError('identity assumptions require explicit opt-in')
            if k not in ('T_anchor_GT','T_marker_IMU','GT_time_to_sensor'):
                raise ValueError('assumptions forbidden for bias, anchors or lever')
            expected={'scale':1,'offset_s':0} if k=='GT_time_to_sensor' else np.eye(4).tolist()
            if d[k]!=expected or evidence.get('independent') is not False or evidence.get('fit_on_evaluation_data') is not False:
                raise ValueError('only declared identity/common-clock assumptions allowed')
            assumed.append(k)
        if not evidence or not evidence.get('recording_id') or not (evidence.get('independent') is True or evidence.get('origin') == 'PUBLISHED_DATASET_CONSTANT' or k in assumed):
            missing.append(k+'_INDEPENDENT_SOURCE');continue
        if evidence['recording_id']=='ISAS-Walk1' or evidence.get('fit_on_evaluation_data') is True:raise ValueError('target recording cannot calibrate itself')
        p=Path(evidence['path'])
        if not p.is_file() or sha(p)!=evidence['sha256']:raise ValueError('calibration evidence identity mismatch: '+k)
    d['assumed_fields']=assumed
    d['reference_status']='ASSUMED_GEOMETRY_AND_CLOCK' if assumed else 'SOURCE_DECLARED'
    for value in (d.get('static_bias_m') or {}).values():number(value)
    for k in ('T_anchor_GT','T_marker_IMU'):
        if d.get(k) is not None:rigid(d[k])
    if missing:return d,None,missing
    T_AG=rigid(d['T_anchor_GT']);T_MI=rigid(d['T_marker_IMU'])
    lever=np.asarray(d['lever_IMU_tag_m'],dtype=float)
    if lever.shape!=(3,) or not np.isfinite(lever).all():raise ValueError('invalid IMU/tag lever')
    for anchor,p in d['anchors_m'].items():
        p=np.asarray(p,dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('invalid anchor '+anchor)
    if not d['anchors_m'] or not d['static_bias_m']:raise ValueError('empty anchor/bias table')
    for b in d['static_bias_m'].values():number(b)
    clock=d['GT_time_to_sensor'];scale=number(clock['scale']);offset=number(clock['offset_s'])
    if scale<=0:raise ValueError('invalid clock scale')
    gt_path=Path(d['gt_path'])
    if sha(gt_path)!=d['gt_sha256']:raise ValueError('GT hash mismatch')
    gt=[(number(scale*t+offset),p,q) for t,p,q in load_gt(gt_path)]
    if any(a[0]>=b[0] for a,b in zip(gt,gt[1:])):raise ValueError('clock mapping collapsed GT timestamps')
    return d,(T_AG,T_MI,lever,gt),[]


def corrections(obs,method,masks,comps,segments,valid_result):
    """Join actual final-use artifacts, never infer acceptance from c_hat alone."""
    oid=obs['obs_id'];m=masks.get(oid)
    if not valid_result:return None,'',None,None,None,'ESTIMATION_FAILED','ESTIMATION_FAILED'
    if method in ('M0','M1'):return None,'',None,False,0.,'NOT_RUN_BASELINE','NO_COMPENSATION'
    if not m:
        if flag(obs['planned']):raise ValueError('planned observation has no final mask: '+oid)
        return None,'',None,False,0.,'NOT_TESTED_UNPLANNED','NO_COMPENSATION'
    candidate=flag(m['candidate']);segment=m['segment_id']
    if not candidate:
        if segment:raise ValueError('noncandidate has segment')
        return False,'',None,False,0.,'NONCANDIDATE','NO_COMPENSATION'
    if not segment or segment not in segments:raise ValueError('candidate missing Stage2 segment')
    s=segments[segment]
    if s['anchor_id']!=obs['anchor_id'] or s['tag_id']!=obs['raw_tag_id']:raise ValueError('segment/link mismatch')
    estimate=number(s['amplitude_m'])
    if estimate<0:raise ValueError('negative dynamic bias')
    accepted=flag(m['decision_use']) and flag(m['final_use']) and not flag(m['fallback_use'])
    if method=='M3':
        if accepted:raise ValueError('suppress_all cannot accept recovery')
        return True,segment,estimate,False,0.,'CANDIDATE','SUPPRESSED'
    c=comps.get(segment)
    if c is None:raise ValueError('M4 missing fixed compensation')
    if c['tag_id']!=obs['raw_tag_id'] or c['anchor_id']!=obs['anchor_id']:raise ValueError('compensation link mismatch')
    delta=number(c['delta_c_fixed_m']);hat=number(c['c_hat_stage2_m'])
    if delta<0 or abs(hat-estimate)>1e-10:raise ValueError('invalid compensation/Stage2 identity')
    if accepted and not(flag(c['decision_use']) and flag(c['final_use'])):raise ValueError('acceptance artifact mismatch')
    if accepted and abs(delta-hat)>1e-10:raise ValueError('M4 full correction does not equal Stage2 amplitude')
    return True,segment,estimate,accepted,delta if accepted else 0.,'CANDIDATE','APPLIED' if accepted else 'SUPPRESSED_OR_FALLBACK'


def evaluate_rows(observations,method,run_id,d,geometry,missing,masks=None,comps=None,segments=None,valid_result=True):
    unique(observations,'obs_id');masks=masks or {};comps=comps or {};segments=segments or {}
    if set(masks)-{r['obs_id'] for r in observations}:raise ValueError('mask references unknown obs_id')
    if set(comps)-set(segments):raise ValueError('compensation references unknown Stage2 segment')
    result=[]
    if geometry:T_AG,T_MI,lever,gt=geometry;times=[r[0] for r in gt]
    for obs in observations:
        t=number(obs['raw_time']);z=number(obs['raw_z_m'])
        candidate,segment,estimate,accepted,delta,detector_status,correction_status=corrections(obs,method,masks,comps,segments,valid_result)
        r=dict.fromkeys(FIELDS)
        r.update(reference_status=d.get('reference_status','SOURCE_DECLARED'),assumed_fields=';'.join(d.get('assumed_fields',[])),
                 run_id=run_id,method=method,timestamp=t,obs_id=obs['obs_id'],tag_id=obs['raw_tag_id'],anchor_id=obs['anchor_id'],
                 measured_range=z,static_range_bias=(d.get('static_bias_m') or {}).get(obs['anchor_id']),detector_candidate=candidate,segment_id=segment,estimated_bias=estimate,
                 recovery_accepted=accepted,applied_correction=delta,corrected_range=z-delta if delta is not None else None,
                 planned=flag(obs['planned']),valid=flag(obs['valid']),detector_status=detector_status,correction_status=correction_status)
        if missing:r['evaluation_status']='UNAVAILABLE_CALIBRATION:'+','.join(missing)
        elif not r['valid'] or z<=0:r['evaluation_status']='INVALID_OBSERVATION'
        elif obs['raw_tag_id']!=str(d['tag_id']):r['evaluation_status']='UNKNOWN_TAG'
        elif obs['anchor_id'] not in d['anchors_m']:r['evaluation_status']='UNKNOWN_ANCHOR'
        elif obs['anchor_id'] not in d['static_bias_m']:r['evaluation_status']='MISSING_STATIC_BIAS'
        else:
            p,q,left,right,state=associate(t,gt,times,ASSOCIATION['max_bracket_gap_s'])
            r.update(evaluation_status=state,gt_left_timestamp=left,gt_right_timestamp=right,
                     gt_bracket_gap_s=right-left if left is not None else None)
            if state=='AVAILABLE':
                T_GM=np.eye(4);T_GM[:3,:3]=rotation(q);T_GM[:3,3]=p
                tag=(T_AG@T_GM@T_MI@np.r_[lever,1])[:3]
                distance=float(np.linalg.norm(tag-np.asarray(d['anchors_m'][obs['anchor_id']])))
                beta=number(d['static_bias_m'][obs['anchor_id']])
                r.update(gt_range=distance,static_range_bias=beta,raw_range_error=z-beta-distance,
                         corrected_range_error=r['corrected_range']-beta-distance if r['corrected_range'] is not None else None)
        result.append(r)
    return result


def summarize(rows):
    output=[]
    for group_by in ['anchor','segment']:
        groups={}
        for r in rows:
            key=(r['run_id'],r['method'],r['anchor_id'],r['segment_id'] or 'NO_SEGMENT') if group_by=='segment' else (r['run_id'],r['method'],r['anchor_id'],'ALL')
            groups.setdefault(key,[]).append(r)
        for key,items in sorted(groups.items()):
            for metric in ['raw_range_error','corrected_range_error']:
                values=np.array([r[metric] for r in items if r[metric] is not None])
                output.append(dict(run_id=key[0],method=key[1],group_by=group_by,anchor_id=key[2],segment_id=key[3],
                    reference_status=items[0].get('reference_status','SOURCE_DECLARED'),assumed_fields=items[0].get('assumed_fields',''),
                    error_type=metric,total_observations=len(items),sample_count=len(values),unavailable_count=len(items)-len(values),
                    RMSE=float(np.sqrt(np.mean(values**2))) if len(values) else None,
                    MAE=float(np.mean(np.abs(values))) if len(values) else None,
                    median=float(np.median(values)) if len(values) else None,
                    P95=float(np.percentile(np.abs(values),95)) if len(values) else None,
                    status='AVAILABLE' if len(values) else 'UNAVAILABLE'))
    return output


def write_csv(path,rows,fields=None):
    with Path(path).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields or list(rows[0]));w.writeheader();w.writerows(rows)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--batch-dir',required=True,type=Path)
    p.add_argument('--calibration',required=True,type=Path)
    p.add_argument('--allow-identity-assumptions',action='store_true',help='Explicit development-only identity extrinsics/common clock; never assumes bias')
    a=p.parse_args();batch=a.batch_dir.resolve()
    out=PRIVATE/'range'/('range-'+uuid.uuid4().hex);out.mkdir(parents=True,exist_ok=False)
    status={'schema':'isas_range_evaluator_v1','estimator_input':False,'estimator_launched':False,
            'association':ASSOCIATION,'output_dir':str(out),'batch_dir':str(batch)}
    files={};rows=[]
    def track(path):
        path=Path(path);files[str(path)]=sha(path);return path
    try:
        d,geometry,missing=calibration(track(a.calibration),a.allow_identity_assumptions)
        status.update(reference_status=d['reference_status'],assumed_fields=d['assumed_fields'])
        (out/'calibration_snapshot.json').write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')
        if geometry:track(d['gt_path'])
        for source in d.get('sources',{}).values():
            if source and source.get('path'):track(source['path'])
        runs=read_csv(track(batch/'runs.csv'))
        backend=json.loads(track(batch/'batch/batch_manifest.json').read_text())
        for run in runs:
            if run['method'] not in ['M0','M1','M3','M4']:continue
            if run['recording_id']!='ISAS-Walk1' or run['dataset_family']!='ISAS':raise ValueError('only ISAS Walk1 supported')
            cell=next((c for c in backend['cells'] if c.get('run_id')==run['run_id']),None)
            if cell is None:
                status.setdefault('unavailable_runs',[]).append({'run_id':run['run_id'],'reason':'NOT_RUN_NO_BACKEND_CELL'});continue
            directory=Path(cell['run_directory'])
            result=json.loads(track(directory/'run_status.json').read_text())
            if result.get('status') in ('RUNNING','PREREGISTERED'):raise ValueError('estimator artifacts not terminal')
            valid=run['status'] in ('success','fallback') and result.get('exit_code')==0
            observations=read_csv(track(directory/'observations.csv'))
            masks={};comps={};segments={}
            for name,key,target in [('final_masks.csv','obs_id',masks),('fixed_compensations.csv','segment_id',comps)]:
                q=directory/name
                if q.exists():target.update(unique(read_csv(track(q)),key))
            if cell.get('parent_cache_manifest'):
                cache=Path(cell['parent_cache_manifest']);manifest=json.loads(track(cache).read_text())
                for payload in manifest['payloads']:
                    if payload['name']=='segments.csv':
                        q=track(cache.parent/payload['name'])
                        if sha(q)!=payload['sha256'].split(':')[-1]:raise ValueError('Stage2 segment hash mismatch')
                        segments=unique(read_csv(q),'segment_id')
            rows+=evaluate_rows(observations,run['method'],run['run_id'],d,geometry,missing,masks,comps,segments,valid)
        if any(sha(p)!=v for p,v in files.items()):raise ValueError('source changed during evaluation')
        write_csv(out/'range_metrics.csv',rows,FIELDS)
        summaries=summarize(rows)
        if summaries:write_csv(out/'range_summary.csv',summaries)
        status.update(status='UNAVAILABLE_CALIBRATION' if missing else ('EVALUATED_ASSUMPTIONS' if d['assumed_fields'] else 'EVALUATED'),missing_calibration=missing,
                      observation_count=len(rows),evaluated_count=sum(r['gt_range'] is not None for r in rows),sources_sha256=files)
        code=2 if missing or not any(r['gt_range'] is not None for r in rows) else 0
        if not missing and code==2:status['status']='UNAVAILABLE_NO_EVALUABLE_OBSERVATIONS'
    except Exception as exc:
        status.update(status='FAILED',reason=f'{type(exc).__name__}: {exc}',sources_sha256=files);code=1
    (out/'evaluation_status.json').write_text(json.dumps(status,indent=2,sort_keys=True,allow_nan=False)+'\n')
    print(json.dumps(status,allow_nan=False))
    return code


if __name__=='__main__':raise SystemExit(main())
