#!/usr/bin/env python3
"""Experiment-only closed-step input transform and deterministic selection. No estimator."""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import shutil
import yaml

VERSION = 'NLOS_INJECTION_V1'
SCHEMA = 'nlos_measurement_cache_v2'
METHODS = ('all_range','robust_cauchy','suppress_all','structured_debias','lcb_fixed_full','lcb_partial')

def sha(data):
    return 'sha256:' + hashlib.sha256(data).hexdigest()

def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()

def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')

def rows(path):
    with Path(path).open(newline='') as f: return list(csv.DictReader(f))

def csv_bytes(values, fields=None):
    out=io.StringIO(); w=csv.DictWriter(out, fieldnames=fields or list(values[0]),lineterminator='\n');w.writeheader();w.writerows(values)
    return out.getvalue().encode()

def v1_id(m):
    text='\n'.join(['t07_estimator_cache_v1',m['base_recording_id'],m['base_source_sha256'],
        format(float(m['recording_time_origin_s']),'.17g'),'imu_units=m/s^2,rad/s,quaternion_wxyz',
        'uwb_units=s,m,dBm','time_basis=sensor_time_from_recording_origin','grouping=source_message_index',
        m['imu_sha256'],m['uwb_sha256'],str(m['imu_count']),str(m['uwb_observation_count']),str(m['uwb_message_count']),''])
    return sha(text.encode())

def cache_id(m):
    old=v1_id(m)
    return sha((SCHEMA+'\n'+old+'\n'+m['transform_sha256']+'\n').encode())

def specification(enabled, tag, anchor, start, end, subset, amplitude_m=.5):
    if type(enabled) is not bool or not all(math.isfinite(v) for v in (start,end,amplitude_m)) or end<start or amplitude_m<0:
        raise ValueError('invalid closed step specification')
    return dict(version=VERSION,seed=911,enabled=enabled,shape='constant_step',interval='closed',
                tag_id=int(tag),anchor_id=int(anchor),start_s=float(start),end_s=float(end),amplitude_m=float(amplitude_m),
                anchor_subset=sorted(set(subset)))

def valid_reason(r, anchors, lo, hi):
    if r['source_valid']!='1': return bytes.fromhex(r['source_validity_reason_hex']).decode() or 'SOURCE_PROTOCOL_INVALID'
    if not all(math.isfinite(float(r[k])) for k in ('sensor_time_s','observed_range_m','fp_rssi_dbm','rx_rssi_dbm')): return 'NONFINITE'
    if int(r['anchor_id']) not in anchors: return 'UNKNOWN_ANCHOR'
    if not lo<=float(r['observed_range_m'])<=hi:return 'INVALID_RANGE'
    return 'VALID'

def generate(base, output, truth_dir, spec, lo=.3, hi=60.):
    base,output,truth_dir=map(Path,(base,output,truth_dir))
    if output.resolve()==truth_dir.resolve() or output.resolve() in truth_dir.resolve().parents or truth_dir.resolve() in output.resolve().parents:
        raise ValueError('truth and estimator directories must be disjoint')
    if output.exists() or truth_dir.exists():raise ValueError('output exists')
    required=set(specification(False,0,0,0,0,[]))
    if set(spec)!=required or spec['version']!=VERSION or spec['shape']!='constant_step' or spec['interval']!='closed' or spec['seed']!=911:
        raise ValueError('unsupported specification')
    if spec!=specification(spec['enabled'],spec['tag_id'],spec['anchor_id'],spec['start_s'],spec['end_s'],spec['anchor_subset'],spec['amplitude_m']):raise ValueError('noncanonical specification')
    m=yaml.safe_load((base/'input_manifest.yaml').read_text()); data=rows(base/'uwb_observations.csv')
    anchors={int(a['id']) for a in yaml.safe_load((base/'anchors.yaml').read_text())}
    if not set(spec['anchor_subset'])<=anchors:raise ValueError('unknown subset')
    if sha((base/'imu.csv').read_bytes())!=m['imu_sha256'] or sha((base/'uwb_observations.csv').read_bytes())!=m['uwb_sha256'] or v1_id(m)!=m['cache_id']:raise ValueError('base identity mismatch')
    audit=[];affected=[]
    for r in data:
        old=float(r['observed_range_m']);reason=valid_reason(r,anchors,lo,hi)
        targeted=int(r['tag_id'])==spec['tag_id'] and int(r['anchor_id'])==spec['anchor_id'] and spec['start_s']<=float(r['sensor_time_s'])<=spec['end_s']
        delta=spec['amplitude_m'] if spec['enabled'] and targeted and reason=='VALID' else 0.
        if delta and not lo<=old+delta<=hi:raise ValueError('INJECTED_RANGE_OUT_OF_VALID_BOUNDS')
        a=dict(r,base_range_m=r['observed_range_m'],injected_bias_delta_m=delta,
               targeted=int(targeted),validity_reason=reason,retained_anchor=int(int(r['anchor_id']) in spec['anchor_subset']))
        if delta:r['observed_range_m']=format(old+delta,'.17g');affected.append(r['obs_id'])
        a['observed_range_m']=r['observed_range_m'];audit.append(a)
    payload=csv_bytes(data) if spec['enabled'] else (base/'uwb_observations.csv').read_bytes()
    transform=dict(specification=spec,crop={'start_s':0.,'duration_s':-1.},base_source_sha256=m['base_source_sha256'])
    m.update(schema=SCHEMA,transform_sha256=sha(canonical(transform)),uwb_sha256=sha(payload));m['cache_id']=cache_id(m)
    output.mkdir(parents=True);truth_dir.mkdir(parents=True)
    shutil.copyfile(base/'imu.csv',output/'imu.csv');(output/'uwb_observations.csv').write_bytes(payload);write_json(output/'input_manifest.json',m)
    truth=dict(schema='nlos_injection_truth_v1',generator_version=VERSION,truth_semantics='INJECTED_COMPONENT_ONLY',total_latent_bias_status='UNKNOWN',
        base_recording_id=m['base_recording_id'],base_source_sha256=m['base_source_sha256'],cache_id=m['cache_id'],transform_sha256=m['transform_sha256'],
        specification=spec,specification_sha256=sha(canonical(spec)),crop=transform['crop'],crop_sha256=sha(canonical(transform['crop'])),
        subset_sha256=sha(canonical(spec['anchor_subset'])),affected_obs_ids=affected,planned_affected_obs_ids=None,planned_status='NOT_YET_PLANNED')
    write_json(truth_dir/'nlos_injection_truth.json',truth);(truth_dir/'nlos_injected_observations.csv').write_bytes(csv_bytes(audit));return m

def ordered_links(dataset,links):
    return sorted(links,key=lambda x:hashlib.sha256(f'{VERSION}|911|{dataset}|{x[0]}|{x[1]}'.encode()).hexdigest())

def persistent_faults(records,gap,n_min,t_min):
    bad=set();group={}
    for r in records:group.setdefault((int(r['tag_id']),int(r['anchor_id'])),[]).append(r)
    for link,rr in group.items():
        run=[]
        def finish():
            if len(run)>=n_min and float(run[-1]['sensor_time'])-float(run[0]['sensor_time'])>=t_min:bad.add(link)
        for r in sorted(rr,key=lambda x:(float(x['sensor_time']),int(x['obs_id']))):
            t=float(r['sensor_time'])
            if r['fault_detected']!='1' or (run and t-float(run[-1]['sensor_time'])>gap):finish();run=[]
            if r['fault_detected']=='1':run.append(r)
        finish()
    return bad

def windows(times,protected,gap,n_min,t_min,duration=8.,recording_end=None):
    candidates=[]
    recording_end=times[-1] if recording_end is None and times else recording_end
    for i,t in enumerate(times):
        if t<=protected:continue
        remaining=[t]
        for nxt in times[i+1:]:
            if nxt-remaining[-1]>gap:break
            remaining.append(nxt)
        # Endpoint coverage may use the original gap allowance, within the full recording.
        d=min(duration,remaining[-1]+gap-t,recording_end-t)
        inside=[x for x in remaining if x<=t+d]
        while inside and d>(len(inside)-1)*gap:
            d=(len(inside)-1)*gap
            inside=[x for x in remaining if x<=t+d]
        if len(inside)>=max(n_min,math.ceil(d/gap)+1) and inside[-1]-t>=t_min and t+d-inside[-1]<=gap:
            candidates.append((t,t+d))
    return candidates

def select_window(dataset, records, raw_rows, gap=1.,n_min=2,t_min=.01):
    planned=[r for r in records if r['valid']=='1' and r['planned']=='1']
    if not planned or any(r['tested']!='1' for r in planned):return None,{'status':'FDE_INCOMPLETE'}
    keyframes=sorted({int(r['keyframe_id']) for r in planned})
    if len(keyframes)<5:return None,{'status':'INSUFFICIENT_INITIALIZATION_FRAMES'}
    protected=max(float(r['sensor_time']) for r in planned if int(r['keyframe_id'])<=keyframes[4])
    bad=persistent_faults(planned,gap,n_min,t_min);links=ordered_links(dataset,{(int(r['tag_id']),int(r['anchor_id'])) for r in planned});audit=[];all_candidates=[]
    for link in links:
        times=sorted(float(r['sensor_time']) for r in planned if (int(r['tag_id']),int(r['anchor_id']))==link)
        ww=[] if link in bad else windows(times,protected,gap,n_min,t_min,recording_end=max(float(r["sensor_time"]) for r in planned))
        legal=[]
        for start,end in ww:
            overflow=any(int(r['tag_id'])==link[0] and int(r['anchor_id'])==link[1] and start<=float(r['sensor_time_s'])<=end and r['source_valid']=='1' and .3<=float(r['observed_range_m'])<=60 and float(r['observed_range_m'])+.5>60 for r in raw_rows)
            if not overflow:legal.append((start,end))
        audit.append(dict(link=link,persistent_fault=link in bad,windows=legal,excluded_reason='PERSISTENT_FAULT' if link in bad else ('NO_QUALIFIED_WINDOW' if not legal else None)))
        all_candidates.extend((link,s,e) for s,e in legal)
    exact=[x for x in all_candidates if x[2]-x[1]==8.]
    if exact:chosen=exact[0];reason='EXACT_8_SECONDS'
    elif all_candidates:
        longest=max(e-s for _,s,e in all_candidates);chosen=next(x for x in all_candidates if x[2]-x[1]==longest);reason='NO_ELIGIBLE_8_SECOND_WINDOW_GLOBAL_LONGEST'
    else:chosen=None;reason='NO_QUALIFIED_WINDOW'
    return chosen,dict(status=reason,protected_through_sensor_time=protected,ranked_links=audit)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--base',required=True);p.add_argument('--output',required=True);p.add_argument('--truth-output',required=True);p.add_argument('--specification',required=True);a=p.parse_args()
    generate(a.base,a.output,a.truth_output,json.loads(Path(a.specification).read_text()))
