#!/usr/bin/env python3
"""Observation-ID treatment and measurement-only SFUISE input adapter."""
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rosbag
import yaml
from formal_native import HERE,ROOT,WS,sha,digest,read,rows,save,write,verify
from estimator_isolation import sandbox_command

BUILD=WS/'evaluator_private/icra/sfuise_baseline/build_ws'
SF_BINS=[BUILD/'devel/lib/sfuise/EstimationInterface',BUILD/'devel/lib/sfuise/SplineFusion',
         BUILD/'devel/lib/sfuise_baseline_adapter/sfuise_trajectory_adapter']
TOPICS=['/waveshare_sense_hat_b','/rtls_flares','/anchor_list']


def serialize(msg):
    s=io.BytesIO();msg.serialize(s);return s.getvalue()


def export(case_key,manifest,native):
    source=rows(manifest.parent/read(manifest)['uwb_file'])
    producer=Path(native['AUX_PRODUCER']['run']) if native['AUX_PRODUCER'].get('run') else None
    status=read(producer/'production_detector_status.json') if producer and (producer/'production_detector_status.json').exists() else {}
    support=read(producer/'production_support_partition.json')['segments'] if status.get('valid') else None
    candidates={str(oid):s for s in (support or []) for oid in s['obs_ids']}
    assert len(candidates)==sum(len(s['obs_ids']) for s in (support or []))
    byid={r['obs_id']:r for r in source};assert set(candidates)<=set(byid)
    final=native.get('FGO_RECOVER_FULL',{});directory=Path(final['run']) if final.get('run') else None
    state=final.get('backend',{});valid_final=state.get('valid_estimate_exported',False)
    fixed=rows(directory/'fixed_compensations.csv') if directory and (directory/'fixed_compensations.csv').exists() else []
    comp={r['segment_id']:r for r in fixed};result=[]
    for r in source:
        seg=candidates.get(r['obs_id']);c=comp.get(seg['segment_id'],{}) if seg else {}
        accepted=c.get('final_use')=='1' and valid_final
        delta=float(c['delta_c_fixed_m']) if accepted else 0.
        sigma=float(c['sigma_c_local_m']) if c.get('sigma_available')=='1' else None
        estimate=float(c['c_hat_stage2_m']) if c else None
        if sigma is not None and not np.isfinite(sigma):sigma=None
        if estimate is not None and not np.isfinite(estimate):estimate=None
        if accepted:assert np.isfinite(delta) and delta>0 and estimate is not None
        result.append({'timestamp':r['sensor_time_s'],'obs_id':r['obs_id'],'anchor_id':r['anchor_id'],
                       'raw_range':float(r['observed_range_m']),'candidate':int(bool(seg)),
                       'segment_id':seg['segment_id'] if seg else '', 'estimated_bias':estimate,'sigma_bias':sigma,
                       'lcb':max(0.,estimate-2*sigma) if estimate is not None and sigma is not None else None,
                       'recovery_accepted':int(accepted),'applied_correction':delta,
                       'corrected_range':float(r['observed_range_m'])-delta,
                       'source_valid':r['source_valid'],
                       'treatment_status':'available' if support is not None and (not candidates or valid_final) else 'unavailable'})
    save(HERE/'exports/corrected_ranges'/f'{case_key}.csv',result)
    save(HERE/'exports/rejected_ranges'/f'{case_key}.csv',[dict(r,reject=int(r['obs_id'] in candidates)) for r in source])
    return {'rows':result,'support_valid':support is not None,'recover_available':support is not None and (not candidates or valid_final),
            'candidate_count':len(candidates),'segment_count':len(support or []),'segments':support or [],
            'fixed':fixed,'fallback':bool(state.get('status','').startswith('FALLBACK')),
            'producer':str(producer) if producer else None}


def make_bag(case_key,sequence,manifest,treatment,method):
    """Keep original serialized IMU/header/untreated fields; only delete/replace selected ranges."""
    out=HERE/'exports/sfuse_inputs'/case_key;out.mkdir(parents=True,exist_ok=True)
    dest=out/(method+'.bag')
    if dest.exists():raise ValueError('input already materialized')
    if method=='SF_REJECT' and not treatment['support_valid']:return None
    if method=='SF_RECOVER' and not treatment['recover_available']:return None
    source=rows(manifest.parent/read(manifest)['uwb_file'])
    bypos={(int(r['source_message_index']),int(r['source_range_index'])):r for r in source}
    tr={r['obs_id']:r for r in treatment['rows']}
    origin=ROOT/f'data/SFUISE/ISAS-{sequence}.bag'
    expected_parent=read(manifest)['base_source_sha256'].split(':')[-1]
    assert sha(origin)==expected_parent
    original_index=0;seen=set();proof=[];imu_hash=hashlib.sha256();out_imu=hashlib.sha256();max_round=0.
    with rosbag.Bag(str(origin)) as bag,rosbag.Bag(str(dest),'w') as output:
        for topic,msg,time_stamp in bag.read_messages(topics=TOPICS):
            before=serialize(msg)
            if topic=='/rtls_flares':
                retained=[]
                for ri,rg in enumerate(msg.ranges):
                    row=bypos.get((original_index,ri))
                    if row is None:raise ValueError('raw bag/cache domain differs')
                    oid=row['obs_id'];seen.add(oid);t=tr[oid]
                    assert msg.header.stamp.to_sec()==float(row['sensor_time_s'])
                    assert int(rg.id)==int(row['anchor_id'])
                    if method=='SF_REJECT' and t['candidate']:
                        proof.append({'obs_id':oid,'action':'removed','source_message_index':original_index,'source_range_index':ri});continue
                    expected=float(row['observed_range_m'])
                    if method=='SF_RECOVER':expected=t['corrected_range']
                    rg.range=expected
                    # Native cache is binary64; ROS range is float32. Its sole representational roundoff is explicit.
                    round_error=abs(float(np.float32(expected))-expected)
                    max_round=max(max_round,round_error)
                    if not np.isfinite(expected) or round_error>max(1e-6,abs(expected)*1e-7):raise ValueError('invalid range conversion')
                    retained.append(rg)
                    proof.append({'obs_id':oid,'action':'retained','source_message_index':original_index,'source_range_index':ri,
                                  'expected_range':expected,'serialized_range':float(np.float32(expected))})
                msg.ranges=retained;original_index+=1
            if topic=='/waveshare_sense_hat_b':
                imu_hash.update(before);out_imu.update(serialize(msg));assert before==serialize(msg)
            output.write(topic,msg,time_stamp)
    assert seen=={r['obs_id'] for r in source};assert imu_hash.hexdigest()==out_imu.hexdigest()
    # Independently deserialize output and compare every preserved field to the original.
    with rosbag.Bag(str(origin)) as original,rosbag.Bag(str(dest)) as derived:
        left=list(original.read_messages(topics=TOPICS));right=list(derived.read_messages())
    assert len(left)==len(right)
    mi=0
    for (topic,a,ta),(topic2,b,tb) in zip(left,right):
        assert topic==topic2 and ta==tb
        if topic!='/rtls_flares':assert serialize(a)==serialize(b);continue
        old=copy.deepcopy(a);old.ranges=[];new=copy.deepcopy(b);new.ranges=[]
        assert serialize(old)==serialize(new),'nonrange message fields changed'
        expected=[]
        for ri,rg in enumerate(a.ranges):
            row=bypos[(mi,ri)];t=tr[row['obs_id']]
            if method=='SF_REJECT' and t['candidate']:continue
            rg.range=t['corrected_range'] if method=='SF_RECOVER' else float(row['observed_range_m'])
            expected.append(rg)
        assert len(expected)==len(b.ranges)
        for x,y in zip(expected,b.ranges):assert serialize(x)==serialize(y),'range roundtrip mismatch'
        mi+=1
    metadata={'method_id':method,'bag':str(dest),'bag_hash':sha(dest),'imu_payload_hash':imu_hash.hexdigest(),
              'integrity':'PASS','gt_topics':[],'truth_fields':[],'max_float32_rounding_m':max_round,
              'retained_obs_count':sum(r['action']=='retained' for r in proof),'removed_obs_count':sum(r['action']=='removed' for r in proof),
              'source_domain_count':len(seen)}
    save(out/(method+'_identity.csv'),proof);write(out/(method+'_input.json'),metadata)
    return dest


def libraries(binary):
    result=[]
    for line in subprocess.check_output(['ldd',str(binary)],text=True).splitlines():
        if '=> /' in line:
            p=Path(line.split('=>',1)[1].strip().split()[0])
            if not str(p).startswith(('/usr/','/lib/','/lib64/')):result.append(p)
    return result


def run_sf(case_key,sequence,bag,method,stage,ready=False):
    lock=verify();out=HERE/'runs'/stage/case_key/method
    if out.exists():raise ValueError('SF run already exists')
    out.mkdir(parents=True)
    config=HERE/f'configs/sfuse/config_test_isas-walk{sequence[-1]}.yaml'
    identity={'git_hash':lock['git_hash'],'config_hash':sha(config),'experiment_fingerprint':lock['experiment_fingerprint'],
              'input_hash':sha(bag) if bag else None,'method_id':method,'case_key':case_key,'sequence':sequence}
    if bag is None:
        result={'status':'failed','failure_reason':'OFFLINE_INFERENCE_TREATMENT_UNAVAILABLE','runtime':None,'run':str(out),'identity':identity}
        write(out/'run_status.json',result);return result
    worker=HERE/'formal_sf_worker.py';files=[bag,config,worker]+SF_BINS
    for binary in SF_BINS:files+=libraries(binary)
    cmd=['/usr/bin/python3',str(worker),str(out),str(BUILD),str(bag),str(config)]
    if ready:cmd.append('--ready')
    wrapped=sandbox_command(cmd,files,out);idx=wrapped.index('--')
    wrapped[idx:idx]=['--ro-bind','/opt/ros/noetic','/opt/ros/noetic','--ro-bind','/etc/hosts','/etc/hosts',
                      '--ro-bind','/etc/resolv.conf','/etc/resolv.conf','--ro-bind','/etc/alternatives','/etc/alternatives']
    proof={'argv':wrapped,'readonly':{str(p):sha(p) for p in files},'writable':str(out),'identity':identity,
           'gt_mounts':[],'truth_mounts':[],'support_mounts':[],'network':'UNSHARED'}
    write(out/'isolation.json',proof)
    with (out/'worker.log').open('w') as log:
        start=time.monotonic();p=subprocess.Popen(wrapped,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:code=p.wait(timeout=1800)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait();code=124
    status=read(out/'run_status.json') if (out/'run_status.json').exists() else {'status':'failed','failure_reason':'WORKER_EXIT_'+str(code)}
    result=dict(status,process_exit_code=code,runtime=time.monotonic()-start,run=str(out),identity=identity)
    if ready and result['status']=='completed':
        from formal_transport_probe import validate
        transport=validate(bag,out/'received.bag');write(out/'transport_integrity.json',transport)
        if transport['status']!='PASS':result.update(status='invalid_input',failure_reason='SENSOR_TRANSPORT_LOSS')
    write(out/'formal_result.json',result);verify();return result
