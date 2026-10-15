#!/usr/bin/env python3
"""Read-only SC8 scoring, plotting and report consumers; never launches solvers."""
import csv
import json
import math
from pathlib import Path
from collections import Counter
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import benchmark_common as common
import sc8_metrics as metric
from sc6_closure import value,spatial,current,partition,verified_rows

ROOT=common.ROOT;CACHE=ROOT/'.cache/sc8';OUT=ROOT/'docs/sc8'
COLORS={'O-FULL':'#0072B2','O-NOFDE':'#777777','O-FDE-ENG':'#0072B2','O-ENG-NOFDE':'#777777',
        'O-NOM':'#56B4E9','S-ONLINE':'#E69F00','S-FINAL':'#8B56A2'}


def clean(obj):
    if isinstance(obj,dict):return {str(k):clean(v) for k,v in obj.items()}
    if isinstance(obj,(list,tuple,np.ndarray)):return [clean(x) for x in obj]
    if isinstance(obj,np.generic):return clean(obj.item())
    if isinstance(obj,float) and not math.isfinite(obj):return None
    return obj


def save(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(clean(obj),ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')


def rows(path,keep_partial=False):
    if not path.exists():return []
    with path.open() as f:
        # A resource stop may leave a partial final line; preserve its count.
        return [r for r in csv.DictReader(f) if None not in r and (keep_partial or None not in r.values())]


def latest(entry):
    paths=[p for p in (CACHE/'runs').glob(entry['id']+'*/execution.json')
           if p.parent.name==entry['id'] or p.parent.name.startswith(entry['id']+'-attempt')]
    def number(p):return int(p.parent.name.split('-attempt')[-1]) if '-attempt' in p.parent.name else 1
    paths.sort(key=number)
    return paths[-1] if paths else None


def trajectory(path):
    data=np.loadtxt(path,ndmin=2)
    if data.shape[1]!=8 or len(data)<2:raise ValueError('insufficient TUM states')
    return data[:,0],data[:,1:4],data[:,4:8]


def native_episodes(data,manifest):
    good=[r for r in data if value(r,'updates')==1];episodes=[]
    for i,f in enumerate(manifest['faults']):
        onset,axis,duration,amplitude,_=f;begin=(onset-1)*.05+.005;end=begin+duration*.001
        following=manifest['faults'][i+1][0] if i+1<len(manifest['faults']) else math.inf
        rr=[r for r in good if onset<=value(r,'epoch')<following]
        alarm=[r for r in rr if value(r,'imu_alarm')==1 and value(r,'first_imu_alarm_ns')>=round(begin*1e9)]
        bridges=[r for r in rr if value(r,'bridge')==1]
        repairs=[r for r in bridges if value(r,'repair')==1 and value(r,'removed_priors')>0]
        rejoin=[r for r in rr if bridges and value(r,'rejoined')==1 and
                value(r,'epoch')>value(bridges[0],'epoch') and (value(r,'epoch')-1)*.05>=end-1e-9]
        unsafe=[r for r in rr if bridges and value(r,'rejoined')==1 and (value(r,'epoch')-1)*.05<end-1e-9]
        first=lambda x:int(value(x[0],'epoch')) if x else None
        episodes.append(dict(kind='IMU',axis=axis,onset_epoch=onset,begin_s=begin,end_s=end,amplitude=amplitude,
            first_fd=first(alarm),first_bridge=first(bridges),first_prior_repair=first(repairs),
            rejoin4=[int(value(r,'epoch')) for r in rejoin[:4]],unsafe_shared_endpoint=[int(value(r,'epoch')) for r in unsafe],
            TTD_sensor_ms=(value(alarm[0],'first_imu_alarm_ns')*1e-9-begin)*1000 if alarm else None,
            TTD_wall_ms=value(alarm[0],'imu_alarm_ms')-begin*1000 if alarm else None,
            TTI_wall_ms=value(bridges[0],'commit_time_ms')-begin*1000 if bridges else None,
            TTR_wall_ms=value(rejoin[0],'commit_time_ms')-begin*1000 if rejoin else None,
            FD=bool(alarm),ISOLATION_COMMITTED=bool(bridges and repairs),
            REJOIN_EXECUTED=bool(repairs and len(rejoin)>=4 and not unsafe),
            ACCURACY_RECOVERED='NOT_IMPLIED_BY_RECEIPT; see paired phase metrics'))
    for i,(onset,anchor,duration,amplitude) in enumerate(manifest.get('uwb_faults',[])):
        rr=[r for r in good if onset<=value(r,'epoch')<onset+duration]
        alarm=[r for r in rr if value(r,'uwb_alarm')==1]
        isolation=[r for r in rr if value(r,'excluded')==anchor]
        after=[r for r in good if value(r,'epoch')>=onset+duration and value(r,'excluded')==-1]
        episodes.append(dict(kind='UWB',anchor_index=anchor,onset_epoch=onset,begin_s=onset*.05,
            end_s=(onset+duration)*.05,amplitude=amplitude,FD=bool(alarm),
            ISOLATION_COMMITTED=bool(isolation),first_fd=int(value(alarm[0],'epoch')) if alarm else None,
            first_isolation=int(value(isolation[0],'epoch')) if isolation else None,
            first_nominal_receipt_after_exclusion=int(value(after[0],'epoch')) if isolation and after else None,
            TTD_wall_ms=value(alarm[0],'uwb_alarm_ms')-onset*50 if alarm else None,
            TTI_wall_ms=value(isolation[0],'commit_time_ms')-onset*50 if isolation else None))
    return episodes


def integrity(data,planned,pace):
    good=[r for r in data if spatial(r)]
    result=dict(planned=planned,received=len(data),attempted=sum(r.get('status')!='SOURCE_GAP_OR_ABORT' for r in data),
        committed=sum(value(r,'updates')==1 for r in data),spatial_valid=len(good),
        timely_spatial=sum(current(r) for r in data) if pace=='paced' else None,
        publication_protected=0,application_qualification='UNQUALIFIED',
        late_spatial=sum(spatial(r) and not current(r) for r in data) if pace=='paced' else None,
        missing=max(0,planned-len(data)),refused=sum(not spatial(r) for r in data),
        status_counts=dict(Counter(r.get('status') for r in data)),axes={})
    for axis,pe,pl in [('H','hpe','hpl'),('V','vpe','vpl')]:
        d=dict(pl=metric.summary([value(r,pl) for r in good]),
               underbound=sum(value(r,pe)>value(r,pl) for r in good),
               si=metric.summary([value(r,pe)/value(r,pl) for r in good if value(r,pl)>0]),
               margin=metric.summary([value(r,pl)-value(r,pe) for r in good]),AL={})
        for al in [2.,3.,5.,10.]:
            counts=Counter(partition(value(r,pe),value(r,pl),al) for r in good)
            counts.update({k:0 for k in ['nominal_admissible','MI_candidate','HMI_candidate','AL_unavailable'] if k not in counts})
            assert sum(counts.values())==len(good)
            d['AL'][str(al)]=dict(counts,within_AL=sum(value(r,pl)<=al for r in good),
                timely_within_AL=sum(current(r) and value(r,pl)<=al for r in data) if pace=='paced' else None)
        result['axes'][axis]=d
    result['joint_AL']={f'{h}/{v}':dict(
        within_AL=sum(value(r,'hpl')<=h and value(r,'vpl')<=v for r in good),
        timely_within_AL=sum(current(r) and value(r,'hpl')<=h and value(r,'vpl')<=v for r in data) if pace=='paced' else None)
        for h,v in [(2,3),(5,5),(10,10)]}
    # Continuity is a finite-run diagnostic, never a calibrated risk claim.
    if pace=='paced':
        epochs={int(value(r,'epoch')):current(r) for r in data};gaps=[];length=0;was_available=False
        for k in range(1,planned+1):
            available=epochs.get(k,False)
            if not available and was_available:length+=1
            elif available and length:gaps.append(length);length=0
            if available:was_available=True
        if length:gaps.append(length)
        result['continuity']=dict(interruptions=len(gaps),max_gap_epochs=max(gaps,default=0))
    return result


def score_entry(entry,execution,run,registry):
    result=dict(id=entry['id'],group=entry['group'],stream=entry['stream'],track=entry['track'],seed=entry['seed'],
        window=entry['window'],pace=entry['pace'],status=execution['status'],requested_method=entry['requested_method'],
        effective_method=execution.get('effective_method'),identity=execution['identity'],
        input_digest=execution['input_digest'],output_hashes=execution.get('output_hashes',{}),
        wall_s=execution.get('elapsed_s'),views={})
    inputs=ROOT/execution['inputs']
    for name,expected in execution.get('input_files',{}).items():
        if common.sha256(inputs/name)!=expected:raise ValueError('INPUT_IDENTITY_MISMATCH '+name)
    for name,expected in execution.get('output_hashes',{}).items():
        if common.sha256(run/name)!=expected:raise ValueError('OUTPUT_IDENTITY_MISMATCH '+name)
    doc=json.loads((inputs/'manifest.json').read_text())
    if 'overlay' in doc:result['overlay']=doc['overlay']
    range0=next(r for r in common.read_csv(inputs/'uwb.csv') if int(r['tag_id'])==int(doc['primary_tag']))
    lever=[float(range0[k]) for k in ['lever_x','lever_y','lever_z']]
    gt=common.load_gt(inputs/'gt.csv',doc['primary_tag']);truth=(gt.time,gt.position,gt.quaternion)
    test_begin=-math.inf
    if entry['stream'].split('-')[0]=='R3' and entry['group']!='E00':
        test_begin=json.loads((CACHE/'calibration/own_prefix20.json').read_text())['test_begin_s']
    result['scoring_begin_s']=test_begin if math.isfinite(test_begin) else None
    result['calibration_scope']=doc.get('sc8_calibration_scope',doc.get('provenance',{}).get('source'))
    native_all=rows(run/'trace.csv',keep_partial=True)
    native=[r for r in native_all if None not in r.values()]
    if entry['track']=='T-SIM' and entry['requested_method'] not in ['SFUISE','O-NOM']:
        verified_rows(native,lever)
        manifest=registry['cases'][entry['stream']];planned=entry.get('epochs',manifest['epochs'])
        result['integrity']=integrity(native,planned,entry['pace'])
        actual=[r for r in native if value(r,'updates')==1]
        selected=[r for r in actual if entry['group']!='E05' or 1101<=value(r,'epoch')<=2100]
        result['planned_epochs']=planned;result['common_scoring_epochs']=[1101,2100] if entry['group']=='E05' else [1,planned]
        if len(selected)>=2:
            t=np.array([value(r,'stamp_ns')*1e-9 for r in selected])
            p=np.array([[value(r,'tag_'+k) for k in 'xyz'] for r in selected])
            q=np.array([[value(r,'q'+k) for k in 'xyzw'] for r in selected])
            score,arrays=metric.evaluate((t,p,q),truth,lever)
            for key in ['velocity_error','accel_bias_error','gyro_bias_error']:
                score[key]=metric.summary([value(r,key) for r in selected])
            result['views'][entry['requested_method']]=score
        measurable=[r for r in native_all if r.get('status')!='SOURCE_GAP_OR_ABORT' and math.isfinite(value(r,'core_ms'))]
        result['partial_rows_with_measured_core']=len(native_all)-len(native)
        result['resource_observation']=execution.get('resource_observation')
        result['timing']={k:metric.summary([value(r,k) for r in measurable]) for k in ['core_ms','source_ms','prepare_ms','candidate_ms','proof_ms','commit_ms','cleanup_ms','envelope_cleanup_ms']}
        result['timing']['measurement_to_sink_ms']=metric.summary([value(r,'delivery_ms') for r in measurable]) if entry['pace']=='paced' else {'n':0,'reason':'OFFLINE_NO_SOURCE_LATENCY'}
        result['timing']['state_age_ms']=result['timing']['measurement_to_sink_ms']
        result['timing']['max_sensor_lookahead_ms']=0
        fine=['source_ms','imu_fd_ms','graph_prepare_ms','candidate_ms','proof_ms','commit_ms','state_receipt_ms','finalize_ms','maintenance_ms','envelope_cleanup_ms']
        exclusive=fine if actual and all(all(math.isfinite(value(r,k)) for k in fine) for r in actual) else ['source_ms','prepare_ms','candidate_ms','proof_ms','commit_ms','cleanup_ms','envelope_cleanup_ms']
        result['exclusive_means_ms']={k:float(np.sum([value(r,k) for r in actual]))/len(measurable) for k in exclusive} if measurable and actual else {}
        if measurable:
            known_sum=sum(result['exclusive_means_ms'].values())
            result['exclusive_mean_core_ms']=float(np.mean([value(r,'core_ms') for r in measurable]))
            result['exclusive_means_ms']['abort_guard_and_clock_overhead']=result['exclusive_mean_core_ms']-known_sum
            assert abs(sum(result['exclusive_means_ms'].values())-result['exclusive_mean_core_ms'])<1e-8
            result['exclusive_denominator']=len(measurable)
            result['exclusive_abort_cost_scope']='Whole measurable abort cost assigned to unallocated bucket; not zeroed or discarded'
        runtime_path=run/'trace.csv.runtime.json'
        if runtime_path.exists():
            runtime=json.loads(runtime_path.read_text());result['runtime']=runtime
            duration=runtime['sensor_duration_s'];result['processing_RTF']=runtime['wall_processing_ms']/1000/duration if duration else None
            result['root_cpu_s']=runtime['cpu_ms']/1000
            result['peak_rss_kib']=runtime['peak_rss_kib']
        else:result['root_cpu_s']=execution.get('children_cpu_s')
        if entry['pace']=='paced':
            core95=result['timing']['core_ms'].get('p95');sink95=result['timing']['measurement_to_sink_ms'].get('p95')
            result['timely_spatial_fraction']=result['integrity']['timely_spatial']/planned
            result['tested_profile_deadline_pass']=bool(core95 is not None and sink95 is not None and core95<=40 and sink95<=50 and result['timely_spatial_fraction']>=.95)
        else:result['tested_profile_deadline_pass']='NOT_APPLIED_OFFLINE_SCALING_FRONTIER'
        result['resources']={k:dict(first=value(actual[0],k),last=value(actual[-1],k),max=max(value(r,k) for r in actual)) for k in ['rss','slots','live','ledger','replay','capsules','active_states','proof_explicit','source_prefix','q','alerts'] } if actual else {}
        result['same_center_M']={axis:dict(strict=sum(value(r,old)-value(r,new)>1e-9 for r in actual),
             widened=sum(value(r,old)<value(r,new) for r in actual),
             reduction_m=metric.summary([value(r,old)-value(r,new) for r in actual]))
             for axis,new,old in [('H','hpl','m_old_hpl'),('V','vpl','m_old_vpl')]}
        result['actual_max_epoch']=max([int(value(r,'epoch')) for r in native],default=0)
        result['x0_retired']=any(value(r,'x0')==0 for r in actual)
        result['episodes']=native_episodes(native,manifest)
        result['semantics']='ACTUAL_NATIVE_COMMITTED_TAG; raw conditional simulation only'
        result['phase_errors']={}
        for name,rr in [('healthy_prefix',[r for r in actual if value(r,'epoch')<121]),
                        ('bridge',[r for r in actual if value(r,'bridge')==1]),
                        ('rejoin',[r for r in actual if value(r,'rejoined')==1]),
                        ('late_last_quarter',[r for r in actual if value(r,'epoch')>.75*planned])]:
            result['phase_errors'][name]={k:metric.summary([value(r,k) for r in rr]) for k in ['ape','hpe','vpe','rotation_error','velocity_error','accel_bias_error','gyro_bias_error']}
    else:
        files={'S-ONLINE':'trajectory.tum.online.tum','S-FINAL':'trajectory.tum.history_final_calibration.tum'} if entry['requested_method']=='SFUISE' else {execution.get('effective_method') or 'UNKNOWN_INTERRUPTED_INTERFACE':'trajectory.tum'}
        for view,name in files.items():
            try:
                estimate=trajectory(run/name);keep=estimate[0]>=test_begin
                estimate=tuple(a[keep] for a in estimate)
                score,arrays=metric.evaluate(estimate,truth,lever)
                score['bias_error']='NA_NO_INDEPENDENT_REAL_GT' if entry['track']!='T-SIM' else 'SEE_RAW_SIM_AUDIT'
                result['views'][view]=score
            except (ValueError,OSError,KeyError) as e:result['views'][view]={'status':'NA','reason':str(e)}
        attempts=rows(run/'attempts.csv')
        if attempts:
            # Keep initialization and unattempted suffix in the full source denominator.
            source_rows=[r for r in common.read_csv(inputs/'uwb.csv') if int(r['tag_id'])==int(doc['primary_tag']) and int(r['valid'])!=0]
            batch_window=doc.get('epoch_batch_window_s',0.);planned_batches=0;i=0
            while i<len(source_rows):
                j=i+1
                while j<len(source_rows) and (source_rows[j]['source_message']==source_rows[i]['source_message'] or float(source_rows[j]['t'])-float(source_rows[i]['t'])<=batch_window+1e-12):j+=1
                planned_batches+=1;i=j
            result['planned_epochs']=planned_batches
            result['attempted_epochs']=len(attempts)
            result['startup_or_unattempted_epochs']=max(0,planned_batches-len(attempts))
            result['committed']=sum(value(r,'committed')==1 for r in attempts)
            result['timely_estimates']=sum(value(r,'committed')==1 and 0<=value(r,'measurement_to_sink_ms')<=50 for r in attempts)
            result['fde_status_counts']=dict(Counter(r['fde_status'] for r in attempts))
            result['timing']={k:metric.summary([value(r,k) for r in attempts]) for k in ['core_ms','measurement_to_sink_ms','state_age_ms']}
            result['timely_estimate_fraction']=result['timely_estimates']/planned_batches if planned_batches else None
            result['root_cpu_s']=execution.get('details',{}).get('cpu_ms',float('nan'))/1000
            result['peak_rss_kib']=execution.get('details',{}).get('peak_rss_kib')
            result['unattempted_after_resource_stop']=max(0,planned_batches-len(attempts)) if execution['status']=='RESOURCE_LIMIT' else 0
        observer=rows(run/'observer.csv');cutoff=rows(run/'trajectory.tum.online_cutoff.csv')
        if observer:
            result['timing']={k:metric.summary([value(r,k) for r in observer]) for k in ['core_ms','cpu_ms']}
            result['timing']['state_age_ms']=metric.summary([(value(r,'source_clock_at_sink_ns')-value(r,'state_ns'))*1e-6 for r in cutoff])
            result['timing']['lookahead_upper_ms']=metric.summary([(value(r,'max_sensor_stamp_used')-value(r,'state_ns'))*1e-6 for r in observer])
            indexed={r['state_ns']:r for r in observer}
            if 'observed_sensor_cutoff_ns' in observer[0]:
                delivery=[(value(r,'source_clock_at_sink_ns')-value(indexed[r['state_ns']],'max_sensor_stamp_used'))*1e-6 for r in cutoff if r['state_ns'] in indexed]
                result['timing']['measurement_to_sink_ms']=metric.summary(delivery)
                result['timing']['input_cutoff_semantics']='MAX_NUMERIC_INPUT: init gravity, propagation, windows and inherited past; separate callback cutoff'
                result['root_cpu_s']=value(observer[-1],'process_cpu_ms')/1000
                result['peak_rss_kib']=value(observer[-1],'peak_rss_kib')
            else:
                result['timing']['measurement_to_sink_ms']='NA_OLD_W0_CALLBACK_CUTOFF_ONLY'
                result['timing']['input_cutoff_semantics']='OLD_W0_CONSERVATIVE_MAX_CALLBACK'
            result['source_clock_resolution_ms']=1.
            result['output_rate_hz']=len(cutoff)/max(1e-9,float(gt.time[-1]-gt.time[0]))
            result['observed_cutoff_vs_numeric_cutoff_separated']='observed_sensor_cutoff_ns' in observer[0]
        result['integrity']=dict(statistical_qualification='UNQUALIFIED_REAL_SOURCE' if entry['track']!='T-SIM' else 'NO_RAW_PL_METHOD',HPL=None,VPL=None,publication_protected=0)
    return result


def evaluate():
    registry=yaml.safe_load((ROOT/'config/sc8_experiments.yaml').read_text());records=[];attempts=[]
    for entry in registry['runs']:
        path=latest(entry)
        if path is None:
            records.append(dict(id=entry['id'],group=entry['group'],stream=entry['stream'],requested_method=entry['requested_method'],status='NOT_ATTEMPTED'));continue
        execution=json.loads(path.read_text())
        if execution['status']=='RUNNING':
            records.append(dict(id=entry['id'],group=entry['group'],stream=entry['stream'],requested_method=entry['requested_method'],status='RUNNING'));continue
        try:record=score_entry(entry,execution,path.parent,registry)
        except Exception as error:
            record=dict(id=entry['id'],group=entry['group'],stream=entry['stream'],status=execution['status'],requested_method=entry['requested_method'],scoring_status='ERROR',scoring_error=str(error))
        records.append(record)
    for path in (CACHE/'runs').glob('*/execution.json'):
        d=json.loads(path.read_text());attempts.append(dict(id=path.parent.name,status=d['status'],source_identity=d['identity'],reason=d.get('reason'),returncode=d.get('returncode')))
    counts=dict(Counter(r['status'] for r in records));assert sum(counts.values())==len(registry['runs'])
    metrics=dict(schema='sc8-metrics-v1',status='IN_PROGRESS',registry_sha256=common.sha256(ROOT/'config/sc8_experiments.yaml'),
        registered=len(registry['runs']),logical_run_status_counts=counts,attempt_processes=len(attempts),
        runs=records,attempts=attempts,statistical_unit='SEED_OR_RECORDING_NOT_FRAMES')
    save(OUT/'SC8_METRICS.json',metrics)
    print('SCORED',counts,'attempts',len(attempts),'scoring errors',sum('scoring_error' in r for r in records),flush=True)
    for r in records:
        if 'scoring_error' in r:print(r['id'],r['scoring_error'],flush=True)
    return metrics


def verify_native_input(audit,inputs):
    """Independent post-run parity of the ACTUAL native ADC/range audit.

    Metadata/YAML and standardized draws are not physical observations. Compare
    only the consumed physical values/times/IDs against the shared source cache;
    shorter directed fixtures compare exactly their immutable source prefix.
    """
    if not audit.exists():return {'status':'NOT_EXPORTED_W0_ONLY'}
    def ns(t):
        v=float(t)*1e9
        return int(math.floor(v+.5) if v>=0 else math.ceil(v-.5))
    imu=iter(common.read_csv(inputs/'imu.csv'));uwb=iter(common.read_csv(inputs/'uwb.csv'))
    ni=nu=0;observed=None
    with audit.open() as f:
        for line in f:
            a=line.split()
            if not a:continue
            if a[0]=='initial':observed=a
            elif a[0]=='imu':
                r=next(imu);assert int(a[2])==ns(r['t']),'native/shared IMU stamp differs'
                assert list(map(float.fromhex,a[4:10]))==[float(r[k]) for k in common.IMU_FIELDS[1:]],'native/shared ADC values differ'
                ni+=1
            elif a[0]=='uwb':
                r=next(uwb)
                assert int(a[9])==ns(r['t']) and int(a[5])==int(r['anchor_id']) and int(a[6])==int(r['tag_id']),'native/shared UWB time/source differs'
                assert int(a[1])==int(r['source_message']) and int(a[3])==int(r['source_range']),'native/shared raw range IDs differ'
                assert float.fromhex(a[13])==float(r['range']) and float.fromhex(a[14])==float(r['sigma']),'native/shared range or original sigma differs'
                assert list(map(float.fromhex,a[10:13]))==[float(r[k]) for k in ['anchor_x','anchor_y','anchor_z']],'native/shared anchor geometry differs'
                nu+=1
    doc=json.loads((inputs/'manifest.json').read_text());initial=doc['initial_observation']
    assert observed and int(observed[1])==ns(initial['t'])
    physical=list(map(float.fromhex,observed[2:18]))
    expected=[*initial['p'],*initial['v'],*initial['q'],*initial['ba'],*initial['bg']]
    assert physical==expected,'native/shared independent initial observation differs'
    return dict(status='PASS_EXACT_PHYSICAL_PREFIX',imu=ni,uwb=nu,independent_initial_observation_equal=True)
