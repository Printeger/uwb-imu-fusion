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


def reference_safe_score(score):
    """Retain numerical proxies without certifying an unresolved physical GT."""
    if score.get('reference_contract')=='OPEN_WALK_TRACKER_BODY_TAG':return score
    result=dict(score)
    result['reference_proxy_scores']=dict(score)
    result['reference_proxy_scores']['reference_contract']='FROZEN_TRACKER_PROXY_ONLY_NOT_PHYSICAL_BODY_OR_TAG'
    for key,val in score.items():
        if isinstance(val,dict) and 'rmse' in val:
            result[key]=dict(metric.summary([]),status='NA_REFERENCE_CONTRACT_OPEN')
    result['time_correspondences']=score.get('matched');result['matched']=0
    result['coverage']=None;result['reference_contract']='OPEN_WALK_TRACKER_BODY_TAG'
    result['reason']='CAL-W gyro/GT derivative rejects tracker=IMU identity; tracker-to-tag point and full body extrinsics not independently established. Frozen numerical reference retained only as proxy.'
    return result


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
    text=path.read_text();lines=text.splitlines();data=[]
    for i,line in enumerate(lines):
        if not line.strip() or line.startswith('#'):continue
        try:
            fields=list(map(float,line.split()))
            if len(fields)!=8:raise ValueError('TUM column count')
        except ValueError:
            if i==len(lines)-1 and not text.endswith('\n'):continue
            raise
        data.append(fields)
    data=np.asarray(data,float)
    if len(data)<2:raise ValueError('insufficient TUM states')
    return data[:,0],data[:,1:4],data[:,4:8]


def dataset_trajectory(run,lever):
    # SIGTERM can leave an incomplete final buffered TUM line. Complete state
    # rows are independently recorded actual post-commit body states; convert
    # only the physical tag point, with no GT fitting or estimator rerun.
    rr=rows(run/'states.csv')
    if len(rr)<2:return trajectory(run/'trajectory.tum')
    t=np.array([value(r,'t') for r in rr]);p=np.array([[value(r,'p'+k) for k in 'xyz'] for r in rr]);q=np.array([[value(r,'q'+k) for k in 'xyzw'] for r in rr])
    tag=p+Rotation.from_quat(q).apply(lever)
    try:
        tt,pp,qq=trajectory(run/'trajectory.tum');lookup={round(v*1e9):i for i,v in enumerate(t)}
        for stamp,point,quat in zip(tt,pp,qq):
            i=lookup.get(round(stamp*1e9))
            if i is None:continue
            assert np.linalg.norm(tag[i]-point)<1e-9,'state/TUM physical tag mismatch'
            assert (Rotation.from_quat(q[i]).inv()*Rotation.from_quat(quat)).magnitude()<1e-9,'state/TUM body orientation mismatch'
    except OSError:pass # no complete TUM buffer, complete states remain
    except ValueError as error:
        if str(error)!='insufficient TUM states':raise
    return t,tag,q


def native_episodes(data,manifest,paced=True):
    good=[r for r in data if value(r,'updates')==1];episodes=[]
    for i,f in enumerate(manifest['faults']):
        onset,axis,duration,amplitude,_=f;begin=(onset-1)*.05+.005;end=begin+duration*.001
        following=manifest['faults'][i+1][0] if i+1<len(manifest['faults']) else math.inf
        rr=[r for r in good if onset<=value(r,'epoch')<following]
        alarm=[r for r in rr if value(r,'imu_alarm')==1 and value(r,'first_imu_alarm_ns')>=round(begin*1e9)]
        bridges=[r for r in rr if value(r,'bridge')==1]
        repairs=[r for r in bridges if value(r,'repair')==1 and value(r,'removed_priors')>0]
        rejoin=[r for r in rr if bridges and value(r,'rejoined')==1 and value(r,'bridge')==0 and
                value(r,'epoch')>value(bridges[0],'epoch') and (value(r,'epoch')-1)*.05>=end-1e-9]
        unsafe=[r for r in rr if bridges and value(r,'rejoined')==1 and value(r,'bridge')==0 and
                value(r,'epoch')>value(bridges[0],'epoch') and (value(r,'epoch')-1)*.05<end-1e-9]
        first=lambda x:int(value(x[0],'epoch')) if x else None
        episodes.append(dict(kind='IMU',axis=axis,onset_epoch=onset,begin_s=begin,end_s=end,amplitude=amplitude,
            first_fd=first(alarm),first_bridge=first(bridges),first_prior_repair=first(repairs),
            rejoin_semantics='first four consecutive committed receipts in native rejoined state; not four separate rejoin transitions',
            rejoin4=[int(value(r,'epoch')) for r in rejoin[:4]],unsafe_shared_endpoint=[int(value(r,'epoch')) for r in unsafe],
            TTD_sensor_ms=(value(alarm[0],'first_imu_alarm_ns')*1e-9-begin)*1000 if alarm else None,
            TTD_wall_ms=value(alarm[0],'imu_alarm_ms')-begin*1000 if alarm else None,
            TTI_wall_ms=value(bridges[0],'commit_time_ms')-begin*1000 if bridges else None,
            TTR_wall_ms=value(rejoin[0],'commit_time_ms')-begin*1000 if rejoin else None,
            FD=bool(alarm),ISOLATION_COMMITTED=bool(bridges and repairs),
            rejoin_first4_consecutive=bool(len(rejoin)>=4 and np.all(np.diff([value(x,'epoch') for x in rejoin[:4]])==1)),
            REJOIN_EXECUTED=bool(repairs and len(rejoin)>=4 and not unsafe and np.all(np.diff([value(x,'epoch') for x in rejoin[:4]])==1)),
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
    serviced=[r for r in data if r.get('status')!='SOURCE_GAP_OR_ABORT' and math.isfinite(value(r,'core_ms'))]
    last=max([value(r,'epoch') for r in serviced],default=0)
    for episode in episodes:
        onset=episode['onset_epoch'];end_epoch=math.ceil(episode['end_s']/.05-1e-8) if episode['kind']=='IMU' else math.ceil(episode['end_s']/.05-1e-8)-1
        episode['exposure']='NOT_CONFIRMED_REACHED' if last<onset else 'PARTIAL_FAULT_INTERVAL' if last<end_epoch else 'FULL_FAULT_INTERVAL_CONFIRMED'
        episode['last_serviced_epoch']=int(last)
        episode['followup_sensor_ms']=max(0.,last*.05-episode['end_s'])*1000
        episode['qualified_service_epochs_in_episode']=sum(value(r,'qualified')==1 and onset<=value(r,'epoch')<=end_epoch for r in data)
        if episode['kind']=='IMU':
            episode['TTI_sensor_frontier_ms']=(episode['first_bridge']*.05-episode['begin_s'])*1000 if episode['first_bridge'] is not None else None
            episode['TTR_sensor_frontier_ms']=(episode['rejoin4'][0]*.05-episode['begin_s'])*1000 if episode['rejoin4'] else None
        else:
            episode['TTD_sensor_ms']=(episode['first_fd']-onset)*50 if episode['first_fd'] is not None else None
            episode['TTI_sensor_frontier_ms']=(episode['first_isolation']-onset)*50 if episode['first_isolation'] is not None else None
        if not paced:
            for key in ['TTD_wall_ms','TTI_wall_ms','TTR_wall_ms']:episode[key]=None
            episode['wall_latency_scope']='NA_OFFLINE_NO_SOURCE_TO_WALL_CLOCK_MAP'
    return episodes


def integrity(data,planned,pace):
    good=[r for r in data if spatial(r)]
    result=dict(planned=planned,received=len(data),attempted=sum(r.get('status')!='SOURCE_GAP_OR_ABORT' for r in data),
        committed=sum(value(r,'updates')==1 for r in data),spatial_valid=len(good),
        timely_spatial=sum(current(r) for r in data) if pace=='paced' else None,
        publication_protected=0,application_qualification='UNQUALIFIED',
        late_spatial=sum(spatial(r) and not current(r) for r in data) if pace=='paced' else None,
        missing=max(0,planned-len(data)),refused=sum(not spatial(r) for r in data),
        status_counts=dict(Counter(r.get('status') for r in data)),
        refusal_reason_counts=dict(Counter(r.get('recovery') for r in data if value(r,'updates')!=1)),axes={})
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
    for key in ['actual_run_id','native_input_parity','fde_executed','statistical_qualification','pl_method','scope_id','resource_observation','runner_error','thread_policy']:
        if key in execution:result[key]=execution[key]
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
        result['window_common_summary']={k:metric.summary([value(r,k) for r in selected if k not in ['hpl','vpl'] or spatial(r)]) for k in ['hpl','vpl','hpe','vpe','ape','core_ms']}
        if entry['group']=='E05':
            result['window_common_coverage']=dict(planned=1000,committed=len(selected),valid_bounds=sum(spatial(r) for r in selected),
                first_epoch=int(value(selected[0],'epoch')) if selected else None,last_epoch=int(value(selected[-1],'epoch')) if selected else None,
                complete_pose_interval={int(value(r,'epoch')) for r in selected}==set(range(1101,2101)))
        result['planned_epochs']=planned;result['common_scoring_epochs']=[1101,2100] if entry['group']=='E05' else [1,planned]
        if len(selected)>=2:
            t=np.array([value(r,'stamp_ns')*1e-9 for r in selected])
            p=np.array([[value(r,'tag_'+k) for k in 'xyz'] for r in selected])
            q=np.array([[value(r,'q'+k) for k in 'xyzw'] for r in selected])
            score,arrays=metric.evaluate((t,p,q),truth,lever)
            score['true_bias']='REGISTERED_SIM_RW_TRUTH; see accel_bias_error and gyro_bias_error'
            for key in ['velocity_error','accel_bias_error','gyro_bias_error']:
                score[key]=metric.summary([value(r,key) for r in selected])
            result['views'][entry['requested_method']]=score
            result['phase_views']={entry['requested_method']:phase_scores((t,p,q),truth,lever,manifest)}
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
        result['resource_scope']='RSS is instantaneous /proc/self/statm resident size sampled at saved native receipts; its reported max is a sampled maximum, not the process high-water mark. Full-process peak_rss_kib is available only with final runtime receipt. In-between and censored inflight peaks are not imputed.'
        result['same_center_M']={axis:dict(strict=sum(value(r,old)-value(r,new)>1e-9 for r in actual),
             widened=sum(value(r,old)<value(r,new) for r in actual),
             reduction_m=metric.summary([value(r,old)-value(r,new) for r in actual]))
             for axis,new,old in [('H','hpl','m_old_hpl'),('V','vpl','m_old_vpl')]}
        result['bias_envelope']={str(j):dict(axis=['accel_x','accel_y','accel_z','gyro_x','gyro_y','gyro_z'][j],unit='m/s^2' if j<3 else 'rad/s',width=metric.summary([value(r,'bwidth_'+str(j)) for r in actual]),
            reference_width=metric.summary([value(r,'reference_bwidth_'+str(j)) for r in actual]),
            last_width=value(actual[-1],'bwidth_'+str(j)) if actual else None,last_reference_width=value(actual[-1],'reference_bwidth_'+str(j)) if actual else None)
            for j in range(6)}
        result['last_recorded_epoch']=max([int(value(r,'epoch')) for r in native],default=0)
        result['actual_max_epoch']=max([int(value(r,'epoch')) for r in native if r.get('status')!='SOURCE_GAP_OR_ABORT'],default=0)
        result['max_committed_epoch']=max([int(value(r,'epoch')) for r in actual],default=0)
        result['native_failures']=[dict(epoch=int(value(r,'epoch')),status=r.get('status'),reason=r.get('recovery')) for r in native if r.get('status')=='ABORT']
        result['x0_retired']=any(value(r,'x0')==0 for r in actual)
        result['episodes']=native_episodes(native,manifest,entry['pace']=='paced')
        if not manifest.get('faults') and not manifest.get('uwb_faults'):
            result['known_healthy_diagnostics']={k:int(sum(value(x,k)==1 for x in actual)) for k in ['imu_alarm','uwb_alarm','bridge','repair','rejoined']}
        if entry.get('in_process_canary_epoch'):
            p=run/'canary.json';result['canary']=json.loads(p.read_text()) if p.exists() else dict(epoch=entry['in_process_canary_epoch'],status='NOT_REACHED_OR_NO_VALID_RECEIPT',admitted=False)
        import re
        console=(run/'console.log').read_text() if (run/'console.log').exists() else ''
        matches=re.findall(r'SC6 denominator scheduled=(\d+) released_imu=(\d+) enqueued_imu=(\d+) released_uwb=(\d+) enqueued_uwb=(\d+) queue_peak=(\d+) committed=(\d+) refused=(\d+)',console)
        if matches:result['source_delivery_counts']=dict(zip(['scheduled','released_imu','enqueued_imu','released_uwb','enqueued_uwb','queue_peak','committed','refused'],map(int,matches[-1])))
        cold=re.findall(r'SC6 COLD_START initialized_ms=([0-9.e+-]+) input_audit_ms=([0-9.e+-]+) ready_ms=([0-9.e+-]+)',console)
        if cold:result['cold_start_ms']=dict(zip(['provider_and_estimator_init','input_audit_export','total_ready'],map(float,cold[-1])))
        result['semantics']='ACTUAL_NATIVE_COMMITTED_TAG; raw conditional simulation only'
        result['phase_errors']={}
        first_fault=min([f[0] for f in manifest.get('faults',[])]+[f[0] for f in manifest.get('uwb_faults',[])],default=planned+1)
        intervals=[((f[0]-1)*.05+.005,(f[0]-1)*.05+.005+f[2]*.001) for f in manifest.get('faults',[])]+[(f[0]*.05,(f[0]+f[2]-1)*.05) for f in manifest.get('uwb_faults',[])]
        for name,rr in [('healthy_prefix',[r for r in actual if value(r,'epoch')<first_fault]),
                        ('fault_active',[r for r in actual if any(a-1e-9<=value(r,'epoch')*.05<=b+1e-9 for a,b in intervals)]),
                        ('bridge',[r for r in actual if value(r,'bridge')==1]),
                        ('rejoin',[r for r in actual if value(r,'rejoined')==1 and value(r,'bridge')==0]),
                        ('late_last_quarter',[r for r in actual if value(r,'epoch')>.75*planned])]:
            result['phase_errors'][name]={k:metric.summary([value(r,k) for r in rr]) for k in ['ape','hpe','vpe','rotation_error','velocity_error','accel_bias_error','gyro_bias_error']}
    else:
        imu_times=common.read_csv(inputs/'imu.csv');source_begin=float(imu_times[0]['t']);source_end=float(imu_times[-1]['t'])
        files={'S-ONLINE':'trajectory.tum.online.tum','S-FINAL':'trajectory.tum.history_final_calibration.tum'} if entry['requested_method']=='SFUISE' else {execution.get('effective_method') or 'UNKNOWN_INTERRUPTED_INTERFACE':'trajectory.tum'}
        for view,name in files.items():
            try:
                estimate=trajectory(run/name) if entry['requested_method']=='SFUISE' else dataset_trajectory(run,lever)
                if view=='S-FINAL':
                    estimate,validity=historical_support(run,estimate)
                    result['historical_export_validity']=validity
                keep=estimate[0]>=test_begin
                estimate=tuple(a[keep] for a in estimate)
                if len(estimate[0])<2:raise ValueError('fewer than two complete states in registered TEST interval; no suffix imputation')
                score,arrays=metric.evaluate(estimate,truth,lever)
                output_grid=gt.time[(gt.time>=max(test_begin,source_begin))&(gt.time<=source_end)]
                covered=metric.sample(*estimate,output_grid)[0]
                score['output_coverage_on_GT_TEST_grid']=dict(planned=len(output_grid),covered=int(covered.sum()),fraction=float(covered.mean()) if len(covered) else None,
                    estimate_states=len(estimate[0]),estimate_interval_s=[float(estimate[0][0]),float(estimate[0][-1])],
                    evaluation_domain_s=[max(test_begin,source_begin),source_end],
                    semantics='Descriptive coverage of the entire supplied GT TEST timeline within original IMU source bounds, not just successful output stamps; interpolation limited to 100 ms; not a causal emission rate or raw qualification')
                score['bias_error']='NA_NO_INDEPENDENT_REAL_GT' if entry['track']!='T-SIM' else 'NA_ESTIMATED_BIAS_NOT_EXPORTED'
                score['source_pose_receipt']=('FIRST_CAUSAL_TUM' if view=='S-ONLINE' else 'FINAL_RETAINED_HISTORY_VALID_DOMAIN; original-idle prefix unavailable') if entry['requested_method']=='SFUISE' else 'COMPLETE_POST_COMMIT_STATE_ROWS; fixed physical tag conversion cross-checked against complete TUM rows; partial final buffer refused'
                if entry['requested_method']=='SFUISE':score['velocity_error']='NA_ESTIMATED_VELOCITY_NOT_EXPORTED' if entry['track']=='T-SIM' else 'NA_NO_INDEPENDENT_REAL_VELOCITY_GT'
                elif entry['track']=='T-SIM':score.update(dataset_sim_errors(run,inputs,entry,registry))
                else:score['velocity_error']='NA_NO_INDEPENDENT_REAL_VELOCITY_GT'
                result['views'][view]=score
                if entry['track']=='T-SIM':phase_manifest=registry['cases'][entry['stream']]
                elif 'overlay' in doc:
                    onset=doc['overlay']['onset_s'];is_uwb_only=entry['stream'].endswith('-FU')
                    phase_manifest={'fault_intervals':[(onset+.045,onset+.195)] if is_uwb_only else [(onset,onset+.5)]}
                else:phase_manifest={}
                result.setdefault('phase_views',{})[view]=phase_scores(estimate,truth,lever,phase_manifest)
            except (ValueError,OSError,KeyError) as e:result['views'][view]={'status':'NA','reason':str(e)}
        attempts=rows(run/'attempts.csv')
        measured_attempts=[r for r in rows(run/'attempts.csv',keep_partial=True) if math.isfinite(value(r,'core_ms'))]
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
            result['general_receipts']={k:dict(Counter(r.get(k,'NOT_EXPORTED_W0') for r in attempts)) for k in ['action','detector_valid','detector_passed','bridge_active','history_removed_groups','history_added_groups','bridge_epochs','reinit_required','internal_protected_output']}
            result['general_recovery_semantics']='Engineering receipts only; no SC7 prior/bridge/rejoin qualification inherited'
            result['timing']={k:metric.summary([value(r,k) for r in measured_attempts]) for k in ['core_ms','measurement_to_sink_ms','state_age_ms']}
            result['partial_rows_with_measured_core']=sum(None in r.values() for r in measured_attempts)
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
                final=run/'observer.csv.final.json'
                if final.exists():
                    result['solver_final_receipt']=json.loads(final.read_text())
                    result['root_cpu_s']=result['solver_final_receipt'].get('process_cpu_ms',result['root_cpu_s']*1000)/1000
            else:
                result['timing']['measurement_to_sink_ms']='NA_OLD_W0_CALLBACK_CUTOFF_ONLY'
                result['timing']['input_cutoff_semantics']='OLD_W0_CONSERVATIVE_MAX_CALLBACK'
            result['source_clock_resolution_ms']=1.
            result['output_rate_hz']=len(cutoff)/max(1e-9,float(gt.time[-1]-gt.time[0]))
            result['observed_cutoff_vs_numeric_cutoff_separated']='observed_sensor_cutoff_ns' in observer[0]
        result['integrity']=dict(statistical_qualification='UNQUALIFIED_REAL_SOURCE' if entry['track']!='T-SIM' else 'NO_RAW_PL_METHOD',HPL=None,VPL=None,publication_protected=0)
    result['thread_policy_semantics']='Configured BLAS env 1; native TBB control 1; general candidate workers 1. ROS/system helper threads are not a measured one-thread guarantee; compare actual root CPU separately.'
    if 'overlay' in doc:
        onset=doc['overlay']['onset_s'];is_uwb_only=entry['stream'].endswith('-FU');physical_begin=onset+.045 if is_uwb_only else onset;physical_end=onset+.195 if is_uwb_only else onset+.5;attempted=rows(run/'attempts.csv')
        observed=max([value(x,'max_sensor_stamp_used') for x in attempted if math.isfinite(value(x,'max_sensor_stamp_used'))],default=float('nan'))
        if entry['requested_method']=='SFUISE':observed=result.get('solver_final_receipt',{}).get('max_numeric_input_ns',float('nan'))*1e-9-1000
        result['overlay_execution_coverage']=dict(manifest_base_onset_s=onset,physical_begin_s=physical_begin,physical_end_s=physical_end,last_confirmed_numeric_input_s=observed,
            onset_confirmed_reached=bool(math.isfinite(observed) and observed>=physical_begin),
            last_fault_input_confirmed_reached=bool(math.isfinite(observed) and observed>=physical_end),
            semantics='Completed native attempt / SF numeric receipt only; absent confirmation is not a claimed miss or false alarm; unmeasured inflight work may exist.')
        if entry['requested_method']=='SFUISE':state_times=[value(x,'state_ns')*1e-9-1000 for x in rows(run/'trajectory.tum.online_cutoff.csv')]
        else:state_times=[value(x,'t') for x in rows(run/'states.csv')]
        result['overlay_execution_coverage'].update(last_saved_state_s=max(state_times) if state_times else None,saved_states_after_overlay_begin=sum(t>=physical_begin for t in state_times),saved_states_after_overlay_end=sum(t>=physical_end for t in state_times),saved_state_scope='Actual complete first-stream state rows; no GT reconstruction or gap fill')
    duration=common.csv_duration(inputs/'imu.csv');result['source_duration_s']=duration
    cpu=result.get('root_cpu_s')
    if cpu is None or not math.isfinite(cpu):cpu=execution.get('children_cpu_s') if entry['requested_method']!='SFUISE' else None
    result['root_cpu_s']=cpu
    result['CPU_RTF']=cpu/duration if cpu is not None and duration else None
    core=result.get('timing',{}).get('core_ms',{})
    result['service_RTF']=core['mean']*core['n']/1000/duration if isinstance(core,dict) and core.get('mean') is not None and duration else None
    result['RTF_scope']='CPU_RTF=root CPU/source duration; service_RTF=all measurable service intervals/source duration; native/dataset processing_RTF also subtracts measured deliberate source waits. SF post-last-publish active tail wall is not measured; no whole-bag speed claim. Censored profile costs remain separate from completed-stream throughput.'
    if execution['status']=='RESOURCE_LIMIT':
        result['censored_profile_cost_ratios']=dict(CPU_over_planned_duration=result.get('CPU_RTF'),measured_service_over_planned_duration=result.get('service_RTF'))
        result['CPU_RTF']=None;result['service_RTF']=None;result['processing_RTF']=None
    if entry['stream'].split('-')[0] in ['R1','R2']:
        result['reference_contract']='OPEN_WALK_TRACKER_BODY_TAG'
        result['views']={view:reference_safe_score(score) for view,score in result['views'].items()}
        result['reference_proxy_phase_views']=result.pop('phase_views',{})
    if entry['stream'] in ['R5','R6']:
        result['input_contract']='OPEN_STAR_GYRO_FRAME_VALIDATION'
        result['input_contract_scope']='Observed frozen-interface behavior only; CAL-S gyro/GT consistency and transform directions remain open. No intrinsic algorithm robustness claim. Official tag-point consistency is independently retained.'
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
        d=json.loads(path.read_text());attempts.append(dict(id=path.parent.name,status=d['status'],source_identity=d['identity'],reason=d.get('reason'),returncode=d.get('returncode'),input_digest=d.get('input_digest'),input_files=d.get('input_files'),output_hashes=d.get('output_hashes'),elapsed_s=d.get('elapsed_s'),actual_run_id=d.get('actual_run_id'),runner_error=d.get('runner_error')))
    validation={}
    for name in ['input-checks/result.json','input-checks/bag-parity.json','input-checks/all-bag-parity.json','spline-export-check/result.json','reference-check/result.json','native-observer-prefix-check.json','sfuise_observer/check.json','equivalence/R3/equivalence.json','final-checks.json']:
        p=CACHE/name
        if p.exists():validation[name]=dict(sha256=common.sha256(p),result=json.loads(p.read_text()))
    counts=dict(Counter(r['status'] for r in records));assert sum(counts.values())==len(registry['runs'])
    attempt_labels=dict(Counter(a['status'] for a in attempts))
    exit_labels=dict(Counter('RUNNING' if a['status']=='RUNNING' else 'EXIT_0' if a.get('returncode')==0 else 'EXIT_NONZERO' if a.get('returncode') is not None else 'NO_EXIT_CODE_RECORDED' for a in attempts))
    pairs=paired_results(records,registry)
    metrics=dict(schema='sc8-metrics-v1',status='IN_PROGRESS',registry_sha256=common.sha256(ROOT/'config/sc8_experiments.yaml'),
        registered=len(registry['runs']),logical_run_status_counts=counts,attempt_processes=len(attempts),attempt_status_counts=attempt_labels,attempt_exit_counts=exit_labels,
        runs=records,attempts=attempts,paired=pairs,overlay_effects=overlay_effects(records,registry),unit_aggregates=unit_aggregates(records),phase_aggregates=phase_aggregates(records),ablation_comparisons=ablation_comparisons(records,registry),validation=validation,auxiliary_solver_audits=registry.get('auxiliary_solver_audits'),statistical_unit='SEED_OR_RECORDING_NOT_FRAMES')
    metrics['status'],metrics['execution_gaps']=evaluation_status(records,validation)
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


def phase_scores(estimate,truth,lever,manifest):
    t=estimate[0];intervals=list(manifest.get('fault_intervals',[]))
    intervals += [((f[0]-1)*.05+.005,(f[0]-1)*.05+.005+f[2]*.001) for f in manifest.get('faults',[])]
    intervals += [(f[0]*.05,(f[0]+f[2]-1)*.05) for f in manifest.get('uwb_faults',[])]
    active=np.zeros(len(t),bool)
    for a,b in intervals:active|=(t>=a-1e-9)&(t<=b+1e-9)
    masks={'late_last_quarter':t>=t[0]+.75*(t[-1]-t[0])}
    if intervals:
        masks.update(healthy_before=t<min(a for a,b in intervals),fault_active=active,
                     post_last_fault=t>max(b for a,b in intervals))
    else:masks['unmodified_recording_or_healthy']=np.ones(len(t),bool)
    out={}
    for phase,mask in masks.items():
        if mask.sum()<2:out[phase]={'status':'NA','n':int(mask.sum())};continue
        try:out[phase]=metric.evaluate(tuple(x[mask] for x in estimate),truth,lever)[0]
        except ValueError as e:out[phase]={'status':'NA','reason':str(e)}
    return out


def run_poses(record,entry):
    path=latest(entry)
    if path is None:return {},None,None
    execution=json.loads(path.read_text());run=path.parent;inputs=ROOT/execution['inputs']
    doc=json.loads((inputs/'manifest.json').read_text());gt=common.load_gt(inputs/'gt.csv',doc['primary_tag'])
    rr=next(r for r in common.read_csv(inputs/'uwb.csv') if int(r['tag_id'])==doc['primary_tag'])
    lever=[float(rr[k]) for k in ['lever_x','lever_y','lever_z']]
    poses={}
    data=[r for r in rows(run/'trace.csv') if value(r,'updates')==1]
    if data:
        poses[entry['requested_method']]=(np.array([value(r,'stamp_ns')*1e-9 for r in data]),
            np.array([[value(r,'tag_'+k) for k in 'xyz'] for r in data]),
            np.array([[value(r,'q'+k) for k in 'xyzw'] for r in data]))
    else:
        for view,name in ({'S-ONLINE':'trajectory.tum.online.tum','S-FINAL':'trajectory.tum.history_final_calibration.tum'} if entry['requested_method']=='SFUISE' else {record.get('effective_method','UNKNOWN'):'trajectory.tum'}).items():
            try:
                poses[view]=trajectory(run/name) if entry['requested_method']=='SFUISE' else dataset_trajectory(run,lever)
                if view=='S-FINAL':poses[view]=historical_support(run,poses[view])[0]
            except (OSError,ValueError):continue
    for view,pose in list(poses.items()):
        mask=np.ones(len(pose[0]),bool)
        if record.get('scoring_begin_s') is not None:mask&=pose[0]>=record['scoring_begin_s']
        if entry['group']=='E05':mask&=(pose[0]>=55.05-1e-8)&(pose[0]<=105+1e-8)
        poses[view]=tuple(x[mask] for x in pose)
        if mask.sum()<2:del poses[view]
    return poses,(gt.time,gt.position,gt.quaternion),lever


def paired_results(records,registry):
    """Independent ON/OFF gains and a separate common-view display grid."""
    grouped={};index={e['id']:e for e in registry['runs']}
    for r in records:
        if r['group'] not in ['E01','E02','E03'] or r['status'] in ['RUNNING','NOT_ATTEMPTED'] or 'scoring_error' in r:continue
        key=(r['group'],r['stream'],index[r['id']]['seed']);grouped.setdefault(key,[]).append(r)
    def grid_for(truth,poses):
        grid=truth[0];valid=np.ones(len(grid),bool)
        for p in poses:valid&=metric.sample(*p,grid)[0]
        return grid[valid]
    def scored(poses,truth,lever,grid):
        result={}
        if len(grid)<2:return result
        grid=grid[metric.sample(*truth,grid)[0]]
        if len(grid)<2:return result
        target=(grid,*metric.sample(*truth,grid)[1:])
        for view,p in poses.items():result[view]=metric.evaluate((grid,*metric.sample(*p,grid)[1:]),target,lever)[0]
        return result
    pairs=[]
    for key,group in grouped.items():
        by={r['requested_method']:r for r in group}
        if not all(m in by for m in ['O-FULL','O-NOFDE']):continue
        poses={};truth=lever=None
        for r in group:
            pp,truth,lever=run_poses(r,index[r['id']]);poses.update(pp)
        if truth is None or not poses:continue
        on=by['O-FULL'].get('effective_method','O-FULL');off=by['O-NOFDE'].get('effective_method','O-NOFDE')
        eligible=on in poses and off in poses
        gain_grid=grid_for(truth,[poses[on],poses[off]]) if eligible else np.array([])
        gain_scores=scored({k:poses[k] for k in [on,off]},truth,lever,gain_grid) if eligible else {}
        display_grid=grid_for(truth,list(poses.values()));scores=scored(poses,truth,lever,display_grid)
        gains={}
        if on in gain_scores and off in gain_scores:
            for name in ['ape_m','hpe_m','vpe_m','are_rad','rpe_tag_0.5s_m','rpe_body_0.5s_m','rpe_body_0.5s_rad','rpe_tag_1s_m','rpe_body_1s_m','rpe_body_1s_rad']:
                a=gain_scores[on][name]['rmse'];b=gain_scores[off][name]['rmse']
                gains[name]=dict(off_minus_on=b-a if a is not None and b is not None else None,percent=100*(b-a)/b if a is not None and b else None)
        observed_gains=gains;post_fault_n=None;fault_effect_evaluable=None
        if by['O-FULL'].get('track')=='T-SEMI':
            begin=by['O-FULL'].get('overlay_execution_coverage',{}).get('physical_begin_s',float('inf'))
            post_fault_n=int(np.sum(gain_grid>=begin));fault_effect_evaluable=post_fault_n>=2
            if not fault_effect_evaluable:gains={k:dict(off_minus_on=None,percent=None) for k in gains}
        history={}
        if all(k in poses for k in ['S-ONLINE','S-FINAL']):
            hg=poses['S-ONLINE'][0];mask=metric.sample(*poses['S-FINAL'],hg)[0]&metric.sample(*truth,hg)[0];hg=hg[mask]
            history=dict(common_n=len(hg),scores=scored({k:poses[k] for k in ['S-ONLINE','S-FINAL']},truth,lever,hg))
        proxy_comparison={};reference_open=by['O-FULL'].get('reference_contract')=='OPEN_WALK_TRACKER_BODY_TAG'
        if reference_open:
            proxy_comparison=dict(scores=scores,gain_scores=gain_scores,observed_interval_gains=observed_gains,SF_history_common=history)
            scores={k:reference_safe_score(v) for k,v in scores.items()};gain_scores={k:reference_safe_score(v) for k,v in gain_scores.items()}
            history=dict(history,scores={k:reference_safe_score(v) for k,v in history.get('scores',{}).items()}) if history else {}
            gains={k:dict(off_minus_on=None,percent=None) for k in gains}
            if fault_effect_evaluable is not None:fault_effect_evaluable=False
        pairs.append(dict(group=key[0],stream=key[1],seed=key[2],common_grid_n=len(gain_grid),
            common_interval_s=[float(gain_grid[0]),float(gain_grid[-1])] if len(gain_grid) else None,
            gains=gains,gain_scores=gain_scores,observed_interval_gains=observed_gains,post_fault_common_n=post_fault_n,fault_effect_evaluable=fault_effect_evaluable,
            reference_contract='OPEN_WALK_TRACKER_BODY_TAG' if reference_open else 'REGISTERED_REFERENCE',reference_proxy_comparison=proxy_comparison,
            input_contract=by['O-FULL'].get('input_contract','REGISTERED_INPUT'),
            gain_scope='ON/OFF first-stream intersection only; independent of SF/historical missing data. For real overlays, fault gains require at least two common post-injection states; pre-only observed interval values remain separate.',
            scores=scores,display_common_grid_n=len(display_grid),display_common_interval_s=[float(display_grid[0]),float(display_grid[-1])] if len(display_grid) else None,
            available_views=list(poses),missing_main_online_views=[k for k in [on,off,'S-ONLINE'] if k not in poses],
            display_scope='intersection of all AVAILABLE views including valid historical support; missing methods remain NA; no claim of ON common coverage if ON absent',
            SF_history_common=history,unit='seed_or_recording',actual_online_cutoffs_differ=True,
            grid_semantics='GT evaluator grid; interpolation <=100ms, no estimator alignment or feedback; state age separate',
            input_digests_equal=len({r.get('input_digest') for r in group})==1,no_output_suffix_imputed=True))
    return pairs


def compressed_ecdf(a,limit=192):
    a=np.sort(np.asarray(a,float));a=a[np.isfinite(a)]
    if not len(a):return {'x':[],'cdf':[],'n':0}
    ix=np.unique(np.r_[0,np.ceil(np.linspace(0,len(a)-1,min(limit,len(a)))).astype(int),len(a)-1])
    return dict(x=a[ix],cdf=(ix+1)/len(a),n=len(a),over40=int((a>40).sum()),over50=int((a>50).sum()),negative=int((a<0).sum()))


def envelope_indices(t,signals,events=(),bins=192):
    """Keep first/last, each bin's actual min/max, and event +/-1 s."""
    t=np.asarray(t);keep={0,len(t)-1}
    for segment in np.array_split(np.arange(len(t)),min(bins,len(t))):
        for signal in signals:
            values=np.asarray(signal)[segment];finite=np.isfinite(values)
            if finite.any():
                ii=segment[finite];vv=values[finite];keep.update([int(ii[np.argmin(vv)]),int(ii[np.argmax(vv)])])
    for event in events:keep.update(np.where(abs(t-event)<=1.)[0].tolist())
    return np.array(sorted(keep),int)


def build_plotdata(metrics,registry):
    index={e['id']:e for e in registry['runs']};curves={};diagnostics={};stanford={};latency={};si={};references={};raw_ranges={};bias_bounds={}
    for r in metrics['runs']:
        if r['group']=='E00' or r['status'] in ['NOT_ATTEMPTED','RUNNING'] or 'scoring_error' in r:continue
        entry=index[r['id']];path=latest(entry);run=path.parent
        raw=rows(run/'trace.csv');good=[x for x in raw if spatial(x)]
        if raw and r['stream'] in ['SC8-D0','SC8-J'] and r.get('seed')==20261020 and ((r['group'] in ['E01','E03'] and r['requested_method']=='O-FULL') or (r['group']=='E07' and r['requested_method']=='O-ABL-JOINT')):
            actual=[x for x in raw if value(x,'updates')==1]
            if actual:
                tt=np.array([value(x,'epoch')*.05 for x in actual]);signals={k:np.array([value(x,k) for x in actual]) for k in ['bwidth_2','bwidth_5','reference_bwidth_2','reference_bwidth_5']}
                ix=envelope_indices(tt,list(signals.values()))
                bias_bounds[r['id']]=dict(stream=r['stream'],method=r['requested_method'],t=tt[ix],full_n=len(tt),**{k:v[ix] for k,v in signals.items()})
        main=r['group'] in ['E01','E03'] and r['stream'] in ['SC8-H','SC8-U','SC8-A','SC8-G','SC8-J','SC8-D0','SC8-G2']
        if main and raw:
            method=r['requested_method'];stanford.setdefault(method,[]).extend([[value(x,'hpe'),value(x,'hpl'),value(x,'vpe'),value(x,'vpl'),int(current(x))] for x in good])
            for axis,pe,pl in [('H','hpe','hpl'),('V','vpe','vpl')]:
                si.setdefault(method+' '+axis,[]).extend(value(x,pe)/value(x,pl) for x in good if value(x,pl)>0)
        if main or r['group']=='E02':
            phase='Real' if r['group']=='E02' else 'Healthy' if r['group']=='E01' else 'Fault'
            method='S-ONLINE' if r['requested_method']=='SFUISE' else r.get('effective_method')
            if raw:vals=[value(x,'core_ms') for x in raw if x.get('status')!='SOURCE_GAP_OR_ABORT']
            else:vals=[value(x,'core_ms') for x in rows(run/('observer.csv' if method=='S-ONLINE' else 'attempts.csv'),keep_partial=method!='S-ONLINE')]
            latency.setdefault(phase+'|'+method+'|core',[]).extend(vals)
            if raw:sinks=[value(x,'delivery_ms') for x in raw if x.get('status')!='SOURCE_GAP_OR_ABORT']
            elif method=='S-ONLINE':
                obs={x['state_ns']:x for x in rows(run/'observer.csv')}
                sinks=[(value(x,'source_clock_at_sink_ns')-value(obs[x['state_ns']],'max_sensor_stamp_used'))*1e-6 for x in rows(run/'trajectory.tum.online_cutoff.csv') if x['state_ns'] in obs]
            else:sinks=[value(x,'measurement_to_sink_ms') for x in rows(run/'attempts.csv',keep_partial=True)]
            latency.setdefault(phase+'|'+method+'|sink',[]).extend(sinks)
        representative=(r['group']=='E02' and r['stream'] in ['R1','R3','R5']) or (r['group']=='E03' and r['stream']=='SC8-J' and r['seed']==20261020) or r['group']=='E08'
        if representative:
            poses,truth,lever=run_poses(r,entry)
            if r['group']=='E03' and r['stream']=='SC8-J' and r.get('seed')==20261020 and not raw_ranges:
                execution=json.loads(path.read_text());source=ROOT/execution['inputs'];rr=common.read_csv(source/'uwb.csv');aid=min(int(x['anchor_id']) for x in rr);rr=[x for x in rr if int(x['anchor_id'])==aid]
                tt=np.array([float(x['t']) for x in rr]);ok,pp,_=metric.sample(*truth,tt)
                anchor=np.array([[float(x[k]) for k in ['anchor_x','anchor_y','anchor_z']] for x in rr]);observed=np.array([float(x['range']) for x in rr])
                raw_ranges=dict(stream='SC8-J',seed=20261020,anchor_id=aid,t=tt[ok],error_m=(observed-np.linalg.norm(pp-anchor,axis=1))[ok],scope='Actual common raw range minus source-truth distance; evaluator-only diagnostic, not estimator input')
            if r['group']=='E02' and truth is not None and r['stream'] not in references:
                mask=truth[0]>=r.get('scoring_begin_s') if r.get('scoring_begin_s') is not None else np.ones(len(truth[0]),bool)
                rt,rp=truth[0][mask],truth[1][mask]
                ri=envelope_indices(rt,[rp[:,j] for j in range(3)])
                references[r['stream']]=dict(t=rt[ri],p=rp[ri],full_n=len(rt),reference_contract=r.get('reference_contract','REGISTERED_REFERENCE'),input_contract=r.get('input_contract','REGISTERED_INPUT'),gap_intervals=[[float(rt[j]),float(rt[j+1])] for j in np.where(np.diff(rt)>.100000001)[0]])
            for view,pose in poses.items():
                try:_,a=metric.evaluate(pose,truth,lever)
                except ValueError:continue
                if not len(a['t']):continue
                events=[(f[0]-1)*.05+.005 for f in registry['cases'].get(r['stream'],{}).get('faults',[])]
                ix=envelope_indices(a['t'],[a[k] for k in ['ape','are','hpe','vpe']],events)
                curves[r['id']+'|'+view]=dict(run=r['id'],stream=r['stream'],view=view,reference_contract=r.get('reference_contract','REGISTERED_REFERENCE'),input_contract=r.get('input_contract','REGISTERED_INPUT'),
                    **{k:np.asarray(v)[ix] for k,v in a.items()},full_matched_n=len(a['t']),display_n=len(ix),
                    gap_intervals=[[float(a['t'][j]),float(a['t'][j+1])] for j in np.where(np.diff(a['t'])>.100000001)[0]])
        if raw and ((r['stream']=='SC8-J' and r.get('seed')==20261020 and r['group']=='E03') or r['group']=='E08'):
            actual=[x for x in raw if value(x,'updates')==1]
            if not actual:continue
            keys=['ape','rotation_error','hpl','vpl','bias_width','core_ms','delivery_ms','slots','rss','ledger','replay','capsules','q','alerts','source_prefix','active_states','proof_explicit','imu_alarm','uwb_alarm','bridge','repair','removed_priors','removed_groups','rejoined','excluded','imu_margin_0','imu_margin_3','temporal_margin_0']
            tt=np.array([value(x,'epoch')*.05 for x in actual]);signals=[[value(x,k) if k not in ['hpl','vpl'] or spatial(x) else float('nan') for x in actual] for k in keys]
            cores=np.array([value(x,'core_ms') for x in actual]);rolling=[np.sort(cores[max(0,i-19):i+1])[math.ceil(.95*min(i+1,20))-1] for i in range(len(cores))]
            keys.append('core_rolling_p95');signals.append(rolling)
            events=[f[0]*.05 for f in registry['cases'][r['stream']]['faults']]
            ix=envelope_indices(tt,[*signals[:10],rolling],events)
            event_indices=[j for j,x in enumerate(actual) if any(value(x,k)==1 for k in ['imu_alarm','uwb_alarm','bridge','repair'])]
            rejoin_state=np.array([value(x,'rejoined')==1 and value(x,'bridge')==0 for x in actual])
            transitions=np.flatnonzero(np.r_[rejoin_state[0],rejoin_state[1:]!=rejoin_state[:-1]])
            for j in transitions:event_indices.extend(range(max(0,j-1),min(len(actual),j+4)))
            ix=np.unique(np.r_[ix,event_indices]).astype(int)
            diagnostics[r['id']]=dict(t=tt[ix],**{k:np.asarray(v)[ix] for k,v in zip(keys,signals)},full_n=len(actual),display_n=len(ix))
    hist={}
    for method,points in stanford.items():
        points=np.asarray(points);hist[method]={}
        for axis,cols,zoom in [('H',(0,1),5.),('V',(2,3),5.)]:
            pe,pl=points[:,cols[0]],points[:,cols[1]];full=max(10.,float(np.max(np.r_[pe,pl]))*1.02)
            axisdata={}
            for view,extent in [('full',full),('zoom',zoom)]:
                xbins=np.linspace(0,extent if view=='full' else .5,65);ybins=np.linspace(0,extent,65);layers={}
                for label,mask in [('timely',points[:,4]==1),('late',points[:,4]==0)]:
                    counts,_,_=np.histogram2d(pe[mask],pl[mask],bins=[xbins,ybins]);layers[label]=counts.astype(int)
                axisdata[view]=dict(x_edges=xbins,y_edges=ybins,layers=layers,total=len(pe),outside=int(((pe>xbins[-1])|(pl>ybins[-1])).sum()),within_pl=int((pe<=pl).sum()))
            hist[method][axis]=axisdata
    return dict(schema='sc8-plotdata-v1',curves=curves,reference_tracks=references,joint_raw_range=raw_ranges,bias_bounds=bias_bounds,native_diagnostics=diagnostics,stanford=hist,
        si={k:dict(compressed_ecdf(v),over1=int(np.sum(np.asarray(v)>1))) for k,v in si.items()},
        latency={k:compressed_ecdf(v) for k,v in latency.items()},
        curve_reduction='Actual per-bin extrema for all displayed signals + first/last + fault +/-1s; native alarm/bridge/repair receipts retained, persistent rejoined-state transitions retain preceding and first4 following receipts. Statistics use every sample. ECDF keeps actual ranks; histogram bins sum actual PE/PL points.',
        histogram_scope='E01/E03 registered main SIM cases only; real/SF PL absent; frame counts descriptive, not independent confidence samples')


def figures():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm
    from matplotlib.lines import Line2D
    metrics=json.loads((OUT/'SC8_METRICS.json').read_text());registry=yaml.safe_load((ROOT/'config/sc8_experiments.yaml').read_text())
    source=OUT/'SC8_PLOTDATA.json'
    if (CACHE/'runs').exists() and any((CACHE/'runs').glob('*/trace.csv')):
        data=build_plotdata(metrics,registry);save(source,data);data=json.loads(source.read_text())
    else:data=json.loads(source.read_text())
    dest=OUT/'figures';dest.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.size':9,'axes.titlesize':10,'axes.labelsize':9,'legend.fontsize':8,
        'xtick.labelsize':8,'ytick.labelsize':8,'svg.fonttype':'none','font.family':'DejaVu Sans',
        'axes.spines.top':False,'axes.spines.right':False,'savefig.dpi':300,'axes.grid':True,'grid.alpha':.2})
    records=metrics['runs'];byid={r['id']:r for r in records};figlog=[]
    def sub(nr=1,nc=1,height=4.5):return plt.subplots(nr,nc,figsize=(180/25.4,height),squeeze=False,constrained_layout=True)
    def finish(fig,name,note):
        import textwrap
        fig.supxlabel(textwrap.fill(note,105),fontsize=8)
        for ext in ['png','svg']:fig.savefig(dest/(name+'.'+ext),dpi=300)
        plt.close(fig);figlog.append(dict(id=name,note=note))
    def val(r,key='ape_m',view=None,stat='rmse'):
        views=r.get('views',{})
        if view is None:view=next(iter(views),'')
        return views.get(view,{}).get(key,{}).get(stat)
    def number(v,d=3):return 'NA' if v is None else f'{v:.{d}g}'
    def no(ax,text='NA: no completed output for this scope'):
        ax.text(.5,.5,text,ha='center',va='center',transform=ax.transAxes,color='#777777')
    def color(view):return COLORS.get(view,'#333333')
    # F00: process completion and statistical admission are separate columns.
    fig,axs=sub(1,2,3.3);ax=axs[0,0]
    labels=[];matrix=np.zeros((6,4));texts=[]
    for i,rid in enumerate(['R1','R2','R3','R4','R5','R6']):
        labels.append(rid);rr=[r for r in records if r['group']=='E00' and r['stream']==rid]
        cell=['REFUSED'];code=[0]
        for method in ['O-FULL','O-NOM','SFUISE']:
            r=next((r for r in rr if r['requested_method']==method),{})
            state=r.get('status','NOT_ATTEMPTED');cell.append('Complete' if state=='SUCCESS' else 'Censored' if state=='RESOURCE_LIMIT' else state.replace('NOT_ATTEMPTED','NA'));code.append(2 if state=='SUCCESS' else 1 if state=='RESOURCE_LIMIT' else 0)
        matrix[i]=code;texts.append(cell)
    from matplotlib.colors import ListedColormap
    ax.imshow(matrix,cmap=ListedColormap(['#e5e5e5','#f2ce7e','#99c9a9']),vmin=0,vmax=2,aspect='auto')
    for i in range(6):
        for j in range(4):ax.text(j,i,texts[i][j],ha='center',va='center',fontsize=8)
    ax.set(xticks=range(4),xticklabels=['Raw PL','ENG FDE','Nominal','SFUISE'],yticks=range(6),yticklabels=labels,title='W0 admission / process outcomes');ax.grid(False)
    inventory=registry['inventory'];items=[next(x for x in inventory if x['id']==rid) for rid in labels]
    ax=axs[0,1];dur=[x['imu_duration_s'] for x in items];ax.barh(range(6),dur,color='#999999');ax.invert_yaxis()
    ax.set(yticks=range(6),yticklabels=labels,xlabel='Original IMU duration (s)',title='All six retained; no accuracy screening')
    for i,x in enumerate(items):ax.text(dur[i],i,f" {x['imu_hz']:.0f} Hz",va='center',fontsize=8)
    finish(fig,'F00_admission','T-REAL raw refused. Walk physical reference OPEN; STAR gyro-input validation OPEN. W0 fixed 20 s prefixes: process completion is not qualification or reference closure.')
    # F01: every independent seed / recording, online and historical separated.
    fig,axs=sub(1,2,5.6);pairs=[p for p in metrics.get('paired',[]) if p['group'] in ['E01','E02']]
    ylabels=[p['stream']+(f" / {str(p['seed'])[-2:]}" if p['group']=='E01' else '')+(' (ref OPEN)' if p.get('reference_contract')=='OPEN_WALK_TRACKER_BODY_TAG' else '')+(' (input OPEN)' if p.get('input_contract')=='OPEN_STAR_GYRO_FRAME_VALIDATION' else '') for p in pairs]
    for ax,key,title in [(axs[0,0],'ape_m','Tag APE RMSE (m)'),(axs[0,1],'are_rad','Body ARE RMSE (rad)')]:
        for i,p in enumerate(pairs):
            scores=p['scores'];points=[]
            for view in ['O-FULL','O-NOFDE','O-FDE-ENG','O-ENG-NOFDE','S-ONLINE','S-FINAL']:
                v=scores.get(view,{}).get(key,{}).get('rmse')
                if v is None:continue
                ax.plot(v,i,marker='D' if view=='S-FINAL' else 'x' if view in ['O-NOFDE','O-ENG-NOFDE'] else 'o',ms=5 if view in ['O-FULL','O-FDE-ENG'] else 4,color=color(view),fillstyle='none' if view in ['S-FINAL','O-FULL','O-FDE-ENG'] else 'full');points.append((view,v))
            ours=[v for k,v in points if k.startswith('O-')]
            if len(ours)==2:ax.plot(ours,[i,i],color='#aaaaaa',lw=.7,zorder=0)
            sf=[v for k,v in points if k.startswith('S-')]
            if len(sf)==2:ax.plot(sf,[i,i],color=COLORS['S-FINAL'],ls='--',lw=.7)
            if not points:ax.text(.02,i,'NA',transform=ax.get_yaxis_transform(),color='#777777')
        ax.set(yticks=range(len(pairs)),yticklabels=ylabels,title=title);ax.invert_yaxis()
        if key=='ape_m':ax.set_xscale('symlog',linthresh=.1)
        values=[s.get(key,{}).get('rmse') for p in pairs for s in p['scores'].values()];values=[v for v in values if v is not None]
        maximum=max(values,default=1.)
        ax.set_xlim(0,maximum*1.2 if key=='ape_m' else maximum+.15)
    axs[0,0].legend(handles=[Line2D([],[],color=color(v),marker='D' if v=='S-FINAL' else 'x' if v=='O-NOFDE' else 'o',ls='--' if v=='S-FINAL' else '',label=v.replace('O-FULL','O-FULL / ENG')) for v in ['O-FULL','O-NOFDE','S-ONLINE','S-FINAL']],loc='lower left',bbox_to_anchor=(0,1.10),ncol=2,fontsize=8)
    finish(fig,'F01_paired_accuracy','Each row is one seed / recording; common AVAILABLE-view domain. Walk physical scores are NA (reference OPEN); STAR rows are input OPEN. Numerical proxies/interface outcomes remain in the report. No alignment; SF cutoff/lag differs. Final history excludes invalid original-idle support.')
    # F02: preselected one recording per family. Same XY axes within each family.
    fig,axs=sub(3,2,7.0)
    for i,rid in enumerate(['R1','R3','R5']):
        cc=[c for c in data['curves'].values() if c['stream']==rid]
        norm=matplotlib.colors.Normalize(vmin=0,vmax=max([max(c['ape']) for c in cc],default=1))
        colored=None
        for c in cc:
            p=np.asarray(c['estimate']);t=np.asarray(c['t']);view=c['view'];ls='--' if view=='S-FINAL' else '-'
            tt,pp=display_with_gaps(t,p,c.get('gap_intervals',[]))
            axs[i,0].plot(pp[:,0],pp[:,1],color=color(view),ls=ls,lw=.8,label=view)
            colored=axs[i,0].scatter(p[:,0],p[:,1],c=c['ape'],norm=norm,cmap='viridis',s=3,
                marker={'S-ONLINE':'^','S-FINAL':'D','O-NOFDE':'x','O-ENG-NOFDE':'x'}.get(view,'o'),linewidths=.2,alpha=.75)
            axs[i,1].plot(tt,pp[:,2],color=color(view),ls=ls,lw=.8,label=view)
        if colored is not None:fig.colorbar(colored,ax=axs[i,0],label='Proxy distance (m)' if rid=='R1' else 'Tag APE (m)',fraction=.045,pad=.02)
        reference=data.get('reference_tracks',{}).get(rid)
        if reference:
            tt,gg=display_with_gaps(np.asarray(reference['t']),np.asarray(reference['p']),reference.get('gap_intervals',[]))
            reference_label='Frozen GT proxy' if rid=='R1' else 'GT'
            axs[i,0].plot(gg[:,0],gg[:,1],'k-',lw=1,label=reference_label);axs[i,1].plot(tt,gg[:,2],'k-',lw=1,label=reference_label)
        elif cc:
            c=max(cc,key=lambda c:len(c['t']));gt=np.asarray(c['truth']);tt,gg=display_with_gaps(np.asarray(c['t']),gt,c.get('gap_intervals',[]));axs[i,0].plot(gg[:,0],gg[:,1],'k-',lw=1,label='GT');axs[i,1].plot(tt,gg[:,2],'k-',lw=1)
        if not cc:no(axs[i,0]);no(axs[i,1])
        axs[i,0].set(title=f'{rid}: tag XY',xlabel='x (m)',ylabel='y (m)',aspect='equal');axs[i,1].set(title=f'{rid}: tag height',xlabel='Sensor time (s)',ylabel='z (m)')
        if rid=='R1':axs[i,0].set_title('R1: tracker proxy / ref OPEN');axs[i,1].set_title('R1: frozen proxy reference')
        if rid=='R5':axs[i,0].set_title('R5: gyro frame OPEN');axs[i,1].set_title('R5: frozen-interface result')
        if cc and max(max(abs(np.asarray(c['estimate']).ravel())) for c in cc)>1000:
            axs[i,0].ticklabel_format(style='sci',axis='both',scilimits=(0,0),useMathText=True)
            if reference:
                gp=np.asarray(reference['p']);rt=np.asarray(reference['t']);low=gp.min(axis=0);high=gp.max(axis=0);pad=np.maximum((high-low)*.15,.2)
                zoom=axs[i,0].inset_axes([.54,.18,.43,.40]);zoom.set_title('GT neighborhood',fontsize=8)
                for c in cc:
                    tt,pp=display_with_gaps(np.asarray(c['t']),np.asarray(c['estimate']),c.get('gap_intervals',[]));zoom.plot(pp[:,0],pp[:,1],color=color(c['view']),ls='--' if c['view']=='S-FINAL' else '-',lw=.65)
                tt,gg=display_with_gaps(rt,gp,reference.get('gap_intervals',[]));zoom.plot(gg[:,0],gg[:,1],'k-',lw=.7);zoom.set(xlim=(low[0]-pad[0],high[0]+pad[0]),ylim=(low[1]-pad[1],high[1]+pad[1]));zoom.tick_params(labelsize=8)
                zoomz=axs[i,1].inset_axes([.50,.50,.47,.32]);zoomz.set_title('GT neighborhood',fontsize=8)
                for c in cc:
                    tt,pp=display_with_gaps(np.asarray(c['t']),np.asarray(c['estimate']),c.get('gap_intervals',[]));zoomz.plot(tt,pp[:,2],color=color(c['view']),ls='--' if c['view']=='S-FINAL' else '-',lw=.65)
                zoomz.plot(rt,gp[:,2],'k-',lw=.7);zoomz.set(xlim=(rt[0],rt[-1]),ylim=(low[2]-pad[2],high[2]+pad[2]));zoomz.tick_params(labelsize=8)
    axs[0,1].legend(fontsize=8,loc='best')
    finish(fig,'F02_real_trajectories','First listed recording per family; R1 uses an unresolved frozen tracker proxy, not physical tag/body GT. Other XY marker colors = observed tag error; R5 gyro-input validation remains OPEN. Lines identify methods. No alignment. R3 TEST after 16.679 s; observed gaps broken, censored tracks absent. Large excursions keep full scale with GT-neighborhood insets.')
    # F03: actual strong joint raw fault, all first emissions and native receipts.
    fig,axs=sub(4,1,7.0)
    for c in data['curves'].values():
        if c['stream']!='SC8-J' or not c['run'].startswith('E03'):continue
        ls='--' if c['view']=='S-FINAL' else '-'
        for ax,key in [(axs[0,0],'ape'),(axs[1,0],'are')]:ax.plot(c['t'],c[key],color=color(c['view']),ls=ls,lw=.9,label=c['view'])
    diag=[(rid,d) for rid,d in data['native_diagnostics'].items() if rid.startswith('E03') and 'O-FULL' in rid]
    if diag:
        rid,d=diag[0];t=np.asarray(d['t']);axs[2,0].plot(t,d['imu_margin_0'],color=COLORS['O-FULL'],label='Absolute margin')
        axs[2,0].plot(t,d['temporal_margin_0'],color=COLORS['O-FULL'],ls='--',label='Adjacent margin');axs[2,0].axhline(0,color='#aa3333',lw=.8)
        if data.get('joint_raw_range'):
            raw=data['joint_raw_range'];range_ax=axs[2,0].twinx();range_ax.plot(raw['t'],raw['error_m'],color='#D55E00',lw=.8,label='Anchor range error');range_ax.set_ylabel('Range error (m)')
            handles,labels=axs[2,0].get_legend_handles_labels();h2,l2=range_ax.get_legend_handles_labels();axs[2,0].legend(handles+h2,labels+l2,ncol=3,fontsize=8,loc='upper right')
        receipt_rows=[('imu_alarm','IMU FD'),('excluded','UWB isolation'),('bridge','Bridge'),('repair','Prior repair'),('rejoined','Rejoined state')]
        for j,(key,label) in enumerate(receipt_rows):
            v=np.asarray(d[key]);ix=v>=0 if key=='excluded' else v==1
            axs[3,0].scatter(t[ix],np.full(ix.sum(),j),s=8,color=COLORS['O-FULL'],marker='|')
        axs[3,0].set(yticks=range(len(receipt_rows)),yticklabels=[label for _,label in receipt_rows])
    else:no(axs[2,0]);no(axs[3,0])
    for ax in axs[:,0]:ax.axvspan(6.005,6.505,color='#cc6677',alpha=.15);ax.axvspan(6.05,6.20,color='#bb8800',alpha=.15);ax.set_xlim(0,30)
    for ax,label in zip(axs[:,0],['Tag APE (m)','Body ARE (rad)','Accel FD margin\n(m/s²)','Native receipt']):ax.set_ylabel(label)
    axs[0,0].set_yscale('symlog',linthresh=.1)
    axs[0,0].set_ylim(bottom=0)
    axs[0,0].set_ylabel('Tag APE (m; symlog)')
    axs[-1,0].set_xlabel('Sensor time (s)');axs[0,0].legend(ncol=4,fontsize=8)
    finish(fig,'F03_joint_fault_receipts','T-SIM SC8-J seed 20261020; red: raw accel 20 m/s² for 500 ms; amber: range +2.25 m for 3 epochs. APE is linear below 0.1 m, logarithmic above; rejoined flag is a persistent state. Receipts do not imply accuracy recovery.')
    # F04: paired independent gains retain negative values and posture effects.
    fig,axs=sub(1,3,4.5);pp=[p for p in metrics.get('paired',[]) if p['group']=='E03'];labels=list(dict.fromkeys(p['stream'] for p in pp))
    for ax,key,unit,scale in [(axs[0,0],'ape_m','APE gain (m)',1),(axs[0,1],'ape_m','APE gain (%)',1),(axs[0,2],'are_rad','ARE gain (rad)',1)]:
        for i,label in enumerate(labels):
            rr=[p for p in pp if p['stream']==label]
            for j,p in enumerate(rr):
                gains=p.get('gains',{}).get(key,{});v=gains.get('percent' if ax is axs[0,1] else 'off_minus_on')
                if v is not None:ax.plot(v,i+(j-(len(rr)-1)/2)*.13,'o',ms=4,color='#0072B2' if p['group']=='E03' and label.startswith('SC8') else '#009E73')
            if not any(p.get('gains',{}).get(key,{}).get('off_minus_on') is not None for p in rr):ax.text(.03,i,'NA',transform=ax.get_yaxis_transform())
        ax.axvline(0,color='black',lw=.8);ax.set(yticks=range(len(labels)),yticklabels=labels if ax is axs[0,0] else [],xlabel=unit);ax.invert_yaxis()
    counts=[]
    for stream in ['SC8-U','SC8-A','SC8-G','SC8-J']:
        episodes=[e for r in records if r['group']=='E03' and r['stream']==stream and r['requested_method']=='O-FULL' for e in r.get('episodes',[])]
        imu=[e for e in episodes if e['kind']=='IMU' and e.get('exposure')=='FULL_FAULT_INTERVAL_CONFIRMED'];uwb=[e for e in episodes if e['kind']=='UWB' and e.get('exposure')=='FULL_FAULT_INTERVAL_CONFIRMED']
        counts.append(stream.replace('SC8-','')+': '+('rejoin '+str(sum(e.get('REJOIN_EXECUTED',False) for e in imu))+'/'+str(len(imu)) if imu else '')+('; ' if imu and uwb else '')+('UWB isolation '+str(sum(e.get('ISOLATION_COMMITTED',False) for e in uwb))+'/'+str(len(uwb)) if uwb else ''))
    finish(fig,'F04_fault_paired_gains','Positive = OFF minus ON improvement; each seed / recording shown. Position gain does not cancel ARE degradation. FULL native episode receipts (fully exposed): '+', '.join(counts)+'. Real physical gains require a verified reference and two common post-injection states; otherwise NA. Walk tracker reference is OPEN. Engineering receipts have no equivalent qualified rejoin claim.')
    # F05 H/V actual binned Stanford counts, full scale and zoom.
    for axis in ['H','V']:
        fig,axs=sub(2,2,6.0);full_extent=max([v[axis]['full']['y_edges'][-1] for v in data['stanford'].values()],default=10)
        image=None
        max_count=max([max(np.asarray(v[axis][vw]['layers']['timely']).ravel()) for v in data['stanford'].values() for vw in ['full','zoom']],default=2)
        for i,method in enumerate(['O-FULL','O-NOFDE']):
            ad=data['stanford'].get(method,{}).get(axis)
            for j,view in enumerate(['full','zoom']):
                ax=axs[i,j]
                if not ad:no(ax);continue
                h=ad[view];xe=np.asarray(h['x_edges']);ye=np.asarray(h['y_edges']);timely=np.asarray(h['layers']['timely']);late=np.asarray(h['layers']['late']);mx=max(2,int(max_count))
                if timely.sum():image=ax.pcolormesh(xe,ye,np.ma.masked_where(timely.T==0,timely.T),norm=LogNorm(vmin=1,vmax=mx),cmap='viridis',rasterized=False)
                if late.sum():ax.pcolormesh(xe,ye,np.ma.masked_where(late.T==0,late.T),norm=LogNorm(vmin=1,vmax=max(2,int(late.max()))),cmap='Greys',alpha=.55)
                extent=full_extent if view=='full' else ye[-1];xextent=full_extent if view=='full' else xe[-1];ax.plot([0,min(xextent,extent)],[0,min(xextent,extent)],'k--',lw=.8)
                for al in [2 if axis=='H' else 3,5,10]:
                    if al<=extent:ax.axhline(al,color='#aa3333',ls=':',lw=.7)
                    if al<=xextent:ax.axvline(al,color='#aa3333',ls=':',lw=.7)
                ax.set(xlim=(0,xextent),ylim=(0,extent),xlabel=f'{axis} PE (m)',ylabel=f'{axis} PL (m)',title=f'{method} / {view}')
                ax.text(.03,.97,f"N={h['total']}; late={int(late.sum())}\noutside view={h['outside']}",va='top',transform=ax.transAxes,fontsize=8)
        if image is not None:fig.colorbar(image,ax=axs.ravel().tolist(),label='Actual timely count per bin (log)',shrink=.65)
        finish(fig,'F05_'+axis+'_Stanford','Conditional T-SIM only, E01/E03 main cases. Gray = late spatial bounds; refusal/missing denominators in metrics. Zero HMI samples do not prove risk.')
    # F06 AL sensitivity and actual SI empirical distributions.
    fig,axs=sub(1,2,3.8);als=[(2,3),(5,5),(10,10)]
    for method in ['O-FULL','O-NOFDE']:
        rr=[r for r in records if r['group'] in ['E01','E03'] and r['stream'] in ['SC8-H','SC8-U','SC8-A','SC8-G','SC8-J','SC8-D0','SC8-G2'] and r['requested_method']==method and 'axes' in r.get('integrity',{})]
        den=sum(r['integrity']['planned'] for r in rr)
        for kind,ls in [('H','-'),('V','--'),('joint',':')]:
            yy=[]
            for h,v in als:
                n=sum((r['integrity']['joint_AL'][f'{h}/{v}']['timely_within_AL'] or 0) if kind=='joint' else (r['integrity']['axes'][kind]['AL'][str(float(h if kind=='H' else v))]['timely_within_AL'] or 0) for r in rr)
                yy.append(100*n/den if den else np.nan)
            axs[0,0].plot(range(3),yy,ls=ls,marker='o',color=color(method),label=method+' '+kind)
        for axis,ls in [('H','-'),('V','--')]:
            s=data['si'].get(method+' '+axis,{})
            if s.get('n'):axs[0,1].plot(s['x'],s['cdf'],ls=ls,color=color(method),label=f"{method} {axis}; >1={s['over1']}/{s['n']}")
    axs[0,0].set(xticks=range(3),xticklabels=['2 / 3','5 / 5','10 / 10'],xlabel='Research H / V AL (m)',ylabel='Timely within-AL / planned (%)',ylim=(0,102));axs[0,0].legend(fontsize=8)
    axs[0,1].axvline(1,color='#aa3333',ls=':');axs[0,1].set(xlabel='SI = PE / PL',ylabel='Empirical CDF',xscale='symlog',ylim=(0,1.02));axs[0,1].legend(fontsize=8)
    axs[0,1].set_xscale('symlog',linthresh=.01);axs[0,1].set_xlim(left=0)
    finish(fig,'F06_AL_and_SI','T-SIM finite-run diagnostics; all planned epochs in availability denominator. SI axis is linear below 0.01, logarithmic above. AL changes only evaluation, never estimator or risk.')
    # F07 common 1101--2100 window frontier; censored runs have no imputed APE.
    fig,axs=sub(2,2,5.8);wr=[r for r in records if r['group']=='E05' and r.get('window')]
    window_ids={w:i+1 for i,w in enumerate([(30,30),(100,100),(300,300),(600,600),(1000,1000),(300,30),(300,100)])};pareto_seen=set()
    for fault,mark in [('SC8-WH','o'),('SC8-WJ','^')]:
        for coupled,ls in [(True,'-'),(False,'--')]:
            rr=sorted([r for r in wr if r['stream']==fault and ((r['window'][0]==r['window'][1]) if coupled else (r['window'][0]==300))],key=lambda r:r['window'][0] if coupled else r['window'][1])
            if not rr:continue
            xx=[r['window'][0] if coupled else r['window'][1] for r in rr];col='#0072B2' if fault=='SC8-WH' else '#D55E00';label=('Healthy' if fault=='SC8-WH' else 'Repeated fault')+(' coupled' if coupled else ' LF=300')
            def seq(fn):return [fn(r) if fn(r) is not None else np.nan for r in rr]
            def quality(ax,fn,complete,name,style):
                ax.plot(xx,[fn(r) if complete(r) else np.nan for r in rr],marker=mark,ls=style,color=col,label=name)
                for x,r in zip(xx,rr):
                    v=fn(r)
                    if v is not None and not complete(r):ax.plot(x,v,marker=mark,fillstyle='none',ls='None',color=col)
            quality(axs[0,0],lambda r:val(r),lambda r:r.get('window_common_coverage',{}).get('complete_pose_interval',False),label,ls)
            for key,style in [('hpl',ls),('vpl',':')]:
                quality(axs[0,1],lambda r,k=key:r.get('window_common_summary',{}).get(k,{}).get('mean'),lambda r,k=key:r.get('window_common_summary',{}).get(k,{}).get('n')==1000,label+' '+key[0].upper(),style)
            axs[1,0].plot(xx,seq(lambda r:r.get('timing',{}).get('core_ms',{}).get('p95')),marker=mark,ls=ls,color=col,label=label)
            for r in rr:
                if r['id'] in pareto_seen:continue
                pareto_seen.add(r['id'])
                latency95=r.get('timing',{}).get('core_ms',{}).get('p95');pl=r.get('window_common_summary',{}).get('hpl',{}).get('mean');rss=r.get('resources',{}).get('rss',{}).get('max')
                if latency95 is not None and pl is not None:
                    n=r.get('window_common_summary',{}).get('hpl',{}).get('n',0);partial=n<1000 or r['status']!='SUCCESS'
                    axs[1,1].scatter(latency95,pl,s=15+((rss or 0)/1024)*.05,facecolors='none' if partial else col,edgecolors=col,marker=mark)
                    axs[1,1].annotate(str(window_ids[tuple(r['window'])])+(f' n={n}' if partial else ''),(latency95,pl),xytext=(0,7 if fault=='SC8-WH' else -15),textcoords='offset points',fontsize=8)
    censored=[f"{tuple(r['window'])} {r['stream'][-1]}: {r.get('actual_max_epoch',0)}/2100" for r in wr if r['status']!='SUCCESS' or not r.get('views')]
    axs[0,0].set(xlabel='LF = LI, or LI at LF=300 (UWB epochs)',ylabel='Tag APE RMSE (m)',title='Common epochs 1101–2100');axs[0,0].legend(fontsize=8)
    axs[0,1].set(xlabel='Window (UWB epochs; 20 Hz)',ylabel='Mean common-interval PL (m)');axs[0,1].legend(fontsize=8)
    axs[1,0].axhline(40,color='#aa3333',ls=':');axs[1,0].set(xlabel='Window (UWB epochs)',ylabel='All measurable core P95 (ms)',yscale='log');axs[1,1].set(xlabel='All measurable core P95 (ms)',ylabel='Common-interval HPL (m)',xscale='log')
    for ax in [axs[0,0],axs[0,1],axs[1,1]]:ax.set_ylim(bottom=0)
    axs[0,0].legend(fontsize=8,loc='lower right');axs[0,1].legend(fontsize=8,loc='lower right')
    axs[1,1].text(.02,.05,'Pareto IDs (LF, LI):\n1:(30,30)  2:(100,100)\n3:(300,300)  4:(600,600)\n5:(1000,1000)\n6:(300,30)  7:(300,100)',transform=axs[1,1].transAxes,va='bottom',fontsize=8)
    if censored:axs[1,1].text(.02,.98,f'{len(censored)}/{len(wr)} stopped/refused or missing common interval.\nActual epochs and resource limits: report E05.\nNo missing APE / PL values imputed.',transform=axs[1,1].transAxes,va='top',fontsize=8)
    finish(fig,'F07_window_frontier','Offline frontier; 100–1000 epochs = 5–50 s. Quality lines require all 1000 common epochs; open points show partial intervals or censored Pareto estimates, labelled n. Bubbles use sampled RSS, not the unknown true peak. No extrapolation; replay changes with LF.')
    # F08 full-boundary latency CDF and exclusive means (not summed P95s).
    fig,axs=sub(3,2,8.0)
    for ax,phase,kind in [(axs[0,0],'Healthy','core'),(axs[0,1],'Fault','core'),(axs[1,0],'Real','core'),(axs[1,1],'Healthy','sink'),(axs[2,0],'Fault','sink')]:
        for key,d in data['latency'].items():
            ph,method,metric_kind=key.split('|')
            if ph!=phase or metric_kind!=kind or not d['n']:continue
            ax.plot(d['x'],d['cdf'],color=color(method),label=f"{method} N={d['n']}")
        ax.axvline(40 if kind=='core' else 50,color='#aa3333',ls=':',lw=.9);ax.set(title=f'{phase}: {kind}',xlabel='Latency (ms)',ylabel='Empirical CDF',xscale='symlog',ylim=(0,1.02));ax.legend(fontsize=8)
    ax=axs[2,1];groups=[];breakdowns=[]
    for stream,label in [('SC8-H','Healthy'),('SC8-J','Joint fault')]:
        rr=[r for r in records if r['stream']==stream and r['group'] in ['E01','E03'] and r['requested_method']=='O-FULL' and r.get('exclusive_means_ms')]
        if rr:
            keys=list(rr[0]['exclusive_means_ms']);groups.append(label);breakdowns.append({k:float(np.mean([r['exclusive_means_ms'].get(k,0) for r in rr])) for k in keys})
    bottom=np.zeros(len(groups));palette=plt.cm.tab20(np.linspace(0,1,6))
    merged={'Source':['source_ms'],'IMU + graph':['imu_fd_ms','graph_prepare_ms'],
            'Candidates':['candidate_ms'],'Proof':['proof_ms','finalize_ms'],
            'Commit + receipt':['commit_ms','state_receipt_ms'],
            'Cleanup + other':['maintenance_ms','envelope_cleanup_ms','abort_guard_and_clock_overhead']}
    if breakdowns:
        for i,(label,keys) in enumerate(merged.items()):
            vv=[sum(b.get(k,0) for k in keys) for b in breakdowns];ax.bar(groups,vv,bottom=bottom,label=label,color=palette[i]);bottom+=vv
        ax.set_ylim(0,max(bottom)*1.9)
    else:no(ax)
    ax.set(title='O-FULL exclusive mean; closes to total',ylabel='Mean service time (ms)');ax.legend(fontsize=8,loc='upper right')
    finish(fig,'F08_latency_and_breakdown','Every measurable slow/abort frame retained; ECDF is descriptive. SF software source clock ±1 ms plus jitter; root CPU/RTF separate from bag waits.')
    # F09 legal ablations; raw values, no mixed-window causal claim.
    fig=plt.figure(figsize=(180/25.4,8.0),constrained_layout=True)
    layout=fig.add_gridspec(3,2,height_ratios=[2.4,1.2,2.0])
    axs=np.array([[fig.add_subplot(layout[0,:]),fig.add_subplot(layout[1,:])],
                  [fig.add_subplot(layout[2,0]),fig.add_subplot(layout[2,1])]])
    rr=[r for r in records if r['group']=='E07'];labels=[];table=[];percent=[]
    comparison={c['id']:c for c in metrics.get('ablation_comparisons',[])}
    for r in rr:
        names={'O-ABL-SUPPORT':'support off','O-ABL-JOINT':'joint + support off','O-ABL-G2':'old numeric (80)'}
        labels.append(r['stream'].replace('SC8-','')+' / '+names.get(r['requested_method'],r['requested_method'])+('\nREFUSED: no qualified state' if r['status']=='FAIL' and not r.get('views') else ''))
        c=comparison.get(r['id'],{});a=c.get('A_common',{});pl=c.get('PL_common',{});p=c.get('P_common',{})
        raw=[a.get('ablated',{}).get('ape_m',{}).get('rmse'),a.get('ablated',{}).get('are_rad',{}).get('rmse'),pl.get('hpl',{}).get('ablated',{}).get('mean'),pl.get('vpl',{}).get('ablated',{}).get('mean'),p.get('ablated',{}).get('p95')]
        base=[a.get('full',{}).get('ape_m',{}).get('rmse'),a.get('full',{}).get('are_rad',{}).get('rmse'),pl.get('hpl',{}).get('full',{}).get('mean'),pl.get('vpl',{}).get('full',{}).get('mean'),p.get('full',{}).get('p95')]
        table.append(raw);percent.append([100*(x-y)/y if x is not None and y else np.nan for x,y in zip(raw,base)])
    ax=axs[0,0];arr=np.asarray(percent)
    if len(arr):
        finite=arr[np.isfinite(arr)];limit=max(1.,float(np.max(abs(finite)))) if len(finite) else 1.
        ax.imshow(arr,cmap='RdBu_r',vmin=-limit,vmax=limit,aspect='auto')
        for i in range(len(arr)):
            for j in range(5):ax.text(j,i,number(table[i][j])+('\n'+f'{arr[i,j]:+.1f}%' if math.isfinite(arr[i,j]) else '\nNA'),ha='center',va='center',fontsize=8)
        ax.set(yticks=range(len(labels)),yticklabels=labels,xticks=range(5),xticklabels=['APE\n(m)','ARE\n(rad)','HPL\n(m)','VPL\n(m)','P95\n(ms)']);ax.grid(False)
    else:no(ax)
    ax.set_title('Raw values + change from FULL (%)')
    ax=axs[0,1];main=[r for r in records if r['group'] in ['E01','E03'] and r['requested_method']=='O-FULL' and r['stream'] in ['SC8-D0','SC8-J'] and r.get('seed')==20261020]
    for i,r in enumerate(main):
        for j,axis in enumerate(['H','V']):
            m=r.get('same_center_M',{}).get(axis,{});v=m.get('reduction_m',{}).get('mean')
            if v is not None:ax.barh(i+j*.25,v,height=.2,color=COLORS['O-FULL'] if axis=='H' else '#56B4E9',label=axis if i==0 else None);ax.text(v,i+j*.25,f" {m.get('strict',0)} strict",va='center',fontsize=8)
    ax.set(yticks=np.arange(len(main))+.125,yticklabels=[r['stream'] for r in main],xlabel='Same-center old PL minus new PL (m)',title='M witness: same committed center');ax.legend()
    ax.margins(x=.25)
    for ax,j,title in [(axs[1,0],2,'Accel z bias envelope'),(axs[1,1],5,'Gyro z bias envelope')]:
        for d in data.get('bias_bounds',{}).values():
            width=np.asarray(d['bwidth_'+str(j)]);ref=np.asarray(d['reference_bwidth_'+str(j)]);label=d['stream'].replace('SC8-','')+' / '+('FULL' if d['method']=='O-FULL' else 'joint+support off')
            ax.plot(d['t'],width/ref,color=COLORS['O-FULL'] if d['method']=='O-FULL' else COLORS['O-NOFDE'],ls='-' if d['stream']=='SC8-D0' else ':',lw=1,label=label)
        ax.axhline(1,color='black',lw=.6,ls='--');ax.set(title=title,xlabel='Sensor time (s)',ylabel='Width / propagation reference');ax.legend(fontsize=8)
    finish(fig,'F09_ablations','Common A / PL / P; red = increase from FULL, blue = decrease. M uses the same center; G2 uses first 80 epochs. Bias ratios are actual raw widths: D0/S0 gain uses original sample0, not continued full-state observability. M3 is shown separately; S1/S2 trajectories were not registered.')
    # F10 long streams preserve absolute sensor time and planned suffix.
    fig,axs=sub(4,1,7.2)
    are_ax=axs[0,0].twinx();are_ax.set_ylabel('Body ARE (rad)',fontsize=8)
    slots_ax=axs[3,0].twinx();slots_ax.set_ylabel('Factor slots',fontsize=8)
    long=[(rid,d) for rid,d in data['native_diagnostics'].items() if rid.startswith('E08')]
    for rid,d in long:
        r=byid[rid];method=r['requested_method'];col=color(method);ls=':' if 'SC8-L0' in rid else '-';label=r['stream']+' '+method;t=np.asarray(d['t'])
        axs[0,0].plot(t,d['ape'],color=col,ls=ls,label=label)
        are_ax.plot(t,d['rotation_error'],color=col,ls='--',alpha=.4,lw=.7)
        axs[1,0].plot(t,d['hpl'],color=col,ls=ls,label=label+' H');axs[1,0].plot(t,d['vpl'],color=col,ls='--',alpha=.7,label=label+' V')
        roll=d['core_rolling_p95']
        axs[2,0].plot(t,roll,color=col,ls=ls,label=label)
        axs[3,0].plot(t,np.asarray(d['rss'])/1024,color=col,ls=ls,label=label)
        slots_ax.plot(t,d['slots'],color=col,ls='--',alpha=.45,lw=.7)
    for ax in axs[:,0]:
        ax.set_xlim(0,500)
        for f in registry['cases']['SC8-L1']['faults']:ax.axvline(f[0]*.05,color='#cc6677',lw=.5,alpha=.6)
    for ax,label in zip(axs[:,0],['Tag APE (m)','H / V PL (m)','20 emitted-state\nrolling core P95 (ms)','Resident memory (MiB)']):ax.set_ylabel(label)
    axs[0,0].legend(ncol=3,fontsize=8);axs[-1,0].set_xlabel('Absolute sensor time (s); no restart or risk reset')
    if not long:no(axs[0,0])
    finish(fig,'F10_long_streams','T-SIM L0=5000, L1=10000 planned epochs. Blank suffixes have no committed output; resource/receipt counts and ARE/bias/slots in report + plot data.')
    save(OUT/'SC8_FIGURES.json',dict(figures=figlog,plotdata_sha256=common.sha256(source),metrics_sha256=common.sha256(OUT/'SC8_METRICS.json')))
    print('FIGURES',len(figlog),'families, PNG + editable SVG',flush=True)


def overlay_effects(records,registry):
    index={e['id']:e for e in registry['runs']};out=[]
    for r in records:
        if r['group']!='E03' or not r['stream'].startswith(('R2-','R3-')) or not r.get('views'):continue
        base=next((b for b in records if b['group']=='E02' and b['stream']==r['stream'].split('-')[0] and b['requested_method']==r['requested_method'] and b.get('views')),None)
        if base is None:continue
        a,truth,lever=run_poses(r,index[r['id']]);b,_,_=run_poses(base,index[base['id']])
        for view,pose in a.items():
            if view not in b:continue
            grid=truth[0];mask=metric.sample(*pose,grid)[0]&metric.sample(*b[view],grid)[0];grid=grid[mask]
            if len(grid)<2:continue
            target=(grid,*metric.sample(*truth,grid)[1:]);sa=metric.evaluate((grid,*metric.sample(*pose,grid)[1:]),target,lever)[0];sb=metric.evaluate((grid,*metric.sample(*b[view],grid)[1:]),target,lever)[0]
            begin=r.get('overlay_execution_coverage',{}).get('physical_begin_s',float('inf'))
            delta={k:sa[k]['rmse']-sb[k]['rmse'] if sa[k]['rmse'] is not None and sb[k]['rmse'] is not None else None for k in ['ape_m','hpe_m','vpe_m','are_rad','rpe_tag_0.5s_m','rpe_body_0.5s_m']}
            reference_open=r.get('reference_contract')=='OPEN_WALK_TRACKER_BODY_TAG'
            out.append(dict(stream=r['stream'],view=view,base_id=base['id'],overlay_id=r['id'],common_n=len(grid),
                interval_s=[float(grid[0]),float(grid[-1])],delta_overlay_minus_original={k:None for k in delta} if reference_open else delta,
                reference_contract=r.get('reference_contract','REGISTERED_REFERENCE'),reference_proxy_delta=delta if reference_open else {},
                common_post_overlay_n=int(np.sum(grid>=begin)),accuracy_after_overlay_evaluable=bool(np.sum(grid>=begin)>=2 and not reference_open),
                delta_committed=r.get('committed',0)-base.get('committed',0) if r.get('committed') is not None and base.get('committed') is not None else None,
                delta_timely_estimates=r.get('timely_estimates',0)-base.get('timely_estimates',0) if r.get('timely_estimates') is not None and base.get('timely_estimates') is not None else None,
                scope='incremental effect on common observed interval; if common_post_overlay_n=0 accuracy describes only pre-overlay prefix. Original recording not certified healthy; coverage impact separate.'))
    return out


def report():
    metrics=json.loads((OUT/'SC8_METRICS.json').read_text());registry=yaml.safe_load((ROOT/'config/sc8_experiments.yaml').read_text());records=metrics['runs']
    f=registry.get('frozen',{});counts=metrics['logical_run_status_counts'];pending=sum(counts.get(k,0) for k in ['NOT_ATTEMPTED','RUNNING'])
    checks=metrics.get('validation',{}).get('final-checks.json',{}).get('result',[]);checked=bool(checks) and all(c['returncode']==0 for c in checks)
    gaps=[r['id'] for r in records if r['status']=='EXECUTION_ERROR' or 'scoring_error' in r]
    late=[r for r in records if r['group']=='E08' and r['stream']=='SC8-L1' and r['status'] not in ['NOT_ATTEMPTED','RUNNING'] and r.get('max_committed_epoch',0)<10000]
    if pending or not checked:status='IN_PROGRESS'
    elif gaps or late:status='SC8_EVALUATED_WITH_EXECUTION_GAPS'
    else:status='SC8_EVALUATION_COMPLETE_FOR_REGISTERED_MATRIX'
    status,all_gaps=evaluation_status(records,metrics.get('validation',{}))
    metrics['status']=status;metrics['execution_gaps']=all_gaps
    def fmt(v):return 'NA' if v is None or (isinstance(v,float) and not math.isfinite(v)) else f'{v:.6g}' if isinstance(v,(float,np.floating)) else str(v)
    def table(headers,rr):return '\n'+'\n'.join(['| '+' | '.join(headers)+' |','|'+'|'.join(['---']*len(headers))+'|']+['| '+' | '.join(fmt(c).replace('|','/') for c in row)+' |' for row in rr])+'\n'
    def m(r,key,stat='rmse',view=None):
        vv=r.get('views',{});view=view or next(iter(vv),'');return vv.get(view,{}).get(key,{}).get(stat)
    def timing(r,key,stat='p95'):
        x=r.get('timing',{}).get(key,{});return x.get(stat) if isinstance(x,dict) else None
    lines=['# SC8 实验结果与复现报告',f'\n**状态：{status}。**',
        '\n本轮实测值来自本次执行 journal；未复用历史 CSV。算法输、界较宽和大窗非实时均作为结果保留。进程 SUCCESS 只表示正常退出，功能、资格、deadline 和应用可用性分别列出。',
        '\n## 当前实测结论',
        '\n\n'.join(findings(metrics)),
        '\n## 身份、预算与证据边界',
        f"\n入口 HEAD `{registry['entry_head']}`；计算冻结提交 `{f.get('identity',{}).get('head','NA')}`。实际源内容/ELF/生产 DSO、loaded libraries、编译环境、平台、噪声、risk/birth q、AL、窗口 guard 和原合同摘要在 [冻结登记](../../config/sc8_experiments.yaml)；未升级 SFUISE、未 push。分析模块可在计算冻结后继续开发，不改变求解。",
        f"\n登记 {metrics['registered']} 个唯一逻辑运行；当前状态 {counts}，合计 {sum(counts.values())}。实际 attempt {metrics['attempt_processes']}，另有辅助 ROS 求解 {sum(x.get('processes',0) for x in registry.get('auxiliary_solver_audits',[]))}，上限 160。W0 额外 attempt（包括预检失败）全部保留（版本状态与实际 exit code 分开，见 JSON），SFUISE 首发/历史共用一个进程。E06 两次确认不增加独立精度样本。",
        '\n- **T-SIM**：当前注册物理 law 下的条件 raw 保证；资格与实际数值/receipt 逐帧核验，仍无真实应用资格。\n- **T-REAL**：原始真实录制上的经验表现；SC7 raw producer 拒绝，不放宽采样率/噪声/运动域/anchor guard。requested O-FULL 实际明确为 **O-FDE-ENG**，off 为 **O-ENG-NOFDE**，均不冒充 SC7 恢复/PL。\n- **T-SEMI**：R2/R3 原始输入上叠加登记故障，GT 不变；原录制本身未认证健康，不将天然异常称已知 false alarm。',
        '\nSFUISE 固定 SHA `75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d`，官方测距剔除保持。插入式只读数值/时间观察与 lossless Float64 ToA wire 单独披露：内部仍为原 TOAData double，剥离插入后原数值语句逐字节相同；双实例确定性检查覆盖状态、标定、knot 与官方剔除状态。原始 float32 wire 会改变量测，故正式流使用与 ours 逐值一致的 double；不是改变噪声/残差/优化器。',
        '\n## E00：数据、CAL/TEST 与因果接口',
        '\n九条输入新做单位/frame/lever/时间/GT/anchor/range 审计。六条测试按数据族预先选择，不按算法精度删序列；本地库存均存在，因此不启用 MILUV 备用；HUEC 和历史 40 条 campaign 未全跑。',
        table(['ID','序列','IMU Hz','时长 s','最大 IMU gap s','anchor/tag','最大测距组跨度 s'],[(x['id'],x['sequence'],x['imu_hz'],x['imu_duration_s'],x['imu_max_gap_s'],f"{x['anchors']}/{x['tags']}",x['batch']['max_span_s']) for x in registry['inventory']]),
        '\nWalk 世界变换只用 CAL-W Walk1：844 个 fit 样本，残差 RMSE 0.178204 m。CAL-O 15-48-55 激励不足（gyro 激励奇异值约 0.103/0.0217/0.0144），没有把它宣称独立完整外参标定。固定 R3 第一 20%（截至 14.678774643 s）用于接口拟合，另隔离 2 s，主 TEST 自 16.678774643 s 开始；该族为 **GT-assisted / test-involved segmented calibration / CALIBRATION_OPEN**。固定拟合时移 0.0155421 s、gyro RMSE 0.147482 rad/s、accel RMSE 0.287109 m/s²；不表示完整 6D 外参独立可辨识。STAR 使用官方几何/外参，未用 R5/R6 GT 再拟合。全部数字参数保存在 registry，不逐方法选对齐或时移。',
        '\n原始每条 range 时间保留，公共组末时间用于两种估计器的批处理近似，STAR 20 ms 异步组不称同步、不给 raw 资格。range_raw→calibrated range 只应用一次，SFUISE runtime offsets=0。GT 只进预登记 CAL 和评分，不给真状态初值、不选镜像、不发 truth topics。仿真 ours 用同一独立含噪初始观测；SFUISE 保留原生传感器初始化，不消费这一额外 prior，因此两者不是纯优化器隔离对照。',
        '\n原生 source-only 导出与正式求解使用同一 ELF，逐值验证真实 ADC/range/时间/ID/独立初始观测。W0 曾发现两个 C++ 入口模板实例约 1e-15 的差异，正式缓存按 native-export-v1 独立留存；旧 W0 不悄悄换输入。SFUISE 静态 map readiness 统一为 max(首 IMU,首合法 range)+25 ms，保留原 sensor 时间和值，等待计入延迟。R6 早期 SIGSEGV 及旧 float32 协议、wrong-lever 工程诊断均保留为失败/被替代版本，不计作最终方法结果。',
        '\n修正 actual realtime 杆臂后，新 direct/ROS fde=off R3 前缀：899/899 样本，tag 最大差 2.5855843e-10 m、姿态最大差 1.7075473e-6 deg（均小于 1e-5）；GT topic 发布 0，truth subscriptions 为空。20× 离线回放仅验证提交路径等价，不作为实时依据。',
        table(['W0 流','requested','effective','进程状态','wall s'],[(r['stream'],r['requested_method'],r.get('effective_method'),r['status'],r.get('wall_s')) for r in records if r['group']=='E00']),
        '\n## 评分、配对和不确定性口径',
        '\n主 APE/HPE/VPE 在实际首发 estimate stamp 对同物理 tag/world 的 GT 插值，ARE 是 body SO(3) 测地角（rad，主表同时给 deg）；不拟合 SE(3)、yaw 或测试时延。tag difference RPE 和减去固定杆臂后的 body SE(3) RPE 分别给 0.5/1 s，任何输入/GT gap>100 ms 的对均拒绝。RPE 的公共 GT 网格与配对表的有限间隔插值只用于评价首次输出流，不改因果输出、不使用最终历史替首发。SFUISE S-FINAL 是同一次运行在同一在线 stamp 网格的可验证子集上由最终 retained history + 最终标定导出，不是全局 batch 最优；artifact 级 revision 与最后输入上界独立记录，不冒充逐点精确依赖。',
        '\n原生旧 W0 联合故障前缀与正式 J600 的前160：raw ADC/range/独立初始观测、IDs/时间、state/PL 和 FD/先验/bridge/rejoin receipts 逐值相同（差0），不增加求解或精度重复样本。独立评分器 13 项测试与 evo v1.29.0 对口径检查覆盖单位、符号、SO(3)、tag/body 杆臂、错误杆臂、固定偏置、SE(3) 世界变换、错序/重复/非有限、真实 dt、缺口与 nearest-rank 慢值。资源截断产生的不完整末行拒绝；完整 post-commit state 行只作固定物理 tag 转换并与完整 TUM 逐点核对，不从 GT 重建。真实 velocity/bias 无独立真值为 NA；仿真原生有物理状态/bias 真值，单列评分。SFUISE 未导出 velocity/bias 真值对口径时也不凭位置差分伪造。',
        '\n历史导出的额外边界审计：冻结适配器将移动窗口的 idle 控制点写入 global history，导致起始 `t < global_origin + 2*dt` 的历史插值引用后续控制点。用原始 SplineState 类、不构造估计器的确定性 codec 检查：在线 p/q/velocity/bias 复原误差 0，安全域后历史误差 0，起始错误可直接复现。原始输出未改写，其 hash、被拒绝点的原始数值和时间永久留在 JSON；过程 trace 最后按约定清理。按样条支撑域对所有流统一拒绝这一历史前缀（通常 2 个点），不是按 PE 选点。S-ONLINE 全部首次输出不受影响；S-FINAL 仅报告可验证历史域，拒绝数、原始错误点/时间保存在 JSON，作为导出接口缺口列出。',
        '\n独立统计单位是 seed 或录制，三个 seed 不夸大显著性；逐帧 Stanford/ECDF 仅是有限运行描述，不作独立置信区间。配对收益为 OFF−ON，百分比用 OFF 作分母，退化保持负数；不同截止时刻的 SFUISE 另列 state age，S-FINAL 不排实时名次。过程被截断时不填后缀误差，配对表显式给公共可观测区间。',
        '\n## E01/E02：健康与真实主对照',
        '\n**Walk 参照缺口（W1 后审计修正）**：冻结适配器把 VIVE tracker 当成 IMU/body 并直接加 IMU→tag 杆臂。CAL-W 968 对角速度的单位旋转 RMSE 0.485762 rad/s，固定旋转诊断拟合为约 179.725°、RMSE 0.105943 rad/s，推翻了 tracker=IMU 假设；bag 无静态外参，tracker→tag 平移/姿态与重力系仍未独立闭合。先前 Walk 表中的数值不再称物理 tag APE/body ARE，主表改为 NA，原数值完整留为 proxy。没有将拟合旋转应用到输入、GT 或估计器，也没有修改冻结参数。论文仅确认额外 tracker 提供 GT，不给本轮所需的完整外参：[原论文 §V-D](https://isas.iar.kit.edu/pdf/RAL23_Li.pdf)。',
        '\n**STAR 输入接口缺口**：发布的 tf_cam_rig / tf_cam_imu 方向标签与冻结组合 C_cam_rigᵀ C_cam_imu 存在解释分歧。只读 CAL-S 检查中，冻结 gyro 与 GT 差分 RMSE 约 0.393 rad/s；按文字方向组合的候选约 0.324 rad/s，仍未闭合，故没有悄悄替换或拟合 TEST。官方 tag_pos、rig 四元数与 CAD 杆臂自洽到约 4e−16 m，位置误差仍是实际管线输出误差；R5/R6 均标 input OPEN，不将发散归因于正确输入下算法本身。细节和两种矩阵见 reference-check 收据。',
        table(['Walk 数值代理（不作物理精度排名）','view','时间匹配 N','冻结 proxy 距离 RMSE m','tracker proxy 四元数差 rad'],[(r['group']+'/'+r['stream'],v,p.get('matched'),p.get('ape_m',{}).get('rmse'),p.get('are_rad',{}).get('rmse')) for r in records if r.get('reference_contract')=='OPEN_WALK_TRACKER_BODY_TAG' and r['group']!='E00' for v,s in r.get('views',{}).items() for p in [s.get('reference_proxy_scores',{})]]),
        table(['组/流/seed','view','status','匹配 N','APE m','HPE m','VPE m','ARE rad / deg','tag RPE0.5 m','body RPE0.5 m / rad'],[(r['group']+'/'+r['stream']+'/'+str(r.get('seed',''))[-2:],v,r['status'],s.get('matched'),s.get('ape_m',{}).get('rmse'),s.get('hpe_m',{}).get('rmse'),s.get('vpe_m',{}).get('rmse'),f"{fmt(s.get('are_rad',{}).get('rmse'))} / {fmt(s.get('are_deg',{}).get('rmse'))}",s.get('rpe_tag_0.5s_m',{}).get('rmse'),f"{fmt(s.get('rpe_body_0.5s_m',{}).get('rmse'))} / {fmt(s.get('rpe_body_0.5s_rad',{}).get('rmse'))}") for r in records if r['group'] in ['E01','E02'] for v,s in r.get('views',{}).items()]),
        table(['实测流/method','attempt / planned','commit / timely','未覆盖/资源后缀','原生 FDE 状态计数'],[(r['stream']+'/'+str(r.get('effective_method',r['requested_method'])),f"{r.get('attempted_epochs','NA')} / {r.get('planned_epochs','NA')}",f"{r.get('committed','NA')} / {r.get('timely_estimates','NA')}",r.get('startup_or_unattempted_epochs'),r.get('fde_status_counts',{'SFUISE':'official rejection; no matched raw PL'})) for r in records if r['group']=='E02']),
        table(['实测 view','整个 GT TEST 网格覆盖 / N','实际输出时间区间 s'],[(r['stream']+'/'+v,f"{fmt(s.get('output_coverage_on_GT_TEST_grid',{}).get('fraction'))} / {s.get('output_coverage_on_GT_TEST_grid',{}).get('planned','NA')}",s.get('output_coverage_on_GT_TEST_grid',{}).get('estimate_interval_s')) for r in records if r['group']=='E02' for v,s in r.get('views',{}).items()]),
        '\n主表是每方法实际 stamp 指标。ON/OFF 收益只用两条首次流的公共区间；F01 的对照点另用所有可用视图（含有效历史域）的共同网格，缺方法明确为 NA；任一方法缺完整 TEST 输出时公共表保留 NA，不将短前缀性能当完整录制性能。',
        table(['流/seed','公共区间 s','公共 N','APE 改善 m / %','ARE 改善 rad','输入 digest 一致'],[(p['stream']+'/'+str(p['seed'])[-2:],p['common_interval_s'],p['common_grid_n'],f"{fmt(p['gains'].get('ape_m',{}).get('off_minus_on'))} / {fmt(p['gains'].get('ape_m',{}).get('percent'))}",p['gains'].get('are_rad',{}).get('off_minus_on'),p['input_digests_equal']) for p in metrics.get('paired',[]) if p['group'] in ['E01','E02']]),
        table(['SF 同次运行历史对照','共同有效 N','首次 / 历史 APE m','首次 / 历史 ARE rad'],[(p['stream']+'/'+str(p['seed'])[-2:],p['SF_history_common']['common_n'],f"{fmt(p['SF_history_common']['scores'].get('S-ONLINE',{}).get('ape_m',{}).get('rmse'))} / {fmt(p['SF_history_common']['scores'].get('S-FINAL',{}).get('ape_m',{}).get('rmse'))}",f"{fmt(p['SF_history_common']['scores'].get('S-ONLINE',{}).get('are_rad',{}).get('rmse'))} / {fmt(p['SF_history_common']['scores'].get('S-FINAL',{}).get('are_rad',{}).get('rmse'))}") for p in metrics.get('paired',[]) if p.get('SF_history_common')]),
        '\n## E03：故障、恢复与实测增量',
        '\n主流 U/A/G/J 各三 seed、600 历元；健康 F0 复用 E01。登记强度保持 range +2.25 m/3 历元、raw accel 20 m/s² 或 gyro 1 rad/s/500 ms，标准噪声 draw 与初始观测不变；另有 D0 六轴、弱短、负、错峰定向例，不全排列。真实 overlay 公共交集若不足两个注入后状态，故障 gain 为 NA，故障前数值另留 JSON，不以零差异冒充恢复效果。R2/R3 仅 FU/FG/FJ；注入 ORIGINAL_SENSOR_X 经固定旋转进入 body，保留 raw sample/range IDs、原时刻、delta 与 GT unchanged，不宣称这一未认证 body 混合方向满足单轴 raw 定理。',
        table(['流/seed','公共 / 注入后公共 N','APE OFF−ON m','APE %','ARE OFF−ON rad'],[(p['stream']+'/'+str(p['seed'])[-2:],f"{p['common_grid_n']} / {fmt(p.get('post_fault_common_n'))}",p['gains'].get('ape_m',{}).get('off_minus_on'),p['gains'].get('ape_m',{}).get('percent'),p['gains'].get('are_rad',{}).get('off_minus_on')) for p in metrics.get('paired',[]) if p['group']=='E03']),
        table(['E03 主流','view','独立单位 N','APE mean / min / max m','ARE mean / min / max rad'],[(a['stream'],a['view'],a['metrics']['ape_m']['n_units'],' / '.join(fmt(a['metrics']['ape_m'][k]) for k in ['mean','min','max']),' / '.join(fmt(a['metrics']['are_rad'][k]) for k in ['mean','min','max'])) for a in metrics.get('unit_aggregates',[]) if a['group']=='E03' and a['stream'] in ['SC8-U','SC8-A','SC8-G','SC8-J']]),
        table(['定向例','status','APE m','ARE rad','raw / planned'],[(r['stream'],r['status'],m(r,'ape_m'),m(r,'are_rad'),f"{r.get('integrity',{}).get('spatial_valid','NA')} / {r.get('planned_epochs','NA')}") for r in records if r['group']=='E03' and r['stream'] not in ['SC8-U','SC8-A','SC8-G','SC8-J'] and r['stream'].startswith('SC8-')]),
        '\n以下为真实原生 episode receipt，FD/隔离/边缘先验替换/bridge/rejoin 分开；sensor TTD 与 wall TTD/TTI/TTR 不混。仅有 FD 或 PL 资格不等于恢复执行；first nominal receipt 不等于误差恢复。阶段 APE/ARE/速度/bias（healthy/fault/bridge/rejoin/late）完整保存在 JSON，位置改善不抵销姿态退化。',
        table(['流/seed/method','episode / exposure','FD','隔离提交','prior / bridge','rejoin4','TTD sensor / wall ms','TTI / TTR wall ms','unsafe endpoint'],[(r['stream']+'/'+str(r.get('seed',''))[-2:]+'/'+r['requested_method'],e['kind']+'/'+str(e.get('axis',e.get('anchor_index')))+'@'+str(e['onset_epoch'])+' / '+e.get('exposure','NA'),e.get('FD'),e.get('ISOLATION_COMMITTED'),f"{e.get('first_prior_repair')} / {e.get('first_bridge')}",e.get('rejoin4'),f"{fmt(e.get('TTD_sensor_ms'))} / {fmt(e.get('TTD_wall_ms'))}",f"{fmt(e.get('TTI_wall_ms'))} / {fmt(e.get('TTR_wall_ms'))}",e.get('unsafe_shared_endpoint')) for r in records if r['group'] in ['E03','E08'] for e in r.get('episodes',[])]),
        '\n以下阶段按每个 seed 的 RMSE 等权汇总；rejoin 是恢复后的状态，可与 late 重叠，不是互斥完整性分区。未执行 bridge/rejoin 的 OFF 阶段为 NA。每个 seed 原值和帧 N 留在 JSON。',
        table(['主流/method/阶段','seed N / receipt N','APE mean [min,max] m','ARE mean [min,max] rad','v mean m/s','ba / bg mean m/s² / rad/s'],[(a['stream']+'/'+a['method']+'/'+a['phase'],f"{a['metrics']['ape']['n_units']} / {a['receipts']}",f"{fmt(a['metrics']['ape']['mean'])} [{fmt(a['metrics']['ape']['min'])},{fmt(a['metrics']['ape']['max'])}]",f"{fmt(a['metrics']['rotation_error']['mean'])} [{fmt(a['metrics']['rotation_error']['min'])},{fmt(a['metrics']['rotation_error']['max'])}]",a['metrics']['velocity_error']['mean'],f"{fmt(a['metrics']['accel_bias_error']['mean'])} / {fmt(a['metrics']['gyro_bias_error']['mean'])}") for a in metrics.get('phase_aggregates',[])]),
        table(['真实 overlay/method','实际注入前沿确认'],[(r['stream']+'/'+str(r.get('effective_method',r['requested_method'])),r.get('overlay_execution_coverage')) for r in records if r['group']=='E03' and r['stream'].startswith(('R2-','R3-'))]),
        table(['真实 overlay','view','公共 / 注入后公共 N','增量 APE m','增量 ARE rad / 注入后可评'],[(e['stream'],e['view'],f"{e['common_n']} / {e.get('common_post_overlay_n',0)}",e['delta_overlay_minus_original'].get('ape_m'),f"{fmt(e['delta_overlay_minus_original'].get('are_rad'))} / {e.get('accuracy_after_overlay_evaluable')}") for e in metrics.get('overlay_effects',[])]),
        '\n## E04：原生完整性、Stanford 与 AL',
        '\nF05-H/V 使用同 stamp/tag/receipt 的实际 PE（横）/PL（纵）计数，不由均值合成。主图 full 尺度与 5 m zoom 均留 overflow；灰色 late bounds 不算及时可用；SFUISE/真实未知 PL 不画同 epsilon 点云。2/3、5/5、10/10 m 仅为研究 AL 后验评价，不改变在线噪声、risk、q 或发布规则。每轴互斥分区和 underbound 另外计数，AL-unavailable 中的 underbound 也不删除。零 HMI 样本不构成概率证明。',
        table(['流/seed/method','planned / committed / raw / timely','missing / refused','H under / V under','joint timely AL2/3 / 5/5 / 10/10','publication protected'],[(r['stream']+'/'+str(r.get('seed',''))[-2:]+'/'+r['requested_method'],f"{i['planned']} / {i['committed']} / {i['spatial_valid']} / {i['timely_spatial']}",f"{i['missing']} / {i['refused']}",f"{i['axes']['H']['underbound']} / {i['axes']['V']['underbound']}",' / '.join(str(i['joint_AL'][k]['timely_within_AL']) for k in ['2/3','5/5','10/10']),i['publication_protected']) for r in records if r['group'] in ['E01','E03'] and r['stream'] in ['SC8-H','SC8-U','SC8-A','SC8-G','SC8-J','SC8-D0','SC8-G2'] and 'axes' in (i:=r.get('integrity',{}))]),
        '\n## E05：窗口成本前沿',
        '\n合法 7 配置 × 两个 2100 历元流；联动 30/100/300/600/1000 与固定 LF=300 的 LI=30/100/300 分开。100–1000 是真实 UWB 历元（5–50 s），不能把 IMU 条数叫窗口。公共评分 1101–2100、故障 1201/1601。表中另给公共提交/有效界 N；截断公共子段可描述但不连成完整区间收益趋势，未确认到达故障不能声称处理了该事件。4 个大窗 canary 是同一健康运行的 LF+2，不重启、不清零风险；必须观察实际 x0 退出/边缘化。预登记 RSS、1200 s wall 和连续 20 个 core>2 s 截断，保留 inflight lower bound，右截断不冒充完整 P95。',
        table(['流 / LF,LI','status','actual / planned','common commits / valid bounds','common APE m','common mean H/V m','P95 core ms','receipt RSS max MiB','x0 retired / resource'],[(r['stream']+' '+str(r.get('window')),r['status'],f"{r.get('actual_max_epoch','NA')}/2100",f"{r.get('window_common_coverage',{}).get('committed',0)} / {r.get('window_common_coverage',{}).get('valid_bounds',0)}",m(r,'ape_m'),f"{fmt(r.get('window_common_summary',{}).get('hpl',{}).get('mean'))} / {fmt(r.get('window_common_summary',{}).get('vpl',{}).get('mean'))}",timing(r,'core_ms'),r.get('resources',{}).get('rss',{}).get('max',0)/1024 if r.get('resources') else None,f"{r.get('x0_retired')} / {r.get('resource_observation')}") for r in records if r['group']=='E05']),
        table(['同进程 canary run','LF+2 实际 receipt'],[(r['id'],r.get('canary')) for r in records if r['group']=='E05' and 'canary' in r]),
        '\n联动结果同时改变 FGO/replay 与 PL 历史，不能将 APE 变化只归因于完整性窗口。扩展窗不要求 40/50 ms，通过或不通过不是调整 guard 的理由；未到公共区间的 APE/PL 为 NA，不按短窗外推 1000。',
        '\n## E06：时间、CPU、RTF 与默认 profile',
        '\ncore 从原始到达验证开始到 cleanup/receipt 完成；sink 含 ready/排队/载荷/接收门，不含 GT 评分。原生 source 资格计算属于 core，独立 source 生成与冷启动另列；独立 producer 按绝对 source 时间发布，初始化需等待真实 2 s 前缀到达，积压不免费。SFUISE core 包含 callback 与两次发布之间所有 active run 开销，CPU/RSS 来自 root solver；整进程 bag/start/drain wall 不作为 core 或算法加速比。SF 软件 clock 仿射映射在 EOF 后继续计时，1 ms 发布粒度与软件 jitter 未提供硬件实时资格。数值输入 cutoff 与仅 buffered callback cutoff 分列。',
        table(['组/流/seed/method','measured core N','P95 core / sink / age ms','P99 core / sink ms','root CPU s','CPU RTF / service RTF / processing RTF','RSS KiB','timely fraction / profile'],[(r['group']+'/'+r['stream']+'/'+str(r.get('seed',''))[-2:]+'/'+str(r.get('effective_method',r['requested_method'])),r.get('timing',{}).get('core_ms',{}).get('n'),f"{fmt(timing(r,'core_ms'))} / {fmt(timing(r,'measurement_to_sink_ms'))} / {fmt(timing(r,'state_age_ms'))}",f"{fmt(timing(r,'core_ms','p99'))} / {fmt(timing(r,'measurement_to_sink_ms','p99'))}",r.get('root_cpu_s'),f"{fmt(r.get('CPU_RTF'))} / {fmt(r.get('service_RTF'))} / {fmt(r.get('processing_RTF'))}",r.get('peak_rss_kib'),f"{fmt(r.get('timely_spatial_fraction',r.get('timely_estimate_fraction')))} / {r.get('tested_profile_deadline_pass','NA: no matched raw PL')}") for r in records if r['group'] in ['E01','E02','E06']]),
        '\n默认 positive profile 必须同时 core P95≤40 ms、measurement-to-sink P95≤50 ms、及时空间有效/所有计划≥95%；所有测到的慢帧、失败候选和 abort 包含。真实只报及时估计率，不借用 raw 空间资格；formal attempt、启动未覆盖、资源停止后未尝试后缀均在全分母。exclusive 模块均值与总 core 均值逐 run 闭合，未分配的 guard/clock/abort 成本单列；阶段 P95 不相加。CPU、wall processing 与 sensor duration 分别记录，不声称全线程实现相同。资源截断不提供完整流 throughput RTF；已花 CPU/可测 service 除以全计划时长的 profile 成本比另存 censored_profile_cost_ratios，不能靠未处理后缀稀释成本后宣称速度。',
        '\n## E07：合法消融',
        table(['流/开关实际 ID','status','APE m / ARE rad','mean H/V PL m','P95 ms','same-center M'],[(r['id'],r['status'],f"{fmt(m(r,'ape_m'))} / {fmt(m(r,'are_rad'))}",f"{fmt(r.get('integrity',{}).get('axes',{}).get('H',{}).get('pl',{}).get('mean'))} / {fmt(r.get('integrity',{}).get('axes',{}).get('V',{}).get('pl',{}).get('mean'))}",timing(r,'core_ms'),r.get('same_center_M')) for r in records if r['group']=='E07']),
        table(['消融 ID','公共区间 s / N','APE ablated / FULL m','ARE ablated / FULL rad','最大 center 差 m'],[(a['id'],f"{a['interval_s']} / {a['common_n']}",f"{fmt(a['A_common']['ablated']['ape_m']['rmse'])} / {fmt(a['A_common']['full']['ape_m']['rmse'])}",f"{fmt(a['A_common']['ablated']['are_rad']['rmse'])} / {fmt(a['A_common']['full']['are_rad']['rmse'])}",a['max_center_difference_m']) for a in metrics.get('ablation_comparisons',[])]),
        '\nterminal support 关闭回原保守 box；joint/sample0 无独立合法开关，使用 joint+support 合并回退原物理 law，并明确不能分别归因。G2 旧数值候选 80 历元用于资格/数值对照，不拿其 80 vs 600 全段均值声称轨迹优劣。同 center M 见真实 old/new 同 stamp；A 完整轨迹与 P 时间分列，Gaussian 未资格结果不能收紧 raw 界。',
        '\n## E08：长期边界',
        table(['流/method','status','actual / planned','APE / ARE','x0 retired','last q / alerts','slots / ledger / replay / RSS last'],[(r['stream']+'/'+r['requested_method'],r['status'],f"{r.get('actual_max_epoch','NA')} / {r.get('planned_epochs','NA')}",f"{fmt(m(r,'ape_m'))} / {fmt(m(r,'are_rad'))}",r.get('x0_retired'),f"{fmt(r.get('resources',{}).get('q',{}).get('last'))} / {fmt(r.get('resources',{}).get('alerts',{}).get('last'))}",' / '.join(fmt(r.get('resources',{}).get(k,{}).get('last')) for k in ['slots','ledger','replay','rss'])) for r in records if r['group']=='E08']),
        '\nL0 5000 FULL、L1 10000 FULL/NOFDE 使用最终冻结二进制、不重启、不重置 source event。晚期故障与早期同幅值，ledger/provider/slot/RSS/queue 增长据实际 receipt 报告，不要求 O(1) 或无限全状态可观测。',
        '\n## 图表与精简数据',
        '\n[指标 JSON](SC8_METRICS.json)、[绘图必要数据](SC8_PLOTDATA.json)、[图表身份](SC8_FIGURES.json)。F00–F10 共 12 个 figure family（F05-H/V 两张），每张 180 mm、300 dpi PNG + 可编辑文字 SVG。只保存实际曲线峰谷/事件附近点和实际二维计数，统计仍用全样本；每 seed/录制指标留存，不将帧数当独立样本。',
    ]
    figure_index=OUT/'SC8_FIGURES.json'
    figure_current=figure_index.exists() and json.loads(figure_index.read_text()).get('metrics_sha256')==common.sha256(OUT/'SC8_METRICS.json')
    if figure_current:
        for item in json.loads(figure_index.read_text())['figures']:lines.append(f"\n![{item['id']}](figures/{item['id']}.png)\n\n{item['note']} [SVG](figures/{item['id']}.svg)")
    if not figure_current:lines.append('\n图表当前为早先进度版本，未与本次 metrics 摘要匹配；本报告不嵌入旧图充当新结果。W3 会重新生成并核对全部图表身份。')
    lines += ['\n## 实际困难与后续采集',
        '\n当前真实工程管线的候选拒绝、姿态大误差和处理积压属于结果；不能用仿真 raw 定理、最终历史或较短成功前缀替它背书。仅有位置改善不代表 body 姿态/bias 已观测。SFUISE 的仿真 velocity/bias 未在冻结导出接口保存，虽有源真值也为 NA_ESTIMATED_STATE_NOT_EXPORTED，属于指标接口限制；不是无真值。',
        '\n手册补充范围的明确缺口：冻结矩阵登记了弱短 IMU、负幅和错峰，但没有独立弱幅 UWB 小矩阵，也没有另跑 S1/S2 bias 轨迹。D0/S0 与 M3 的结果不能顶替 S1/S2；不引用旧 SC7 曲线冒充本轮。S0 的 bias 包含收缩依赖原始 sample0 信息，不表示持续获得全部 bias 或 15D 可观测性。W0 额外 attempt 后本轮计划占满 160 上限，这些未登记补充不事后补点或伪称通过。',
        '\n后续可另采独立标定录制：覆盖三轴角运动、平移与不同姿态，独立测量 body/tag/IMU/GT 杆臂和时间延迟，并留完全不参与拟合的 TEST。优先记录硬件时间同步、原始逐条 range 和连续 IMU 缺口，保存提交/拒绝/发布时刻、依赖截止与原生恢复 receipt。若希望把经验结果提升为真实 raw 保证，还需独立验证初始 law、噪声/运动域/采样/anchor 映射及证明接口，不能由当前六条经验轨迹或零超界样本推导。',
        '\n现有接口的优先补缺：Walk 需要独立的 tracker↔IMU 旋转/平移和 tag 测点定义；STAR 需要按原始设备 TF 与时钟约定闭合 gyro/rig 转换，并在独立 CAL 同时验证角速度、重力和杆臂。当前 CAL 诊断不能替代这些独立信息，也不能用 TEST 误差挑翻转矩阵或时延。修复后应另冻结版本重跑受影响格，本轮不在 160 预算内悄悄换源或改写旧结果。',
        '\n这些采集与理论扩展建议不改变本轮冻结算法，也不阻止本轮仿真、经验和窗口边界的结项。']
    lines += ['\n## SC8-A01–A24 对账',table(['验收','证据/状态'],[
        ('A01','入口/冻结 identity、ELF/DSO/libraries/platform；旧报告只库存；生产合同 hash 未改'),('A02','E00 九条库存，六条固定测试，备用未启用'),('A03','字段/单位/时间审计与899/899 direct/ROS；Walk tracker/body/tag参照未闭合，物理分数NA，诊断与proxy保留'),('A04','CAL-W 与 R3 prefix20 +2s 隔离，test-involved 明示，无逐方法测试拟合'),('A05','三通道和 requested/effective 分离；真实 raw 拒绝/PL NA'),('A06','SF 固定 pin、官方剔除、首发/历史/cutoff/revision，数值观察双实例检查'),('A07','共同 source-only、native 逐值 audit；lossless bag parity，原始数据不改'),('A08','13 项独立 scorer 测试 + evo 对口径'),('A09',f"E01: {dict(Counter(r['status'] for r in records if r['group']=='E01'))}"),('A10',f"E02: {dict(Counter(r['status'] for r in records if r['group']=='E02'))}; 工程结果不叫 FULL raw"),('A11',f"E03: {dict(Counter(r['status'] for r in records if r['group']=='E03'))}; 原生 episode 与真实增量/阶段分开"),('A12','seed/录制统计，负收益保留，无帧级伪独立 CI'),('A13','实际 H/V Stanford 分母对账，AL/SI/late/拒绝分层，零 HMI 不当证明'),('A14',f"E05: {dict(Counter(r['status'] for r in records if r['group']=='E05'))}; 7 唯一合法配置"),('A15','同进程 LF+2 canary、边缘化与 resource/inflight 右截断；公共 1101–2100，缺数据 NA'),('A16','逐 run core/sink/age/CPU/RTF；exclusive 均值闭合，P95不相加'),('A17','40/50 ms 和95%全计划分母；真实及时估计≠raw资格'),('A18',f"E07: {dict(Counter(r['status'] for r in records if r['group']=='E07'))}; 合并开关与 M/A/P 明示"),('A19',f"E08: {dict(Counter(r['status'] for r in records if r['group']=='E08'))}; late gaps={len(late)}"),('A20','F00–F10 实际图/精简绘图数据；NA和overflow保留' if figure_current else 'PENDING：图表与当前 metrics 摘要尚未全部匹配'),('A21','最终相关回归 PASS' if checked else 'PENDING：最终相关回归未全部结束'),('A22',str(counts)+f"; attempts={metrics['attempt_processes']}，辅助另计，完整版本/失败身份留存"),('A23','唯一中文报告+English Results，实际冻结 registry/metrics/plot CLI'),('A24','阶段本地 commit；只清理本轮 owned cache，保留原始数据/用户旧证据，不 push')]),
        '\n## 复现与续跑',
        '```bash\npython3 tools/sc8_experiments.py preflight --build\npython3 tools/sc8_sfuise_observer.py\npython3 tools/sc8_sfuise_observer.py check\npython3 tools/test_sc8_metrics.py\npython3 tools/test_sc8_reference.py\npython3 tools/test_sc8_inputs.py\npython3 tools/test_sc8_inputs.py bags\n# 在已冻结环境续跑；已有完成/失败 attempt 默认均复用，不自动重复求解\npython3 tools/sc8_experiments.py run --group E01,E02,E03\npython3 tools/sc8_experiments.py run --group E05,E07,E08\npython3 tools/sc8_experiments.py run --group E06\npython3 tools/sc8_experiments.py checks\npython3 tools/sc8_experiments.py evaluate\npython3 tools/sc8_experiments.py figures\npython3 tools/sc8_experiments.py report\n# 全部运行和图表结束后审计；清理只接受当前 PASS 审计\npython3 tools/sc8_analysis.py audit\npython3 tools/sc8_cleanup.py --apply\npython3 tools/sc8_experiments.py report\n```',
        '\n在新环境重建时应核对 frozen source/依赖/ELF 身份；若编译器、库或 ELF 不同，须另存版本化 registry，不能覆盖此轮身份。registry 内保存完整数值 CAL 参数，输入重建使用冻结标定，不在 TEST 重拟合。只重画图可用永久 metrics+plotdata：`figures` 在原始 trace 不存在时读取保存的 plotdata，不启动求解。',
        '\n执行缺口：'+('; '.join(all_gaps) if all_gaps else '暂无已发生缺口；以最终矩阵和回归为准。'),
        f"\n断点：冻结 `{f.get('identity',{}).get('head','NA')}`；待运行 {pending}，运行错误/未完成长流 {len(gaps)+len(late)}。所有逻辑 ID 和实际 attempt 见 JSON。资源上限是结果；接口/执行缺口没有改写 PASS。最终审计和本轮 cache 清理状态另记录在本报告的本地阶段提交日志。",
        '\n## English Results',
        f"\nThe frozen evaluation registered {metrics['registered']} unique runs and retained all {metrics['attempt_processes']} execution attempts (plus the registered ROS counterpart). Current process outcomes are {counts}. Accuracy is scored without fitted trajectory alignment, with each seed or recording retained as the independent unit. First causal SFUISE outputs and final retained-history outputs are separate views of the same run; differing initialization information, input cutoffs and state age preclude an unconditional online ranking. Only registered simulated sources can receive conditional raw protection bounds. Real and semi-synthetic recordings use an explicitly labelled engineering FDE pipeline with no matched raw PL. Walk physical tag/body reference remains open: CAL-W rejects tracker=IMU identity, so physical scores are unavailable and frozen proxy differences remain separately labelled. Historical bootstrap points with invalid original-idle support are explicitly unavailable; first causal outputs are unchanged. Censored suffixes, failures, negative gains and attitude degradation remain visible. Large-window results are measured cost frontiers, not an assertion of real-time operation at 1000 epochs. The evaluation status is {status}; zero observed HMI events are not a probabilistic proof."]
    healthy=[r for r in records if r['group']=='E01' and r['stream']=='SC8-H' and r['requested_method']=='O-FULL' and 'ape_m' in r.get('views',{}).get('O-FULL',{})]
    if healthy:
        values=[r['views']['O-FULL']['ape_m']['rmse'] for r in healthy]
        rates=[r['integrity']['joint_AL']['2/3']['timely_within_AL']/r['integrity']['planned'] for r in healthy]
        lines.append(f"\nAcross {len(healthy)} healthy seeds, full-pipeline tag APE RMSE ranged from {min(values):.6g} to {max(values):.6g} m. The default timing and timely-spatial tests passed {sum(r.get('tested_profile_deadline_pass') is True for r in healthy)}/{len(healthy)}, while timely joint availability at the research H/V alert limits of 2/3 m was only {100*min(rates):.4g}–{100*max(rates):.4g}%. Small measured error does not imply equally tight protection bounds.")
    for stream in ['SC8-U','SC8-A','SC8-G','SC8-J']:
        pairs=[p for p in metrics.get('paired',[]) if p['group']=='E03' and p['stream']==stream]
        values=[p['gains'].get('ape_m',{}).get('off_minus_on') for p in pairs];values=[v for v in values if v is not None]
        angles=[p['gains'].get('are_rad',{}).get('off_minus_on') for p in pairs];angles=[v for v in angles if v is not None]
        if values and angles:lines.append(f"\nFor {stream}, mean paired OFF-minus-ON improvements were {np.mean(values):.6g} m in APE and {np.mean(angles):.6g} rad in ARE across {len(values)} seeds. Negative values denote deterioration; position and attitude effects are reported separately.")
    for filename in ['SC8_AUDIT.json','SC8_CLEANUP.json']:
        path=OUT/filename
        if path.exists():
            receipt=json.loads(path.read_text());lines.append(f'\n最终本地审计/清理收据：[{filename}]({filename})，状态 {receipt.get("status")}。')
    import re
    document=re.sub(r'(?m)^(#{1,6} .+)\n(?!\n)',r'\1\n\n','\n'.join(lines)+'\n')
    (OUT/'SC8_RESULTS.md').write_text(document)
    print('REPORT',status,'pending',pending,'checks',checked,'gaps',len(all_gaps),flush=True)


def evaluation_status(records,validation):
    pending=any(r['status'] in ['NOT_ATTEMPTED','RUNNING'] for r in records)
    checks=validation.get('final-checks.json',{}).get('result',[])
    checked=bool(checks) and all(c['returncode']==0 for c in checks)
    gaps=[r['id'] for r in records if r['status']=='EXECUTION_ERROR' or 'scoring_error' in r]
    if any(r.get('track')=='T-SIM' and r['requested_method']=='SFUISE' and r.get('views') for r in records):gaps.append('SFUISE simulated velocity/bias estimates were not exported; source truth exists but corresponding state-error metrics remain NA, no pose-difference proxy')
    gaps.append('STAR gyro-frame input validation OPEN: published transform direction labels and CAL-S gyro/GT consistency do not close; frozen outputs retained as interface-level outcomes, no intrinsic algorithm ranking')
    gaps.append('Walk physical tag/body GT reference OPEN: CAL-W rejects tracker=IMU identity; frozen numerical proxy metrics are retained separately, not physical accuracy')
    gaps.append('Registered scope omission: no separate weak-UWB amplitude matrix or S1/S2 bias trajectories; no historical curves substituted')
    gaps += [r['id']+': long stream did not reach final planned epoch' for r in records if r['group']=='E08' and r['stream']=='SC8-L1' and r['status'] not in ['NOT_ATTEMPTED','RUNNING'] and r.get('max_committed_epoch',0)<10000]
    gaps += [r['id']+': real engineering run censored before complete recording' for r in records if r.get('track') in ['T-REAL','T-SEMI'] and r['group']!='E00' and r['status']=='RESOURCE_LIMIT']
    historical=[r for r in records if r['group']!='E00' and r.get('historical_export_validity',{}).get('rejected_n',0)]
    if historical:gaps.append('SFUISE historical original-idle prefix unavailable: '+str(sum(r['historical_export_validity']['rejected_n'] for r in historical))+' support-invalid exports across '+str(len(historical))+' formal runs; first causal outputs unaffected')
    return ('IN_PROGRESS' if pending or not checked else 'SC8_EVALUATED_WITH_EXECUTION_GAPS' if gaps else 'SC8_EVALUATION_COMPLETE_FOR_REGISTERED_MATRIX'),gaps


def unit_aggregates(records):
    grouped={}
    for r in records:
        if r['group'] not in ['E01','E02','E03']:continue
        for view,score in r.get('views',{}).items():
            grouped.setdefault((r['group'],r['stream'],view),[]).append((r,score))
    out=[]
    for (group,stream,view),rr in grouped.items():
        agg={}
        for name in ['ape_m','hpe_m','vpe_m','are_rad','rpe_tag_0.5s_m','rpe_body_0.5s_m','rpe_body_0.5s_rad','rpe_tag_1s_m','rpe_body_1s_m','rpe_body_1s_rad']:
            vals=[s.get(name,{}).get('rmse') for _,s in rr];a=np.array([x for x in vals if x is not None])
            agg[name]=dict(n_units=len(a),n_missing=len(vals)-len(a),mean=float(a.mean()) if len(a) else None,
                std=float(a.std(ddof=1)) if len(a)>1 else None,min=float(a.min()) if len(a) else None,max=float(a.max()) if len(a) else None)
        out.append(dict(group=group,stream=stream,view=view,unit='equally weighted independent seed / recording RMSE; no frame-level CI',run_ids=[r['id'] for r,_ in rr],metrics=agg))
    return out


def phase_aggregates(records):
    result=[]
    for stream in ['SC8-U','SC8-A','SC8-G','SC8-J']:
        for method in ['O-FULL','O-NOFDE']:
            runs=[r for r in records if r['group']=='E03' and r['stream']==stream and r['requested_method']==method and r.get('phase_errors')]
            for phase in ['healthy_prefix','fault_active','bridge','rejoin','late_last_quarter']:
                values={};receipts=sum(r['phase_errors'].get(phase,{}).get('ape',{}).get('n',0) for r in runs)
                for key in ['ape','rotation_error','velocity_error','accel_bias_error','gyro_bias_error']:
                    units=[r['phase_errors'].get(phase,{}).get(key,{}).get('rmse') for r in runs];units=[x for x in units if x is not None]
                    values[key]=dict(n_units=len(units),mean=float(np.mean(units)) if units else None,min=min(units) if units else None,max=max(units) if units else None)
                result.append(dict(stream=stream,method=method,phase=phase,receipts=receipts,metrics=values,scope='Equally weighted seed RMSE; stages may overlap (persistent rejoined state versus late), not integrity partitions'))
    return result


def ablation_comparisons(records,registry):
    out=[];index={e['id']:e for e in registry['runs']}
    for r in records:
        if r['group']!='E07' or not r.get('views'):continue
        base=next((b for b in records if b['group'] in ['E01','E03'] and b['stream']==r['stream'] and b.get('seed')==20261020 and b['requested_method']=='O-FULL' and b.get('views')),None)
        if base is None:continue
        pa,truth,lever=run_poses(r,index[r['id']]);pb,_,_=run_poses(base,index[base['id']])
        if not pa or not pb:continue
        a=next(iter(pa.values()));b=next(iter(pb.values()));grid=truth[0];mask=metric.sample(*a,grid)[0]&metric.sample(*b,grid)[0];grid=grid[mask]
        if len(grid)<2:continue
        target=(grid,*metric.sample(*truth,grid)[1:]);aa=metric.evaluate((grid,*metric.sample(*a,grid)[1:]),target,lever)[0];bb=metric.evaluate((grid,*metric.sample(*b,grid)[1:]),target,lever)[0]
        ra={int(value(x,'stamp_ns')):x for x in rows(latest(index[r['id']]).parent/'trace.csv') if value(x,'updates')==1};rb={int(value(x,'stamp_ns')):x for x in rows(latest(index[base['id']]).parent/'trace.csv') if value(x,'updates')==1};stamps=sorted(ra.keys()&rb.keys());same_center=max([np.linalg.norm([value(ra[t],'tag_'+k)-value(rb[t],'tag_'+k) for k in 'xyz']) for t in stamps],default=float('nan'))
        pstats={}
        for label,rec in [('ablated',r),('full',base)]:
            allrows=rows(latest(index[rec['id']]).parent/'trace.csv',keep_partial=True)
            pstats[label]=metric.summary([value(x,'core_ms') for x in allrows if x.get('status')!='SOURCE_GAP_OR_ABORT' and grid[0]-1e-8<=value(x,'epoch')*.05<=grid[-1]+1e-8])
        out.append(dict(id=r['id'],baseline=base['id'],common_n=len(grid),interval_s=[float(grid[0]),float(grid[-1])],
            input_digest_equal=r['input_digest']==base['input_digest'],max_center_difference_m=same_center,
            A_common=dict(ablated=aa,full=bb),P_common=pstats,PL_common={k:dict(ablated=metric.summary([value(ra[t],k) for t in stamps if spatial(ra[t]) and spatial(rb[t])]),full=metric.summary([value(rb[t],k) for t in stamps if spatial(ra[t]) and spatial(rb[t])])) for k in ['hpl','vpl']},
            performance_scope='Measured P includes complete run; G2 old prefix 80 has separate common-prefix A/PL comparison. No time speedup from unmatched lengths.'))
    return out


def findings(metrics):
    rr=metrics['runs'];paragraphs=[]
    healthy=[r for r in rr if r['group']=='E01' and r['stream']=='SC8-H' and r['requested_method']=='O-FULL' and r.get('views')]
    if healthy:
        a=[next(iter(r['views'].values()))['ape_m']['rmse'] for r in healthy]
        ar=[next(iter(r['views'].values()))['are_rad']['rmse'] for r in healthy]
        passes=sum(r.get('tested_profile_deadline_pass') is True for r in healthy)
        paragraphs.append(f"健康 D1/M3/G1：{len(a)} 个独立 seed 的 FULL tag APE RMSE 为 {min(a):.6g}–{max(a):.6g} m，ARE 为 {min(ar):.6g}–{max(ar):.6g} rad；默认双 P95 与及时空间比例条件通过 {passes}/{len(a)}。这只覆盖实测的注册 profile，不授予真实应用资格。")
        for al in ['2/3','5/5']:
            rates=[r['integrity']['joint_AL'][al]['timely_within_AL']/r['integrity']['planned'] for r in healthy]
            paragraphs.append(f"同一健康组在研究 AL(H,V)=({al.replace('/',',')}) m 下，joint 及时可用率为 {100*min(rates):.4g}%–{100*max(rates):.4g}%。空间 raw 资格、deadline 和满足所选 AL 是三个不同条件；厘米级实际误差不意味着 PL 也为厘米级。")
    for stream in ['SC8-U','SC8-A','SC8-G','SC8-J']:
        pp=[p for p in metrics.get('paired',[]) if p['group']=='E03' and p['stream']==stream]
        vals=[p['gains'].get('ape_m',{}).get('off_minus_on') for p in pp];vals=[x for x in vals if x is not None]
        angles=[p['gains'].get('are_rad',{}).get('off_minus_on') for p in pp];angles=[x for x in angles if x is not None]
        if vals:paragraphs.append(f"{stream}：{len(vals)} 个 seed 的公共观测区间上 OFF−ON APE 改善 {min(vals):.6g}–{max(vals):.6g} m（均值 {np.mean(vals):.6g} m）；ARE 改善 {min(angles):.6g}–{max(angles):.6g} rad。负值是退化，不能由另一个指标抵销。")
    real=[r for r in rr if r['group']=='E02' and r['requested_method']=='O-FULL' and r['status'] not in ['NOT_ATTEMPTED','RUNNING']]
    if real:paragraphs.append(f"真实工程 FDE：已结束 {len(real)}/6 条完整录制尝试，其中 {sum(r['status']=='RESOURCE_LIMIT' for r in real)} 条在预登记 wall 资源门限截断；SC7 raw 资格仍全部拒绝。未完成后缀不填误差，不能将有限公共区间的收益推广到全录制。")
    if any(r.get('reference_contract')=='OPEN_WALK_TRACKER_BODY_TAG' for r in rr):paragraphs.append('Walk R1/R2 的 tracker→IMU/body/tag 参照未闭合：物理 APE/ARE/RPE 撤回为 NA，原冻结参考下的数值只留作明确标注的代理诊断。没有按 TEST 选择翻转、时移或新杆臂来改善任何方法的分数。')
    for r in rr:
        if r['group']=='E02' and r['stream']=='R5' and 'ape_m' in r.get('views',{}).get('S-ONLINE',{}):
            score=r['views']['S-ONLINE'];paragraphs.append(f"STAR-Loc R5 的 SFUISE 首次在线流 tag APE RMSE {score['ape_m']['rmse']:.6g} m、最大 {score['ape_m']['max']:.6g} m，保留整个实测尺度；官方剔除未关闭，codec/单位/原始输入逐项核对，不能删去发散段后宣称成功。CAL-S 对 gyro/rig 转换的方向约定和角速度一致性尚未闭合，因此这些值描述冻结接口下的实际行为，不作为正确输入下算法本身的优劣证据。工程 ON 只有截断前缀，不能以其短段误差与该完整流作全程排名。")
    native=[r for r in rr if r['group']!='E00' and 'axes' in r.get('integrity',{})]
    if native:
        under=sum(r['integrity']['axes'][axis]['underbound'] for r in native for axis in ['H','V']);n=sum(r['integrity']['spatial_valid'] for r in native)
        paragraphs.append(f"实际原生条件 raw 记录共 {n} 个合格 tag receipt（不同场景/消融/窗口不可当作独立概率样本），H/V 合计 underbound {under} 次。计数包含迟到与 AL-unavailable 的合格界；无论是否为零，都不作为风险概率证明。")
    window=[r for r in rr if r['group']=='E05' and r['status'] not in ['NOT_ATTEMPTED','RUNNING']]
    if window:paragraphs.append(f"窗口前沿已结束 {len(window)}/14：资源截断 {sum(r['status']=='RESOURCE_LIMIT' for r in window)}，进入公共 1101–2100 评分区间 {sum(bool(r.get('views')) for r in window)}。窗口/回放同时变化，缺失公共区间不外推；1000 历元是否经历边缘化只认实际 receipt。")
    base=next((r for r in window if r['stream']=='SC8-WH' and r['window']==[30,30] and r.get('window_common_coverage',{}).get('complete_pose_interval')),None)
    large=next((r for r in window if r['stream']=='SC8-WH' and r['window']==[1000,1000] and r.get('window_common_coverage',{}).get('complete_pose_interval')),None)
    if base and large:
        a=base['views']['O-FULL']['ape_m']['rmse'];b=large['views']['O-FULL']['ape_m']['rmse'];p=base['timing']['core_ms']['p95'];q=large['timing']['core_ms']['p95']
        paragraphs.append(f"同一注册健康 seed、完整公共 1000 历元：30/30→1000/1000 的 APE 为 {a:.7g}→{b:.7g} m，HPL/VPL 几乎未变；core P95 为 {p:.6g}→{q:.6g} ms。该流没有可观的误差/界收益，却付出明显成本；这是一条流的成本前沿，不推广为所有运动或几何都无大窗收益。")
    for r in rr:
        if r['group']=='E07' and r.get('native_failures'):
            fail=r['native_failures'][0]
            paragraphs.append(f"消融 {r['stream']}/{r['requested_method']} 在历元 {fail['epoch']} 返回 {fail['reason']}，最后提交 {r.get('max_committed_epoch',0)}/{r.get('planned_epochs')}。无合格轨迹时精度与 PL 为 NA；实际 abort 耗时和全部计划分母仍保留，不补跑或放宽资格。")
    for r in rr:
        if r['group']=='E08' and r['status'] not in ['NOT_ATTEMPTED','RUNNING']:
            paragraphs.append(f"长流 {r['stream']}/{r['requested_method']}：实际最后提交 {r.get('max_committed_epoch',0)}/{r.get('planned_epochs')}，进程 {r['status']}；最后实际服务历元 {r.get('actual_max_epoch',0)}，source queue 记录 {r.get('source_delivery_counts',{})}。未提交后缀和未确认到达的晚期故障不算恢复成功。")
    return paragraphs


def audit():
    """Final read-only reconciliation. No solver, input mutation or cleanup."""
    metrics=json.loads((OUT/'SC8_METRICS.json').read_text());registry=yaml.safe_load((ROOT/'config/sc8_experiments.yaml').read_text())
    from sc8_experiments import current_identity
    now=current_identity();frozen=registry['frozen']['identity'];checks=[]
    def check(name,condition,detail=None):checks.append(dict(check=name,passed=bool(condition),detail=detail))
    check('frozen_computation',now['computational_content_sha256']==frozen['computational_content_sha256'])
    check('frozen_solver_ELF_and_DSO',now['binaries']==frozen['binaries'])
    changed_formal=[r['id'] for r in metrics['runs'] if r['group']!='E00' and (r.get('identity',{}).get('computational_content_sha256')!=frozen['computational_content_sha256'] or r.get('identity',{}).get('binaries')!=frozen['binaries'])]
    check('every_formal_run_used_frozen_computation_and_ELFs',not changed_formal,changed_formal)
    changed_libraries=[p for p,h in registry['frozen'].get('loaded_libraries',{}).items() if not Path(p).is_file() or common.sha256(Path(p))!=h]
    changed_contracts=[p for p,h in registry['frozen'].get('unchanged_contracts',{}).items() if not (ROOT/p).is_file() or common.sha256(ROOT/p)!=h]
    check('frozen_loaded_libraries',not changed_libraries,changed_libraries)
    check('original_noise_risk_AL_and_publication_contracts',not changed_contracts,changed_contracts)
    import subprocess
    import run_sfuise_benchmark as sf
    baseline_head=subprocess.check_output(['git','-C',str(sf.SFUISE_ROOT),'rev-parse','HEAD'],text=True).strip()
    baseline_status=subprocess.check_output(['git','-C',str(sf.SFUISE_ROOT),'status','--porcelain'],text=True).strip()
    check('pinned_original_SFUISE_source_clean',baseline_head==registry['frozen']['baseline_pin'] and not baseline_status,dict(head=baseline_head,status=baseline_status))
    import hashlib
    design={k:registry[k] for k in ['seeds','cases','runs','resource_stops','evaluation','calibration','inventory']}
    check('frozen_registered_design',hashlib.sha256(json.dumps(design,sort_keys=True).encode()).hexdigest()==registry['frozen']['design_sha256'])
    check('all_registered_attempted',all(r['status'] not in ['NOT_ATTEMPTED','RUNNING'] for r in metrics['runs']))
    check('logical_status_partition',sum(metrics['logical_run_status_counts'].values())==len(registry['runs'])==len(metrics['runs']))
    auxiliary=sum(a.get('processes',0) for a in registry.get('auxiliary_solver_audits',[]))
    check('solver_process_ceiling',metrics['attempt_processes']+auxiliary<=160,dict(attempts=metrics['attempt_processes'],auxiliary=auxiliary))
    check('scoring_errors_retained_and_absent',not any('scoring_error' in r for r in metrics['runs']),[r['id'] for r in metrics['runs'] if 'scoring_error' in r])
    pairbad=[p['stream'] for p in metrics.get('paired',[]) if not p['input_digests_equal']]
    check('main_physical_input_pairing',not pairbad,pairbad)
    unsupported=[p['stream'] for p in metrics.get('paired',[]) if p.get('post_fault_common_n') is not None and p['post_fault_common_n']<2 and any(g.get('off_minus_on') is not None for g in p['gains'].values())]
    check('real_fault_gains_require_post_injection_common_states',not unsupported,unsupported)
    reference=metrics.get('validation',{}).get('reference-check/result.json',{}).get('result',{})
    check('CAL_reference_diagnostic_retained_as_OPEN',reference.get('status')=='OPEN_REFERENCE_CONTRACT' and reference.get('original_parameter_changes') is False)
    walk=[r for r in metrics['runs'] if r.get('reference_contract')=='OPEN_WALK_TRACKER_BODY_TAG']
    check('unresolved_walk_reference_has_no_physical_APE_or_ARE_claim',bool(walk) and all(s.get(k,{}).get('rmse') is None for r in walk for s in r.get('views',{}).values() for k in ['ape_m','are_rad']))
    check('unresolved_walk_reference_has_no_physical_paired_gain',all(g.get('off_minus_on') is None for p in metrics.get('paired',[]) if p.get('reference_contract')=='OPEN_WALK_TRACKER_BODY_TAG' for g in p['gains'].values()))
    star=reference.get('STAR_CAL_only',{})
    check('STAR_tag_point_reference_verified_input_frame_gap_disclosed',star.get('status')=='INPUT_GYRO_FRAME_VALIDATION_OPEN' and star.get('official_tag_lever_consistency_rmse_m',1)<1e-9 and star.get('candidate_applied') is False)
    baseline=[r for r in metrics['runs'] if r['group']=='E01' and r['stream']=='SC8-H' and r.get('seed')==20261020]
    full=next((r for r in baseline if r['requested_method']=='O-FULL'),{})
    nominal=next((r for r in baseline if r['requested_method']=='O-NOM'),{})
    for key in ['ape_m','are_rad','velocity_error','accel_bias_error','gyro_bias_error']:
        a=full.get('views',{}).get('O-FULL',{}).get(key,{}).get('rmse');b=nominal.get('views',{}).get('O-NOM',{}).get(key,{}).get('rmse')
        check('healthy_full_nominal_same_path_'+key,a is not None and b is not None and abs(a-b)<1e-12)
    native=[];paritybad=[]
    byid={e['id']:e for e in registry['runs']}
    for r in metrics['runs']:
        if r['group']=='E00' or r.get('track')!='T-SIM' or r['requested_method'] in ['O-NOM','SFUISE']:continue
        entry=byid[r['id']];expected=entry.get('epochs',registry['cases'][entry['stream']]['epochs']);p=r.get('native_input_parity',{})
        if p.get('status')!='PASS_EXACT_PHYSICAL_PREFIX' or p.get('imu')!=expected*10+1 or p.get('uwb')!=expected*8:paritybad.append(r['id'])
        integ=r.get('integrity',{})
        if 'axes' not in integ:continue
        native.append(r['id'])
        check(r['id']+': denominator',integ['planned']==integ['spatial_valid']+integ['refused']+integ['missing'])
        for axis in ['H','V']:
            for al,v in integ['axes'][axis]['AL'].items():check(r['id']+f': {axis} AL{al} partition',sum(v[k] for k in ['nominal_admissible','MI_candidate','HMI_candidate','AL_unavailable'])==integ['spatial_valid'])
        if r.get('exclusive_means_ms'):check(r['id']+': exclusive mean closure',abs(sum(r['exclusive_means_ms'].values())-r['exclusive_mean_core_ms'])<1e-8)
    check('native_actual_physical_input_parity_all_formal',not paritybad,paritybad)
    for name in ['input-checks/result.json','input-checks/bag-parity.json','input-checks/all-bag-parity.json','spline-export-check/result.json','native-observer-prefix-check.json','sfuise_observer/check.json','equivalence/R3/equivalence.json']:
        result=metrics.get('validation',{}).get(name,{}).get('result',{})
        check(name,result.get('status')=='PASS' or result.get('returncode')==0)
    final=metrics.get('validation',{}).get('final-checks.json',{}).get('result',[])
    check('focused_final_regression',bool(final) and all(r['returncode']==0 for r in final))
    fig_index=json.loads((OUT/'SC8_FIGURES.json').read_text());check('figure_families',len(fig_index['figures'])==12)
    check('figure_metrics_identity',fig_index['metrics_sha256']==common.sha256(OUT/'SC8_METRICS.json'))
    check('figure_plotdata_identity',fig_index['plotdata_sha256']==common.sha256(OUT/'SC8_PLOTDATA.json'))
    from PIL import Image
    artifacts={}
    for family in fig_index['figures']:
        for extension in ['png','svg']:
            p=OUT/'figures'/(family['id']+'.'+extension);check(p.name,p.is_file() and p.stat().st_size>0)
            if p.exists():artifacts[str(p.relative_to(OUT))]=common.sha256(p)
        with Image.open(OUT/'figures'/(family['id']+'.png')) as im:
            check(family['id']+': 300dpi_180mm',abs(im.width-180/25.4*300)<=2 and abs(im.info.get('dpi',(0,0))[0]-300)<1)
    pdata=json.loads((OUT/'SC8_PLOTDATA.json').read_text())
    for method,axes in pdata['stanford'].items():
        for axis,views in axes.items():
            for view,h in views.items():check(f'{method} {axis} {view}: actual histogram tally',sum(np.asarray(a).sum() for a in h['layers'].values())+h['outside']==h['total'])
    previous=json.loads((OUT/'SC8_AUDIT.json').read_text()) if (OUT/'SC8_AUDIT.json').exists() else {}
    retained={x['attempt']:x for x in previous.get('non_success_evidence',[])}
    for path in sorted((CACHE/'runs').glob('*/execution.json')):
        execution=json.loads(path.read_text())
        if execution['status']=='SUCCESS':continue
        evidence=retained.setdefault(path.parent.name,dict(attempt=path.parent.name))
        evidence.update({k:execution[k] for k in ['status','returncode','reason','runner_error','resource_reason','resource_observation'] if k in execution})
        if execution.get('details',{}).get('reason'):evidence['details_reason']=execution['details']['reason']
        console=path.parent/'console.log'
        if console.exists():
            text=console.read_text(errors='replace');evidence['console_sha256']=common.sha256(console)
            evidence['console_tail']=text.splitlines()[-12:]
    result=dict(status='PASS' if all(c['passed'] for c in checks) else 'FAIL',checks=checks,artifacts=artifacts,
        scope='Reconciliation and truthful-disclosure audit only. PASS does not close OPEN references/input interfaces, certify the system, or remove execution gaps.',
        metrics_sha256=common.sha256(OUT/'SC8_METRICS.json'),registry_sha256=common.sha256(ROOT/'config/sc8_experiments.yaml'),
        plotdata_sha256=common.sha256(OUT/'SC8_PLOTDATA.json'),
        analysis_source_sha256={str(p.relative_to(ROOT)):common.sha256(p) for p in [ROOT/'tools/sc8_analysis.py',ROOT/'tools/sc8_metrics.py',ROOT/'tools/sc8_cleanup.py',ROOT/'tools/test_sc8_reference.py',ROOT/'tools/test_sc8_metrics.py'] if p.exists()},
        non_success_evidence=list(retained.values()),
        metrics_status=metrics['status'],execution_gaps=metrics.get('execution_gaps'),
        numerical_regression_scope='Deterministic unit tests excluded from experimental solver budget by Handbook §7; ROS actual counterpart conservatively counted.',
        cleanup='NOT_PERFORMED_BY_AUDIT')
    save(OUT/'SC8_AUDIT.json',result);print('AUDIT',result['status'],'checks',len(checks),flush=True)
    for c in checks:
        if not c['passed']:print('FAILED',c['check'],c['detail'],flush=True)
    return result



def dataset_sim_errors(run,inputs,entry,registry):
    rr=rows(run/'states.csv');bias={}
    with (inputs/'raw.input').open() as f:
        for line in f:
            a=line.split()
            if a and a[0]=='imu':bias[int(a[2])]=np.array(list(map(float.fromhex,a[16:22])))
    tt=np.array([value(r,'t') for r in rr]);motion=registry['cases'][entry['stream']]['motion']
    if motion=='M0':velocity=np.cos(tt[:,None]*[.5,.4,.3])*[.05,.032,.015]
    else:velocity=np.column_stack([.48*np.cos(.6*tt),.3*np.cos(.5*tt),.16*np.cos(.4*tt)])
    actual=np.array([[value(r,'v'+k) for k in 'xyz'] for r in rr]);ba=[];bg=[]
    for r,t in zip(rr,tt):
        truth=bias[round(t*1e9)];observed=np.array([value(r,k) for k in ['bax','bay','baz','bgx','bgy','bgz']]);ba.append(np.linalg.norm(observed[:3]-truth[:3]));bg.append(np.linalg.norm(observed[3:]-truth[3:]))
    return dict(velocity_error=metric.summary(np.linalg.norm(actual-velocity,axis=1)),
        accel_bias_error=metric.summary(ba),gyro_bias_error=metric.summary(bg),
        true_bias='REGISTERED_SIM_RW_TRUTH from source-only latent audit; evaluation-only, no estimator feedback')


def display_with_gaps(t,p,gaps):
    if not gaps:return t,p
    tt=[];pp=[]
    for i,(stamp,point) in enumerate(zip(t,p)):
        if i and any(t[i-1]<=a+1e-8 and stamp>=b-1e-8 for a,b in gaps):
            tt.append(float('nan'));pp.append(np.full_like(point,float('nan')))
        tt.append(stamp);pp.append(point)
    return np.asarray(tt),np.asarray(pp)


def historical_support(run,pose):
    """Reject only the mathematically undefined original-idle export domain.

    Frozen global_.updateKnots copies the latest window's idles into the global
    aggregate. Interpolation at t < origin+2*dt then refers to wrong controls.
    This rule is derived from the upstream spline support, applies to all streams
    without looking at PE, and never changes an estimator output or raw artifact.
    """
    obs=rows(run/'observer.csv')
    if len(obs)<2:raise ValueError('historical exporter support cannot be identified from observer receipts')
    stamps=[int(x['state_ns']) for x in obs];indices=[int(x['estimate_index']) for x in obs]
    periods=[]
    for a,b,i,j in zip(stamps,stamps[1:],indices,indices[1:]):
        if j>i:
            assert (b-a)%(j-i)==0,'nonuniform spline clock receipt'
            periods.append((b-a)//(j-i))
    dt=Counter(periods).most_common(1)[0][0];assert dt>0
    origin=stamps[0]-indices[0]*dt+1
    safe_ns=origin+2*dt-1000000000000
    valid=np.rint(pose[0]*1e9).astype(np.int64)>=safe_ns
    meta=dict(status='PARTIAL_HISTORICAL_EXPORT_SUPPORT',raw_n=len(valid),rejected_n=int((~valid).sum()),valid_n=int(valid.sum()),
        safe_from_sensor_ns=safe_ns,global_origin_sensor_ns=origin-1000000000000,dt_ns=dt,
        rejected_timestamp_s=pose[0][~valid],rejected_raw_positions=pose[1][~valid],rejected_raw_quaternions=pose[2][~valid],
        reason='Global aggregate overwrote original idles; t < origin+2dt cannot be reconstructed. Mask is support-based, independent of error; raw files/hashes preserved.',
        validation='test_sc8_spline_export.py: online codec exact, late interpolation identical, no estimator constructed')
    return tuple(x[valid] for x in pose),meta

if __name__=='__main__':
    import sys
    if sys.argv[1:]==['audit']:
        raise SystemExit(0 if audit()['status']=='PASS' else 1)
    raise SystemExit('Use: python3 tools/sc8_analysis.py audit (after the registered matrix and final regressions)')
