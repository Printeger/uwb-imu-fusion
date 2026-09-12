#!/usr/bin/env python3
"""MILUV-only frozen Recover versus Reject orchestration; no estimator implementation."""
import argparse,ast,copy,csv,io,json,math,subprocess
from pathlib import Path
import numpy as np
import pandas as pd
import yaml
import run_recover_vs_reject as r
import rr_admitted as backend

RECORDINGS=('default_1_random3_0','default_1_circular3D_0')
ROOT=r.ROOT
PROTOCOL=ROOT/'experiments/MILUV_RECOVER_VS_REJECT_PROTOCOL.md'
AUDIT=r.WS/'evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41'
SOURCES=AUDIT/'sources'
IMU_SOURCE=r.WS/'evaluator_private/icra/miluv_rr_sources_20260912'
UWB_FIELDS=('timestamp','from_id','to_id','range_raw')
IMU_FIELDS=('timestamp',)+tuple(p+a for p in ('linear_acceleration.','angular_velocity.') for a in 'xyz')
TAG=10


def geometry():
    anchors={int(k):ast.literal_eval(v) for k,v in yaml.safe_load((SOURCES/'anchors.yaml').read_text())['0'].items()}
    lever=ast.literal_eval(yaml.safe_load((SOURCES/'tags.yaml').read_text())['ifo001'][str(TAG)])
    return dict(anchors=anchors,lever_body_m=lever,tag_id=TAG,constellation='0',
        origin_status='AUTHOR_BODY_IMU_COLOCATION_APPROXIMATION',axis_transform=np.eye(3).tolist(),
        acceleration='SPECIFIC_FORCE_MPS2',gyro='BODY_RADPS',time='PUBLISHED_COMMON_RELATIVE_SECONDS_NO_SECOND_TIMESHIFT',
        imu_bias_removal=False,beta='UNCORRECTED_MISSING_CALIBRATION',
        author_commit='399ad46f4e6e88c3c98b40aae33fe8e2c6719cff')


def cache_payloads(name,uwb,imu,geo):
    uu=[];ii=[]
    for row in uwb:
        if int(row['from_id'])!=TAG:continue
        i=row['source_row'];z=float(row['range_raw']);t=float(row['timestamp'])
        if not math.isfinite(t):raise ValueError('NONFINITE_UWB_TIME')
        valid=math.isfinite(z) and z>0
        if int(row['to_id']) not in geo['anchors']:raise ValueError('UNKNOWN_ANCHOR')
        uu.append(dict(zip(r.UWB_HEADER,[r.stable_id(name,i),i,0,i,row['timestamp'],TAG,int(row['to_id']),row['range_raw'],0,0,int(valid),'' if valid else 'INVALID_SOURCE_RANGE'.encode().hex()])))
    for i,row in enumerate(imu):
        vals=[row[k] for k in IMU_FIELDS]
        if not np.isfinite(np.array(vals,float)).all():raise ValueError('NONFINITE_IMU')
        ii.append(dict(zip(r.IMU_HEADER,[i,*vals,0,1,0,0,0])))
    for seq in [uu,ii]:
        if not seq or any(float(b['sensor_time_s'])<float(a['sensor_time_s']) for a,b in zip(seq,seq[1:])):raise ValueError('EMPTY_OR_NONMONOTONIC_INPUT')
    ub=r.csv_bytes(uu,r.UWB_HEADER);ib=r.csv_bytes(ii,r.IMU_HEADER)
    import hashlib
    m=dict(schema='nlos_measurement_cache_v2',base_recording_id=name,
        base_source_sha256='sha256:'+r.digest(dict(uwb=uwb,imu=imu)),recording_time_origin_s=0.,
        time_basis='sensor_time_from_recording_origin',imu_units='acc_mps2,gyro_radps,orientation_quaternion_wxyz',
        uwb_units='time_s,range_m,rssi_dbm',uwb_message_grouping='source_message_index',imu_file='imu.csv',uwb_file='uwb_observations.csv',
        imu_sha256='sha256:'+hashlib.sha256(ib).hexdigest(),uwb_sha256='sha256:'+hashlib.sha256(ub).hexdigest(),
        imu_count=len(ii),uwb_observation_count=len(uu),uwb_message_count=len(uu),
        transform_sha256='sha256:'+r.digest(dict(version='MILUV_RAW_TAG10_PX4_V1',geometry=geo,rssi='NEUTRAL_ZERO_PLACEHOLDER',orientation='UNUSED_IDENTITY_PLACEHOLDER')))
    m['cache_id']=r.cache_id(m);return m,ub,ib


def prepare(out):
    out.mkdir(parents=True,exist_ok=False);geo=geometry();cfg=yaml.safe_load(r.CONFIG.read_text())
    assert cfg['keyframe']['step']==4
    assert [cfg['nlos'][k] for k in ['cusum_forward_kappa','cusum_backward_kappa','cusum_forward_h','cusum_backward_h']]==[.5,.5,7.0234689587858723,7.0234689587858714]
    assert subprocess.check_output(['git','-C',str(backend.SF_SOURCE),'rev-parse','HEAD'],text=True).strip()=='75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d'
    implementations=[Path(__file__).resolve(),Path(r.__file__),Path(backend.__file__),
        ROOT/'experiments/scripts/evaluate_miluv_rr.py',ROOT/'experiments/scripts/test_miluv_rr.py',
        ROOT/'experiments/scripts/rr_evaluate.py',ROOT/'experiments/scripts/evaluate_recover_vs_reject.py',ROOT/'experiments/scripts/rr_sfuise_worker.py',
        ROOT/'experiments/scripts/audit_obstacle_ranges.py',PROTOCOL]
    lock=dict(schema='MILUV_RR_LOCK_V1',role='DEVELOPMENT_PREVIOUSLY_AUDITED_NOT_HELD_OUT',recordings=list(RECORDINGS),methods=list(r.METHODS),
        geometry=geo,evaluation={**r.EVALUATION,'reference':'tag10_antenna'},rows=[],core_files=backend.core_files(),
        implementation=backend.filemap(implementations),source_hashes={},prepared_hashes={},
        scientific_budget=dict(max_tasks=10,seconds_per_tree=1800,retries=0),
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        dirty_tracked_diff_sha256=r.digest(subprocess.check_output(['git','diff','--binary'],text=True)))
    exp=pd.read_csv(SOURCES/'experiments.csv',dtype={'anchor_constellation':str})
    prior=pd.read_csv(AUDIT/'report-da32f56f/persistent_links.csv')
    for name in RECORDINGS:
        assert exp.loc[exp.experiment==name,'anchor_constellation'].iloc[0]=='0'
        assert len(prior[(prior.family=='MILUV')&(prior.recording==name)&(prior.tag_id==TAG)])>0
        src=ROOT/'data/MILUV'/name/'ifo001';d=out/name;d.mkdir();inp=d/'input';inp.mkdir()
        u=r.projection(src/'uwb_range.csv',UWB_FIELDS);im=r.projection(src/'imu_px4.csv',IMU_FIELDS)
        m,ub,ib=cache_payloads(name,u,im,geo)
        (inp/'uwb_observations.csv').write_bytes(ub);(inp/'imu.csv').write_bytes(ib);r.write(inp/'input_manifest.json',m)
        cfg2=copy.deepcopy(cfg);cfg2['dataset']['cache_manifest']=str(inp/'input_manifest.json')
        cfg2['anchors']=[dict(id=k,pos=v,prior_sigma=.1) for k,v in sorted(geo['anchors'].items())];cfg2['extrinsics']['lever_arm_init']=geo['lever_body_m']
        (d/'config.yaml').write_text(yaml.safe_dump(cfg2,sort_keys=False))
        c=copy.deepcopy(cfg2);c['nlos'].update(mode='disabled',score_recoverability=False,final_inference_enabled=False)
        (d/'cauchy.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
        sf=yaml.safe_load(backend.SF_CONFIG.read_text());sf.update(topic_imu='/rr/imu',topic_ground_truth='/rr/NO_GT',offset=geo['lever_body_m'],
            toa_offset=[0.]*len(geo['anchors']),uwb_frequency=40,imu_frequency=125,acc_ratio=False,gyro_unit=False)
        assert sf['uwb_sample_coeff']==sf['imu_sample_coeff']==1
        (d/'sfuise.yaml').write_text(yaml.safe_dump(sf,sort_keys=False))
        times=[float(v['timestamp']) for v in u if int(v['from_id'])==TAG]
        lock['rows'].append(dict(recording=name,interval_s=[min(times),max(times)],tag_id=TAG,input=str(inp/'input_manifest.json'),
            uwb_count=m['uwb_observation_count'],imu_count=m['imu_count'],measurement_id=m['cache_id']))
        r.write(d/'preparation.json',dict(status='CACHE_PUBLISHED_NOT_CPP_PREPARED',geometry=geo,
            raw_uwb_rows=len(u),selected_tag10_rows=len(times),excluded_tag11_rows=len(u)-len(times),source_index='ZERO_BASED_ORIGINAL_CSV_ROW',
            gt_access_by_estimator=False,raw_ranges_unchanged=True,imu_unchanged=True))
        for fn in ['uwb_range.csv','imu_px4.csv','mocap.csv']:lock['source_hashes'][str(src/fn)]=r.sha(src/fn)
        lock['source_hashes'][str(src.parent/'timeshift.yaml')]=r.sha(src.parent/'timeshift.yaml')
        for p in [*inp.iterdir(),d/'config.yaml',d/'cauchy.yaml',d/'sfuise.yaml']:lock['prepared_hashes'][str(p)]=r.sha(p)
    for p in [SOURCES/'anchors.yaml',SOURCES/'tags.yaml',SOURCES/'experiments.csv',SOURCES/'cleanup_csv.py',SOURCES/'process_uwb.py',
              IMU_SOURCE/'miluv__data.py',IMU_SOURCE/'miluv__utils.py',IMU_SOURCE/'preprocess__read_bags.py',AUDIT/'report-da32f56f/persistent_links.csv']:
        lock['source_hashes'][str(p)]=r.sha(p)
    lock['lock_id']=r.digest(lock);r.write(out/'lock.json',lock);print(out);return 0


def execute(out):
    lock=backend.verify(out);pre=r.read(out/'preflight.json')
    if pre['status']!='PASS' or pre['lock_id']!=lock['lock_id']:raise ValueError('PREFLIGHT_NOT_PASSED')
    for row in pre['rows']:
        if r.sha(out/row['recording']/'measurements.bag')!=row['ros']['bag_sha256']:raise ValueError('ROS_BAG_CHANGED')
    p=out/'execution.json'
    if p.exists():raise ValueError('NO_SCIENCE_RETRY')
    ledger=dict(lock_id=lock['lock_id'],tasks=[],scientific_processes_started=0,status='RUNNING');r.write(p,ledger)
    for name in RECORDINGS:
        cache=None
        for task in r.TASKS:
            if task in r.METHODS[:2] and cache is None:
                result=dict(recording=name,task=task,status='NOT_RUN',reason='PRODUCER_FAILED',exit_code=None)
            else:
                ledger['current_task']=[name,task];ledger['scientific_processes_started']+=1;r.write(p,ledger)
                try:result=backend.call_sfuise(out,name) if task=='SFUISE-ToA' else backend.call_backend(out,name,task,cache if task in r.METHODS[:2] else None)
                except Exception as exc:
                    inv=out/name/(task+'_invocation.json')
                    result=r.read(inv) if inv.exists() else dict(recording=name,task=task,exit_code=None)
                    result.update(status='FAILURE',reason=type(exc).__name__+': '+str(exc))
                if task=='producer' and result['status']=='SUCCESS':cache=Path(result['cache_manifest'])
            if result.get('run_directory'):result['artifact_hashes']=backend.filemap(q for q in Path(result['run_directory']).rglob('*') if q.is_file())
            ledger['tasks'].append(result);r.write(p,ledger);print(name,task,result['status'],result.get('reason'),flush=True)
    ledger['status']='SUCCESS' if all(t['status']=='SUCCESS' for t in ledger['tasks']) else 'COMPLETE_WITH_RETAINED_FAILURES'
    ledger.pop('current_task',None);r.write(p,ledger);return 0


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('stage',choices=['prepare','preflight','execute','evaluate','verify'])
    parser.add_argument('--run',type=Path,required=True);args=parser.parse_args();out=args.run.resolve()
    if args.stage=='prepare':return prepare(out)
    if args.stage=='preflight':return backend.preflight(out)
    if args.stage=='execute':return execute(out)
    if args.stage=='verify':backend.verify(out);print('LOCK_CORE_INPUT_IMPLEMENTATION_PASS');return 0
    from evaluate_miluv_rr import evaluate
    return evaluate(out)

if __name__=='__main__':raise SystemExit(main())
