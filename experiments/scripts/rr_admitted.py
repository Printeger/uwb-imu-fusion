"""GT-assisted, measurement-isolated execution of the locked RR matrix."""
import copy,csv,json,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
import pandas as pd
import yaml
import run_recover_vs_reject as r
from estimator_isolation import sandbox_command
import run_experiments as scheduler

GEOMETRY=r.ROOT/'experiments/recover_vs_reject_gt_geometry.json'
RUNNER=r.WS/'devel/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner'
BUILD=r.WS/'evaluator_private/icra/sfuise_baseline/build_ws'
SF_SOURCE=r.ROOT/'experiments/compare_algorithm/SFUISE'
SF_CONFIG=SF_SOURCE/'sfuise/config/config_test_isas-walk1.yaml'
SF_BINARIES=[BUILD/'devel/lib/sfuise'/n for n in ('EstimationInterface','SplineFusion')]+[BUILD/'devel/lib/sfuise_baseline_adapter/sfuise_trajectory_adapter']


def libs(binary):
    paths=[]
    for line in subprocess.check_output(['ldd',str(binary)],text=True).splitlines():
        if '=> /' in line:
            p=Path(line.split('=>',1)[1].strip().split()[0])
            if not str(p).startswith(('/usr/','/lib/','/lib64/')):paths.append(p)
    return paths


def filemap(paths):return {str(p):r.sha(p) for p in paths}


def core_files():
    f={str(r.ROOT/k):v for k,v in r.frozen_files().items()}
    for p in [RUNNER,*libs(RUNNER),*SF_BINARIES,SF_CONFIG]:f[str(p)]=r.sha(p)
    return f


def prepare(out):
    out.mkdir(parents=True,exist_ok=False);geo=r.read(GEOMETRY);cfg=yaml.safe_load(r.CONFIG.read_text())
    assert cfg['nlos']['cusum_forward_kappa']==cfg['nlos']['cusum_backward_kappa']==.5
    assert cfg['keyframe']['step']==4
    assert cfg['nlos']['cusum_forward_h']==7.0234689587858723 and cfg['nlos']['cusum_backward_h']==7.0234689587858714
    anchors=pd.read_csv(r.ROOT/'data/starloc/mocap/uwb_markers_v2.csv',index_col=0)
    params={v['name']:v for v in r.read(r.ROOT/'data/starloc/dataset_params.json')}
    lock=dict(schema='RR_ADMITTED_RUN_V2',role='GT_ASSISTED_DEVELOPMENT',recordings=list(r.RECORDINGS),methods=list(r.METHODS),
       evaluation=r.EVALUATION,geometry=geo,rows=[],core_files=core_files(),source_hashes={},prepared_hashes={},
       implementation=filemap([Path(__file__),Path(r.__file__),r.ROOT/'experiments/scripts/evaluate_recover_vs_reject.py',
         r.ROOT/'experiments/scripts/rr_evaluate.py',r.ROOT/'experiments/scripts/rr_sfuise_worker.py',GEOMETRY,
         r.ROOT/'experiments/RECOVER_VS_REJECT_GT_ADMISSION.md',
         r.ROOT/'experiments/scripts/test_rr_admitted.py',r.ROOT/'experiments/scripts/test_recover_vs_reject.py']),
       scientific_budget=dict(max_tasks=15,seconds_per_tree=1800,retries=0))
    if subprocess.check_output(['git','-C',str(SF_SOURCE),'rev-parse','HEAD'],text=True).strip()!='75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d':raise ValueError('SFUISE_VERSION_CHANGED')
    for name in r.RECORDINGS:
        assert params[name]['landmarks']=='v2'
        src=r.ROOT/'data/starloc/data'/name;d=out/name;d.mkdir();inp=d/'input';inp.mkdir()
        uwb=r.projection(src/'uwb.csv',r.UWB_COLUMNS);imu=r.projection(src/'imu.csv',r.IMU_COLUMNS)
        m,u,im=r.cache_payloads(name,uwb,imu,geo['recordings'][name])
        (inp/'uwb_observations.csv').write_bytes(u);(inp/'imu.csv').write_bytes(im);r.write(inp/'input_manifest.json',m)
        c=copy.deepcopy(cfg);c['dataset']['cache_manifest']=str(inp/'input_manifest.json')
        c['anchors']=[dict(id=int(a),pos=row.to_numpy(float).tolist(),prior_sigma=.1) for a,row in anchors.iterrows()]
        c['extrinsics']['lever_arm_init']=geo['lever_body_m'];(d/'config.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
        b=copy.deepcopy(c);b['nlos'].update(mode='disabled',score_recoverability=False,final_inference_enabled=False)
        (d/'cauchy.yaml').write_text(yaml.safe_dump(b,sort_keys=False))
        sf=yaml.safe_load(SF_CONFIG.read_text());sf.update(topic_imu='/rr/imu',topic_ground_truth='/rr/NO_GT',
            offset=geo['lever_body_m'],toa_offset=[0.]*len(anchors),uwb_frequency=250,imu_frequency=200,acc_ratio=False,gyro_unit=False)
        (d/'sfuise.yaml').write_text(yaml.safe_dump(sf,sort_keys=False))
        selected=[v for v in uwb if int(v['from_id'])==1];times=[float(v['time_s']) for v in selected]
        r.write(d/'preparation.json',dict(status='CACHE_PUBLISHED_NOT_YET_CPP_PREPARED',geometry_status='GT_ASSISTED_APPROXIMATE',
          input_manifest_sha256=r.sha(inp/'input_manifest.json'),quality_fields='NEUTRAL_ZERO_PLACEHOLDERS',
          raw_source_row_mapping='source_message_index=zero_based_source_csv_row; source_range_index=0',beta='UNCORRECTED_MISSING_CALIBRATION'))
        lock['rows'].append(dict(recording=name,status='ADMITTED_GT_ASSISTED_APPROXIMATE',interval_s=[min(times),max(times)],
            uwb_count=len(selected),imu_count=len(imu),input=str(inp/'input_manifest.json'),measurement_id=m['cache_id']))
        for fn in ['uwb.csv','imu.csv','calib.json']:lock['source_hashes'][str(src/fn)]=r.sha(src/fn)
        for p in [*inp.iterdir(),d/'config.yaml',d/'cauchy.yaml',d/'sfuise.yaml']:lock['prepared_hashes'][str(p)]=r.sha(p)
    for p in [r.ROOT/'data/starloc/mocap/uwb_markers_v2.csv',r.ROOT/'data/starloc/dataset_params.json']:lock['source_hashes'][str(p)]=r.sha(p)
    lock['lock_id']=r.digest(lock);r.write(out/'lock.json',lock);print(str(out));return 0


def verify(out):
    lock=r.read(out/'lock.json');v=dict(lock);h=v.pop('lock_id')
    if r.digest(v)!=h:raise ValueError('LOCK_TAMPERED')
    for k in ['core_files','source_hashes','implementation','prepared_hashes']:
        for path,sha in lock[k].items():
            if r.sha(path)!=sha:raise ValueError('IDENTITY_CHANGED:'+path)
    return lock


def backend_command(out,name,task,prepare_only=False,cache=None):
    d=out/name;config=d/('cauchy.yaml' if task=='robust_cauchy' else 'config.yaml')
    inp=d/'input';root=d/('prepare_runs' if prepare_only else 'backend');root.mkdir(exist_ok=True)
    command=[str(RUNNER),'--config',str(config),'--output-root',str(root),'--run-id',task]
    if prepare_only:command+=['--prepare-only']
    elif task=='producer':command+=['--execution-type','CACHE_PRODUCER']
    else:
        command+=['--method',task,'--execution-type','BASELINE_TRAJECTORY' if task=='robust_cauchy' else 'FINAL_TRAJECTORY']
        if cache:command+=['--stage2-cache-manifest',str(cache),'--operating-point-id','rr-'+task]
    files=[config,*inp.iterdir(),RUNNER,*libs(RUNNER)]
    if cache:
        doc=r.read(cache);files+=[cache]
        if doc['stage1_provider']!='PL_BIDIRECTIONAL_CUSUM_V1' or doc['cache_id']!=scheduler.stage2_cache_identity(doc):raise ValueError('WRONG_CACHE_IDENTITY')
        for payload in doc['payloads']:
            p=cache.parent/payload['name']
            if p.parent!=cache.parent or 'sha256:'+r.sha(p)!=payload['sha256']:raise ValueError('CACHE_PAYLOAD_MISMATCH')
            files.append(p)
    wrapped=sandbox_command(command,files,root)
    wrapped=wrapped[:-len(command)]+['/usr/bin/env','-i','PATH=/usr/bin:/bin','LANG=C','HOME=/tmp','LD_LIBRARY_PATH='+':'.join(sorted({str(p.parent) for p in libs(RUNNER)})),
            'UIFGO_T09_ROBUST_SCALE=2.3849',*command]
    return wrapped,root/task,files


def ros_bag(d):
    import rosbag,rospy
    from sensor_msgs.msg import Imu
    from isas_msgs.msg import RTLSStick,RTLSRange,Anchorlist,AnchorPosition
    cfg=yaml.safe_load((d/'config.yaml').read_text());u=pd.read_csv(d/'input/uwb_observations.csv');im=pd.read_csv(d/'input/imu.csv')
    messages=[]
    for row in im.itertuples():
        msg=Imu();msg.header.stamp=rospy.Time.from_sec(row.sensor_time_s);msg.header.frame_id='imu_origin_rig_axes'
        msg.orientation_covariance[0]=-1
        for a in 'xyz':setattr(msg.linear_acceleration,a,getattr(row,'acc_'+a+'_mps2'));setattr(msg.angular_velocity,a,getattr(row,'gyro_'+a+'_radps'))
        messages.append((row.sensor_time_s,'/rr/imu',msg))
    for row in u.itertuples():
        msg=RTLSStick();msg.header.stamp=rospy.Time.from_sec(row.sensor_time_s);msg.header.seq=int(row.source_message_index);msg.id=int(row.tag_id)
        rg=RTLSRange();rg.id=int(row.anchor_id);rg.range=row.observed_range_m;rg.ra=int(row.source_valid);msg.ranges=[rg]
        messages.append((row.sensor_time_s,'/rtls_flares',msg))
    al=Anchorlist()
    for a in sorted(cfg['anchors'],key=lambda a:a['id']):
        ap=AnchorPosition();ap.id=a['id'];ap.position.x,ap.position.y,ap.position.z=a['pos'];al.anchor.append(ap)
    lo=min(v[0] for v in messages);hi=max(v[0] for v in messages)
    for t in np.arange(max(0,lo),hi,1.):messages.append((t,'/anchor_list',al))
    path=d/'measurements.bag'
    with rosbag.Bag(str(path),'w') as bag:
        for t,topic,msg in sorted(messages,key=lambda x:x[0]):bag.write(topic,msg,rospy.Time.from_sec(t))
    ui=ii=0;maxerr=0.;maxdt=0.
    with rosbag.Bag(str(path)) as bag:
        for topic,msg,stamp in bag.read_messages():
            if topic=='/rtls_flares':
                row=u.iloc[ui];ui+=1
                assert msg.header.seq==row.source_message_index and msg.id==row.tag_id and len(msg.ranges)==1 and msg.ranges[0].id==row.anchor_id
                assert msg.ranges[0].range==float(np.float32(row.observed_range_m))
                maxerr=max(maxerr,abs(msg.ranges[0].range-row.observed_range_m));maxdt=max(maxdt,abs(msg.header.stamp.to_sec()-row.sensor_time_s))
            elif topic=='/rr/imu':
                row=im.iloc[ii];ii+=1
                for a in 'xyz':assert getattr(msg.linear_acceleration,a)==row['acc_'+a+'_mps2'] and getattr(msg.angular_velocity,a)==row['gyro_'+a+'_radps']
                maxdt=max(maxdt,abs(msg.header.stamp.to_sec()-row.sensor_time_s))
            elif topic=='/anchor_list':assert len(msg.anchor)==len(cfg['anchors'])
            else:raise ValueError('UNEXPECTED_TOPIC')
    assert ui==len(u) and ii==len(im) and maxdt<=1.01e-9
    result=dict(status='PASS',uwb_rows=ui,imu_rows=ii,max_range_float32_error_m=maxerr,max_timestamp_error_s=maxdt,
                bag_sha256=r.sha(path),GT_topics=[],support_inputs=[],body='IMU_ORIGIN_RIG_AXES')
    r.write(d/'ros_roundtrip.json',result);return result


def preflight(out):
    lock=verify(out);p=out/'preflight.json'
    if p.exists():raise ValueError('PREFLIGHT_ALREADY_EXISTS')
    rows=[]
    for row in lock['rows']:
        name=row['recording'];command,run,files=backend_command(out,name,'prepare',True)
        process=r.science_process(command,out/name/'prepare_process',timeout=1800)
        status=r.read(run/'run_status.json') if (run/'run_status.json').exists() else {}
        entry=dict(recording=name,process=process,backend=status,allowlist=filemap(files))
        if process['exit_code']==0 and status.get('optimizer_calls')==0:
            bc,br,bf=backend_command(out,name,'robust_cauchy',True)
            bp=r.science_process(bc,out/name/'cauchy_prepare_process',timeout=1800)
            entry['cauchy_prepare']=bp
            if bp['exit_code']!=0 or r.read(br/'common_preparation.json')!=r.read(run/'common_preparation.json'):
                raise ValueError('CAUCHY_COMMON_PREPARATION_MISMATCH')
            entry['ros']=ros_bag(out/name)
            entry['sfuise_startup']=call_sfuise(out,name,probe=True)
        rows.append(entry);r.write(p,dict(status='IN_PROGRESS',rows=rows,lock_id=lock['lock_id']))
    ok=all(x['process']['exit_code']==0 and x.get('ros',{}).get('status')=='PASS' and x.get('sfuise_startup',{}).get('status')=='SUCCESS' for x in rows)
    # Freeze the only ROS input after actual full-source roundtrip, before science.
    doc=dict(status='PASS' if ok else 'FAILED',rows=rows,lock_id=lock['lock_id'],scientific_tasks=0)
    r.write(p,doc);print(json.dumps(doc));return 0 if ok else 1


def call_backend(out,name,task,cache=None):
    command,run,files=backend_command(out,name,task,cache=cache)
    proc=r.science_process(command,out/name/(task+'_process'))
    status=r.read(run/'run_status.json') if (run/'run_status.json').exists() else {}
    result=dict(task=task,recording=name,status='SUCCESS' if proc['exit_code']==0 else proc['status'],exit_code=proc['exit_code'],
        reason=status.get('reason',status.get('failure_reason','')),run_directory=str(run),process=proc,allowlist=filemap(files))
    r.write(out/name/(task+'_invocation.json'),result)
    if proc['exit_code']!=0:return result
    common=r.read(run/'common_preparation.json')
    expected=r.read(out/name/'prepare_runs/prepare/common_preparation.json')
    if common!=expected:raise ValueError('PREPARED_INITIAL_STATE_CHANGED')
    if task=='producer':
        cfg=yaml.safe_load((out/name/'config.yaml').read_text())
        doc=scheduler.publish_cache(run,out/name/'caches','PL_BIDIRECTIONAL_CUSUM',common['common_preparation_id'],
            scheduler.common_config_identity(cfg),scheduler.stage2_producer_config_identity(cfg,out/name),False,scheduler.producer_provenance(RUNNER))
        result['cache_manifest']=str(out/name/'caches'/doc['cache_id'].split(':')[-1]/'stage2_cache_manifest.json')
    if cache:
        doc=r.read(cache)
        if common['common_preparation_id']!=doc['common_preparation_id']:raise ValueError('COMMON_PREPARATION_CHANGED')
        result['cache_manifest']=str(cache);result['cache_id']=doc['cache_id'];result['stage2_values_sha256']=doc['stage2_values_sha256']
        ok,reason=scheduler.validate_final_success(run,task,doc['cache_id'],'rr-'+task,
              {k:yaml.safe_load((out/name/'config.yaml').read_text())['nlos'][k] for k in ['tau_eta','tau_s_m','tau_gamma']})
        if not ok:raise ValueError(reason)
    result['common_preparation']=common
    r.write(out/name/(task+'_invocation.json'),result)
    return result


def call_sfuise(out,name,probe=False):
    d=out/name;root=d/('sfuise_probe' if probe else 'sfuise');root.mkdir()
    scripts=[r.ROOT/'experiments/scripts/rr_sfuise_worker.py']
    files=[d/'measurements.bag',d/'sfuise.yaml',*SF_BINARIES,*scripts]
    for b in SF_BINARIES:files+=libs(b)
    command=['/usr/bin/python3',str(scripts[0]),str(d),str(BUILD)]+(['--probe'] if probe else [])
    wrapped=sandbox_command(command,files,root)
    # ROS runtime/package metadata are required; data, GT and other method outputs remain absent.
    idx=wrapped.index('--');wrapped[idx:idx]=['--ro-bind','/opt/ros/noetic','/opt/ros/noetic','--ro-bind','/etc/hosts','/etc/hosts',
        '--ro-bind','/etc/resolv.conf','/etc/resolv.conf','--ro-bind','/etc/alternatives','/etc/alternatives']
    process=r.science_process(wrapped,d/('sfuise_probe_process' if probe else 'sfuise_process'),timeout=120 if probe else 1800)
    status=r.read(root/'run_status.json') if (root/'run_status.json').exists() else {}
    return dict(recording=name,task='SFUISE-ToA',status='SUCCESS' if process['exit_code']==0 and status.get('status')=='SUCCESS' else 'FAILURE',
        exit_code=process['exit_code'],reason=status.get('reason',''),run_directory=str(root),process=process,allowlist=filemap(files))


def execute(out):
    lock=verify(out);pre=r.read(out/'preflight.json')
    if pre['status']!='PASS' or pre['lock_id']!=lock['lock_id']:raise ValueError('PREFLIGHT_NOT_PASSED')
    for row in pre['rows']:
        if r.sha(out/row['recording']/'measurements.bag')!=row['ros']['bag_sha256']:raise ValueError('ROS_BAG_CHANGED')
    path=out/'execution.json'
    if path.exists():raise ValueError('NO_SCIENCE_RETRY')
    ledger=dict(lock_id=lock['lock_id'],tasks=[],scientific_processes_started=0,status='RUNNING')
    r.write(path,ledger)
    for name in r.RECORDINGS:
        cache=None
        for task in r.TASKS:
            if task in r.METHODS[:2] and cache is None:
                result=dict(recording=name,task=task,status='NOT_RUN',reason='PRODUCER_FAILED',exit_code=None)
            else:
                ledger['scientific_processes_started']+=1;ledger['current_task']=[name,task];r.write(path,ledger)
                try:result=call_sfuise(out,name) if task=='SFUISE-ToA' else call_backend(out,name,task,cache if task in r.METHODS[:2] else None)
                except Exception as exc:
                    result=dict(recording=name,task=task,status='FAILURE',reason=type(exc).__name__+': '+str(exc),exit_code=None)
                if task=='producer' and result['status']=='SUCCESS':cache=Path(result['cache_manifest'])
            if result.get('run_directory'):
                result['artifact_hashes']=filemap(p for p in Path(result['run_directory']).rglob('*') if p.is_file())
            ledger['tasks'].append(result);r.write(path,ledger);print(name,task,result['status'],result.get('reason'),flush=True)
    ledger['status']='COMPLETE_WITH_RETAINED_FAILURES' if any(x['status']!='SUCCESS' for x in ledger['tasks']) else 'SUCCESS'
    ledger.pop('current_task',None);r.write(path,ledger);return 0


def evaluate(out):
    from rr_evaluate import evaluate as fn
    return fn(out)
