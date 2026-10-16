#!/usr/bin/env python3
"""Bounded Walk follow-up. All generated inputs/logs belong to this cache only."""
import argparse
import hashlib
import json
import subprocess
import time
import resource
import shutil
from pathlib import Path
import numpy as np
import pandas as pd
import yaml
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation, Slerp
from scipy.optimize import minimize_scalar
import benchmark_common as common
import run_dataset_benchmark as datasets
from sc4_system_research import BIN, BUILD, environment

ROOT = common.ROOT
CACHE = ROOT / '.cache/sc8_walk_followup'
CONTRACT = ROOT / 'config/sc8_walk_followup.yaml'
REPORT = ROOT / 'docs/sc8_followup/SC8_WALK_RESULTS.md'


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def sensor_hash(path):
    return {n: common.sha256(path/n) for n in ('imu.csv','uwb.csv','anchors.csv')}


def gyro_pairs(imu, gt, offset):
    ts = imu.t.to_numpy()[::5]
    gyro = imu[['gx','gy','gz']].to_numpy()[::5]
    t = ts + offset
    good = (t-.03 > gt.t.min()) & (t+.03 < gt.t.max())
    s = Slerp(gt.t, Rotation.from_quat(gt[['qx','qy','qz','qw']]))
    omega = (s(t[good]-.03).inv()*s(t[good]+.03)).as_rotvec()/.06
    return gyro[good], omega


def cal_diagnostic(path):
    imu = pd.read_csv(path/'imu.csv')
    gt = pd.read_csv(path/'gt.csv').drop_duplicates('t')
    # Temporal halves fixed before fitting: training first 60%, held-out last 40%.
    cut = float(imu.t.min()+.6*(imu.t.max()-imu.t.min()))
    train = imu[imu.t < cut-.1]
    hold = imu[imu.t > cut+.1]
    def fit(offset):
        a,b = gyro_pairs(train,gt,offset)
        r,e = datasets._wahba(a-a.mean(0),b-b.mean(0))
        return r, b.mean(0)-r@a.mean(0), e
    result = minimize_scalar(lambda x: fit(x)[2], bounds=(-.1,.1), method='bounded', options={'xatol':1e-5})
    r,bias,err = fit(float(result.x))
    a,b = gyro_pairs(hold,gt,float(result.x))
    a0,b0 = gyro_pairs(imu,gt,0.)
    return dict(scope='Walk1 CAL only; diagnostic, never applied to estimator or TEST GT',
        training_end_s=cut, time_gt_minus_imu_s=float(result.x), search_interval_s=[-.1,.1],
        R_tracker_from_imu=r.tolist(), tracker_gyro_offset_radps=bias.tolist(),
        fit_rmse_radps=err, heldout_rmse_radps=float(np.sqrt(np.mean(np.sum((a@r.T+bias-b)**2,axis=1)))),
        identity_rmse_radps=float(np.sqrt(np.mean(np.sum((a0-b0)**2,axis=1)))),
        excitation_singular_values=np.linalg.svd(a0-a0.mean(0),compute_uv=False).tolist(),
        determinant=float(np.linalg.det(r)), orthogonality_error=float(np.linalg.norm(r.T@r-np.eye(3))),
        translation_identification='NOT_CLOSED: gyro alone has zero translation Jacobian; no independent tracker-to-tag dimensions or time synchronization uncertainty',
        qualification='REFERENCE_OPEN')


def prepare():
    cfg = datasets.load_config(ROOT/'config/benchmark/sfuise.yaml')
    inventory = []
    for n in (1,2,3):
        seq = cfg['sequences'][n-1]
        path = CACHE/'data'/seq['id']
        if not (path/'manifest.json').exists():
            datasets.prepare_sfuise(cfg,seq,path)
            # Preserve actual tracker pose. The legacy adapter's lever projection
            # was a proxy, not measured physical tag truth. Read source directly.
            rows=[]
            source=Path(cfg['_data_root'])/seq['path']
            times=[]
            for topic,ns,msg in datasets.bag_messages(source,{cfg['imu']['topic'],cfg['uwb']['topic'],cfg['ground_truth']['topic']}):
                t=common.header_time(msg,ns);times.append(t)
                if topic==cfg['ground_truth']['topic']:
                    rows.append((t,datasets.v3(msg.transform.translation),datasets.q4(msg.transform.rotation)))
            origin=min(times)
            common.write_csv(path/'gt.csv',common.GT_FIELDS,[dict(zip(common.GT_FIELDS,[t-origin,*p,*q,cfg['primary_tag']])) for t,p,q in rows])
            manifest=json.loads((path/'manifest.json').read_text())
            manifest.update(reference_qualification='REFERENCE_OPEN',split='CAL' if n==1 else 'TEST',raw_qualified=False)
            manifest['provenance']['gt_reference']='MEASURED_VIVE_TRACKER_ONLY_NOT_IMU_OR_TAG'
            manifest['files']=datasets.rows_file_metadata(path)
            save(path/'manifest.json',manifest)
        manifest=json.loads((path/'manifest.json').read_text())
        with AnyReader([Path(manifest['source'])]) as reader:
            topics={c.topic:dict(type=c.msgtype,count=c.msgcount) for c in reader.connections}
        imu=pd.read_csv(path/'imu.csv');uwb=pd.read_csv(path/'uwb.csv');gt=pd.read_csv(path/'gt.csv')
        inventory.append(dict(sequence=seq['id'],split=manifest['split'],source_sha256=manifest['source_sha256'],
            files=datasets.rows_file_metadata(path),topics=topics,imu_hz=float(1/np.median(np.diff(imu.t))),
            imu_max_gap_s=float(np.diff(imu.t).max()),imu_duration_s=float(imu.t.max()-imu.t.min()),
            initial_accel_norm_median=float(np.median(np.linalg.norm(imu[['ax','ay','az']].iloc[:160],axis=1))),
            gt_max_gap_s=float(np.diff(gt.t).max()),raw_batches=int(uwb.source_message.nunique()),
            invalid_ranges=int((uwb.valid==0).sum()),valid_batches=int(uwb[uwb.valid!=0].source_message.nunique()),
            anchor_ids=sorted(map(int,uwb.anchor_id.unique())),tag_ids=sorted(map(int,uwb.tag_id.unique())),
            max_range_offset_error=float(np.max(np.abs(uwb['range']-uwb.range_raw-uwb.anchor_id.map({int(k):v for k,v in manifest['provenance']['toa_offset_by_anchor'].items()}))))))
    registry=dict(schema='WalkReferenceContractV1',entry_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        source_converter_sha256=common.sha256(Path(datasets.__file__)),inventory=inventory,
        reference_qualification='REFERENCE_OPEN',raw_qualified=False,hardware_protection=False,
        convention='T_AB maps B to A; T_WI=T_WT*T_TI; p_Wtag=p_WI+R_WI*l_Itag',
        known_imu_tag_offset_m=cfg['uwb']['lever_arm_body_m'],known_offset_evidence='SFUISE pinned official config; no independent uncertainty',
        missing=['T_TI translation and independently verified rotation','tracker-to-tag measurement definition','clock offset/drift and uncertainty','gravity/world rotation uncertainty'],
        cal_diagnostic=cal_diagnostic(CACHE/'data/ISAS-Walk1'),
        diagnostics=dict(walk2_prefix_batches=[1,120],walk2_later_batches=[200,240],selection='source order, no GT selection'),
        limits=dict(run_wall_s=1200,rss_bytes=7*1024**3),
        injection_registration=dict(type='research stress fault, not empirical probability law',
            weak=dict(range_m=.3,accel_mps2=2.,gyro_radps=.1),strong=dict(range_m=2.25,accel_mps2=20.,gyro_radps=1.),
            weak_start_s=20.,strong_start_s=40.,imu_duration_s=.5,uwb_batches_weak=1,uwb_batches_strong=3,
            anchor_id=7475,imu_axis='original_sensor_x',joint_uwb_offset_s=-.18,
            walk2=['U','A','G','J'],walk3=['J'],joint_imu='gyro'),
        identity=dict(binaries={str(p):common.sha256(p) for p in [BIN/'nominal_dataset_runner',BIN.parent/'libuwb_imu_pl.so']},
            sfuise_pin=subprocess.check_output(['git','-C',str(ROOT.parent/'SFUISE'),'rev-parse','HEAD'],text=True).strip()))
    if CONTRACT.exists():
        old=yaml.safe_load(CONTRACT.read_text())
        assert old['inventory']==registry['inventory'],'Frozen source inventory changed'
    else:
        CONTRACT.write_text(yaml.safe_dump(registry,sort_keys=False,allow_unicode=True))
    save(CACHE/'w0.json',registry)
    print(json.dumps(dict(inventory=inventory,cal=registry['cal_diagnostic']),indent=2))


def run_native(sequence, method, source=None, label=None, paced=True, audit=False, best_effort=False, pruning=False, fixed_config=None):
    import sc8_experiments as old
    source=source or CACHE/'data'/sequence
    run=CACHE/'runs'/(label or (sequence+'-'+method))
    if (run/'execution.json').exists():
        return json.loads((run/'execution.json').read_text())
    run.mkdir(parents=True,exist_ok=True)
    entry=dict(requested_method='O-FULL' if method=='FDE' else 'O-NOFDE',window=[30,30],stream=sequence)
    config=old.dataset_config(entry,source,run)
    if fixed_config is not None:config.write_text(Path(fixed_config).read_text())
    if audit:
        cfg=yaml.safe_load(config.read_text());cfg['output'].update(write_candidates=True,write_hypothesis_evidence=True)
        config.write_text(yaml.safe_dump(cfg,sort_keys=False))
    args=[str(BIN/'nominal_dataset_runner'),'--config',str(config),'--input',str(source),'--output',str(run),
          '--tag-policy','primary','--causal-bootstrap']
    if method!='NOM':args+=['--pipeline-mode','on' if method=='FDE' else 'off']
    if paced:args+=['--paced']
    if audit:args+=['--engineering-audit']
    if best_effort:args+=['--research-best-effort']
    if pruning:args+=['--research-eligibility-pruning']
    identity_before=dict(binary_sha256=common.sha256(BIN/'nominal_dataset_runner'),dso_sha256=common.sha256(BIN.parent/'libuwb_imu_pl.so'))
    begin=time.monotonic();before=resource.getrusage(resource.RUSAGE_CHILDREN)
    with (run/'process.log').open('w') as log:
        proc=subprocess.Popen(args,env=dict(environment(),UWB_IMU_PL_CANDIDATE_WORKERS='1'),stdout=log,stderr=subprocess.STDOUT)
        sampled_rss_kib=0;status='FINISHED'
        while proc.poll() is None:
            try:
                lines=Path('/proc')/str(proc.pid)/'status'
                rss=next(int(x.split()[1]) for x in lines.read_text().splitlines() if x.startswith('VmRSS:'))
                sampled_rss_kib=max(sampled_rss_kib,rss)
            except (OSError,StopIteration):pass
            if time.monotonic()-begin>1200 or sampled_rss_kib*1024>7*1024**3:
                status='RESOURCE_LIMIT';proc.terminate()
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
                break
            time.sleep(.2)
        code=proc.returncode
        if code and status=='FINISHED':status='FAIL'
    after=resource.getrusage(resource.RUSAGE_CHILDREN)
    result=dict(command=args,returncode=code,status=status,wall_s=time.monotonic()-begin,
        cpu_s=after.ru_utime+after.ru_stime-before.ru_utime-before.ru_stime,
        sensor_hash=sensor_hash(source),binary_sha256=common.sha256(BIN/'nominal_dataset_runner'),
        dso_sha256=common.sha256(BIN.parent/'libuwb_imu_pl.so'),sampled_rss_kib=sampled_rss_kib,identity_before=identity_before)
    assert result['binary_sha256']==identity_before['binary_sha256'] and result['dso_sha256']==identity_before['dso_sha256']
    save(run/'execution.json',result)
    print(sequence,method,status,code,flush=True)
    return result


def noninterference():
    source=CACHE/'data/ISAS-Walk1';pair=[]
    for variant in ('original','future-gt-tampered'):
        dest=CACHE/'noninterference'/variant;dest.mkdir(parents=True,exist_ok=True)
        for name in ('imu.csv','uwb.csv','gt.csv'):
            df=pd.read_csv(source/name);df=df[df.t<=7.]
            if name=='gt.csv' and variant!='original':
                df.loc[df.t>5.,'px']=1e6
            df.to_csv(dest/name,index=False,float_format='%.17g')
        for name in ('anchors.csv','manifest.json'):shutil.copy2(source/name,dest/name)
        run_native('ISAS-Walk1','NOM',dest,'noninterference-v2-'+variant,paced=False)
        pair.append(CACHE/'runs'/('noninterference-v2-'+variant))
    # Whole state output and initialization must be bit-identical; timing excluded.
    names=['trajectory.tum','bootstrap.json']
    for name in names:assert (pair[0]/name).read_bytes()==(pair[1]/name).read_bytes(),name
    a=pd.read_csv(pair[0]/'attempts.csv');b=pd.read_csv(pair[1]/'attempts.csv')
    cols=['batch','t','committed','backend_updates','transaction_id','action_id','action','fde_status','max_sensor_stamp_used']
    pd.testing.assert_frame_equal(a[cols],b[cols])
    save(CACHE/'noninterference.json',dict(status='PASS',states=len(a),compared=names+cols))


def baseline():
    for seq in ('ISAS-Walk2','ISAS-Walk3'):
        for method in ('NOM','NOFDE'):run_native(seq,method)


def sfuise_baseline():
    import run_sfuise_benchmark as sf
    sf.checked_pin()
    setup=sf.workspace(CACHE)/'devel/setup.bash'
    for seq in ('ISAS-Walk2','ISAS-Walk3'):
        run=CACHE/'runs'/(seq+'-SFUISE')
        if (run/'run_status.json').exists():continue
        result=sf.run_one(setup,CACHE/'data'/seq,run,playback_rate=1.,fusion_executable='sfuise_sc8_observed',observer=True)
        save(run/'input_identity.json',sensor_hash(CACHE/'data'/seq))
        print(seq,result['status'],flush=True)


def summarize_native(run, sequence):
    a=pd.read_csv(run/'attempts.csv')
    u=pd.read_csv(CACHE/'data'/sequence/'uwb.csv')
    imu=pd.read_csv(CACHE/'data'/sequence/'imu.csv')
    boot=json.loads((run/'bootstrap.json').read_text())
    planned=u[u.valid!=0].groupby('source_message').t.max()
    ready=planned[(planned>boot['timestamp_s']) & (planned<=imu.t.max())]
    c=a[a.committed==1]
    assert (a.backend_updates<=1).all() and c.transaction_id.is_unique and c.t.is_unique
    assert (a.max_sensor_stamp_used>=a.t-1e-8).all()
    def stats(x):
        x=np.asarray(x);x=x[np.isfinite(x)]
        return dict(zip(['p50','p95','p99','max'],map(float,np.sort(x)[np.maximum(0,np.ceil(np.array([.5,.95,.99,1])*len(x)).astype(int)-1)]))) if len(x) else None
    status=json.loads((run/'run_status.json').read_text()) if (run/'run_status.json').exists() else {}
    execution=json.loads((run/'execution.json').read_text())
    return dict(raw_planned=int(u.source_message.nunique()),planned_valid=len(planned),eligible_planned=len(ready),
        startup=int((planned<=boot['timestamp_s']).sum()),after_imu_end=int((planned>imu.t.max()).sum()),
        wholly_invalid=int(u.source_message.nunique()-len(planned)),attempted=len(a),committed=len(c),
        coverage_all_valid=len(c)/len(planned),coverage_eligible=len(c)/len(ready),
        rejected=len(a)-len(c),unattempted_eligible=len(ready)-len(a),
        interval_s=[float(c.t.min()),float(c.t.max())],bootstrap=boot,
        statuses={str(k):int(v) for k,v in a.fde_status.value_counts().items()},
        core_ms=stats(a.core_ms),sink_ms=stats(a.measurement_to_sink_ms),
        queue_ms=stats(np.maximum(0,a.measurement_to_sink_ms-a.core_ms)),
        cpu_s=status['cpu_ms']/1000 if 'cpu_ms' in status else execution['cpu_s'],peak_rss_kib=status.get('peak_rss_kib',execution.get('sampled_rss_kib')),
        rss_scope='native process peak' if 'peak_rss_kib' in status else 'sampled RSS lower bound',
        source_complete=len(a)==len(ready),status=execution['status'],
        service_RTF=float(a.core_ms.sum()/1000/(imu.t.max()-imu.t.min())),
        physical_APE=None,physical_ARE=None,physical_RPE=None,reference='REFERENCE_OPEN',raw_qualified=False)


def summarize_baseline():
    results={}
    for seq in ('ISAS-Walk2','ISAS-Walk3'):
        for method in ('NOM','NOFDE'):
            run=CACHE/'runs'/(seq+'-'+method)
            results[seq+'-'+method]=summarize_native(run,seq)
        a=pd.read_csv(CACHE/'runs'/(seq+'-NOM')/'states.csv')
        b=pd.read_csv(CACHE/'runs'/(seq+'-NOFDE')/'states.csv')
        pd.testing.assert_frame_equal(a,b,check_exact=True)
    save(CACHE/'baseline_summary.json',results)
    print(json.dumps(results,indent=2))


def inject():
    reg=yaml.safe_load(CONTRACT.read_text())['injection_registration']
    for seq,cases in [('ISAS-Walk2',reg['walk2']),('ISAS-Walk3',reg['walk3'])]:
        base=CACHE/'data'/seq
        for case in cases:
            dest=CACHE/'injected'/(seq+'-'+case)
            if (dest/'injection.json').exists():continue
            dest.mkdir(parents=True,exist_ok=True)
            imu=pd.read_csv(base/'imu.csv',float_precision='round_trip')
            uwb=pd.read_csv(base/'uwb.csv',float_precision='round_trip')
            episodes=[]
            for strength in ('weak','strong'):
                t=reg[strength+'_start_s'];amplitudes=reg[strength]
                if case in ('U','J'):
                    begin=t+(reg['joint_uwb_offset_s'] if case=='J' else 0)
                    # Select source IDs irrespective of validity; retain all original flags.
                    selected=uwb[(uwb.t>=begin)&(uwb.anchor_id==reg['anchor_id'])].iloc[:reg['uwb_batches_'+strength]]
                    uwb.loc[selected.index,'range']+=amplitudes['range_m']
                    uwb.loc[selected.index,'range_raw']+=amplitudes['range_m']
                    episodes.append(dict(strength=strength,sensor='UWB',begin_s=float(selected.t.min()),end_s=float(selected.t.max()),
                        amplitude=amplitudes['range_m'],anchor_id=reg['anchor_id'],
                        source_ids=selected[['source_message','source_range','valid']].astype(int).values.tolist()))
                if case in ('A','G','J'):
                    field='ax' if case=='A' else 'gx';amp=amplitudes['accel_mps2' if case=='A' else 'gyro_radps']
                    mask=(imu.t>=t)&(imu.t<t+reg['imu_duration_s'])
                    imu.loc[mask,field]+=amp
                    episodes.append(dict(strength=strength,sensor='IMU',field=field,amplitude=amp,begin_s=t,end_s=t+reg['imu_duration_s'],
                        source_ids=(np.flatnonzero(mask)+1).tolist(),sample_times=imu.loc[mask,'t'].tolist()))
            imu.to_csv(dest/'imu.csv',index=False,float_format='%.17g')
            uwb.to_csv(dest/'uwb.csv',index=False,float_format='%.17g')
            for name in ('gt.csv','anchors.csv','manifest.json'):shutil.copy2(base/name,dest/name)
            save(dest/'injection.json',dict(base_hash=sensor_hash(base),injected_hash=sensor_hash(dest),episodes=episodes,
                gt_unchanged=common.sha256(base/'gt.csv')==common.sha256(dest/'gt.csv'),label_access='scorer only'))
            # Read back and compare every cell. Labels never enter online manifest.
            orig_i=pd.read_csv(base/'imu.csv',float_precision='round_trip')
            orig_u=pd.read_csv(base/'uwb.csv',float_precision='round_trip')
            actual_i=pd.read_csv(dest/'imu.csv',float_precision='round_trip')
            actual_u=pd.read_csv(dest/'uwb.csv',float_precision='round_trip')
            np.testing.assert_array_equal(actual_i.to_numpy(),imu.to_numpy())
            np.testing.assert_array_equal(actual_u.to_numpy(),uwb.to_numpy())
            assert orig_i.t.equals(actual_i.t)
            for key in ['source_message','source_range','tag_id','anchor_id','valid','t']:
                assert orig_u[key].equals(actual_u[key]),key
            print('INJECTED',seq,case,[(e['sensor'],e['strength'],len(e['source_ids'])) for e in episodes])


def cal_accel():
    from scipy.signal import savgol_filter
    from scipy.interpolate import CubicSpline
    reg=yaml.safe_load(CONTRACT.read_text()); cal=reg['cal_diagnostic']
    gt=pd.read_csv(CACHE/'data/ISAS-Walk1/gt.csv').drop_duplicates('t');imu=pd.read_csv(CACHE/'data/ISAS-Walk1/imu.csv')
    t=np.arange(gt.t.min()+.3,gt.t.max()-.3,.02)
    slerp=Slerp(gt.t,Rotation.from_quat(gt[['qx','qy','qz','qw']]))
    rot=slerp(t);p=CubicSpline(gt.t,gt[['px','py','pz']])(t)
    a=savgol_filter(p,31,3,deriv=2,delta=.02,axis=0)
    w=(slerp(t-.01).inv()*slerp(t+.01)).as_rotvec()/.02
    alpha=savgol_filter(w,31,3,deriv=1,delta=.02,axis=0)
    def skew(v):return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    R=np.array(cal['R_tracker_from_imu']);q=t-cal['time_gt_minus_imu_s']
    f=CubicSpline(imu.t,imu[['ax','ay','az']])(q)@R.T
    m=np.array([skew(aa)+skew(ww)@skew(ww) for aa,ww in zip(alpha,w)])
    A=np.concatenate([m,-rot.as_matrix().transpose(0,2,1),np.tile(np.eye(3),(len(t),1,1))],axis=2)
    y=f-rot.inv().apply(a)
    train=(t<cal['training_end_s']-.5)&(t>imu.t.min()+.5)
    hold=(t>cal['training_end_s']+.5)&(t<imu.t.max()-.5)
    mat=A[train].reshape(-1,9);rhs=y[train].reshape(-1)
    x,res,rank,sv=np.linalg.lstsq(mat,rhs,rcond=None)
    e=mat@x-rhs;cov=np.linalg.inv(mat.T@mat)*(e@e/(len(e)-9))
    out=dict(scope='CAL-only diagnostic, 0.62s Savitzky-Golay support; no parameters applied',rank=int(rank),singular_values=sv.tolist(),condition=float(sv[0]/sv[-1]),
     tracker_to_imu_translation_m=x[:3].tolist(),gravity_tracker_world_mps2=x[3:6].tolist(),bias_tracker_mps2=x[6:].tolist(),
     training_rmse_mps2=float(np.sqrt(np.mean(e**2))),heldout_rmse_mps2=float(np.sqrt(np.mean((A[hold].reshape(-1,9)@x-y[hold].reshape(-1))**2))),
     conditional_ols_standard_error=np.sqrt(np.diag(cov)).tolist(),qualification='REFERENCE_OPEN: derivative-correlated tracker errors and unknown mounting/time truth invalidate OLS uncertainty as hardware bound')
    save(CACHE/'cal_accel_diagnostic.json',out)
    print(json.dumps(out,indent=2))

def profile():
    source=CACHE/'data/ISAS-Walk2';dest=CACHE/'profile-input'
    dest.mkdir(parents=True,exist_ok=True)
    uwb=pd.read_csv(source/'uwb.csv',float_precision='round_trip')
    end=float(uwb[uwb.source_message<120].t.max())
    for name in ('imu.csv','uwb.csv','gt.csv'):
        df=pd.read_csv(source/name,float_precision='round_trip')
        df[df.t<=end+.02].to_csv(dest/name,index=False,float_format='%.17g')
    for name in ('anchors.csv','manifest.json'):shutil.copy2(source/name,dest/name)
    run_native('ISAS-Walk2','FDE',dest,'profile-original',paced=False,audit=True)


def sfuise_summary():
    import sc8_analysis as analysis
    results={}
    for seq in ('ISAS-Walk2','ISAS-Walk3'):
        run=CACHE/'runs'/(seq+'-SFUISE');source=CACHE/'data'/seq
        expected_i=common.read_csv(source/'imu.csv');u=common.read_csv(source/'uwb.csv')
        groups=[]
        for row in u:
            if not groups or groups[-1][0]['source_message']!=row['source_message']:groups.append([])
            groups[-1].append(row)
        actual_i=[];actual_u=[];topics=set()
        with AnyReader([run/'input.bag']) as reader:
            for connection,stamp,raw in reader.messages():
                topics.add(connection.topic);m=reader.deserialize(raw,connection.msgtype)
                if connection.topic.endswith('/imu_ds'):
                    actual_i.append((m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                        [m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]))
                if connection.topic.endswith('/toa_ds64'):actual_u.append(list(m.data))
        assert not any('gt' in x or 'truth' in x for x in topics)
        assert len(actual_i)==len(expected_i) and len(actual_u)==len(groups)
        def ns(t):return 10**12+int(np.floor(float(t)*1e9+.5))
        for row,(stamp,v) in zip(expected_i,actual_i):
            assert stamp==ns(row['t']) and v==[float(row[k]) for k in common.IMU_FIELDS[1:]]
        for group,data in zip(groups,actual_u):
            assert data[0]==ns(group[-1]['t']) and data[1]==len(group)
            for j,row in enumerate(group):assert data[2+3*j:5+3*j]==[float(row['anchor_id']),float(row['range']),float(row['valid'])]
        online=analysis.trajectory(run/'trajectory.tum.online.tum')
        hist,validity=analysis.historical_support(run,analysis.trajectory(run/'trajectory.tum.history_final_calibration.tum'))
        obs=pd.read_csv(run/'observer.csv');final=json.loads((run/'observer.csv.final.json').read_text())
        results[seq]=dict(status=json.loads((run/'run_status.json').read_text())['status'],
            parity='PASS: all ADC/range/ID/timestamp doubles exact; no GT topics',
            sensor_hash=sensor_hash(source),online_n=len(online[0]),final_n=len(hist[0]),
            first_online_s=float(online[0][0]),last_online_s=float(online[0][-1]),
            final_history_support=analysis.clean(validity),final_receipt=final,
            numeric_cutoff_last_s=float(obs.max_sensor_stamp_used.iloc[-1]*1e-9-1000),
            history_cutoff='whole final export input upper bound; not per-point exact dependency',
            physical_APE=None,physical_ARE=None,physical_RPE=None,reference='REFERENCE_OPEN')
    save(CACHE/'sfuise_summary.json',results)
    print(json.dumps(results,indent=2))


def local():
    source=CACHE/'profile-input'
    run_native('ISAS-Walk2','FDE',source,'profile-pruned',paced=False,audit=True,pruning=True)
    left=CACHE/'runs/profile-original';right=CACHE/'runs/profile-pruned'
    a=pd.read_csv(left/'attempts.csv');b=pd.read_csv(right/'attempts.csv')
    fields=['batch','t','committed','backend_updates','transaction_id','action_id','action','fde_status',
            'detector_valid','detector_passed','detector_statistic','detector_threshold','detector_dof',
            'base_graph_version','history_removed_groups','history_added_groups','bridge_active','bridge_epochs','reinit_required','internal_protected_output']
    pd.testing.assert_frame_equal(a[fields],b[fields],check_exact=True)
    pd.testing.assert_frame_equal(pd.read_csv(left/'states.csv'),pd.read_csv(right/'states.csv'),check_exact=True)
    for name in ('factor_ledger.csv','transactions.csv','health.csv','bridge.csv'):
        x=pd.read_csv(left/'native_audit'/name);y=pd.read_csv(right/'native_audit'/name)
        # Counters/time metadata may differ, ledger source IDs and all native receipts cannot.
        pd.testing.assert_frame_equal(x,y,check_exact=True)
    x=pd.read_csv(left/'native_audit/candidates.csv');y=pd.read_csv(right/'native_audit/candidates.csv')
    identity=['timestamp_ns','window_id','action_id','action_type','physical_source_ids','removed_group_ids','added_group_ids','bridge_mode','cardinality','selected']
    pd.testing.assert_frame_equal(x[identity],y[identity],check_exact=True)
    d=pd.read_csv(right/'native_audit/diagnostic_candidates.csv')
    skip=d.skip_reason.eq('SKIPPED_INELIGIBLE')
    assert skip.any() and not d.loc[skip,'kernel_evaluated'].any()
    numeric=['valid','post_detector_passed','rank','dof','hpl_m','vpl_m','risk_allocation','information_logdet']
    pd.testing.assert_frame_equal(x.loc[~skip,numeric],y.loc[~skip,numeric],check_exact=True)
    save(CACHE/'pruning_equivalence.json',dict(status='PASS',attempts=len(a),native_commits=int(a.committed.sum()),
        actions=len(x),not_solved=int(skip.sum()),exact=fields+identity+['states15D','ledger','transactions','health','bridge'],
        core_before_ms=float(a.core_ms.sum()),core_after_ms=float(b.core_ms.sum())))
    run_native('ISAS-Walk2','FDE',source,'profile-best-effort',paced=False,audit=True,pruning=True,best_effort=True)
    c=pd.read_csv(CACHE/'runs/profile-best-effort/attempts.csv')
    fallback=c.estimation_outcome.eq('UNPROTECTED_BEST_EFFORT')
    assert (c.backend_updates==1).all() and c.transaction_id.is_unique and c.t.is_unique
    assert not c.internal_protected_output.any()
    assert (c.loc[c.fde_transaction_id!=c.transaction_id,'fde_backend_updates']==0).all()
    save(CACHE/'best_effort_local.json',dict(status='PASS',attempts=len(c),commits=int(c.committed.sum()),
        outcomes=c.estimation_outcome.value_counts().astype(int).to_dict(),fde_status=c.fde_status.value_counts().astype(int).to_dict()))


def continuous():
    for seq in ('ISAS-Walk2','ISAS-Walk3'):
        run_native(seq,'FDE',label=seq+'-FDE-RESEARCH',audit=True,pruning=True,best_effort=True)


def paired():
    for seq,cases in [('ISAS-Walk2',['U','A','G','J']),('ISAS-Walk3',['J'])]:
        for case in cases:
            source=CACHE/'injected'/(seq+'-'+case)
            for method in ('NOFDE','FDE'):
                run_native(seq,method,source,seq+'-'+case+'-'+method,audit=method=='FDE',pruning=method=='FDE',best_effort=method=='FDE')


def negative():
    """One bounded source-gap case exercises the actual runner fallback guard."""
    base=CACHE/'data/ISAS-Walk2';source=CACHE/'negative-source-gap'
    source.mkdir(exist_ok=True)
    imu=pd.read_csv(base/'imu.csv',float_precision='round_trip')
    uwb=pd.read_csv(base/'uwb.csv',float_precision='round_trip')
    imu=imu[(imu.t<=6)&~((imu.t>=5.2)&(imu.t<=5.3))]
    uwb=uwb[uwb.t<=6]
    imu.to_csv(source/'imu.csv',index=False,float_format='%.17g')
    uwb.to_csv(source/'uwb.csv',index=False,float_format='%.17g')
    for name in ('anchors.csv','gt.csv','manifest.json'):shutil.copy2(base/name,source/name)
    # Legacy dataset_config adapts max_gap to input. Freeze the original law so
    # this negative case tests a gap, rather than silently widening the gate.
    result=run_native('ISAS-Walk2','FDE',source,'negative-source-gap-fixed',paced=False,audit=True,pruning=True,best_effort=True,
        fixed_config=CACHE/'runs/ISAS-Walk2-FDE-RESEARCH/effective_nominal_config.yaml')
    run=CACHE/'runs/negative-source-gap-fixed'
    assert result['status']=='FAIL'
    state=pd.read_csv(run/'states.csv');attempts=pd.read_csv(run/'attempts.csv')
    assert len(state)>0 and state.t.max()<5.3
    status=json.loads((run/'run_status.json').read_text())
    assert 'gap' in json.dumps(status).lower(),status
    assert (attempts.internal_protected_output==0).all()
    save(CACHE/'negative_summary.json',dict(status='PASS_EXPECTED_SOURCE_GAP_REFUSAL',
        process=result,run_status=status,attempts=len(attempts),committed=len(state),
        last_state_s=float(state.t.max()),gap_s=[5.2,5.3],
        raw_batches=int(uwb.source_message.nunique()),valid_batches=int(uwb[uwb.valid!=0].source_message.nunique())))


def extended_summary(run, sequence):
    from collections import Counter
    result=summarize_native(run,sequence)
    a=pd.read_csv(run/'attempts.csv')
    if 'estimation_outcome' in a:
        result['outcomes']={str(k):int(v) for k,v in a.estimation_outcome.value_counts().items()}
        result['actual_fde_recovery']=int(a.estimation_outcome.eq('ACTUAL_FDE_RECOVERY').sum())
        result['fde_rejected_nominal_continued']=int(((a.fde_backend_updates==0)&(a.committed==1)).sum())
        fallback=a[a.estimation_outcome.eq('UNPROTECTED_BEST_EFFORT')]
        assert (fallback.committed==1).all() and (fallback.backend_updates==1).all()
        assert (fallback.internal_protected_output==0).all()
        retried=fallback[fallback.fde_backend_updates==0]
        assert (retried.transaction_id!=retried.fde_transaction_id).all()
        result['receipt_checks']='PASS: unique timestamps/transactions, single backend update, unprotected fallback'
    else:result['actual_fde_recovery']=0
    result['alarm_count']=int(((a.detector_valid==1)&(a.detector_passed==0)).sum())
    result['input_identity']=json.loads((run/'execution.json').read_text())
    result['input_identity']['effective_config_sha256']=common.sha256(run/'effective_nominal_config.yaml')
    result['first_state']=pd.read_csv(run/'states.csv').iloc[0].to_dict()
    result['last_state']=pd.read_csv(run/'states.csv').iloc[-1].to_dict()
    result['trace_sha256']={name:common.sha256(run/name) for name in ('attempts.csv','states.csv','bootstrap.json','execution.json')}
    result['bridge_attempts']=int(a.bridge_active.sum())
    result['history_replacement_attempts']=int(a.history_removed_groups.notna().sum())
    result['protected_attempts']=int(a.internal_protected_output.sum())
    result['timely_50ms']=int((a.measurement_to_sink_ms<=50).sum())
    result['fastest_epoch']=a.loc[a.core_ms.idxmin(),['batch','t','core_ms','fde_status']].to_dict()
    result['slowest_epoch']=a.loc[a.core_ms.idxmax(),['batch','t','core_ms','fde_status']].to_dict()
    audit=run/'native_audit'
    if audit.exists():
        d=pd.read_csv(audit/'diagnostic_attempts.csv')
        final_stamp_ns=round(float(a.t.max())*1e9)
        result['audit_unfinalized_attempts']=int((d.input_timestamp_ns>final_stamp_ns).sum())
        d=d[d.input_timestamp_ns<=final_stamp_ns]
        result['census']={k:int(d[k].sum()) for k in ['generated_actions','kernel_evaluated_actions','pl_evaluated_actions','selected_actions',
            'base_svd','base_llt','candidate_reference_svd','candidate_inner_llt','imu_oracle_reintegrations']}
        result['hypotheses_max']=int(d.hypothesis_count.max())
        result['resource_growth']={k:dict(first=float(d[k].iloc[0]),last=float(d[k].iloc[-1]),maximum=float(d[k].max()))
            for k in ['cache_entries','cache_bytes','marginalization_count','raw_imu_samples']}
        timing=pd.read_csv(audit/'timing.csv')
        if len(timing):
            result['resource_growth']['logged_factor_count']=dict(first=int(timing.factor_count.iloc[0]),last=int(timing.factor_count.iloc[-1]),maximum=int(timing.factor_count.max()))
        result['resource_growth']['backend_slot_count']='NOT_EXPORTED; do not substitute cumulative factor IDs'
        result['risk_reasons']=dict(Counter(d.risk_ledger_terms.fillna('NA')))
        result['primary_failures']=dict(Counter(d.primary_failure.fillna('NONE')))
        history=pd.read_csv(audit/'diagnostic_history_summary.csv')
        history=history[history.input_timestamp_ns<=final_stamp_ns]
        result['history_summary']=dict(records=len(history),
            counts={k:int(history[k].sum()) for k in ['present','valid','capacity_ok','claims_full_coverage']},
            maxima={k:int(history[k].max()) for k in ['omitted_epoch_count','material_gap_epoch_count','fault_columns','emitted_rows']})
        s=pd.read_csv(audit/'diagnostic_stages.csv')
        s=s[s.input_timestamp_ns<=final_stamp_ns]
        exclusive=s[~s.stage.str.startswith(('window_','candidate_batch_'))&s.stage.ne('core_total')]
        costs=exclusive.groupby('stage').wall_ms.sum()
        costs['wrapper_logging_fallback_and_unassigned']=float(a.core_ms.sum()-costs.sum())
        assert costs['wrapper_logging_fallback_and_unassigned']>=0
        result['exclusive_mean_ms']={str(k):float(v/len(a)) for k,v in costs.items()}
        causes=Counter();selected=0;skipped=0;candidate_ids=0;rank=Counter();dofs=Counter()
        for c in pd.read_csv(audit/'diagnostic_candidates.csv',chunksize=10000):
            c=c[c.input_timestamp_ns<=final_stamp_ns]
            causes.update(c.skip_reason.fillna('NONE'));skipped+=int(c.skip_reason.eq('SKIPPED_INELIGIBLE').sum())
        for c in pd.read_csv(audit/'candidates.csv',usecols=['timestamp_ns','selected','rank','dof','valid','post_detector_passed'],chunksize=10000):
            c=c[c.timestamp_ns<=final_stamp_ns]
            candidate_ids+=len(c);selected+=int(c.selected.sum());rank.update(c['rank']);dofs.update(c.dof)
        result['candidate_results']=dict(records=candidate_ids,selected=selected,not_solved=skipped,
            terminal_counts=dict(causes),rank_counts={str(k):int(v) for k,v in rank.items()},dof_counts={str(k):int(v) for k,v in dofs.items()})
    # Sparse plot samples retain first/last and every eighth actual receipt.
    result['timeline']=[dict(t=float(row.t),commit=int(row.committed),alarm=int(row.detector_valid==1 and row.detector_passed==0),
        core_ms=float(row.core_ms),queue_ms=float(max(0,row.measurement_to_sink_ms-row.core_ms)),status=row.fde_status)
        for row in a.iloc[sorted(set([0,len(a)-1]+list(range(0,len(a),8))))].itertuples()]
    return result


def evaluate():
    results={}
    for seq in ('ISAS-Walk2','ISAS-Walk3'):
        for method in ('NOM','NOFDE','FDE-RESEARCH'):
            run=CACHE/'runs'/(seq+'-'+method)
            if (run/'execution.json').exists():results[run.name]=extended_summary(run,seq)
        if seq+'-FDE-RESEARCH' in results:
            assert results[seq+'-FDE-RESEARCH']['input_identity']['sensor_hash']==results[seq+'-NOFDE']['input_identity']['sensor_hash']
            left=pd.read_csv(CACHE/'runs'/(seq+'-NOFDE')/'states.csv')
            right=pd.read_csv(CACHE/'runs'/(seq+'-FDE-RESEARCH')/'states.csv')
            common=left.merge(right,on='t',suffixes=('_off','_on'))
            results[seq+'-FDE-RESEARCH']['paired_state_diagnostic']=dict(n=len(common),
                maximum_component_difference=max(float(np.max(np.abs(common[k+'_off']-common[k+'_on']))) for k in left.columns if k!='t'),
                label='state difference only, not physical accuracy or FDE recovery')
    for seq,cases in [('ISAS-Walk2',['U','A','G','J']),('ISAS-Walk3',['J'])]:
        for case in cases:
            pair={}
            for method in ('NOFDE','FDE'):
                run=CACHE/'runs'/(seq+'-'+case+'-'+method)
                if not (run/'execution.json').exists():continue
                result=extended_summary(run,seq)
                injection=json.loads((CACHE/'injected'/(seq+'-'+case)/'injection.json').read_text())
                result['injection']=injection
                a=pd.read_csv(run/'attempts.csv');episodes=[]
                ledger_path=run/'native_audit/factor_ledger.csv'
                intervals=None
                if ledger_path.exists():
                    ledger=pd.read_csv(ledger_path,usecols=['factor_id','sensor','factor_kind','time_begin_ns','time_end_ns'])
                    intervals=ledger[ledger.sensor.eq('IMU')&ledger.factor_kind.eq('COMBINED_IMU')].drop_duplicates('factor_id')
                for e in injection['episodes']:
                    ep=dict(e);active=a[(a.t>=e['begin_s'])&(a.t<=e['end_s']+.1)]
                    after=a[a.t>e['end_s']+.1]
                    valid_count=sum(row[2]!=0 for row in e['source_ids']) if e['sensor']=='UWB' else len(e['source_ids'])
                    reached=bool(len(a) and a.max_sensor_stamp_used.max()>=e['end_s'])
                    ep.update(source_time_reached=reached,valid_injected_samples=valid_count,
                        observed=reached and valid_count>0,
                        activity_attempts=len(active),activity_commits=int(active.committed.sum()),
                        alarm_attempts=int(((active.detector_valid==1)&(active.detector_passed==0)).sum()),
                        actual_recovery_commits=int(active.estimation_outcome.eq('ACTUAL_FDE_RECOVERY').sum()) if 'estimation_outcome' in active else 0,
                        bridge_commits=int(((active.bridge_active==1)&(active.committed==1)).sum()),
                        post_commits=int(after.committed.sum()),physical_phase_APE=None,physical_phase_ARE=None,physical_phase_RPE=None)
                    # A detector alarm is not support-correct isolation. A nonzero
                    # action requires manual native source-ledger verification.
                    ep['correct_exclusion']='OPEN_NEEDS_NATIVE_SUPPORT_REVIEW' if ep['actual_recovery_commits'] else False
                    ep['rejoin']='OPEN_NO_LEGAL_ISOLATION_RECEIPT' if not ep['bridge_commits'] else 'REVIEW_NATIVE_BRIDGE'
                    if e['sensor']=='IMU' and intervals is not None:
                        supports=intervals[(intervals.time_end_ns>=e['begin_s']*1e9)&(intervals.time_begin_ns<=e['end_s']*1e9)]
                        ep['native_pim_intervals']=supports[['factor_id','time_begin_ns','time_end_ns']].astype('int64').values.tolist()
                        ep['adc_samples_in_pim_support']=sum(bool(((supports.time_begin_ns<=t*1e9)&(supports.time_end_ns>=t*1e9)).any()) for t in e['sample_times'])
                    episodes.append(ep)
                result['episodes']=episodes;results[run.name]=result;pair[method]=result
            if len(pair)==2:
                assert pair['FDE']['input_identity']['sensor_hash']==pair['NOFDE']['input_identity']['sensor_hash']
                left=pd.read_csv(CACHE/'runs'/(seq+'-'+case+'-NOFDE')/'states.csv')
                right=pd.read_csv(CACHE/'runs'/(seq+'-'+case+'-FDE')/'states.csv')
                common=left.merge(right,on='t',suffixes=('_off','_on'))
                maxdiff=max(float(np.max(np.abs(common[k+'_off']-common[k+'_on']))) for k in left.columns if k!='t') if len(common) else None
                results[seq+'-'+case+'-FDE']['paired_state_diagnostic']=dict(n=len(common),maximum_component_difference=maxdiff,
                    label='state difference only, not physical accuracy or FDE recovery')
    save(CACHE/'evaluation.json',results)
    print('evaluated',len(results),'actual runs; absent runs remain unexecuted')


def figures(results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':9,'svg.fonttype':'none'})
    out=REPORT.parent
    colors={'NOM':'#56B4E9','NOFDE':'#777777','FDE-RESEARCH':'#0072B2'}
    fig,axs=plt.subplots(1,2,figsize=(10,4.5),layout='constrained')
    for ax,seq in zip(axs,('ISAS-Walk2','ISAS-Walk3')):
        methods=['NOM','NOFDE','FDE-RESEARCH'];data=[results[seq+'-'+m] for m in methods]
        bars=ax.bar(methods,[100*x['coverage_all_valid'] for x in data],color=[colors[m] for m in methods])
        for bar,x in zip(bars,data):ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+1,f"{x['committed']}/{x['planned_valid']}",ha='center')
        ax.axhline(95,color='black',ls='--',lw=.8);ax.set_ylim(0,115);ax.set_title(seq)
        ax.set_ylabel('Native commits / all valid planned batches (%)')
        ax.text(.5,.47,'Physical APE / ARE / RPE: NA\nTracker–IMU/tag reference OPEN',transform=ax.transAxes,ha='center',bbox=dict(facecolor='white',edgecolor='.7'))
        sf=json.loads((CACHE/'sfuise_summary.json').read_text())[seq]
        ax.text(.5,.20,f"SFUISE: online {sf['online_n']}, history {sf['final_n']} states\nDifferent output rate / initialization; no count ranking",transform=ax.transAxes,ha='center')
    fig.suptitle('Walk full-recording output coverage — ordinary estimates are not FDE recovery')
    fig.savefig(out/'walk_coverage.svg');plt.close(fig)

    cases=[('ISAS-Walk2',c) for c in ['U','A','G','J']]+[('ISAS-Walk3','J')]
    fig,axs=plt.subplots(len(cases),1,figsize=(10,7.5),sharex=True,layout='constrained')
    for ax,(seq,case) in zip(axs,cases):
        key=seq+'-'+case+'-FDE';r=results[key];a=pd.read_csv(CACHE/'runs'/key/'attempts.csv')
        ax.scatter(a.t,np.full(len(a),.25),c=np.where(a.committed==1,'#56B4E9','#999999'),s=2,label='ordinary commit / refusal')
        alarm=a[(a.detector_valid==1)&(a.detector_passed==0)]
        ax.scatter(alarm.t,np.full(len(alarm),.65),s=3,color='#CC79A7',label='alarm')
        recovery=a[a.estimation_outcome=='ACTUAL_FDE_RECOVERY']
        if len(recovery):ax.scatter(recovery.t,np.full(len(recovery),1),s=15,marker='x',color='#009E73',label='actual action')
        for ep in r['episodes']:
            end=ep['end_s'] if ep['end_s']>ep['begin_s'] else ep['begin_s']+.06
            ax.axvspan(ep['begin_s'],end,alpha=.3,color='#D55E00' if ep['sensor']=='UWB' else '#E69F00')
        ax.set_ylim(0,1.2);ax.set_yticks([.25,.65,1],['commit','alarm','isolation']);ax.set_ylabel(seq[-1]+': '+case)
        ax.text(.99,.78,f"recovery commits={r['actual_fde_recovery']}; physical errors NA",ha='right',transform=ax.transAxes)
    axs[-1].set_xlabel('Source time (s); orange=IMU injection, red=UWB injection')
    fig.suptitle('Registered added faults and native outcomes — no inferred recovery from best-effort')
    fig.savefig(out/'walk_fault_timeline.svg');plt.close(fig)

    fig,axs=plt.subplots(1,3,figsize=(13,4.4),layout='constrained')
    for n,seq in enumerate(('ISAS-Walk2','ISAS-Walk3')):
        r=results[seq+'-FDE-RESEARCH'];cost=r['exclusive_mean_ms'];bottom=0
        groups={'window/prepare':sum(v for k,v in cost.items() if k in ['integrity_window','prepare']),
            'hypothesis/model':sum(v for k,v in cost.items() if k in ['hypothesis_evidence','model_generation','current_sensitivity','historical_sensitivity']),
            'action census':cost.get('health_actions',0),'candidate':cost.get('candidate_evaluation',0),
            'other':sum(cost.values())-sum(v for k,v in cost.items() if k in ['integrity_window','prepare','hypothesis_evidence','model_generation','current_sensitivity','historical_sensitivity','health_actions','candidate_evaluation'])}
        for (label,value),color in zip(groups.items(),['#56B4E9','#E69F00','#0072B2','#009E73','#999999']):
            axs[0].bar(seq[-1],value,bottom=bottom,color=color,label=label if n==0 else None);bottom+=value
        timeline=r['timeline'];axs[1].plot([x['t'] for x in timeline],[x['queue_ms']/1000 for x in timeline],label=seq)
        for j,(status,count) in enumerate(r['statuses'].items()):
            axs[2].barh(seq[-1]+' '+status,count)
    axs[0].set(title='Exclusive mean service costs',ylabel='ms / attempted batch',xlabel='Walk');axs[0].legend(fontsize=7)
    axs[1].set(title='Measured queue delay (sparse actual receipts)',xlabel='Source time (s)',ylabel='s');axs[1].legend()
    axs[2].set(title='Native FDE status (all attempts)',xlabel='Count')
    fig.suptitle('Engineering FDE runtime — raw PL / HMI probability: UNQUALIFIED')
    fig.savefig(out/'walk_runtime.svg');plt.close(fig)

def evidence():
    """Save compact measured evidence in the one report before owned-cache cleanup."""
    evaluate()
    results=json.loads((CACHE/'evaluation.json').read_text())
    assert len(results)==16, 'Every registered native run must have an execution result'
    figures(results)
    lines=['## 全录制与注入：冻结实测表', '',
        '分位数采用 nearest-rank；延迟包含完整等待队列。所有物理误差及其 ON−OFF 收益均为 NA。', '',
        '|运行|结束|原始/有效/eligible|attempt/commit/未尝试后缀|覆盖有效计划|报警/真实恢复/bridge|未保护best-effort|',
        '|---|---|---|---|---|---|---|']
    for name,r in results.items():
        lines.append(f"|{name}|{r['status']}|{r['raw_planned']}/{r['planned_valid']}/{r['eligible_planned']}|{r['attempted']}/{r['committed']}/{r['unattempted_eligible']}|{100*r['coverage_all_valid']:.3f}%|{r['alarm_count']}/{r['actual_fde_recovery']}/{r['bridge_attempts']}|{r.get('outcomes',{}).get('UNPROTECTED_BEST_EFFORT',0)}|")
    lines+=['','|运行|core P50/P95/P99/max ms|sink P50/P95/P99/max ms|queue P50/P95/P99/max ms|CPU s/服务RTF/RSS KiB|≤50ms次数|',
        '|---|---|---|---|---|---|']
    def four(v):return '/'.join(f'{v[k]:.3f}' for k in ('p50','p95','p99','max'))
    for name,r in results.items():
        lines.append(f"|{name}|{four(r['core_ms'])}|{four(r['sink_ms'])}|{four(r['queue_ms'])}|{r['cpu_s']:.3f}/{r['service_RTF']:.3f}/{r['peak_rss_kib']}|{r['timely_50ms']}/{r['attempted']}|")
    lines+=['','|运行|FDE状态全分母|动作记录/未求解/selected|prior替换/受保护输出|', '|---|---|---|---|']
    for name,r in results.items():
        c=r.get('candidate_results',{})
        lines.append(f"|{name}|{json.dumps(r['statuses'],ensure_ascii=False)}|{c.get('records','NA')}/{c.get('not_solved','NA')}/{c.get('selected','NA')}|{r['history_replacement_attempts']}/{r['protected_attempts']}|")
    lines+=['','注入活动统计包含结束后0.1秒，以覆盖异步下一批读取；这是响应观察窗，不是故障时刻修改。原始自然异常无真值标签，报警不能解释为新增故障的因果检出率。', '',
        '|运行/episode|真实注入秒|原始/有效注入样本数|observed|活动attempt/commit/alarm|正确排除/真实恢复/bridge|之后commit|',
        '|---|---|---|---|---|---|---|']
    for name,r in results.items():
        for e in r.get('episodes',[]):
            lines.append(f"|{name}/{e['sensor']}-{e['strength']}|{e['begin_s']:.6f}–{e['end_s']:.6f}|{len(e['source_ids'])}/{e['valid_injected_samples']}|{e['observed']}|{e['activity_attempts']}/{e['activity_commits']}/{e['alarm_attempts']}|{e['correct_exclusion']}/{e['actual_recovery_commits']}/{e['bridge_commits']}|{e['post_commits']}|")
    lines+=['','|IMU episode|修改ADC源行ID|落在实际PIM支撑内的样本|跨越的原生PIM因子ID|','|---|---|---|---|']
    for name,r in results.items():
        for e in r.get('episodes',[]):
            if 'native_pim_intervals' in e:
                lines.append(f"|{name}/{e['strength']}|{e['source_ids'][0]}–{e['source_ids'][-1]}|{e['adc_samples_in_pim_support']}/{len(e['source_ids'])}|{','.join(str(x[0]) for x in e['native_pim_intervals'])}|")
    lines+=['','## 冻结身份与可复核摘要','',
        '下列摘要保留输入与trace内容哈希、首末原生状态、事务检查、census计数、最慢实际epoch和资源增长；完整大trace在本轮验收后清理，按复现命令重建。', '','```json']
    compact={name:{k:v for k,v in r.items() if k in ('input_identity','trace_sha256','first_state','last_state','receipt_checks',
        'census','candidate_results','history_summary','primary_failures','audit_unfinalized_attempts','hypotheses_max','resource_growth','exclusive_mean_ms','fastest_epoch','slowest_epoch','paired_state_diagnostic')} for name,r in results.items()}
    # Commands contain reproducible local paths; input/binary hashes bind actual executions.
    freeze={p:common.sha256(ROOT/p) for p in ['apps/nominal_dataset_runner.cpp',
        'include/uwb_imu_pl/integrity/integrity_monitor.hpp','src/uwb_imu_pl/integrity/integrity_monitor.cpp',
        'tools/sc8_walk_followup.py','config/sc8_walk_followup.yaml','test/test_fde_post_selection.cpp']}
    lines.append(json.dumps(dict(source_freeze=freeze,runs=compact),ensure_ascii=False,indent=2));lines+=['```','',
        '![全录制覆盖率与参考资格](walk_coverage.svg)', '',
        '![真实注入、报警与动作时间线](walk_fault_timeline.svg)', '',
        '![互斥阶段耗时与实际队列](walk_runtime.svg)','']
    marker='## 全录制与注入：冻结实测表'
    prior=REPORT.read_text().split(marker)[0].rstrip()
    REPORT.write_text(prior+'\n\n'+'\n'.join(lines))


def main():
    commands={'prepare':prepare,'noninterference':noninterference,'baseline':baseline,'sfuise':sfuise_baseline,
        'summarize':summarize_baseline,'inject':inject,'cal-accel':cal_accel,'profile':profile,
        'sfuise-summary':sfuise_summary,'local':local,'continuous':continuous,'paired':paired,'negative':negative,'evaluate':evaluate,'evidence':evidence}
    p=argparse.ArgumentParser();p.add_argument('command',choices=list(commands));args=p.parse_args()
    commands[args.command]()

if __name__=='__main__':main()
