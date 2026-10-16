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


def run_native(sequence, method, source=None, label=None, paced=True, audit=False, best_effort=False):
    import sc8_experiments as old
    source=source or CACHE/'data'/sequence
    run=CACHE/'runs'/(label or (sequence+'-'+method))
    if (run/'execution.json').exists():
        return json.loads((run/'execution.json').read_text())
    run.mkdir(parents=True,exist_ok=True)
    entry=dict(requested_method='O-FULL' if method=='FDE' else 'O-NOFDE',window=[30,30],stream=sequence)
    config=old.dataset_config(entry,source,run)
    if audit:
        cfg=yaml.safe_load(config.read_text());cfg['output'].update(write_candidates=True,write_hypothesis_evidence=True)
        config.write_text(yaml.safe_dump(cfg,sort_keys=False))
    args=[str(BIN/'nominal_dataset_runner'),'--config',str(config),'--input',str(source),'--output',str(run),
          '--tag-policy','primary','--causal-bootstrap']
    if method!='NOM':args+=['--pipeline-mode','on' if method=='FDE' else 'off']
    if paced:args+=['--paced']
    if audit:args+=['--engineering-audit']
    if best_effort:args+=['--research-best-effort']
    begin=time.monotonic();before=resource.getrusage(resource.RUSAGE_CHILDREN)
    with (run/'process.log').open('w') as log:
        proc=subprocess.Popen(args,env=dict(environment(),UWB_IMU_PL_CANDIDATE_WORKERS='1'),stdout=log,stderr=subprocess.STDOUT)
        try:code=proc.wait(timeout=1200);status='FINISHED'
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            code=proc.returncode;status='RESOURCE_LIMIT'
    after=resource.getrusage(resource.RUSAGE_CHILDREN)
    result=dict(command=args,returncode=code,status=status,wall_s=time.monotonic()-begin,
        cpu_s=after.ru_utime+after.ru_stime-before.ru_utime-before.ru_stime,
        sensor_hash=sensor_hash(source),binary_sha256=common.sha256(BIN/'nominal_dataset_runner'),
        dso_sha256=common.sha256(BIN.parent/'libuwb_imu_pl.so'))
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
    return dict(raw_planned=int(u.source_message.nunique()),planned_valid=len(planned),eligible_planned=len(ready),
        startup=int((planned<=boot['timestamp_s']).sum()),after_imu_end=int((planned>imu.t.max()).sum()),
        wholly_invalid=int(u.source_message.nunique()-len(planned)),attempted=len(a),committed=len(c),
        coverage_all_valid=len(c)/len(planned),coverage_eligible=len(c)/len(ready),
        rejected=len(a)-len(c),unattempted_eligible=len(ready)-len(a),
        interval_s=[float(c.t.min()),float(c.t.max())],bootstrap=boot,
        statuses={str(k):int(v) for k,v in a.fde_status.value_counts().items()},
        core_ms=stats(a.core_ms),sink_ms=stats(a.measurement_to_sink_ms),
        queue_ms=stats(np.maximum(0,a.measurement_to_sink_ms-a.core_ms)),
        cpu_s=status.get('cpu_ms',0)/1000,peak_rss_kib=status.get('peak_rss_kib'),
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


def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['prepare','noninterference','baseline','sfuise','summarize','inject','cal-accel','profile','sfuise-summary']);args=p.parse_args()
    {'prepare':prepare,'noninterference':noninterference,'baseline':baseline,'sfuise':sfuise_baseline,'summarize':summarize_baseline,'inject':inject,'cal-accel':cal_accel,'profile':profile,'sfuise-summary':sfuise_summary}[args.command]()

if __name__=='__main__':main()
