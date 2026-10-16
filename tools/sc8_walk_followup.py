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


def run_native(sequence, method, source=None, label=None, paced=True):
    import sc8_experiments as old
    source=source or CACHE/'data'/sequence
    run=CACHE/'runs'/(label or (sequence+'-'+method))
    if (run/'execution.json').exists():
        return json.loads((run/'execution.json').read_text())
    run.mkdir(parents=True,exist_ok=True)
    entry=dict(requested_method='O-FULL' if method=='FDE' else 'O-NOFDE',window=[30,30],stream=sequence)
    config=old.dataset_config(entry,source,run)
    args=[str(BIN/'nominal_dataset_runner'),'--config',str(config),'--input',str(source),'--output',str(run),
          '--tag-policy','primary','--causal-bootstrap']
    if method!='NOM':args+=['--pipeline-mode','on' if method=='FDE' else 'off']
    if paced:args+=['--paced']
    begin=time.monotonic();before=resource.getrusage(resource.RUSAGE_CHILDREN)
    with (run/'process.log').open('w') as log:
        proc=subprocess.Popen(args,env=environment(),stdout=log,stderr=subprocess.STDOUT)
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
                df.loc[df.t>5.,'x']=1e6
            df.to_csv(dest/name,index=False,float_format='%.17g')
        for name in ('anchors.csv','manifest.json'):shutil.copy2(source/name,dest/name)
        run_native('ISAS-Walk1','NOM',dest,'noninterference-'+variant,paced=False)
        pair.append(CACHE/'runs'/('noninterference-'+variant))
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


def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['prepare','noninterference','baseline']);args=p.parse_args()
    {'prepare':prepare,'noninterference':noninterference,'baseline':baseline}[args.command]()

if __name__=='__main__':main()
