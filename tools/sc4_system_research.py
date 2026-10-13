#!/usr/bin/env python3
"""SC4 fixed replay, dependency-equivalence and runtime/application evaluation."""
import argparse,csv,hashlib,json,os,re,subprocess,time
from collections import Counter
from pathlib import Path
import numpy as np
import yaml
import application_al_policy as application
ROOT=Path(__file__).resolve().parents[1]
BIN=ROOT.parents[1]/'devel/.private/uwb_imu_pl/lib/uwb_imu_pl'
BUILD=ROOT.parents[1]/'build/uwb_imu_pl'
CACHE=ROOT/'.cache/sc4-system'
STATE=['tag_x','tag_y','tag_z','vx','vy','vz','bax','bay','baz','bgx','bgy','bgz','qw','qx','qy','qz']
SEMANTIC=STATE+['epoch','functional','bound_status','finite','qualified','source','scope','timestamp_ns','transaction','version','updates','hpl','vpl','cumulative_risk','physical_expected','physical_evaluated','range_masks','excluded','imu_bridge','imu_rejoined','carry','actions_expected','actions_evaluated','history_epochs','active_slots','retained_ids','input_ids','removed_groups','formal_eligible','publication_protected','application_available','source_time_origin_s','task_horizon_ns','current_risk_status']
def environment():
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    env['LD_LIBRARY_PATH']=str(BIN.parent)+':'+env.get('LD_LIBRARY_PATH','')
    for name in list(env):
        if name.startswith(('SC1_','SC2_','SC3_')) or name=='HYBRID_SHARED_EPISODE':env.pop(name)
    return env
def native(config,scene,count,profile='optimized',paced=False,fde='on',label=None):
    CACHE.mkdir(parents=True,exist_ok=True);label=label or (scene+'-'+profile+'-'+fde)
    path=CACHE/(label+'.csv');log=CACHE/(label+'.log')
    with log.open('w') as sink:
        result=subprocess.run([str(BIN/'epoch_raim_runner'),config,str(path),scene,'system',fde,str(count),profile,'paced' if paced else 'offline'],cwd=ROOT,env=environment(),stdout=sink,stderr=subprocess.STDOUT)
    rows=[]
    if path.exists():
        with path.open() as source:rows=list(csv.DictReader(source))
    if result.returncode:
        committed=sum(int(r.get('updates',0)) for r in rows)
        print('FAILED',label,'scheduled',count,'rows',len(rows),'committed',committed,'remaining',count-len(rows),log.read_text()[-1500:],flush=True)
        result.check_returncode()
    assert len(rows)==count and all(r['formal_eligible']==r['publication_protected']==r['application_available']=='0' for r in rows)
    return rows,log.read_text()
def equivalent(a,b):
    assert len(a)==len(b)
    for x,y in zip(a,b):
        for key in SEMANTIC:
            assert x[key]==y[key],(x['epoch'],key,x[key],y[key])
def statistics(values):
    v=np.asarray(values,dtype=float);v=v[np.isfinite(v)]
    return 'UNKNOWN' if not len(v) else '/'.join(f'{x:.8g}' for x in [v.mean(),np.median(v),np.percentile(v,95),v.max()])
def summarize(label,rows,log=''):
    assert all(int(r['numeric_evaluated'])+int(r['numeric_skipped_post_rejected'])+int(r['numeric_skipped_dominated'])==int(r['actions_evaluated']) for r in rows if r['source']=='raw-source-rw-v1' and r['updates']=='1' and r['qualified']=='1')
    committed=sum(int(r['updates']) for r in rows);qualified=sum(int(r['qualified']) for r in rows)
    print('DENOM',label,'scheduled='+str(len(rows)),'attempted='+str(committed),'committed='+str(committed),'aborted=0','finite='+str(sum(int(r['finite']) for r in rows)),'qualified='+str(qualified),'refused='+str(len(rows)-qualified),'late='+str(sum(r['on_time']=='0' for r in rows)),'core_miss='+str(sum(r['core_on_time']=='0' for r in rows)),flush=True)
    print('BOUND',label,dict(Counter(r['bound_status'] for r in rows)),flush=True)
    for key in ['core_ms','full_ms','arrival_to_output_ms','queue_ms','measurement_age_ms','prepare_ms','candidate_ms','commit_ms','bound_ms','cleanup_ms','optimizer_ms','diagnostics_ms','reintegrations_ms','source_init_ms','max_rss_kb','history_epochs','active_slots']:
        print(label,key,'mean/median/P95/max',statistics([r[key] for r in rows]),flush=True)
    for row in rows:
        if int(row['epoch']) in [15,16,17,18,26,27,31,50,51,58,60]:
            print('SNAPSHOT',label,{k:row[k] for k in ['epoch','functional','qualified','excluded','imu_bridge','imu_rejoined','carry','actions_evaluated','numeric_evaluated','numeric_skipped_post_rejected','numeric_skipped_dominated','total_rebuilds','total_reintegrations','core_ms','full_ms','arrival_to_output_ms']},flush=True)
    if log:print(label,log.strip(),flush=True)
    policy=yaml.safe_load((ROOT/'config/application_al_policy.yaml').read_text());before=json.dumps(rows,sort_keys=True)
    assert application.validate(policy)['status']=='APPLICATION_AL_UNQUALIFIED'
    for hal,val in [(2,3),(5,5),(10,10)]:
        n=sum(r['qualified']=='1' and float(r['hpl'])<=hal and float(r['vpl'])<=val for r in rows)
        print('AL',label,hal,val,'spatial_numeric_pass',n,'application_available=0',flush=True)
    assert before==json.dumps(rows,sort_keys=True)
def performance():
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip(),'freeze requires locally saved clean source'
    print('FREEZE_SHA',subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),flush=True)
    for scene in ['SC3_U0','SC3_G']:
        pair={}
        for profile in ['reference','optimized']:
            rows,log=native('config/sc4_performance.yaml',scene,50,profile,paced=True,label='perf-'+scene+'-'+profile);pair[profile]=rows
            assert all(r['updates']=='1' and r['qualified']=='1' for r in rows)
            summarize('perf-'+scene+'-'+profile,rows[15:],log)
            print('WARMUP',scene,profile,'scheduled/committed=15/15',flush=True)
        equivalent(pair['reference'],pair['optimized'])
        print('EQUIVALENT',scene,'50 exact full-state/source/action/receipt/risk/PL results',flush=True)
    print('PERFORMANCE MATRIX scheduled/attempted/committed=200; warmup=60 measured=140; 4 processes',flush=True)
def functional():
    for scene in ['SC3_U1','SC3_U2','SC3_IA','SC3_IG','SC3_A','SC3_STAGGERED']:
        rows,log=native('config/sc4_system.yaml',scene,30,label='functional-'+scene)
        assert all(r['updates']=='1' and r['qualified']=='1' for r in rows)
        summarize('functional-'+scene,rows,log)
        # Post-scoring only; these fixed motion parameters never enter selection.
        from scipy.spatial.transform import Rotation
        t=np.array([int(r['timestamp_ns'])*1e-9 for r in rows]);amp=np.array([.1,.08,.05]);freq=np.array([.5,.4,.3]);axis=np.array([1,2,3])/np.sqrt(14)
        tag=amp*np.sin(t[:,None]*freq)+np.array([0,0,1])+Rotation.from_rotvec(.2*t[:,None]*axis).apply(np.tile([.2,-.1,.08],(len(t),1)))
        estimates=np.array([[float(r[k]) for k in STATE[:3]] for r in rows]);pe=estimates-tag
        assert all(np.linalg.norm(d[:2])<=float(r['hpl']) and abs(d[2])<=float(r['vpl']) for d,r in zip(pe,rows))
        print('POST_SCORE',scene,'PE_H',statistics(np.linalg.norm(pe[:,:2],axis=1)),'PE_V',statistics(abs(pe[:,2])),flush=True)
def prepare_record():
    # Read-only short segment of the existing representative own_vicon bag.
    # Reuse original benchmark units/topics/field adapters, no new calibration.
    import benchmark_common as common
    import run_dataset_benchmark as dataset
    config=dataset.load_config(ROOT/'config/benchmark/own_vicon.yaml');sequence=config['sequences'][0]
    path=Path(config['_data_root'])/sequence['path'];cache=CACHE/'record-input';cache.mkdir(parents=True,exist_ok=True)
    topics={config['imu']['topic'],config['uwb']['topic'],config['ground_truth']['topic']}
    anchor_topics={config['anchors']['source_topics'].format(id=x):x for x in config['anchors']['ids']};topics.update(anchor_topics)
    imu=[];uwb=[];gt=[];anchors={x:[] for x in config['anchors']['ids']};start=None
    for topic,bag_ns,msg in dataset.bag_messages(path,topics):
        if start is None:start=bag_ns
        if bag_ns-start>8_000_000_000:break
        timestamp=common.header_time(msg,bag_ns)
        if topic==config['imu']['topic']:imu.append((timestamp,msg))
        elif topic==config['uwb']['topic']:uwb.append((timestamp,int(msg.id),msg.nodes))
        elif topic==config['ground_truth']['topic']:gt.append((timestamp,msg.pose))
        elif topic in anchor_topics:anchors[anchor_topics[topic]].append(dataset.v3(msg.pose.position))
    assert imu and uwb and gt and all(anchors.values())
    anchor={key:np.median(value,axis=0) for key,value in anchors.items()};origin=min(imu[0][0],uwb[0][0],gt[0][0])
    common.write_csv(cache/'imu.csv',common.IMU_FIELDS,[dataset.imu_row(t-origin,m,accel_scale=config['imu']['acceleration_scale']) for t,m in imu])
    rows=[]
    for message,(t,tag,nodes) in enumerate(uwb):
        for i,node in enumerate(nodes):
            if int(node.id) in anchor:rows.append(dataset.uwb_row(t-origin,message,i,tag,int(node.id),float(node.dis),config['uwb']['sigma_m'],config['uwb']['lever_arm_body_m'],anchor[int(node.id)],valid=float(node.dis)>0))
    common.write_csv(cache/'uwb.csv',common.UWB_FIELDS,rows)
    common.write_csv(cache/'ground_truth.csv',common.GT_FIELDS,[common.pose_row(t-origin,p.position,p.orientation,0) for t,p in gt])
    print('REAL_SOURCE',str(path),'sha256',common.sha256(path),'read_only8s','imu',len(imu),'ranges',len(rows),'GT_postscore',len(gt),'units livox_g_times9.80665/radps; identity-colocated seed, no GT fit/raw certificate',flush=True)
    return cache
def record():
    cache=prepare_record();pair={}
    for mode in ['off','on']:
        rows,log=native('config/sc4_system.yaml','record:'+str(cache),35,fde=mode,label='record-'+mode);pair[mode]=rows
        assert all(r['qualified']=='0' and r['scope']=='NONE' and r['source']=='RECORD_UNCALIBRATED' for r in rows)
        summarize('record-'+mode,rows,log)
        with (cache/'ground_truth.csv').open() as stream:gt=list(csv.DictReader(stream))
        times=np.array([float(g['t']) for g in gt]);positions=np.array([[float(g[k]) for k in ['px','py','pz']] for g in gt]);sample=np.array([float(r['source_time_origin_s'])+int(r['timestamp_ns'])*1e-9 for r in rows])
        assert sample.min()>=times.min() and sample.max()<=times.max()
        target=np.column_stack([np.interp(sample,times,positions[:,k]) for k in range(3)])
        actual=np.array([[float(r[k]) for k in STATE[:3]] for r in rows]);pe=np.linalg.norm(actual-target,axis=1)
        print('REAL_POST_SCORE',mode,'world-tag identity-colocated approximation PE mean/median/P95/max',statistics(pe),'RMSE',float(np.sqrt(np.mean(pe**2))),'not a safety bound',flush=True)
    print('REAL_RECORD actual35 off/on native epochs; calibration/admitted integrity/hardware TTA UNKNOWN; natural-fault detection rate UNKNOWN',flush=True)
def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('command',choices=['checks','local','performance','functional','record']);parser.add_argument('--build',action='store_true');args=parser.parse_args();CACHE.mkdir(parents=True,exist_ok=True)
    if args.build:subprocess.run(['cmake','--build',str(BUILD),'--target','sc4_system_checks','epoch_raim_runner','-j2'],cwd=ROOT,env=environment(),check=True)
    if args.command in ['checks','local']:
        command=[str(BIN/'sc4_system_checks')]+([] if args.command=='checks' else ['local']);subprocess.run(command,cwd=ROOT,env=environment(),check=True)
    else:globals()[args.command]()
if __name__=='__main__':main()
