#!/usr/bin/env python3
"""SC4 fixed replay, dependency-equivalence and runtime/application evaluation."""
import argparse,csv,hashlib,json,os,re,subprocess,time,tempfile
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
def rolling_native(config,scene,count,folder,label,enable=1):
    trace=folder/(label+'.csv')
    result=subprocess.run([str(BIN/'epoch_raim_runner'),str(config),str(trace),scene,'rolling','on',str(count),str(enable)],cwd=ROOT,env=environment(),capture_output=True,text=True)
    rows=[]
    if trace.exists():
        with trace.open() as stream:rows=list(csv.DictReader(stream))
    if result.returncode:
        actual=sum(int(r['updates']) for r in rows)
        terminal=sum(int(x) for x in re.findall(r'failed epoch=\d+ backend_updates=(\d+)',result.stderr))
        print('ROLLING FAILED',label,'scheduled',count,'receipts',actual,'terminal_updates',terminal,'unattempted',count-len(rows)-bool(terminal),result.stdout,result.stderr,flush=True)
        result.check_returncode()
    assert len(rows)==count and all(None not in r and None not in r.values() for r in rows),(label,'trace schema')
    print(result.stdout.strip(),flush=True)
    return rows

def rolling_validate(label,rows,lf=30,li=30,replay=50,mission=200):
    issued=[r for r in rows if r['qualified']=='1']
    assert len(issued)==min(len(rows),mission),label
    risk=issued[0]['risk_upper'];event=issued[0]['source_event']
    for r in issued:
        k=int(r['epoch'])
        assert r['updates']=='1' and int(r['version'])==k+1
        assert int(r['fgo_nodes'])==min(k+1,lf) and r['x0_present']==str(int(k<lf))
        assert int(r['boundary_epoch'])==max(0,k+1-li)
        assert int(r['generation'])==k and int(r['parent'])==k-1
        assert int(r['integrity_material'])<=li-1 and int(r['replay_samples'])<=10*replay+1 and int(r['capsules'])<=replay+1
        assert int(r['source_prefix'])==21+128*k and int(r['task_horizon'])==mission
        assert r['risk_upper']==risk and r['source_event']==event and float(risk)<=1e-6
        assert sum(int(r[x]) for x in ['not_started','active','ended_residual','excluded'])==28*mission
        assert int(r['shared_adc_index'])==10*int(r['boundary_epoch'])
        assert int(r['imu_read_first'])==10*(k-1)+1 and int(r['imu_read_last'])==10*k+1
        assert int(r['proof_uwb_first'])>=max(1,k-li+1) and int(r['proof_uwb_last'])==k
        assert float(r['horizontal_pe'])<=float(r['hpl']) and float(r['vertical_pe'])<=float(r['vpl'])
        assert r['application_status']=='APPLICATION_AL_UNQUALIFIED' and r['formal_eligible']==r['publication_protected']=='0'
    for r in rows[len(issued):]:
        assert r['bound_status']=='TASK_HORIZON_EXPIRED' and r['updates']=='0' and r['risk_upper']==risk and r['source_event']==event
    print('ROLLING DENOM',label,'scheduled',len(rows),'receipts',len(issued),'qualified',len(issued),'abort0','expired',len(rows)-len(issued),'core40_miss',sum(r['core_on_time']=='0' for r in issued),'full50_miss',sum(r['on_time']=='0' for r in issued),flush=True)
    for key in ['hpl','vpl','horizontal_pe','vertical_pe','core_ms','full_ms','resident_kib','factor_slots','active_factors','backend_values','ledger_entries','native_intervals','native_raw_samples','replay_samples','capsules','proof_bytes','physical_mask_visits']:
        print('ROLLING STATS',label,key,statistics([r[key] for r in issued]),flush=True)
    for k in [51,61,72,90,150,200]:
        if k<=len(issued):print('ROLLING SNAPSHOT',label,{x:issued[k-1][x] for x in ['epoch','qualified','x0_present','boundary_epoch','parent','proof_uwb_first','imu_read_first','recovery_status','imu_bridge','imu_rejoined','excluded_anchor','removed_priors','hpl','vpl','source_prefix','risk_upper']},flush=True)
    before=json.dumps(rows,sort_keys=True)
    for hal,val in [(2,3),(5,5),(10,10)]:
        count=sum(float(r['hpl'])<=hal and float(r['vpl'])<=val for r in issued)
        print('ROLLING AL',label,hal,val,'numeric',count,'application_available0',flush=True)
    assert before==json.dumps(rows,sort_keys=True),'AL changed immutable native trace'
    return issued

def rolling(main_only=False,remaining_only=False):
    root=ROOT/'.cache/sc5-rolling';root.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='matrix-',dir=root) as name:
        folder=Path(name);config=yaml.safe_load((ROOT/'config/sc5_rolling.yaml').read_text())
        if not remaining_only:
            for label,scene in [('R0','SC3_U0'),('R1','SC3_A'),('R2','SC3_G')]:
                rows=rolling_native('config/sc5_rolling.yaml',scene,200,folder,label)
                issued=rolling_validate(label,rows)
                if label!='R0':
                    assert issued[60]['imu_bridge']=='1' and issued[60]['excluded_anchor']=='0' and issued[60]['removed_priors']=='1'
                    assert all(r['imu_rejoined']=='1' for r in issued[71:75])
                else:assert all(r['imu_bridge']=='0' for r in issued)
        if main_only:return
        delayed=dict(config,sc2_witness_onset_epoch=20,sc3_uwb_witness_onset_epoch=20)
        delayed['rolling_integrity']=dict(config['rolling_integrity'],recovery_extra_epochs=30)
        path=folder/'delayed.yaml';path.write_text(yaml.safe_dump(delayed,sort_keys=False))
        rows=rolling_native(path,'SC3_A',80,folder,'R3',55);rolling_validate('R3',rows,replay=60)
        assert rows[54]['history_marginalized']==rows[54]['boundary_repair']=='1' and rows[54]['removed_priors']=='1'
        assert all(r['imu_rejoined']=='1' for r in rows[55:60])
        for lf,li in [(20,20),(40,40),(30,10)]:
            variant=dict(config,fixed_lag_epochs=lf)
            variant['rolling_integrity']=dict(config['rolling_integrity'],window_mode='follow_fgo' if lf==li else 'explicit',explicit_epochs=None if lf==li else li)
            path=folder/f'{lf}-{li}.yaml';path.write_text(yaml.safe_dump(variant,sort_keys=False))
            rows=rolling_native(path,'SC3_U0',70,folder,f'R4-{lf}-{li}');rolling_validate(f'R4-{lf}-{li}',rows,lf,li,lf+20)
        delayed['rolling_integrity']=dict(config['rolling_integrity'],recovery_extra_epochs=0)
        path=folder/'outside.yaml';path.write_text(yaml.safe_dump(delayed,sort_keys=False))
        rows=rolling_native(path,'SC3_A',60,folder,'R5-outside',55);rolling_validate('R5-outside',rows,replay=30)
        assert all(r['recovery_status']=='BOUND_VALID_BUT_RECOVERY_UNAVAILABLE' and r['boundary_repair']=='0' for r in rows[54:])
        short=dict(config);short['rolling_integrity']=dict(config['rolling_integrity'],mission_epochs=2)
        path=folder/'expiry.yaml';path.write_text(yaml.safe_dump(short,sort_keys=False))
        rows=rolling_native(path,'SC3_U0',3,folder,'R5-expiry');rolling_validate('R5-expiry',rows,mission=2)
    print('ROLLING matrix PASS; temporary traces removed; math/recovery separate from realtime/AL/hardware',flush=True)

def open_native(config,scene,count,folder,label,hook=None):
    trace=folder/(label+'.csv')
    cmd=[str(BIN/'epoch_raim_runner'),str(config),str(trace),scene,'open-rolling','on',str(count)]
    if hook:cmd.append(hook)
    r=subprocess.run(cmd,cwd=ROOT,env=environment(),capture_output=True,text=True)
    rows=[]
    if trace.exists():
        with trace.open() as f:rows=list(csv.DictReader(f))
    print(r.stdout.strip(),flush=True)
    if r.returncode:
        print('OPEN FAILURE',label,'scheduled',count,'emitted',len(rows),'stderr',r.stderr,'unattempted',count-len(rows),flush=True)
        r.check_returncode()
    assert len(rows)==count
    return rows

def open_validate(label,rows,lf=30,li=30,replay=60,repeated=False):
    previous=0
    for x in rows:
        k=int(x['epoch'])
        assert None not in x and None not in x.values(),(label,k,'schema')
        assert x['qualified']==x['updates']=='1' and int(x['version'])==k+1
        assert int(x['generation'])==k and int(x['parent'])==k-1 and int(x['source_prefix'])==21+128*k
        assert int(x['boundary'])==max(0,k+1-li) and x['x0']==str(int(k<lf))
        risk=float(x['risk_upper']);assert previous<risk<=1e-6;previous=risk
        assert x['event'].startswith('time-uniform-block-rw-repeat-v1:')
        assert int(x['replay_samples'])<=10*replay+1 and int(x['capsules'])<=replay+1 and int(x['explicit_batches'])<=li-1
        assert int(x['proof_uwb_first'])>=max(1,k-li+1) and int(x['imu_first'])==10*(k-1)+1
        assert float(x['horizontal_pe'])<=float(x['hpl']) and float(x['vertical_pe'])<=float(x['vpl'])
        assert all(np.isfinite(float(x[key])) for key in ['tag_x','tag_y','tag_z','vx','vy','vz','qw','qx','qy','qz'])
        assert x['formal_eligible']==x['publication_protected']=='0' and x['application_status']=='APPLICATION_AL_UNQUALIFIED'
        assert x['current_protected']==str(int(x['on_time']=='1'))
    repairs=[x for x in rows if x['prior_repaired']=='1']
    if repeated:
        assert len(repairs)==2 and rows[-1]['alerts']=='2',(label,'repeat recovery',[(x['epoch'],x['alerts']) for x in repairs])
        for repair in repairs:
            tail=[x for x in rows if int(x['epoch'])>int(repair['epoch']) and x['alerts']==repair['alerts'] and x['rejoined']=='1']
            assert len(tail)>=4
            print('OPEN RECOVERY',label,'repair',repair['epoch'],'removed_priors',repair['removed_priors'],'removed_groups',repair['removed_groups'],'rejoin4',[x['epoch'] for x in tail[:4]],flush=True)
    print('OPEN DENOM',label,'scheduled',len(rows),'native',len(rows),'qualified',len(rows),'abort0','core40_miss',sum(x['core_on_time']=='0' for x in rows),'full50_miss',sum(x['on_time']=='0' for x in rows),flush=True)
    for key in ['hpl','vpl','width_x','width_y','width_z','horizontal_pe','vertical_pe','bias_width','core_ms','full_ms','resident_kib','slots','live_factors','ledger_entries','active_imu','ended_imu','births']:
        print('OPEN STATS',label,key,'min',min(float(x[key]) for x in rows),'mean/median/P95/max',statistics([x[key] for x in rows]),flush=True)
    for k in [51,200,201,321,500,1000]:
        if k<=len(rows):print('OPEN SNAPSHOT',label,{key:rows[k-1][key] for key in ['epoch','qualified','event','risk_upper','q','q_shared_first','generation','parent','boundary','source_prefix','alerts','ended_imu','recovery','hpl','vpl','proof_uwb_first','imu_first']},flush=True)
    before=json.dumps(rows,sort_keys=True)
    for hal,val in [(2,3),(5,5),(10,10)]:print('OPEN AL',label,hal,val,'numeric',sum(float(x['hpl'])<=hal and float(x['vpl'])<=val for x in rows),'application_available0',flush=True)
    assert before==json.dumps(rows,sort_keys=True)

def open_matrix(remaining=False):
    root=ROOT/'.cache/sc5-open';root.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='matrix-',dir=root) as name:
        folder=Path(name)
        if not remaining:
            for label,scene,count in [('O0','OE_HEALTHY',500),('O1','OE_MIXED',1000),('O2','OE_GYRO',500)]:
                rows=open_native('config/sc5_open.yaml',scene,count,folder,label)
                open_validate(label,rows,repeated=label!='O0')
        else:
            config=yaml.safe_load((ROOT/'config/sc5_open.yaml').read_text())
            for lf,li in [(20,20),(40,40),(30,10)]:
                v=dict(config,fixed_lag_epochs=lf);v['rolling_integrity']=dict(config['rolling_integrity'],window_mode='follow_fgo' if lf==li else 'explicit',explicit_epochs=None if lf==li else li)
                path=folder/f'window-{lf}-{li}.yaml';path.write_text(yaml.safe_dump(v,sort_keys=False))
                rows=open_native(path,'OE_HEALTHY',250,folder,f'O3-{lf}-{li}');open_validate(f'O3-{lf}-{li}',rows,lf,li,lf+30)
            for hook in ['cancel:3','overload:3']:
                rows=open_native('config/sc5_open.yaml','OE_HEALTHY',8,folder,hook.replace(':','-'),hook)
                assert all(x['qualified']==x['current_protected']==x['updates']=='0' for x in rows[2:])
                print('OPEN NEGATIVE',hook,'2 native/qualified,6 refused,0 stale protection',flush=True)
    print('OPEN MATRIX PASS transient artifacts removed',flush=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('command',choices=['checks','local','performance','functional','record','rolling-check','rolling','rolling-boundary','open-check','open','open-remaining']);parser.add_argument('--build',action='store_true');parser.add_argument('--main-only',action='store_true');parser.add_argument('--remaining-only',action='store_true');args=parser.parse_args();CACHE.mkdir(parents=True,exist_ok=True)
    if args.command.startswith('open'):
        if args.build:subprocess.run(['cmake','--build',str(BUILD),'--target','time_uniform_event_checks','open_rolling_checks','epoch_raim_runner','-j2'],cwd=ROOT,env=environment(),check=True)
        if args.command=='open-check':
            for target in ['time_uniform_event_checks','open_rolling_checks']:subprocess.run([str(BIN/target)],cwd=ROOT,env=environment(),check=True)
        else:open_matrix(args.command=='open-remaining')
        return
    if args.command.startswith('rolling'):
        if args.build:subprocess.run(['cmake','--build',str(BUILD),'--target','sc5_rolling_checks','sc5_math_checks','sc5_event_checks','epoch_raim_runner','-j2'],cwd=ROOT,env=environment(),check=True)
        if args.command=='rolling':rolling(args.main_only,args.remaining_only)
        else:
            for target,extra in ([('sc5_rolling_checks',['boundary'])] if args.command=='rolling-boundary' else [('sc5_rolling_checks',[]),('sc5_rolling_checks',['boundary']),('sc5_math_checks',[]),('sc5_event_checks',[])]):subprocess.run([str(BIN/target),*extra],cwd=ROOT,env=environment(),check=True)
        return
    if args.build:subprocess.run(['cmake','--build',str(BUILD),'--target','sc4_system_checks','epoch_raim_runner','-j2'],cwd=ROOT,env=environment(),check=True)
    if args.command in ['checks','local']:
        command=[str(BIN/'sc4_system_checks')]+([] if args.command=='checks' else ['local']);subprocess.run(command,cwd=ROOT,env=environment(),check=True)
    else:globals()[args.command]()
if __name__=='__main__':main()
