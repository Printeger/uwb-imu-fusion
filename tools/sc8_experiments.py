#!/usr/bin/env python3
"""SC8 reproducible registry/runner. Never reuses historical result CSVs."""
import argparse
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml
import numpy as np

import benchmark_common as common
import run_dataset_benchmark as datasets
from sc4_system_research import BIN, BUILD, environment
from sc6_closure import law

ROOT=common.ROOT
CACHE=ROOT/'.cache/sc8'
REPORT=ROOT/'docs/sc8'
REGISTRY=ROOT/'config/sc8_experiments.yaml'
SEEDS=[20261020,20261021,20261022]
CANDIDATES=[('R1','sfuise','ISAS-Walk2'),('R2','sfuise','ISAS-Walk3'),
 ('R3','own_vicon','2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle'),
 ('R4','own_vicon','2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle'),
 ('R5','starloc','loop-3d-z_s3'),('R6','starloc','loop-2d-fast_s3'),
 ('CAL-W','sfuise','ISAS-Walk1'),
 ('CAL-O','own_vicon','2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle'),
 ('CAL-S','starloc','loop-2d_s3')]


def save(path,obj):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n');temp.replace(path)


def sha(path):return common.sha256(Path(path))


def command(args,log,env=None,timeout_s=None):
    log=Path(log);log.parent.mkdir(parents=True,exist_ok=True)
    env=env or dict(environment(),UWB_IMU_PL_CANDIDATE_WORKERS="1")
    with log.open('w') as f:
        if timeout_s is None:
            p=subprocess.run([str(x) for x in args],cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT)
            code=p.returncode
        else:
            start=time.monotonic()
            p=subprocess.Popen([str(x) for x in args],cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT)
            while p.poll() is None:
                if time.monotonic()-start>timeout_s:
                    p.terminate()
                    save(log.parent/'resource_limit.json',dict(reason='REGISTERED_WALL_LIMIT',wall_limit_s=timeout_s))
                    break
                time.sleep(.5)
            try:code=p.wait(timeout=10)
            except subprocess.TimeoutExpired:p.kill();code=p.wait()
    print(log.name,'exit',code,log.read_text()[-500:],flush=True)
    return code


def prepare():
    """Fresh adapters; CAL-only transforms. Does not inspect algorithm APE."""
    import pandas as pd
    from scipy.spatial.transform import Rotation
    inventory=[]
    for rid,family,name in CANDIDATES:
        config=datasets.load_config(ROOT/f'config/benchmark/{family}.yaml')
        seq=next(s for s in datasets.sequences(config) if s['id']==name)
        dest=CACHE/'data'/rid
        if not (dest/'manifest.json').exists():
            print('PREPARE',rid,name,flush=True)
            fn={'sfuise':datasets.prepare_sfuise,'own_vicon':datasets.prepare_own,'starloc':datasets.prepare_starloc}[family]
            fn(config,seq,dest)
        inventory.append(dict(id=rid,family=family,sequence=name,cache=str(dest.relative_to(ROOT))))
    caldir=CACHE/'calibration';caldir.mkdir(parents=True,exist_ok=True)
    calw=caldir/'walk1_world.json'
    if not calw.exists():
        cal=datasets.fit_sfuise_world_calibration([CACHE/'data/CAL-W'])
        cal.update(scope='CAL-W_ISAS-Walk1_ONLY',test_involved=False,
                   identification='range-derived rigid world fit; model adequacy not guaranteed')
        save(calw,cal)
        datasets.apply_sfuise_world_calibration([CACHE/'data'/x for x in ['CAL-W','R1','R2']],cal,calw)
    calo=caldir/'own_prefix20.json'
    if not calo.exists():
        # CAL-O is near-static: gyro excitation below the recorded fit noise.
        # Preserve its failed identification diagnostic, use a fixed R3 prefix.
        for rid in ['CAL-O','R3','R4']:
            config=datasets.load_config(ROOT/'config/benchmark/own_vicon.yaml')
            name=next(x[2] for x in CANDIDATES if x[0]==rid)
            seq=next(x for x in config['sequences'] if x['id']==name)
            datasets.prepare_own(config,seq,CACHE/'data'/rid)
        prefix=CACHE/'calibration/own-prefix-input';prefix.mkdir(parents=True,exist_ok=True)
        import pandas as pd
        start=float(pd.read_csv(CACHE/'data/R3/imu.csv').t.iloc[0])
        end=float(pd.read_csv(CACHE/'data/R3/imu.csv').t.iloc[-1])
        cut=start+.2*(end-start)
        for name in ['imu.csv','gt.csv','uwb.csv']:
            df=pd.read_csv(CACHE/'data/R3'/name)
            df[df.t<=cut].to_csv(prefix/name,index=False,float_format='%.17g')
        cal=datasets.fit_own_vicon_calibration([prefix])
        a,b=datasets._own_gyro_pairs([prefix],cal['time_imu_to_gt_s'])
        singular=np.linalg.svd(a-a.mean(axis=0),compute_uv=False)
        cal.update(scope='R3_FIXED_FIRST_20_PERCENT',test_involved=True,
                   cal_end_sensor_s=cut,isolation_s=2.,test_begin_s=cut+2.,
                   angular_excitation_singular_values=singular.tolist(),
                   full_6d_identifiability='CALIBRATION_OPEN: no independent lever/time truth; fit diagnostics only')
        save(calo,cal)
        datasets.apply_own_vicon_calibration([CACHE/'data'/x for x in ['CAL-O','R3','R4']],cal,calo)
    for entry in inventory:
        rid=entry['id'];dest=CACHE/'data'/rid
        doc=json.loads((dest/'manifest.json').read_text())
        # Fresh calibration functions have historical default provenance labels;
        # resolve their actual source, never inherit an all-three claim.
        doc['sc8_calibration_scope']=('CAL-W_ONLY' if entry['family']=='sfuise' else
                                      'R3_PREFIX20_TEST_INVOLVED_CALIBRATION_OPEN' if entry['family']=='own_vicon' else 'OFFICIAL')
        doc['test_involved_calibration']=entry['family']=='own_vicon'
        doc['statistical_qualification']='UNQUALIFIED_REAL_SOURCE'
        doc['files']=datasets.rows_file_metadata(dest)
        save(dest/'manifest.json',doc)
        try: audit=common.validate_cache(dest)
        except Exception as e: audit={'status':'FAIL','reason':str(e)}
        imu=pd.read_csv(dest/'imu.csv');uwb=pd.read_csv(dest/'uwb.csv');gt=pd.read_csv(dest/'gt.csv')
        primary=uwb[uwb.tag_id==doc['primary_tag']]
        times=primary.t.to_numpy()
        spans=[];raw_time_groups=[]
        window=doc['epoch_batch_window_s'];begin=0
        messages=primary.source_message.to_numpy()
        while begin<len(times):
            end=begin+1
            while end<len(times) and (messages[end]==messages[begin] or times[end]-times[begin]<=window+1e-12):end+=1
            spans.append(float(times[end-1]-times[begin]));raw_time_groups.append(end-begin);begin=end
        entry.update(manifest_sha256=sha(dest/'manifest.json'),files=doc['files'],
                     provenance=doc['provenance'],calibration_scope=doc['sc8_calibration_scope'],
                     source=doc['source'],audit=audit,
                     anchors=int(uwb.anchor_id.nunique()),tags=int(uwb.tag_id.nunique()),
                     imu_hz=float(1/np.median(np.diff(imu.t))),imu_duration_s=float(imu.t.iloc[-1]-imu.t.iloc[0]),
                     imu_max_gap_s=float(np.diff(imu.t).max()),gt_max_gap_s=float(np.diff(np.unique(gt.t)).max()),
                     batch=dict(groups=len(spans),max_span_s=max(spans),p95_span_s=float(sorted(spans)[math.ceil(.95*len(spans))-1]),
                                retimestamp_to='LAST_RANGE_FOR_SFUISE',original_range_timestamps_preserved=True,
                                span_model='ASYNC_APPROXIMATION_UNQUALIFIED' if max(spans)>1e-8 else 'native_same_stamp'),
                     requested_method='O-FULL',qualification='RAW_SOURCE_REFUSED',
                     refusal_reasons=['unregistered_initial_law','uncertified_noise_and_motion_domain','native_imu_rate_and_anchor_inventory'])
    save(CACHE/'inventory.json',inventory)
    print('INVENTORY',len(inventory),'fresh inputs; own prefix calibration is disclosed test-involved/open',flush=True)


def preflight(build=False):
    CACHE.mkdir(parents=True,exist_ok=True)
    import run_sfuise_benchmark as sfuise
    pin=sfuise.checked_pin()
    entry={'head':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
           'status':subprocess.check_output(['git','status','--short'],text=True),
           'platform':common.system_identity(),'baseline_pin':pin,
           'baseline_remote':subprocess.check_output(['git','-C',str(sfuise.SFUISE_ROOT),'remote','-v'],text=True),
           'affinity':sorted(os.sched_getaffinity(0)),
           'source_documents':{str(p):sha(ROOT/p) for p in map(Path,[
               'docs/requirements/sc8/SC8_EXPERIMENT_HANDBOOK.md',
               'docs/fde_next/SC7_RESULTS.md','docs/fde_next/SC6_CLOSURE_RESULTS.md',
               'docs/benchmark/uwb_imu_accuracy_timing_report.md'])}}
    if not (CACHE/'entry.json').exists():save(CACHE/'entry.json',entry)
    if build:
        code=command(['cmake','--build',BUILD,'--target','epoch_raim_runner','sc7_checks','nominal_dataset_runner','-j2'],CACHE/'build.log')
        if code:raise RuntimeError('build failed')
    prepare()



def registry_design():
    cases={}
    def case(name,epochs=600,motion='M3',geometry='G1',imu=(),uwb=()):
        cases[name]=dict(epochs=epochs,motion=motion,geometry=geometry,
                         faults=[list(x) for x in imu],uwb_faults=[list(x) for x in uwb])
    case('SC8-H');case('SC8-U',uwb=[(121,0,3,2.25)])
    case('SC8-A',imu=[(121,0,500,20.,0)])
    case('SC8-G',imu=[(121,3,500,1.,0)])
    case('SC8-J',imu=[(121,0,500,20.,0)],uwb=[(121,0,3,2.25)])
    case('SC8-D0',motion='M0',geometry='G0')
    case('SC8-G2',geometry='G2')
    case('SC8-WH',epochs=2100)
    case('SC8-WJ',epochs=2100,imu=[(1201,0,500,20.,0),(1601,3,500,1.,0)],
         uwb=[(1201,0,3,2.25),(1601,0,3,2.25)])
    for axis in range(6):
        case(f'SC8-AX{axis}',epochs=160,motion='M0',geometry='G0',
             imu=[(61,axis,500,20. if axis<3 else 1.,0)])
    case('SC8-WEAK',epochs=200,imu=[(61,0,50,1.,0)])
    case('SC8-NEG',epochs=200,imu=[(61,3,500,-1.,0)])
    case('SC8-STAGGER',epochs=200,imu=[(61,0,500,20.,0)],uwb=[(58,0,3,2.25)])
    for name in ['L0','L1']:
        original=law()['sc6']['cases'][name]
        case('SC8-'+name,epochs=original['epochs'],motion=original['motion'],geometry=original['geometry'],
             imu=original['faults'],uwb=[(f[4],0,3,2.25) for f in original['faults']])
    entries=[]
    def add(group,stream,method,seed=SEEDS[0],window=(30,30),**more):
        label=f'{group}-{stream}-{method}-{seed}-{window[0]}x{window[1]}'
        entry=dict(id=label,group=group,stream=stream,requested_method=method,seed=seed,
                   window=list(window),pace='offline' if group=='E05' else 'paced',**more)
        entry['track']='T-SIM' if stream.startswith('SC8-') else 'T-SEMI' if '-' in stream else 'T-REAL'
        entries.append(entry)
    # Interface prefixes are fixed in advance, not selected by measured APE.
    for rid,_,_ in CANDIDATES[:6]:
        for method in ['O-NOM','O-FULL','SFUISE']:
            add('E00',rid,method,prefix_s=20.)
    add('E00','SC8-H','O-FULL',epochs=80,hook='audit')
    add('E00','SC8-H','O-NOFDE',epochs=80,hook='plain')
    for stream,seeds in [('SC8-H',SEEDS),('SC8-D0',SEEDS[:1]),('SC8-G2',SEEDS[:1])]:
        for seed in seeds:
            for method in ['O-FULL','O-NOFDE','SFUISE']:add('E01',stream,method,seed)
    add('E01','SC8-H','O-NOM')
    for rid,_,_ in CANDIDATES[:6]:
        for method in ['O-FULL','O-NOFDE','SFUISE']:add('E02',rid,method)
    for stream in ['SC8-U','SC8-A','SC8-G','SC8-J']:
        for seed in SEEDS:
            for method in ['O-FULL','O-NOFDE','SFUISE']:add('E03',stream,method,seed)
    for rid in ['R2','R3']:
        for fault in ['FU','FG','FJ']:
            for method in ['O-FULL','O-NOFDE','SFUISE']:add('E03',rid+'-'+fault,method)
    for stream in [*[f'SC8-AX{i}' for i in range(6)],'SC8-WEAK','SC8-NEG','SC8-STAGGER']:
        add('E03',stream,'O-FULL')
    pairs=[(30,30),(100,100),(300,300),(600,600),(1000,1000),(300,30),(300,100)]
    for window in pairs:
        for stream in ['SC8-WH','SC8-WJ']:add('E05',stream,'O-FULL',window=window)
    for size in [100,300,600,1000]:
        add('E05',f'SC8-WH','O-CANARY',window=(size,size),epochs=size+2)
    for stream in ['SC8-D0','SC8-J']:
        for variant in ['O-ABL-SUPPORT','O-ABL-JOINT']:add('E07',stream,variant)
    add('E07','SC8-G2','O-ABL-G2',epochs=80)
    for stream,method in [('SC8-L0','O-FULL'),('SC8-L1','O-FULL'),('SC8-L1','O-NOFDE')]:add('E08',stream,method)
    assert len(entries)<=160 and len({e['id'] for e in entries})==len(entries)
    available=int(next(l.split()[1] for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith('MemAvailable:')))*1024
    return dict(schema='sc8-experiments-v1',entry_head=json.loads((CACHE/'entry.json').read_text())['head'],
                phase='W0_INTERFACE_VALIDATION',contract='docs/requirements/sc8/SC8_EXPERIMENT_HANDBOOK.md',
                seeds=SEEDS,solver_processes=len(entries),maximum_solver_processes=160,
                resource_stops=dict(rss_bytes=min(8*1024**3,available//2),scaling_wall_s=1200,consecutive_core_over_2s=20),
                evaluation=dict(main_alignment='NONE',statistics_unit='seed_or_recording',
                    rpe_s=[.5,1.],research_AL_m=[[2,3],[5,5],[10,10]],
                    core_p95_ms=40.,sink_p95_ms=50.,timely_spatial_min=.95,
                    scaling_common_epochs=[1101,2100],
                    own_calibration='R3_FIXED_FIRST_20_PERCENT_THEN_2S_ISOLATION_TEST_INVOLVED',
                    SFUISE_initialization='OFFICIAL_NATIVE_SENSOR_ONLY; independent prior not consumed; separate initial-information contract',
                    real_effective_on='O-FDE-ENG_GENERAL_PRODUCTION_PIPELINE_NOT_SC7',
                    real_effective_off='O-ENG-NOFDE_GENERAL_PRODUCTION_PIPELINE',
                    real_raw_source='REFUSED; no noise/frame/anchor/rate theorem extension'),
                inventory=json.loads((CACHE/'inventory.json').read_text()),cases=cases,runs=entries)


def register():
    if REGISTRY.exists():
        print('REGISTERED registry retained',REGISTRY,flush=True);return
    d=registry_design();REGISTRY.write_text(yaml.safe_dump(d,sort_keys=False,allow_unicode=True))
    print('REGISTERED',len(d['runs']),'solver processes',flush=True)


def sim_config(entry,registry):
    result={'raw_base_law':'config/sc7.yaml','fixed_lag_epochs':entry['window'][0],
            'closure':copy.deepcopy(law()['closure'])}
    result['closure']['additional_cases']={entry['stream']:registry['cases'][entry['stream']]}
    lf,li=entry['window']
    result['rolling_integrity']=dict(law()['rolling_integrity'],window_mode='follow_fgo' if lf==li else 'explicit',
                                     explicit_epochs=li if lf!=li else None)
    if entry['requested_method'].startswith('O-ABL'):
        result['sc7']=copy.deepcopy(law(ROOT/'config/sc7.yaml')['sc7'])
        if entry['requested_method']=='O-ABL-SUPPORT':result['sc7']['tag_support']=False
        if entry['requested_method']=='O-ABL-JOINT':result['sc7'].update(joint_raw=False,tag_support=False)
        if entry['requested_method']=='O-ABL-G2':result['sc7']['numerical_recovery']=False
    return result


def physical_digest(cache):
    # Canonical ordered multiset, all original times/IDs/validity kept.
    h=hashlib.sha256()
    for name in ['imu.csv','uwb.csv','anchors.csv']:
        h.update(name.encode()+b'\0');h.update((cache/name).read_bytes())
    return h.hexdigest()


def input_for(entry,registry):
    stream=entry['stream'];name=stream+'-'+str(entry['seed'])
    if 'prefix_s' in entry:name+='-prefix'+str(entry['prefix_s'])
    dest=CACHE/'inputs'/name
    if (dest/'manifest.json').exists():return dest
    dest.mkdir(parents=True,exist_ok=True)
    if stream.startswith('SC8-'):
        config=sim_config(entry,registry)
        # Input generation is independent of window/method. No FGO solve.
        config['fixed_lag_epochs']=30
        config['rolling_integrity']=dict(law()['rolling_integrity'])
        config_path=dest/'source.yaml';config_path.write_text(yaml.safe_dump(config))
        audit=dest/'raw.input'
        count=registry['cases'][stream]['epochs']
        code=command([BIN/'sc7_checks','inputs',config_path,stream,count,entry['seed'],audit],dest/'generate.log')
        if code:raise RuntimeError('source generation failed')
        import run_dataset_benchmark as ds
        from scipy.spatial.transform import Rotation
        imu=[];uwb=[];anchors={};initial=None;resolved_law=law()
        for line in audit.read_text().splitlines():
            a=line.split()
            if not a:continue
            if a[0]=='initial':initial=a
            if a[0]=='imu':
                imu.append(dict(zip(common.IMU_FIELDS,[int(a[2])*1e-9,*map(float.fromhex,a[4:10])])))
            if a[0]=='uwb':
                aid=int(a[5]);anchor=list(map(float.fromhex,a[10:13]));anchors[aid]=anchor
                uwb.append(ds.uwb_row(int(a[9])*1e-9,int(a[1]),int(a[3]),int(a[6]),aid,float.fromhex(a[13]),float.fromhex(a[14]),resolved_law['lever'],anchor))
        common.write_csv(dest/'imu.csv',common.IMU_FIELDS,imu)
        common.write_csv(dest/'uwb.csv',common.UWB_FIELDS,uwb)
        common.write_csv(dest/'anchors.csv',common.ANCHOR_FIELDS,
                         [dict(zip(common.ANCHOR_FIELDS,[k,*v])) for k,v in anchors.items()])
        motion=registry['cases'][stream]['motion']
        times=np.arange(count+1)*.05
        if motion=='M0':
            p=np.sin(times[:,None]*[.5,.4,.3])*[.1,.08,.05]+[0,0,1]
            axis=np.array([1,2,3])/np.sqrt(14);q=Rotation.from_rotvec(.2*times[:,None]*axis)
        else:
            p=np.column_stack([.8*np.sin(.6*times),.6*np.sin(.5*times),1+.4*np.sin(.4*times)])
            q=(Rotation.from_euler('x',.3*np.sin(.4*times))*Rotation.from_euler('y',.4*np.sin(.3*times))*Rotation.from_euler('z',.5*np.sin(.2*times)))
        tag=p+q.apply(law()['lever'])
        common.write_csv(dest/'gt.csv',common.GT_FIELDS,
                         [dict(zip(common.GT_FIELDS,[t,*pp,*qq,int(uwb[0]['tag_id'])])) for t,pp,qq in zip(times,tag,q.as_quat())])
        manifest=dict(schema=common.CACHE_SCHEMA,dataset='SC8-SIM',sequence=stream,primary_tag=int(uwb[0]['tag_id']),
                      epoch_batch_window_s=0.,bootstrap_prefer_below_anchors=False,
                      initial_observation=dict(t=int(initial[1])*1e-9,p=list(map(float.fromhex,initial[2:5])),
                        v=list(map(float.fromhex,initial[5:8])),q=list(map(float.fromhex,initial[8:12])),
                        ba=list(map(float.fromhex,initial[12:15])),bg=list(map(float.fromhex,initial[15:18]))),
                      estimator_inputs=['imu.csv','uwb.csv','anchors.csv'],evaluator_only=['gt.csv'],
                      provenance=dict(source='Sc6Provider',raw_audit_sha256=sha(audit),
                                      law_sha256=sha(config_path),seed=entry['seed']))
    else:
        rid=stream.split('-')[0];src=CACHE/'data'/rid
        manifest=json.loads((src/'manifest.json').read_text())
        rows={n:common.read_csv(src/n) for n in ['imu.csv','uwb.csv','anchors.csv','gt.csv']}
        affected=[]
        if '-' in stream:
            fault=stream.split('-')[1]
            # R3 injection occurs strictly inside the held-out TEST suffix.
            onset=20.005 if rid=='R3' else 6.005;uwb_onset=onset+.045
            anchor=min(int(r['anchor_id']) for r in rows['uwb.csv'])
            if fault in ['FG','FJ']:
                axis='gx' if fault=='FG' else 'ax';amplitude=1. if fault=='FG' else 20.
                for i,row in enumerate(rows['imu.csv']):
                    if onset<=float(row['t'])<onset+.5:
                        row[axis]=float(row[axis])+amplitude;affected.append(['imu',i,float(row['t']),axis,amplitude])
            if fault in ['FU','FJ']:
                for row in rows['uwb.csv']:
                    if int(row['anchor_id'])==anchor and uwb_onset<=float(row['t'])<uwb_onset+.15:
                        for field in ['range','range_raw']:row[field]=float(row[field])+2.25
                        affected.append(['uwb',int(row['source_message']),int(row['source_range']),float(row['t']),anchor,2.25])
            manifest['overlay']=dict(id=stream,layer='RAW_BEFORE_INTEGRATION',onset_s=onset,affected=affected,
                                     base_digest=physical_digest(src),GT_changed=False,qualification='UNQUALIFIED_REAL_SOURCE')
        if 'prefix_s' in entry:
            cutoff=float(entry['prefix_s'])
            for n in ['imu.csv','uwb.csv','gt.csv']:rows[n]=[r for r in rows[n] if float(r['t'])<=cutoff]
        for n,fields in [('imu.csv',common.IMU_FIELDS),('uwb.csv',common.UWB_FIELDS),('anchors.csv',common.ANCHOR_FIELDS),('gt.csv',common.GT_FIELDS)]:
            common.write_csv(dest/n,fields,rows[n])
    manifest['files']=datasets.rows_file_metadata(dest)
    manifest['physical_input_sha256']=physical_digest(dest)
    save(dest/'manifest.json',manifest)
    return dest


def dataset_config(entry,inputs,run):
    mode=entry['requested_method'];base='fde_joint_order1.yaml' if mode=='O-FULL' else 'fde_off.yaml'
    cfg=yaml.safe_load((ROOT/'config'/base).read_text())
    cfg['fault_models']['manifest_path']=str(ROOT/'config/integrity_fault_manifest.yaml')
    # Time validity comes from metadata only; never changes sigma/noise/risk.
    imu=common.read_csv(inputs/'imu.csv');maxgap=max(np.diff([float(r['t']) for r in imu]))
    cfg['imu']['max_gap_s']=max(cfg['imu']['max_gap_s'],float(maxgap)+1e-6)
    manifest=json.loads((inputs/'manifest.json').read_text());window=manifest.get('epoch_batch_window_s',0.)
    cfg['incremental']['max_time_skew_s']=max(cfg['incremental']['max_time_skew_s'],window+1e-6)
    cfg['incremental']['epoch_bin_s']=max(cfg['incremental']['epoch_bin_s'],cfg['incremental']['max_time_skew_s'])
    cfg['incremental']['fixed_lag_epochs']=entry['window'][0]
    cfg.pop('estimation_tuning',None)
    if mode!='O-FULL':cfg['estimation_tuning']={'version':1,'causal_bootstrap':True}
    cfg['anchors']=[dict(id=int(r['anchor_id']),position_m=[float(r[k]) for k in ['x','y','z']],
                         sigma_m=.05,frame=cfg['realtime']['world_frame'],map_version='SC8-'+entry['stream'])
                    for r in common.read_csv(inputs/'anchors.csv')]
    path=run/'effective_nominal_config.yaml';path.write_text(yaml.safe_dump(cfg,sort_keys=False));return path


def current_identity():
    paths=subprocess.check_output(['git','ls-files','research','src','include','apps','tools','benchmark','config','CMakeLists.txt'],text=True).splitlines()
    paths+=['tools/sc8_experiments.py','tools/sc8_metrics.py','tools/sc8_sfuise_observer.py','tools/test_sc8_metrics.py']
    digest=hashlib.sha256()
    for p in sorted(set(paths)):
        if p=='config/sc8_experiments.yaml':continue
        if (ROOT/p).is_file():digest.update(p.encode()+b'\0'+(ROOT/p).read_bytes())
    sfuise_ws=ROOT.parents[1]/'.uwb_benchmark/sfuise_isolated_ws/devel/lib'
    binaries=[BIN/'epoch_raim_runner',BIN/'nominal_dataset_runner',BIN.parent/'libuwb_imu_pl.so',
              sfuise_ws/'uwb_benchmark_sfuise_adapter/sfuise_sc8_observed',
              sfuise_ws/'uwb_benchmark_sfuise_adapter/sfuise_trajectory_adapter',sfuise_ws/'sfuise/SplineFusion']
    return dict(head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                source_content_sha256=digest.hexdigest(),binaries={str(p):sha(p) for p in binaries if p.exists()})


def run_one(entry,registry,retry_failed=False):
    run=CACHE/'runs'/entry['id'];run.mkdir(parents=True,exist_ok=True)
    ident=current_identity();status_path=run/'execution.json'
    attempt=1
    while (CACHE/'runs'/(entry['id']+'-attempt'+str(attempt+1))/'execution.json').exists():
        attempt+=1
    if attempt>1:
        run=CACHE/'runs'/(entry['id']+'-attempt'+str(attempt));status_path=run/'execution.json'
    if retry_failed and status_path.exists() and json.loads(status_path.read_text())['status'] not in ['SUCCESS','RUNNING']:
        attempt+=1;run=CACHE/'runs'/(entry['id']+'-attempt'+str(attempt));run.mkdir(parents=True,exist_ok=True);status_path=run/'execution.json'
    if status_path.exists():
        previous=json.loads(status_path.read_text())
        if entry['group']!='E00' and (previous['identity']['source_content_sha256']!=ident['source_content_sha256'] or previous['identity']['binaries']!=ident['binaries']):
            raise RuntimeError('source/binary changed: preserve previous result and explicitly version affected run '+entry['id'])
        print('REUSE',entry['id'],previous['status'],flush=True);return previous
    inputs=input_for(entry,registry);method=entry['requested_method'];start=time.monotonic()
    result=dict(run=entry,attempt=attempt,identity=ident,input_digest=physical_digest(inputs),inputs=str(inputs.relative_to(ROOT)),
                status='RUNNING',requested_method=method)
    save(status_path,result)
    print('START',entry['id'],flush=True)
    if method=='SFUISE':
        import run_sfuise_benchmark as sf
        setup=sf.workspace(CACHE)/'devel/setup.bash'
        stat=sf.run_one(setup,inputs,run,playback_rate=1.,fusion_executable='sfuise_sc8_observed',observer=True)
        result.update(status=stat['status'],returncode=stat['fusion_returncode'],details=stat,effective_method='SFUISE_PINNED_OBSERVED')
    elif entry['track']=='T-SIM' and method!='O-NOM':
        config=run/'config.yaml';config.write_text(yaml.safe_dump(sim_config(entry,registry)))
        epochs=entry.get('epochs',registry['cases'][entry['stream']]['epochs'])
        args=[BIN/'epoch_raim_runner',config,run/'trace.csv',entry['stream'],'sc6',
              'off' if method=='O-NOFDE' else 'on',epochs,'tight-optimized',entry['pace'],entry['seed'],entry.get('hook','audit')]
        # Resource watchdog observes the process; no window/source guard changes.
        with (run/'console.log').open('w') as f:
            proc=subprocess.Popen(list(map(str,args)),cwd=ROOT,env=environment(),stdout=f,stderr=subprocess.STDOUT)
            reason=None
            while proc.poll() is None:
                if entry['group']=='E05':
                    if time.monotonic()-start>registry['resource_stops']['scaling_wall_s']:reason='WALL_20_MIN'
                    statusfile=Path(f'/proc/{proc.pid}/status')
                    if statusfile.exists():
                        for line in statusfile.read_text().splitlines():
                            if line.startswith('VmRSS:') and int(line.split()[1])*1024>registry['resource_stops']['rss_bytes']:reason='RSS_LIMIT'
                    trace=run/'trace.csv'
                    if trace.exists():
                        try:
                            rows=list(csv.DictReader(trace.open()))
                            slow=[float(r['core_ms'])>2000 for r in rows[-20:] if r.get('core_ms')]
                            if len(slow)==20 and all(slow):reason='20_CONSECUTIVE_CORE_OVER_2S'
                        except (ValueError,TypeError):pass # partial last write
                    if reason:proc.terminate();break
                time.sleep(.5)
            try:code=proc.wait(timeout=10)
            except subprocess.TimeoutExpired:proc.kill();code=proc.wait()
        result.update(status='RESOURCE_LIMIT' if reason else 'SUCCESS' if code==0 else 'FAIL',returncode=code,
                      resource_reason=reason,command=list(map(str,args)),effective_method=method)
    else:
        config=dataset_config(entry,inputs,run)
        args=[BIN/'nominal_dataset_runner','--config',config,'--input',inputs,'--output',run,'--tag-policy','primary','--causal-bootstrap']
        if method!='O-NOM':args+=['--pipeline-mode','on' if method=='O-FULL' else 'off']
        if entry['pace']=='paced':args+=['--paced']
        code=command(args,run/'console.log',timeout_s=registry['resource_stops'].get('general_solver_wall_s',1200))
        stat=json.loads((run/'run_status.json').read_text()) if (run/'run_status.json').exists() else {}
        result.update(status='RESOURCE_LIMIT' if (run/'resource_limit.json').exists() else 'SUCCESS' if code==0 else 'FAIL',returncode=code,details=stat,command=list(map(str,args)),
                      effective_method='O-NOM' if method=='O-NOM' else 'O-FDE-ENG' if method=='O-FULL' else 'O-ENG-NOFDE')
    result['elapsed_s']=time.monotonic()-start
    result['output_hashes']={p.name:sha(p) for p in run.iterdir() if p.is_file() and p.suffix in ['.csv','.tum','.input','.yaml']}
    save(status_path,result);print('END',entry['id'],result['status'],round(result['elapsed_s'],2),flush=True)
    return result


def run_groups(groups,only=None,retry_failed=False):
    registry=yaml.safe_load(REGISTRY.read_text());groups=groups.split(',')
    for entry in registry['runs']:
        if entry['group'] in groups and (only is None or only in entry['id']):run_one(entry,registry,retry_failed)


def parser():
    p=argparse.ArgumentParser(__doc__);p.add_argument('command',choices=['preflight','register','run'])
    p.add_argument('--build',action='store_true');p.add_argument('--group',default='E00');p.add_argument('--only');p.add_argument('--retry-failed',action='store_true');return p

if __name__=='__main__':
    args=parser().parse_args()
    if args.command=='preflight':preflight(args.build)
    elif args.command=='register':register()
    elif args.command=='run':run_groups(args.group,args.only,args.retry_failed)
