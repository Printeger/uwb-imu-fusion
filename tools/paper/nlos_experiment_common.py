"""Experiment orchestration primitives; no truth is passed to child estimators."""
import json,os,signal,subprocess,time
from pathlib import Path
import yaml
from nlos_injection import write_json,sha,rows,METHODS
ROOT=Path(__file__).resolve().parents[2]
BIN=Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner')

def execute(command,log,limit=1800,env=None):
    log=Path(log);log.parent.mkdir(parents=True,exist_ok=True);start=time.monotonic()
    record=dict(command=list(map(str,command)),cwd=str(Path.cwd()),wall_limit_s=limit,status='RUNNING',log=str(log))
    write_json(str(log)+'.command.json',record)
    with log.open('w') as f:
        p=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=env)
        try:code=p.wait(timeout=limit)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait();code=124
    record.update(exit_code=code,wall_s=time.monotonic()-start,status='TIMEOUT' if code==124 else 'EXITED');write_json(str(log)+'.command.json',record);return code

def config_for(dataset,base,cache,path,subset=None):
    cfg=yaml.safe_load((ROOT/f'config/paper/ie0911/{dataset}.yaml').read_text());cfg.pop('sfuise',None)
    cfg['dataset']={'interface':'t07_cache','cache_manifest':str(Path(cache).resolve()),'cache_start_s':0.,'cache_duration_s':-1.}
    anchors=yaml.safe_load((Path(base)/'anchors.yaml').read_text())
    cfg['anchors']=[dict(id=a['id'],pos=list(map(float,a['pos'])),prior_sigma=float(a['prior_sigma'])) for a in anchors]
    cfg['nlos']['final_inference_enabled']=False
    Path(path).parent.mkdir(parents=True,exist_ok=True);Path(path).write_text(yaml.safe_dump(cfg,sort_keys=True));return cfg

def screen(config,out,name,subset=None,extra=None):
    cmd=[str(BIN),'--config',str(config),'--output-root',str(out),'--run-id',name,'--stop-after-fde','--execution-type','CACHE_PRODUCER','--method','structured_debias']
    if subset:cmd+=['--anchor-ids',','.join(map(str,subset))]
    return execute((extra or [])+cmd,Path(out)/(name+'.log'))

def manifest_unit(dataset,unit,config,subset=None,modes=METHODS,producer=True):
    cells=[]
    for mode in modes:
        if mode in ('all_range','robust_cauchy'):
            c=dict(mode=mode,execution_type='BASELINE_TRAJECTORY',path='DIRECT_COMMON_PREPARATION')
            if mode=='robust_cauchy':c['robust_scale']=2.3849
            cells.append(c)
    if producer:
        cells.append(dict(mode='structured_debias',execution_type='CACHE_PRODUCER',path='AUTO_DISCOVERY',producer_id='shared'))
        for mode in modes:
            if mode not in ('all_range','robust_cauchy'):cells.append(dict(mode=mode,execution_type='FINAL_TRAJECTORY',path='AUTO_DISCOVERY',producer_id='shared',thresholds={}))
    result=dict(run_unit_id=unit,recording_id=dataset,base_trajectory_id=dataset,seed=911,prefix_identity='full_original_crop',config=str(config),cells=cells)
    if subset:result['anchor_ids']=subset
    return result

def batch(unit,path,out):
    doc=dict(schema='uifgo_t09_batch_v2',role='development',parameter_provenance='IE0911_DEVELOPMENT_TEST_ONLY_PENDING_VALIDATION',run_units=[unit]);Path(path).write_text(yaml.safe_dump(doc,sort_keys=False))
    env=os.environ.copy();env['UIFGO_EXPERIMENT_WALL_LIMIT_S']='1800'
    return execute(['python3',str(ROOT/'tools/paper/run_experiments.py'),'--manifest',str(path),'--runner',str(BIN),'--output-root',str(out)],str(out)+'.log',limit=1800*8+120,env=env)

def read_json(p):return json.loads(Path(p).read_text()) if Path(p).is_file() else {}

def seal(directory):
    directory=Path(directory);items={str(p.relative_to(directory)):sha(p.read_bytes()) for p in sorted(directory.rglob('*')) if p.is_file() and p.name!='sealed_hashes.json'}
    write_json(directory/'sealed_hashes.json',items);return sha(json.dumps(items,sort_keys=True).encode())

def same_initialization(clean,injected):
    a=rows(Path(clean)/'observations.csv');b=rows(Path(injected)/'observations.csv')
    cols=('obs_id','source_message','source_range','source_observation','raw_time','raw_tag_id','anchor_id','valid','planned','keyframe_id','nominal_sigma_m')
    plan=[[r[c] for c in cols] for r in a]==[[r[c] for c in cols] for r in b]
    x=read_json(Path(clean)/'common_preparation.json');y=read_json(Path(injected)/'common_preparation.json')
    return dict(plan_and_noise_equal=plan,pre_raw_lm_values_equal=bool(x.get('values_sha256')) and x.get('values_sha256')==y.get('values_sha256'),clean_values_sha256=x.get('values_sha256'),injected_values_sha256=y.get('values_sha256'))
