#!/usr/bin/env python3
"""SC6 shared native simulation runner and independent post-delivery scorer."""
import argparse,csv,hashlib,json,subprocess,sys
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from sc4_system_research import ROOT,BIN,BUILD,environment
CACHE=ROOT/'.cache/sc6-20261014'
CONFIG=ROOT/'config/sc6.yaml'

def identity():
    return dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                elf=hashlib.sha256((BIN/'epoch_raim_runner').read_bytes()).hexdigest(),
                dso=hashlib.sha256((BIN.parent/'libuwb_imu_pl.so').read_bytes()).hexdigest(),
                config=hashlib.sha256(CONFIG.read_bytes()).hexdigest())
def stats(values):
    v=np.array(values,dtype=float)
    return {} if not len(v) else dict(mean=float(v.mean()),median=float(np.median(v)),p95=float(np.percentile(v,95)),max=float(v.max()),first=float(v[0]),last=float(v[-1]))
def score(rows,manifest):
    good=[r for r in rows if r['updates']=='1']
    n=len(rows);result=dict(scheduled=n,native=len(good),finite=0,qualified=0,timely=0,abort=sum(r['status']=='ABORT' for r in rows),unattempted=sum(r['status']=='SOURCE_GAP_OR_ABORT' for r in rows),refused=n-len(good))
    if not good:return result
    assert all(None not in r and None not in r.values() for r in good),'malformed complete row'
    result.update(finite=sum(bool(np.isfinite(float(r['ape']))) for r in good),qualified=sum(r['qualified']=='1' for r in good),timely=sum(r['timely']=='1' for r in good),core40_miss=sum(float(r['core_ms'])>40 for r in good),delivery50_miss=sum(float(r['delivery_ms'])>50 for r in good),h_exceed=sum(float(r['hpe'])>float(r['hpl']) for r in good),v_exceed=sum(float(r['vpe'])>float(r['vpl']) for r in good))
    result['metrics']={k:stats([float(r[k]) for r in good]) for k in ['ape','hpe','vpe','hpl','vpl','velocity_error','rotation_error','accel_bias_error','gyro_bias_error','source_ms','prepare_ms','candidate_ms','proof_ms','commit_ms','cleanup_ms','core_ms','delivery_ms','slots','live','ledger','rss','bias_width','alerts','replay','capsules','q','risk']}
    result['rmse']={k:float(np.sqrt(np.mean([float(r[k])**2 for r in good]))) for k in ['ape','hpe','vpe','velocity_error','rotation_error','accel_bias_error','gyro_bias_error']}
    # Online body RPE. Only immutable terminal outputs are used; no alignment.
    by_epoch={int(r['epoch']):r for r in good};rpe={}
    p=lambda r,prefix:np.array([float(r[prefix+x]) for x in ['x','y','z']])
    q=lambda r,prefix:Rotation.from_quat([float(r[prefix+x]) for x in ['x','y','z','w']])
    for delta in [1,20]:
        trans=[];angles=[]
        for k,a in by_epoch.items():
            if k+delta not in by_epoch:continue
            b=by_epoch[k+delta];ra=q(a,'q');rb=q(b,'q');ga=q(a,'gt_q');gb=q(b,'gt_q')
            rt=ga.inv()*gb;rr=ra.inv()*rb
            pt=ga.inv().apply(p(b,'gt_p')-p(a,'gt_p'));pr=ra.inv().apply(p(b,'p')-p(a,'p'))
            trans.append(np.linalg.norm(rt.inv().apply(pr-pt)));angles.append((rt.inv()*rr).magnitude())
        rpe[str(delta)]=dict(planned=max(0,n-delta),valid=len(trans),translation=stats(trans),rotation=stats(angles))
    result['rpe']=rpe
    episodes=[]
    for onset,axis,duration,amplitude,uwb_onset in manifest['faults']:
        start=(onset-1)*50+5;end=start+duration
        detected=[r for r in good if int(r['epoch'])>=onset and r['bridge']=='1' and int(r['epoch'])*50<end+1000]
        repair=[r for r in detected if r['repair']=='1']
        rejoined=[int(r['epoch']) for r in good if int(r['epoch'])*50>=end and r['rejoined']=='1' and int(r['epoch'])<onset+50]
        isolated=[int(r['epoch']) for r in good if uwb_onset<=int(r['epoch'])<uwb_onset+3 and r['excluded']=='0']
        episodes.append(dict(onset=onset,axis=axis,duration=duration,amplitude=amplitude,first_bridge=int(detected[0]['epoch']) if detected else None,prior_repairs=[int(r['epoch']) for r in repair],rejoin4=rejoined[:4],uwb_onset=uwb_onset,correct_uwb_isolations=isolated))
    result['episodes']=episodes
    result['healthy_extra_exclusions']=sum(int(r['excluded'])>=0 and not any(f[4]<=int(r['epoch'])<f[4]+3 for f in manifest['faults']) for r in good)
    result['stage_rmse']={}
    for stage,predicate in [('normal',lambda k:not any(f[0]<=k<f[0]+10 for f in manifest['faults'])),('fault',lambda k:any(f[0]<=k<f[0]+10 for f in manifest['faults'])),('repair_after',lambda k:any(f[0]+10<=k<f[0]+50 for f in manifest['faults']))]:
        subset=[r for r in good if predicate(int(r['epoch']))]
        result['stage_rmse'][stage]=dict(n=len(subset),**{key:float(np.sqrt(np.mean([float(r[key])**2 for r in subset]))) if subset else None for key in ['ape','velocity_error','rotation_error','accel_bias_error','gyro_bias_error']})
    # Diagnostic AL is evaluated entirely after immutable native results.
    result['al']={f'{h}/{v}':sum(float(r['hpl'])<=h and float(r['vpl'])<=v for r in good) for h,v in [(2,3),(5,5),(10,10)]}
    assert all(r['formal_eligible']==r['publication_protected']==r['application_available']=='0' for r in good)
    previous=0
    for r in good:
        k=int(r['epoch']);assert int(r['source_prefix'])==21+128*k and int(r['version'])==k+1
        assert int(r['generation'])==k and int(r['parent'])==k-1
        assert previous<float(r['risk'])<=1e-6;previous=float(r['risk'])
    return result

def run(case,profile='tight-optimized',fde='on',mode='offline',count=0,seed=20261010,window=None,label=None):
    CACHE.mkdir(parents=True,exist_ok=True);label=label or f'{case}-{profile}-{fde}-{mode}-{seed}'+(f'-{window}' if window else '')
    csv_path=CACHE/(label+'.csv');summary_path=CACHE/(label+'.json');log=CACHE/(label+'.log')
    if summary_path.exists():
        old=json.loads(summary_path.read_text())
        if old['identity']==identity():print('REUSE',label,flush=True);return old
        raise RuntimeError('existing result has different identity: '+label)
    config=CONFIG
    if window:
        lf,li=map(int,window.split('/'));v=yaml.safe_load(CONFIG.read_text());base=yaml.safe_load((ROOT/'config/sc5_open.yaml').read_text());v['fixed_lag_epochs']=lf;v['rolling_integrity']=dict(base['rolling_integrity'],window_mode='follow_fgo' if lf==li else 'explicit',explicit_epochs=None if lf==li else li);config=CACHE/(label+'.yaml');config.write_text(yaml.safe_dump(v))
    command=[str(BIN/'epoch_raim_runner'),str(config),str(csv_path),case,'sc6',fde,str(count),profile,mode,str(seed)]
    ident=identity()
    with log.open('w') as sink:r=subprocess.run(command,cwd=ROOT,env=environment(),stdout=sink,stderr=subprocess.STDOUT)
    rows=list(csv.DictReader(csv_path.open())) if csv_path.exists() else []
    manifest=yaml.safe_load(CONFIG.read_text())['sc6']['cases'][case]
    summary=dict(identity=ident,command=command,case=case,profile=profile,fde=fde,mode=mode,seed=seed,window=window or '30/30',returncode=r.returncode,**score(rows,manifest))
    summary['planned']=count or manifest['epochs'];summary['never_emitted']=summary['planned']-len(rows)
    summary_path.write_text(json.dumps(summary,indent=2));print('SC6 RESULT',label,json.dumps({k:v for k,v in summary.items() if k not in ['metrics','command','identity','rpe','stage_rmse']}),flush=True)
    if r.returncode:print(log.read_text()[-2000:],flush=True)
    return summary

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['run','check','evaluate']);p.add_argument('--case',default='S0');p.add_argument('--profile',default='tight-optimized');p.add_argument('--fde',default='on');p.add_argument('--mode',default='offline');p.add_argument('--epochs',type=int,default=0);p.add_argument('--seed',type=int,default=20261010);p.add_argument('--window');p.add_argument('--label');p.add_argument('--build',action='store_true');a=p.parse_args();CACHE.mkdir(parents=True,exist_ok=True)
    if a.build:subprocess.run(['cmake','--build',str(BUILD),'--target','epoch_raim_runner','open_rolling_checks','-j2'],cwd=ROOT,env=environment(),check=True)
    if a.command=='run':run(a.case,a.profile,a.fde,a.mode,a.epochs,a.seed,a.window,a.label)
    elif a.command=='check':subprocess.run([str(BIN/'open_rolling_checks')],cwd=ROOT,env=environment(),check=True)
    else:
        for path in sorted(CACHE.glob('*.json')):
            data=json.loads(path.read_text())
            if 'rmse' in data:print(path.stem,data['native'],data['planned'],data['rmse'],data['core40_miss'],data['delivery50_miss'])
if __name__=='__main__':main()
