#!/usr/bin/env python3
"""Metadata-only external schedules for the SC9 common-core simulator."""
import argparse,csv,hashlib,json,math,subprocess
from pathlib import Path
from sc4_system_research import ROOT,BIN,environment

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def matrix(kind,run_id):
    if not run_id or not all(c.isalnum() or c in '-_' for c in run_id):raise ValueError('simple run ID required')
    destination=ROOT/'.cache/sc9/simulation'/run_id;destination.mkdir(parents=True,exist_ok=True)
    schedule=ROOT/'.cache/sc9/simulation-B2-schedule.json';binary=BIN/'sc9_simulation_runner'
    cases=[('B2-'+f,400,f,30,30,True) for f in ['healthy','U','A','G','J']]+[('B3-J',400,'J',30,30,False)]
    cases += [('B2-window-'+str(lag)+'-'+str(window),160,'J',lag,window,True) for lag,window in [(20,20),(40,40),(30,10)]]
    if kind=='long':cases=[('L-healthy',5000,'healthy',30,30,True),('L-repeated',10000,'repeated',30,30,True)]
    sources={str(p.relative_to(ROOT)):sha(p) for folder in ['research','include','src','config'] for p in sorted((ROOT/folder).rglob('*')) if p.is_file() and p.suffix in ['.cpp','.hpp','.h','.yaml']}
    sources['tools/sc9_simulation.py']=sha(__file__);sources['CMakeLists.txt']=sha(ROOT/'CMakeLists.txt')
    frozen=dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),sources=sources,
                elf=sha(binary),dso=sha(BIN.parent/'libuwb_imu_pl.so'),schedule=sha(schedule),cases=cases,
                registration=sha(ROOT/'.cache/sc9/generalized-simulation-registration.json'),
                timing='unpaced core and inclusive provider+core; not an actual sink-latency award',hardware_qualification=False)
    frozen=json.loads(json.dumps(frozen));freeze=destination/'freeze.json'
    if freeze.exists():
        if json.loads(freeze.read_text())!=frozen:raise RuntimeError('frozen identity changed; use a new run ID')
    else:freeze.write_text(json.dumps(frozen,indent=2)+'\n')
    if any(x.strip() in ['cc1plus','cc1','clang','clang++'] for x in subprocess.check_output(['ps','-eo','comm'],text=True).splitlines()):raise RuntimeError('compiler active')
    results={}
    def dist(xs):
        x=sorted(xs);return dict(n=len(x),p50=x[math.ceil(.5*len(x))-1],p95=x[math.ceil(.95*len(x))-1],p99=x[math.ceil(.99*len(x))-1],max=max(x)) if x else None
    for name,count,fault,lag,window,external in cases:
        output=destination/(name+'.csv');receipt=destination/(name+'.json')
        if receipt.exists():
            r=json.loads(receipt.read_text())
            if not r['pass'] or r['output_sha']!=sha(output):raise RuntimeError('retained failure or changed output: '+name)
            results[name]=r;continue
        command=[str(binary),str(count),fault,str(output),str(lag),str(window)]+([str(schedule)] if external else [])
        with (destination/(name+'.log')).open('w') as log:
            code=subprocess.run(['/usr/bin/time','-v','-o',str(destination/(name+'.resources'))]+command,cwd=ROOT,env=environment(),stdout=log,stderr=subprocess.STDOUT).returncode
        rows=list(csv.DictReader(output.open())) if output.exists() else []
        sums={k:sum(int(r[k]) for r in rows) for k in ['contained','latent_event','imu_alarm','uwb_alarm','bridge','repair','rejoined']}
        exclusions=sum(bool(r['excluded_source']) for r in rows)
        continuity=all(int(r['epoch'])==i+1 and int(r['generation'])==i+1 and r['updates']=='1' and float(r['risk'])<=1e-6 for i,r in enumerate(rows))
        continuity &= all(int(b['prefix'])>int(a['prefix']) and float(b['risk'])>=float(a['risk']) for a,b in zip(rows,rows[1:]))
        recovery=[int(r['epoch']) for r in rows if r['rejoined']=='1']
        functional=fault=='healthy' or (exclusions>0 if fault=='U' else bool(recovery and recovery[-1]<=count-4 and sums['bridge']>0))
        if name in ['B2-J','B3-J','L-repeated']:functional &= sums['repair']>0 and exclusions>0
        passed=code==0 and len(rows)==count and sums['contained']==count and sums['latent_event']==count and continuity and functional
        r=dict(command=command,returncode=code,planned=count,received=len(rows),output_sha=sha(output) if output.exists() else None,
               counts=sums,exclusions=exclusions,rejoin_epochs=recovery,continuity=continuity,functional=functional,
               core_ms=dist([float(x['core_ms']) for x in rows]),provider_plus_core_ms=dist([float(x['provider_plus_core_ms']) for x in rows]),
               max_resources={k:max((int(x[k]) for x in rows),default=0) for k in ['rss','slots','ledger','replay','capsules','prefix']},
               timed_spatial_rows=sum(float(x['provider_plus_core_ms'])<=min(50,float(x['period_ms'])) and float(x['core_ms'])<=40 and float(x['hpl'])<=2 and float(x['vpl'])<=3 for x in rows),
               h_exceedances=sum(float(x['herror'])>float(x['hpl']) for x in rows),v_exceedances=sum(float(x['verror'])>float(x['vpl']) for x in rows),
               misses=[int(x['epoch']) for x in rows if float(x['core_ms'])>40 or float(x['provider_plus_core_ms'])>min(50,float(x['period_ms']))])
        r['pass']=passed;receipt.write_text(json.dumps(r,indent=2)+'\n');results[name]=r
        print(name,'pass',passed,'rows',len(rows),'actions',sums,'exclusions',exclusions,flush=True)
        (destination/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
        if not passed:raise RuntimeError('retained failed matrix case: '+name)
        if sha(binary)!=frozen['elf'] or sha(schedule)!=frozen['schedule']:raise RuntimeError('runtime identity changed')

def prepare(wire,output,count):
    wire=Path(wire);output=Path(output)
    if output.exists():raise RuntimeError('retain existing schedule; choose a new path')
    events=[json.loads(line) for line in wire.read_text().splitlines()]
    imu=[e['stamp_ns'] for e in events if e['kind']=='imu']
    raw=[e for e in events if e['kind']=='range']
    anchors=sorted({r['anchor_id'] for e in raw for r in e['ranges']})
    start=imu[0];delta=sorted(b-a for a,b in zip(imu,imu[1:]))
    assert min(delta)>0
    period=max(imu[-1],max(r['stamp_ns'] for e in raw for r in e['ranges']))-start+delta[len(delta)//2]
    loops=math.ceil((count+20)/len(raw))
    result=dict(schema='sc9-exogenous-schedule-v1',source_sha=hashlib.sha256(wire.read_bytes()).hexdigest(),
                scope='original per-channel integer times and validity only; synthetic values and geometry; repeated recordings use declared period seam',
                source_anchor_ids=anchors,repeat_period_ns=period,loops=loops,imu_ns=[],batches=[])
    for cycle in range(loops):
        shift=cycle*period-start
        result['imu_ns'].extend(t+shift for t in imu)
        for e in raw:
            rows=[dict(anchor=anchors.index(r['anchor_id']),stamp_ns=r['stamp_ns']+shift,valid=r['valid']) for r in e['ranges']]
            result['batches'].append(dict(end_ns=max(r['stamp_ns'] for r in rows),ranges=rows))
    assert min(result['imu_ns'])>=0 and all(b>a for a,b in zip(result['imu_ns'],result['imu_ns'][1:]))
    assert max(b-a for a,b in zip(result['imu_ns'],result['imu_ns'][1:]))<=50000000
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,separators=(',',':'))+'\n')
    print(output,'IMU',len(result['imu_ns']),'batches',len(result['batches']),'sources',len(anchors),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('wire',nargs='?');p.add_argument('output',nargs='?');p.add_argument('--epochs',type=int,default=400)
    p.add_argument('--matrix',choices=['short','long']);p.add_argument('--run-id')
    a=p.parse_args()
    if a.matrix:matrix(a.matrix,a.run_id)
    else:prepare(a.wire,a.output,a.epochs)
