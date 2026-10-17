#!/usr/bin/env python3
"""Metadata-only external schedules for the SC9 common-core simulator."""
import argparse,csv,hashlib,json,math,subprocess
from pathlib import Path
from sc4_system_research import ROOT,BIN,environment

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def matrix(kind,run_id,geometry='original'):
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
                registration=sha(ROOT/'.cache/sc9'/('generalized-simulation-registration.json' if geometry=='original' else 'nondegenerate5-simulation-registration.json')),
                geometry=geometry,
                timing=('independent absolute virtual-sensor releases; counted seeded decoding/provider/core; actual result file sink before privileged scoring' if kind=='paced' else 'unpaced core and inclusive provider+core; not an actual sink-latency award'),hardware_qualification=False)
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
            if kind=='paced' and any(r['delivery'][key]!=sha(str(output)+suffix) for key,suffix in [('sink_sha','.sink.csv'),('timing_sha','.delivery.csv')]):raise RuntimeError('changed paced artifacts: '+name)
            results[name]=r;continue
        command=[str(binary),str(count),fault,str(output),str(lag),str(window)]+([str(schedule)] if external else [])
        if external and (geometry!='original' or kind=='paced'):command.append(geometry)
        if kind=='paced':
            if not external:command.extend(['-','original'])
            command.append('paced')
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
        if kind=='paced':
            delivery_path=Path(str(output)+'.delivery.csv');sink_path=Path(str(output)+'.sink.csv')
            clocks=list(csv.DictReader(delivery_path.open())) if delivery_path.exists() else []
            sink=list(csv.DictReader(sink_path.open())) if sink_path.exists() else []
            parity=len(clocks)==len(sink)==len(rows)==count
            offsets=set()
            for expected,actual,timing in zip(rows,sink,clocks):
                parity &= all(expected[k]==actual[k] for k in ['epoch','generation','hpl','vpl','source_digest','receipt_digest','boundary_digest']) and expected['bound_status']==actual['status'] and expected['epoch']==timing['epoch']
                parity &= expected['stamp_ns']==timing['source_batch_end_ns'] and int(timing['measurement_target_ns'])<=int(timing['release_target_ns'])<=int(timing['release_actual_ns'])<=int(timing['decode_start_ns'])<=int(timing['sink_ns'])
                offset=int(timing['release_target_ns'])-int(timing['source_batch_end_ns']);offsets.add(offset)
                parity &= int(timing['measurement_target_ns'])==int(timing['measurement_ns'])+offset
                parity &= abs((int(timing['sink_ns'])-int(timing['measurement_target_ns']))/1e6-float(timing['measurement_to_sink_ms']))<1e-9
            parity &= len(offsets)==1
            d=dict(rows=len(clocks),same_receipt_and_clock_order=parity,
                measurement_to_sink_ms=dist([float(x['measurement_to_sink_ms']) for x in clocks]),
                release_lateness_ms=dist([float(x['release_lateness_ms']) for x in clocks]),queue_ms=dist([float(x['queue_ms']) for x in clocks]),
                max_queue=max((int(x['queue_high']) for x in clocks),default=0),
                timely=sum(int(x['timely']) for x in clocks),timely_spatial=sum(int(x['timely_spatial']) for x in clocks),
                misses=[int(x['epoch']) for x in clocks if x['timely']!='1'],
                sink_sha=sha(sink_path) if sink_path.exists() else None,timing_sha=sha(delivery_path) if delivery_path.exists() else None)
            d['realtime_pass']=bool(parity and r['core_ms']['p95']<=40 and d['measurement_to_sink_ms']['p95']<=50 and d['timely']/count>=.95 and d['timely_spatial']/count>=.95)
            r['delivery']=d;passed &= parity
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
    p.add_argument('--matrix',choices=['short','long','paced']);p.add_argument('--run-id')
    p.add_argument('--geometry',choices=['original','nondegenerate5'],default='original')
    a=p.parse_args()
    if a.matrix:matrix(a.matrix,a.run_id,a.geometry)
    else:prepare(a.wire,a.output,a.epochs)
