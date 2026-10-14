#!/usr/bin/env python3
"""SC7 scoped native experiments. Temporary evidence lives only in .cache/sc7."""
import argparse, csv, hashlib, json, subprocess, re
from pathlib import Path
import numpy as np
import yaml
from sc4_system_research import ROOT, BIN, BUILD, environment
from sc6_closure import score, law, sha, verified_rows, committed, value, stats
CACHE = ROOT / '.cache/sc7'
SEEDS = [20261010, 20261011]

def execute(command, log):
    with log.open('w') as f:
        r = subprocess.run([str(x) for x in command], cwd=ROOT, env=environment(), stdout=f, stderr=subprocess.STDOUT)
    print(log.name, 'returncode', r.returncode, log.read_text()[-1000:], flush=True)
    return r.returncode

def attribute(only=None):
    summaries = json.loads((CACHE/'attribution-summary.json').read_text()) if (CACHE/'attribution-summary.json').exists() else {}
    for case, motion, geometry, common in [('A00','M0','G0',True),('A01','M0','G1',True),('A30','M3','G0',True),('A31','M3','G1',True),('legacy','M0','G0',False),('S1','M1','G1',False),('S2','M2','G1',False)]:
        if only and case not in only: continue
        config = {'raw_base_law':'config/sc7.yaml','closure':{'additional_cases':{case:{'epochs':300,'motion':motion,'geometry':geometry,'faults':[]}}},'sc7':{'version':'sc7-r1','attribute':True}}
        inherited=law()['closure']; inherited['additional_cases']={**inherited.get('additional_cases',{}),**config['closure']['additional_cases']};config['closure']=inherited
        if case in ['S1','S2']: config['closure']['additional_cases'][case]['faults'] = law()['sc6']['cases'][case]['faults']
        if common: config['sc7']['common_domain']='D1'
        path=CACHE/(case+'.yaml');path.write_text(yaml.safe_dump(config));out=CACHE/(case+'-attribute.csv')
        code=execute([BIN/'sc7_checks','attribute',path,case,300,SEEDS[0],out],CACHE/(case+'-attribute.log'))
        if code: raise RuntimeError(case+' attribution failed')
        rows=list(csv.DictReader(out.open())); summaries[case]={'rows':len(rows),'config_sha256':sha(path),'snapshots':{}}
        for epoch in sorted({int(r['output_epoch']) for r in rows}):
            group=[r for r in rows if int(r['output_epoch'])==epoch and int(r['state_epoch'])==epoch]
            stages={}
            for r in group:
                stages[r['stage']]={k:float(r[k]) for k in ['hpl','vpl','z_half','z_offset','low_z','high_z']}
            last_hull=max(i for i,r in enumerate(group) if r['stage']=='range_hull')
            leaves=[]
            for r in reversed(group[:last_hull]):
                if r['stage'] not in ['range_leaf','range_frontier']: break
                leaves.append(r)
            if leaves:
                stages['union_before_hull']={k:max(float(r[k]) for r in leaves) for k in ['hpl','vpl']}
                stages['extrema_sources']={key:{'mask':int((min if key.startswith('low') else max)(leaves,key=lambda r:float(r[key]))['mask']),'class':'OUTER_ENDPOINT_NO_FEASIBILITY_WITNESS'} for key in ['low_x','high_x','low_y','high_y','low_z','high_z']}
                assert abs(stages['union_before_hull']['vpl']-stages['range_hull']['vpl'])<1e-12
            summaries[case]['snapshots'][epoch]=stages
        print(case,json.dumps(summaries[case]['snapshots']),flush=True)
    (CACHE/'attribution-summary.json').write_text(json.dumps(summaries,indent=2)+'\n')

def independent_geometry():
    from scipy.optimize import least_squares
    path=CACHE/'w1-S3-20261010-30-30-on-paced.csv.input'
    rows=[l.split() for l in path.read_text().splitlines() if l.startswith('uwb 1 ')]
    anchors=np.array([[float.fromhex(x) for x in r[10:13]] for r in rows]);ranges=np.array([float.fromhex(r[13]) for r in rows]);out=[]
    native_stats={int(m):float(v) for m,v in re.findall(r'G2 mask=(-?\d+).*? stat=([\d.e+-]+)',(CACHE/'geometry-checks.log').read_text())}
    for mask in [-1,*range(8)]:
        idx=[i for i in range(8) if i!=mask];a,y=anchors[idx],ranges[idx];solutions=[]
        for z in [-2.,-1.,1.,2.]:
            f=least_squares(lambda p:(np.linalg.norm(p-a,axis=1)-y)/.1,[0,0,z],max_nfev=200,xtol=1e-12,ftol=1e-12,gtol=1e-12)
            solutions.append(dict(success=bool(f.success),objective=float(2*f.cost),root=f.x.tolist()))
        assert all(s['success'] for s in solutions)
        assert abs(min(s['objective'] for s in solutions)-native_stats[mask])<1e-4
        out.append(dict(mask=mask,solutions=solutions))
    (CACHE/'g2-independent.json').write_text(json.dumps(dict(input_sha=sha(path),masks=out),indent=2)+'\n')
    print('independent SciPy all9 WLS objective agreement and36/36 diagnostic convergence',flush=True)

def source_identity():
    paths=subprocess.check_output(['git','ls-files','research','src','include','config','tools','CMakeLists.txt'],cwd=ROOT,text=True).splitlines()
    paths+=['research/numerical_candidate.cpp','research/numerical_candidate.hpp']
    digest=hashlib.sha256()
    for p in sorted(set(paths)):
        if (ROOT/p).is_file():digest.update(p.encode()+b'\0'+(ROOT/p).read_bytes())
    return dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),sources_sha256=digest.hexdigest(),elf=sha(BIN/'epoch_raim_runner'),dso=sha(BIN.parent/'libuwb_imu_pl.so'))

def native(case, seed=SEEDS[0], count=0, mode='offline', fde='on', variant='new', window='30/30', hook='audit'):
    name=f'{variant}-{case}-{seed}-{window.replace("/","-")}-{fde}-{mode}'
    config={'raw_base_law':'config/sc6_closure.yaml' if variant=='old' else 'config/sc7.yaml'}
    lf,li=map(int,window.split('/'));config['fixed_lag_epochs']=lf
    config['rolling_integrity']=dict(law()['rolling_integrity'],window_mode='follow_fgo' if lf==li else 'explicit',explicit_epochs=None if lf==li else li)
    path=CACHE/(name+'.yaml');path.write_text(yaml.safe_dump(config));trace=CACHE/(name+'.csv');log=CACHE/(name+'.log');ident=source_identity()
    command=[BIN/'epoch_raim_runner',path,trace,case,'sc6',fde,count,'tight-optimized',mode,seed,hook]
    code=execute(command,log);rows=list(csv.DictReader(trace.open())) if trace.exists() else [];resolved=law(path);manifest=resolved['sc6']['cases'].get(case) or resolved['closure']['additional_cases'][case];scheduled=count or manifest['epochs']
    verified_rows(rows,resolved['lever'])
    summary=dict(identity=ident,command=[str(x) for x in command],returncode=code,manifest=manifest,input_sha=sha(str(trace)+'.input') if hook=='audit' else None,trace_sha=sha(trace),**score(rows,manifest,scheduled))
    good=[r for r in rows if committed(r)]
    summary['bias_axes']={str(j):stats([value(r,'bwidth_'+str(j)) for r in good]) for j in range(6)}
    summary['x0_retired']=any(r.get('x0')=='0' for r in good)
    (CACHE/(name+'.json')).write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(name,summary['qualified'],summary['timely'],'/',scheduled,'P95',summary['timing']['core_ms']['p95'],summary['timing']['delivery_ms']['p95'],flush=True)
    return summary

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['check','attribute','geometry','run']);p.add_argument('--case',action='append');p.add_argument('--seed',type=int,default=SEEDS[0]);p.add_argument('--epochs',type=int,default=0);p.add_argument('--fde',default='on');p.add_argument('--variant',default='new');p.add_argument('--mode',default='offline');p.add_argument('--window',default='30/30');a=p.parse_args();CACHE.mkdir(parents=True,exist_ok=True)
    if a.command=='check':
        if execute([BIN/'sc7_checks'],CACHE/'checks.log'):raise SystemExit(1)
    elif a.command=='attribute':attribute(a.case)
    elif a.command=='geometry':
        if execute([BIN/'sc7_checks','geometry'],CACHE/'geometry-checks.log'):raise SystemExit(1)
        for seed in SEEDS:native('S3',seed,300,'paced',variant='w1')
        independent_geometry()
    else:
        for case in a.case or ['S0']:native(case,a.seed,a.epochs,a.mode,a.fde,a.variant,a.window)
if __name__=='__main__':main()
