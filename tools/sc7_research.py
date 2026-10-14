#!/usr/bin/env python3
"""SC7 scoped native experiments. Temporary evidence lives only in .cache/sc7."""
import argparse, csv, hashlib, json, subprocess
from pathlib import Path
import numpy as np
import yaml
from sc4_system_research import ROOT, BIN, BUILD, environment
from sc6_closure import score, law, sha
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

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['check','attribute']);p.add_argument('--case',action='append');a=p.parse_args();CACHE.mkdir(parents=True,exist_ok=True)
    if a.command=='check':
        if execute([BIN/'sc7_checks'],CACHE/'checks.log'):raise SystemExit(1)
    else:attribute(a.case)
if __name__=='__main__':main()
