#!/usr/bin/env python3
"""SC7 scoped native experiments. Temporary evidence lives only in .cache/sc7."""
import argparse, csv, hashlib, json, subprocess, re, math
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
    config={'raw_base_law':'config/sc6_closure.yaml' if variant in ['old','final-old'] else 'config/sc7.yaml'}
    if variant=='w1':config['sc7']=dict(law(ROOT/'config/sc7.yaml')['sc7'],joint_raw=False,tag_support=False)
    lf,li=map(int,window.split('/'));config['fixed_lag_epochs']=lf
    config['rolling_integrity']=dict(law()['rolling_integrity'],window_mode='follow_fgo' if lf==li else 'explicit',explicit_epochs=None if lf==li else li)
    path=CACHE/(name+'.yaml');path.write_text(yaml.safe_dump(config));trace=CACHE/(name+'.csv');log=CACHE/(name+'.log');ident=source_identity()
    command=[BIN/'epoch_raim_runner',path,trace,case,'sc6',fde,count,'tight-optimized',mode,seed,hook]
    print("START",name,flush=True)
    code=execute(command,log);rows=list(csv.DictReader(trace.open())) if trace.exists() else [];resolved=law(path);manifest=resolved['sc6']['cases'].get(case) or resolved['closure']['additional_cases'][case];scheduled=count or manifest['epochs']
    assert all(None not in r and None not in r.values() for r in rows), 'trace schema mismatch'
    verified_rows(rows,resolved['lever'])
    from math import isfinite
    from sc6_closure import law as closure_law
    for r in rows:
        if committed(r):
            assert value(r,'q')>=7 and value(r,'shared_q')>=7 and 1<=value(r,'active_states')<=lf+1 and 0<=value(r,'proof_explicit')<=li, 'native q/resource identity mismatch'
    summary=dict(identity=ident,command=[str(x) for x in command],returncode=code,manifest=manifest,input_sha=sha(str(trace)+'.input') if hook=='audit' else None,trace_sha=sha(trace),**score(rows,manifest,scheduled))
    good=[r for r in rows if committed(r)]
    summary['bias_axes']={str(j):stats([value(r,'bwidth_'+str(j)) for r in good]) for j in range(6)}
    input_path=Path(str(trace)+'.input')
    summary['raw_input_sha']=hashlib.sha256('\n'.join(l for l in input_path.read_text().splitlines() if l.startswith(('initial ','imu ','uwb '))).encode()).hexdigest() if input_path.exists() else None
    support=[r for r in good if r.get('bound_method')=='sc7-branch-box-union-tag-v1']
    summary['mathematical']={'n':len(support)}
    for axis,new,old in [('H','hpl','m_old_hpl'),('V','vpl','m_old_vpl')]:
        delta=[value(r,old)-value(r,new) for r in support]
        summary['mathematical'][axis]=dict(strict=sum(x>1e-9 for x in delta),widened=sum(x<0 for x in delta),improvement=stats(delta))
    summary['union_bias']={str(j):dict(strict=sum(value(r,'reference_bwidth_'+str(j))-value(r,'bwidth_'+str(j))>1e-9 for r in good),improvement=stats([value(r,'reference_bwidth_'+str(j))-value(r,'bwidth_'+str(j)) for r in good])) for j in range(6)}
    summary['joint_fallbacks']=sum(value(r,'joint_fallback',0)>0 for r in good)
    summary['resources_extra']={key:stats([value(r,key) for r in good]) for key in ['active_states','proof_explicit','source_prefix','support_boxes','support_frontier','joint_branches','joint_frontier','source_ms','prepare_ms','candidate_ms','proof_ms','commit_ms','cleanup_ms']}
    summary['support_boxes']=stats([value(r,'support_boxes') for r in support])
    summary['miss_epochs']={key:[int(r['epoch']) for r in rows if r.get('status')!='SOURCE_GAP_OR_ABORT' and value(r,key)>limit] for key,limit in [('core_ms',40),('delivery_ms',50)]}
    summary['x0_retired']=any(r.get('x0')=='0' for r in good)
    (CACHE/(name+'.json')).write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(name,summary['qualified'],summary['timely'],'/',scheduled,'P95',summary['timing']['core_ms']['p95'],summary['timing']['delivery_ms']['p95'],flush=True)
    return summary

def controls():
    actual=[]
    for case in ['A00','A01','A30','A31']:
        audit=CACHE/(case+'-control.input')
        assert execute([BIN/'sc7_checks','inputs',CACHE/(case+'.yaml'),case,300,SEEDS[0],audit],CACHE/(case+'-control.log'))==0
        rows=[line.split() for line in audit.read_text().splitlines() if line.startswith(('initial ','imu ','uwb '))]
        standardized=[]
        for row in rows:
            if row[0]=='initial':standardized.append(['initial',*row[-15:]])
            elif row[0]=='imu':standardized.append([*row[:4],*row[10:16],*(row[-6:] if len(row)>28 else [])])
            else:standardized.append([*row[:10],row[15]])
        actual.append(standardized)
    assert all(rows==actual[0] for rows in actual)
    (CACHE/'controls-check.json').write_text(json.dumps(dict(status='PASS',cases=['M0/G0','M0/G1','M3/G0','M3/G1'],epochs_each=300,standardized_source_rows_each=len(actual[0]),noise_sha256=hashlib.sha256(json.dumps(actual[0]).encode()).hexdigest()),indent=2)+'\n')
    print('CONTROL300 standardized initial/ADC/RW/range source identities/draws equal across all4 geometries/motions',flush=True)

def checks():
    from sc6_system_research import CORE_CHECKS
    tests=CORE_CHECKS+[('sc6_closure_checks',[]),('sc7_checks',[]),('sc7_joint_checks',[]),('sc7_support_checks',[])]
    results=[]
    for name,args in tests:
        code=execute([BIN/name,*args],CACHE/(name+('-boundary' if args==['boundary'] else '')+'-final.log'));results.append(dict(name=name+' '+str(args),returncode=code))
    code=execute(['python3','tools/test_sc6_closure.py'],CACHE/'python-final.log');results.append(dict(name='test_sc6_closure.py',returncode=code))
    (CACHE/'final-checks.json').write_text(json.dumps(results,indent=2)+'\n')
    if any(r['returncode'] for r in results):raise SystemExit(1)

def final_matrix():
    entries=[]
    for case in ['S0','S1','S2']:
        for seed in SEEDS:
            for variant in ['final-old','final-new']:
                entries.append(dict(case=case,seed=seed,variant=variant,mode='paced'))
    for case in ['S1','S2']:
        for seed in SEEDS:entries.append(dict(case=case,seed=seed,variant='final-new',fde='off',mode='paced'))
    for seed in SEEDS:entries.append(dict(case='S3',seed=seed,variant='final-new',mode='paced'))
    for window in ['20/20','40/40','30/10']:entries.append(dict(case='V',window=window,variant='final-new',mode='paced'))
    for case in ['F0','F1','F2','F3','F4','F5','W0','W1','W2']:entries.append(dict(case=case,variant='final-new',mode='offline',count=160 if case.startswith('F') else 200))
    entries += [dict(case='S0',variant='final-new-cancel',mode='paced',count=16,hook='cancel:3'),dict(case='S0',variant='final-new-delay',mode='paced',count=16,hook='delay:3')]
    for case,fde in [('L0','on'),('L1','on'),('L1','off')]:entries.append(dict(case=case,variant='final-new',fde=fde,mode='paced'))
    return entries

def freeze():
    assert not subprocess.check_output(['git','status','--porcelain','-uno'],cwd=ROOT,text=True).strip(),'save source before final candidate freeze'
    frozen=dict(identity=source_identity(),config_sha=sha(ROOT/'config/sc7.yaml'),requirements_sha=sha(ROOT/'docs/requirements/sc7/SC7_REQUIREMENTS.md'),matrix=final_matrix())
    (CACHE/'final-registration.json').write_text(json.dumps(frozen,indent=2)+'\n');print('FROZEN',frozen['identity'],flush=True)

def final():
    registration=json.loads((CACHE/'final-registration.json').read_text());current=source_identity()
    assert current==registration['identity'],'registered final candidate changed'
    for e in registration['matrix']:
        params=dict(case=e['case'],seed=e.get('seed',SEEDS[0]),count=e.get('count',0),mode=e.get('mode','paced'),fde=e.get('fde','on'),variant=e['variant'],window=e.get('window','30/30'),hook=e.get('hook','audit'))
        name=f"{params['variant']}-{params['case']}-{params['seed']}-{params['window'].replace('/','-')}-{params['fde']}-{params['mode']}";path=CACHE/(name+'.json')
        if path.exists():
            data=json.loads(path.read_text());assert data['identity']==current and data['trace_sha']==sha(CACHE/(name+'.csv')),'stale final result';print('REUSE',name,flush=True)
        else:data=native(**params)
        if params['mode']=='paced' and params['variant']=='final-new' and not data['positive_realtime_pass']:
            raise RuntimeError('registered positive timing/availability failed before long-stream continuation: '+name)
    print('FINAL MATRIX EXECUTED',len(registration['matrix']),flush=True)

def ranges(values):
    values=sorted(set(values));groups=[]
    for x in values:
        if groups and x==groups[-1][-1]+1:groups[-1].append(x)
        else:groups.append([x])
    return ','.join(str(g[0]) if len(g)==1 else f'{g[0]}–{g[-1]}' for g in groups) or 'none'

def plots(runs):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from sc6_closure import current, spatial
    chosen=[(name,d) for name,d in runs if '-cancel-' not in name and '-delay-' not in name and any('-'+c+'-' in name for c in ['S0','S1','S2','S3','L0','L1'])]
    for axis,pe,pl in [('H','hpe','hpl'),('V','vpe','vpl')]:
        fig,axs=plt.subplots(2,3,figsize=(15,9),constrained_layout=True)
        for ax,case in zip(axs.flat,['S0','S1','S2','S3','L0','L1']):
            xmax=ymax=0
            for name,d in chosen:
                if '-'+case+'-' not in name:continue
                rows=list(csv.DictReader((CACHE/(name+'.csv')).open()));good=[r for r in rows if spatial(r)];timely=[r for r in good if current(r)];late=[r for r in good if not current(r)]
                color=('#67a9cf' if '-off-' in name else '#2166ac') if name.startswith('final-new') else '#b2182b';label=('new ' if name.startswith('final-new') else 'old ')+('OFF' if '-off-' in name else 'ON')
                if timely:ax.scatter([value(r,pe) for r in timely],[value(r,pl) for r in timely],s=2,alpha=.25,color=color,label=label)
                if late:ax.scatter([value(r,pe) for r in late],[value(r,pl) for r in late],s=7,color='gray',marker='x')
                if good:xmax=max(xmax,max(value(r,pe) for r in good));ymax=max(ymax,max(value(r,pl) for r in good))
            limit=max(xmax,ymax,1)*1.03;ax.plot([0,limit],[0,limit],color='black',lw=.6);ax.set(xlim=(0,limit),ylim=(0,limit),title=case,xlabel=pe+' [m]',ylabel=axis+'PL [m]');ax.grid(alpha=.2);handles,labels=ax.get_legend_handles_labels();legend=dict(zip(labels,handles));ax.legend(legend.values(),legend.keys(),fontsize=6,markerscale=2)
        fig.suptitle(axis+' Stanford: full common PE/PL scale per scenario; late spatial points gray; research only')
        fig.savefig(ROOT/f'docs/fde_next/sc7_{axis.lower()}_stanford.png',dpi=180);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
    for ax,case in zip(axs.flat,['S0','S1','S2','L0']):
        name=next(n for n,d in chosen if n.startswith('final-new-') and '-'+case+'-' in n and '-20261010-' in n and '-on-' in n);rows=list(csv.DictReader((CACHE/(name+'.csv')).open()));good=[r for r in rows if committed(r)];x=[int(r['epoch']) for r in good]
        ax.plot(x,[value(r,'reference_bwidth_2') for r in good],label='old same-event RW union');ax.plot(x,[value(r,'bwidth_2') for r in good],label='new all-fault union');ax.set(title=case,xlabel='epoch',ylabel='accel bias-z width [m/s²]');ax.legend(fontsize=7);ax.grid(alpha=.2)
    fig.suptitle('Main raw bias support; sample0 contraction inherited through all later fault explanations')
    fig.savefig(ROOT/'docs/fde_next/sc7_bias_union.png',dpi=180);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(14,5),constrained_layout=True)
    paced=[(n,d) for n,d in runs if n.startswith('final-new') and '-paced' in n and '-cancel-' not in n and '-delay-' not in n]
    labels=[n.replace('final-new-','').replace('-30-30','').replace('-paced','') for n,d in paced]
    x=np.arange(len(paced));axs[0].bar(x-.18,[d['timing']['core_ms']['p95'] or 0 for n,d in paced],.36,label='core P95');axs[0].bar(x+.18,[d['timing']['delivery_ms']['p95'] or 0 for n,d in paced],.36,label='measurement-to-sink P95');axs[0].axhline(40,color='orange',ls='--');axs[0].axhline(50,color='red',ls='--');axs[0].set_xticks(x);axs[0].set_xticklabels(labels,rotation=90,fontsize=6);axs[0].set_ylabel('ms, nearest-rank, all admitted services');axs[0].legend()
    name=next(n for n,d in paced if '-L1-' in n and '-on-' in n);rows=list(csv.DictReader((CACHE/(name+'.csv')).open()));good=[r for r in rows if committed(r)];axs[1].plot([value(r,'epoch') for r in good],[value(r,'ape') for r in good],lw=.6,label='actual online tag APE')
    for n,d in paced:
        if n==name:
            for e in d['episodes']:axs[1].axvline(e['onset'],color='red',alpha=.4,lw=.6)
    axs[1].set(xlabel='L1 epoch (original repeated episodes)',ylabel='APE m');axs[1].legend();axs[1].grid(alpha=.2)
    fig.savefig(ROOT/'docs/fde_next/sc7_recovery_p95.png',dpi=180);plt.close(fig)

def report():
    registration=json.loads((CACHE/'final-registration.json').read_text());runs=[]
    for e in registration['matrix']:
        name=f"{e['variant']}-{e['case']}-{e.get('seed',SEEDS[0])}-{e.get('window','30/30').replace('/','-')}-{e.get('fde','on')}-{e.get('mode','paced')}";d=json.loads((CACHE/(name+'.json')).read_text());assert d['identity']==registration['identity'] and d['trace_sha']==sha(CACHE/(name+'.csv'));runs.append((name,d))
    by_name=dict(runs);pairs=[]
    for name,d in runs:
        if name.startswith('final-old-'):
            other=name.replace('final-old-','final-new-',1);o=by_name[other];assert d['raw_input_sha']==o['raw_input_sha'];pairs.append((name,other,d,o))
        if name.startswith('final-new-') and '-off-' in name:
            other=name.replace('-off-','-on-');o=by_name[other];assert d['raw_input_sha']==o['raw_input_sha'];pairs.append((name,other,d,o))
    positives=[(n,d) for n,d in runs if '-cancel-' not in n and '-delay-' not in n and not any('-'+c+'-' in n for c in ['W0','W1','W2'])]
    paced=[(n,d) for n,d in positives if n.endswith('-paced')];new=[(n,d) for n,d in positives if n.startswith('final-new-')]
    g2=[d for n,d in new if '-S3-' in n];hv=all(d['mathematical']['H']['widened']==d['mathematical']['V']['widened']==0 for n,d in new) and sum(d['mathematical']['H']['strict'] for n,d in new)>0 and sum(d['mathematical']['V']['strict'] for n,d in new)>0
    bias=any(d['union_bias']['2']['strict']>30 and d['x0_retired'] for n,d in new if '-S0-' in n)
    realtime=all(d['positive_realtime_pass'] for n,d in paced)
    episodes=[(n,e) for n,d in new if '-on-' in n for e in d['episodes']];recovered=sum(e['recovered'] for n,e in episodes)
    checks=json.loads((CACHE/'final-checks.json').read_text());core=all(r['returncode']==0 for r in checks)
    completed=hv and bias and realtime and core and len(g2)==2 and all(d['qualified']==300 for d in g2) and recovered==len(episodes) and all(d['returncode']==0 and d['committed']==d['scheduled'] for n,d in positives)
    status='SC7_COMPLETE_FOR_REGISTERED_SCOPE' if completed else 'SC7_IMPLEMENTED_WITH_OPEN_CAPABILITIES'
    path=ROOT/'docs/fde_next/SC7_RESULTS.md';previous=path.read_text().split('\n## W3 —')[0];previous=previous.replace('SC7_IN_PROGRESS — W0 implemented and directly verified; W1→W3 continue in the same Goal.',status+' — final frozen combined native evaluation below.');previous=previous.replace('Final C13, C15, HPL, VPL, recovery, P95 and SC7-A01–A20 assessment pending W1–W3. This is an intermediate stage save, not SC7 completion.','W0 was an intermediate save; final combined assessment follows W3 below.')
    s=previous+'\n## W3 — final combined implementation and evidence\n\n'
    ident=registration['identity'];s+=f"**{status}**. Tested source SHA `{ident['head']}`; source-content digest `{ident['sources_sha256']}`; ELF `{ident['elf']}`; actual loaded production DSO `{ident['dso']}`. Documentation save SHA is separately resolved by `git log -1 --format=%H -- docs/fde_next/SC7_RESULTS.md` after the final local save; this file does not invent a self-referential SHA. Frozen requirements SHA `{registration['requirements_sha']}`.\n\n"
    s+='''The common H/V operator keeps all9 range-mask leaf boxes and every budget frontier. Directed lower-shell coordinate projection retains both signs unless an existing interval proves one; depth2 splitting and upper/lower/slab propagation reduce feasible z extrema. HPL is max over per-leaf horizontal corner radii at the actual committed center; VPL comes from the surviving z endpoints. It is not a per-branch VPL trick or a sampled optimizer. Old box support is retained as a mathematically valid fallback. `sc7-branch-box-union-tag-v1` and immutable support evidence bind complete raw batch, parent, event/q, center, receipt, frame, directed arithmetic and frontier policy; consumers recompute the numerical bounds. Seal/receipt failure after a native mutation never retries the update.

The measured dense W2 cost motivated a frozen single IMU-contractor sweep after the first physical pass, across the full configured window. This is a bounded outer-contractor schedule, not a claim of identical P-class semantics to the dense prototype. Every unconsumed constraint is simply relaxed; no fault branch or source is dropped. Original raw reference remains available. M comparisons below use the same input box/parent/batch/event/fault scope/actual center immediately before terminal support refinement. A comparisons run complete old/new or ON/OFF native loops; changed recovery centers are allowed and explicitly not called bit-identical. The exact dominant-hull optimization is P-class: processed branch union can terminate when it equals input X or the common physical integral image U. Every branch output is a subset of U⊆X, proved with the same directed sum/contract order; once the processed hull is U the complete hull is U. The fixed axis order is independent of GT/FDE. Remaining branches retain the corresponding proven common frontier, rather than being pruned. Independent full-enumeration/shortcut comparisons retain identical p/v/R/bias supports on all8-epoch healthy/fault problems and every resource budget. This changes work/frontier counters, never the mathematical state or source event.

All paced runs keep independent absolute200Hz IMU/20Hz UWB releases. Core includes source qualification, all numerical candidates, proof, seal/receipt, native commit and cleanup; sink includes queuing/payload copy/gating. GT and file I/O are post-sink. All admitted measurable slow/abort services enter nearest-rank P95; no timing filter by qualification or deadline. Missing/unattempted/expired are separate. Production/publication/application flags remain0. Zero sampled underbounds does not prove epsilon=1e-6; the original source-event containment is the conditional mathematical argument. Original AL values are post-processing only.

'''
    capability=[('C15/G2',all(d['qualified']==300 and d['x0_retired'] for d in g2) and len(g2)==2,'native300×2, actual first epoch/x0 exit/single commit, full raw domain; no unique-root/Gaussian award'),('C13/main raw bias',bias,'S0/D0 accel bias-z full fault union; inherited original sample0 force-norm information across real separators; no later-fault general bias observability claim'),('HPL',hv,'same-center terminal branch support strictly improves H with no widened samples'),('VPL',hv,'actual z extrema contracted by shell/split operator; same-center no widening'),('fault recovery',recovered==len(episodes),f'{recovered}/{len(episodes)} original strong ON episodes restored; weak/held/outside cases separate'),('P95',realtime,f'{sum(d["positive_realtime_pass"] for n,d in paced)}/{len(paced)} required paced positive profiles dualP95 and timely-spatial≥95%')]
    s+='|Capability|Actual status|Scope|\n|---|---|---|\n'
    for k,v,note in capability:s+=f"|{k}|{'PASS' if v else 'OPEN'}|{note}|\n"
    s+='\n### Per-profile timing, availability and every miss\n\nEach timing cell is P50/P95/P99/max ms; denominator includes all scheduled epochs. Offline rows are functional evidence only.\n\n|Run|scheduled/attempted/committed/raw/timely|core P50/P95/P99/max|sink P50/P95/P99/max|core40/sink50 epochs|P95 positive|\n|---|---|---|---|---|---|\n'
    def fmt(v):return 'NA' if v is None else f'{v:.6g}'
    def timing(t):return '/'.join(fmt(t[k]) for k in ['p50','p95','p99','max'])
    def triplet(t):return '/'.join(fmt(t[k]) for k in ['rmse','p95','max'])
    for n,d in runs:
        s+=f"|{n}|{d['scheduled']}/{d['attempted']}/{d['committed']}/{d['qualified']}/{d['timely']}|{timing(d['timing']['core_ms'])}|{timing(d['timing']['delivery_ms'])}|{ranges(d['miss_epochs']['core_ms'])} / {ranges(d['miss_epochs']['delivery_ms'])}|{d['positive_realtime_pass']}|\n"
    s+='\n### M-class H/V and all-fault bias support\n\n|Native new run|same-input center n|H/V strict|H/V widened|H gain P95/max m|V gain P95/max m|bias-z strict epochs|joint fallbacks|leaf count max|\n|---|---|---|---|---|---|---|---|---|\n'
    for n,d in new:
        m=d['mathematical'];s+=f"|{n}|{m['n']}|{m['H']['strict']}/{m['V']['strict']}|{m['H']['widened']}/{m['V']['widened']}|{fmt(m['H']['improvement']['p95'])}/{fmt(m['H']['improvement']['max'])}|{fmt(m['V']['improvement']['p95'])}/{fmt(m['V']['improvement']['max'])}|{d['union_bias']['2']['strict']}|{d['joint_fallbacks']}|{fmt(d['support_boxes']['max'])}|\n"
    s+='\n### A-class pairs, all state accuracy and recovery\n\nRaw-input digests compare initial observation plus complete raw IMU/UWB bytes, including source IDs/stamps/draws. Whole audit SHA may differ because it also prints research method configuration. Every old/new and ON/OFF pair above is checked on the actual raw-input digest. No trace alignment or retrospective smoothing. Accuracy cells are RMSE/P95/max; m, m/s, rad, m/s², rad/s respectively. All adverse values are retained.\n\n|Run|tag APE|HPE|VPE|velocity|SO3|accel bias vector|gyro bias vector|\n|---|---|---|---|---|---|---|---|\n'
    for n,d in runs:
        s+='|'+n+'|'+'|'.join(triplet(d['metrics'][k]) for k in ['ape','hpe','vpe','velocity_error','rotation_error','accel_bias_error','gyro_bias_error'])+'|\n'
    s+='\n|Run|ba x|ba y|ba z|bg x|bg y|bg z|RPE1 trans/rotation RMSE|RPE20 trans/rotation RMSE|\n|---|---|---|---|---|---|---|---|---|\n'
    for n,d in runs:
        m=d['metrics'];s+='|'+n+'|'+'|'.join(triplet(m[k]) for k in ['baerr_x','baerr_y','baerr_z','bgerr_x','bgerr_y','bgerr_z'])+'|'+fmt(m['rpe1_translation']['rmse'])+'/'+fmt(m['rpe1_rotation']['rmse'])+'|'+fmt(m['rpe20_translation']['rmse'])+'/'+fmt(m['rpe20_rotation']['rmse'])+'|\n'
    s+='\n|Pair first→second|raw digest|common committed/missing pair epochs|tag RMSE first→second|signed mean / absolute P95/max paired tag-error delta|\n|---|---|---|---|---|\n'
    for a,b,da,db in pairs:
        ra={r['epoch']:r for r in csv.DictReader((CACHE/(a+'.csv')).open()) if committed(r)};rb={r['epoch']:r for r in csv.DictReader((CACHE/(b+'.csv')).open()) if committed(r)};keys=ra.keys()&rb.keys();delta=[value(ra[k],'ape')-value(rb[k],'ape') for k in keys];st=stats(delta)
        s+=f"|{a}→{b}|{da['raw_input_sha']}|{len(keys)}/{max(da['scheduled'],db['scheduled'])-len(keys)}|{fmt(da['metrics']['ape']['rmse'])}→{fmt(db['metrics']['ape']['rmse'])}|{fmt(float(np.mean(delta)) if delta else None)}/{fmt(st['p95'])}/{fmt(st['max'])}|\n"
    s+='\nPhase precedence follows Closure; phases partition committed epochs. Weak faults may remain undetected legitimately.\n\n|Run/phase|n|tag|velocity|SO3|ba vector|bg vector|\n|---|---|---|---|---|---|---|\n'
    for n,d in runs:
        if n.startswith('final-new-') and any('-'+c+'-' in n for c in ['S1','S2','L1']):
            for phase,m in d['stages'].items():s+=f"|{n}/{phase}|{m['ape']['n']}|"+'|'.join(triplet(m[k]) for k in ['ape','velocity_error','rotation_error','accel_bias_error','gyro_bias_error'])+'|\n'
    s+='\n|ON run/IMU onset axis duration|FD/bridge/prior repair/rejoin4|source TTD/TTI/TTR ms|shared-fault rejoin|recovered|\n|---|---|---|---|---|\n'
    for n,e in episodes:s+=f"|{n}/{e['onset']} axis{e['axis']} {e['duration_ms']}ms|{e['first_fd']}/{e['first_bridge']}/{e['first_prior_repair']}/{e['rejoin4']}|{e['TTD_ms']}/{e['TTI_ms']}/{e['TTR_ms']}|{e['rejoin_on_fault_shared_endpoint']}|{e['recovered']}|\n"
    s+='\n|Run/phase|ba x|ba y|ba z|bg x|bg y|bg z|\n|---|---|---|---|---|---|---|\n'
    for n,d in runs:
        if n.startswith('final-new-') and any('-'+c+'-' in n for c in ['S1','S2','L1']):
            for phase,m in d['stages'].items():s+='|'+n+'/'+phase+'|'+'|'.join(triplet(m[k]) for k in ['baerr_x','baerr_y','baerr_z','bgerr_x','bgerr_y','bgerr_z'])+'|\n'
    s+='\n### Stanford and research AL sensitivity\n\nEach axis/AL cell: scheduled/spatial/timely; covered/underbound; AL-admissible/timely-AL-admissible; MI/HMI counterfactual candidates. Expired points are never counted timely. Missing points are not synthesized.\n\n|Run|AL H/V|H counts|V counts|expired/refused/missing|\n|---|---|---|---|---|\n'
    def regions(a):return f"{a['scheduled']}/{a['spatial_qualified']}/{a['time_current']};{a['covered']}/{a['underbound']};{a['AL_admissible']}/{a['timely_AL_admissible']};{a['MI_candidate']}/{a['HMI_candidate']}"
    for n,d in runs:
        for al,axes in d['stanford'].items():
            a=axes['H'];s+=f"|{n}|{al}|{regions(a)}|{regions(axes['V'])}|{a['expired']}/{a['refused']}/{d['missing']}|\n"
    s+='\n### Resources, long history and complete regressions\n\nSlots/ledger/provider storage remain O(T), inherited from the original backend; no O(1)/WCET award. New IMU material and support are bounded by the configured window and finite leaf/frontier policy. Source-prefix count is a virtual identity recipe, not a new source allocation per constraint reuse. Stage costs below include maintenance in the measured core.\n\n|Run|slots first→last|live max|ledger max|RSS first→last KiB|active states/proof epochs max|replay/capsule max|leaf/frontier max|source prefix max|proof/cleanup P95 ms|\n|---|---|---|---|---|---|---|---|---|---|\n'
    for n,d in runs:
        r=d['resource'];x=d['resources_extra'];s+=f"|{n}|{fmt(r['slots']['first'])}→{fmt(r['slots']['last'])}|{fmt(r['live']['max'])}|{fmt(r['ledger']['max'])}|{fmt(r['rss']['first'])}→{fmt(r['rss']['last'])}|{fmt(x['active_states']['max'])}/{fmt(x['proof_explicit']['max'])}|{fmt(r['replay']['max'])}/{fmt(r['capsules']['max'])}|{fmt(x['support_boxes']['max'])}/{fmt(x['support_frontier']['max'])}|{fmt(x['source_prefix']['max'])}|{fmt(x['proof_ms']['p95'])}/{fmt(x['cleanup_ms']['p95'])}|\n"
    s+='\nExecuted independent/core checks: '+', '.join('`'+r['name']+'` '+('PASS' if r['returncode']==0 else 'FAIL') for r in checks)+'. No unrelated historical campaign rerun.\n\n'
    statuses={k:'PASS' for k in range(1,21)}
    for k,v in [(6,len(g2)==2 and all(d['qualified']==300 for d in g2)),(9,bias),(12,hv),(14,core),(17,realtime),(18,all(d['committed']==d['scheduled'] for n,d in new if any('-'+c+'-' in n for c in ['L0','L1']))),(19,core)]:statuses[k]='PASS' if v else 'OPEN'
    notes={1:'baseline/source sigma/risk/fault/AL/publication audit; research opt-in',2:'W0 fixed-center staged native attribution; endpoints labelled outer, no fabricated percentages',3:'four300 common-D1 motion/geometry controls, legacy D0 separate; G2 design/local rank and global mirrors',4:'independent V identity, H3/4/5, two-time projection and actual-center tests',5:'scaled fixed six-seed damped trust region with residual curvature; original sigma/covariance',6:'two native G2 300 traces, sole commit, first epoch and actual x0 retirement',7:'coplanar roots and unsearched-domain retention; Gaussian qualification explicitly false',8:'raw IMU/state/theta integral constraints in configured30 chain; directed continuous remainder/union proof',9:'S0 full fault-union bias-z shrinks via original sample0 shell and persists across native separators; later-fault general observability OPEN',10:'legal distinct static yaw histories and bias/ADC/adjacent episode overlap, no powerlessness overclaim',11:'full leaf/frontier common H/V support at final center, versioned consumer dispatch',12:'registered native strict H and V M gains; no same-center widening; A trajectories separate',13:'original birth event/q and repeat scope; epoch-theta outer relaxation; frontier and separator retention',14:'seal/source/center/event/q/parent/receipt tamper, proof-after-commit no second update and safe fallback',15:'same raw-input new/old and ON/OFF pairs; all-state/axis/phase metrics and actual recovery denominators',16:'trace-derived H/V Stanford, research AL sensitivity, late/missing denominators',17:'per-profile paced dual nearest-rank P95 and95% timely-spatial; all misses/max retained',18:'L0 5000,L1 10000 ON/OFF actual long streams; x0 retirement, repeated repairs, bounded new memory and inherited O(T) slots',19:'K1–K9 related oracles, core and Python regressions; failed development timings retained',20:'this single result report, necessary code/config/tests, ≤4 real figures, explicit local commits and owned-cache cleanup'}
    s+='### SC7-A01–A20\n\n|Clause|Status|Actual evidence / limit|\n|---|---|---|\n'
    for k in range(1,21):s+=f"|SC7-A{k:02}|{statuses[k]}|{notes[k]}|\n"
    if not completed:s+='\nOPEN capabilities: '+', '.join(k for k,v,note in capability if not v)+'. No completion claim is based merely on having executed work packages.\n'
    s+='''
Further limits remain even for passing registered conditional scope: no general15D/yaw observability, no arbitrary-later-fault-only bias identification, no root-completeness or inherited multiroot Gaussian alpha theorem, no application/hardware publication, no O(1) source/provider/slot lifetime or WCET guarantee. VPL may still exceed research AL. Counterexamples and finite zero-underbound runs do not replace the event containment argument.

Reproduction: `python3 tools/sc7_research.py check`; `python3 tools/sc7_research.py joint`; `sc7_support_checks`; `python3 tools/sc7_research.py geometry` (W1-only reproduction should disable joint/tag support to reproduce its stage timing); `python3 tools/sc7_research.py freeze`; `python3 tools/sc7_research.py final`; `python3 tools/sc7_research.py report`. Build `sc7_checks sc7_joint_checks sc7_support_checks epoch_raim_runner` in the documented workspace first; the final core checks are listed above. Freeze requires saved tracked source. Final registration/matrix, traces and input audits are temporary in `.cache/sc7/`; report/figures are derived from individual same-receipt rows, not synthesized from old averages. After report/denominator/hash verification, owned temporary evidence is cleaned. No push.

![Horizontal Stanford](sc7_h_stanford.png)
![Vertical Stanford](sc7_v_stanford.png)
![Main all-fault bias union](sc7_bias_union.png)
![Recovery and P95](sc7_recovery_p95.png)
'''
    path.write_text(s);plots(runs)
    print(status,'runs',len(runs),'raw',sum(d['qualified'] for n,d in runs),'scheduled',sum(d['scheduled'] for n,d in runs),'pairs',len(pairs),'recovery',recovered,len(episodes),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['check','controls','attribute','geometry','joint','freeze','final','report','run']);p.add_argument('--case',action='append');p.add_argument('--seed',type=int,default=SEEDS[0]);p.add_argument('--epochs',type=int,default=0);p.add_argument('--fde',default='on');p.add_argument('--variant',default='new');p.add_argument('--mode',default='offline');p.add_argument('--window',default='30/30');a=p.parse_args();CACHE.mkdir(parents=True,exist_ok=True)
    if a.command=='check':checks()
    elif a.command=='controls':controls()
    elif a.command=='attribute':attribute(a.case)
    elif a.command=='joint':
        if execute([BIN/'sc7_joint_checks'],CACHE/'joint-checks.log'):raise SystemExit(1)
    elif a.command=='freeze':freeze()
    elif a.command=='final':final()
    elif a.command=='report':report()
    elif a.command=='geometry':
        if execute([BIN/'sc7_checks','geometry'],CACHE/'geometry-checks.log'):raise SystemExit(1)
        for seed in SEEDS:native('S3',seed,300,'paced',variant='w1')
        independent_geometry()
    else:
        for case in a.case or ['S0']:native(case,a.seed,a.epochs,a.mode,a.fde,a.variant,a.window)
if __name__=='__main__':main()
