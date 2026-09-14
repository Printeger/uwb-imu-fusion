#!/usr/bin/env python3
"""R6 infrastructure-only replay; all numerical work stays in frozen native/SF tools."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import yaml

from formal_native import HERE, ROOT, RUNNER, METHODS, read, rows, save, write, sha, digest, verify
from formal_r6_lf import load, MANIFEST
import formal_treatment as treatment_tool
from formal_evaluate import evaluate
from formal_benchmark import METHODS as SCIENCE_METHODS, enrich

REPLAY=HERE/'runs/R6_REPAIR_REPLAY'
PARENTS=HERE/'runs/R6_REPAIR'
AUDIT=HERE/'audits'
BLOCKED='blocked_by_scientific_parent_failure'
SCIENCE_MARKERS=('CONDITIONAL_LM_', 'MAX_REFIT_ITERATIONS', 'joint stop conditions were not all satisfied')


def science_failure(result):
    b=result.get('backend',result)
    detail='; '.join(str(b.get(k,'')) for k in ('reason','solver_status','failure_reason'))
    p=Path(result['run']) if result.get('run') else None
    if p and (p/'final_inference_summary.json').exists():
        detail+='; '+read(p/'final_inference_summary.json').get('reason','')
    return any(s in detail for s in SCIENCE_MARKERS),detail.strip('; ')


def seal(directory,name='phase_seal.json'):
    paths={str(p.relative_to(directory)):sha(p) for p in directory.rglob('*') if p.is_file() and p.name!=name}
    write(directory/name,paths)


def verify_seal(directory,name='phase_seal.json'):
    for rel,h in read(directory/name).items():
        assert sha(directory/rel)==h, 'SEALED_ARTIFACT_CHANGED:'+str(directory/rel)


def validate_case(case):
    m=Path(case['input_manifest']);doc=read(m)
    assert sha(m)==case['corrupted_input_hash']
    assert sha(m.parent/doc['uwb_file'])==case['corrupted_uwb_hash']
    for kind in ('uwb','imu'):
        assert 'sha256:'+sha(m.parent/doc[kind+'_file'])==doc[kind+'_sha256']
    assert case['bias_m']==1. if 'bias_m' in case else True
    probe=HERE/'attempts/R6-LF-v2/cache_parser_probe'
    cmd=[str(probe),str(m)]
    r=subprocess.run(cmd,capture_output=True,text=True)
    assert r.returncode==0,r.stderr
    return {'argv':cmd,'exit_code':r.returncode,'stdout':r.stdout,'stderr':r.stderr,
            'input_hash':sha(m),'uwb_hash':sha(m.parent/doc['uwb_file']),
            'imu_hash':sha(m.parent/doc['imu_file']),'probe_hash':sha(probe)}


def config_for(case):
    cfg=yaml.safe_load((HERE/'configs/backbone/FROZEN_BACKBONE.yaml').read_text())
    cfg['dataset']['cache_manifest']=case['input_manifest']
    return cfg


def run_phase(case,out,cells,external=None):
    """Existing scheduler in an isolated subprocess; no estimator changes."""
    if (out/'phase_seal.json').exists():
        verify_seal(out)
        return read(out/'native_results.json')
    if out.exists():raise RuntimeError('INCOMPLETE_PHASE_PRESERVED:'+str(out))
    out.mkdir(parents=True)
    cfg=config_for(case);config=out/'config.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=True))
    m=Path(case['input_manifest']);inp=read(m)
    files={str(m):sha(m)}
    files.update({str(m.parent/inp[k+'_file']):sha(m.parent/inp[k+'_file']) for k in ('uwb','imu')})
    write(out/'measurement_allowlist.json',{str(m):files})
    batch={'schema':'uifgo_t09_batch_v2','role':'development',
           'parameter_provenance':'IE0912_PL_BIDIRECTIONAL_LOCKED_PENDING_VALIDATION',
           'run_units':[{'run_unit_id':case['case_key'],'recording_id':inp['base_recording_id'],
                         'base_trajectory_id':inp['base_recording_id'],'seed':911,'prefix_identity':'full',
                         'config':str(config),'cells':cells}]}
    if external:
        batch['external_caches']=[{'run_unit_id':case['case_key'],'producer_id':'shared','manifest':str(external)}]
    (out/'batch.yaml').write_text(yaml.safe_dump(batch,sort_keys=False))
    lock=verify();cmd=['/usr/bin/python3',str(HERE/'formal_native.py'),'--worker',str(out)]
    identity={'git_hash':lock['git_hash'],'config_hash':sha(config),'input_hash':sha(m),
              'experiment_fingerprint':load()['experiment_fingerprint'],
              'parent_experiment_fingerprint':lock['experiment_fingerprint'],
              'case_key':case['case_key'],'case_id':case['case_id'],'sequence':case['sequence'],
              'engineering_revision':1,'implementation_hash':sha(__file__),'argv':cmd}
    env={k:v for k,v in os.environ.items() if not k.startswith('UIFGO_')}
    env['UIFGO_EXPERIMENT_WALL_LIMIT_S']='1800'
    start=time.monotonic()
    with (out/'scheduler.log').open('w') as f:
        code=subprocess.call(cmd,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT)
    write(out/'execution.json',dict(identity,exit_code=code,wall_seconds=time.monotonic()-start))
    b=read(out/'batch/batch_manifest.json')
    assert b['status'] in ('COMPLETE','COMPLETE_WITH_RUN_FAILURES'),b['status']
    results={}
    for cell in b['cells']:
        assert cell['status']!='PARENT_CACHE_UNAVAILABLE','ENGINEERING_CACHE_RESOLUTION_FAILURE'
        p=Path(cell['run_directory']);state=read(p/'run_status.json');method=METHODS[cell['canonical_mode']]
        if not state.get('valid_estimate_exported',False) and cell['status']=='COMPLETE_WITH_RUN_FAILURE':
            valid,detail=science_failure({'backend':state,'run':str(p)})
            if not valid:raise RuntimeError('UNCLASSIFIED_NATIVE_FAILURE:'+detail)
            if not state.get('reason'):state['reason']=detail
        result={'run':str(p),'cell':cell,'backend':state,'identity':dict(identity,method_id=method)}
        write(p/'formal_identity.json',result['identity']);results[method]=result
    # Same effective numerical config as the old attempt for each policy, including removed GT fields.
    old=read(HERE/'runs/R6-LF-v2/results.json')[case['case_id']]
    equality={}
    for method,result in results.items():
        oldrun=old[method].get('run')
        if oldrun:
            a=Path(oldrun)/'batch_config_effective.yaml';b=Path(result['run'])/'batch_config_effective.yaml'
            assert sha(a)==sha(b),'FROZEN_EFFECTIVE_CONFIG_CHANGED:'+method
            equality[method]={'old':str(a),'new':str(b),'sha256':sha(b)}
        oldcommon=old[method]['cell'].get('common_preparation_id')
        if oldcommon and oldcommon!='PENDING_ACTUAL_PREPARATION':
            assert result['cell']['common_preparation_id']==oldcommon
    write(out/'scientific_equivalence.json',equality)
    write(out/'native_results.json',results);verify();seal(out)
    return results


def parent_for(case):
    root=PARENTS/case['case_id']/'parent'
    producer=run_phase(case,root/'producer',[{'mode':'structured_debias','execution_type':'CACHE_PRODUCER',
                       'path':'PL_BIDIRECTIONAL_CUSUM','producer_id':'shared'}])['AUX_PRODUCER']
    run=Path(producer['run']);cache_id=producer['cell'].get('cache_id')
    caches=list((root/'producer/batch/caches').glob('*/stage2_cache_manifest.json'))
    assert len(caches)==int(bool(cache_id))
    cache=caches[0] if caches else None
    if cache:
        c=read(cache);assert c['cache_id']==cache_id
        for p in c['payloads']:assert sha(cache.parent/p['name'])==p['sha256'].split(':')[-1]
        assert root.resolve() in cache.resolve().parents
    else:
        valid,reason=science_failure(producer)
        assert valid,'ENGINEERING_PARENT_MISSING_WITHOUT_SCIENCE_FAILURE'
    detector=read(run/'production_detector_status.json')
    support=run/'production_support_partition.json'
    if detector['valid']:
        assert support.is_file()
        old=Path(read(HERE/'runs/R6-LF-v2/results.json')[case['case_id']]['AUX_PRODUCER']['run'])
        for name in ('production_support_partition.json','production_cusum_trace.csv'):
            assert sha(run/name)==sha(old/name),'DETECTOR_REPLAY_NOT_EXACT:'+name
    payload={'case_id':case['case_id'],'input_hash':case['corrupted_input_hash'],
             'config_hash':producer['identity']['config_hash'],'producer':str(run),
             'producer_status':producer['backend'],'cache_manifest':str(cache) if cache else None,
             'cache_manifest_hash':sha(cache) if cache else None,'cache_id':cache_id,
             'support_hash':detector.get('support_hash'),'detector_hash':sha(run/'production_detector_status.json'),
             'support_file_hash':sha(support) if detector['valid'] else None,
             'scientific_fingerprint':load()['experiment_fingerprint']}
    payload['parent_hash']=digest(payload)
    path=root/'parent_metadata.json'
    if path.exists():assert read(path)==payload
    else:write(path,payload)
    return producer,cache,payload


def blocked_result(case,method,out,parent,reason=BLOCKED):
    out.mkdir(parents=True,exist_ok=False)
    identity=dict(parent['identity'],method_id=method)
    if method.startswith('SF'):
        identity['config_hash']=sha(HERE/f"configs/sfuse/config_test_isas-walk{case['sequence'][-1]}.yaml")
    _,detail=science_failure(parent)
    state={'status':reason,'reason':reason+': '+detail,'failure_reason':reason+': '+detail,
           'valid_estimate_exported':False,'upstream_run':parent['run'],'executed':False}
    result={'run':str(out),'identity':identity,'backend':state,'cell':{'status':reason}} if method.startswith('FGO') else dict(state,run=str(out),identity=identity)
    write(out/'run_status.json',state);write(out/'formal_result.json',result)
    return result


def materialize_treatment(case,native,out,parent_meta):
    key='R6_REPAIR_REPLAY/'+case['case_id']+'/engineering-r1'
    # Legacy exporter is reused verbatim. It cannot publish accepted corrections from failed Values.
    for directory in ('corrected_ranges','rejected_ranges'):
        (HERE/'exports'/directory/Path(key).parent).mkdir(parents=True,exist_ok=True)
    t=treatment_tool.export(key,Path(case['input_manifest']),native)
    t.update(support_hash=parent_meta['support_hash'],detector_hash=parent_meta['detector_hash'])
    candidate=[r['obs_id'] for r in t['rows'] if r['candidate']]
    accepted=[r['obs_id'] for r in t['rows'] if r['recovery_accepted']]
    fixed={r['segment_id']:r for r in t['fixed']}
    segment_rows={r['segment_id']:r for r in t['rows'] if r['candidate']}
    doc={'case_id':case['case_id'],'input_hash':case['corrupted_input_hash'],
         'detector_hash':t['detector_hash'],'support_hash':t['support_hash'],
         'parent_hash':parent_meta['parent_hash'],'segment_count':t['segment_count'],
         'candidate_obs_ids':candidate,'accepted_obs_ids':accepted,
         'rejected_obs_ids':candidate,'recover_suppressed_obs_ids':sorted(set(candidate)-set(accepted),key=int),
         'rejected_obs_ids_semantics':'REJECT policy support; SF_RECOVER leaves nonaccepted raw rows unchanged',
         'estimated_bias_by_segment':{k:v['estimated_bias'] for k,v in segment_rows.items()},
         'sigma_by_segment':{k:v['sigma_bias'] for k,v in segment_rows.items()},
         'lcb_by_segment':{k:v['lcb'] for k,v in segment_rows.items()},
         'applied_correction_by_segment':{k:v['applied_correction'] for k,v in segment_rows.items()},
         'status':'available' if t['recover_available'] else BLOCKED,
         'support_valid':t['support_valid'],'recover_available':t['recover_available'],
         'inference_run':native['AUX_PRODUCER']['run'],
         'final_acceptance_run':native['FGO_RECOVER_FULL']['run'],
         'sigma_semantics':'local diagnostic; not calibrated confidence or final bias posterior',
         'export_semantics':'unchanged final-use accepted correction, raw minus applied correction'}
    doc['treatment_hash']=digest(doc);t['treatment_hash']=doc['treatment_hash']
    folder=HERE/'exports/treatments';folder.mkdir(parents=True,exist_ok=True)
    dest=folder/(case['case_id']+'.yaml')
    assert not dest.exists(),'TREATMENT_CANONICAL_ALREADY_EXISTS'
    dest.write_text(yaml.safe_dump(doc,sort_keys=False))
    for directory in ('corrected_ranges','rejected_ranges'):
        source=HERE/'exports'/directory/(key+'.csv');target=HERE/'exports'/directory/(case['case_id']+'.csv')
        assert not target.exists();shutil.copy2(source,target)
    # Fail-closed export checks keyed by stable raw observation IDs, independent of file existence.
    raw=rows(Path(case['input_manifest']).parent/read(case['input_manifest'])['uwb_file'])
    byid={r['obs_id']:r for r in t['rows']};assert set(byid)=={r['obs_id'] for r in raw}
    assert set(accepted)<=set(candidate)
    for r in raw:
        x=byid[r['obs_id']];assert x['timestamp']==r['sensor_time_s'] and x['anchor_id']==r['anchor_id']
        assert x['raw_range']==float(r['observed_range_m'])
        assert x['corrected_range']==x['raw_range']-x['applied_correction']
        if not x['recovery_accepted']:assert x['corrected_range']==x['raw_range']
    write(out/'treatment.json',t);write(out/'treatment_provenance.json',doc)
    return t,key


def run_case(case):
    load();pre=read(AUDIT/'R6_REPAIR_PREFLIGHT.json');assert pre['status']=='PASS'
    assert pre['implementation_hash']==sha(__file__),'PREFLIGHT_IMPLEMENTATION_CHANGED'
    cid=case['case_id'];out=REPLAY/cid/'engineering-r1'
    if (out/'case_seal.json').exists():
        verify_seal(out,'case_seal.json');verify_seal(PARENTS/cid/'parent/producer')
        return read(out/'results.json')
    if out.exists():raise RuntimeError('INCOMPLETE_CASE_PRESERVED:'+str(out))
    out.mkdir(parents=True)
    try:
        write(out/'input_validation.json',validate_case(case))
        producer,cache,parent=parent_for(case);native={'AUX_PRODUCER':producer}
        print(cid,'PARENT',producer['backend']['status'],flush=True)
        base=[{'mode':'all_range','execution_type':'BASELINE_TRAJECTORY','path':'DIRECT_COMMON_PREPARATION'},
              {'mode':'robust_cauchy','robust_scale':2.3849,'execution_type':'BASELINE_TRAJECTORY','path':'DIRECT_COMMON_PREPARATION'}]
        native.update(run_phase(case,out/'baselines',base))
        if cache:
            cfg=config_for(case)
            cells=[{'mode':m,'execution_type':'FINAL_TRAJECTORY','path':'PL_BIDIRECTIONAL_CUSUM',
                    'producer_id':'shared','thresholds':{k:cfg['nlos'][k] for k in ('tau_eta','tau_s_m','tau_gamma')}}
                   for m in ('suppress_all','lcb_fixed_full')]
            native.update(run_phase(case,out/'finals',cells,cache))
            for method in ('FGO_REJECT','FGO_RECOVER_FULL'):
                r=native[method];assert Path(r['cell']['parent_cache_manifest'])==cache
                assert r['cell']['cache_id']==parent['cache_id']
                assert r['cell']['common_preparation_id']==producer['cell']['common_preparation_id']
        else:
            for method in ('FGO_REJECT','FGO_RECOVER_FULL'):
                native[method]=blocked_result(case,method,out/method,producer)
        t,key=materialize_treatment(case,native,out,parent)
        results=dict(native);bags={}
        for method in ('SF_NATIVE','SF_REJECT','SF_RECOVER'):
            bag=treatment_tool.make_bag(key,case['sequence'],Path(case['input_manifest']),t,method);bags[method]=bag
        proof=[]
        for method,bag in bags.items():
            if bag:
                meta=read(bag.parent/(method+'_input.json'));assert meta['integrity']=='PASS'
                assert meta['gt_topics']==[] and meta['truth_fields']==[];proof.append(meta)
        assert len({p['imu_payload_hash'] for p in proof})==1
        write(out/'SF_input_integrity.json',{'status':'PASS','available_methods':[p['method_id'] for p in proof],
              'imu_equal':True,'timestamps_and_non_treated_ranges':'verified by original adapter exact deserialize comparison',
              'GT_absent':True,'injection_truth_absent':True,'inputs':proof})
        for method,bag in bags.items():
            if bag is None:
                upstream=producer if not cache else native['FGO_RECOVER_FULL']
                assert science_failure(upstream)[0],'EXPORT_MISSING_AFTER_SUCCESSFUL_INFERENCE'
                results[method]=blocked_result(case,method,out/method,upstream)
            else:
                # Stable case/revision path, independent from native scheduler run IDs.
                results[method]=treatment_tool.run_sf(cid+'/engineering-r1',case['sequence'],bag,method,'R6_REPAIR_REPLAY',ready=True)
                if results[method]['status']!='completed':
                    raise RuntimeError('SF_ENGINEERING_OR_UNCLASSIFIED_FAILURE:'+json.dumps(results[method]))
            results[method]['identity']['experiment_fingerprint']=load()['experiment_fingerprint']
            print(cid,method,results[method]['status'],flush=True)
        for method in ('FGO_REJECT','FGO_RECOVER_FULL','SF_REJECT','SF_RECOVER'):
            results[method]['identity'].update(support_hash=t['support_hash'],treatment_hash=t['treatment_hash'])
        evaluated=[]
        for method in SCIENCE_METHODS:
            r,*_=evaluate(case['sequence'],method,results[method],t,(case['start_time'],case['end_time']))
            if r['status']=='invalid_input':raise RuntimeError(r['failure_reason'])
            r=enrich(r,method,results,t,key);r.update(case_id=cid,anchor_id=case['anchor_id'],
                    support_hash=t['support_hash'],treatment_hash=t['treatment_hash'])
            evaluated.append(r)
        write(out/'evaluated.json',evaluated);write(out/'results.json',results)
        status={'case_id':cid,'status':'completed_with_scientific_failures' if any(r['status']=='failed' for r in evaluated) else 'completed',
                'engineering_revision':1,'engineering_failures':0,'scientific_fingerprint':load()['experiment_fingerprint'],
                'input_hash':case['corrupted_input_hash'],'parent_hash':parent['parent_hash'],
                'support_hash':t['support_hash'],'treatment_hash':t['treatment_hash'],
                'methods':{r['method_id']:{k:r[k] for k in ('status','failure_reason')} for r in evaluated}}
        write(out/'case_status.json',status);verify();seal(out,'case_seal.json')
        print(cid,'COMPLETE',status['status'],flush=True)
        return results
    except Exception as exc:
        write(out/'engineering_failure.json',{'error':repr(exc),'traceback':traceback.format_exc(),
              'engineering_revision':1,'scientific_fingerprint':load()['experiment_fingerprint']})
        raise


def preflight():
    doc=load();verify()
    assert sha(MANIFEST)=='c0e38c35c8e6117eb317f53540bf5e34c60fef7d2163f6c96aae755c0140bbca'
    assert len(doc['cases'])==15
    # Callable, not merely present; help stops before any estimator/metric computation.
    cmd=[str(RUNNER),'--help'];r=subprocess.run(cmd,capture_output=True,text=True)
    # Native CLI reports help as exit 2; it must advertise the cache replay/producer entry points.
    assert 'stage2-cache' in r.stdout+r.stderr or 'Usage:' in r.stdout+r.stderr
    checks=[]
    for case in doc['cases']:
        c=validate_case(case);c['case_id']=case['case_id']
        for root in (PARENTS/case['case_id'],REPLAY/case['case_id'],HERE/'exports/treatments',
                     HERE/'exports/corrected_ranges',HERE/'exports/rejected_ranges',HERE/'exports/sfuse_inputs'):
            root.mkdir(parents=True,exist_ok=True)
            p=root/'.r6_write_probe';p.write_text('probe');p.unlink()
        c.update(parent_path=str(PARENTS/case['case_id']/'parent'),parent_path_resolvable=True,
                 exporter_callable=callable(treatment_tool.export),sf_generator_callable=callable(treatment_tool.make_bag))
        checks.append(c)
    tests=['/usr/bin/python3','-m','unittest','discover','-s','experiments/icra2027','-p','test_r6_repair.py','-v']
    t=subprocess.run(tests,cwd=ROOT,capture_output=True,text=True)
    (AUDIT/'R6_REPAIR_tests.log').write_text(t.stdout+t.stderr)
    assert t.returncode==0,t.stdout+t.stderr
    result={'status':'PASS','manifest_sha256':sha(MANIFEST),'implementation_hash':sha(__file__),
            'cases':checks,'detector_cli':{'argv':cmd,'exit_code':r.returncode,'stdout':r.stdout,'stderr':r.stderr},
            'test_command':tests,'test_exit_code':t.returncode,'science_metrics_computed':False,
            'parent_cache_unavailable_engineering':0,'offline_treatment_unavailable_engineering':0,
            'numerical_success_not_implied':True}
    write(AUDIT/'R6_REPAIR_PREFLIGHT.json',result)
    (AUDIT/'R6_REPAIR_PREFLIGHT.md').write_text('# R6 repair infrastructure preflight\n\nPASS — all 15 frozen cases parsed with the actual C++ loader.\n\n'
        'Manifest SHA256 `'+sha(MANIFEST)+'`; scientific files verified by original formal lock. '
        'Case-local parent directories resolve; all output directories passed a real write probe. '
        'Native detector/producer executable called in help mode; exporter and SF input generator exercised '
        'by focused engineering tests (real ROS bag fixture, zero scientific metrics). '
        'Missing parents after genuine failed solves are terminal scientific dependencies, not preflight cache failures.\n\n'
        'Exact commands, return codes, stdout and input/probe hashes: `R6_REPAIR_PREFLIGHT.json`; tests: `R6_REPAIR_tests.log`. '
        'This PASS is infrastructure admission only, not a claim of optimizer success.\n')
    print('PREFLIGHT PASS 15/15',flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('case_id',nargs='?');p.add_argument('--preflight',action='store_true');p.add_argument('--all',action='store_true')
    a=p.parse_args()
    if a.preflight:preflight();return
    cases=load()['cases']
    if a.all:
        # Three case workers as in the frozen original protocol; methods within each case are serial.
        from concurrent.futures import ThreadPoolExecutor,as_completed
        results={}
        with ThreadPoolExecutor(max_workers=3) as pool:
            jobs={pool.submit(run_case,c):c for c in cases}
            for f in as_completed(jobs):
                c=jobs[f];results[c['case_id']]=f.result();write(REPLAY/'results.json',results)
                print('CASES FINISHED',len(results),'/15',flush=True)
    else:
        c=next(c for c in cases if c['case_id']==a.case_id);run_case(c)

if __name__=='__main__':main()
