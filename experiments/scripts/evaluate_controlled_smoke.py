#!/usr/bin/env python3
"""Post-run canonical evaluator. Injection truth never passed to any backend."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import uuid
import numpy as np
import run_walk1_smoke as h
import canonical_injection as inj
import evaluate_range_gt as rg
import evaluate_runs as ev


def rmse(values):
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None


def window_metrics(trajectory,gt):
    lo,hi=h.PROTOCOL['evaluation_interval_s']
    matches,_=ev.match_trajectories([r for r in ev.load_tum(trajectory) if lo<=r[0]<=hi],gt,h.PROTOCOL['time_association'])
    if len(matches)<3:return {'status':'UNAVAILABLE_INSUFFICIENT_MATCHES'}
    est=np.array([e[1] for e,g in matches]);truth=np.array([g[1] for e,g in matches])
    centered=est-est.mean(axis=0)
    if np.linalg.matrix_rank(centered)<2:return {'status':'UNAVAILABLE_DEGENERATE_ALIGNMENT'}
    U,_,Vt=np.linalg.svd(centered.T@(truth-truth.mean(axis=0)))
    D=np.eye(3);D[2,2]=np.linalg.det(Vt.T@U.T);R=Vt.T@D@U.T
    error=np.linalg.norm((R@est.T).T+truth.mean(axis=0)-R@est.mean(axis=0)-truth,axis=1)
    mask=[inj.SPEC['onset_s']<=e[0]<inj.SPEC['offset_s'] for e,g in matches]
    return {'status':'AVAILABLE','ATE_RMSE_m':rmse(error),'NLOS_window_RMSE_m':rmse(error[mask]),
            'matched_count':len(matches),'window_count':sum(mask),
            'matched_identity':h.digest([[e[0],g[0]] for e,g in matches]),
            'alignment':'FULL_INTERVAL_SE3_SCALE_ONE_NO_WINDOW_REALIGN',
            'window_time_basis':'estimator sensor timestamp',
            'reference_status':h.PROTOCOL['metric_scope']}


def evaluate(root):
    root=Path(root).resolve();experiment=h.read(root/'experiment.json')
    if experiment.get('status')!='ESTIMATOR_ARTIFACTS_SEALED':raise ValueError('estimator must finish before evaluator')
    private=Path(experiment['private_dir']);truth=h.read(private/'injection_truth.json')
    if truth['specification']!=inj.SPEC:raise ValueError('injection protocol mismatch')
    inj.validate_child(truth['child_manifest'])
    if h.sha(inj.PARENT)!=truth['parent_manifest_sha256'] or h.sha(truth['child_manifest'])!=truth['child_manifest_sha256'] or h.sha(h.GT)!=truth['gt_sha256']:
        raise ValueError('input/GT seal changed')
    out=private/('evaluation-'+uuid.uuid4().hex);out.mkdir()
    tracked={}
    def track(p):
        p=Path(p);tracked[str(p)]=h.sha(p);return p
    track(private/'injection_truth.json');track(h.GT)
    gt=ev.load_tum(h.GT)
    clean=rg.unique(rg.read_csv(inj.PARENT.parent/'uwb_observations.csv'),'obs_id')
    truthids=set(truth['affected_obs_ids']);metrics=[];detection=[];segments=[];supportrows=[];range_summary=[]
    batch_ids={};producer_sources={};range_outputs={};plan_ids={};isolation=[]
    for condition in ('clean','corrupted'):
        batch=root/condition
        backend=h.read(track(batch/'batch/batch_manifest.json'));runs=rg.read_csv(track(batch/'runs.csv'))
        batch_ids[condition]={r['run_id'] for r in runs}
        for p in (batch/'batch').glob('*.isolation.json'):
            mount=h.read(track(p));paths=mount['read_only_files'];writable=Path(mount['writable_directory']).resolve()
            if any('/evaluator_private/' in x for x in paths) or writable!= (batch/'batch/runs').resolve():raise ValueError('truth isolation violated')
            for other in (root/('corrupted' if condition=='clean' else 'clean'),):
                if any(str(other) in x for x in paths):raise ValueError('cross-condition cache/trajectory reuse')
            isolation.append({'condition':condition,'manifest':str(p),'read_only_count':len(paths),'truth_mount_count':0})
        producer=next((c for c in backend['cells'] if c['execution_type']=='CACHE_PRODUCER'),None)
        producer_sources[condition]={}
        if producer and producer.get('run_directory'):
            directory=Path(producer['run_directory']);support=directory/'production_support.json'
            observations=rg.read_csv(track(directory/'observations.csv')) if (directory/'observations.csv').exists() else []
            planned={r['obs_id'] for r in observations if r['planned']=='1' and r['valid']=='1'};plan_ids[condition]=planned
            predicted=set()
            if support.exists():
                doc=h.read(track(support));producer_sources[condition]=doc
                for segment in doc['segments']:
                    for oid in segment['obs_ids']:
                        predicted.add(str(oid));supportrows.append({'condition':condition,'producer_run_id':producer['run_id'],
                            'segment_id':segment['segment_id'],'obs_id':str(oid),'anchor_id':str(segment['anchor_id'])})
            truthset=(truthids & planned) if condition=='corrupted' else set()
            detection.append({'condition':condition,'status':'AVAILABLE' if support.exists() else 'UNAVAILABLE',
                'planned_count':len(planned),'planned_injected_count':len(truthset),'support_count':len(predicted),
                'TP':len(predicted&truthset) if support.exists() else None,'FP':len(predicted-truthset) if support.exists() else None,
                'FN':len(truthset-predicted) if support.exists() else None,'semantics':'INJECTED_COMPONENT_ON_VALID_PLANNED_OBSERVATIONS'})
            if (directory/'segments.csv').exists():
                for seg in rg.read_csv(track(directory/'segments.csv')):
                    segments.append(dict(seg,condition=condition,producer_run_id=producer['run_id'],
                        stage2_status=h.read(directory/'run_status.json').get('status','UNAVAILABLE')))
        for run in runs:
            if run['method']=='AUX_STAGE2_PRODUCER':continue
            m={'condition':condition,'method':run['method'],'run_id':run['run_id'],'run_status':run['status'],
               'failure_reason':run['failure_reason'],'status':'UNAVAILABLE','ATE_RMSE_m':None,'NLOS_window_RMSE_m':None}
            if run['status']!='failure':
                m.update(window_metrics(track(run['output_trajectory_path']),gt))
                # Cross-check against the unmodified unified evaluator, not a new metric definition.
                old=next(r for r in rg.read_csv(batch/'trajectory_metrics.csv') if r['method']==run['method'])
                if m['status']=='AVAILABLE' and abs(m['ATE_RMSE_m']-float(old['ATE_RMSE_m']))>1e-10:raise ValueError('unified evaluator alignment mismatch')
            metrics.append(m)
        calibration=h.ROOT.parents[1]/'evaluator_private/icra/walk1_range_identity_assumed.json'
        command=[sys.executable,str(h.ROOT/'experiments/scripts/evaluate_range_gt.py'),'--batch-dir',str(batch),'--calibration',str(calibration),'--allow-identity-assumptions']
        result=subprocess.run(command,capture_output=True,text=True)
        (out/(condition+'_range.log')).write_text(result.stdout+result.stderr)
        range_status=json.loads(result.stdout);range_outputs[condition]={'exit_code':result.returncode,**range_status}
        rangefile=Path(range_status['output_dir'])/'range_metrics.csv'
        if not rangefile.exists():continue
        paired=[]
        for row in rg.read_csv(track(rangefile)):
            original=clean[row['obs_id']]
            expected=float(original['observed_range_m'])+(1. if condition=='corrupted' and row['obs_id'] in truthids else 0.)
            if float(row['measured_range'])!=expected:raise ValueError('raw child/clean obs identity mismatch')
            before=float(row['measured_range'])-float(original['observed_range_m'])
            after=float(row['corrected_range'])-float(original['observed_range_m']) if row['corrected_range'] else None
            paired.append(dict(row,injected_component_truth=1. if condition=='corrupted' and row['obs_id'] in truthids else 0.,
                               target_window_observation=row['obs_id'] in truthids,
                               paired_range_error_before=before,paired_range_error_after=after))
        rg.write_csv(out/(condition+'_paired_range.csv'),paired)
        for method in ('M0','M1','M3','M4'):
            for scope in ('all_valid','affected_valid','affected_valid_planned'):
                items=[r for r in paired if r['method']==method and r['valid']=='True' and
                       (scope=='all_valid' or r['target_window_observation']) and
                       (scope!='affected_valid_planned' or r['planned']=='True')]
                values=lambda key:[float(r[key]) for r in items if r[key] is not None and r[key]!='']
                range_summary.append({'condition':condition,'method':method,'scope':scope,'count':len(items),
                    'paired_injected_component_before_RMSE_m':rmse(values('paired_range_error_before')),
                    'paired_injected_component_after_RMSE_m':rmse(values('paired_range_error_after')),
                    'geometric_raw_RMSE_m':rmse(values('raw_range_error')),'geometric_corrected_RMSE_m':rmse(values('corrected_range_error')),
                    'geometric_reference_status':'ASSUMED_GEOMETRY_AND_CLOCK',
                    'correction_available_count':len(values('paired_range_error_after'))})
    if batch_ids['clean'] & batch_ids['corrupted']:raise ValueError('run IDs reused across conditions')
    if producer_sources['clean'] and producer_sources['corrupted']:
        for key in ['source_hash','discovery_snapshot_hash']:
            if producer_sources['clean'][key]==producer_sources['corrupted'][key]:raise ValueError('front-end source/snapshot reused')
    # Enrich the unified table while preserving its existing RPE/protocol fields.
    unified=rg.read_csv(root/'trajectory_metrics.csv')
    for row in unified:
        m=next(m for m in metrics if m['run_id']==row['run_id'])
        row.update(NLOS_window_RMSE_m=m.get('NLOS_window_RMSE_m'),NLOS_window_GT_samples=m.get('window_count',0),
                   window_alignment='FULL_INTERVAL_SE3_NO_WINDOW_REALIGN',window_start_s=inj.SPEC['onset_s'],window_end_s=inj.SPEC['offset_s'])
    h.save_csv(root/'trajectory_metrics.csv',unified)
    paired_difference=[]
    for condition in ('clean','corrupted'):
        a=next(m for m in metrics if m['condition']==condition and m['method']=='M4')
        b=next(m for m in metrics if m['condition']==condition and m['method']=='M3')
        comparable=a['status']==b['status']=='AVAILABLE' and a['matched_identity']==b['matched_identity']
        paired_difference.append({'condition':condition,'status':'PAIRED' if comparable else 'UNAVAILABLE_OR_GT_MISMATCH',
            'definition':'Recover(M4)-Reject(M3); negative is improvement',
            'ATE_RMSE_difference_m':a['ATE_RMSE_m']-b['ATE_RMSE_m'] if comparable else None,
            'NLOS_window_RMSE_difference_m':a['NLOS_window_RMSE_m']-b['NLOS_window_RMSE_m'] if comparable else None})
    available=[m for m in metrics if m['status']=='AVAILABLE']
    same_gt=len(available)==8 and len({m['matched_identity'] for m in available})==1
    same_plan=plan_ids.get('clean')==plan_ids.get('corrupted') and bool(plan_ids.get('clean'))
    for name,rows in [('trajectory_metrics.csv',metrics),('detection_metrics.csv',detection),('range_summary.csv',range_summary),('paired_difference.csv',paired_difference)]:h.save_csv(out/name,rows)
    rg.write_csv(out/'detection_support.csv',supportrows,['condition','producer_run_id','segment_id','obs_id','anchor_id'])
    if segments:h.save_csv(out/'estimated_segment_bias.csv',segments)
    else:rg.write_csv(out/'estimated_segment_bias.csv',[],['condition','producer_run_id','segment_id','amplitude_m','stage2_status'])
    if any(h.sha(p)!=digest for p,digest in tracked.items()):raise ValueError('estimator/input artifacts changed during evaluation')
    ok=same_gt and same_plan and all(c==0 for c in experiment['batch_exit_codes'].values()) and all(s['exit_code']==0 for s in range_outputs.values())
    summary={'status':'CANONICAL_PIPELINE_PASS' if ok else 'CANONICAL_PIPELINE_INCOMPLETE_OR_FAILED',
             'output_dir':str(out),'injection_truth':str(private/'injection_truth.json'),'affected_raw_count':len(truthids),
             'all_eight_identical_GT_set':same_gt,'clean_corrupted_identical_planned_ids':same_plan,
             'fresh_frontends':producer_sources,'isolation_audit':isolation,'range_evaluations':range_outputs,
             'trajectory_metrics':metrics,'detection':detection,'paired_difference':paired_difference,
             'source_hashes':tracked,'estimator_launched_by_evaluator':False}
    h.write(out/'summary.json',summary)
    h.write(root/'evaluation_pointer.json',{'summary':str(out/'summary.json'),'status':summary['status']})
    print(json.dumps({'status':summary['status'],'output_dir':str(out),'metrics':metrics,'detection':detection,'paired_difference':paired_difference}))
    return 0 if ok else 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--experiment-dir',required=True,type=Path)
    raise SystemExit(evaluate(p.parse_args().experiment_dir))
