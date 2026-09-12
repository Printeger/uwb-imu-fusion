"""Final sealed-artifact evaluator for admitted Recover vs Reject runs."""
import csv,json
from pathlib import Path
import numpy as np
import pandas as pd
import run_recover_vs_reject as r
import evaluate_recover_vs_reject as e


def rows(path):
    if not Path(path).exists():return []
    with Path(path).open() as f:return list(csv.DictReader(f))


def boolval(v):return str(v).lower() in ('1','true')


def identity(out,name,task):
    run=Path(task['run_directory']);prep=r.read(run/'common_preparation.json');cache=r.read(task['cache_manifest'])
    inp=r.read(out/name/'input/input_manifest.json');obs=rows(run/'observations.csv')
    return dict(measurement_id=inp['cache_id'],raw_uwb_sha256=inp['uwb_sha256'],imu_sha256=inp['imu_sha256'],
        input_plan_hash=r.digest([{k:v[k] for k in ['obs_id','valid','planned','keyframe_id']} for v in obs]),
        nominal_sigma_hash=r.digest([{k:v[k] for k in ['obs_id','nominal_sigma_m']} for v in obs]),
        original_values_hash=prep['values_sha256'],common_preparation_id=prep['common_preparation_id'],
        support_hash=cache['support_partition_sha256'],stage2_cache_id=cache['cache_id'],stage2_values_hash=cache['stage2_values_sha256'])


def final_offsets(run,planned):
    masks=rows(run/'final_masks.csv');fixed=rows(run/'fixed_compensations.csv')
    summary=r.read(run/'final_inference_summary.json') if (run/'final_inference_summary.json').exists() else {}
    fallback=summary.get('fallback_attempt_count',0)>0
    segments={v['segment_id']:v for v in fixed};offsets={};accepted=set()
    for m in masks:
        if boolval(m['candidate']) and boolval(m['final_use']) and not boolval(m['fallback_use']):
            obs=int(m['obs_id']);s=segments.get(m['segment_id'])
            if s is None or not boolval(s['final_use']):raise ValueError('FINAL_OFFSET_MASK_MISMATCH')
            accepted.add(obs);offsets[obs]=float(s['delta_c_fixed_m'])
    if fallback and accepted:raise ValueError('FALLBACK_WITH_RECOVERY')
    candidate=sum(boolval(v['candidate']) for v in masks)
    cs={v['segment_id'] for v in masks if boolval(v['candidate'])}
    ds={v['segment_id'] for v in masks if boolval(v['candidate']) and boolval(v['decision_use'])}
    fs={v['segment_id'] for v in masks if boolval(v['candidate']) and boolval(v['final_use']) and not boolval(v['fallback_use'])}
    return offsets,accepted,e.outcome_metrics(candidate,len(cs),len(ds),len(fs),fallback)


def evaluate(out):
    from rr_admitted import verify
    lock=verify(out);ledger=r.read(out/'execution.json')
    if ledger['lock_id']!=lock['lock_id'] or ledger['status']=='RUNNING':raise ValueError('RUN_NOT_SEALED')
    for task in ledger['tasks']:
        for path,digest in task.get('artifact_hashes',{}).items():
            if r.sha(path)!=digest:raise ValueError('SCIENTIFIC_ARTIFACT_CHANGED:'+path)
    if (out/'metrics.csv').exists():raise ValueError('EVALUATION_EXISTS')
    private=r.WS/'evaluator_private/icra/recover_vs_reject'/out.name;private.mkdir(parents=True,exist_ok=False)
    metric_rows=[];pair_rows=[];common_rows=[];audits=[];pair_checks=[]
    for rec in lock['rows']:
        name=rec['recording'];lo,hi=rec['interval_s'];times=e.grid(lo,hi)
        audits.append(e.audit_recording(name,private,lock['source_hashes'][str(r.ROOT/'data/starloc/data'/name/'uwb.csv')],estimator_status='SEE_RUN_LEDGER'))
        reference=pd.read_csv(private/name/'range_reference.csv',dtype={'obs_id':str})
        g=pd.read_csv(private/name/'tag_gt.csv').drop_duplicates('time_s').sort_values('time_s')
        gt=e.interpolate_gt(g.time_s.to_numpy(),g[['x','y','z']].to_numpy(),times)
        windows=r.read(private/name/'window_union.json')
        tasks={v['task']:v for v in ledger['tasks'] if v['recording']==name};estimates={};local={}
        prepared=out/name/'prepare_runs/prepare/observations.csv'
        plan=[int(v['obs_id']) for v in rows(prepared) if boolval(v['valid']) and boolval(v['planned'])]
        raw={int(v.obs_id):float(v.range_m) for v in reference.itertuples()};hgt={int(v.obs_id):float(v.h_GT_m) for v in reference.itertuples()}
        for method in r.METHODS:
            task=tasks[method];run=Path(task.get('run_directory',out/name/'NO_RESULT'))
            status=r.read(run/'run_status.json') if (run/'run_status.json').exists() else {}
            row=dict(recording=name,method=method,status=task['status'],failure=task.get('reason',''),exit_code=task.get('exit_code'),
                ATE_RMSE_m=None,ATE_P95_m=None,Window_RMSE_m=None,evaluated_samples=0,coverage=0.,
                candidate_observations=None,candidate_segments=None,decision_accepted_segments=None,final_accepted_segments=None,
                fallback=None,range_before_RMSE_m=None,range_after_RMSE_m=None,range_count=None,
                recovered_range_count=None,recovered_before_RMSE_m=None,recovered_after_RMSE_m=None)
            estimates[method]=np.full_like(gt,np.nan)
            if task['status']=='SUCCESS':
                if method!='SFUISE-ToA' and not status.get('valid_estimate_exported', (run/'trajectory.tum').exists()):raise ValueError('INVALID_ESTIMATE')
                tum=np.loadtxt(run/'trajectory.tum',ndmin=2);et,ep=e.tag_trajectory(tum,lock['geometry']['lever_body_m'])
                estimates[method]=e.nearest_estimate(et,ep,times,[lo,hi])
                row.update(e.matched_metrics({method:estimates[method]},gt,times,windows)[method])
                offsets={};accepted=set()
                if method in r.METHODS[:2]:
                    offsets,accepted,counts=final_offsets(run,plan);row.update(counts)
                else:row['fallback']=False
                row.update(e.range_metrics(plan,raw,hgt,offsets,accepted,row['fallback']))
            else:
                # Available producer support is reported even if its dependent final failed.
                producer=tasks['producer'];pr=Path(producer.get('run_directory',out/name/'NO_RESULT'))
                if method in r.METHODS[:2] and (pr/'production_detector_status.json').exists():
                    det=r.read(pr/'production_detector_status.json')
                    row['candidate_observations']=det.get('candidate_observation_count')
                    row['candidate_segments']=det.get('final_segment_count')
            local[method]=row
        rr_methods=r.METHODS[:2];pair=dict(recording=name,delta_RR_m=None,delta_RR_NLOS_m=None,interpretation='NA',
            fallback=None,common_samples=0,comparison_status='UNAVAILABLE')
        if all(tasks[m]['status']=='SUCCESS' for m in rr_methods):
            id1,id2=[identity(out,name,tasks[m]) for m in rr_methods];r.require_pair(id1,id2)
            pair_checks.append(dict(recording=name,status='PASS',identity=id1))
            pm=e.matched_metrics({m:estimates[m] for m in rr_methods},gt,times,windows)
            pair.update(e.paired_difference(pm[rr_methods[0]],pm[rr_methods[1]],local[rr_methods[1]]['fallback']),
                        common_samples=pm[rr_methods[0]]['evaluated_samples'],comparison_status='MATCHED_IDENTICAL_STAGE2')
            # Headline RR rows use the same complete common interval, not separate coverage.
            for m in rr_methods:local[m].update(pm[m])
        else:pair_checks.append(dict(recording=name,status='UNAVAILABLE_DEPENDENT_FAILURE'))
        four=e.matched_metrics(estimates,gt,times,windows)
        for m,v in four.items():common_rows.append(dict(recording=name,method=m,**v))
        pair['four_method_common_samples']=four[r.METHODS[0]]['evaluated_samples'];pair_rows.append(pair)
        metric_rows += list(local.values())
        # Save associated raw estimates; alignment is evaluator-only and never fed back.
        r.write(private/name/'grid.json',times.tolist())
        np.savez(private/name/'associated.npz',gt=gt,**estimates)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(9,3))
        for m,a in estimates.items():
            valid=np.isfinite(a).all(1)&np.isfinite(gt).all(1);err=e.alignment_errors(a[valid],gt[valid])
            if err is not None:ax.plot(times[valid],err,label=m,lw=.8)
        for a,b in windows:ax.axvspan(a,b,alpha=.1,color='grey')
        ax.set(xlabel='time [s]',ylabel='aligned antenna error [m]',title=name+' (individual coverage diagnostic)')
        if ax.lines:ax.legend(fontsize=8)
        fig.tight_layout();fig.savefig(private/name/'trajectory_errors.png',dpi=130);plt.close(fig)
    fields=list(dict.fromkeys(k for row in metric_rows for k in row))
    e.save_csv(out/'metrics.csv',metric_rows,fields);e.save_csv(out/'paired_differences.csv',pair_rows)
    e.save_csv(out/'four_method_common.csv',common_rows);e.save_csv(out/'evaluator_audit.csv',audits)
    r.write(out/'pairing_checks.json',pair_checks)
    r.write(out/'evaluation.json',dict(status='COMPLETE_WITH_FAILURES_RETAINED',private_directory=str(private),
        lock_id=lock['lock_id'],metrics=metric_rows,pairs=pair_rows))
    print(json.dumps({'metrics':metric_rows,'pairs':pair_rows},indent=2));return 0
