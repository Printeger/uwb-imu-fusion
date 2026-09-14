#!/usr/bin/env python3
"""Evaluator-only extension: unchanged R1/R2 association and SE3 score functions."""
from pathlib import Path
import numpy as np
from formal_native import HERE,WS,read,rows,save,write,sha,verify
from evaluate_clean import associate,score,ev


def evaluate(sequence, method, result, treatment=None, window=None):
    identity=result['identity'];run=Path(result['run']) if result.get('run') else None
    state=result.get('backend',result)
    native=method.startswith('FGO')
    valid=state.get('valid_estimate_exported',False) if native else state.get('status')=='completed'
    status='completed' if valid else 'failed'
    if valid and state.get('status')=='NO_CANDIDATES':status='no_candidates'
    if valid and 'FALLBACK' in state.get('status',''):status='fallback'
    summary=read(run/'final_inference_summary.json') if run and (run/'final_inference_summary.json').exists() else {}
    fallback=summary.get('fallback_attempt_count',0)>0 or status=='fallback'
    if valid and fallback:status='fallback'
    obs=rows(run/'observations.csv') if run and (run/'observations.csv').exists() else []
    row={'sequence':sequence,'method_id':method,'status':status,
         'failure_reason':'' if valid else state.get('reason',state.get('failure_reason',result.get('cell',{}).get('status','UNAVAILABLE'))),
         'runtime':state.get('elapsed_seconds',state.get('runtime')),
         'runtime_scope':'NATIVE_CELL_PLUS_SHARED_PRODUCER' if method in ('FGO_REJECT','FGO_RECOVER_FULL') else
                         'NATIVE_ESTIMATOR_CELL' if native else 'SF_END_TO_END_1X_PLAYBACK',
         'candidate_count':treatment['candidate_count'] if treatment and treatment['support_valid'] else None,
         'candidate_count_semantics':'SHARED_OFFLINE_DETECTOR; base/robust do not apply support',
         'accepted_segment_count':sum(c.get('final_use')=='1' for c in treatment['fixed']) if treatment and method in ('FGO_RECOVER_FULL','SF_RECOVER') else 0,
         'used_range_count':sum(o['strategy_used']=='1' for o in obs) if obs and valid else None,
         'state_count':len(ev.load_tum(run/'trajectory.tum')) if native and valid and run and (run/'trajectory.tum').exists() else None,
         'trajectory_sample_count':len(ev.load_tum(run/'trajectory.tum')) if valid and run and (run/'trajectory.tum').exists() else None,
         'fallback':int(fallback),'run':str(run),
         'ATE_RMSE':None,'ATE_RMSE_aligned':None,'ATE_P95':None,'RPE_1s':None,'fault_window_RMSE':None,
         **{k:identity[k] for k in ('git_hash','config_hash','experiment_fingerprint','input_hash')}}
    if not native and run and (run/'adapter_runtime.txt').exists():
        row['used_range_count_semantics']='SF_INTERNAL_USED_COUNT_NOT_EXPORTED; retained input count separately'
    if not valid:return row,[],None,None
    try:
        gt=WS/f'res/ie0911_step2_truth_20260911_01/sfuise_walk{sequence[-1]}/ground_truth.tum'
        common=read(HERE/'audits/frame_fits_evaluator_only.json')[sequence]['common_gt_times']
        lock=verify();manifest=Path(lock['inputs'][int(sequence[-1])-1]['manifest'])
        raw=rows(manifest.parent/read(manifest)['uwb_file']);times=[float(x['sensor_time_s']) for x in raw]
        mapping=associate(ev.load_tum(run/'trajectory.tum'),ev.load_tum(gt),min(times),max(times))
        if not set(common)<=set(mapping):raise ValueError('FROZEN_COMMON_GT_COVERAGE_MISSING_'+str(len(set(common)-set(mapping))))
        metrics,points,fit=score(mapping,common);row.update(metrics,ATE_RMSE=metrics['ATE_RMSE_aligned'])
        if window:
            errors=[p['aligned_error_m'] for p in points if window[0]<=p['gt_time']<window[1]]
            if not errors:raise ValueError('NO_FAULT_WINDOW_GT')
            row['fault_window_RMSE']=float(np.sqrt(np.mean(np.square(errors))))
        write(run/'formal_evaluation.json',{'metrics':row,'fit':fit,'gt_hash':sha(gt),'evaluator_only':True})
        return row,points,fit,mapping
    except (ValueError,KeyError,FileNotFoundError) as exc:
        row.update(status='invalid_input',failure_reason='EVALUATION: '+str(exc))
        return row,[],None,None


def semantic_bag_hash(path):
    import hashlib
    import rosbag
    from formal_treatment import serialize
    h=hashlib.sha256()
    with rosbag.Bag(str(path)) as bag:
        for topic,msg,t in bag.read_messages():
            h.update(topic.encode());h.update(str(t.to_nsec()).encode());h.update(serialize(msg))
    return h.hexdigest()


def clean_equivalence(sequence, results, bags, treatment):
    metric={m:evaluate(sequence,m,r,treatment) for m,r in results.items()}
    data=[]
    for m in ('SF_REJECT','SF_RECOVER'):
        applicable=treatment['support_valid'] and treatment['candidate_count']==0
        row={'sequence':sequence,'method_id':m,'applicable_no_candidates':int(applicable),
             'status':'not_applicable_detected' if not applicable else 'pending'}
        if applicable:
            assert semantic_bag_hash(bags[m])==semantic_bag_hash(bags['SF_NATIVE']),'CLEAN_INPUT_DIFFERENCE'
            row['input_equivalent']=True
            a=metric['SF_NATIVE'];b=metric[m]
            if a[3] is None or b[3] is None:
                row.update(status='failed',failure_reason='SF_RUN_OR_EVALUATION_FAILED')
            else:
                keys=read(HERE/'audits/frame_fits_evaluator_only.json')[sequence]['common_gt_times']
                pos=[np.linalg.norm(np.array(a[3][t][0][1])-np.array(b[3][t][0][1])) for t in keys]
                rot=[ev.rotation_angle(ev.quaternion_matrix(a[3][t][0][2]).T@ev.quaternion_matrix(b[3][t][0][2])) for t in keys]
                delta=abs(a[0]['ATE_RMSE']-b[0]['ATE_RMSE'])
                row.update(max_position_difference_m=float(max(pos)),max_rotation_difference_rad=float(max(rot)),ATE_difference_m=delta,
                           status='PASS' if max(pos)<=.001 and max(rot)<=.001 and delta<=.001 else 'FAIL_TOLERANCE')
        data.append(row)
    return data
