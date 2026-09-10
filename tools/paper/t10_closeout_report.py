#!/usr/bin/env python3
"""Summarize only frozen T10 results; no estimator, tuning or truth access."""
import json
from pathlib import Path
import numpy as np
import t10_closeout as c

POLICIES=('suppress_all','structured_debias','fit_only','s_fit','full_gate')


def number(value):
    try:
        result=float(value)
        return result if np.isfinite(result) else None
    except (ValueError,TypeError):
        return None


def matrix(path):
    if not path.is_file(): return None
    rows=c.read_rows(path)
    if not rows: return None
    m=np.zeros((1+max(int(r['row']) for r in rows),1+max(int(r['column']) for r in rows)))
    for row in rows: m[int(row['row']),int(row['column'])]=float(row['value'])
    return m


def metric(data,name):
    return data.get(name,{}).get('value')


def main():
    root=Path(c.obj(c.MASTER)['evidence_root'])
    inputs=[]; groups=[]; finals=[]
    for seed in (20101,20102):
        base=f'a10_val_turn_{1 if seed==20101 else 2:02d}_seed{seed}'
        for condition in ('A','B','C'):
            case=root/'science'/f'{base}_step2_{condition}'
            meta=c.obj(case/'case.json')
            output=Path(meta.get('output_root',str(case/'output')))
            run=c.obj(case/'RUN_RESULT.json')
            status=c.obj(output/'pipeline_status.json')
            evaluation=c.obj(case/'evaluation.json')
            score_rows=c.read_rows(output/'scores.csv') if (output/'scores.csv').is_file() else []
            partitions=c.read_rows(output/'stage1/partition.csv') if (output/'stage1/partition.csv').is_file() and (output/'stage1/partition.csv').stat().st_size else None
            segments=c.read_rows(output/'segments.csv') if (output/'segments.csv').is_file() else []
            stage1=c.obj(output/'stage1/status.json') if (output/'stage1/status.json').is_file() and (output/'stage1/status.json').stat().st_size else {}
            candidate_count=sum(int(p['obs_count']) for p in partitions) if partitions is not None and stage1.get('status')=='CONVERGED' else None
            policy_decisions={p:c.read_rows(output/'decisions'/f'{p}.csv') if (output/'decisions'/f'{p}.csv').is_file() else None for p in POLICIES}
            different=None
            if policy_decisions['s_fit'] is not None and policy_decisions['full_gate'] is not None:
                different=[(x['group_id'],x['decision']) for x in policy_decisions['s_fit']] != [(x['group_id'],x['decision']) for x in policy_decisions['full_gate']]
            failure=status.get('reason') if status.get('status') not in ('SCORED','NO_CANDIDATES') else None
            row={'seed':seed,'scenario':'step2','condition':condition,'status':status['status'],
                 'candidate_observations':candidate_count,'segments':len(partitions) if candidate_count is not None else None,
                 'groups':len(score_rows) if status.get('status') in ('SCORED','NO_CANDIDATES') else None,
                 'eligible':status.get('eligible'),'unavailable':status.get('unavailable'),
                 's_fit_full_gate_differ':different,'algorithm_failure':failure,
                 'numerical_failure':failure if failure and any(x in failure for x in ('LAMBDA','STATIONAR','NUMERICAL','ITERATION')) else None,
                 'source':str(case),'source_output':str(output),'new_optimizer_run':condition!='A',
                 'elapsed_s':run.get('elapsed_s'),'truth_open_count':run.get('truth_open_count')}
            for score in score_rows:
                directory=output/'scoring'/score['group_id']
                n=matrix(directory/'N.csv');r=matrix(directory/'R.csv')
                ordinals=set(score['ordinals'].split(';'))
                part=[p for p in partitions if p['ordinal'] in ordinals]
                fits=[p for p in segments if p['ordinal'] in ordinals]
                decisions={p:next((x for x in policy_decisions[p] if x['group_id']==score['group_id']),{}) for p in POLICIES}
                eta=number(score['eta']);s=number(score['s_m'])
                g={'seed':seed,'scenario':'step2','condition':condition,'group_id':score['group_id'],
                   'segment_ids':[p['segment_id'] for p in part],'segment_ordinals':sorted(ordinals),
                   'observation_count':sum(int(p['obs_count']) for p in part),
                   'segment_observation_counts':[int(p['obs_count']) for p in part],
                   'status':score['status'],'eligible':score['eligible'] in ('1','true'),
                   'N':n.tolist() if n is not None else None,'N_eigenvalues':np.linalg.eigvalsh(n).tolist() if n is not None else None,
                   'lambda_min_R_c':float(np.linalg.eigvalsh(r)[0]) if r is not None else None,
                   'eta':eta,'s_m':s,'gamma':[number(p['postfit_gamma']) for p in fits],
                   'N_differs_from_6400I':not np.allclose(n,6400*np.eye(n.shape[0]),rtol=1e-12,atol=1e-10) if n is not None else None,
                   'old_fixed_N_eta_mapping_error':eta-1/(6400*s*s) if eta is not None and s is not None and s>0 else None,
                   's_fit_decision':decisions['s_fit'].get('decision'),'full_gate_decision':decisions['full_gate'].get('decision'),
                   's_fit_full_gate_differ':decisions['s_fit'].get('decision')!=decisions['full_gate'].get('decision'),
                   'decisions':{p:decisions[p].get('decision') for p in POLICIES},
                   'numerical_failure':score['numerical_reason'] if score['status'] not in ('OK','SHORT_SUPPORT','BOUNDARY_OR_KKT_INVALID') else None,'source':str(directory)}
                groups.append(g)
            for policy in POLICIES:
                result=evaluation.get('policies',{}).get(policy,{})
                trajectory=result.get('trajectory_historical_3_6_s',{})
                full=result.get('trajectory_full',{})
                delta=result.get('historical_delta_vs_suppress_all',{})
                f={'seed':seed,'scenario':'step2','condition':condition,'policy':policy,
                   'status':result.get('status',{}).get('status','NOT_RUN_PRODUCER_FAILED'),
                   'valid_estimate':result.get('status',{}).get('valid_estimate',False),
                   'decision_use_count':result.get('accepted_candidate_observations'),
                   'final_use_count':result.get('final_used_candidate_observations'),
                   'historical_rmse_m':metric(trajectory,'rmse_m'),'historical_p95_m':metric(trajectory,'p95_m'),
                   'historical_matches':trajectory.get('matches'),'full_rmse_m':metric(full,'rmse_m'),'full_p95_m':metric(full,'p95_m'),
                   'delta_rmse_vs_suppress_m':metric(delta,'rmse_m'),'delta_p95_vs_suppress_m':metric(delta,'p95_m'),
                   'decision_accepted_bias_rmse_m':metric(result.get('decision_time',{}),'accepted_bias_rmse_m'),
                   'final_accepted_bias_rmse_m':metric(result.get('final_time',{}),'accepted_bias_rmse_m'),
                   'decision_bad_correction_rate':metric(result.get('decision_time',{}),'bad_correction_rate'),
                   'final_bad_correction_rate':metric(result.get('final_time',{}),'bad_correction_rate'),
                   'candidate_use_coverage':metric(result.get('coverage',{}),'candidate_use_coverage'),
                   'fallback_attempted':result.get('fallback',{}).get('attempted'),
                   'algorithm_failure':failure or (None if result.get('status',{}).get('valid_estimate') else result.get('status',{}).get('status'))}
                finals.append(f)
            structured=next(f for f in finals if f['seed']==seed and f['condition']==condition and f['policy']=='structured_debias')
            row.update(structured_delta_rmse_m=structured['delta_rmse_vs_suppress_m'],structured_delta_p95_m=structured['delta_p95_vs_suppress_m'])
            inputs.append(row)
    for group in groups:
        group['final_results'] = [f for f in finals if f['seed']==group['seed'] and f['condition']==group['condition']]
    for group in groups:
        group['final_results']={f['policy']:{k:f[k] for k in ('status','valid_estimate','final_use_count','historical_rmse_m','historical_p95_m','algorithm_failure')} for f in finals if f['seed']==group['seed'] and f['condition']==group['condition']}
    c.write(root/'RESULTS.json',{'inputs':inputs,'groups':groups,'finals':finals})
    for name,rows in [('INPUT_RESULTS',inputs),('GROUP_RESULTS',groups),('FINAL_RESULTS',finals)]:
        c.write_rows(root/(name+'.csv'),[{k:json.dumps(v,sort_keys=True) if isinstance(v,(dict,list)) else v for k,v in row.items()} for row in rows])
    print(json.dumps({'inputs':inputs,'groups':groups},indent=2))


if __name__=='__main__':
    main()
