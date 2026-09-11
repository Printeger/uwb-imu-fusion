#!/usr/bin/env python3
"""Independent T11 evaluator: truth opens only after global freeze + complete U13."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from t11_prefix import MASTER, POLICIES
from t10_closeout import obj, sha, write, read_rows, write_rows, OLD
from a19_r08_evaluate import trajectory, rmse
from t11_verify import matrix


def group_details(case):
    out=case/'output'
    if not (out/'scores.csv').exists(): return []
    parts={r['ordinal']:r for r in read_rows(out/'stage1/partition.csv')}
    segs={r['ordinal']:r for r in read_rows(out/'segments.csv')}
    result=[]
    for score in read_rows(out/'scores.csv'):
        ordinals=score['ordinals'].split(';');segments=[parts[x] for x in ordinals]
        signature=sorted([sorted(filter(None,s['obs_ids'].split(';'))) for s in segments])
        result.append({'score':score,'segments':segments,'stage2_segments':[segs[x] for x in ordinals],
            'signature':signature,'wholly_historical':all(float(s['start'])>=3 and float(s['end'])<=6 for s in segments),
            'N':matrix(out/'scoring'/score['group_id']/'N.csv').tolist(),
            'Rc':matrix(out/'scoring'/score['group_id']/'R.csv').tolist()})
    return result


def final_metrics(case, policy, history, motion, truth):
    out=case/'output';p=out/'final'/policy
    status=obj(p/'validation_status.json') if (p/'validation_status.json').exists() else {'valid_estimate':False,'status':'NOT_RUN'}
    result={'status':status,'history_count':len(history),'bias_field_rmse_m':None,'accepted_only_rmse_m':None,
            'accepted_count':None,'coverage':None,'trajectory_rmse_m':None,'trajectory_p95_m':None,'trajectory_matches':None}
    if not status.get('valid_estimate'): return result
    masks={r['obs_id']:r for r in read_rows(p/'final_masks.csv')}
    if not set(history).issubset(masks):raise ValueError('HISTORY_MASK_INCOMPLETE')
    amplitudes={r['segment_id']:float(r['amplitude_m']) for r in read_rows(p/'segment_bias.csv')}
    applied={};accepted=[]
    for obs_id in history:
        row=masks[obs_id]
        use=row['candidate'] in ('1','true') and row['final_use'] in ('1','true')
        applied[obs_id]=amplitudes[row['segment_id']] if use else 0.
        if use:accepted.append(obs_id)
    errors={i:applied[i]-truth[i] for i in history}
    traj=trajectory(p/'trajectory.tum',motion,(3.,6.))
    result.update(bias_field_rmse_m=rmse(list(errors.values())),
        accepted_only_rmse_m=rmse([errors[i] for i in accepted]) if accepted else None,
        accepted_only_status='AVAILABLE' if accepted else 'UNDEFINED',accepted_count=len(accepted),
        coverage=len(accepted)/len(history),trajectory_rmse_m=traj['rmse_m']['value'],
        trajectory_p95_m=traj['p95_m']['value'],trajectory_matches=traj['matches'],
        fallback=obj(p/'fallback_attempt.json'),applied_correction_by_history_obs=applied)
    return result


def structural_final(case, policy, history):
    p=case/'output/final'/policy
    status=obj(p/'validation_status.json') if (p/'validation_status.json').exists() else {'valid_estimate':False,'status':'NOT_RUN'}
    result={'status':status,'metrics_status':'NOT_RUN_U13_NOT_PASSED','accepted_count':None,'coverage':None}
    if status.get('valid_estimate'):
        masks={r['obs_id']:r for r in read_rows(p/'final_masks.csv')}
        if not set(history).issubset(masks):raise ValueError('HISTORY_MASK_INCOMPLETE')
        accepted=sum(masks[i]['candidate'] in ('1','true') and masks[i]['final_use'] in ('1','true') for i in history)
        result.update(accepted_count=accepted,coverage=accepted/len(history),
                      fallback=obj(p/'fallback_attempt.json'),history_count=len(history))
    return result


def main():
    master=obj(MASTER);root=Path(master['evidence_root']);freeze=obj(root/'PRE_EVALUATION_FREEZE.json')
    if freeze['master_sha256']!=sha(MASTER):raise ValueError('MASTER_CHANGED')
    for path,hash_value in freeze['payloads'].items():
        if sha(path)!=hash_value:raise ValueError('FREEZE_CHANGED:'+path)
    u13=obj(root/'U13_RESULT.json');read_truth=u13['status']=='PASS'
    table=[];all_details={};fixed_rows=[]
    for seed in master['seeds']:
        base=next(x['row']['base'] for x in master['inputs'] if x['row']['seed']==seed)
        generation=OLD/'inputs'/base
        # Fixed history ID universe comes from immutable input, never from support.
        raw=read_rows(generation/'raw/step2/uwb_observations.csv')
        history=[r['obs_id'] for r in raw if 3<=float(r['sensor_time_s'])<=6]
        motion=truth=None
        if read_truth:
            gm=obj(generation/'generation_manifest.json')
            for name in ('evaluation/motion.csv','evaluation/step2_range_truth.csv'):
                expected=gm['payload_sha256'][name];expected=expected if expected.startswith('sha256:') else 'sha256:'+expected
                if sha(generation/name)!=expected:raise ValueError('TRUTH_HASH_MISMATCH')
            motion=[(float(r['time_s']),np.array([float(r[x]) for x in ('px_m','py_m','pz_m')])) for r in read_rows(generation/'evaluation/motion.csv')]
            truth={r['obs_id']:float(r['total_dynamic_latent_bias_m']) for r in read_rows(generation/'evaluation/step2_range_truth.csv')}
        if read_truth:
            time_sets=[]
            for h in (0,1,2):
                for policy in POLICIES:
                    p=root/'runs'/f'{seed}_H{h}'/'output/final'/policy
                    if (p/'validation_status.json').exists() and obj(p/'validation_status.json').get('valid_estimate'):
                        time_sets.append([float(line.split()[0]) for line in (p/'trajectory.tum').read_text().splitlines()
                                          if line.strip() and not line.startswith('#') and 3<=float(line.split()[0])<=6])
            if time_sets and any(t!=time_sets[0] for t in time_sets):raise ValueError('UNMATCHED_HISTORY_TRAJECTORY_TIMES')
        details=[group_details(root/'runs'/f'{seed}_H{h}') for h in (0,1,2)]
        signatures=[{json.dumps(g['signature']) for g in groups if g['wholly_historical']} for groups in details]
        comparable=bool(signatures[0]) and signatures[0]==signatures[1]==signatures[2]
        for h in (0,1,2):
            case=root/'runs'/f'{seed}_H{h}';out=case/'output'
            pipeline=obj(out/'pipeline_status.json') if (out/'pipeline_status.json').exists() else {'status':'FAILED_OR_TIMEOUT'}
            row={'seed':seed,'H_s':h,'cutoff_s':6+h,'uwb_count':master['expected_counts'][str(h)][0],
                 'imu_count':master['expected_counts'][str(h)][1],'history_obs_count':len(history),
                 'status':pipeline['status'],'failure_reason':pipeline.get('reason',''),
                 'candidate_count':sum(int(s['obs_count']) for s in read_rows(out/'stage1/partition.csv')) if (out/'stage1/partition.csv').exists() else None, 'group_count':len(details[h]) if (out/'scores.csv').exists() else None,
                 'comparable':comparable,'comparability_reason':'EXACT_OBSERVATION_AMPLITUDE_PARTITION' if comparable else 'HISTORY_SUPPORT_OR_GROUP_PARTITION_DIFFERS_OR_UNAVAILABLE',
                 'groups_json':f'GROUP_DETAILS.json#{seed}_H{h}','truth_evaluation':'RUN' if read_truth else 'NOT_RUN_U13_NOT_PASSED'}
            entry={'groups':details[h],'pipeline':pipeline,'historical_obs_ids':history,'policies':{}}
            if read_truth and (out/'segments.csv').exists():
                amps={r['segment_id']:float(r['amplitude_m']) for r in read_rows(out/'segments.csv')}
                stage2={i:0. for i in history}
                for part in read_rows(out/'stage1/partition.csv'):
                    for i in part['obs_ids'].split(';'):
                        if i in stage2:stage2[i]=amps[part['segment_id']]
                entry['stage2_history_bias_field_rmse_m']=rmse([stage2[i]-truth[i] for i in history])
            for policy in POLICIES:
                met=final_metrics(case,policy,history,motion,truth) if read_truth else structural_final(case,policy,history)
                entry['policies'][policy]=met
                for key in ('bias_field_rmse_m','accepted_only_rmse_m','accepted_count','coverage','trajectory_rmse_m','trajectory_p95_m','trajectory_matches'):
                    row[policy+'_'+key]=met.get(key)
                actual=out/'final'/policy/'validation_status.json'
                row[policy+'_status']=obj(actual).get('status','UNKNOWN') if actual.exists() else 'NOT_RUN'
                row[policy+'_valid_estimate']=obj(actual).get('valid_estimate') if actual.exists() else None
            if (out/'stage1/partition.csv').exists():
                current={x for p in read_rows(out/'stage1/partition.csv') for x in p['obs_ids'].split(';') if x in history}
                entry['historical_candidate_ids']=sorted(current)
            all_details[f'{seed}_H{h}']=entry;table.append(row)
    for seed in master['seeds']:
        for h in (1,2):
            current=all_details[f'{seed}_H{h}'];previous=all_details[f'{seed}_H{h-1}']
            if 'historical_candidate_ids' in current and 'historical_candidate_ids' in previous:
                a,b=set(previous['historical_candidate_ids']),set(current['historical_candidate_ids'])
                current['historical_candidate_change_from_previous']={'added':sorted(b-a),'removed':sorted(a-b)}
            else:current['historical_candidate_change_from_previous']={'status':'UNAVAILABLE_FAILED_STAGE'}
    fixed_audit=obj(root/'FIXED_MODEL_AUDIT.json')
    for seed in master['seeds']:
        groups=[g for g in fixed_audit['groups'] if g['seed']==seed]
        for h in (0,1,2):
            series=[{'group':g.get('group'),'data':next((s for s in g.get('series',[]) if s['H']==h),None),'status':g['status']} for g in groups]
            fixed_rows.append({'seed':seed,'H_s':h,'label':'diagnostic_fixed_model','status':'PASS' if groups and all(g['status']=='PASS' for g in groups) else 'UNAVAILABLE_OR_FAILED','groups_json':json.dumps(series,separators=(',',':'))})
    verdict='T11-C'
    if read_truth and fixed_audit['status']=='PASS':
        e2e=True
        for seed in master['seeds']:
            rows=[r for r in table if r['seed']==seed]
            if not all(r['comparable'] for r in rows):e2e=False;continue
            groups=[all_details[f'{seed}_H{h}']['groups'] for h in (0,1,2)]
            for g0 in groups[0]:
                if not g0['wholly_historical']:continue
                matched=[next(g for g in gs if g['signature']==g0['signature']) for gs in groups]
                if any(g['score']['valid_score_exported']!='1' for g in matched):e2e=False;continue
                rcs=[float(np.linalg.eigvalsh(g['Rc'])[0]) for g in matched];ss=[float(g['score']['s_m']) for g in matched]
                tol=lambda a,b:1e-10+1e-7*max(abs(a),abs(b))
                e2e &= rcs[2]-rcs[0]>tol(rcs[2],rcs[0]) and ss[0]-ss[2]>tol(ss[0],ss[2])
                e2e &= all(rcs[i+1]>=rcs[i]-tol(rcs[i+1],rcs[i]) and ss[i+1]<=ss[i]+tol(ss[i+1],ss[i]) for i in (0,1))
        verdict='T11-A' if e2e else 'T11-B'
    write_rows(root/'PREFIX_RESULTS.csv',table)
    write_rows(root/'FIXED_MODEL_RESULTS.csv',fixed_rows)
    write(root/'GROUP_DETAILS.json',all_details)
    write(root/'CONTEXT_FIGURE_DATA.json',{'history_s':[3,6],'prefix':table,'fixed_model':fixed_rows})
    write(root/'DECISION.json',{'verdict':verdict,'U13':u13['status'],'fixed_model':fixed_audit['status'],
          'truth_read':read_truth,'T10':'C2-C','C1_C2_C3':'NOT_UPGRADED','freeze_sha256':sha(root/'PRE_EVALUATION_FREEZE.json')})
    print(verdict,'truth_read',read_truth)

if __name__=='__main__':main()
