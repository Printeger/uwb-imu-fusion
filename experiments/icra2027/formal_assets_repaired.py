#!/usr/bin/env python3
"""R6 repaired destinations/provenance; unchanged LF-v2 metric and decision calculations."""
import json
import csv
from pathlib import Path
import numpy as np
from formal_native import HERE,read,rows,save,write,sha,verify
from formal_evaluate import evaluate
from formal_benchmark import METHODS,enrich,detection,table_assets
import formal_r6_lf as formal_injection


def number(value):
    if value is None or value=='':return None
    x=float(value)
    return x if np.isfinite(x) else None


def median(values):
    v=[x for x in map(number,values) if x is not None]
    return float(np.median(v)) if v else None


def materialize():
    lock=verify();manifest=formal_injection.load();results=read(HERE/'runs/R6_REPAIR_REPLAY/results.json')
    clean={(r['sequence'],r['method_id']):r for r in rows(HERE/'metrics/E1_clean_metrics.csv')}
    runrows=[];pairs=[];detect=[];ranges=[];segments=[];details={}
    for case in manifest['cases']:
        cid=case['case_id'];key='R6_REPAIR_REPLAY/'+cid+'/engineering-r1';seq=case['sequence'];native=results[cid]
        treatment=read(HERE/'runs/R6_REPAIR_REPLAY'/cid/'engineering-r1/treatment.json');t=treatment
        truth=set(case['affected_obs_ids']);plannedtruth=set(case['planned_affected_obs_ids']);rr={}
        identity={k:native['AUX_PRODUCER']['identity'][k] for k in ('git_hash','config_hash','experiment_fingerprint','input_hash','method_id')}
        detect.append(dict(detection(case,t),**identity))
        if not t['support_valid']:
            for field in ('true_positive','false_positive','false_negative','candidate_count','segment_count'):
                detect[-1][field]=None
        for method in METHODS:
            row,points,fit,mapping=evaluate(seq,method,native[method],t,(case['start_time'],case['end_time']))
            row=enrich(row,method,native,t,key);row.update(case_id=cid,anchor_id=case['anchor_id'])
            row['downstream_input_hash']=row['input_hash'] if method.startswith('SF') and native[method].get('status')=='completed' else None
            if method.startswith('SF') and native[method].get('status')=='blocked_by_scientific_parent_failure':
                row['input_hash_semantics']='RAW_MEASUREMENT_PARENT; downstream treatment unavailable, SF not launched'
            if not row['input_hash']:
                row['input_hash']=case['corrupted_input_hash']
                row['input_hash_semantics']='RAW_MEASUREMENT_PARENT; downstream treatment unavailable, SF not launched'
            baseline=number(clean[(seq,method)]['ATE_RMSE']);ate=row['ATE_RMSE']
            row['delta_corrupted_minus_clean_ATE']=ate-baseline if ate is not None and baseline is not None else None
            runrows.append(row);rr[method]=row;details[(cid,method)]=(points,fit)
        pair={'case_id':cid,'sequence':seq,'anchor_id':case['anchor_id'],'experiment_fingerprint':manifest['experiment_fingerprint'],
              'git_hash':lock['git_hash'],'input_hash':case['corrupted_input_hash'],
              'config_hash':sha(HERE/'configs/formal_lock.json'),'method_id':'PAIRED_RECOVER_MINUS_REJECT'}
        for name,a,b,metric in [('delta_FGO_RR_ATE','FGO_RECOVER_FULL','FGO_REJECT','ATE_RMSE'),
                                ('delta_FGO_RR_fault','FGO_RECOVER_FULL','FGO_REJECT','fault_window_RMSE'),
                                ('delta_SF_RR_ATE','SF_RECOVER','SF_REJECT','ATE_RMSE'),
                                ('delta_SF_RR_fault','SF_RECOVER','SF_REJECT','fault_window_RMSE'),
                                ('delta_SF_recover_native','SF_RECOVER','SF_NATIVE','ATE_RMSE')]:
            x,y=rr[a][metric],rr[b][metric];pair[name]=x-y if x is not None and y is not None else None
        pair.update({m+'_status':rr[m]['status'] for m in METHODS})
        for method in ('FGO_REJECT','FGO_RECOVER_FULL','SF_REJECT','SF_RECOVER'):
            assert native[method]['identity']['support_hash']==t['support_hash']
            assert native[method]['identity']['treatment_hash']==t['treatment_hash']
        pair.update(support_hash=t['support_hash'],treatment_hash=t['treatment_hash'],shared_support_treatment_verified=True)
        pairs.append(pair)
        for r in t['rows']:
            component=1. if r['obs_id'] in truth else 0.
            error=r['estimated_bias']-component if r['estimated_bias'] is not None else None
            ranges.append(dict(r,case_id=cid,sequence=seq,injected_component_m=component,
                               bias_estimation_error=error,injected_component_error=r['applied_correction']-component,
                               total_latent_bias_truth='UNKNOWN_PARENT_BIAS',**identity))
        fixed={r['segment_id']:r for r in t['fixed']}
        for s in t['segments']:
            f=fixed.get(s['segment_id'],{});ids={str(oid) for oid in s['obs_ids']};component=len(ids&truth)/len(ids)
            estimate=number(f.get('c_hat_stage2_m'));sigma=number(f.get('sigma_c_local_m')) if f.get('sigma_available')=='1' else None
            lcb=max(0.,estimate-2*sigma) if estimate is not None and sigma is not None else None
            segments.append({'case_id':cid,'sequence':seq,'segment_id':s['segment_id'],'anchor_id':s['anchor_id'],
                             'candidate_count':len(ids),'candidate_observations':json.dumps(sorted(ids,key=int)),
                             'start_time':s['start_time'],'end_time':s['end_time'],
                             'estimated_bias':estimate,'sigma_bias':sigma,'LCB':lcb,
                             'accepted':f.get('final_use')=='1','decision_accepted':f.get('decision_use')=='1',
                             'status':'completed' if f else 'failed',
                             'decision_status':'accepted' if f.get('final_use')=='1' else 'rejected' if f else 'unavailable_stage2_failure',
                             'applied_correction':number(f.get('delta_c_fixed_m')) if f.get('final_use')=='1' else 0.,
                             'injected_component_mean':component,'bias_estimation_error':estimate-component if estimate is not None else None,
                             'bias_truth_semantics':'INJECTED_COMPONENT_ONLY_NOT_TOTAL_PARENT_EXCESS','reason':f.get('reason'),**identity})
    for file,data in [('E2_canonical_runs',runrows),('E2_canonical_pairwise',pairs),('E2_canonical_detection',detect),
                      ('E2_canonical_range',ranges),('E2_canonical_segments',segments)]:save(HERE/'metrics'/(file+'_repaired.csv'),data)
    table=[]
    for m in METHODS:
        data=[r for r in runrows if r['method_id']==m]
        table.append({'Method':m,'Planned':15,'Evaluable':sum(r['ATE_RMSE'] is not None for r in data),
                      'Median ATE':median([r['ATE_RMSE'] for r in data]),
                      'Median fault RMSE':median([r['fault_window_RMSE'] for r in data]),
                      'Median clean degradation':median([r['delta_corrupted_minus_clean_ATE'] for r in data]),
                      'Failed':sum(r['status'] in ('failed','invalid_input') for r in data),
                      'Fallback':sum(r['fallback'] for r in data),'No candidates':sum(r['status']=='no_candidates' for r in data)})
    save(HERE/'metrics/R6_REPAIR_method_summary.csv',table)
    classification=go_nogo(manifest,runrows,pairs,detect,segments)
    write(HERE/'runs/R6_REPAIR_REPLAY/stage_status.json',{'status':'completed','planned_cases':15,'attempted_cases':len(results),
                                         'method_rows':len(runrows),'RECOVERY_CLAIM':classification})
    return classification


def go_nogo(manifest,runs,pairs,detect,segments):
    planned=[c['case_id'] for c in manifest['cases']]
    failures={c:[r['method_id']+': '+r['failure_reason'] for r in runs if r['case_id']==c and r['status'] in ('failed','invalid_input')] for c in planned}
    failures={k:v for k,v in failures.items() if v};fallback=sorted({r['case_id'] for r in runs if r['fallback']})
    no_candidates=[r['case_id'] for r in detect if r['status']=='completed' and r['candidate_count']==0]
    summaries={};stable={}
    for prefix in ('FGO','SF'):
        a=[p['delta_'+prefix+'_RR_ATE'] for p in pairs];f=[p['delta_'+prefix+'_RR_fault'] for p in pairs]
        valid=sum(x is not None and y is not None for x,y in zip(a,f))
        na=sum(x is not None and x<0 for x in a);nf=sum(x is not None and x<0 for x in f)
        ma,mf=median(a),median(f)
        stable[prefix]=valid>=12 and na>=10 and nf>=10 and ma is not None and mf is not None and ma<0 and mf<0
        summaries[prefix]={'valid_pairs':valid,'median_delta_RR_ATE':ma,'median_delta_RR_fault':mf,
                           'count_delta_RR_ATE_lt_0':na,'count_delta_RR_fault_lt_0':nf,
                           'worst_recovery_ATE_degradation':max((x for x in a if x is not None),default=None),
                           'worst_recovery_fault_degradation':max((x for x in f if x is not None),default=None),'stable':stable[prefix]}
    summaries['SF']['median_SF_RECOVER_minus_SF_NATIVE']=median([p['delta_SF_recover_native'] for p in pairs])
    claim=('STRONG' if stable['FGO'] and stable['SF'] else 'PARTIAL_NATIVE' if stable['FGO'] else
           'PARTIAL_TRANSFER' if stable['SF'] else 'NOT_SUPPORTED')
    det={'event_recall_of_15':sum(r['event_detected']==1 for r in detect)/15,
         **{'median_'+k:median([r[k] for r in detect]) for k in ('precision','recall','F1','offline_onset_error','offline_offset_error')}}
    errors=[r['bias_estimation_error'] for r in segments if r['bias_estimation_error'] is not None]
    bias={'median_bias_estimation_error':median(errors),'median_absolute_bias_estimation_error':median([abs(e) for e in errors]),
          'semantics':'per-segment estimated bias minus mean injected component; parent latent bias UNKNOWN',
          'sigma_semantics':'local diagnostic, not calibrated confidence or final bias posterior'}
    summary={'planned_cases':15,'completed_cases_all_7_evaluable':15-len(failures),'attempted_cases':15,'failed_cases':failures,
             'fallback_cases':fallback,'no_candidate_cases':no_candidates,'Native_FGO':summaries['FGO'],'SFUISE':summaries['SF'],
             'Detector':det,'Bias':bias,'RECOVERY_CLAIM':claim}
    write(HERE/'audits/E2_GO_NOGO_REPAIRED.json',summary)
    (HERE/'audits/E2_GO_NOGO_REPAIRED.md').write_text('# E2 canonical GO / NO-GO\n\nRECOVERY_CLAIM = '+claim+'\n\n'+
        'Planned cases = 15. Every planned method/case is retained; no outcome-based tuning or case selection. '
        'Completed cases below require all seven evaluable trajectories; attempted cases include failures.\n\n'+
        '```json\n'+json.dumps(summary,indent=2)+'\n```\n\n'+
        'Stable improvement was preregistered as >=12 valid pairs, >=10/15 strict improvements in both ATE and fault RMSE, '
        'and both medians <0. Failures/missing/no-candidate ties do not count as improvements. '
        'This is descriptive classification, not a significance test. Bias truth is the injected component only; '
        'these exposed development recordings do not establish unseen-dataset generalization or physical NLOS performance.\n\n'+
        'No automatic R7/R8/R9/R10, parameter tuning, or algorithm modification.\n')
    return claim


if __name__=="__main__":materialize()
