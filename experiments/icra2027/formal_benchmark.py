#!/usr/bin/env python3
"""R5/R6 orchestration, no scientific model changes and no outcome-based retries."""
import argparse
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import numpy as np
from formal_native import HERE,verify,read,rows,save,write,sha,digest,run_native
from formal_treatment import export,make_bag,run_sf
from formal_evaluate import evaluate
import formal_injection

METHODS=['FGO_BASE','FGO_ROBUST','FGO_REJECT','FGO_RECOVER_FULL','SF_NATIVE','SF_REJECT','SF_RECOVER']


def enrich(row,method,native,treatment,key):
    if method in ('FGO_REJECT','FGO_RECOVER_FULL'):
        producer=native['AUX_PRODUCER']['backend'].get('elapsed_seconds')
        row['shared_producer_runtime']=producer
        row['final_cell_runtime']=row['runtime']
        if producer is not None and row['runtime'] is not None:row['runtime']+=producer
    if method.startswith('SF'):
        meta=HERE/'exports/sfuse_inputs'/key/(method+'_input.json')
        if meta.exists():row['retained_input_range_count']=read(meta)['retained_obs_count']
        if method!='SF_NATIVE' and treatment['support_valid'] and not treatment['candidate_count'] and row['status']=='completed':
            row['status']='no_candidates'
        row['offline_inference_runtime']=sum(native['AUX_PRODUCER']['backend'].get('elapsed_seconds') or 0 for _ in [0]) if method!='SF_NATIVE' else 0
        if method=='SF_RECOVER':row['offline_inference_runtime']+=native['FGO_RECOVER_FULL']['backend'].get('elapsed_seconds') or 0
    return row


def r5():
    assert read(HERE/'runs/R4-transport-ready/stage_status.json')['status']=='PASS'
    assert read(HERE/'runs/R4-transport-ready/integrity_gate.json')['status']=='PASS'
    lock=verify();sf=read(HERE/'runs/R4-transport-ready/results.json');allrows=[];result={}
    for n in (1,2,3):
        seq=f'Walk{n}';key=f'clean{n}';manifest=Path(lock['inputs'][n-1]['manifest'])
        native=run_native(key,seq,manifest,'R5');t=export('R5-'+key,manifest,native)
        # Reuse R4's fresh, frozen clean SF runs, NOT old R1/R2 trajectories.
        # Treatment identity must match the fresh R5 native offline inference.
        old=read(HERE/'exports/corrected_ranges'/f'{key}_treatment.json')
        assert t['rows']==old['rows'],'CLEAN_OFFLINE_TREATMENT_NOT_REPRODUCIBLE'
        result[seq]=dict(native,**sf[seq]);write(HERE/'runs/R5/results.json',result)
        for method in METHODS:
            row,_,_,_=evaluate(seq,method,result[seq][method],t)
            row=enrich(row,method,native,t,key);row['source_reused_from_R4']=int(method.startswith('SF'))
            allrows.append(row)
        save(HERE/'metrics/E1_clean_metrics.csv',allrows)
        print('R5',seq,'complete',flush=True)
    table=[]
    for seq in result:
        r={r['method_id']:r for r in allrows if r['sequence']==seq}
        table.append({'Sequence':seq,'Used UWB':r['FGO_BASE']['used_range_count'],'Frozen FGO ATE':r['FGO_BASE']['ATE_RMSE'],
                      'Robust FGO ATE':r['FGO_ROBUST']['ATE_RMSE'],'SFUISE ATE':r['SF_NATIVE']['ATE_RMSE'],
                      'Candidates':r['FGO_REJECT']['candidate_count'],'Reject ATE':r['FGO_REJECT']['ATE_RMSE'],
                      'Recover ATE':r['FGO_RECOVER_FULL']['ATE_RMSE']})
    table_assets('TABLE_I_clean_backbone',table)
    write(HERE/'runs/R5/stage_status.json',{'status':'completed','rows':len(allrows),
                                         'failures':[r for r in allrows if r['status'] in ('failed','invalid_input')]})
    return allrows


def table_assets(name,data):
    for fmt in ('csv','latex'):(HERE/'tables'/fmt).mkdir(parents=True,exist_ok=True)
    save(HERE/'tables/csv'/(name+'.csv'),data)
    fields=list(data[0]);lines=['\\begin{tabular}{'+'l'*len(fields)+'}','\\hline',
                              ' & '.join(k.replace('_','\\_') for k in fields)+' \\\\', '\\hline']
    for r in data:
        def val(x):return 'NA' if x is None else f'{x:.4f}' if isinstance(x,float) else str(x).replace('_','\\_')
        lines.append(' & '.join(val(r[k]) for k in fields)+' \\\\')
    lines+=['\\hline','\\end{tabular}'];(HERE/'tables/latex'/(name+'.tex')).write_text('\n'.join(lines)+'\n')


def freeze_extensions():
    p=HERE/'configs/FORMAL_METRIC_PROTOCOL.json'
    if p.exists():return
    doc={'timing':'seconds; SF playback 1x; up to three independent case workers, wall runtime not a speed ranking',
         'robust_baseline':'existing robust_cauchy with locked historical default scale 2.3849',
         'detector_domain':'planned raw-factor observation IDs; truth restricted to this same domain',
         'precision_empty':'0 if no predicted positives; recall 0 if truth positive and no detections',
         'event_detected':'at least one target truth-positive planned observation in frozen detector support',
         'delay':'first forward threshold crossing on target anchor within [start,end), minus start; otherwise unavailable',
         'onset_offset':'earliest start/latest end of target support segments intersecting truth window minus respective truth boundary',
         'false_positive_duration':'sum of disjoint per-link candidate segment spans outside target truth interval; other links entirely false',
         'bias_errors':'estimated total static-excluded excess minus injected component (1 m target/in-window, 0 otherwise); parent latent bias unknown',
         'fault_RMSE':'same full-trajectory common-GT SE3 alignment; error restricted to [start,end)',
         'no_GT_or_truth_estimation':True,'method_ids':METHODS,
         'source_hashes':{str(p):sha(p) for p in [Path(__file__),HERE/'formal_evaluate.py',HERE/'formal_injection.py',HERE/'formal_treatment.py',HERE/'formal_sf_worker.py',HERE/'formal_transport_probe.py']}}
    doc['sha256']=digest(doc);write(p,doc)


def detection(case,t):
    truth=set(case['planned_affected_obs_ids']);pred={r['obs_id'] for r in t['rows'] if r['candidate']}
    tp=len(pred&truth);fp=len(pred-truth);fn=len(truth-pred)
    precision=tp/(tp+fp) if tp+fp else 0.;recall=tp/(tp+fn) if tp+fn else None
    start,end=case['start_time'],case['end_time'];aid=case['anchor_id']
    overlap=[s for s in t['segments'] if int(s['anchor_id'])==aid and float(s['start_time'])<end and float(s['end_time'])>=start]
    duration=0.
    for s in t['segments']:
        lo,hi=float(s['start_time']),float(s['end_time']);inside=max(0.,min(hi,end)-max(lo,start)) if int(s['anchor_id'])==aid else 0.
        duration+=max(0.,hi-lo)-inside
    trace=rows(Path(t['producer'])/'production_cusum_trace.csv') if t['producer'] and (Path(t['producer'])/'production_cusum_trace.csv').exists() else []
    alarms=[float(r['timestamp']) for r in trace if int(r['anchor_id'])==aid and r['forward_threshold_crossing']=='1' and start<=float(r['timestamp'])<end]
    return {'case_id':case['case_id'],'sequence':case['sequence'],'anchor_id':aid,'status':'completed' if t['support_valid'] else 'failed',
            'precision':precision if t['support_valid'] else None,'recall':recall if t['support_valid'] else None,
            'F1':2*precision*recall/(precision+recall) if t['support_valid'] and precision+recall else 0. if t['support_valid'] else None,
            'event_detected':int(tp>0) if t['support_valid'] else None,'true_positive':tp,'false_positive':fp,'false_negative':fn,
            'forward_alarm_delay':min(alarms)-start if alarms else None,
            'offline_onset_error':min(float(s['start_time']) for s in overlap)-start if overlap else None,
            'offline_offset_error':max(float(s['end_time']) for s in overlap)-end if overlap else None,
            'false_positive_duration':duration if t['support_valid'] else None,'candidate_count':len(pred),
            'segment_count':t['segment_count'],'domain':'PLANNED_OBSERVATIONS','truth_count':len(truth)}


def run_case(case):
    formal_injection.load();key=case['case_key'];seq=case['sequence'];manifest=Path(case['input_manifest'])
    native=run_native(key,seq,manifest,'R6');t=export(key,manifest,native);results=dict(native)
    write(HERE/'exports/corrected_ranges'/f'{key}_treatment.json',t)
    for method in ('SF_NATIVE','SF_REJECT','SF_RECOVER'):
        bag=make_bag(key,seq,manifest,t,method);results[method]=run_sf(key,seq,bag,method,'R6',ready=True)
        print(case['case_id'],method,results[method]['status'],flush=True)
    assert len({results[m]['identity']['config_hash'] for m in ('SF_NATIVE','SF_REJECT','SF_RECOVER')})==1
    write(HERE/'runs/R6'/key/'results.json',results)
    return results


def r6():
    assert read(HERE/'runs/R5/stage_status.json')['status']=='completed'
    freeze_extensions();manifest=formal_injection.freeze()
    print('R6 MANIFEST FROZEN',sha(formal_injection.MANIFEST),'15 cases',flush=True)
    completed={};errors={}
    # Thread orchestration only; each estimator/SF process has its own isolated namespace and output.
    with ThreadPoolExecutor(max_workers=3) as pool:
        futures={pool.submit(run_case,c):c for c in manifest['cases']}
        for future in as_completed(futures):
            c=futures[future]
            try:completed[c['case_id']]=future.result()
            except Exception as e:
                errors[c['case_id']]=type(e).__name__+': '+str(e)
                write(HERE/'runs/R6'/c['case_key']/'orchestration_failure.json',{'error':errors[c['case_id']]})
                # Integrity exceptions invalidate scientific comparisons, unlike backend solver failures.
                for f in futures:f.cancel()
                raise
            write(HERE/'runs/R6/results.json',completed)
            print('R6 CASE FINISHED',c['case_id'],len(completed),'/15',flush=True)
    return completed


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--r5',action='store_true');p.add_argument('--r6',action='store_true');args=p.parse_args()
    if args.r5:r5()
    if args.r6:r6()
