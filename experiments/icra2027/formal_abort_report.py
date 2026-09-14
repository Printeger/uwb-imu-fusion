#!/usr/bin/env python3
"""Seal the authorized R6 input-error stop; no input edits or estimator reruns."""
import csv
import hashlib
from pathlib import Path
import yaml
from formal_native import HERE,ROOT,verify,read,rows,write,save,sha
from formal_evaluate import evaluate
from formal_benchmark import METHODS,table_assets
from formal_injection import load,MANIFEST

REASON='CANONICAL_NATIVE_CACHE_HEADER_CRLF; exact LF header required; rejected before initialization/solver'


def csv_output(path,data,fields=None):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields or list(dict.fromkeys(k for r in data for k in r)),lineterminator='\n')
        writer.writeheader();writer.writerows(data)


def main():
    lock=verify();manifest=load();original_hash=sha(MANIFEST)
    expected=b'obs_id,source_message_index,source_range_index,source_observation_index,sensor_time_s,tag_id,anchor_id,observed_range_m,fp_rssi_dbm,rx_rssi_dbm,source_valid,source_validity_reason_hex'
    runs=[];pairwise=[];detection=[];range_rows=[];audit=[];attempted=[];detail={}
    for case in manifest['cases']:
        key=case['case_key'];cid=case['case_id'];seq=case['sequence'];path=Path(case['input_manifest'])
        input_doc=read(path);payload=(path.parent/input_doc['uwb_file']).read_bytes()
        header=payload.split(b'\n',1)[0]
        assert header==expected+b'\r'
        assert sha(path)==case['corrupted_input_hash'] and sha(path.parent/input_doc['uwb_file'])==case['corrupted_uwb_hash']
        out=HERE/'runs/R6'/key;native=read(out/'native_results.json') if (out/'native_results.json').exists() else {}
        if native:attempted.append(cid)
        audit.append({'case_id':cid,'input_manifest':str(path),'input_hash':sha(path),'uwb_hash':case['corrupted_uwb_hash'],
                      'header_hex':header.hex(),'expected_header_hex':expected.hex(),'CRLF_count':payload.count(b'\r\n'),
                      'status':'invalid_input','native_attempted':bool(native),'reason':REASON})
        for method in METHODS:
            sf=method.startswith('SF');method_out=out/method if sf else Path(native[method]['run']) if native.get(method,{}).get('run') else None
            result=read(method_out/'formal_result.json') if sf and method_out and (method_out/'formal_result.json').exists() else native.get(method)
            bag=HERE/'exports/sfuse_inputs'/key/(method+'.bag') if sf else None
            r={'case_id':cid,'sequence':seq,'anchor_id':case['anchor_id'],'method_id':method,
               'status':'invalid_input','execution_status':'NOT_RUN_INPUT_INVALID','failure_reason':REASON,
               'ATE_RMSE':None,'full_ATE_RMSE':None,'ATE_P95':None,'RPE_1s':None,'fault_window_RMSE':None,
               'delta_corrupted_minus_clean_ATE':None,'runtime':None,'fallback':0,'candidate_count':None,'accepted_segment_count':None,
               'used_range_count':None,'git_hash':lock['git_hash'],'experiment_fingerprint':lock['experiment_fingerprint'],
               'config_hash':sha(HERE/f'configs/sfuse/config_test_isas-walk{seq[-1]}.yaml') if sf else sha(HERE/'configs/backbone/FROZEN_BACKBONE.yaml'),
               'input_hash':sha(bag) if bag and bag.exists() else case['corrupted_input_hash'],
               'input_hash_semantics':'SF_SENSOR_BAG' if bag and bag.exists() else 'RAW_MEASUREMENT_PARENT_MANIFEST',
               'comparison_status':'INVALID_CANONICAL_NATIVE_INPUT','run':str(method_out) if method_out else None}
            if result:
                state=result.get('backend',result);r['original_status']=state.get('status',result.get('cell',{}).get('status'))
                r['runtime']=state.get('elapsed_seconds',state.get('runtime'))
                r['config_hash']=result['identity']['config_hash']
                if not sf and result.get('run'):
                    assert state.get('reason')=='invalid T07 UWB cache header'
                    r.update(execution_status='INPUT_PARSE_REJECTED_BEFORE_ESTIMATION',process_exit_code=state['exit_code'])
                elif sf and state.get('status')=='completed':
                    metric,points,fit,_=evaluate(seq,method,result,window=(case['start_time'],case['end_time']))
                    r.update(metric);r.update(case_id=cid,anchor_id=case['anchor_id'],execution_status='COMPLETED_SF_ONLY_NOT_PAIRED',
                                             comparison_status='INVALID_CANONICAL_NATIVE_INPUT',full_ATE_RMSE=metric['ATE_RMSE'])
                    clean=next(x for x in rows(HERE/'metrics/E1_clean_metrics.csv') if x['sequence']==seq and x['method_id']==method)
                    r['delta_corrupted_minus_clean_ATE']=metric['ATE_RMSE']-float(clean['ATE_RMSE']) if metric['ATE_RMSE'] is not None and clean['ATE_RMSE'] else None
                    detail[(cid,method)]=(points,fit)
                elif sf:r.update(execution_status='NOT_RUN_OFFLINE_INFERENCE_UNAVAILABLE',failure_reason='OFFLINE_INFERENCE_UNAVAILABLE_AFTER_'+REASON)
            elif sf and method_out and (method_out/'isolation.json').exists():
                r.update(status='failed',execution_status='INTERRUPTED_AUTHORIZED_INPUT_ERROR_STOP',
                         failure_reason='ABORTED_AFTER_GLOBAL_CANONICAL_NATIVE_INPUT_ERROR')
                if (method_out/'run_status.json').exists():r['runtime']=read(method_out/'run_status.json').get('runtime')
            runs.append(r)
            if method_out and method_out.exists():write(method_out/'formal_stop_identity.json',r)
        pairwise.append({'case_id':cid,'sequence':seq,'anchor_id':case['anchor_id'],'method_id':'PAIRED_RECOVER_MINUS_REJECT',
                         'git_hash':lock['git_hash'],'config_hash':sha(HERE/'configs/formal_lock.json'),
                         'experiment_fingerprint':lock['experiment_fingerprint'],'input_hash':case['corrupted_input_hash'],
                         'status':'invalid_input','failure_reason':REASON,
                         **{k:None for k in ['delta_FGO_RR_ATE','delta_FGO_RR_fault','delta_SF_RR_ATE','delta_SF_RR_fault','delta_SF_recover_native']}})
        detection.append({'case_id':cid,'sequence':seq,'anchor_id':case['anchor_id'],'status':'invalid_input','execution_status':'NOT_RUN',
                          'failure_reason':REASON,'method_id':'PL_BIDIRECTIONAL_CUSUM','git_hash':lock['git_hash'],
                          'config_hash':sha(HERE/'configs/detector/FROZEN_DETECTOR.yaml'),'experiment_fingerprint':lock['experiment_fingerprint'],
                          'input_hash':case['corrupted_input_hash'],
                          **{k:None for k in ['precision','recall','F1','event_detected','forward_alarm_delay','offline_onset_error',
                                             'offline_offset_error','false_positive_duration','candidate_count','segment_count']}})
        truth=set(case['affected_obs_ids'])
        for raw in rows(path.parent/input_doc['uwb_file']):
            range_rows.append({'case_id':cid,'sequence':seq,'obs_id':raw['obs_id'],'timestamp':raw['sensor_time_s'],
                               'anchor_id':raw['anchor_id'],'raw_range':raw['observed_range_m'],'status':'invalid_input',
                               'injected_component_m':1. if raw['obs_id'] in truth else 0.,'candidate':None,'segment_id':None,
                               'estimated_bias':None,'sigma_bias':None,'LCB':None,'recovery_accepted':None,'applied_correction':None,
                               'corrected_range':None,'bias_estimation_error':None,'injected_component_error':None,
                               'method_id':'OFFLINE_RECOVERY_NOT_RUN','git_hash':lock['git_hash'],
                               'config_hash':sha(HERE/'configs/methods/FROZEN_RECOVERY.yaml'),
                               'experiment_fingerprint':lock['experiment_fingerprint'],'input_hash':case['corrupted_input_hash']})
    for name,data in [('E2_canonical_runs',runs),('E2_canonical_pairwise',pairwise),('E2_canonical_detection',detection),('E2_canonical_range',range_rows)]:
        csv_output(HERE/'metrics'/(name+'.csv'),data)
    csv_output(HERE/'metrics/E2_canonical_segments.csv',[],['case_id','segment_id','estimated_bias','sigma_bias','LCB','accepted','applied_correction','bias_estimation_error','status'])
    csv_output(HERE/'audits/R6_input_format_audit.csv',audit)
    summary={'planned_cases':15,'completed_cases':0,'attempted_native_input_cases':len(attempted),'attempted_case_ids':attempted,
             'not_started_case_ids':[c['case_id'] for c in manifest['cases'] if c['case_id'] not in attempted],
             'interrupted_sf_case_ids':[r['case_id'] for r in runs if r['execution_status']=='INTERRUPTED_AUTHORIZED_INPUT_ERROR_STOP'],
             'native_solver_failure_cases':0,
             'invalid_input_cases':[c['case_id'] for c in manifest['cases']],'failed_cases':15,'fallback_cases':[],
             'no_candidate_cases':[],'classification_status':'NOT_EVALUATED_INVALID_INPUT','RECOVERY_CLAIM':'NOT_EVALUATED_INVALID_INPUT',
             'reason':REASON,'manifest_unchanged_sha256':original_hash,
             'Native_FGO':{'valid_pairs':0,'median_delta_FGO_RR_ATE':None,'median_delta_FGO_RR_fault':None,
                           'count_delta_FGO_RR_ATE_lt_0':None,'count_delta_FGO_RR_fault_lt_0':None,'worst_recovery_degradation':None},
             'SFUISE':{'valid_pairs':0,'median_delta_SF_RR_ATE':None,'median_delta_SF_RR_fault':None,
                       'count_delta_SF_RR_ATE_lt_0':None,'count_delta_SF_RR_fault_lt_0':None,'median_SF_RECOVER_minus_SF_NATIVE':None,
                       'worst_recovery_degradation':None,'completed_unpaired_SF_NATIVE_runs':sum(r['status']=='completed' for r in runs)},
             'Detector':{k:None for k in ['event_recall','median_precision','median_recall','median_F1','median_onset_error','median_offset_error']},
             'Bias':{'median_bias_estimation_error':None,'median_absolute_bias_estimation_error':None}}
    write(HERE/'audits/E2_GO_NOGO.json',summary)
    (HERE/'audits/E2_GO_NOGO.md').write_text('# E2 GO / NO-GO: INPUT VALIDITY NO-GO\n\n'
        'RECOVERY_CLAIM = NOT_EVALUATED_INVALID_INPUT\n\n'
        'This is NOT RESULT D / NOT_SUPPORTED: no valid paired recovery experiment ran. '
        'The host CSV serializer emitted CRLF, while the unchanged C++ cache parser requires an exact LF-delimited header. '
        'The agent incorrectly reused the metrics CSV writer for measurement-cache payloads and omitted an exact native-parser header preflight before freeze. '
        'This is an orchestration/input defect, not solver failure or evidence against recovery.\n\n'
        'Execution stopped under user early-stop condition 6. The canonical manifest and all 15 inputs are retained unchanged. '
        'No normalization, replacement hash, parameter change or scientific retry was made after freeze.\n\n'
        '```json\n'+__import__('json').dumps(summary,indent=2)+'\n```\n\n'
        'All unstarted/aborted rows are explicit in E2_canonical_runs.csv. Zero candidates is NOT inferred from parser failure. '
        'E2_canonical_segments.csv has a header and zero rows because no segments were produced. '
        'Table II and Figure 2 are labelled INPUT INVALID diagnostic/availability assets, not scientific claim evidence.\n')
    table=[]
    for m in METHODS:
        r=[x for x in runs if x['method_id']==m]
        table.append({'Method':m,'Planned':15,'Valid paired cases':0,'Completed unpaired runs':sum(x['status']=='completed' for x in r),
                      'Median paired ATE':None,'Median paired fault RMSE':None,'Status':'INPUT_INVALID_NOT_SCIENTIFIC_TABLE'})
    table_assets('TABLE_II_persistent_faults',table)
    invalid_figure(manifest,detail)
    write(HERE/'runs/R6/stage_status.json',{'status':'FAIL','formal_status':'invalid_input','execution_status':'AUTHORIZED_STOP_CONDITION_6',
                                         'planned_cases':15,'completed_cases':0,'RECOVERY_CLAIM':'NOT_EVALUATED_INVALID_INPUT','reason':REASON})
    write(HERE/'audits/R6_stop_process_audit.json',{'orchestrator_pid':996546,'orchestrator_exit_code':143,
          'commands':['kill -STOP 996546','kill -INT 1005805 1009201 1009423','kill -TERM 996546','kill -CONT 996546'],
          'reason':REASON,'remaining_experiment_processes':'NONE_OBSERVED','files_removed':0,'manifest_modified':False})
    assert sha(MANIFEST)==original_hash;verify()


def invalid_figure(manifest,detail):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    case=next(c for c in manifest['cases'] if c['case_id']==manifest['primary_case']);seq=case['sequence']
    lock=verify();parent=Path(lock['inputs'][int(seq[-1])-1]['manifest']);pm=read(parent)
    base={r['obs_id']:r for r in rows(parent.parent/pm['uwb_file'])};input_path=Path(case['input_manifest'])
    raw=rows(input_path.parent/read(input_path)['uwb_file']);origin=pm['recording_time_origin_s']
    timeline=[{'case_id':case['case_id'],'obs_id':r['obs_id'],'timestamp':r['sensor_time_s'],
               'clean_parent_range':base[r['obs_id']]['observed_range_m'],'injected_range':r['observed_range_m'],
               'corrected_range':None,'source_valid':r['source_valid'],
               'injection_interval':int(case['start_time']<=float(r['sensor_time_s'])<case['end_time']),
               'status':'INVALID_NATIVE_INPUT; correction NOT_RUN'} for r in raw if int(r['anchor_id'])==case['anchor_id']]
    trajectory=[];points,fit=detail.get((case['case_id'],'SF_NATIVE'),([],None))
    for p in points:
        xyz=np.array([p['est_'+a] for a in 'xyz'])@np.array(fit['rotation']).T+fit['translation']
        trajectory.append({'case_id':case['case_id'],'method_id':'SF_NATIVE_UNPAIRED_DIAGNOSTIC','timestamp':p['gt_time'],
                           **{a:float(xyz[i]) for i,a in enumerate('xyz')},**{'gt_'+a:p['gt_'+a] for a in 'xyz'}})
    csv_output(HERE/'figures/data/FIG_2_trajectory.csv',trajectory, None if trajectory else ['case_id','method_id','timestamp','x','y','z','gt_x','gt_y','gt_z'])
    csv_output(HERE/'figures/data/FIG_2_range_timeline.csv',timeline)
    csv_output(HERE/'figures/data/FIG_2_detector_timeline.csv',[],['timestamp','conditional_innovation','CUSUM_support','estimated_bias','LCB','admission','status'])
    fig,axes=plt.subplots(1,3,figsize=(15,4.5))
    if trajectory:
        axes[0].plot([p['x'] for p in trajectory],[p['y'] for p in trajectory],label='SF_NATIVE (unpaired diagnostic)')
        axes[0].plot([p['gt_x'] for p in trajectory],[p['gt_y'] for p in trajectory],'k--',label='GT')
        axes[0].legend(fontsize=7);axes[0].axis('equal')
    axes[0].set(title='(a) NO VALID PAIRED COMPARISON',xlabel='aligned x (m)',ylabel='aligned y (m)')
    valid=[r for r in timeline if r['source_valid']=='1']
    for col,label in [('clean_parent_range','clean parent'),('injected_range','injected')]:
        axes[1].plot([float(r['timestamp'])-origin for r in valid],[float(r[col]) for r in valid],label=label,lw=.8)
    axes[1].axvspan(case['start_time']-origin,case['end_time']-origin,alpha=.15,color='red',label='injection interval')
    axes[1].set(title='(b) corrected range NOT_RUN',xlabel='time from recording origin (s)',ylabel='range (m)');axes[1].legend(fontsize=7)
    axes[2].text(.5,.5,'Detector / recovery NOT_RUN\nNative cache parser rejected CRLF header\nNo zero-candidate or bias estimate inferred',ha='center',va='center',transform=axes[2].transAxes)
    axes[2].set_title('(c) unavailable');axes[2].axis('off')
    fig.suptitle(case['case_id']+' — INPUT INVALID; NOT A SCIENTIFIC RESULT',color='darkred')
    fig.tight_layout()
    for fmt in ('pdf','png'):
        (HERE/'figures'/fmt).mkdir(parents=True,exist_ok=True)
        fig.savefig(HERE/'figures'/fmt/('FIG_2_representative_case.'+fmt),dpi=180)
    plt.close(fig)
    write(HERE/'figures/data/FIG_2_selection.json',{'case_id':case['case_id'],'fallback':False,
          'status':'INVALID_INPUT_DIAGNOSTIC_ONLY','reason':'All canonical native inputs invalid; fallback would not restore validity.'})


if __name__=='__main__':main()
