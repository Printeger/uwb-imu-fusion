#!/usr/bin/env python3
"""Audit/report generation; never executes scientific estimators."""
import argparse
import json
import subprocess
from formal_native import HERE,ROOT,read,rows,write,save,sha,verify


def reports(blocked=None):
    lock=verify();r3=read(HERE/'runs/R3/stage_status.json')
    r4p=HERE/'runs/R4-transport-ready/stage_status.json';r4=read(r4p) if r4p.exists() else {'status':'NOT_RUN'}
    r5p=HERE/'runs/R5/stage_status.json';r5=read(r5p) if r5p.exists() else {'status':'NOT_RUN'}
    r6p=HERE/'runs/R6/stage_status.json';r6=read(r6p) if r6p.exists() else {'status':'NOT_RUN'}
    clean=rows(HERE/'metrics/E1_clean_metrics.csv') if (HERE/'metrics/E1_clean_metrics.csv').exists() else []
    for r in clean:
        r['candidate_count_semantics']='SHARED_OFFLINE_DETECTOR; base/robust do not apply support'
        if r['method_id'] not in ('FGO_RECOVER_FULL','SF_RECOVER'):r['accepted_segment_count']='0'
        if r['method_id'].startswith('SF'):
            if r.get('state_count'):r['trajectory_sample_count']=r['state_count'];r['state_count']=''
        if r['method_id'] in ('FGO_BASE','FGO_ROBUST'):r['runtime_scope']='NATIVE_ESTIMATOR_CELL'
        if not r['input_hash']:
            r['input_hash']=sha(lock['inputs'][int(r['sequence'][-1])-1]['manifest'])
            r['input_hash_semantics']='RAW_MEASUREMENT_PARENT; downstream treatment unavailable, SF not launched'
            r['downstream_input_hash']=''
    if clean:save(HERE/'metrics/E1_clean_metrics.csv',clean)
    e2=read(HERE/'audits/E2_GO_NOGO.json') if (HERE/'audits/E2_GO_NOGO.json').exists() else {}
    evidence=[]
    for pattern in ('metrics/E*.csv','metrics/clean*.csv','tables/csv/*.csv','tables/latex/*.tex',
                    'figures/data/FIG_2*','figures/pdf/FIG_2*','figures/png/FIG_2*','configs/**/FROZEN*','configs/formal_lock.json',
                    'configs/FORMAL_METRIC_PROTOCOL.json','manifests/injection_manifest_canonical*'):
        evidence.extend(HERE.glob(pattern))
    hashes={str(p.relative_to(HERE)):sha(p) for p in sorted(set(evidence)) if p.is_file()}
    write(HERE/'audits/R3_R6_artifact_hashes.json',hashes)
    command=['/usr/bin/python3','-m','unittest','discover','-s','experiments/icra2027','-p','test_*py']
    tests=subprocess.run(command,cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    (HERE/'audits/formal_engineering_tests.log').write_text(tests.stdout)
    write(HERE/'audits/formal_verification.json',{'protected_hashes':'PASS','fingerprint':lock['experiment_fingerprint'],
                                               'test_command':command,'test_exit_code':tests.returncode,'test_log':'formal_engineering_tests.log'})
    notes=('Input transport correction: first R4 attempt could not start bwrap because a system library symlink was redundantly mounted; '
           'no sensor replay occurred. The second attempt ran but lost early sensor messages (Walk1 ToA receipts 968/969/970 of 970) '
           'and failed the frozen clean equivalence tolerance. These runs and metrics remain under R4 / R4-runtime-fixed. '
           'The wrapper was corrected to verify subscriber connections, wait for playback subscribers at unchanged 1x speed, '
           'and record forwarded sensor messages. No SFUISE source, config, solver, sensor timestamps or measurement scheduling was changed. '
           'A transport-only header.seq comparison was corrected because roscpp assigns a publication counter; timestamp, frame, all ToA payload '
           'and IMU acceleration/gyro fields are compared exactly. The original failed probe and semantic review are both retained. '
           'Only transport-validated R4-transport-ready runs qualify for R5 reuse.')
    (HERE/'audits/sfuse_adapter_report.md').write_text('# R4 SFUISE adapter\n\nR4 '+r4['status']+'\n\n'+notes+'\n\n'+
        'Frozen clean tolerance: 0.001 m position, 0.001 rad orientation, 0.001 m aligned ATE. '
        'Walk1 alone has NO_CANDIDATES; Walk2/3 detected clean support, so no-candidate equivalence is not applicable there.\n\n'+
        'SF_REJECT deletes exactly frozen candidate IDs. SF_RECOVER subtracts final-use accepted lcb_fixed_full offsets only; '
        'nonaccepted measurements remain raw. Failed offline recovery stays failed and is never silently replaced by native input.\n\n'+
        'All variants use exactly the same official per-sequence ToA configuration. No GT/truth topics are exported. '
        'Explicit empty-root/network bwrap namespaces mount only sensor bags, official configs, binaries/runtime and private outputs. '
        'Every input is independently roundtrip checked for IMU, timestamps, nonrange fields, untreated ranges and retained IDs. '
        'ROS float32 rounding is bounded and logged in *_input.json.\n\n'+
        'Evidence: `../metrics/clean_sf_equivalence_transport_ready.csv`, `../exports/sfuse_inputs/`, '
        '`../runs/R4-transport-ready/` (isolation.json, transport_integrity*.json, commands, logs, statuses).\n')
    failure_rows=[r for r in clean if r['status'] in ('failed','invalid_input','fallback','no_candidates')]
    if (HERE/'metrics/E2_canonical_runs.csv').exists():
        failure_rows+= [r for r in rows(HERE/'metrics/E2_canonical_runs.csv') if r['status'] in ('failed','invalid_input','fallback','no_candidates')]
    native=read(HERE/'runs/R3/results.json');clean_detection={}
    for seq,r in native.items():
        state=read(__import__('pathlib').Path(r['AUX_PRODUCER']['run'])/'production_detector_status.json')
        clean_detection[seq]={'candidate_count':state['candidate_observation_count'],'segments':state['final_segment_count'],
                              'producer_status':r['AUX_PRODUCER']['backend'].get('status')}
    claim=e2.get('RECOVERY_CLAIM','NOT_EVALUATED')
    next_stage=('R4 input/adapter validity repair; do not advance to R5' if r4['status']!='PASS' else
                'R6.1–R6.3 input serialization/parser-only preflight and an explicitly authorized new versioned manifest; then restart R6, with unchanged scientific parameters' if r6['status']!='completed' else
                'R7 (robustness), only after separate user authorization; do not change the frozen configuration' if claim=='STRONG' else
                'R6 claim/evidence review; no automatic R7 because transferable stable recovery is not established')
    content=['# R3–R6 execution summary','',f'R3: {r3["status"]}; R4: {r4["status"]}; R5: {r5["status"]}; R6: {r6["status"]}.',
             '',f'RECOVERY_CLAIM = {claim}', '', '## Scope and identity','',
             'B0_CURRENT frozen. No B1/B2/B3, clean-backbone optimization, TDoA, new datasets, trajectory/model changes or tuning. '
             'Only ISAS Walk1/2/3; injected truth and GT evaluator-only. No commit/push.',
             '', 'Git: `'+lock['git_hash']+'`; dirty source snapshot is hash-locked, not represented as a clean checkout.',
             'Experiment fingerprint: `'+lock['experiment_fingerprint']+'`.',
             'Configuration/source/input hashes: `../configs/formal_lock.json`; delivery hashes: `R3_R6_artifact_hashes.json`.',
             '', '## R3 clean detector freeze','', '```json',json.dumps(clean_detection,indent=2),'```',
             'All detector passes completed. Walk2 Stage2 MAX_REFIT_ITERATIONS is a retained solver failure, not detector failure or permission to retune. '
             'Prior R0 verified Walk1 only; Walk2/3 are new clean exposure measurements. R3 PASS means configuration/valid detector signals frozen, '
             'not that every downstream solver succeeded or that clean no-harm is established.',
             '', '## R4 adapter and transport integrity','',notes,
             '', '## R5 clean summary','',
             'Fresh native R5 methods; fresh frozen R4 transport-validated SF runs reused only after exact offline-treatment equality checks. '
             'All methods use the unchanged R1/R2 common GT sets (227/292/313), association and scale-1 SE3 alignment. '
             'Raw-frame accuracy remains unavailable because frame/reference-point extrinsics are not independently established. '
             'SFUISE internal factor-use count is not exported: used_range_count remains NA with retained input count separately, '
             'not falsely described as internal graph use.',
             '```json',json.dumps([{k:r.get(k) for k in ('sequence','method_id','ATE_RMSE','ATE_P95','RPE_1s','candidate_count','status','failure_reason')} for r in clean],indent=2),'```',
             '', '## R6 canonical summary','', '```json',json.dumps(e2 or {'status':r6['status']},indent=2),'```',
             'If R6 is input-invalid, Table II is an availability ledger and Figure 2 is watermarked diagnostic-only, '
             'not a completed scientific comparison. Header-only segment/detector files mean NOT_RUN, not zero candidates. '
             'The agent reused a metrics CSV writer that emitted CRLF and failed to preflight the native exact-header parser before manifest freeze. '
             'This serialization error is not a solver failure or evidence against recovery. The frozen invalid inputs have not been rewritten.',
             'Controlled injected-component bias truth is not total real bias truth. This is a preregistered benchmark on exposed development recordings, '
             'not evidence of held-out generalization or physical NLOS. Local sigma is not a calibrated confidence guarantee.',
             '', '## All failed / fallback / no-candidate formal rows','', '```json',
             json.dumps([{k:r.get(k) for k in ('case_id','sequence','method_id','status','failure_reason','run')} for r in failure_rows],indent=2),'```',
             'Engineering startup/transport failures are additionally preserved in R3/clean1, R4 and R4-runtime-fixed; never mixed with valid scientific metrics.',
             '', '## Produced assets','']
    content+=['- `../'+p+'`' for p in hashes]
    content+=['','## Exact next recommendation','',next_stage+'.','No R7–R10 was started.']
    if blocked:content+=['','## Authorized early-stop condition','',blocked]
    (HERE/'audits/R3_R6_EXECUTION_SUMMARY.md').write_text('\n'.join(content)+'\n')
    return next_stage


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--blocked');a=p.parse_args();print(reports(a.blocked))
