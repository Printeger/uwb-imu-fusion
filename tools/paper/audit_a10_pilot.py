#!/usr/bin/env python3
"""Read-only audit and accounting for the closed A10 development allocation."""
import argparse
import csv
import json
from pathlib import Path
import re
import statistics

import yaml

import check_synthetic_input as checker
import generate_synthetic_input as gen


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return checker.rows(path) if path.is_file() else []


def audit(root):
    plan=read(root/'PILOT_MANIFEST.json')
    commands=read(root/'PILOT_EXECUTIONS.json')
    generation=read(root/'inputs/generation_manifest.json')
    published=read(root/'PUBLISHED_CACHES.json')
    assert len(commands)==len(plan['cells'])==9
    assert [r['run_id'] for r in commands]==[r['run_id'] for r in plan['cells']]
    assert gen.file_sha(root/'inputs/generation_manifest.json')==plan['generation_manifest_sha256']
    checker.verify(root/'inputs')  # rechecks restored truth and all actual payload bytes
    before=read(root/'BINARY_IDENTITY_BEFORE.json')
    after=read(root/'BINARY_IDENTITY_AFTER.json')
    assert before['files']==after['files']
    binary_by_real={r['realpath']:r['sha256'] for r in before['files']}
    # Actual A10 maps are the loading authority.
    summary=[]
    common={}
    signatures={}
    for cell,command in zip(plan['cells'],commands):
        cid=cell['run_id']; scenario=cell['scenario']; rd=root/'runs'/cid
        status=read(rd/'run_status.json')
        failure=read(rd/'discovery_failure_diagnostics.json')
        last=failure['last_conditional_lm']
        im=read(rd/'input_manifest.json')
        cm=read(root/'inputs/raw'/scenario/'input_manifest.json')
        config_path=root/'configs'/Path(cell['config']).name
        config=yaml.safe_load(config_path.read_text())
        assert gen.file_sha(config_path)==cell['config_sha256']==im['config_hash_sha256']
        assert cm['cache_id']==cell['input_cache_id']==im['cache_id']==im['source_hash_sha256']
        assert cm['base_recording_id']==im['recording_id']==generation['recording_id']
        assert im['cache_unit_conversion_applied'] is False
        assert not im['gt_read'] and not im['oracle_support_read'] and not im['gt_or_oracle_read']
        assert config['nlos']['discovery_max_outer_iterations']==500
        assert config['nlos']['max_refit_iterations']==200 and config['solver']['lm_max_iter']==50
        assert config['nlos']['discovery_short_min_count']==2 and config['nlos']['discovery_short_min_duration_s']==.01
        assert config['nlos']['boundary_epsilon_m']==1e-9
        assert config['calibration']['fixed_beta_by_link']=={f'0:{i+1}':b for i,b in enumerate(gen.BETA)}
        raw=rows(root/'inputs/raw'/scenario/'uwb_observations.csv')
        ledger=rows(rd/'observations.csv')
        assert len(raw)==len(ledger)==328
        assert [r['obs_id'] for r in raw]==[r['obs_id'] for r in ledger]
        for r,l in zip(raw,ledger):
            assert float(r['observed_range_m'])==float(l['raw_z_m'])
            assert float(r['sensor_time_s'])==float(l['raw_time'])
            assert float(l['nominal_sigma_m'])==.05
            assert l['valid']==l['planned']==l['strategy_used']=='1'
        current_common=read(rd/'common_preparation.json')
        comparison=(im['input_plan_hash_sha256'],im['calibration_hash_sha256'],current_common)
        if scenario in common:
            assert common[scenario]==comparison
        common[scenario]=comparison
        # P1/P2/P3 differ only in the five preregistered support fields.
        stripped=json.loads(json.dumps(config))
        for key in ('lambda_l1','lambda_tv','active_bias_min_m','change_point_min_m','merge_max_difference_m'):
            stripped['nlos'].pop(key)
        if scenario in signatures:
            assert signatures[scenario]==stripped
        signatures[scenario]=stripped
        assert status['exit_code']==command['exit_code']==1
        assert status['reason']=='CONDITIONAL_LM_STATIONARITY_NOT_REACHED'
        assert last['outer_iteration']==1 and last['iterate_call_count']==50
        assert last['lambda_trial_accounting_status']=='COMPLETE'
        assert last['lambda_trial_count']==last['accepted_update_count']+last['rejected_lambda_trial_count']
        assert failure['outer_trace_rows']==failure['chain_results_retained']==0
        assert not status['stage2_refit_run'] and not status['gate_or_fallback_run']
        assert not rows(rd/'discovery_iterations.csv') and not rows(rd/'admm_trace.csv')
        assert not read(rd/'partition.json')['segments']
        assert not (rd/'trajectory.tum').exists() and not (rd/'stage2_values.csv').exists()
        assert published[cid]['status']=='UNAVAILABLE_STAGE2_NOT_CONVERGED'
        assert command['external_wall_s']<120 and command['runner_maps_captured']
        assert command['truth_original_path_absent_polls']>0 and not command['truth_open_trace_matches']
        mapped={r['realpath']:r['sha256'] for r in command['mapped_elf_identity']}
        for real,sha in mapped.items():
            if real in binary_by_real:
                assert sha==binary_by_real[real]
        assert any('libuwb_imu_fgo.so' in p for p in mapped)
        assert any('libgtsam.so' in p for p in mapped)
        trace=(root/'commands'/('pilot_'+cid)/'file_access.trace').read_text()
        accessed=[]
        for line in trace.splitlines():
            if ('openat(' in line or 'open(' in line) and 'O_RDONLY' in line:
                match=re.search(r'"([^\"]+)"',line)
                if match:
                    accessed.append(match.group(1))
        assert not any('/evaluation/' in p or 'generation_manifest.json' in p or '_range_truth.csv' in p for p in accessed)
        assert any(p.endswith('/imu.csv') for p in accessed)
        assert any(p.endswith('/uwb_observations.csv') for p in accessed)
        resources=(root/'commands'/('pilot_'+cid)/'resources.txt').read_text()
        rss=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',resources).group(1))
        summary.append(dict(run_id=cid,scenario=scenario,support_id=cell['support_id'],seed=10101,role='development',
            exit_code=1,stage1='FAILED_CONDITIONAL_LM_STATIONARITY_NOT_REACHED',
            stage1_completed_outer=0,conditional_outer_attempt=1,conditional_calls=50,
            conditional_trials=last['lambda_trial_count'],conditional_accepted_updates=last['accepted_update_count'],
            final_conditional_scaled_gradient=last['last_qualification_stationarity']['max_scaled_gradient_objective'],
            stage2='NOT_RUN',stage3='NOT_RUN',stage4='NOT_RUN',
            segment_rows_emitted=0,group_rows_emitted=0,eligible_groups=None,unavailable_score_group_rows=0,
            group_count=None,segment_count=None,count_status='NOT_EVALUATED_STAGE1_FAILED',
            valid_score_count=0,scoring_unavailable_run_count=1,stage2_cache_id=None,
            stage2_cache_status='UNAVAILABLE_STAGE2_NOT_RUN',input_cache_id=im['cache_id'],
            input_plan_id=im['input_plan_hash_sha256'],common_preparation_id=current_common['common_preparation_id'],
            fixed_valid_observations=328,planned_observations=328,keyframes=41,
            stage1_seconds=status['stage1_seconds'],stage2_seconds=None,
            runner_wall_s=status['elapsed_seconds'],external_wall_s=command['external_wall_s'],peak_rss_kib=rss,
            resource_semantics='GNU time peak over runner/timeout/strace tree; wall includes tracing/audit overhead',
            truth_open_matches=0,actual_elf_identity='VERIFIED_AGAINST_BEFORE_AFTER'))
    fields=list(summary[0])
    with (root/'PILOT_SUMMARY.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(summary)
    result=dict(status='AUDIT_PASS_PILOT_ESTIMATION_FAILED',runs=summary,planned_processes=9,actual_processes=9,
                retries=0,timeouts=0,stage1_successes=0,stage2_successes=0,valid_scores=0,
                stage2_cache_count=0,raw_input_cache_count=3,held_out_validation='NOT_RUN',test='NOT_RUN',gate_lock='NOT_RUN',
                total_external_wall_s=sum(r['external_wall_s'] for r in summary),
                median_external_wall_s=statistics.median(r['external_wall_s'] for r in summary),
                max_external_wall_s=max(r['external_wall_s'] for r in summary),
                max_peak_rss_kib=max(r['peak_rss_kib'] for r in summary),
                support_comparison='NOT_IDENTIFIABLE_NO_CHAIN_UPDATE',
                valid_score_provenance_audit='NOT_APPLICABLE_NO_SCORES',
                calibration_semantics='EXACT_SYNTHETIC_ASSUMPTIONS; C++ reports EXPLICIT_COMPLETE_NOT_PROVENANCE_VERIFIED',
                orientation_config_note='Executed configs included ignored imu.use_imu_orientation_init=false; actual default use_imu_orientation_init=false plus all has_orientation=0 prevented orientation initialization. Delivered driver uses initialization.use_imu_orientation=false; no estimator rerun.',
                a08_graph_fd='15/18; three smallest-step failures remain UNKNOWN')
    gen.write_json(root/'PILOT_AUDIT.json',result)
    return {k:v for k,v in result.items() if k!='runs'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evidence-root',type=Path,required=True)
    args=p.parse_args()
    print(json.dumps(audit(args.evidence_root.resolve()),indent=2))


if __name__=='__main__':
    main()
