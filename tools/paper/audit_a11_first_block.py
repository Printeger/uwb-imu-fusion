#!/usr/bin/env python3
"""Read-only A11 evidence checks; never launches estimator or reads truth values."""
import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path


def read(path):
    return json.loads(path.read_text())


def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def same(a,b):
    return math.isfinite(float(a)) and abs(float(a)-float(b)) <= 1e-12+1e-12*abs(float(b))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-root',type=Path,required=True)
    args=parser.parse_args()
    root=args.evidence_root.resolve()
    executions=read(root/'EXECUTIONS.json')
    assert len(executions)==3
    assert [x['run_id'] for x in executions]==[f'P1_{s}_seed10101' for s in ('los','step','ramp')]
    assert read(root/'FROZEN_IDENTITIES.json')==read(root/'FROZEN_AFTER.json')
    before=read(root/'BINARY_BEFORE.json')['files']
    after=read(root/'BINARY_AFTER.json')['files']
    assert before==after
    binary_by_path={x['realpath']:x['sha256'] for x in before}
    summary=[]
    failures=[]
    details=[]
    for execution in executions:
        rid=execution['run_id'];scene=rid.split('_')[1]
        run=root/'runs'/rid;base=root/'frozen_a10/baseline'/rid
        assert execution['runner_maps_captured']
        assert not execution['forbidden_truth_open_lines']
        assert execution['external_wall_s']<120
        for elf in execution['mapped_elf_identity']:
            if elf['realpath'] in binary_by_path:
                assert elf['sha256']==binary_by_path[elf['realpath']]
        assert read(run/'common_preparation.json')==read(base/'common_preparation.json')
        inp=read(run/'input_manifest.json')
        assert inp==read(base/'input_manifest.json')
        for name in ('config_effective.yaml','observations.csv'):
            assert (run/name).read_bytes()==(base/name).read_bytes()
        failure=read(run/'discovery_failure_diagnostics.json')
        assert failure['outer_trace_rows']==0 and failure['chain_results_retained']==0
        assert failure['last_conditional_lm']['outer_iteration']==1
        assert not (run/'scores_decision.csv').exists()
        assert not (run/'stage2_cache_manifest.json').exists()
        calls=rows(run/'conditional_lm_calls.csv');stationarity=rows(run/'first_block_stationarity.csv')
        assert len(calls)==len(stationarity) and 50<=len(calls)<=200
        assert [int(x['call_index']) for x in calls]==list(range(1,len(calls)+1))
        c50=calls[49];s50=stationarity[49]
        baseline=read(base/'discovery_failure_diagnostics.json')['last_conditional_lm']
        comparisons={k:same(a,baseline[k]) for k,a in dict(initial_error=calls[0]['error_before'],
            previous_error=c50['error_before'],current_error=c50['error_after'],lambda_=c50['lambda_after']).items() if k!='lambda_'}
        comparisons['lambda']=same(c50['lambda_after'],baseline['lambda'])
        for key,field in [('rotation','max_pose_rotation_gradient_objective_per_rad'),
                          ('translation','max_pose_translation_gradient_objective_per_m'),
                          ('velocity','max_velocity_gradient_objective_per_mps'),
                          ('accel_bias','max_accel_bias_gradient_objective_per_mps2'),
                          ('gyro_bias','max_gyro_bias_gradient_objective_per_radps'),
                          ('max_scaled_gradient','max_scaled_gradient_objective'),
                          ('roundoff','roundoff_allowance_objective')]:
            comparisons[key]=same(s50[key],baseline['last_qualification_stationarity'][field])
        comparisons['iterations']=int(c50['optimizer_iterations_after'])==baseline['iterations']
        comparisons['trials']=int(c50['inner_iterations_after'])==baseline['lambda_trial_count']
        comparisons['accepted']=sum(int(x['accepted_state_update']) for x in calls[:50])==baseline['accepted_update_count']
        comparisons['rejected']=sum(int(x['rejected_lambda_trials_before_acceptance']) for x in calls[:50])==baseline['rejected_lambda_trial_count']
        assert all(comparisons.values()), comparisons
        accepted=rows(run/'first_block_actual_accepted_delta.csv')
        selected=[x for x in accepted if x['selected']=='1']
        assert [int(x['call']) for x in selected]==[49,50]
        assert all(x['accepted_retract_matches']=='1' and float(x['max_local_difference'])<=1e-12
                   and int(x['key_count'])==123 and int(x['dimension_count'])==615 for x in selected)
        gfd=rows(run/'conditional_lm_direction_finite_difference.csv')
        ffd=rows(run/'conditional_lm_direction_factor_derivatives.csv')
        coord=rows(run/'first_block_call50_coordinate_fd.csv')
        assert len(gfd)==6 and len(coord)==3
        factor_count=read(run/'conditional_lm_diagnostic_summary.json')['graph_factor_count']
        assert len(ffd)==2*factor_count*3
        for x in gfd+ffd:
            # Re-evaluate frozen numeric rule instead of trusting the agrees column.
            analytic=float(x['gradient_dot_unit_direction']);fd=float(x['central_derivative'])
            agrees=math.isfinite(fd) and abs(fd-analytic)<=5e-9+5e-3*abs(analytic)
            assert agrees == (x['agrees']=='1')
        for x in coord:
            analytic=float(x['analytic']);fd=float(x['central_fd'])
            assert (abs(fd-analytic)<=5e-9+5e-3*abs(analytic)) == (x['agrees']=='1')
        composition=[]
        for point in gfd:
            selected_factors=[x for x in ffd if x['call_index']==point['call_index'] and x['step']==point['step']]
            assert len(selected_factors)==factor_count
            assert sorted(int(x['factor_index']) for x in selected_factors)==list(range(factor_count))
            a_sum=math.fsum(float(x['gradient_dot_unit_direction']) for x in selected_factors)
            fd_sum=math.fsum(float(x['central_derivative']) for x in selected_factors)
            composition.append(dict(call=int(point['call_index']),step=float(point['step']),
                graph_analytic=float(point['gradient_dot_unit_direction']),factor_analytic_sum=a_sum,
                graph_fd=float(point['central_derivative']),factor_fd_sum=fd_sum))
        passed=all(x['agrees']=='1' for x in gfd+ffd+coord)
        budget=read(run/'first_block_budget.json')
        assert budget['derivative_gate_passed']==passed
        if not passed:
            assert len(calls)==50 and budget['continuation_calls']==0
        counts=Counter(x['dynamic_type'] for x in ffd if x['agrees']=='0')
        for x in ffd:
            if x['agrees']=='0': failures.append(dict(scene=scene,**x))
        rss=next(float(line.split(':')[-1]) for line in (root/'commands'/('run_'+rid)/'resources.txt').read_text().splitlines()
                 if 'Maximum resident set size' in line)
        summary.append(dict(scene=scene,exit_code=execution['exit_code'],calls=len(calls),
            call50_reproduced=all(comparisons.values()),error50=float(c50['error_after']),gradient50=float(s50['max_scaled_gradient']),
            graph_fd_pass=sum(x['agrees']=='1' for x in gfd),graph_fd_total=len(gfd),
            factor_fd_pass=sum(x['agrees']=='1' for x in ffd),factor_fd_total=len(ffd),
            coordinate_fd_pass=sum(x['agrees']=='1' for x in coord),coordinate_fd_total=len(coord),
            derivative_gate_passed=passed,continuation_calls=budget['continuation_calls'],
            final_original_stationary=stationarity[-1]['stationary']=='1',
            final_max_gradient=float(stationarity[-1]['max_scaled_gradient']),
            stage1_completed_outer=0,chain_updates=0,stage2='NOT_RUN',gate='NOT_RUN',
            segment_group_eligible='NOT_EVALUATED',raw_cache_id=inp['cache_id'],
            external_wall_s=execution['external_wall_s'],max_rss_kib=rss))
        details.append(dict(scene=scene,call50_comparisons=comparisons,composition=composition,failed_fd_by_factor_type=dict(counts),
                            continuation=budget,last_conditional_lm=failure['last_conditional_lm']))
    for name,data in [('SUMMARY.csv',summary),('FACTOR_FD_FAILURES.csv',failures)]:
        with (root/name).open('w') as f:
            fields=list(data[0]) if data else ['scene']+list(ffd[0])
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(data)
    result=dict(schema='t10_a11_readonly_audit_v1',status='PASS_EVIDENCE_ACCOUNTING_NOT_ESTIMATOR_SUCCESS',
                details=details,summary=summary,estimator_processes=3,
                measured_external_wall_s=sum(x['external_wall_s'] for x in summary),
                truth_isolation='NO_TRUTH_FILE_OPENS_AND_FROZEN_HASHES_UNCHANGED',
                held_out_validation_test='NOT_RUN',a08_historical_graph_fd='15/18_LIMITATION_RETAINED')
    (root/'AUDIT.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
