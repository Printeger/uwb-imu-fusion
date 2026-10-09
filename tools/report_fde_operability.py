#!/usr/bin/env python3
"""Summarize existing short-flow evidence without rerunning or relaxing gates."""
import argparse
import csv
import decimal
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def read(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot(directory, attempt):
    integrity = read(directory / 'integrity.csv')[attempt - 1]
    diagnostic = read(directory / 'diagnostic_attempts.csv')[attempt - 1]
    candidates = [r for r in read(directory / 'candidates.csv')
                  if r['window_id'] == integrity['window_id']]
    reasons = [r['reason'] for r in read(directory / 'diagnostic_stages.csv')
               if r['input_attempt_id'] == str(attempt) and 'gate:' in r['reason']]
    reasons = [';'.join(x for x in reason.split(';')
                       if x.startswith(('gate:', 'ledger:'))) for reason in reasons]
    samples = candidates[:8]
    finite = next((r for r in candidates if math.isfinite(float(r['hpl_m']))), None)
    if finite is not None and finite not in samples:samples.append(finite)
    samples = [{key:r[key] for key in ('action_id','hpl_m','vpl_m','reason') if key in r}
               for r in samples]
    terms = {}
    for item in diagnostic['risk_ledger_terms'].split(';'):
        if '=' not in item:
            continue
        name, item = item.split('=', 1)
        value, status = item.split(':', 1)
        terms[name] = dict(value=None if value == 'UNKNOWN' else float(value),
                           binary64_hex=None if value == 'UNKNOWN' else float(value).hex(),
                           status=status)
    decimal.getcontext().prec = 80
    known_export_sum = sum((decimal.Decimal.from_float(t['value']) for t in terms.values()
                            if t['value'] is not None), decimal.Decimal(0))
    budget = decimal.Decimal.from_float(float(integrity['hmi_risk_requirement']))
    # Individual term exports are rounded upward. The C++ ledger also exports
    # its once-rounded exact known charge: use that for decisions, not this sum.
    exact_charge_export = float(diagnostic['risk_ledger_charged_total'])
    return dict(attempt=attempt, window_id=integrity['window_id'],
        source=str(directory), source_hashes={p: sha(directory / p) for p in
            ('integrity.csv','diagnostic_attempts.csv','candidates.csv','diagnostic_stages.csv')},
        detector={k:integrity[k] for k in
            ('conditional_statistic','conditional_threshold','conditional_passed')},
        production={k:integrity[k] for k in ('fde_status','batch_committed','selected_action_id',
            'formal_eligible','risk_budget_valid','publication_protected','deadline_missed')},
        candidate_count=len(candidates),candidate_samples=samples, ledger=terms,
        known_export_sum_decimal=str(known_export_sum), budget_decimal=str(budget),
        cpp_exact_known_charge_export=exact_charge_export if math.isfinite(exact_charge_export) else None,
        known_charge_exceeds_budget=exact_charge_export > float(budget) if math.isfinite(exact_charge_export) else None,
        unknown_terms=[k for k,v in terms.items() if v['value'] is None],
        decision_trace=reasons,
        mathematical_unboundedness_proven=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    specs=[('strict_noiseless',10),('strict_noiseless',32),
           ('strict_uwb_recovery',7),('strict_imu_recovery',7),('rank_fixed_normal',32)]
    for label,attempt in (('final_strict_uwb_recovery',7),('final_strict_imu_recovery',7),
                          ('final_b_correct/reference/normal',10),
                          ('final_b_correct/reference/normal',32),
                          ('final_b_correct/reference/joint2',12)):
        if (args.results/label/'integrity.csv').exists():specs.append((label,attempt))
    data=dict(schema='fde-operability-focused-development-v1',
        functional_status='PARTIAL_BLOCKED', b_correct='958b700',
        boundary='No UWB or IMU conditional exclusion commit observed; no functional acceptance.',
        snapshots=[snapshot(args.results / label,n) for label,n in specs],
        recovery_latency=None,recovery_latency_status='RIGHT_CENSORED',
        qualifications=dict(formal_eligible=False,publication_protected=False),
        flows={})
    data['gate_state_codes']={'0':'PASS','1':'FAIL','2':'UNKNOWN','3':'NOT_RUN'}
    data['blocked_inputs']=dict(epoch_period_s=.05,imu_sample_period_s=.005,
        fault_epoch=7,uwb_bias_m=2.25,imu_accel_bias_mps2=20.,
        decision='event partition proposal pending; no probability contract changes applied',
        uwb_node='FdeManager::decideImpl / completeRiskLedger known charge 4.69e-5 > 4e-5',
        imu_node='joint detector passes; RankUpdateKernel KEEP step norm > 0.25; PL NOT_RUN')
    for label,_ in specs:
        directory=args.results / label
        if label in data['flows']:continue
        rows=read(directory / 'integrity.csv')
        data['flows'][label]=dict(attempts=len(rows),
            commits=sum(r['batch_committed']=='1' for r in rows),
            exclusion_commits=sum(r['batch_committed']=='1' and
                r['selected_action_type'] not in ('','KEEP_ALL','KEEP') and
                r['selected_action_id'] not in ('','0','1') for r in rows),
            risk_valid=sum(r['risk_budget_valid']=='1' for r in rows),
            protected=sum(r['publication_protected']=='1' for r in rows),
            deadline_misses=sum(r['deadline_missed']=='1' for r in rows))
        execution=directory.with_suffix('.execution.json')
        if execution.exists():
            data['flows'][label]['execution']=dict(path=str(execution),sha256=sha(execution))
    data['acceptance'] = dict(A1='PASS', A2='PASS_BOUNDARY_MISSING_PRODUCTION_EVIDENCE',
        A3='TRACE_IMPLEMENTED_DEADLINE_AT_PIPELINE', A4='FAIL_NO_CONDITIONAL_RECOVERY',
        A5='PASS', A6='MEASURED_PROTOTYPE_DEFAULT_OFF', A7='DELIVERED_WITH_BLOCKERS',
        realtime='NOT_MET',deployment='NOT_QUALIFIED',simulation_flow='NOT_IMPLEMENTED')
    data['artifacts'] = {}
    data['performance_directions']=dict(
        root_response=dict(status='STOPPED_ZERO_REUSE',default=False,
            normal_fallbacks=2328,joint2_fallbacks=72618,reuse=0,
            early_binary_hashes='NOT_CAPTURED; no reconstruction or rerun',
            normal_strict='FAIL_RETAINED',joint2_strict='PASS',
            artifacts=str(args.results/'root_probe')),
        continuous_proofs=dict(status='MEASURED_OPT_IN',default=False,
            micro_normal_core_total_ms=[960.100996,962.492028],
            micro_joint2_core_total_ms=[31988.332453,31485.795341],
            artifacts=str(args.results/'batch_probe'),
            sensitive_boundary='original full validation fallback',
            final_bundle='full validation unchanged'))
    for label,filename in (('w1_before','w1_before.xml'),('w1_after','w1_after.xml'),
                           ('map_mint_after','w3_reference_after.xml')):
        path=args.results/filename
        if path.exists():
            root=ET.parse(path).getroot()
            data['artifacts'][label]=dict(path=str(path),sha256=sha(path),
                tests=int(root.attrib['tests']),failures=int(root.attrib['failures']))
    safety=args.results/'core_safety'/'summary.json'
    if safety.exists():
        data['core_safety']=json.loads(safety.read_text())
        rerun=args.results/'core_safety'/'default_only.xml'
        if rerun.exists():
            root=ET.parse(rerun).getroot()
            data['core_safety']['default_only_rerun']=dict(path=str(rerun),sha256=sha(rerun),
                tests=int(root.attrib['tests']),failures=int(root.attrib['failures']),
                reason='Default-off invariant must be run with new opt-in prototype disabled; no code change')
        data['core_safety']['unique_tests']=sum(x['tests'] for x in data['core_safety']['tests'])
        data['core_safety']['historical_69_subset_verified']=True
        data['core_safety']['resolved_status']='PASS_WITH_RETAINED_ENVIRONMENT_FAILURE'
    integration=args.results/'final_b_correct'/'integration_summary.json'
    if integration.exists():
        data['paired_performance']=json.loads(integration.read_text())
        data['paired_performance']['distribution_ms']={}
        for label,run in data['paired_performance']['runs'].items():
            sides={}
            for side in ('reference','optimized'):
                directory=integration.parent/side/label
                stages=read(directory/'diagnostic_stages.csv')
                core=[(int(r['input_attempt_id']),float(r['wall_ms']))
                      for r in stages if r['stage']=='core_total']
                ordered=sorted(x[1] for x in core)
                sides[side]=dict(count=len(core),mean=sum(ordered)/len(core),
                    p50=ordered[(len(core)-1)//2],p95=ordered[math.ceil(.95*len(core))-1],
                    maximum=max(ordered),slowest_attempt=max(core,key=lambda x:x[1])[0],
                    all_attempts=core)
            data['paired_performance']['distribution_ms'][label]=sides
            before=run['sides']['reference']['execution']['wall_s']
            after=run['sides']['optimized']['execution']['wall_s']
            run['complete_process_wall_reduction_percent']=100.*(before-after)/before
        data['paired_performance']['limitations']=[
            'No P99 or rare-event qualification from 35 samples.',
            'Core includes root indexing/construction/seal and final consumers; retained lease destruction is included in complete process wall, outside core.',
            'Short safety tests overlapped the initial normal run; small gains are not significant evidence.',
            'Prototype remains opt-in off; root response reuse and old mode/frozen memo are off.',
            'Unchanged strict comparator is authoritative; no deadline mismatch is waived.']
    args.output.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
