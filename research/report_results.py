#!/usr/bin/env python3
"""Check the small retained research results and append to the existing report."""
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'results/fde_research_20261010'
REPORT = ROOT / 'docs/benchmark/fde_operability_20261010.json'

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def rows(name):
    with (RESULTS / name).open() as stream:
        return list(csv.DictReader(stream))

def scalar(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except ValueError:
        return value

report = json.loads(REPORT.read_text())
previous = report.get('research_continuation', {})
summary = {
    'status': 'PARTIAL_BLOCKED',
    'research_status': 'CONDITIONAL; isolated A-D prototypes and scoped E comparison executed',
    'production_contract_changed': False,
    'production_prior_qualification': 'UNQUALIFIED',
    'formal_eligible': False, 'publication_protected': False,
    'parent_head': 'd9e575aa3a6d659f66303fd27eeb7d3bdff1e6b6',
    'production_numerical_reference': 'a1ef121; later metadata/exception fixes retained',
    'research_reference': 'exact raw-sample/adjacent-episode model; not the old joint-order2 profile',
    'config_sha256': digest(ROOT / 'config/fde_imu_order1.yaml'),
    'source_sha256': {str(p.relative_to(ROOT)): digest(p) for p in sorted(list((ROOT / 'research').glob('*.cpp')) + list((ROOT / 'research').glob('*.hpp')))},
    'assumptions': {
        'contract': 'simulation-only/raw-range-episode/v1', 'horizon_seconds': .35,
        'event_partition': 'nominal or one (epoch,anchor) UWB event or one accel-x raw batch; categorical',
        'uwb_atom_probability': 1e-4, 'imu_atom_probability': 1e-5,
        'nominal_mass': .99433, 'hardware_probability_evidence': None,
        'raw_period_seconds': .005, 'episode_seconds': .05,
        'healthy_range_error_bound_m': 1e-12, 'lever_arm_m': [0, 0, 0],
        'motion': 'declared stationary generator',
        'imu_positive_additional_sensor': 'independent simulated velocity observation, weight sigma .02 m/s',
        'joint_probability': 'categorical intersection, no independence product and no persistent-source merging',
        'scope_limit': 'seven epochs; no evidence eligibility past horizon; no hardware qualification',
    },
    'verification': {'support_checks': 184, 'range_prior_consumer_checks': 23,
                     'prior_checks': 11, 'production_boundary_gtests': 6,
                     'prior_core_suite': 'previous 102 PASS reused; public numerical layer unchanged'},
    'flows': {}, 'performance': {}, 'risk_ledgers': {},
    'semantic_audit': previous.get('semantic_audit', {}),
}
flow_names = {'normal': 'accepted_normal.csv', 'uwb': 'accepted_uwb_final.csv', 'imu': 'accepted_imu.csv'}
for case, name in flow_names.items():
    flow = rows(name)
    assert len(flow) == 7
    for row in flow:
        assert all(row[key] == '1' for key in ['selected', 'commit', 'backend_updates', 'risk_closes', 'conditional_available'])
        assert row['formal_eligible'] == row['publication_protected'] == '0'
        assert math.isfinite(float(row['hpl'])) and math.isfinite(float(row['vpl']))
        assert float(row['position_error']) <= math.hypot(float(row['hpl']), float(row['vpl'])) + 1e-12
    fault = flow[2]
    assert (fault['alarm'], fault['exclusion']) == (('0', '0') if case == 'normal' else ('1', '1'))
    if case == 'imu':
        assert fault['bridge'] == '1'
    summary['flows'][case] = {
        'status': 'CONDITIONAL', 'selected_commits': 7,
        'fault_epoch': None if case == 'normal' else 3,
        'subsequent_selected_commits': None if case == 'normal' else 4,
        'fault_row': {k: scalar(v) for k, v in fault.items()},
        'first_fault_processing_ms': None if case == 'normal' else float(fault['core_ms']),
        'protected_recovery_latency_ms': None,
        'protected_latency_status': 'RIGHT_CENSORED',
        'source': str((RESULTS / name).relative_to(ROOT)), 'sha256': digest(RESULTS / name),
    }
    log = (RESULTS / ('accepted_uwb_final.log' if case == 'uwb' else f'accepted_{case}.log')).read_text()
    ledger = {}
    for line in log.splitlines():
        if ' status=' in line and '=' in line:
            term, rest = line.split('=', 1)
            value, remainder = rest.split(' status=', 1)
            state, source = remainder.split(' source=', 1)
            ledger[term] = {'value': float(value), 'status': state, 'source': source}
    assert ledger and sum(v['value'] for v in ledger.values()) <= 4e-5 + 1e-19
    summary['risk_ledgers'][case] = ledger

for case in ['normal', 'joint2']:
    branch = {}
    data = {}
    for variant in ['reference', 'optimized']:
        name = f'cost_{case}_{variant}_35.csv'
        data[variant] = rows(name)
        assert len(data[variant]) == 35
        assert [int(x['attempt']) for x in data[variant]] == list(range(1, 36))
        branch[variant] = {
            'attempts': 35,
            'median_ms': {key: statistics.median(float(row[key]) for row in data[variant]) for key in ['model_ms', 'evidence_ms', 'flat_ms', 'total_ms']},
            'maximum_total_ms': max(float(row['total_ms']) for row in data[variant]),
            'slowest_attempt': max(data[variant], key=lambda r: float(r['total_ms'])),
            'source': str((RESULTS / name).relative_to(ROOT)), 'sha256': digest(RESULTS / name),
        }
    keys = ['modes', 'hypotheses', 'response_calls', 'empty_support', 'mode_fingerprint', 'profile_sum', 'hpl', 'vpl', 'pl_status', 'formal_eligible']
    assert all(all(a[k] == b[k] for k in keys) for a, b in zip(data['reference'], data['optimized']))
    branch['median_total_gain_fraction'] = 1 - branch['optimized']['median_ms']['total_ms'] / branch['reference']['median_ms']['total_ms']
    branch['deadline_branch_flips'] = {str(limit): sum((float(a['total_ms']) <= limit) != (float(b['total_ms']) <= limit) for a, b in zip(data['reference'], data['optimized'])) for limit in [40, 50]}
    branch['strict_elapsed_deadline_comparison'] = 'FAIL' if any(branch['deadline_branch_flips'].values()) else 'PASS'
    branch['fixed_snapshot_numerical_comparison'] = 'PASS'
    branch['original_production_strict_comparator'] = 'NOT_RERUN; prior strict FAIL preserved'
    branch['protected_latency_ms'] = None
    branch['protected_latency_status'] = 'RIGHT_CENSORED'
    branch['terminal'] = 'original exhaustive Flat PL rejects unmonitorable hypothesis 62; every attempt retained'
    branch['scope'] = 'research construction+Evidence+exhaustive Flat+seal/index/validation/destruction; no claim of protected availability or isolated Flat gain'
    summary['performance'][case] = branch
    a = Path(f'/tmp/fde_research_{case}_reference.csv.semantics.txt')
    b = Path(f'/tmp/fde_research_{case}_optimized.csv.semantics.txt')
    if a.exists() and b.exists():
        assert a.read_bytes() == b.read_bytes()
        summary['semantic_audit'][case] = {'literal_numeric_fields_equal': True, 'bytes_compared': a.stat().st_size, 'sha256': digest(a)}
    assert summary['semantic_audit'].get(case, {}).get('literal_numeric_fields_equal')

summary['power'] = {
    'status': 'CONDITIONAL frozen linear Gaussian; not hardware p_md',
    'four_windows': 'stationary epoch3/12, rotating12, flat-geometry12',
    'source': 'results/fde_research_20261010/power.csv',
    'stationary_epoch12': [{k: scalar(v) for k, v in row.items()} for row in rows('power.csv') if row['case'] == 'stationary' and row['epoch'] == '12'],
    'shared_noise': 'actual Combined FD cross covariance norm ~4.335038e-9; generalized quadratic law checked; Cantelli lower miss bound ~.98244 in the two-factor fixture',
}
summary['stopping_conditions'] = [
    'Old UWB epoch7 risk 4.69e-5 > 4e-5 remains; accepted event counterexample not repeated.',
    'Original 50ms IMU fault lacks parity power and is absorbed mainly by velocity; no lowered detector or expanded step gate.',
    'Stationary gyro-z has no independent yaw observation; raw shared noise prevents inferring ordinary nc-chi-square calibration.',
    'Protected adoption of raw episode/joint covariance/odometer/range-set/shared-selector contract requires approved production scope and external sensor evidence.',
    'Only research input-construction optimization has measured net gain; Evidence/Flat algorithms and old disabled caches unchanged.',
]
summary['production_candidates'] = [
    'Readonly default fail-closed PriorEvidence interface, subject to normal review.',
    'Raw support/adjacent sample and malformed proof diagnostics; no automatic scope switch.',
    'Actual odometer and raw episode/noise/common-reference PL adoption need explicit contract approval and hardware evidence.',
]
report['research_continuation'] = summary
REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
print('VERIFIED 21 selected commits, two exclusions plus four followups each, 140 retained attempts; PARTIAL_BLOCKED')
