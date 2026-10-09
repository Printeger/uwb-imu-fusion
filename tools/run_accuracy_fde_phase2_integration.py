#!/usr/bin/env python3
"""One fixed, same-build phase2 reference/default integration comparison.

Writes compact identities and comparisons; never copies executables or datasets.
The original comparator remains authoritative, including any deadline FAIL.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
REFERENCE_FLAGS = (
    'HISTORY_UPDATES', 'GRAM_CERTIFICATES', 'PL_VALIDATION',
    'CLASSIFICATION_HASHES', 'FROZEN_VALIDATION',
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def metrics(directory):
    stages = rows(directory / 'diagnostic_stages.csv')
    attempts = rows(directory / 'diagnostic_attempts.csv')
    integrity = rows(directory / 'integrity.csv')
    timings = rows(directory / 'timing.csv')
    totals = {}
    for row in stages:
        if row['wall_ms']:
            totals[row['stage']] = totals.get(row['stage'], 0.) + float(row['wall_ms'])
    work_lines = '\n'.join(line for line in directory.with_suffix('.log').read_text().splitlines()
                           if line.startswith(('numerical_work ', 'history_root_work ')))
    work = dict(re.findall(r'(\w+)=(\d+)', work_lines))
    count = len(attempts)
    return {
        'attempts': count,
        'stage_mean_ms': {key: value / count for key, value in totals.items()},
        'arrival_to_publish_mean_ms': sum(float(r['wall_ms']) for r in timings
            if r['stage'] == 'arrival_to_publish') / count,
        'work': {key: int(value) for key, value in work.items()},
        'hypotheses_total': sum(int(r['hypothesis_count']) for r in attempts),
        'generated_actions_total': sum(int(r['generated_actions']) for r in attempts),
        'fault_gram_svd_total': sum(int(r['fault_gram_svd']) for r in attempts),
        'numerical_mismatches': sum(int(r['numerical_contract_mismatches']) for r in attempts),
        'protected_outputs': sum(r['publication_protected'] == '1' for r in integrity),
        'formal_eligible_outputs': sum(r['formal_eligible'] == '1' for r in integrity),
        'risk_valid_outputs': sum(r['risk_budget_valid'] == '1' for r in integrity),
        'deadline_missed_outputs': sum(r['deadline_missed'] == '1' for r in integrity),
        'committed_outputs': sum(r['batch_committed'] == '1' for r in integrity),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--epochs', type=int, default=35)
    parser.add_argument('--reference-model', choices=('phase2', 'b-correct'), default='phase2',
                        help='b-correct disables only the new continuous proof prototype')
    args = parser.parse_args()
    if args.epochs <= 0:
        parser.error('epochs must be positive')
    binary = args.binary.resolve()
    library = binary.parent.parent / 'libuwb_imu_pl.so'
    output = args.output.resolve()
    if output.exists():
        parser.error('output already exists; preserve previous evidence with a new path')
    dirty = subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no'],
                                    cwd=ROOT, text=True)
    if dirty:
        parser.error('commit tracked changes before measuring a source identity')
    identity = {
        'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'binary_sha256': sha(binary), 'library_sha256': sha(library),
        'runner_sha256': sha(Path(__file__)), 'tracked_status': dirty,
    }
    report = {'schema': 'uwb-imu-pl/phase2-combined-reference-default/v1',
              'identity': identity, 'repetitions': 1, 'runs': {},
              'scope': 'complete analysis; fixed native rotating synthetic stream, not live real-time acceptance',
              'caution': 'batch children nested in candidate wall; no phase-sum or P99/statistical guarantee',
              'frozen_memo_default': False, 'mode_cache_default': False}
    report['reference_model'] = args.reference_model
    for scenario, label, config_name in (
            ('noiseless', 'normal', 'fde_joint_order1.yaml'),
            ('joint_recovery', 'joint2', 'fde_joint_order2.yaml')):
        config = ROOT / 'config' / config_name
        config_hash = sha(config)
        item = {'sides': {}}
        for side in ('reference', 'optimized'):
            directory = output / side / label
            directory.parent.mkdir(parents=True, exist_ok=True)
            environment = {key: value for key, value in os.environ.items()
                           if not key.startswith('UWB_IMU_PL_')}
            environment.update(OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
                MKL_NUM_THREADS='1', LD_LIBRARY_PATH=str(library.parent),
                UWB_IMU_PL_SCENARIO=scenario, UWB_IMU_PL_MODE_RESPONSE_CACHE='0',
                UWB_IMU_PL_FROZEN_VALIDATION_REUSE='0')
            if args.reference_model == 'b-correct':
                environment.update(UWB_IMU_PL_ROOT_RESPONSE_REUSE='0',
                                   UWB_IMU_PL_BATCH_FROZEN_PROOFS='1')
                if side == 'reference':
                    environment['UWB_IMU_PL_REFERENCE_BATCH_FROZEN_PROOFS'] = '1'
            elif side == 'reference':
                environment.update({'UWB_IMU_PL_EXHAUSTIVE_' + flag: '1'
                                    for flag in REFERENCE_FLAGS})
            command = [str(binary), str(config), str(directory), str(args.epochs)]
            start = time.monotonic()
            with directory.with_suffix('.log').open('w') as stream:
                proc = subprocess.run(command, env=environment, stdout=stream,
                                      stderr=subprocess.STDOUT, cwd=ROOT)
            execution = dict(identity, command=command, config_sha256=config_hash,
                environment={key: value for key, value in environment.items()
                             if key.startswith(('UWB_IMU_PL_', 'OMP_', 'OPENBLAS_', 'MKL_', 'LD_LIBRARY'))},
                exit_code=proc.returncode, wall_s=time.monotonic() - start)
            directory.with_suffix('.execution.json').write_text(json.dumps(execution, indent=2) + '\n')
            if proc.returncode:
                raise RuntimeError(f'{label}/{side} failed; execution record retained')
            if sha(binary) != identity['binary_sha256'] or sha(library) != identity['library_sha256'] or sha(config) != config_hash:
                raise RuntimeError('build/config identity changed during measurement')
            item['sides'][side] = {'execution': execution, **metrics(directory)}
            print(label, side, proc.returncode, flush=True)
        comparison = output / (label + '.comparison.json')
        compared = subprocess.run(['python3', str(ROOT / 'tools/compare_p1_05_workers.py'),
            str(output / 'reference' / label), str(output / 'optimized' / label),
            '--before-workers', '1', '--after-workers', '1', '--performance-optimization',
            '--output', str(comparison)], check=False, cwd=ROOT)
        if compared.returncode not in (0, 1):
            raise RuntimeError('comparator failed to produce an equivalence decision')
        detail = json.loads(comparison.read_text())
        item['comparison_exit_code'] = compared.returncode
        item['comparison'] = {
            'status': detail['status'],
            'publication_binding_checks': detail['publication_binding_checks'],
            'file_statuses': {name: result['status'] for name, result in detail['files'].items()},
            'mismatches': {name: result['mismatches'] for name, result in detail['files'].items()
                           if result['mismatches']},
            'wall_packet_digest_contract': detail['wall_packet_digest_contract'],
        }
        item['comparison_sha256'] = sha(comparison)
        before = item['sides']['reference']['stage_mean_ms']['core_total']
        after = item['sides']['optimized']['stage_mean_ms']['core_total']
        item['core_reduction_percent'] = 100. * (before - after) / before
        report['runs'][label] = item
        # Preserve a completed scenario even if a later scenario fails.
        (output / 'integration_summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print('combined report', output / 'integration_summary.json', flush=True)
    return 0 if all(run['comparison']['status'] == 'PASS'
                    for run in report['runs'].values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
