#!/usr/bin/env python3
"""Clean native FGO audit; GT is absent from the estimator filesystem."""
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

import yaml
from fingerprint import HERE, ROOT, WS, RUNNER, sha, digest

sys.path.insert(0, str(ROOT / 'experiments/scripts'))
sys.path.insert(0, str(ROOT / 'tools/paper'))


def worker(out):
    import run_experiments as scheduler
    import estimator_isolation as isolation
    version = yaml.safe_load((HERE / 'VERSION.yaml').read_text())
    approved = {v['manifest']: v['files'] for v in version['inputs']}

    def measurements(config):
        cfg = yaml.safe_load(Path(config).read_text())
        if cfg['dataset']['interface'] != 't07_cache' or cfg.get('sfuise'):
            raise ValueError('measurement-only cache required')
        expected = approved[cfg['dataset']['cache_manifest']]
        for path, expected_hash in expected.items():
            if sha(path) != expected_hash:
                raise ValueError('frozen clean input changed: ' + path)
        return [Path(config).resolve()] + [Path(p) for p in expected]

    isolation.measurement_files = measurements
    scheduler.runner_call = isolation.runner_call
    sys.argv = ['run_experiments.py', '--manifest', str(out / 'batch.yaml'),
                '--output-root', str(out / 'batch'), '--runner', str(RUNNER)]
    return scheduler.main()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--worker', type=Path)
    p.add_argument('--author-offsets-dev', action='store_true',
                   help='requires explicit user scope amendment; not independently calibrated')
    a = p.parse_args()
    if a.worker:
        return worker(a.worker)
    version = yaml.safe_load((HERE / 'VERSION.yaml').read_text())
    assert sha(RUNNER) == version['runner_sha256']
    variants = ['B1_CAL'] if a.author_offsets_dev else ['B0_CURRENT']
    label = variants[0]
    out = HERE / 'runs' / (label + '-' + datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
                          + '-' + uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    base = yaml.safe_load((HERE / 'configs/detector/starting_pipeline.yaml').read_text())
    units = []
    for n in (1, 2, 3):
        cfg = json.loads(json.dumps(base))
        cfg['dataset']['cache_manifest'] = version['inputs'][n-1]['manifest']
        cfg['nlos']['mode'] = 'disabled'
        cfg['nlos']['score_recoverability'] = False
        cfg['nlos']['final_inference_enabled'] = False
        if a.author_offsets_dev:
            official = yaml.safe_load((HERE / f'configs/sfuse/config_test_isas-walk{n}.yaml').read_text())
            anchors = sorted(a['id'] for a in cfg['anchors'])
            assert len(anchors) == len(official['toa_offset']) == 5
            cfg['calibration']['fixed_beta_by_link'] = {
                f'27956:{aid}': -offset for aid, offset in zip(anchors, official['toa_offset'])}
        config = HERE / 'configs/backbone' / f'Walk{n}_{label}.yaml'
        if config.exists():
            if yaml.safe_load(config.read_text()) != cfg:
                raise RuntimeError('refusing to replace a different frozen config')
        else:
            config.write_text(yaml.safe_dump(cfg, sort_keys=True))
        units.append({'run_unit_id': f'Walk{n}_{label}', 'recording_id': f'ISAS-Walk{n}',
                      'base_trajectory_id': f'ISAS-Walk{n}', 'seed': 911,
                      'prefix_identity': 'full_original_crop', 'config': str(config),
                      'cells': [{'mode': 'all_range', 'execution_type': 'BASELINE_TRAJECTORY',
                                 'path': 'DIRECT_COMMON_PREPARATION'}]})
    batch = {'schema': 'uifgo_t09_batch_v2', 'role': 'development',
             'parameter_provenance': 'ICRA_V6_R0_R2_CLEAN_DEVELOPMENT_NOT_FORMAL', 'run_units': units}
    (out / 'batch.yaml').write_text(yaml.safe_dump(batch, sort_keys=False))
    cmd = [sys.executable, str(Path(__file__).resolve()), '--worker', str(out)]
    doc = {'fingerprint': version['fingerprint'], 'variant': label, 'argv': cmd,
           'cwd': str(ROOT), 'config_hashes': {u['config']: sha(u['config']) for u in units},
           'calibration_provenance': 'AUTHOR_CONFIG_INDEPENDENCE_UNVERIFIED' if a.author_offsets_dev else 'UNCALIBRATED',
           'harness_sha256': sha(__file__), 'status': 'RUNNING'}
    path = out / 'run_metadata.json'
    path.write_text(json.dumps(doc, indent=2) + '\n')
    env = {k: v for k, v in os.environ.items() if not k.startswith('UIFGO_')}
    env['UIFGO_EXPERIMENT_WALL_LIMIT_S'] = '1800'
    start = time.monotonic()
    with (out / 'command.log').open('w') as log:
        code = subprocess.call(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    doc.update(exit_code=code, runtime_s=time.monotonic()-start,
               status='scheduler_completed' if code == 0 else 'scheduler_failed')
    path.write_text(json.dumps(doc, indent=2) + '\n')
    for run in (out / 'batch/runs').glob('*'):
        if run.is_dir():
            (run / 'experiment_fingerprint.json').write_text(json.dumps({
                'fingerprint': version['fingerprint'], 'variant_config_hash': doc['config_hashes'],
                'harness_sha256': doc['harness_sha256']}) + '\n')
    print(json.dumps(dict(doc, output=str(out))))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
