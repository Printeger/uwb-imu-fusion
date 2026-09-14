#!/usr/bin/env python3
"""Freeze the actual dirty source/build/config identity; print it deterministically."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
WS = ROOT.parents[1]
RUNNER = WS / 'devel/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner'
SF = ROOT / 'experiments/compare_algorithm/SFUISE'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def digest(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def command(argv, cwd=ROOT):
    return subprocess.check_output(argv, cwd=cwd, text=True).strip()


def freeze():
    if (HERE / 'VERSION.yaml').exists():
        raise RuntimeError('VERSION already exists; do not overwrite a freeze')
    for name in ('manifests', 'configs/backbone', 'configs/detector', 'configs/methods',
                 'configs/sfuse', 'audits', 'runs', 'metrics', 'exports/corrected_ranges',
                 'exports/rejected_ranges', 'exports/sfuse_inputs', 'tables/csv', 'tables/latex',
                 'figures/data', 'figures/pdf', 'figures/png', 'logs', 'paper_assets'):
        (HERE / name).mkdir(parents=True, exist_ok=True)
    source = {}
    for directory in ('src', 'include', 'tools/paper', 'experiments/scripts', 'config'):
        for p in sorted((ROOT / directory).rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts:
                source[str(p.relative_to(ROOT))] = sha(p)
    source['CMakeLists.txt'] = sha(ROOT / 'CMakeLists.txt')
    diff = subprocess.check_output(['git', 'diff', 'HEAD', '--', 'src', 'include', 'test',
                                    'tools', 'config', 'CMakeLists.txt'], cwd=ROOT)
    (HERE / 'configs/source_worktree.patch').write_bytes(diff)
    cfg = ROOT / 'config/paper/ie0911/sfuise_walk1_pl_bidirectional_clean.yaml'
    shutil.copy2(cfg, HERE / 'configs/detector/starting_pipeline.yaml')
    cfgs = {}
    inputs = []
    for n in (1, 2, 3):
        sf = SF / f'sfuise/config/config_test_isas-walk{n}.yaml'
        shutil.copy2(sf, HERE / 'configs/sfuse' / sf.name)
        cfgs[str(sf.relative_to(ROOT))] = sha(sf)
        manifest = WS / f'res/nlos_injection_20260911_01/inputs/sfuise_walk{n}_normal_clean/input_manifest.json'
        data = json.loads(manifest.read_text())
        files = [manifest, manifest.parent / data['uwb_file'], manifest.parent / data['imu_file']]
        inputs.append({'sequence': f'Walk{n}', 'role': 'development',
                       'manifest': str(manifest), 'files': {str(p): sha(p) for p in files},
                       'raw_bag_sha256': data['base_source_sha256']})
    (HERE / 'manifests/dataset_manifest.yaml').write_text(yaml.safe_dump(inputs, sort_keys=True))
    ldd = command(['ldd', str(RUNNER)])
    deps = {}
    for line in ldd.splitlines():
        if '=> /' in line:
            p = Path(line.split('=>', 1)[1].strip().split()[0])
            deps[str(p)] = sha(p)
    cache = WS / 'build/uwb_imu_fgo/CMakeCache.txt'
    build = [line for line in cache.read_text().splitlines()
             if line.startswith(('CMAKE_BUILD_TYPE:', 'CMAKE_CXX_COMPILER:', 'GTSAM_DIR:', 'Eigen3_DIR:'))]
    doc = {'schema': 'icra2027_provenance_v1', 'git_commit': command(['git', 'rev-parse', 'HEAD']),
           'git_branch': command(['git', 'branch', '--show-current']),
           'source_files': source, 'source_worktree_patch_sha256': hashlib.sha256(diff).hexdigest(),
           'initial_worktree_status': command(['git', 'status', '--short']),
           'build': build, 'compiler': command(['/usr/bin/c++', '--version']),
           'cpu': command(['lscpu']), 'python': platform.python_version(),
           'thread_environment': {k: os.environ.get(k, 'UNSET') for k in
                                  ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')},
           'runner': str(RUNNER), 'runner_sha256': sha(RUNNER), 'linked_libraries': deps,
           'evaluator_sha256': sha(ROOT / 'tools/paper/evaluate_runs.py'),
           'starting_config_sha256': sha(cfg), 'sfuise_commit': command(['git', 'rev-parse', 'HEAD'], SF),
           'sfuise_configs': cfgs, 'inputs': inputs,
           'missing_provenance': ['independent tracker-to-IMU transform', 'independent UWB-to-VIVE transform',
                                  'official ToA offset calibration procedure/recording']}
    doc['fingerprint'] = digest(doc)
    (HERE / 'VERSION.yaml').write_text(yaml.safe_dump(doc, sort_keys=True))
    print(doc['fingerprint'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--freeze', action='store_true')
    a = p.parse_args()
    if a.freeze:
        freeze()
    else:
        doc = yaml.safe_load((HERE / 'VERSION.yaml').read_text())
        fingerprint = doc.pop('fingerprint')
        if digest(doc) != fingerprint:
            raise RuntimeError('VERSION fingerprint mismatch')
        print(fingerprint)


if __name__ == '__main__':
    main()
