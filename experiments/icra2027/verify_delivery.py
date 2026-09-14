#!/usr/bin/env python3
"""Verify source/input preservation, isolated runs, common GT and density accounting."""
import json
import os
from pathlib import Path
import subprocess
import tempfile

import yaml
from fingerprint import HERE, ROOT, sha, digest
from evaluate_clean import rows, read


def main():
    version = yaml.safe_load((HERE / 'VERSION.yaml').read_text())
    fingerprint = version.pop('fingerprint')
    assert digest(version) == fingerprint
    checked = {}
    for relative, expected in version['source_files'].items():
        assert sha(ROOT / relative) == expected, relative
    checked['frozen_source_files_unchanged'] = len(version['source_files'])
    for inp in version['inputs']:
        for path, expected in inp['files'].items():
            assert sha(path) == expected, path
    checked['frozen_input_files_unchanged'] = sum(len(i['files']) for i in version['inputs'])
    for path, expected in version['linked_libraries'].items():
        assert sha(path) == expected, path
    assert sha(version['runner']) == version['runner_sha256']
    checked['binary_and_libraries_unchanged'] = True
    # Reconstruct the index from the frozen commit, not the user's dirty index.
    with tempfile.TemporaryDirectory(prefix='icra-r0-patch-check-') as temp:
        env = dict(os.environ, GIT_INDEX_FILE=str(Path(temp) / 'index'))
        subprocess.check_call(['git', 'read-tree', version['git_commit']], cwd=ROOT, env=env)
        subprocess.check_call(['git', 'apply', '--check', '--cached', str(HERE / 'configs/source_worktree.patch')],
                              cwd=ROOT, env=env)
    checked['source_patch_applies_to_clean_commit'] = True
    checked['independent_clean_checkout_build'] = 'NOT_RUN'
    checked['runs'] = []
    for meta in sorted((HERE / 'runs').glob('*/run_metadata.json')):
        doc = read(meta)
        assert doc['fingerprint'] == fingerprint
        assert doc['exit_code'] == 0
        checked['runs'].append({'metadata': str(meta), **doc})
    for n in (1, 2, 3):
        data = [r for r in rows(HERE / 'metrics/backbone_parity.csv')
                if r['sequence'] == f'Walk{n}' and r['status'] == 'completed']
        assert len({r['common_gt_sha256'] for r in data}) == 1
        assert len({r['n_gt_samples'] for r in data}) == 1
        assert all(not r['ATE_RMSE_raw'] for r in data)
    checked['common_gt_sets_identical_and_raw_frame_unavailable'] = True
    for r in rows(HERE / 'audits/range_density_audit.csv'):
        counts = json.loads(r['drop_reasons'])
        assert sum(counts.values()) == int(r['raw_range_count'])
        assert r['used_range_count'] == r['planned_range_count']
        assert int(r['additional_valid_at_existing_state']) == 0
    checked['density_accounting_exact'] = True
    # Verify R0 repeated B0 Walk1 agrees with the fresh backbone batch exactly.
    trajectories = []
    for run in (HERE / 'runs').glob('R0-*/*/batch/runs/*'):
        if read(run / 'run_manifest.json').get('canonical_mode') == 'all_range':
            trajectories.append(sha(run / 'trajectory.tum'))
    b0 = next(r for r in rows(HERE / 'metrics/run_status.csv')
              if r['sequence'] == 'Walk1' and r['variant'] == 'B0_CURRENT')
    assert trajectories and all(t == b0['trajectory_sha256'] for t in trajectories)
    checked['walk1_r0_b0_trajectory_byte_identical'] = True
    checked['fingerprint'] = fingerprint
    checked['status'] = 'PASS_ENGINEERING_AND_PRESERVATION_NOT_CALIBRATION_ADMISSION'
    (HERE / 'audits/verification.json').write_text(json.dumps(checked, indent=2) + '\n')
    print(json.dumps({k: v for k, v in checked.items() if k != 'runs'}, indent=2))


if __name__ == '__main__':
    main()
