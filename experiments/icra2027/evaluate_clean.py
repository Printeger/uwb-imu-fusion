#!/usr/bin/env python3
"""Post-run common-GT frame/density audit. Never called inside an estimator."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
import yaml
from fingerprint import HERE, ROOT, WS, sha, digest

sys.path.insert(0, str(ROOT / 'tools/paper'))
import evaluate_runs as ev

TOLERANCE = 0.02


def read(path):
    return json.loads(Path(path).read_text())


def rows(path):
    with Path(path).open(newline='') as f:
        return list(csv.DictReader(f))


def save(path, data):
    keys = list(dict.fromkeys(k for row in data for k in row))
    with Path(path).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(data)


def associate(trajectory, gt, lo, hi):
    """Existing nearest-GT policy, then one closest estimate per GT timestamp."""
    matched, _ = ev.match_trajectories([e for e in trajectory if lo <= e[0] <= hi],
                                      [g for g in gt if lo <= g[0] <= hi],
                                      {'policy': 'nearest_within_tolerance', 'tolerance_s': TOLERANCE})
    result = {}
    for e, g in matched:
        old = result.get(g[0])
        if old is None or (abs(e[0]-g[0]), e[0]) < (abs(old[0][0]-g[0]), old[0][0]):
            result[g[0]] = (e, g)
    return result


def score(mapping, common):
    if len(common) < 3:
        raise ValueError('fewer than three common GT samples')
    pairs = [mapping[t] for t in common]
    p = np.array([e[1] for e, _ in pairs])
    g = np.array([g[1] for _, g in pairs])
    x, y = p-p.mean(axis=0), g-g.mean(axis=0)
    if np.linalg.matrix_rank(x) < 2:
        raise ValueError('degenerate SE3 alignment')
    u, _, vt = np.linalg.svd(x.T @ y)
    d = np.eye(3)
    d[2, 2] = np.linalg.det(vt.T @ u.T)
    r = vt.T @ d @ u.T
    t = g.mean(axis=0) - r @ p.mean(axis=0)
    aligned = p @ r.T + t
    raw = np.linalg.norm(p-g, axis=1)
    error = np.linalg.norm(aligned-g, axis=1)
    translated = np.linalg.norm(x-y, axis=1)
    rpe = []
    for i, ti in enumerate(common):
        j = int(np.searchsorted(common, ti + 1.0))
        if j < len(common) and abs(common[j]-ti-1.0) <= TOLERANCE:
            rpe.append(ev.se3_relative_pose_error(pairs[i][0], pairs[j][0], pairs[i][1], pairs[j][1])[0])
    metrics = {'n_gt_samples': len(common), 'common_gt_sha256': digest(common),
               'unaligned_coordinate_RMSE_m': float(np.sqrt(np.mean(raw**2))),
               'translation_only_RMSE_m': float(np.sqrt(np.mean(translated**2))),
               'ATE_RMSE_raw': None, 'raw_frame_status': 'UNAVAILABLE_FRAME_AND_POINT_PROVENANCE',
               'ATE_RMSE_aligned': float(np.sqrt(np.mean(error**2))),
               'ATE_median': float(np.median(error)), 'ATE_P95': float(np.percentile(error, 95)),
               'RPE_1s': float(np.sqrt(np.mean(np.square(rpe)))) if rpe else None,
               'rpe_pairs': len(rpe), 'alignment_mode': 'SE3_SCALE_1_PER_TRAJECTORY_COMMON_GT',
               'reference_point_status': 'TRACKER_BODY_COLOCATION_PROXY',
               'fit_translation_norm_m': float(np.linalg.norm(t)),
               'fit_rotation_deg': float(np.degrees(np.arccos(np.clip((np.trace(r)-1)/2, -1, 1)))),
               'mean_time_delta_s': float(np.mean([abs(e[0]-g[0]) for e, g in pairs])),
               'evaluation_start': common[0], 'evaluation_end': common[-1]}
    points = [{'gt_time': gt[0], 'estimate_time': e[0], 'raw_error_m': float(raw[i]),
               'aligned_error_m': float(error[i]), **{f'est_{axis}': float(p[i,k]) for k,axis in enumerate('xyz')},
               **{f'gt_{axis}': float(g[i,k]) for k,axis in enumerate('xyz')}}
              for i, (e, gt) in enumerate(pairs)]
    return metrics, points, {'rotation': r.tolist(), 'translation': t.tolist(),
                            'use': 'EVALUATOR_ONLY_NOT_CALIBRATION'}


def density(run, sequence, variant):
    obs = rows(run / 'observations.csv')
    factors = rows(run / 'baseline_factor_audit.csv')
    # Audit the actual all-range final mask, not just the input plan.
    used = {f['obs_id'] for f in factors if f['final_use'] == '1'}
    assert len(used) == sum(f['final_use'] == '1' for f in factors)
    assert used == {o['obs_id'] for o in obs if o['strategy_used'] == '1'}
    planned_times = {float(o['raw_time']) for o in obs if o['planned'] == '1'}
    counts, ledger = [], []
    for o in obs:
        reason = (o['validity_reason'] if o['valid'] != '1' else
                  'KEYFRAME_STEP_4' if o['planned'] != '1' else
                  'FINAL_USED' if o['obs_id'] in used else 'FINAL_NOT_USED')
        ledger.append({'sequence': sequence, 'variant': variant, 'obs_id': o['obs_id'],
                       'anchor_id': o['anchor_id'], 'raw_time': o['raw_time'],
                       'valid': o['valid'], 'planned': o['planned'], 'final_used': int(o['obs_id'] in used),
                       'reason': reason,
                       'exact_existing_state_available': int(float(o['raw_time']) in planned_times)})
    for anchor in ['ALL'] + sorted({o['anchor_id'] for o in obs}, key=int):
        selected = [o for o in ledger if anchor == 'ALL' or o['anchor_id'] == anchor]
        reason_counts = {r: sum(o['reason'] == r for o in selected) for r in sorted({o['reason'] for o in selected})}
        counts.append({'sequence': sequence, 'variant': variant, 'anchor_id': anchor,
                       'raw_range_count': len(selected), 'valid_range_count': sum(o['valid'] == '1' for o in selected),
                       'planned_range_count': sum(o['planned'] == '1' for o in selected),
                       'used_range_count': sum(o['final_used'] for o in selected),
                       'additional_valid_at_existing_state': sum(o['valid'] == '1' and o['planned'] != '1'
                                                                and o['exact_existing_state_available'] for o in selected),
                       'drop_reasons': json.dumps(reason_counts, sort_keys=True),
                       'state_count': len(planned_times),
                       'state_interval_median_s': float(np.median(np.diff(sorted(planned_times))))})
    return counts, ledger


def sources(batch):
    doc = read(batch / 'batch/batch_manifest.json')
    result = {}
    for cell in doc['cells']:
        n = int(cell['run_unit_id'][4])
        result[n] = Path(cell['run_directory'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--b0', type=Path, required=True)
    parser.add_argument('--b1', type=Path)
    parser.add_argument('--sfuise', type=Path, default=ROOT / 'experiments/results/sfuise-toa-20260912T104509Z-8543708554f5')
    a = parser.parse_args()
    native = {'B0_CURRENT': sources(a.b0)}
    if a.b1:
        native['B1_CAL'] = sources(a.b1)
    sf = {int(r['sequence'][-1]): r for r in read(a.sfuise / 'batch_manifest.json')['runs']}
    frame, parity, density_rows, ledger, transforms, status_rows = [], [], [], [], {}, []
    for n in (1, 2, 3):
        sequence = f'Walk{n}'
        manifest = WS / f'res/nlos_injection_20260911_01/inputs/sfuise_walk{n}_normal_clean/input_manifest.json'
        uwb = rows(manifest.parent / 'uwb_observations.csv')
        lo, hi = min(float(r['sensor_time_s']) for r in uwb), max(float(r['sensor_time_s']) for r in uwb)
        gt = WS / f'res/ie0911_step2_truth_20260911_01/sfuise_walk{n}/ground_truth.tum'
        paths = {v: runs[n] / 'trajectory.tum' for v, runs in native.items()}
        paths['SF_NATIVE'] = Path(sf[n]['trajectory_path'])
        assert sf[n]['raw_bag_sha256'] == read(manifest)['base_source_sha256'].split(':')[-1]
        maps, states, diagnostics = {}, {}, {}
        for variant, path in paths.items():
            state = read(path.parent / 'run_status.json')
            states[variant] = state
            valid = state.get('status') in {'SUCCESS', 'OK', 'NO_CANDIDATES'}
            if valid and path.is_file():
                maps[variant] = associate(ev.load_tum(path), ev.load_tum(gt), lo, hi)
            status_rows.append({'sequence': sequence, 'variant': variant, 'source_status': state.get('status'),
                                'status': 'completed' if variant in maps else 'failed',
                                'failure_reason': state.get('reason', state.get('failure_reason', '')),
                                'trajectory_path': str(path), 'trajectory_sha256': sha(path) if path.exists() else '',
                                'gt_sha256': sha(gt), 'gt_input_to_estimator': False,
                                'source_run_reused': variant == 'SF_NATIVE'})
            if variant != 'SF_NATIVE' and (path.parent / 'baseline_factor_audit.csv').exists():
                counts, obs = density(path.parent, sequence, variant)
                density_rows.extend(counts)
                ledger.extend(obs)
                diagnostics[variant] = counts[0]
        common = sorted(set.intersection(*(set(m) for m in maps.values()))) if maps else []
        transforms[sequence] = {'common_gt_times': common, 'fits': {}}
        point_rows = []
        for variant, mapping in maps.items():
            metrics, points, fit = score(mapping, common)
            record = {'sequence': sequence, 'variant': variant, 'status': 'completed', **metrics}
            record.update(raw_range_count=len(uwb),
                          input_evaluation_start=lo, input_evaluation_end=hi,
                          used_range_count=diagnostics.get(variant, {}).get('used_range_count'),
                          state_count=diagnostics.get(variant, {}).get('state_count'),
                          runtime_s=states[variant].get('elapsed_seconds', states[variant].get('wall_time_s')),
                          runtime_scope='END_TO_END_1X_PLAYBACK' if variant == 'SF_NATIVE' else 'NATIVE_ESTIMATOR',
                          trajectory_sha256=sha(paths[variant]))
            parity.append(record)
            if variant in {'B0_CURRENT', 'SF_NATIVE'}:
                frame.append(record)
            point_rows.extend(dict(row, variant=variant) for row in points)
            transforms[sequence]['fits'][variant] = fit
        for variant in ['B0_CURRENT', 'B1_CAL', 'B2_CAL_DENSE', 'B3_HIGHER_RATE']:
            if variant not in maps:
                reason = ('DEFERRED_USER_STOP_AFTER_R2' if variant == 'B3_HIGHER_RATE' else
                          'NO_ADDITIONAL_EXACT_TIME_RANGES_AT_FIXED_STATES' if variant == 'B2_CAL_DENSE' else
                          'CALIBRATION_INDEPENDENCE_UNVERIFIED' if variant not in paths else 'ESTIMATION_FAILED')
                parity.append({'sequence': sequence, 'variant': variant, 'status': 'NOT_RUN' if variant not in paths else 'failed',
                               'failure_reason': reason, 'raw_range_count': len(uwb)})
                if variant not in paths:
                    status_rows.append({'sequence': sequence, 'variant': variant, 'status': 'NOT_RUN',
                                        'failure_reason': reason, 'gt_input_to_estimator': False})
        save(HERE / f'figures/data/frame_audit_walk{n}.csv', point_rows)
    save(HERE / 'metrics/run_status.csv', status_rows)
    save(HERE / 'metrics/frame_audit_metrics.csv', frame)
    save(HERE / 'metrics/backbone_parity.csv', parity)
    save(HERE / 'tables/csv/backbone_parity.csv', parity)
    save(HERE / 'figures/data/backbone_parity.csv', parity)
    save(HERE / 'audits/range_density_audit.csv', density_rows)
    save(HERE / 'audits/range_density_observations.csv', ledger)
    (HERE / 'audits/frame_fits_evaluator_only.json').write_text(json.dumps(transforms, indent=2) + '\n')
    print(json.dumps({'frame': frame, 'parity': parity}, indent=2))


if __name__ == '__main__':
    main()
