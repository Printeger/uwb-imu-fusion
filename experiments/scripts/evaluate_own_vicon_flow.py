#!/usr/bin/env python3
"""Independent evaluator for the single own_vicon flow run."""
from pathlib import Path
import math

import numpy as np
import pandas as pd

import evaluate_recover_vs_reject as e
import rr_evaluate as common
import run_recover_vs_reject as r
from run_own_vicon_flow import BAG, GT_TOPIC, NAME


def audit_recording(out, lock, private):
    import rosbag
    origin = r.read(out / NAME / 'preparation.json')['time_origin_epoch_s']
    gt_rows = []
    with rosbag.Bag(str(BAG)) as bag:
        for _, msg, _ in bag.read_messages(topics=[GT_TOPIC]):
            p = msg.pose.position
            gt_rows.append((msg.header.stamp.to_sec() - origin, p.x, p.y, p.z))
    gt = pd.DataFrame(gt_rows, columns=['time_s','x','y','z']).drop_duplicates('time_s').sort_values('time_s')
    uwb = pd.read_csv(out / NAME / 'input/uwb_observations.csv')
    anchors = {int(key):np.asarray(value, float) for key, value in lock['geometry']['anchors'].items()}
    query = uwb.sensor_time_s.to_numpy(float)
    pos = e.interpolate_gt(gt.time_s.to_numpy(), gt[['x','y','z']].to_numpy(), query)
    rows = []
    for i, row in enumerate(uwb.itertuples()):
        h = float(np.linalg.norm(pos[i] - anchors[int(row.anchor_id)])) if np.isfinite(pos[i]).all() else float('nan')
        z = float(row.observed_range_m)
        rows.append(dict(source_row=int(row.source_observation_index), obs_id=int(row.obs_id),
            time_s=float(row.sensor_time_s), tag_id=int(row.tag_id), anchor_id=int(row.anchor_id),
            range_m=z, h_GT_m=h, error_m=z-h if math.isfinite(h) and z > 0 else float('nan')))
    events = e.events(rows)
    windows = e.union_intervals([(event['start_s'], event['end_s']) for event in events])
    directory = private / NAME
    directory.mkdir()
    e.save_csv(directory/'range_reference.csv', rows)
    e.save_csv(directory/'tag_gt.csv', gt.to_dict('records'))
    e.save_csv(directory/'all_positive_error_events.csv', events,
               ['tag_id','anchor_id','start_s','end_s','count','mean_error_m'])
    r.write(directory/'window_union.json', windows)
    return dict(recording=NAME, event_count=len(events), window_union_count=len(windows),
        window_union_duration_s=sum(hi-lo for lo,hi in windows),
        reference_count=sum(math.isfinite(row['error_m']) for row in rows))


def evaluate(out):
    from rr_admitted import verify
    lock = verify(out)
    ledger = r.read(out/'execution.json')
    if ledger['lock_id'] != lock['lock_id'] or ledger['status'] == 'RUNNING':
        raise ValueError('RUN_NOT_SEALED')
    for task in ledger['tasks']:
        for path, digest in task.get('artifact_hashes', {}).items():
            if r.sha(path) != digest:
                raise ValueError('SCIENTIFIC_ARTIFACT_CHANGED:' + path)
    if (out/'metrics.csv').exists():
        raise ValueError('EVALUATION_EXISTS')
    private = r.WS/'evaluator_private/icra/own_vicon_flow'/out.name
    private.mkdir(parents=True, exist_ok=False)
    audit = audit_recording(out, lock, private)
    lo, hi = lock['rows'][0]['interval_s']
    times = e.grid(lo, hi)
    gt_frame = pd.read_csv(private/NAME/'tag_gt.csv').drop_duplicates('time_s').sort_values('time_s')
    gt = e.interpolate_gt(gt_frame.time_s.to_numpy(), gt_frame[['x','y','z']].to_numpy(), times)
    windows = r.read(private/NAME/'window_union.json')
    tasks = {task['task']:task for task in ledger['tasks']}
    estimates, local = {}, {}
    prepared = out/NAME/'prepare_runs/prepare/observations.csv'
    plan = [int(row['obs_id']) for row in common.rows(prepared)
            if common.boolval(row['valid']) and common.boolval(row['planned'])]
    reference = pd.read_csv(private/NAME/'range_reference.csv')
    raw = {int(row.obs_id):float(row.range_m) for row in reference.itertuples()}
    hgt = {int(row.obs_id):float(row.h_GT_m) for row in reference.itertuples()}
    for method in r.METHODS:
        task = tasks[method]
        run = Path(task.get('run_directory', out/NAME/'NO_RESULT'))
        status = r.read(run/'run_status.json') if (run/'run_status.json').exists() else {}
        row = dict(recording=NAME, method=method, status=task['status'], failure=task.get('reason',''),
            exit_code=task.get('exit_code'), ATE_RMSE_m=None, ATE_P95_m=None, Window_RMSE_m=None,
            evaluated_samples=0, coverage=0., candidate_observations=None, candidate_segments=None,
            decision_accepted_segments=None, final_accepted_segments=None, fallback=None,
            range_before_RMSE_m=None, range_after_RMSE_m=None, range_count=None,
            recovered_range_count=None, recovered_before_RMSE_m=None, recovered_after_RMSE_m=None)
        estimates[method] = np.full_like(gt, np.nan)
        if task['status'] == 'SUCCESS':
            if method != 'SFUISE-ToA' and not status.get('valid_estimate_exported', (run/'trajectory.tum').exists()):
                raise ValueError('INVALID_ESTIMATE')
            tum = np.loadtxt(run/'trajectory.tum', ndmin=2)
            et, ep = e.tag_trajectory(tum, lock['geometry']['lever_body_m'])
            estimates[method] = e.nearest_estimate(et, ep, times, [lo, hi])
            row.update(e.matched_metrics({method:estimates[method]}, gt, times, windows)[method])
            offsets, accepted = {}, set()
            if method in r.METHODS[:2]:
                offsets, accepted, counts = common.final_offsets(run, plan)
                row.update(counts)
            else:
                row['fallback'] = False
            row.update(e.range_metrics(plan, raw, hgt, offsets, accepted, row['fallback']))
        else:
            producer = tasks['producer']
            detector = Path(producer.get('run_directory', out/NAME/'NO_RESULT'))/'production_detector_status.json'
            if method in r.METHODS[:2] and detector.exists():
                doc = r.read(detector)
                row['candidate_observations'] = doc.get('candidate_observation_count')
                row['candidate_segments'] = doc.get('final_segment_count')
        local[method] = row
    pair = dict(recording=NAME, delta_RR_m=None, delta_RR_NLOS_m=None, interpretation='NA',
                fallback=None, common_samples=0, comparison_status='UNAVAILABLE')
    rr_methods = r.METHODS[:2]
    if all(tasks[method]['status'] == 'SUCCESS' for method in rr_methods):
        identities = [common.identity(out, NAME, tasks[method]) for method in rr_methods]
        r.require_pair(*identities)
        paired = e.matched_metrics({method:estimates[method] for method in rr_methods}, gt, times, windows)
        pair.update(e.paired_difference(paired[rr_methods[0]], paired[rr_methods[1]],
                                        local[rr_methods[1]]['fallback']),
                    common_samples=paired[rr_methods[0]]['evaluated_samples'],
                    comparison_status='MATCHED_IDENTICAL_STAGE2')
        for method in rr_methods:
            local[method].update(paired[method])
        pairing = dict(recording=NAME, status='PASS', identity=identities[0])
    else:
        pairing = dict(recording=NAME, status='UNAVAILABLE_DEPENDENT_FAILURE')
    four = e.matched_metrics(estimates, gt, times, windows)
    pair['four_method_common_samples'] = four[r.METHODS[0]]['evaluated_samples']
    metrics = list(local.values())
    e.save_csv(out/'metrics.csv', metrics)
    e.save_csv(out/'paired_differences.csv', [pair])
    e.save_csv(out/'four_method_common.csv', [dict(recording=NAME, method=method, **value)
                                               for method,value in four.items()])
    e.save_csv(out/'evaluator_audit.csv', [audit])
    r.write(out/'pairing_checks.json', [pairing])
    r.write(out/'evaluation.json', dict(status='COMPLETE_WITH_FAILURES_RETAINED',
        private_directory=str(private), lock_id=lock['lock_id'], metrics=metrics, pairs=[pair]))
    print({'metrics':metrics, 'pair':pair, 'audit':audit})
    return 0


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    raise SystemExit(evaluate(parser.parse_args().run.resolve()))
