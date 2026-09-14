#!/usr/bin/env python3
"""Run the frozen RR flow once on the audited own_vicon 15-31-28 bag."""
import argparse
import copy
import hashlib
import json
import math
import subprocess
from pathlib import Path

import numpy as np
import yaml

import run_recover_vs_reject as r
import rr_admitted as backend

NAME = '2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle'
RECORDINGS = (NAME,)
BAG = r.ROOT / 'data/own_vicon' / (NAME + '.bag')
BASE_PROTOCOL = r.ROOT / 'experiments/OWN_VICON_FLOW_PROTOCOL.md'
PROTOCOL = r.ROOT / 'experiments/OWN_VICON_INITIALIZATION_FIX_PROTOCOL.md'
IMU_TOPIC = '/livox/imu'
UWB_TOPIC = '/nlink_linktrack_nodeframe3'
GT_TOPIC = '/vrpn_client_node/tas_uwb_0/pose'
ANCHOR_TOPICS = {i: f'/vrpn_client_node/tas_uwb_{i}/pose' for i in range(1, 5)}
ACC_SCALE_MPS2_PER_G = 9.81


def stable_id(message_index, range_index):
    value = f'{NAME}|source_message={message_index}|source_range={range_index}'
    h = 1469598103934665603
    for byte in value.encode():
        h = ((h ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return h


def extract_measurements(path=BAG):
    """Read only measurement topics; GT and anchor topics are absent from the query."""
    import rosbag
    imu_raw, uwb_raw = [], []
    message_index = 0
    with rosbag.Bag(str(path)) as bag:
        for topic, msg, _ in bag.read_messages(topics=[IMU_TOPIC, UWB_TOPIC]):
            stamp = msg.header.stamp.to_sec()
            if topic == IMU_TOPIC:
                imu_raw.append((stamp, msg.linear_acceleration.x, msg.linear_acceleration.y,
                                msg.linear_acceleration.z, msg.angular_velocity.x,
                                msg.angular_velocity.y, msg.angular_velocity.z))
            else:
                for range_index, node in enumerate(msg.nodes):
                    uwb_raw.append(dict(source_message_index=message_index,
                                        source_range_index=range_index,
                                        source_observation_index=len(uwb_raw), sensor_epoch_s=stamp,
                                        tag_id=int(msg.id), anchor_id=int(node.id),
                                        range_m=float(node.dis), fp_rssi_dbm=float(node.fp_rssi),
                                        rx_rssi_dbm=float(node.rx_rssi)))
                message_index += 1
    if not imu_raw or not uwb_raw:
        raise ValueError('EMPTY_MEASUREMENT_TOPIC')
    origin = min(imu_raw[0][0], uwb_raw[0]['sensor_epoch_s'])
    imu = []
    for i, row in enumerate(imu_raw):
        values = [row[0] - origin, *(v * ACC_SCALE_MPS2_PER_G for v in row[1:4]), *row[4:]]
        if not np.isfinite(values).all():
            raise ValueError('NONFINITE_IMU')
        imu.append(dict(zip(r.IMU_HEADER, [i, *values, 0, 1, 0, 0, 0])))
    uwb = []
    for row in uwb_raw:
        t = row['sensor_epoch_s'] - origin
        z = row['range_m']
        valid = math.isfinite(z) and z > 0 and row['anchor_id'] in ANCHOR_TOPICS
        uwb.append(dict(zip(r.UWB_HEADER, [stable_id(row['source_message_index'], row['source_range_index']),
            row['source_message_index'], row['source_range_index'], row['source_observation_index'], t,
            row['tag_id'], row['anchor_id'], z, row['fp_rssi_dbm'], row['rx_rssi_dbm'], int(valid),
            '' if valid else 'INVALID_SOURCE_RANGE_OR_ANCHOR'.encode().hex()])))
    for rows in (imu, uwb):
        if any(float(b['sensor_time_s']) < float(a['sensor_time_s']) for a, b in zip(rows, rows[1:])):
            raise ValueError('NONMONOTONIC_MEASUREMENT_TIME')
    return origin, imu, uwb


def anchor_geometry(path=BAG):
    import rosbag
    points = {key: [] for key in ANCHOR_TOPICS}
    reverse = {topic: key for key, topic in ANCHOR_TOPICS.items()}
    with rosbag.Bag(str(path)) as bag:
        for topic, msg, _ in bag.read_messages(topics=list(reverse)):
            p = msg.pose.position
            points[reverse[topic]].append([p.x, p.y, p.z])
    anchors, spans = {}, {}
    for key, values in points.items():
        array = np.asarray(values, dtype=float)
        if len(array) < 100 or not np.isfinite(array).all():
            raise ValueError('INVALID_ANCHOR_VICON')
        anchors[key] = np.median(array, axis=0).tolist()
        spans[key] = float(np.ptp(array, axis=0).max())
        if spans[key] > 0.03:
            raise ValueError('NONSTATIC_ANCHOR_VICON')
    return anchors, spans


def cache_payloads(origin, imu, uwb, anchors):
    ub = r.csv_bytes(uwb, r.UWB_HEADER)
    ib = r.csv_bytes(imu, r.IMU_HEADER)
    manifest = dict(schema='nlos_measurement_cache_v2', base_recording_id=NAME,
        base_source_sha256='sha256:' + r.sha(BAG), recording_time_origin_s=0.,
        time_basis='sensor_time_from_recording_origin',
        imu_units='acc_mps2,gyro_radps,orientation_quaternion_wxyz',
        uwb_units='time_s,range_m,rssi_dbm', uwb_message_grouping='source_message_index',
        imu_file='imu.csv', uwb_file='uwb_observations.csv',
        imu_sha256='sha256:' + hashlib.sha256(ib).hexdigest(),
        uwb_sha256='sha256:' + hashlib.sha256(ub).hexdigest(), imu_count=len(imu),
        uwb_observation_count=len(uwb),
        uwb_message_count=len({int(row['source_message_index']) for row in uwb}),
        transform_sha256='sha256:' + r.digest(dict(version='OWN_VICON_BAG_V1',
            acceleration='raw_livox_g_times_9.81_to_specific_force_mps2', gyro='raw_radps',
            axes='livox_frame_identity', time='header_stamp_common_origin_no_fitted_offset',
            lever_body_m=[0., 0., 0.], anchors=anchors, orientation='unused_identity_placeholder')))
    manifest['cache_id'] = r.cache_id(manifest)
    return manifest, ub, ib


def prepare(out):
    out.mkdir(parents=True, exist_ok=False)
    if not BAG.exists():
        raise FileNotFoundError(BAG)
    origin, imu, uwb = extract_measurements()
    anchors, spans = anchor_geometry()
    manifest, ub, ib = cache_payloads(origin, imu, uwb, anchors)
    cfg = yaml.safe_load(r.CONFIG.read_text())
    assert cfg['keyframe']['step'] == 4
    assert [cfg['nlos'][k] for k in ('cusum_forward_kappa', 'cusum_backward_kappa',
        'cusum_forward_h', 'cusum_backward_h')] == [.5, .5, 7.0234689587858723, 7.0234689587858714]
    sf_commit = subprocess.check_output(['git', '-C', str(backend.SF_SOURCE), 'rev-parse', 'HEAD'], text=True).strip()
    if sf_commit != '75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d':
        raise ValueError('SFUISE_VERSION_CHANGED')
    d = out / NAME
    inp = d / 'input'
    inp.mkdir(parents=True)
    (inp / 'uwb_observations.csv').write_bytes(ub)
    (inp / 'imu.csv').write_bytes(ib)
    r.write(inp / 'input_manifest.json', manifest)
    config = copy.deepcopy(cfg)
    config['dataset']['cache_manifest'] = str(inp / 'input_manifest.json')
    config['anchors'] = [dict(id=key, pos=value, prior_sigma=.1) for key, value in sorted(anchors.items())]
    config['extrinsics']['lever_arm_init'] = [0., 0., 0.]
    (d / 'config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    cauchy = copy.deepcopy(config)
    cauchy['nlos'].update(mode='disabled', score_recoverability=False, final_inference_enabled=False)
    (d / 'cauchy.yaml').write_text(yaml.safe_dump(cauchy, sort_keys=False))
    sf = yaml.safe_load(backend.SF_CONFIG.read_text())
    sf.update(topic_imu='/rr/imu', topic_ground_truth='/rr/NO_GT', offset=[0., 0., 0.],
              toa_offset=[0.] * 4, uwb_frequency=50, imu_frequency=200,
              acc_ratio=False, gyro_unit=False)
    assert sf['uwb_sample_coeff'] == sf['imu_sample_coeff'] == 1
    (d / 'sfuise.yaml').write_text(yaml.safe_dump(sf, sort_keys=False))
    interval = [min(float(row['sensor_time_s']) for row in uwb),
                max(float(row['sensor_time_s']) for row in uwb)]
    geometry = dict(anchors=anchors, anchor_max_component_spans_m=spans,
                    lever_body_m=[0., 0., 0.], reference='VICON_TAG0_APPROX_UWB_ANTENNA',
                    status='USER_APPROXIMATE_COLOCATION', acceleration='SPECIFIC_FORCE_MPS2',
                    raw_acceleration='LIVOX_G', gyro='LIVOX_FRAME_RADPS',
                    time='HEADER_STAMP_COMMON_ORIGIN_NO_FITTED_OFFSET')
    implementations = [Path(__file__).resolve(), Path(r.__file__), Path(backend.__file__),
        r.ROOT / 'experiments/scripts/evaluate_own_vicon_flow.py',
        r.ROOT / 'experiments/scripts/test_own_vicon_flow.py', BASE_PROTOCOL, PROTOCOL]
    lock = dict(schema='OWN_VICON_FLOW_LOCK_V2_INITIALIZATION_FIX',
        role='DEVELOPMENT_SINGLE_RECORDING_FLOW',
        recordings=list(RECORDINGS), methods=list(r.METHODS),
        evaluation={**r.EVALUATION, 'reference':'tag0_vicon_approx_uwb_antenna'}, geometry=geometry,
        rows=[dict(recording=NAME, interval_s=interval, tag_id=0,
                   input=str(inp / 'input_manifest.json'), uwb_count=len(uwb),
                   uwb_message_count=manifest['uwb_message_count'], imu_count=len(imu),
                   measurement_id=manifest['cache_id'])], core_files=backend.core_files(),
        implementation=backend.filemap(implementations), source_hashes={str(BAG):r.sha(BAG)},
        prepared_hashes={}, scientific_budget=dict(max_tasks=5, seconds_per_tree=1800, retries=0),
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
        dirty_tracked_diff_sha256=r.digest(subprocess.check_output(['git','diff','--binary'], text=True)))
    r.write(d / 'preparation.json', dict(status='CACHE_PUBLISHED_NOT_CPP_PREPARED', geometry=geometry,
        gt_access_by_estimator=False, measurement_topics=[IMU_TOPIC, UWB_TOPIC],
        excluded_gt_topic=GT_TOPIC, raw_ranges_unchanged=True, imu_acceleration_scale=ACC_SCALE_MPS2_PER_G,
        time_origin_epoch_s=origin))
    for path in [*inp.iterdir(), d/'config.yaml', d/'cauchy.yaml', d/'sfuise.yaml']:
        lock['prepared_hashes'][str(path)] = r.sha(path)
    lock['lock_id'] = r.digest(lock)
    r.write(out / 'lock.json', lock)
    print(out)
    return 0


def execute(out):
    lock = backend.verify(out)
    preflight = r.read(out / 'preflight.json')
    if preflight['status'] != 'PASS' or preflight['lock_id'] != lock['lock_id']:
        raise ValueError('PREFLIGHT_NOT_PASSED')
    row = preflight['rows'][0]
    if r.sha(out / NAME / 'measurements.bag') != row['ros']['bag_sha256']:
        raise ValueError('ROS_BAG_CHANGED')
    path = out / 'execution.json'
    if path.exists():
        raise ValueError('NO_SCIENCE_RETRY')
    ledger = dict(lock_id=lock['lock_id'], tasks=[], scientific_processes_started=0, status='RUNNING')
    r.write(path, ledger)
    cache = None
    for task in r.TASKS:
        if task in r.METHODS[:2] and cache is None:
            result = dict(recording=NAME, task=task, status='NOT_RUN', reason='PRODUCER_FAILED', exit_code=None)
        else:
            ledger['current_task'] = [NAME, task]
            ledger['scientific_processes_started'] += 1
            r.write(path, ledger)
            try:
                result = (backend.call_sfuise(out, NAME) if task == 'SFUISE-ToA' else
                          backend.call_backend(out, NAME, task, cache if task in r.METHODS[:2] else None))
            except Exception as exc:
                invocation = out / NAME / (task + '_invocation.json')
                result = r.read(invocation) if invocation.exists() else dict(recording=NAME, task=task, exit_code=None)
                result.update(status='FAILURE', reason=type(exc).__name__ + ': ' + str(exc))
            if task == 'producer' and result['status'] == 'SUCCESS':
                cache = Path(result['cache_manifest'])
        if result.get('run_directory'):
            result['artifact_hashes'] = backend.filemap(p for p in Path(result['run_directory']).rglob('*') if p.is_file())
        ledger['tasks'].append(result)
        r.write(path, ledger)
        print(NAME, task, result['status'], result.get('reason', ''), flush=True)
    ledger['status'] = ('SUCCESS' if all(task['status'] == 'SUCCESS' for task in ledger['tasks'])
                        else 'COMPLETE_WITH_RETAINED_FAILURES')
    ledger.pop('current_task', None)
    r.write(path, ledger)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=['prepare','preflight','execute','evaluate','verify'])
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    out = args.run.resolve()
    if args.stage == 'prepare':
        return prepare(out)
    if args.stage == 'preflight':
        return backend.preflight(out)
    if args.stage == 'execute':
        return execute(out)
    if args.stage == 'verify':
        backend.verify(out)
        print('LOCK_CORE_INPUT_IMPLEMENTATION_PASS')
        return 0
    from evaluate_own_vicon_flow import evaluate
    return evaluate(out)


if __name__ == '__main__':
    raise SystemExit(main())
