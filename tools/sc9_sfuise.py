#!/usr/bin/env python3
"""SC9 canonical-input SFUISE comparison; original estimator stays pinned."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess

import run_sfuise_benchmark as baseline

ROOT = baseline.ROOT
CACHE = ROOT / '.cache/sc9/sfuise'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare(case):
    wire = ROOT / '.cache/sc9/wire/canonical-v1' / (case + '-base')
    source = ROOT / '.cache/sc9/data' / ('ISAS-' + case)
    output = CACHE / 'inputs' / case
    output.mkdir(parents=True, exist_ok=False)
    wm = json.loads((wire / 'manifest.json').read_text())
    events = [json.loads(line) for line in (wire / 'events.jsonl').read_text().splitlines()]
    origin = wm['first_imu_stamp_ns']
    offset = round(wm['legacy_first_imu_time_s'] * 1e9)
    def times(row, event):
        row['_stamp_ns'] = event['stamp_ns'] - origin + offset
        row['_received_ns'] = event['received_ns'] - origin + offset
        row['t'] = row['_stamp_ns'] / 1e9
    imu = list(csv.DictReader((source / 'imu.csv').open()))
    ie = [e for e in events if e['kind'] == 'imu']
    assert len(imu) == len(ie)
    for row, event in zip(imu, ie):
        assert [float(row[k]) for k in ['ax', 'ay', 'az']] == event['accel']
        assert [float(row[k]) for k in ['gx', 'gy', 'gz']] == event['gyro']
        times(row, event)
    uwb = list(csv.DictReader((source / 'uwb.csv').open()))
    ranges = [r for e in events if e['kind'] == 'range' for r in e['ranges']]
    assert len(uwb) == len(ranges)
    for row, event in zip(uwb, ranges):
        assert int(row['anchor_id']) == event['anchor_id'] < 65536
        assert int(row['source_message']) == event['source_sequence']
        assert bool(int(row['valid'])) == event['valid']
        assert float(row['range']) == event['range_m']
        assert float(row['range_raw']) == event['raw_range_m']
        times(row, event)
    for name, rows in [('imu.csv', imu), ('uwb.csv', uwb)]:
        with (output / name).open('w') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
    shutil.copyfile(source / 'anchors.csv', output / 'anchors.csv')
    manifest = json.loads((source / 'manifest.json').read_text())
    manifest['files'] = {n: {'sha256': sha(output/n)} for n in ['imu.csv', 'uwb.csv', 'anchors.csv']}
    manifest['evaluator_only'] = []
    manifest['sc9_canonical'] = dict(wire=str(wire), wire_sha=sha(wire/'events.jsonl'),
        first_imu_ns=origin, relative_origin_ns=offset,
        origin_rounding_error_s=offset/1e9-wm['legacy_first_imu_time_s'],
        mapping='single constant origin translation; exact integer header and receive intervals; sensor values unchanged; no GT file')
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(output, len(imu), len(uwb), flush=True)


def verify_bag(case, bag):
    import rosbag
    source = CACHE / 'inputs' / case
    meta = json.loads((source/'manifest.json').read_text())['sc9_canonical']
    events = [json.loads(line) for line in (Path(meta['wire'])/'events.jsonl').read_text().splitlines()]
    shift = 1000000000000 + meta['relative_origin_ns'] - meta['first_imu_ns']
    topics = ['/EstimationInterface/imu_ds', '/EstimationInterface/toa_ds64']
    expected = sorted(events, key=lambda e: e['received_ns'])
    count = 0
    with rosbag.Bag(str(bag)) as stream:
        assert all('ground_truth' not in t and '/gt' not in t for t in stream.get_type_and_topic_info().topics)
        actual = list(stream.read_messages(topics=topics))
    assert len(actual) == len(expected)
    for (topic, message, received), event in zip(actual, expected):
        assert received.to_nsec() == event['received_ns'] + shift
        if event['kind'] == 'imu':
            assert topic == topics[0] and message.header.stamp.to_nsec() == event['stamp_ns']+shift
            assert [getattr(message.linear_acceleration, a) for a in 'xyz'] == event['accel']
            assert [getattr(message.angular_velocity, a) for a in 'xyz'] == event['gyro']
        else:
            assert topic == topics[1]
            payload = [float(event['stamp_ns']+shift), float(len(event['ranges']))]
            for r in event['ranges']:
                assert r['stamp_ns'] == event['stamp_ns']
                payload.extend([float(r['anchor_id']), r['range_m'], float(r['valid'])])
            assert list(message.data) == payload
        count += 1
    result = dict(events=count, exact_values_headers_receipts_and_order=True, gt_topics=0,
                  bag_sha=sha(bag), wire_sha=meta['wire_sha'])
    (bag.parent/'canonical-input-parity.json').write_text(json.dumps(result, indent=2)+'\n')
    print(result, flush=True)


def build():
    import sc8_sfuise_observer as observer
    observer.CACHE = CACHE / 'observer'
    source = observer.generate()
    ws = baseline.workspace(CACHE)
    with (CACHE/'build.log').open('w') as log:
        subprocess.run(['catkin_make', '-C', str(ws), '-DCMAKE_BUILD_TYPE=Release',
            '-DCMAKE_CXX_FLAGS_DEBUG=-O3 -DNDEBUG',
            f'-DSFUISE_INCLUDE_DIR={baseline.SFUISE_ROOT}/sfuise/include',
            f'-DSFUISE_SC8_OBSERVER_SOURCE={source}', '-j2'], cwd=ws,
            stdout=log, stderr=subprocess.STDOUT, check=True)


def run(case, run_id):
    baseline.checked_pin()
    if any(c.strip() in ['cc1plus', 'clang', 'clang++'] for c in subprocess.check_output(['ps', '-eo', 'comm'], text=True).splitlines()):
        raise RuntimeError('compiler active')
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 11342))
    if not run_id or not all(c.isalnum() or c in '-_' for c in run_id):
        raise ValueError('simple run identity required')
    destination = CACHE/'runs'/run_id/case
    destination.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_HOME=str(destination/'roshome'), ROS_LOG_DIR=str(destination/'roslogs'),
                      ROS_HOSTNAME='127.0.0.1', ROS_IP='127.0.0.1')
    ws = baseline.workspace(CACHE); setup = ws/'devel/setup.bash'
    binaries = [ws/'devel/lib/uwb_benchmark_sfuise_adapter'/n for n in ['sfuise_sc8_observed', 'sfuise_trajectory_adapter']]
    frozen = dict(head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        pinned=baseline.PIN, binaries={str(p):sha(p) for p in binaries},
        sources={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__).resolve(), ROOT/'tools/run_sfuise_benchmark.py', ROOT/'tools/sc8_sfuise_observer.py', ROOT/'benchmark/sfuise_adapter_ros/src/sfuise_trajectory_adapter.cpp']},
        input_manifest=sha(CACHE/'inputs'/case/'manifest.json'), model=sha(ROOT/'.cache/sc9/empirical-model.json'),
        observer=json.loads((CACHE/'observer/patch_identity.json').read_text()), rate=1)
    (destination/'freeze.json').write_text(json.dumps(frozen, indent=2)+'\n')
    result = baseline.run_one(setup, CACHE/'inputs'/case, destination, 1., 'sfuise_sc8_observed', True)
    subprocess.run(['python3', str(Path(__file__)), 'verify-bag', '--case', case,
                    '--bag', str(destination/'input.bag')], env=baseline.source_environment(setup), check=True)
    assert result['status'] == 'SUCCESS', result
    print(result, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('action', choices=['prepare', 'build', 'run', 'verify-bag'])
    parser.add_argument('--case', choices=['Walk2', 'Walk3'], default='Walk2')
    parser.add_argument('--run-id'); parser.add_argument('--bag', type=Path)
    args = parser.parse_args()
    if args.action == 'prepare': prepare(args.case)
    elif args.action == 'build': build()
    elif args.action == 'verify-bag': verify_bag(args.case, args.bag)
    else: run(args.case, args.run_id)
