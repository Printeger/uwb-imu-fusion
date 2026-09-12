#!/usr/bin/env python3
"""Read-only local inventory for the five ICRA dataset families.

The script never invokes an estimator and never changes files below data/.  Two
known interrupted own_vicon bags have no ROS index; with --recover-unindexed a
temporary copy is reindexed and inspected, while the original hash and bytes
remain the recording identity.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
from datetime import datetime, timezone

import rosbag


ROOT = Path(__file__).resolve().parents[2]


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def rel(path):
    return str(path.resolve().relative_to(ROOT.resolve()))


def finite_values(values):
    return all(math.isfinite(float(value)) for value in values)


class TimeAudit:
    def __init__(self):
        self.count = 0
        self.minimum = None
        self.maximum = None
        self.previous = None
        self.backward_steps = 0
        self.duplicate_steps = 0
        self.negative_values = 0

    def add(self, value):
        value = float(value)
        self.count += 1
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)
        if self.previous is not None:
            self.backward_steps += value < self.previous
            self.duplicate_steps += value == self.previous
        self.negative_values += value < 0
        self.previous = value

    def result(self):
        return {
            'count': self.count,
            'min_s': self.minimum,
            'max_s': self.maximum,
            'backward_steps_in_file_order': self.backward_steps,
            'duplicate_steps_in_file_order': self.duplicate_steps,
            'negative_values': self.negative_values,
        }


def stamp_seconds(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def csv_audit(path, required_columns, numeric_columns, time_column,
              id_columns=()):
    rows = 0
    malformed = 0
    nonfinite = 0
    times = TimeAudit()
    ids = {name: set() for name in id_columns}
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        header = reader.fieldnames or []
        duplicate_columns = sorted({x for x in header if header.count(x) > 1})
        missing_columns = sorted(set(required_columns) - set(header))
        for row in reader:
            rows += 1
            try:
                values = [row[name] for name in numeric_columns]
                if any(value is None or value == '' for value in values):
                    malformed += 1
                    continue
                if not finite_values(values):
                    nonfinite += 1
                times.add(row[time_column])
                for name in id_columns:
                    ids[name].add(str(row[name]))
            except (KeyError, TypeError, ValueError):
                malformed += 1
    return {
        'path': rel(path),
        'bytes': path.stat().st_size,
        'sha256': sha256(path),
        'columns': header,
        'duplicate_columns': duplicate_columns,
        'missing_required_columns': missing_columns,
        'rows': rows,
        'malformed_selected_rows': malformed,
        'nonfinite_selected_rows': nonfinite,
        'time': times.result(),
        'ids': {name: sorted(values) for name, values in ids.items()},
    }


def vector_values(value):
    return (value.x, value.y, value.z)


def pose_values(pose):
    return (*vector_values(pose.position), pose.orientation.x,
            pose.orientation.y, pose.orientation.z, pose.orientation.w)


def inspect_huec_bag(path):
    uwb_topics = [f'/dwm1001/anchor{x}' for x in (3, 5, 9, 12)]
    selected = uwb_topics + ['/imu/data', '/odometry/local_gps']
    audits = {topic: TimeAudit() for topic in selected}
    counts = {topic: 0 for topic in selected}
    nonfinite = {topic: 0 for topic in selected}
    nonfinite_fields = {topic: {} for topic in selected}
    nonfinite_examples = {topic: [] for topic in selected}
    anchor_ids = set()
    anchor_coordinates = {}
    with rosbag.Bag(str(path), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        for topic, msg, _ in bag.read_messages(topics=selected):
            counts[topic] += 1
            if topic in uwb_topics:
                audits[topic].add(stamp_seconds(msg.stamp))
                anchor_ids.add(str(msg.id))
                anchor_coordinates.setdefault(str(msg.id), [msg.x, msg.y, msg.z])
                named_values = {'x': msg.x, 'y': msg.y, 'z': msg.z,
                                'distanceFromTag': msg.distanceFromTag,
                                'rssi': msg.rssi, 'rssi_fp': msg.rssi_fp}
            elif topic == '/imu/data':
                audits[topic].add(stamp_seconds(msg.header.stamp))
                named_values = {
                    'angular_velocity_x': msg.angular_velocity.x,
                    'angular_velocity_y': msg.angular_velocity.y,
                    'angular_velocity_z': msg.angular_velocity.z,
                    'linear_acceleration_x': msg.linear_acceleration.x,
                    'linear_acceleration_y': msg.linear_acceleration.y,
                    'linear_acceleration_z': msg.linear_acceleration.z,
                }
            else:
                audits[topic].add(stamp_seconds(msg.header.stamp))
                values = pose_values(msg.pose.pose)
                named_values = {name: value for name, value in zip(
                    ('position_x', 'position_y', 'position_z', 'orientation_x',
                     'orientation_y', 'orientation_z', 'orientation_w'), values)}
            bad_fields = [name for name, value in named_values.items()
                          if not math.isfinite(float(value))]
            if bad_fields:
                nonfinite[topic] += 1
                for name in bad_fields:
                    nonfinite_fields[topic][name] = nonfinite_fields[topic].get(name, 0) + 1
                if len(nonfinite_examples[topic]) < 5:
                    nonfinite_examples[topic].append({
                        'message_ordinal_in_topic': counts[topic] - 1,
                        'time_s': audits[topic].previous,
                        'fields': bad_fields,
                    })
        return {
            'raw_data_path': rel(path),
            'bytes': path.stat().st_size,
            'sha256': sha256(path),
            'bag_indexed': True,
            'bag_time_s': {'start': bag.get_start_time(),
                           'end': bag.get_end_time()},
            'topics': {
                topic: {
                    'message_type': info[topic].msg_type if topic in info else None,
                    'declared_count': info[topic].message_count if topic in info else 0,
                    'scanned_count': counts[topic],
                    'nonfinite_selected_messages': nonfinite[topic],
                    'nonfinite_fields': nonfinite_fields[topic],
                    'nonfinite_examples': nonfinite_examples[topic],
                    'header_or_message_time': audits[topic].result(),
                } for topic in selected
            },
            'tag_ids': [],
            'tag_id_status': 'NOT_PRESENT_IN_ANCHOR_MESSAGE',
            'anchor_ids': sorted(anchor_ids),
            'anchor_coordinates_first_message_m': anchor_coordinates,
        }


def own_scan(path):
    uwb_topic = '/nlink_linktrack_nodeframe3'
    imu_topic = '/livox/imu'
    vicon_topics = [f'/vrpn_client_node/tas_uwb_{x}/pose' for x in range(5)]
    selected = [uwb_topic, imu_topic] + vicon_topics
    audits = {topic: TimeAudit() for topic in selected}
    counts = {topic: 0 for topic in selected}
    nonfinite = {topic: 0 for topic in selected}
    tag_ids, anchor_ids = set(), set()
    first_vicon_positions = {}
    with rosbag.Bag(str(path), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        for topic, msg, _ in bag.read_messages(topics=selected):
            counts[topic] += 1
            audits[topic].add(stamp_seconds(msg.header.stamp))
            if topic == uwb_topic:
                tag_ids.add(str(msg.id))
                values = []
                for node in msg.nodes:
                    anchor_ids.add(str(node.id))
                    values.extend((node.dis, node.fp_rssi, node.rx_rssi))
            elif topic == imu_topic:
                values = (*vector_values(msg.angular_velocity),
                          *vector_values(msg.linear_acceleration))
            else:
                values = pose_values(msg.pose)
                first_vicon_positions.setdefault(topic, list(vector_values(msg.pose.position)))
            nonfinite[topic] += not finite_values(values)
        return {
            'content_bytes_after_optional_reindex': path.stat().st_size,
            'bag_time_s': {'start': bag.get_start_time(),
                           'end': bag.get_end_time()},
            'topics': {
                topic: {
                    'message_type': info[topic].msg_type if topic in info else None,
                    'declared_count': info[topic].message_count if topic in info else 0,
                    'scanned_count': counts[topic],
                    'nonfinite_selected_messages': nonfinite[topic],
                    'header_time': audits[topic].result(),
                } for topic in selected
            },
            'tag_ids': sorted(tag_ids),
            'anchor_ids': sorted(anchor_ids),
            'vicon_first_position_m': first_vicon_positions,
        }


def inspect_own_bag(path, recover_unindexed):
    base = {
        'raw_data_path': rel(path),
        'bytes': path.stat().st_size,
        'sha256': sha256(path),
    }
    try:
        scanned = own_scan(path)
        base.update({'original_bag_indexed': True,
                     'inspection_source': 'ORIGINAL_READ_ONLY'}, **scanned)
        return base
    except rosbag.bag.ROSBagUnindexedException as exc:
        base['original_bag_indexed'] = False
        base['original_open_error'] = str(exc)
        if not recover_unindexed:
            base['inspection_source'] = 'ORIGINAL_FILE_ONLY'
            return base
    with tempfile.TemporaryDirectory(prefix='own_vicon_reindex_') as tmp:
        copy = Path(tmp) / 'recording.bag'
        shutil.copy2(path, copy)
        completed = subprocess.run(
            ['rosbag', 'reindex', str(copy)], text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
        base['temporary_reindex_exit_code'] = completed.returncode
        if completed.returncode != 0:
            base['inspection_source'] = 'TEMP_COPY_REINDEX_FAILED'
            base['temporary_reindex_output_tail'] = completed.stdout[-2000:]
            return base
        base.update({'inspection_source': 'TEMP_COPY_REINDEXED_READ_ONLY_SOURCE'},
                    **own_scan(copy))
        return base


def inspect_sfuise_bag(path):
    uwb_topic = '/rtls_flares'
    anchor_topic = '/anchor_list'
    imu_topic = '/waveshare_sense_hat_b'
    gt_topic = '/vive/transform/tracker_1_ref'
    selected = [uwb_topic, anchor_topic, imu_topic, gt_topic]
    audits = {topic: TimeAudit() for topic in selected}
    counts = {topic: 0 for topic in selected}
    nonfinite = {topic: 0 for topic in selected}
    tag_ids, anchor_ids = set(), set()
    anchor_coordinates = {}
    frames = {topic: set() for topic in selected}
    with rosbag.Bag(str(path), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        for topic, msg, bag_time in bag.read_messages(topics=selected):
            counts[topic] += 1
            if hasattr(msg, 'header'):
                audits[topic].add(stamp_seconds(msg.header.stamp))
                frames[topic].add(msg.header.frame_id)
            else:
                audits[topic].add(bag_time.to_sec())
            if topic == uwb_topic:
                tag_ids.add(str(msg.id))
                values = []
                for value in msg.ranges:
                    anchor_ids.add(str(value.id))
                    values.extend((value.range, value.fpp, value.rxp))
            elif topic == anchor_topic:
                values = []
                for anchor in msg.anchor:
                    anchor_coordinates.setdefault(
                        str(anchor.id), list(vector_values(anchor.position)))
                    values.extend(vector_values(anchor.position))
            elif topic == imu_topic:
                values = (*vector_values(msg.angular_velocity),
                          *vector_values(msg.linear_acceleration))
            else:
                frames[topic].add(msg.child_frame_id)
                values = (*vector_values(msg.transform.translation),
                          msg.transform.rotation.x, msg.transform.rotation.y,
                          msg.transform.rotation.z, msg.transform.rotation.w)
            nonfinite[topic] += not finite_values(values)
        return {
            'raw_data_path': rel(path),
            'bytes': path.stat().st_size,
            'sha256': sha256(path),
            'bag_indexed': True,
            'bag_time_s': {'start': bag.get_start_time(),
                           'end': bag.get_end_time()},
            'topics': {
                topic: {
                    'message_type': info[topic].msg_type if topic in info else None,
                    'declared_count': info[topic].message_count if topic in info else 0,
                    'scanned_count': counts[topic],
                    'nonfinite_selected_messages': nonfinite[topic],
                    'header_time': audits[topic].result(),
                    'frames': sorted(frames[topic]),
                } for topic in selected
            },
            'tag_ids': sorted(tag_ids),
            'anchor_ids': sorted(anchor_ids),
            'anchor_coordinates_first_message_m': anchor_coordinates,
        }


def inspect_miluv(root):
    result = {}
    for sequence in sorted(p for p in root.iterdir() if p.is_dir()):
        robot = sequence / 'ifo001'
        uwb = csv_audit(
            robot / 'uwb_range.csv',
            ('range', 'range_raw', 'from_id', 'to_id', 'timestamp'),
            ('range', 'range_raw', 'from_id', 'to_id', 'timestamp'),
            'timestamp', ('from_id', 'to_id'))
        imu = csv_audit(
            robot / 'imu_px4.csv',
            ('timestamp', 'angular_velocity.x', 'angular_velocity.y',
             'angular_velocity.z', 'linear_acceleration.x',
             'linear_acceleration.y', 'linear_acceleration.z'),
            ('timestamp', 'angular_velocity.x', 'angular_velocity.y',
             'angular_velocity.z', 'linear_acceleration.x',
             'linear_acceleration.y', 'linear_acceleration.z'),
            'timestamp')
        gt = csv_audit(
            robot / 'mocap.csv',
            ('timestamp', 'pose.position.x', 'pose.position.y',
             'pose.position.z', 'pose.orientation.x',
             'pose.orientation.y', 'pose.orientation.z',
             'pose.orientation.w'),
            ('timestamp', 'pose.position.x', 'pose.position.y',
             'pose.position.z', 'pose.orientation.x',
             'pose.orientation.y', 'pose.orientation.z',
             'pose.orientation.w'), 'timestamp')
        shift = sequence / 'timeshift.yaml'
        result[sequence.name] = {
            'uwb': uwb, 'imu': imu, 'gt': gt,
            'timeshift': {'path': rel(shift), 'bytes': shift.stat().st_size,
                          'sha256': sha256(shift),
                          'text': shift.read_text().strip().splitlines()},
        }
    return result


def inspect_starloc(root):
    params_path = root / 'dataset_params.json'
    params = {item['name']: item for item in json.loads(params_path.read_text())}
    result = {}
    for sequence in sorted(p for p in (root / 'data').iterdir() if p.is_dir()):
        uwb = csv_audit(
            sequence / 'uwb.csv', ('time_s', 'range', 'from_id', 'to_id'),
            ('time_s', 'range', 'from_id', 'to_id'), 'time_s',
            ('from_id', 'to_id'))
        imu = csv_audit(
            sequence / 'imu.csv',
            ('time_s', 'angular_velocity_x', 'angular_velocity_y',
             'angular_velocity_z', 'linear_acceleration_x',
             'linear_acceleration_y', 'linear_acceleration_z',
             'x', 'y', 'z', 'w', 'rot_x', 'rot_y', 'rot_z'),
            ('time_s', 'angular_velocity_x', 'angular_velocity_y',
             'angular_velocity_z', 'linear_acceleration_x',
             'linear_acceleration_y', 'linear_acceleration_z',
             'x', 'y', 'z', 'w', 'rot_x', 'rot_y', 'rot_z'), 'time_s')
        calib = sequence / 'calib.json'
        calib_value = json.loads(calib.read_text())
        finite_calib = finite_values(
            value for key, value in calib_value.items()
            if isinstance(value, (int, float))) and all(
                finite_values(v.values()) for v in calib_value.values()
                if isinstance(v, dict))
        version = params[sequence.name]['landmarks']
        anchors = root / 'mocap' / f'uwb_markers_{version}.csv'
        result[sequence.name] = {
            'dataset_params': params[sequence.name],
            'uwb': uwb,
            'imu_and_embedded_gt': imu,
            'calibration': {'path': rel(calib), 'bytes': calib.stat().st_size,
                            'sha256': sha256(calib), 'finite': finite_calib},
            'anchor_coordinates': {
                'path': rel(anchors), 'bytes': anchors.stat().st_size,
                'sha256': sha256(anchors)},
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, default=ROOT)
    parser.add_argument('--recover-unindexed', action='store_true')
    args = parser.parse_args()
    if args.repo_root.resolve() != ROOT.resolve():
        raise SystemExit('This inventory version is bound to its repository root')
    huec_paths = sorted((ROOT / 'data/HUEC/Dynamic_measurements').glob(
        '**/combined.bag'))
    own_paths = sorted((ROOT / 'data/own_vicon').glob('*.bag'))
    sfuise_paths = sorted((ROOT / 'data/SFUISE').glob('ISAS-Walk*.bag'))
    document = {
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'scope': 'READ_ONLY_METADATA_AND_SELECTED_FIELD_VALIDATION_NO_ESTIMATOR',
        'source_data_modified': False,
        'HUEC': {path.parent.relative_to(ROOT / 'data/HUEC').as_posix():
                 inspect_huec_bag(path) for path in huec_paths},
        'MILUV': inspect_miluv(ROOT / 'data/MILUV'),
        'own_vicon': {path.stem: inspect_own_bag(path, args.recover_unindexed)
                      for path in own_paths},
        'SFUISE': {path.stem: inspect_sfuise_bag(path)
                   for path in sfuise_paths},
        'starloc': inspect_starloc(ROOT / 'data/starloc'),
    }
    print(json.dumps(document, indent=2, sort_keys=True, allow_nan=False))


if __name__ == '__main__':
    main()
