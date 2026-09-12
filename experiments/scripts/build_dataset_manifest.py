#!/usr/bin/env python3
"""Build the checked v2 inventory from the read-only metadata artifact.

This is an explicit maintenance command.  It writes only the manifest JSON and
never reads estimator output or changes anything below data/.
"""
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / 'experiments/datasets/five_dataset_metadata_observed.json'
OUTPUT = ROOT / 'experiments/datasets/dataset_manifest.json'


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def fact(status, value, evidence, notes):
    return {'status': status, 'value': value,
            'evidence': evidence if isinstance(evidence, list) else [evidence],
            'notes': notes}


def observed(value, pointer, notes):
    return fact('OBSERVED', value, pointer, notes)


def declared(value, evidence, notes):
    return fact('SOURCE_DECLARED', value, evidence, notes)


def unknown(evidence, notes):
    return fact('UNKNOWN', None, evidence, notes)


def file_item(role, item):
    return {'role': role, 'path': item['path'], 'sha256': item['sha256']}


def bag_file(role, item):
    return {'role': role, 'path': item['raw_data_path'], 'sha256': item['sha256']}


def local_validation(pointer, issues=(), extra_checks=()):
    issues = list(issues)
    return {
        'artifact_pointer': pointer,
        'outcome': 'PASS_WITH_RECORDED_ISSUES' if issues else 'PASS',
        'source_content_checked': True,
        'checks': ['PATH', 'SHA256', 'STRUCTURE', 'IDENTITY', 'SELECTED_FIELD_FINITE',
                   'TIME_ORDER'] + list(extra_checks),
        'issues': issues,
    }


def finish(entry, unresolved):
    issues = entry['local_validation']['issues']
    entry.update({
        'metadata_status': ('LOCAL_CONTENT_VALIDATED_WITH_RECORDED_ISSUES'
                            if issues else 'LOCAL_CONTENT_VALIDATED'),
        'formal_admission': 'NOT_ADMITTED',
        'estimator_runs_this_task': 'NOT_RUN',
        'unresolved_items': unresolved,
    })
    return entry


def common_entry(family, rid, raw, files, measurement_type, evidence,
                 role, used, development_evidence, split_group, validation):
    return {
        'dataset_family': family,
        'recording_id': rid,
        'raw_data_path': raw['path'] if 'path' in raw else raw['raw_data_path'],
        'raw_sha256': raw['sha256'],
        'recording_files': files,
        'uwb_measurement_type': measurement_type,
        'measurement_type_evidence': evidence,
        'role': role,
        'split_group_id': split_group,
        'used_during_method_development': used,
        'development_use_evidence': development_evidence,
        'local_validation': validation,
    }


def huec_entries(data):
    result = []
    for key, item in sorted(data.items()):
        parts = key.split('/')
        condition, trajectory, case = parts[1], parts[2], parts[3]
        rid = f'HUEC-{condition}-{trajectory}-{case}'
        pointer = f'experiments/datasets/five_dataset_metadata_observed.json#HUEC/{key}'
        issues = []
        for topic, audit in item['topics'].items():
            if audit['nonfinite_selected_messages']:
                issues.append(
                    f'{topic}: {audit["nonfinite_selected_messages"]} message(s) '
                    'contain nonfinite selected auxiliary/measurement fields')
        entry = common_entry(
            'HUEC', rid, item, [bag_file('RAW_CONTAINER', item)],
            'ABSOLUTE_RANGE',
            [pointer + '/topics', 'data/HUEC/custom_msg/msg/Anchor.msg: distanceFromTag'],
            'UNASSIGNED', None,
            ['Repository exposure audit found no recording-specific development-use ledger; keep UNKNOWN'],
            None, local_validation(pointer, issues))
        entry.update({
            'tag_ids': unknown(pointer + '/tag_id_status',
                               'Anchor messages do not carry a tag ID; do not invent one.'),
            'anchor_ids': observed(item['anchor_ids'], pointer + '/anchor_ids',
                                   'IDs scanned from every UWB anchor topic.'),
            'anchor_coordinates_source': observed(
                {'source': 'localizer_dwm1001/Anchor x,y,z', 'frame': 'UNDECLARED',
                 'unit': 'm', 'first_message_coordinates_m': item['anchor_coordinates_first_message_m']},
                pointer + '/anchor_coordinates_first_message_m',
                'Coordinates are embedded in the range message; independent survey provenance is unknown.'),
            'imu_source': observed(
                {'path': item['raw_data_path'], 'topic': '/imu/data',
                 'type': 'sensor_msgs/Imu', 'frame': 'imu_link',
                 'acceleration_unit': 'm/s^2', 'angular_velocity_unit': 'rad/s'},
                pointer + '/topics/~1imu~1data',
                'Units follow sensor_msgs semantics; device calibration provenance is not supplied.'),
            'gt_source': declared(
                {'path': item['raw_data_path'], 'topic': '/odometry/local_gps',
                 'type': 'nav_msgs/Odometry', 'use': 'EVALUATOR_ONLY'},
                ['data/HUEC/README.md', pointer + '/topics/~1odometry~1local_gps'],
                'README describes RTK-corrected GNSS; this task does not test its accuracy.'),
            'gt_reference_point': observed(
                {'parent_frame': 'map', 'child_frame': 'gps', 'reference_point': 'GNSS antenna/receiver origin'},
                pointer + '/topics/~1odometry~1local_gps',
                'The physical GNSS antenna point and its lever arm to IMU/tag remain unverified.'),
            'tag_imu_extrinsics': unknown(pointer,
                                          'No recording-bound tag-to-IMU transform was found.'),
            'gt_extrinsics': unknown(pointer,
                                     'GNSS-to-IMU/tag lever arm and frame transform were not supplied.'),
            'time_units': observed({'uwb': 's', 'imu': 's', 'gt': 's'}, pointer + '/topics',
                                   'ROS secs+nsecs were converted to seconds only for auditing.'),
            'time_synchronization': observed(
                {'uwb_field': 'Anchor.stamp', 'imu_field': 'Imu.header.stamp',
                 'gt_field': 'Odometry.header.stamp',
                 'mapping_to_seconds': 'secs + nsecs*1e-9',
                 'offset_drift_evidence': 'UNKNOWN'},
                pointer + '/topics',
                'All selected streams are monotonic, but shared physical clock and bag/header offset are unverified.'),
            'range_calibration': observed(
                {'input_field': 'distanceFromTag', 'input_unit': 'm',
                 'applied_corrections': 'UNKNOWN', 'fixed_beta_source': None,
                 'independence_evidence': None},
                ['data/HUEC/custom_msg/msg/Anchor.msg', pointer + '/topics'],
                'No independent LOS beta calibration was identified; zero is not treated as calibrated.'),
            'physical_obstruction': declared(
                {'recording_condition': condition, 'granularity': 'recording/directory label',
                 'link_time_annotation': None}, key,
                'The LOS/NLOS directory label is preserved; it is not a per-observation truth label.'),
            'license': declared('Apache-2.0 repository LICENSE', 'data/HUEC/LICENSE',
                                'Dataset redistribution scope should still be confirmed for publication.'),
        })
        unresolved = [
            'Tag ID absent from range messages',
            'Independent anchor survey/frame provenance',
            'Tag-IMU and GNSS-to-IMU/tag extrinsics',
            'Clock offset/drift validation',
            'Independent LOS fixed beta calibration',
            'Recording-specific method-development exposure and split assignment',
        ]
        if issues:
            unresolved.append('One nonfinite auxiliary quality value is recorded in local validation')
        result.append(finish(entry, unresolved))
    return result


def miluv_entries(data):
    result = []
    for rid, item in sorted(data.items()):
        pointer = f'experiments/datasets/five_dataset_metadata_observed.json#MILUV/{rid}'
        files = [file_item('UWB', item['uwb']), file_item('IMU', item['imu']),
                 file_item('GT', item['gt']), file_item('TIME_SYNC', item['timeshift'])]
        used = rid in ('default_1_random3_0', 'default_1_circular3D_0')
        role = 'DEV' if used else 'UNASSIGNED'
        development = ([
            'doc/ie_0911/STEP1_RESULT.md and STEP2_RESULTS.md: sequence used in paper-path development runs'
        ] if used else [
            'No recording-specific use ledger found for cirObstacles; keep UNKNOWN'
        ])
        entry = common_entry(
            'MILUV', rid, item['uwb'], files, 'ABSOLUTE_RANGE',
            [pointer + '/uwb/columns', 'src/data_loader.cpp: LoadMiluvCsv reads range/from_id/to_id'],
            role, True if used else None, development,
            f'MILUV/{rid}' if used else None, local_validation(pointer))
        anchor_source = unknown(
            pointer, 'No recording-bound anchor-coordinate file was found for this sequence.')
        extrinsics = unknown(
            pointer, 'No recording-bound tag-to-IMU extrinsic provenance was found.')
        if rid in ('default_1_random3_0', 'default_1_circular3D_0'):
            config = ('config/miluv_random.yaml' if 'random3' in rid
                      else 'config/miluv_circular.yaml')
            anchor_source = declared(
                {'source': config + ': anchors', 'frame': 'MILUV mocap/world', 'unit': 'm'},
                config, 'Coordinates are project configuration declarations; independent survey provenance is open.')
            extrinsics = declared(
                {'tag_levers_m': {'10': [0.13189, -0.17245, -0.05249],
                                  '11': [-0.17542, 0.15712, -0.05307]},
                 'convention': 'Project configuration tag levers; transform direction requires source closure'},
                config, 'Values were already used in development; calibration independence is unverified.')
        obstruction = unknown(pointer, 'No physical obstruction annotation was found.')
        if rid.startswith('cirObstacles'):
            obstruction = declared(
                {'recording_condition': 'circular obstacles', 'granularity': 'sequence name',
                 'link_time_annotation': None}, f'data/MILUV/{rid}',
                'Sequence naming is not a per-link or per-observation obstruction truth label.')
        entry.update({
            'tag_ids': observed(item['uwb']['ids']['from_id'], pointer + '/uwb/ids/from_id',
                                'All from_id values scanned from uwb_range.csv.'),
            'anchor_ids': observed(item['uwb']['ids']['to_id'], pointer + '/uwb/ids/to_id',
                                   'All to_id values scanned from uwb_range.csv.'),
            'anchor_coordinates_source': anchor_source,
            'imu_source': observed(
                {'path': item['imu']['path'], 'stream': 'PX4 IMU CSV',
                 'frame': 'PX4/body (axis convention requires source closure)',
                 'acceleration_unit': 'm/s^2', 'angular_velocity_unit': 'rad/s'},
                [pointer + '/imu/columns', 'src/data_loader.cpp: LoadMiluvCsv'],
                'The project loader consumes imu_px4.csv; imu_cam.csv is not selected here.'),
            'gt_source': observed(
                {'path': item['gt']['path'], 'columns': 'mocap pose', 'use': 'EVALUATOR_ONLY'},
                pointer + '/gt/columns', 'Mocap is inventoried separately from estimator measurements.'),
            'gt_reference_point': declared(
                'ifo001 motion-capture rigid-body origin',
                [item['gt']['path'], 'src/data_loader.cpp: LoadGroundTruthMiluv'],
                'Relation to PX4 IMU and either UWB tag remains unverified.'),
            'tag_imu_extrinsics': extrinsics,
            'gt_extrinsics': unknown(pointer,
                                     'Mocap-rigid-body to IMU/tag transform was not closed locally.'),
            'time_units': observed({'uwb': 's', 'imu': 's', 'gt': 's'}, pointer,
                                   'CSV timestamp columns contain finite seconds.'),
            'time_synchronization': observed(
                {'uwb_field': 'uwb_range.csv:timestamp', 'imu_field': 'imu_px4.csv:timestamp',
                 'gt_field': 'mocap.csv:timestamp', 'mapping_to_seconds': 'identity',
                 'timeshift_file': item['timeshift']['path'],
                 'offset_drift_evidence': 'timeshift semantics/application state UNVERIFIED'},
                [pointer, item['timeshift']['path']],
                'A timeshift file exists, but this task does not assume it has or has not already been applied.'),
            'range_calibration': observed(
                {'raw_field': 'range_raw', 'project_loader_field': 'range', 'input_unit': 'm',
                 'evaluation_only_fields': ['gt_range', 'bias', 'bias_raw'],
                 'fixed_beta_source': None, 'independence_evidence': None},
                [pointer + '/uwb/columns', 'src/data_loader.cpp: LoadMiluvCsv'],
                'Raw/calibrated selection must be frozen before formal admission; GT/bias columns are forbidden estimator inputs.'),
            'physical_obstruction': obstruction,
            'license': unknown(pointer, 'No local dataset license file was found under data/MILUV.'),
        })
        result.append(finish(entry, [
            'Independent anchor survey provenance',
            'GT-rigid-body to IMU/tag extrinsics',
            'Timeshift application and residual clock offset/drift',
            'Raw range versus processed range selection and independent fixed beta',
            'Dataset publication/license scope',
        ] + ([] if used else ['Method-development exposure and split assignment'])))
    return result


def own_entries(data):
    result = []
    for rid, item in sorted(data.items()):
        pointer = f'experiments/datasets/five_dataset_metadata_observed.json#own_vicon/{rid}'
        used = '15-31-28' in rid
        issues = []
        checks = []
        if not item['original_bag_indexed']:
            issues.append('Original ROS bag has no index; content was scanned only after reindexing a temporary copy')
            checks.append('TEMP_COPY_REINDEX_RECOVERY')
        entry = common_entry(
            'own_vicon', rid, item, [bag_file('RAW_CONTAINER', item)],
            'ABSOLUTE_RANGE',
            [pointer + '/topics/~1nlink_linktrack_nodeframe3: nodes[].dis'],
            'DEV' if used else 'UNASSIGNED', True if used else None,
            (['experiments/HARNESS_RESULT.md and doc/ie_0911/STEP1_RESULT.md: no-obstacle recording used']
             if used else ['No recording-specific development-use ledger found; keep UNKNOWN']),
            f'own_vicon/{rid}' if used else None,
            local_validation(pointer, issues, checks))
        condition = 'NO_OBSTACLE' if 'no_obstacle' in rid else 'OBSTACLE'
        entry.update({
            'tag_ids': observed(item['tag_ids'], pointer + '/tag_ids',
                                'Scanned LinktrackNodeframe3 frame id values.'),
            'anchor_ids': observed(item['anchor_ids'], pointer + '/anchor_ids',
                                   'Scanned nodes[].id values.'),
            'anchor_coordinates_source': observed(
                {'source': 'bag Vicon rigid-body pose topics tas_uwb_1..4',
                 'first_positions_m': item['vicon_first_position_m'],
                 'frame': 'world', 'unit': 'm'},
                pointer + '/vicon_first_position_m',
                'The bag supplies observed anchor rigid-body poses; survey/averaging provenance is not independent.'),
            'imu_source': observed(
                {'path': item['raw_data_path'], 'topic': '/livox/imu',
                 'type': 'sensor_msgs/Imu', 'frame': 'livox_frame',
                 'raw_acceleration_unit': 'UNVERIFIED', 'raw_angular_velocity_unit': 'UNVERIFIED'},
                pointer + '/topics/~1livox~1imu',
                'Values are finite; device scaling and calibration must be closed before admission.'),
            'gt_source': observed(
                {'path': item['raw_data_path'], 'topic': '/vrpn_client_node/tas_uwb_0/pose',
                 'type': 'geometry_msgs/PoseStamped', 'use': 'EVALUATOR_ONLY'},
                pointer + '/topics/~1vrpn_client_node~1tas_uwb_0~1pose',
                'Vicon pose is kept outside estimator measurement inputs.'),
            'gt_reference_point': observed(
                {'frame': 'world', 'rigid_body': 'tas_uwb_0', 'reference_point': 'Vicon rigid-body origin'},
                pointer + '/topics/~1vrpn_client_node~1tas_uwb_0~1pose',
                'Physical relationship to antenna and Livox IMU is unverified.'),
            'tag_imu_extrinsics': unknown(pointer,
                                          'No independently calibrated tag-to-Livox-IMU transform was found.'),
            'gt_extrinsics': unknown(pointer,
                                     'Vicon rigid-body to tag/IMU transform was not supplied.'),
            'time_units': observed({'uwb': 's', 'imu': 's', 'gt': 's'}, pointer + '/topics',
                                   'ROS header stamps were converted as secs+nsecs*1e-9.'),
            'time_synchronization': observed(
                {'uwb_field': 'LinktrackNodeframe3.header.stamp',
                 'imu_field': 'Imu.header.stamp', 'gt_field': 'PoseStamped.header.stamp',
                 'mapping_to_seconds': 'secs + nsecs*1e-9',
                 'offset_drift_evidence': 'UNKNOWN'},
                pointer + '/topics',
                'Selected streams are monotonic; shared physical clock, offset and drift were not calibrated here.'),
            'range_calibration': observed(
                {'input_field': 'nodes[].dis', 'input_unit': 'm',
                 'applied_corrections': 'device/firmware UNKNOWN',
                 'fixed_beta_source': None, 'independence_evidence': None},
                pointer + '/topics/~1nlink_linktrack_nodeframe3',
                'No independent LOS fixed beta calibration is present.'),
            'physical_obstruction': declared(
                {'recording_condition': condition, 'granularity': 'filename/session',
                 'materials': None, 'link_time_annotation': None}, item['raw_data_path'],
                'Filename condition is preserved without treating it as per-observation truth.'),
            'license': unknown(pointer, 'No recording-specific data license/publication authorization was found.'),
        })
        unresolved = [
            'Independent anchor coordinate survey',
            'Tag-IMU and Vicon-rigid-body extrinsics',
            'Livox IMU raw acceleration/gyro unit and calibration provenance',
            'Clock offset/drift validation',
            'Independent LOS fixed beta calibration',
            'Recording publication/license scope',
        ]
        if not used:
            unresolved.append('Method-development exposure and split assignment')
        if issues:
            unresolved.append('Original bag is unindexed; only a temporary recovered copy was content-scanned')
        result.append(finish(entry, unresolved))
    return result


def sfuise_entries(data):
    result = []
    for rid, item in sorted(data.items()):
        pointer = f'experiments/datasets/five_dataset_metadata_observed.json#SFUISE/{rid}'
        config = f'config/paper/ie0911/sfuise_walk{rid[-1]}.yaml'
        entry = common_entry(
            'SFUISE', rid, item, [bag_file('RAW_CONTAINER', item)],
            'ABSOLUTE_TOA',
            ['https://github.com/KIT-ISAS/SFUISE#usage',
             pointer + '/topics/~1rtls_flares'],
            'DEV', True,
            ['doc/ie_sprint/STATUS.md: ISAS Walk recordings used throughout method development'],
            f'SFUISE/{rid}', local_validation(pointer))
        entry.update({
            'tag_ids': observed(item['tag_ids'], pointer + '/tag_ids',
                                'All /rtls_flares message IDs scanned.'),
            'anchor_ids': observed(item['anchor_ids'], pointer + '/anchor_ids',
                                   'All ranges[].id values scanned, including protocol-invalid observations.'),
            'anchor_coordinates_source': observed(
                {'source': 'bag:/anchor_list anchor[].position', 'frame': 'dataset anchor frame',
                 'unit': 'm', 'first_message_coordinates_m': item['anchor_coordinates_first_message_m']},
                pointer + '/anchor_coordinates_first_message_m',
                'This is not evidence of an independent survey; full-bag averaging is not prefix-safe by default.'),
            'imu_source': observed(
                {'path': item['raw_data_path'], 'topic': '/waveshare_sense_hat_b',
                 'type': 'sensor_msgs/Imu', 'frame': 'base_link',
                 'acceleration_unit': 'm/s^2', 'angular_velocity_unit': 'rad/s'},
                pointer + '/topics/~1waveshare_sense_hat_b',
                'Sensor source is content-checked; calibration provenance remains external.'),
            'gt_source': observed(
                {'path': item['raw_data_path'], 'topic': '/vive/transform/tracker_1_ref',
                 'type': 'geometry_msgs/TransformStamped', 'use': 'EVALUATOR_ONLY'},
                pointer + '/topics/~1vive~1transform~1tracker_1_ref',
                'GT is inventoried for evaluation and is not an estimator input.'),
            'gt_reference_point': observed(
                {'parent_frame': 'vive/reference', 'child_frame': 'vive/tracker_1',
                 'reference_point': 'Vive tracker frame origin'},
                pointer + '/topics/~1vive~1transform~1tracker_1_ref/frames',
                'Tracker origin is not assumed to coincide with tag or IMU.'),
            'tag_imu_extrinsics': declared(
                {'translation_m': [0.1, -0.025, 0.0], 'rotation_xyzw': None,
                 'convention': 'p_tag_world=p_imu_world+R_world_imu*lever_imu'},
                config, 'Existing development configuration; independent calibration and rotation are unverified.'),
            'gt_extrinsics': unknown(pointer,
                                     'Vive tracker-to-tag/IMU transform was not supplied.'),
            'time_units': observed({'uwb': 's', 'imu': 's', 'gt': 's'}, pointer + '/topics',
                                   'ROS header stamps use secs+nsecs.'),
            'time_synchronization': observed(
                {'uwb_field': 'RTLSFlares.header.stamp', 'imu_field': 'Imu.header.stamp',
                 'gt_field': 'TransformStamped.header.stamp',
                 'mapping_to_seconds': 'secs + nsecs*1e-9',
                 'offset_drift_evidence': 'UNKNOWN'}, pointer + '/topics',
                'Monotonic timestamps do not prove a shared physical clock or zero offset/drift.'),
            'range_calibration': declared(
                {'input_field': 'ranges[].range', 'input_unit': 'm',
                 'applied_corrections': 'loader copies field unchanged; firmware processing UNKNOWN',
                 'fixed_beta_source': None, 'independence_evidence': None},
                [pointer + '/topics/~1rtls_flares', config],
                'fixed_beta_by_link is empty; this is MISSING_CALIBRATION rather than calibrated zero.'),
            'physical_obstruction': unknown(pointer,
                                            'No physical obstruction annotation is present in the local bags.'),
            'license': unknown('https://github.com/KIT-ISAS/SFUISE#license',
                               'Repository source license does not by itself close local bag redistribution rights.'),
        })
        result.append(finish(entry, [
            'Independent anchor survey provenance',
            'Vive tracker-to-tag/IMU transform',
            'Clock offset/drift validation',
            'Independent LOS fixed beta calibration',
            'Dataset bag publication/license scope',
        ]))
    return result


def starloc_entries(data):
    result = []
    for rid, item in sorted(data.items()):
        pointer = f'experiments/datasets/five_dataset_metadata_observed.json#starloc/{rid}'
        files = [file_item('UWB', item['uwb']),
                 file_item('IMU', item['imu_and_embedded_gt']),
                 file_item('GT', item['imu_and_embedded_gt']),
                 file_item('CALIBRATION', item['calibration']),
                 file_item('ANCHOR_COORDINATES', item['anchor_coordinates'])]
        entry = common_entry(
            'starloc', rid, item['uwb'], files, 'ABSOLUTE_RANGE',
            ['data/starloc/README.md: range is raw distance in meters', pointer + '/uwb/columns'],
            'UNASSIGNED', None,
            ['No recording-specific method-development ledger found; keep UNKNOWN'],
            None, local_validation(pointer))
        entry.update({
            'tag_ids': observed(item['uwb']['ids']['from_id'], pointer + '/uwb/ids/from_id',
                                'All from_id values scanned from uwb.csv.'),
            'anchor_ids': observed(item['uwb']['ids']['to_id'], pointer + '/uwb/ids/to_id',
                                   'All to_id values scanned from uwb.csv.'),
            'anchor_coordinates_source': observed(
                {'source': item['anchor_coordinates']['path'],
                 'landmark_version': item['dataset_params']['landmarks'],
                 'frame': 'Vicon world', 'unit': 'm'},
                [pointer + '/anchor_coordinates', 'data/starloc/dataset_params.json'],
                'Marker coordinates are supplied by the dataset; independent survey uncertainty is not stated locally.'),
            'imu_source': observed(
                {'path': item['imu_and_embedded_gt']['path'],
                 'stream': 'stereo-camera internal IMU CSV',
                 'angular_velocity_unit': 'rad/s', 'angular_velocity_frame': 'IMU',
                 'linear_acceleration_unit': 'm/s^2', 'linear_acceleration_frame': 'rig'},
                ['data/starloc/README.md', pointer + '/imu_and_embedded_gt/columns'],
                'Mixed declared frames must be handled explicitly by any later adapter.'),
            'gt_source': observed(
                {'path': item['imu_and_embedded_gt']['path'],
                 'columns': ['x', 'y', 'z', 'w', 'rot_x', 'rot_y', 'rot_z'],
                 'source': 'Vicon rig pose', 'use': 'EVALUATOR_ONLY'},
                ['data/starloc/README.md', pointer + '/imu_and_embedded_gt/columns'],
                'GT columns coexist with IMU and must be stripped from future measurement-only estimator input.'),
            'gt_reference_point': declared(
                'Sensor rig frame F_r origin tracked by Vicon', 'data/starloc/README.md#frames',
                'The UWB tag and IMU are distinct reference points.'),
            'tag_imu_extrinsics': unknown(pointer,
                                          'Per-tag transform to IMU is not explicitly supplied in the local files.'),
            'gt_extrinsics': declared(
                {'source': item['calibration']['path'],
                 'available_transforms': ['tf_cam_imu', 'tf_cam_rig'],
                 'composition': 'rig-to-IMU can be composed only under the README frame convention'},
                ['data/starloc/README.md', item['calibration']['path']],
                'Values are finite; transform direction and tag relation must be made explicit in an adapter.'),
            'time_units': observed({'uwb': 's', 'imu': 's', 'gt': 's'}, pointer,
                                   'All selected CSV time_s columns are finite seconds.'),
            'time_synchronization': declared(
                {'uwb_field': 'uwb.csv:time_s', 'imu_field': 'imu.csv:time_s',
                 'gt_field': 'embedded Vicon pose at each CSV row time_s',
                 'epoch': 'seconds since first Vicon measurement',
                 'mapping_to_seconds': 'identity', 'residual_offset_drift_evidence': 'UNKNOWN'},
                ['data/starloc/README.md', pointer],
                'Common published epoch does not quantify residual clock offset or drift.'),
            'range_calibration': observed(
                {'raw_field': 'range', 'input_unit': 'm',
                 'optional_calibrated_field': ('range_calib' if 'range_calib' in item['uwb']['columns'] else None),
                 'evaluation_only_fields': ['gt_range', 'bias'],
                 'selected_for_inventory': 'range'},
                [pointer + '/uwb/columns', 'data/starloc/README.md'],
                'The inventory identifies raw range; no future adapter may consume GT/bias or silently switch calibration.'),
            'physical_obstruction': unknown(pointer,
                                            'No physical obstruction material/link/time annotation was found.'),
            'license': unknown(pointer, 'No local LICENSE/COPYING file was found in the starloc checkout.'),
        })
        result.append(finish(entry, [
            'Per-tag to IMU transform',
            'Explicit rig/camera/IMU transform direction for the future adapter',
            'Independent anchor survey uncertainty',
            'Residual clock offset/drift validation',
            'Independent LOS fixed beta calibration',
            'Recording-specific method-development exposure and split assignment',
            'Dataset redistribution license scope',
        ]))
    return result


def main():
    data = json.loads(ARTIFACT.read_text())
    recordings = []
    recordings.extend(huec_entries(data['HUEC']))
    recordings.extend(miluv_entries(data['MILUV']))
    recordings.extend(own_entries(data['own_vicon']))
    recordings.extend(sfuise_entries(data['SFUISE']))
    recordings.extend(starloc_entries(data['starloc']))
    document = {
        'schema_version': 'icra_dataset_manifest_v2',
        'path_base': 'repository_root',
        'measurement_policy': {
            'accepted': ['ABSOLUTE_TOA', 'ABSOLUTE_TWR', 'ABSOLUTE_RANGE'],
            'rejected': ['TDOA', 'TIME_DIFFERENCE_OF_ARRIVAL', 'RANGE_DIFFERENCE',
                         'ANCHOR_PAIR_DISTANCE_DIFFERENCE'],
        },
        'inventory_artifact': {
            'path': str(ARTIFACT.relative_to(ROOT)),
            'sha256': digest(ARTIFACT),
        },
        'recordings': recordings,
    }
    OUTPUT.write_text(json.dumps(document, indent=2, ensure_ascii=False,
                                 allow_nan=False) + '\n')
    print(f'wrote {OUTPUT.relative_to(ROOT)} with {len(recordings)} recordings')


if __name__ == '__main__':
    main()
