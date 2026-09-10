#!/usr/bin/env python3
"""Self-contained analytic UWB/IMU generator; no estimation or GT initialization.

The legacy development invocation remains unchanged. The opt-in validation
invocation is closed over T10_A10_SPLIT_PROPOSAL.json reservations and emits a
truth-free context beside every raw scenario.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import tempfile

import numpy as np

VERSION = 'uifgo_total_synthetic_v1'
VALIDATION_CONTEXT_SCHEMA = 'uifgo_t10_validation_admission_v1'
MOTION = dict(base_trajectory_id='a10_dev_turn_01', static_s=2.2, tau_s=.8,
              rate_radps=.45, x_amplitude_m=.8, y_amplitude_m=.6,
              z_amplitude_m=.1, z_origin_m=1., yaw_amplitude_rad=.4)
ANCHORS = [[x, y, z] for x in (-3., 3.) for y in (-3., 3.) for z in (0., 3.)]
BETA = [.03, -.02, .04, -.01, .02, -.03, .01, -.04]
BA = [.003, -.002, .004]
BG = [.0002, -.0001, .00015]
SPEC = dict(duration_s=8., imu_hz=200, uwb_hz=5, gravity_world_mps2=[0., 0., -9.81],
            lever_m=[0., 0., 0.], anchors_m=ANCHORS, beta_m=BETA,
            accel_bias_mps2=BA, gyro_bias_radps=BG, range_sigma_m=.05,
            accel_noise_density=.002, gyro_noise_density=.0002,
            bias_random_walk_generated=False, time_offset_s=0., clock_drift=0.,
            extra_random_nlos=False, packet_dropouts=False)
IMU_HEADER = ('source_index,sensor_time_s,acc_x_mps2,acc_y_mps2,acc_z_mps2,'
              'gyro_x_radps,gyro_y_radps,gyro_z_radps,has_orientation,qw,qx,qy,qz')
UWB_HEADER = ('obs_id,source_message_index,source_range_index,source_observation_index,'
              'sensor_time_s,tag_id,anchor_id,observed_range_m,fp_rssi_dbm,rx_rssi_dbm,'
              'source_valid,source_validity_reason_hex')


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def sha(data):
    return 'sha256:' + hashlib.sha256(data).hexdigest()


def file_sha(path):
    return sha(Path(path).read_bytes())


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')


def write_csv(path, header, rows):
    with Path(path).open('w', newline='') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(header.split(',') if isinstance(header, str) else header)
        for row in rows:
            writer.writerow([format(float(v), '.17g') if isinstance(v, (float, np.floating)) else v
                             for v in row])


def observation_id(recording, message, range_index):
    # Matches src/paper_input.cpp (its nonstandard FNV offset is intentional).
    value = 1469598103934665603
    for byte in f'{recording}|source_message={message}|source_range={range_index}'.encode():
        value = ((value ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return value


def cache_id(manifest):
    m = manifest
    lines = [m['schema'], m['base_recording_id'], m['base_source_sha256'],
             format(m['recording_time_origin_s'], '.17g'),
             'imu_units=m/s^2,rad/s,quaternion_wxyz', 'uwb_units=s,m,dBm',
             'time_basis=sensor_time_from_recording_origin', 'grouping=source_message_index',
             m['imu_sha256'], m['uwb_sha256'], str(m['imu_count']),
             str(m['uwb_observation_count']), str(m['uwb_message_count'])]
    return sha(('\n'.join(lines) + '\n').encode())


def motion(t, spec=MOTION):
    u = max(0., float(t) - spec['static_s'])
    tau, rate = spec['tau_s'], spec['rate_radps']
    e = math.exp(-u/tau)
    theta = rate*(u - 2*tau*(1-e) + tau/2*(1-e*e))
    d = rate*(1-e)**2
    dd = 2*rate*(1-e)*e/tau
    a, b, c = (spec[k] for k in ('x_amplitude_m', 'y_amplitude_m', 'z_amplitude_m'))
    p = np.array([a*math.sin(theta), b*(1-math.cos(theta)),
                  spec['z_origin_m'] + c*(1-math.cos(2*theta))])
    dp = np.array([a*math.cos(theta), b*math.sin(theta), 2*c*math.sin(2*theta)])
    ddp = np.array([-a*math.sin(theta), b*math.cos(theta), 4*c*math.cos(2*theta)])
    velocity, accel = dp*d, ddp*d*d + dp*dd
    yaw = spec['yaw_amplitude_rad']*math.sin(theta)
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0.],
                  [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
    omega = np.array([0., 0., spec['yaw_amplitude_rad']*math.cos(theta)*d])
    force = R.T @ (accel - np.array(SPEC['gravity_world_mps2']))
    return p, velocity, accel, R, omega, force, yaw


def latent_bias(scenario, t, anchor):
    if scenario == 'los' or not 3. <= t <= 6.:
        return 0.
    if scenario in ('step', 'step2') and anchor in (1, 2):
        return (.6, .9)[anchor-1]
    if scenario == 'step1' and anchor == 1:
        return .6
    if scenario == 'step3' and anchor in (1, 2, 3):
        return (.6, .9, .7)[anchor-1]
    if scenario in ('ramp', 'ramp_steep') and anchor in (1, 2):
        return (.2, .3)[anchor-1] + (.2, .25)[anchor-1]*(t-3.)
    if scenario == 'ramp_gentle' and anchor in (1, 2):
        return (.2, .3)[anchor-1] + (.05, .075)[anchor-1]*(t-3.)
    if scenario in ('step', 'ramp', 'step1', 'step2', 'step3',
                    'ramp_gentle', 'ramp_steep'):
        return 0.
    raise ValueError('unknown scenario')


def generate(output, seed, role='development', motion_spec=None, scenarios=None,
             validation_identity=None):
    output = Path(output).resolve()
    if isinstance(seed, bool) or not isinstance(seed, int) or seed <= 0:
        raise ValueError('seed must be a positive integer')
    if output.exists():
        raise FileExistsError(f'refusing overwrite: {output}')
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='.synthetic-', dir=output.parent))
    try:
        (staging/'raw').mkdir()
        (staging/'evaluation').mkdir()
        source_sha = file_sha(__file__)
        motion_spec = dict(MOTION if motion_spec is None else motion_spec)
        scenarios = tuple(('los', 'step', 'ramp') if scenarios is None else scenarios)
        base = dict(generator_version=VERSION, generator_sha256=source_sha, motion=motion_spec,
                    sensor_assumptions=SPEC, seed=seed, role=role,
                    rng='NumPy PCG64 SeedSequence([seed,stream_id]); streams 0/1/2 UWB/accel/gyro',
                    numpy_version=np.__version__)
        base_sha = sha(canonical(base))
        recording = 'synthetic-' + base_sha.split(':')[1]
        it = np.arange(1601, dtype=float)/200
        ut = np.arange(41, dtype=float)/5
        rng = [np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed, s])))
               for s in range(3)]
        range_noise = rng[0].normal(0., .05, (41, 8))
        acc_noise = rng[1].normal(0., .002/math.sqrt(.005), (1601, 3))
        gyro_noise = rng[2].normal(0., .0002/math.sqrt(.005), (1601, 3))
        imu_rows, motion_rows, imu_truth = [], [], []
        for j, t in enumerate(it):
            p, v, a, R, w, f, yaw = motion(t, motion_spec)
            imu_rows.append([j, t, *(f+BA+acc_noise[j]), *(w+BG+gyro_noise[j]), 0, 1., 0., 0., 0.])
            motion_rows.append([t, *p, *v, *a, yaw, math.cos(yaw/2), 0., 0., math.sin(yaw/2), *w, *f])
            imu_truth.append([j, t, *BA, *BG, *acc_noise[j], *gyro_noise[j]])
        write_csv(staging/'evaluation/motion.csv',
                  'time_s,px_m,py_m,pz_m,vx_mps,vy_mps,vz_mps,ax_mps2,ay_mps2,az_mps2,yaw_rad,'
                  'qw,qx,qy,qz,wx_radps,wy_radps,wz_radps,fx_mps2,fy_mps2,fz_mps2', motion_rows)
        write_csv(staging/'evaluation/imu_latent.csv',
                  'source_index,time_s,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z,noise_ax,noise_ay,noise_az,'
                  'noise_gx,noise_gy,noise_gz', imu_truth)
        cases = {}
        for scenario in scenarios:
            raw = staging/'raw'/scenario
            raw.mkdir()
            write_csv(raw/'imu.csv', IMU_HEADER, imu_rows)
            rows, truth = [], []
            for k, t in enumerate(ut):
                p, _, _, R, _, _, _ = motion(t, motion_spec)
                antenna = p + R @ np.array(SPEC['lever_m'])
                for r, anchor in enumerate(ANCHORS):
                    obs_id = observation_id(recording, k, r)
                    h = float(np.linalg.norm(antenna-anchor))
                    bias = latent_bias(scenario, t, r+1)
                    rows.append([obs_id, k, r, 8*k+r, t, 0, r+1,
                                 h+BETA[r]+bias+range_noise[k,r], -80., -78., 1, ''])
                    truth.append([obs_id, t, 0, r+1, h, BETA[r], bias,
                                  BETA[r]+bias, range_noise[k,r]])
            write_csv(raw/'uwb_observations.csv', UWB_HEADER, rows)
            write_csv(staging/'evaluation'/f'{scenario}_range_truth.csv',
                      'obs_id,time_s,tag_id,anchor_id,geometric_range_m,fixed_beta_m,'
                      'total_dynamic_latent_bias_m,total_deterministic_offset_m,range_noise_m', truth)
            manifest = dict(schema='t07_estimator_cache_v1', base_recording_id=recording,
                            base_source_sha256=base_sha, recording_time_origin_s=0.,
                            time_basis='sensor_time_from_recording_origin',
                            imu_units='acc_mps2,gyro_radps,orientation_quaternion_wxyz',
                            uwb_units='time_s,range_m,rssi_dbm',
                            uwb_message_grouping='source_message_index', imu_file='imu.csv',
                            imu_sha256=file_sha(raw/'imu.csv'), imu_count=1601,
                            uwb_file='uwb_observations.csv', uwb_sha256=file_sha(raw/'uwb_observations.csv'),
                            uwb_observation_count=328, uwb_message_count=41)
            manifest['cache_id'] = cache_id(manifest)
            write_json(raw/'input_manifest.json', manifest)
            context_file = None
            if role == 'validation':
                if not validation_identity:
                    raise ValueError('validation identity is required')
                context = dict(validation_identity,
                    schema=VALIDATION_CONTEXT_SCHEMA, role='validation',
                    base_trajectory_id=motion_spec['base_trajectory_id'], seed=seed,
                    scenario_id=scenario, recording_id=recording,
                    cache_id=manifest['cache_id'],
                    raw_input_manifest_sha256=file_sha(raw/'input_manifest.json'),
                    nominal_noise_sha256=sha(canonical({
                        key: SPEC[key] for key in ('range_sigma_m','accel_noise_density',
                                                  'gyro_noise_density')})),
                    synthetic_assumptions_sha256=sha(canonical(SPEC)),
                    truth_paths_forbidden=True)
                context_file = 'validation_context.json'
                write_json(raw/context_file, context)
            cases[scenario] = dict(cache_id=manifest['cache_id'],
                                   input=f'raw/{scenario}/input_manifest.json',
                                   context=(f'raw/{scenario}/{context_file}'
                                            if context_file else None),
                                   truth=f'evaluation/{scenario}_range_truth.csv',
                                   truth_semantics='TOTAL_SYNTHETIC_LATENT_DYNAMIC_EXCESS_PATH')
        write_json(staging/'generation_manifest.json', dict(
            schema=VERSION, base=base, base_source_sha256=base_sha, recording_id=recording,
            python_version=platform.python_version(), calibration_provenance='EXACT_SYNTHETIC_ASSUMPTIONS',
            scenarios=cases, validation_identity=validation_identity,
            dynamic_bias_protocol=dict(interval_closed_s=[3.,6.],
                step1_m={1:.6}, step2_m={1:.6,2:.9}, step3_m={1:.6,2:.9,3:.7},
                ramp_start_m={1:.2,2:.3}, ramp_gentle_slope_mps={1:.05,2:.075},
                ramp_steep_slope_mps={1:.2,2:.25}),
            payload_sha256={str(p.relative_to(staging)): file_sha(p)
                            for p in sorted(staging.rglob('*')) if p.is_file()}))
        staging.rename(output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return output/'generation_manifest.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--role', choices=['development', 'validation'], default='development')
    parser.add_argument('--split-proposal', type=Path)
    parser.add_argument('--reservation-key')
    parser.add_argument('--admission-sha256')
    parser.add_argument('--budget-id')
    parser.add_argument('--metric-sha256')
    args = parser.parse_args()
    if args.role == 'development':
        if any((args.split_proposal, args.reservation_key, args.admission_sha256,
                args.budget_id, args.metric_sha256)):
            raise ValueError('validation arguments are forbidden for development')
        print(generate(args.output_root, args.seed))
        return
    required = (args.split_proposal, args.reservation_key, args.admission_sha256,
                args.budget_id, args.metric_sha256)
    if any(item is None for item in required):
        raise ValueError('validation requires split/reservation/admission/budget/metric')
    split = json.loads(args.split_proposal.read_text())
    records = [record for record in split['records']
               if record['reservation_key'] == args.reservation_key]
    if len(records) != 1:
        raise ValueError('reservation must identify exactly one split record')
    record = records[0]
    if record['role'] != 'validation' or record['status'] != 'NOT_GENERATED_NOT_RUN':
        raise ValueError('reservation is not unused validation')
    if record['seed'] != args.seed:
        raise ValueError('seed does not match reservation')
    identity = dict(
        reservation_key=record['reservation_key'], ancestry_group=record['ancestry_group'],
        split_sha256=file_sha(args.split_proposal),
        admission_sha256='sha256:' + args.admission_sha256.replace('sha256:', '', 1),
        budget_id=args.budget_id,
        metric_implementation_sha256='sha256:' + args.metric_sha256.replace('sha256:', '', 1))
    print(generate(args.output_root, args.seed, 'validation', record['motion'],
                   record['scenario_ids'], identity))


if __name__ == '__main__':
    main()
