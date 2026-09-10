#!/usr/bin/env python3
"""Evaluation-side consistency checks. Never imported by the estimator."""
import argparse
import csv
import json
from pathlib import Path
import tempfile

import numpy as np

import generate_synthetic_input as gen


def rows(path):
    with Path(path).open(newline='') as f:
        return list(csv.DictReader(f))


def matrix(table, keys):
    return np.array([[float(r[k]) for k in keys] for r in table])


def require_close(actual, expected, tolerance, message):
    error = float(np.max(np.abs(np.asarray(actual)-np.asarray(expected))))
    if not np.isfinite(error) or error > tolerance:
        raise AssertionError(f'{message}: error {error} > {tolerance}')
    return error


def verify(root):
    root = Path(root)
    manifest = json.loads((root/'generation_manifest.json').read_text())
    for path, digest in manifest['payload_sha256'].items():
        assert gen.file_sha(root/path) == digest, ('payload', path)
    assert gen.sha(gen.canonical(manifest['base'])) == manifest['base_source_sha256']
    assert manifest['base']['role'] == 'development'
    truth_m = rows(root/'evaluation/motion.csv')
    p = matrix(truth_m, ['px_m','py_m','pz_m'])
    v = matrix(truth_m, ['vx_mps','vy_mps','vz_mps'])
    a = matrix(truth_m, ['ax_mps2','ay_mps2','az_mps2'])
    w = matrix(truth_m, ['wx_radps','wy_radps','wz_radps'])
    f = matrix(truth_m, ['fx_mps2','fy_mps2','fz_mps2'])
    yaw = matrix(truth_m, ['yaw_rad'])[:,0]
    ts = matrix(truth_m, ['time_s'])[:,0]
    quats = matrix(truth_m, ['qw','qx','qy','qz'])
    R = np.array([[[np.cos(y),-np.sin(y),0.], [np.sin(y),np.cos(y),0.], [0.,0.,1.]] for y in yaw])
    stats = {}
    stats['time_max_error_s'] = require_close(ts, np.arange(1601)/200, 0., 'IMU time')
    stats['rotation_orthogonality'] = require_close(R.transpose(0,2,1) @ R, np.eye(3), 1e-14, 'rotation')
    require_close(np.linalg.det(R), 1., 1e-14, 'det R')
    require_close(np.linalg.norm(quats,axis=1), 1., 1e-14, 'quaternion norm')
    require_close(2*np.arctan2(quats[:,3],quats[:,0]), yaw, 1e-14, 'quaternion convention')
    stats['force_world_closure_mps2'] = require_close(
        np.einsum('nij,nj->ni', R, f)+[0,0,-9.81], a, 1e-12, 'specific force gravity/sign')
    # Independent central differences, deliberately excluding only the C2 static join.
    ve, ae, we = [], [], []
    for t in np.linspace(.1, 7.9, 79):
        h = 1e-4
        if abs(t-gen.MOTION['static_s']) < h:
            continue  # C2 join: centered derivative has O(h), not O(h²), error.
        m, lo, hi = gen.motion(t), gen.motion(t-h), gen.motion(t+h)
        ve.append(require_close((hi[0]-lo[0])/(2*h), m[1], 1e-7, 'analytic velocity FD'))
        ae.append(require_close((hi[1]-lo[1])/(2*h), m[2], 1e-7, 'analytic acceleration FD'))
        skew = m[3].T @ ((hi[3]-lo[3])/(2*h))
        we.append(require_close([skew[2,1],skew[0,2],skew[1,0]], m[4], 1e-7, 'body omega FD'))
    join = gen.motion(gen.MOTION['static_s'])
    for side in (-1,1):
        adjacent = gen.motion(gen.MOTION['static_s']+side*1e-8)
        for index in (0,1,2):
            require_close(adjacent[index], join[index], 2e-8, 'C2 join continuity')
    stats.update(velocity_fd_max=max(ve), acceleration_fd_max=max(ae), omega_fd_max=max(we))
    # Noise-free first-order inertial propagation, independent of analytic v/p evaluation.
    pi, vi, Ri = p[0].copy(), v[0].copy(), R[0].copy()
    dt = .005
    pos_err, vel_err = 0., 0.
    for j in range(1600):
        ai = Ri @ f[j] + [0.,0.,-9.81]
        pi += vi*dt + .5*ai*dt*dt
        vi += ai*dt
        phi = w[j,2]*dt
        Ri = Ri @ np.array([[np.cos(phi),-np.sin(phi),0.], [np.sin(phi),np.cos(phi),0.], [0.,0.,1.]])
        pos_err = max(pos_err,float(np.linalg.norm(pi-p[j+1])))
        vel_err = max(vel_err,float(np.linalg.norm(vi-v[j+1])))
    assert pos_err < .003 and vel_err < .003, (pos_err,vel_err)
    stats.update(inertial_forward_max_position_error_m=pos_err,
                 inertial_forward_max_velocity_error_mps=vel_err)
    it = rows(root/'evaluation/imu_latent.csv')
    raw_imu = rows(root/'raw/los/imu.csv')
    assert len(it) == len(raw_imu) == 1601
    assert all(r['has_orientation']=='0' and [r[k] for k in ('qw','qx','qy','qz')]==['1','0','0','0'] for r in raw_imu)
    require_close(matrix(it,['ba_x','ba_y','ba_z']), gen.BA, 0., 'constant ba')
    require_close(matrix(it,['bg_x','bg_y','bg_z']), gen.BG, 0., 'constant bg')
    stats['raw_acc_closure_mps2'] = require_close(matrix(raw_imu,['acc_x_mps2','acc_y_mps2','acc_z_mps2']),
        f+matrix(it,['ba_x','ba_y','ba_z'])+matrix(it,['noise_ax','noise_ay','noise_az']), 1e-12, 'raw accel')
    stats['raw_gyro_closure_radps'] = require_close(matrix(raw_imu,['gyro_x_radps','gyro_y_radps','gyro_z_radps']),
        w+matrix(it,['bg_x','bg_y','bg_z'])+matrix(it,['noise_gx','noise_gy','noise_gz']), 1e-12, 'raw gyro')
    closure = []
    noise_by_case = []
    id_sets = []
    for scenario in ('los','step','ramp'):
        raw = root/'raw'/scenario
        cm = json.loads((raw/'input_manifest.json').read_text())
        assert gen.cache_id(cm) == cm['cache_id']
        assert cm['base_recording_id'] == manifest['recording_id']
        assert set(p.name for p in raw.iterdir()) == {'input_manifest.json','imu.csv','uwb_observations.csv'}
        assert gen.file_sha(raw/'imu.csv') == cm['imu_sha256'] == gen.file_sha(root/'raw/los/imu.csv')
        assert gen.file_sha(raw/'uwb_observations.csv') == cm['uwb_sha256']
        rr, tr = rows(raw/'uwb_observations.csv'), rows(root/'evaluation'/f'{scenario}_range_truth.csv')
        assert len(rr) == len(tr) == 328
        assert len(set(r['obs_id'] for r in rr)) == 328
        assert [r['obs_id'] for r in rr] == [r['obs_id'] for r in tr]
        id_sets.append([r['obs_id'] for r in rr])
        for j, (r,b) in enumerate(zip(rr,tr)):
            k, l = divmod(j,8)
            t = k/5
            anchor = np.array(gen.ANCHORS[l])
            assert int(r['obs_id']) == gen.observation_id(manifest['recording_id'], k,l)
            assert [int(r[x]) for x in ('source_message_index','source_range_index','source_observation_index','tag_id','anchor_id')] == [k,l,j,0,l+1]
            assert float(r['sensor_time_s']) == float(b['time_s']) == t
            assert r['source_valid']=='1'
            # Evaluate geometry from stored motion at the matching IMU sample, not gen.motion.
            h_geom = np.linalg.norm(p[k*40] + R[k*40]@np.array(gen.SPEC['lever_m']) - anchor)
            require_close(float(b['geometric_range_m']), h_geom, 1e-12, 'stored geometry')
            require_close(float(b['fixed_beta_m']), gen.BETA[l], 0., 'beta identity')
            require_close(float(b['total_dynamic_latent_bias_m']), gen.latent_bias(scenario,t,l+1), 0., 'total bias')
            require_close(float(b['total_deterministic_offset_m']), gen.BETA[l]+float(b['total_dynamic_latent_bias_m']), 1e-12, 'offset components')
            closure.append(require_close(float(r['observed_range_m']),
                h_geom+gen.BETA[l]+float(b['total_dynamic_latent_bias_m'])+float(b['range_noise_m']), 1e-12, 'raw UWB'))
        noise_by_case.append(matrix(tr,['range_noise_m']))
    assert id_sets[0] == id_sets[1] == id_sets[2]
    require_close(noise_by_case[0], noise_by_case[1], 0., 'paired noise')
    require_close(noise_by_case[0], noise_by_case[2], 0., 'paired noise')
    stats.update(uwb_measurement_max_error_m=max(closure), observations_per_scenario=328,
                 imu_samples=1601, max_speed_mps=float(np.max(np.linalg.norm(v,axis=1))),
                 max_acceleration_mps2=float(np.max(np.linalg.norm(a,axis=1))),
                 status='PASS')
    return stats


def metamorphic(root):
    seed = json.loads((root/'generation_manifest.json').read_text())['base']['seed']
    with tempfile.TemporaryDirectory(prefix='a10-generator-check-') as tmp:
        tmp = Path(tmp)
        gen.generate(tmp/'same',seed)
        gen.generate(tmp/'different',seed+1)
        for p in root.rglob('*'):
            if p.is_file():
                assert p.read_bytes() == (tmp/'same'/p.relative_to(root)).read_bytes(), p
        verify(tmp/'different')
        for filename, fields in [('uwb_observations.csv',['observed_range_m']),
                                 ('imu.csv',['acc_x_mps2','acc_y_mps2','acc_z_mps2']),
                                 ('imu.csv',['gyro_x_radps','gyro_y_radps','gyro_z_radps'])]:
            a=matrix(rows(root/'raw/los'/filename), fields)
            b=matrix(rows(tmp/'different/raw/los'/filename), fields)
            assert np.all(a != b), (filename,fields)
        assert (root/'evaluation/motion.csv').read_bytes() == (tmp/'different/evaluation/motion.csv').read_bytes()
        # Preserve manifest hash integrity while corrupting a truth numeric value: closure must reject it.
        path=tmp/'same/evaluation/step_range_truth.csv'
        table=rows(path)
        table[0]['range_noise_m']=str(float(table[0]['range_noise_m'])+.1)
        gen.write_csv(path,list(table[0]),[list(r.values()) for r in table])
        mp=tmp/'same/generation_manifest.json'
        manifest=json.loads(mp.read_text())
        manifest['payload_sha256']['evaluation/step_range_truth.csv']=gen.file_sha(path)
        gen.write_json(mp,manifest)
        try:
            verify(tmp/'same')
        except AssertionError as error:
            corruption=str(error)
        else:
            raise AssertionError('closure failed to reject rehashed truth corruption')
        try:
            gen.generate(root,seed)
        except FileExistsError:
            pass
        else:
            raise AssertionError('overwrite not rejected')
    return dict(same_seed_all_bytes=True,different_seed_all_random_channels_changed=True,
                different_seed_motion_identical=True, rehashed_truth_corruption_rejected=corruption,
                overwrite_rejected=True, generator_only_extra_seed=seed+1)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-root', type=Path, required=True)
    p.add_argument('--report', type=Path, required=True)
    args=p.parse_args()
    report=dict(consistency=verify(args.input_root),metamorphic=metamorphic(args.input_root))
    gen.write_json(args.report,report)
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
