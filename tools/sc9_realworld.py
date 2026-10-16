#!/usr/bin/env python3
"""SC9 native integration checks. Intermediate evidence is owned by .cache/sc9."""
import argparse
import csv
import hashlib
import json
import struct
import subprocess
from pathlib import Path

from sc4_system_research import ROOT, BIN, environment

CACHE = ROOT / '.cache/sc9'
# Fixed before observing these runs. Both fault rows extend past injection and x0 exit.
LEGACY = [('S0', 35), ('F0', 76), ('F3', 76), ('S3', 35)]
SEMANTIC = '''epoch mutations updates qualified status recovery version generation
parent boundary x0 source_prefix risk q shared_q alerts uwb_alarm bridge rejoined
excluded repair removed_priors removed_groups hpl vpl bias_width slots live ledger
replay capsules width_x width_y width_z vwidth_x vwidth_y vwidth_z rwidth_x rwidth_y
rwidth_z bwidth_0 bwidth_1 bwidth_2 bwidth_3 bwidth_4 bwidth_5 tag_x tag_y tag_z px py
pz vx vy vz qw qx qy qz bax bay baz bgx bgy bgz stamp_ns fd_status imu_alarm
alert_generation first_imu_alarm_ns imu_active imu_ended imu_excluded uwb_active
imu_no_active imu_q temporal_alerts active_states proof_explicit support_boxes
support_frontier bound_method joint_branches joint_empty joint_frontier joint_fallback
formal_eligible publication_protected application_available'''.split()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identity():
    env = environment()
    binary = BIN / 'epoch_raim_runner'
    loaded = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
    dso = next(line.split('=>')[1].split()[0] for line in loaded.splitlines()
               if 'libuwb_imu_pl.so =>' in line)
    return dict(head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                                             text=True).strip(),
                elf=sha(binary), dso=sha(dso), loaded_dso=dso,
                requirements=sha(ROOT / 'docs/requirements/sc9/SC9_REALWORLD_INTEGRATION_REQUIREMENTS.md'))


def legacy(freeze):
    CACHE.mkdir(parents=True, exist_ok=True)
    baseline = CACHE / 'legacy-baseline.json'
    if freeze and baseline.exists():
        raise RuntimeError('baseline exists; refusing to overwrite pre-change evidence')
    result = dict(identity=identity(), profiles={})
    for scene, count in LEGACY:
        path = CACHE / f'legacy-{scene}-{"before" if freeze else "after"}.csv'
        command = [BIN / 'epoch_raim_runner', ROOT / 'config/sc7.yaml', path,
                   scene, 'sc6', 'on', str(count), 'tight-optimized', 'offline', '20261010']
        with path.with_suffix('.log').open('w') as log:
            subprocess.run(list(map(str, command)), cwd=ROOT, env=environment(),
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == count
        assert all(r['updates'] == r['mutations'] == r['qualified'] == '1' for r in rows)
        assert all(r['publication_protected'] == '0' for r in rows)
        assert any(r['x0'] == '0' for r in rows)
        result['profiles'][scene] = [{k: r[k] for k in SEMANTIC} for r in rows]
        print(scene, 'commits/raw', len(rows), 'repairs', sum(r['repair'] == '1' for r in rows),
              'rejoins', sum(r['rejoined'] == '1' for r in rows), flush=True)
    result['final_bound_roundoff'] = []
    if not freeze:
        old = json.loads(baseline.read_text())
        for scene, _ in LEGACY:
            for before, after in zip(old['profiles'][scene], result['profiles'][scene]):
                for key in SEMANTIC:
                    if before[key] == after[key]:
                        continue
                    # Final outward distance evaluation may move by one binary64
                    # ULP after compiler/inlining changes. Only H/V receive this
                    # narrow allowance; actions, source/receipt and state stay exact.
                    assert key in ('hpl', 'vpl'), (scene, before['epoch'], key, before[key], after[key])
                    a, b = float(before[key]), float(after[key])
                    bits = lambda v: struct.unpack('>Q', struct.pack('>d', v))[0]
                    assert a >= 0 and b >= 0 and abs(bits(a)-bits(b)) <= 1, (scene, before['epoch'], key, a, b)
                    result['final_bound_roundoff'].append([scene, before['epoch'], key, before[key], after[key]])
    target = baseline if freeze else CACHE / 'legacy-parity.json'
    target.write_text(json.dumps(result, indent=2) + '\n')
    print('FROZEN' if freeze else ('LEGACY_PARITY_ONE_ULP_FINAL_BOUND' if result['final_bound_roundoff'] else 'EXACT_LEGACY_PARITY'), result['identity'])


def check():
    CACHE.mkdir(parents=True, exist_ok=True)
    for name in ['sc9_input_checks', 'sc9_event_checks', 'sc9_time_checks', 'sc9_joint_checks', 'sc9_recovery_checks', 'sc9_boundary_checks', 'sc9_core_checks', 'sc9_bootstrap_checks', 'sc9_time_factor_checks', 'sc9_topology_checks']:
        with (CACHE / (name + '.log')).open('w') as log:
            subprocess.run([str(BIN / name)], cwd=ROOT, env=environment(),
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        print((CACHE / (name + '.log')).read_text(), end='')


def prepare():
    # Reuse the frozen raw decoder and measured-tracker export. The old
    # contract is read-only; all newly generated material stays in SC9 cache.
    import contextlib
    import sc8_walk_followup as walk
    walk.CACHE = CACHE
    assert walk.CONTRACT.exists(), 'expected existing frozen Walk source contract'
    CACHE.mkdir(parents=True, exist_ok=True)
    with (CACHE / 'prepare.log').open('w') as log, contextlib.redirect_stdout(log):
        walk.prepare()
    import pandas as pd
    registration = {}
    for sequence in ['ISAS-Walk2', 'ISAS-Walk3']:
        data = CACHE / 'data' / sequence
        ranges = pd.read_csv(data / 'uwb.csv', float_precision='round_trip')
        imu = pd.read_csv(data / 'imu.csv', float_precision='round_trip')
        candidates = []
        # Pre-result rule uses only time, valid, and source IDs. Require three
        # consecutive observations of one source to be valid, after20s.
        for anchor, group in ranges.groupby('anchor_id', sort=True):
            records = group.sort_values(['t', 'source_message']).to_dict('records')
            for i in range(len(records) - 2):
                window = records[i:i+3]
                if window[0]['t'] >= 20 and all(r['valid'] for r in window):
                    candidates.append((window[0]['t'], int(anchor), window))
                    break
        assert candidates, sequence + ': no valid source-only injection window'
        t, anchor, window = min(candidates, key=lambda x: (x[0], x[1]))
        begin = t + .18
        active = imu[(imu.t >= begin) & (imu.t < begin + .5)]
        assert len(active) >= 2 and imu.t.max() > begin + 1.0
        registration[sequence] = dict(
            selection='earliest>=20s three consecutive valid records of a physical anchor; tie by anchor ID; no GT/residual/detection',
            sensor_hash=walk.sensor_hash(data), anchor_id=anchor,
            range_source=[{k: int(r[k]) for k in ['source_message', 'source_range']} for r in window],
            range_times_s=[r['t'] for r in window], imu_begin_s=begin, imu_end_s=begin+.5,
            imu_samples=len(active), range_add_m=2.25, accel_add_mps2=20., gyro_add_radps=1.,
            axis='original_sensor_x', cases=['U','A','G','J'] if sequence.endswith('2') else ['J'])
    target = CACHE / 'injection-registration.json'
    if target.exists():
        assert json.loads(target.read_text()) == registration, 'frozen injection selection changed'
    else:
        target.write_text(json.dumps(registration, indent=2) + '\n')
    print(json.dumps(registration, indent=2))


def calibrate():
    """CAL-only range/pose mounting fit; never reads Walk2/3 or their GT."""
    import numpy as np
    import pandas as pd
    from scipy.interpolate import CubicSpline
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation, Slerp
    from scipy.signal import savgol_filter
    path = CACHE / 'data/ISAS-Walk1'
    gt = pd.read_csv(path/'gt.csv', float_precision='round_trip').drop_duplicates('t')
    imu = pd.read_csv(path/'imu.csv', float_precision='round_trip')
    ranges = pd.read_csv(path/'uwb.csv', float_precision='round_trip')
    cal = json.loads((CACHE/'w0.json').read_text())['cal_diagnostic']
    cut = cal['training_end_s']
    position = CubicSpline(gt.t, gt[['px','py','pz']])
    attitude = Slerp(gt.t, Rotation.from_quat(gt[['qx','qy','qz','qw']]))
    rows = ranges[(ranges.valid == 1) & (ranges.t > gt.t.min()+.11) &
                  (ranges.t < gt.t.max()-.11)].copy()
    train = rows[rows.t < cut-.5]
    hold = rows[rows.t > cut+.5]
    # A training-only squared-range seed gives an initial world transform.
    # This seed never selects a raw protection branch and is not a certificate.
    nominal, observed = [], []
    for _, group in train.groupby('source_message'):
        if group.anchor_id.nunique() < 4:
            continue
        anchors = group[['anchor_x','anchor_y','anchor_z']].to_numpy()
        r = group['range'].to_numpy()
        design = 2*(anchors[1:]-anchors[0])
        if np.linalg.matrix_rank(design) < 3:
            continue
        rhs = (anchors[1:]**2).sum(1)-(anchors[0]**2).sum()-r[1:]**2+r[0]**2
        observed.append(np.linalg.lstsq(design, rhs, rcond=None)[0])
        nominal.append(position(group.t.mean()))
    x, y = np.asarray(nominal), np.asarray(observed)
    assert len(x) > 20
    u, _, vt = np.linalg.svd((x-x.mean(0)).T@(y-y.mean(0)))
    rotation = vt.T@np.diag([1,1,np.linalg.det(vt.T@u.T)])@u.T
    seed = np.r_[Rotation.from_matrix(rotation).as_rotvec(), y.mean(0)-rotation@x.mean(0),
                 np.zeros(3), 0.]

    def residual(parameters, data):
        t = data.t.to_numpy()+parameters[9]
        tag_tracker_world = position(t)+attitude(t).apply(parameters[6:9])
        tag_anchor_world = Rotation.from_rotvec(parameters[:3]).apply(tag_tracker_world)+parameters[3:6]
        return np.linalg.norm(tag_anchor_world-data[['anchor_x','anchor_y','anchor_z']].to_numpy(),axis=1)-data['range'].to_numpy()

    def fit(data, initial):
        lower = np.r_[np.full(6,-np.inf),np.full(3,-2.),-.1]
        upper = np.r_[np.full(6,np.inf),np.full(3,2.),.1]
        return least_squares(lambda p: residual(p,data), initial, bounds=(lower,upper),
                             loss='soft_l1', f_scale=.15, max_nfev=200,
                             xtol=1e-10, ftol=1e-10, gtol=1e-10)
    result = fit(train,seed)
    assert result.success
    e_train, e_hold = residual(result.x,train), residual(result.x,hold)
    singular = np.linalg.svd(result.jac, compute_uv=False)
    rotation_ti = np.asarray(cal['R_tracker_from_imu'])
    lever_i = np.array([.1,-.025,0.])
    translation_ti = result.x[6:9]-rotation_ti@lever_i
    reference = dict(R_anchor_from_tracker_world=Rotation.from_rotvec(result.x[:3]).as_matrix().tolist(),
                     t_anchor_from_tracker_world_m=result.x[3:6].tolist(),
                     tracker_to_tag_m=result.x[6:9].tolist(),
                     R_tracker_from_imu=rotation_ti.tolist(),tracker_to_imu_m=translation_ti.tolist(),
                     range_to_tracker_time_s=float(result.x[9]),imu_to_tracker_time_s=cal['time_gt_minus_imu_s'])
    per_anchor = {}
    for anchor in sorted(rows.anchor_id.unique()):
        per_anchor[str(anchor)] = {}
        for label,data,error in [('train',train,e_train),('hold',hold,e_hold)]:
            e = error[data.anchor_id.to_numpy()==anchor]
            per_anchor[str(anchor)][label] = dict(n=len(e),mean_m=float(e.mean()),
                std_m=float(e.std()),rmse_m=float(np.sqrt(np.mean(e**2))),
                lag1=float(np.corrcoef(e[:-1],e[1:])[0,1]))
    # Parameter stability uses only the first50% CAL subset, not TEST or hold.
    earlier = train[train.t < imu.t.min()+.5*(imu.t.max()-imu.t.min())]
    alternate = fit(earlier,seed)
    assert alternate.success
    stability = dict(training_fraction=[.5,.6],
        mount_delta_m=float(np.linalg.norm(result.x[6:9]-alternate.x[6:9])),
        world_rotation_delta_rad=float((Rotation.from_rotvec(result.x[:3]).inv()*Rotation.from_rotvec(alternate.x[:3])).magnitude()),
        world_translation_delta_m=float(np.linalg.norm(result.x[3:6]-alternate.x[3:6])),
        clock_delta_s=float(abs(result.x[9]-alternate.x[9])))
    # Independent acceleration relation using the RANGE-derived mounting,
    # compare multiple predeclared differentiation supports on CAL only.
    acceleration = []
    t = np.arange(max(gt.t.min(),imu.t.min())+.6,min(gt.t.max(),imu.t.max())-.6,.02)
    rot = attitude(t)
    omega = (attitude(t-.01).inv()*attitude(t+.01)).as_rotvec()/.02
    force = CubicSpline(imu.t,imu[['ax','ay','az']])(t-cal['time_gt_minus_imu_s'])@rotation_ti.T
    def skew(v):
        return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    for width in [21,31,41]:
        acc = savgol_filter(position(t),width,3,deriv=2,delta=.02,axis=0)
        alpha = savgol_filter(omega,width,3,deriv=1,delta=.02,axis=0)
        mount = np.array([skew(a)+skew(w)@skew(w) for a,w in zip(alpha,omega)])
        design = np.concatenate([-rot.as_matrix().transpose(0,2,1),np.tile(np.eye(3),(len(t),1,1))],axis=2)
        response = force-rot.inv().apply(acc)-np.einsum('nij,j->ni',mount,translation_ti)
        fit_rows = (t<cut-.5)
        hold_rows = (t>cut+.5)
        coefficients,_,rank,sv = np.linalg.lstsq(design[fit_rows].reshape(-1,6),response[fit_rows].reshape(-1),rcond=None)
        error = np.einsum('nij,j->ni',design,coefficients)-response
        acceleration.append(dict(smoothing_samples=width,rank=int(rank),singular_values=sv.tolist(),
            gravity_tracker_world_mps2=coefficients[:3].tolist(),bias_tracker_mps2=coefficients[3:].tolist(),
            gravity_anchor_world_mps2=(Rotation.from_rotvec(result.x[:3]).apply(coefficients[:3])).tolist(),
            training_rmse_mps2=float(np.sqrt(np.mean(error[fit_rows]**2))),
            heldout_rmse_mps2=float(np.sqrt(np.mean(error[hold_rows]**2)))))
    output = dict(scope='CAL_ONLY_EMPIRICAL_DIAGNOSTIC; no parameters applied yet',
        source_sha={n:sha(path/n) for n in ['imu.csv','uwb.csv','gt.csv','anchors.csv']},
        training_end_s=cut,heldout_guard_s=.5,reference=reference,
        fit_loss='soft_l1 scale0.15m from preexisting range config; mounting box+-2m, clock+-0.1s',
        training_rows=len(train),heldout_rows=len(hold),
        training_rmse_m=float(np.sqrt(np.mean(e_train**2))),heldout_rmse_m=float(np.sqrt(np.mean(e_hold**2))),
        singular_values=singular.tolist(),rank=int(np.linalg.matrix_rank(result.jac)),
        parameter_stability=stability,per_anchor=per_anchor,acceleration_crosscheck=acceleration,
        hardware_raw_risk='UNQUALIFIED',reference_award='PENDING_STABILITY_AND_MODEL_REVIEW')
    target = CACHE/'calibration-diagnostic.json'
    target.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))


def inject_registered():
    import shutil
    import numpy as np
    import pandas as pd
    registration = json.loads((CACHE/'injection-registration.json').read_text())
    for sequence, rule in registration.items():
        base = CACHE/'data'/sequence
        for name, digest in rule['sensor_hash'].items():
            assert sha(base/name) == digest
        for case in rule['cases']:
            target = CACHE/'injected'/(sequence+'-'+case)
            if (target/'injection.json').exists():
                old = json.loads((target/'injection.json').read_text())
                assert old['registration_sha'] == sha(CACHE/'injection-registration.json')
                assert all(sha(target/name) == digest for name,digest in old['injected_sha'].items())
                continue
            target.mkdir(parents=True,exist_ok=True)
            imu = pd.read_csv(base/'imu.csv',float_precision='round_trip')
            ranges = pd.read_csv(base/'uwb.csv',float_precision='round_trip')
            original_imu, original_ranges = imu.copy(), ranges.copy()
            range_rows, imu_rows = [], []
            if case in ('U','J'):
                for identity in rule['range_source']:
                    selection = (ranges.source_message == identity['source_message']) & (ranges.source_range == identity['source_range'])
                    assert selection.sum() == 1 and bool(ranges.loc[selection,'valid'].iloc[0])
                    assert int(ranges.loc[selection,'anchor_id'].iloc[0]) == rule['anchor_id']
                    range_rows.extend(np.flatnonzero(selection).tolist())
                    ranges.loc[selection,['range','range_raw']] += rule['range_add_m']
            field = None
            if case in ('A','G','J'):
                # J retains the earlier Walk convention: UWB plus gyro-x.
                field = 'ax' if case == 'A' else 'gx'
                amplitude = rule['accel_add_mps2'] if case == 'A' else rule['gyro_add_radps']
                selection = (imu.t >= rule['imu_begin_s']) & (imu.t < rule['imu_end_s'])
                assert int(selection.sum()) == rule['imu_samples']
                imu_rows = np.flatnonzero(selection).tolist()
                imu.loc[selection,field] += amplitude
            imu.to_csv(target/'imu.csv',index=False,float_format='%.17g')
            ranges.to_csv(target/'uwb.csv',index=False,float_format='%.17g')
            for name in ['anchors.csv','manifest.json']:
                shutil.copy2(base/name,target/name)
            # GT and injection labels are not input requirements of the adapter.
            read_i = pd.read_csv(target/'imu.csv',float_precision='round_trip',dtype=imu.dtypes.to_dict())
            read_r = pd.read_csv(target/'uwb.csv',float_precision='round_trip',dtype=ranges.dtypes.to_dict())
            np.testing.assert_array_equal(imu.to_numpy(),read_i.to_numpy())
            np.testing.assert_array_equal(ranges.to_numpy(),read_r.to_numpy())
            for name in original_ranges.columns.difference(['range','range_raw']):
                pd.testing.assert_series_equal(original_ranges[name],read_r[name],check_exact=True)
            for name in original_imu.columns:
                if name != field: pd.testing.assert_series_equal(original_imu[name],read_i[name],check_exact=True)
            metadata = dict(sequence=sequence,case=case,registration_sha=sha(CACHE/'injection-registration.json'),
                base_sha=rule['sensor_hash'],injected_sha={name:sha(target/name) for name in rule['sensor_hash']},
                range_rows=range_rows,imu_rows=imu_rows,imu_field=field,
                range_source=rule['range_source'] if range_rows else [],
                scope='SC9 source-only registered strong overlay; scorer labels only; no validity/time/identity changes')
            (target/'injection.json').write_text(json.dumps(metadata,indent=2)+'\n')
            print(sequence,case,'valid range rows',len(range_rows),'IMU rows',len(imu_rows),'field',field)


def empirical_model():
    """Declare CAL-only model parameters before any SC9 TEST trajectory run."""
    import numpy as np
    import pandas as pd
    from scipy.interpolate import CubicSpline
    from scipy.signal import savgol_filter
    from scipy.spatial.transform import Rotation, Slerp
    diagnostic = json.loads((CACHE/'calibration-diagnostic.json').read_text())
    gyro_cal = json.loads((CACHE/'w0.json').read_text())['cal_diagnostic']
    path = CACHE/'data/ISAS-Walk1'
    for name, expected in diagnostic['source_sha'].items():
        assert sha(path/name) == expected
    imu = pd.read_csv(path/'imu.csv', float_precision='round_trip')
    gt = pd.read_csv(path/'gt.csv', float_precision='round_trip').drop_duplicates('t')
    reference = diagnostic['reference']
    rotation_ti = np.asarray(reference['R_tracker_from_imu'])
    mounting = np.asarray(reference['tracker_to_imu_m'])
    attitude = Slerp(gt.t, Rotation.from_quat(gt[['qx','qy','qz','qw']]))
    position = CubicSpline(gt.t, gt[['px','py','pz']])
    # Shortest of the already declared differentiation supports. This is a
    # modeling choice after CAL diagnostics, not untouched held-out validation.
    force_cal = diagnostic['acceleration_crosscheck'][0]
    gravity_anchor = np.asarray(force_cal['gravity_anchor_world_mps2'])
    gravity = float(np.linalg.norm(gravity_anchor))
    direction = gravity_anchor/gravity
    target = np.array([0.,0.,-1.])
    axis = np.cross(direction,target)
    length = np.linalg.norm(axis)
    cosine = float(direction@target)
    if length > 1e-12:
        rotate_world = Rotation.from_rotvec(axis/length*np.arctan2(length,cosine))
    elif cosine > 0:
        rotate_world = Rotation.identity()
    else:
        candidate = np.eye(3)[np.argmin(np.abs(direction))]
        axis = np.cross(direction,candidate);axis /= np.linalg.norm(axis)
        rotate_world = Rotation.from_rotvec(np.pi*axis)
    assert np.linalg.norm(rotate_world.apply(gravity_anchor)-gravity*target) < 1e-10
    acceleration_bias = rotation_ti.T @ np.asarray(force_cal['bias_tracker_mps2'])
    gyro_bias = -rotation_ti.T @ np.asarray(gyro_cal['tracker_gyro_offset_radps'])
    time = np.arange(max(gt.t.min(), imu.t.min())+.6,
                     min(gt.t.max(), imu.t.max())-.6, .02)
    rot = attitude(time)
    body_position = position(time)+rot.apply(mounting)
    omega_tracker = (attitude(time-.01).inv()*attitude(time+.01)).as_rotvec()/.02
    omega_body = omega_tracker @ rotation_ti
    cut = diagnostic['training_end_s']
    train = time < cut-.5
    hold = time > cut+.5
    domains = []
    for width in [21,31,41]:
        derivatives = [savgol_filter(body_position,width,4,deriv=d,delta=.02,axis=0) for d in [1,2,3]]
        angular = savgol_filter(omega_body,width,3,deriv=1,delta=.02,axis=0)
        norms = dict(zip(['speed','acceleration','jerk','rate','alpha'],
                         [np.linalg.norm(x,axis=1) for x in derivatives+[omega_body,angular]]))
        domains.append(dict(smoothing_samples=width,
                            train_max={k:float(v[train].max()) for k,v in norms.items()},
                            hold_max={k:float(v[hold].max()) for k,v in norms.items()}))
    # Fixed 1.5 margin on CAL training maxima, with all three supports retained.
    # This is an empirical domain declaration, not a bound on unseen motion.
    domain = {k:1.5*max(d['train_max'][k] for d in domains) for k in domains[0]['train_max']}
    raw_train = imu[imu.t < cut-.5]
    high_pass = np.diff(raw_train[['ax','ay','az','gx','gy','gz']].to_numpy(),n=2,axis=0)
    centered = high_pass-np.median(high_pass,axis=0)
    scale = 1.4826*np.median(np.abs(centered),axis=0)/np.sqrt(6.)
    adc = 1.5*scale
    # Bias domains include model/time/reference discrepancies. They are not
    # ADC tails and are not interpreted as independent startup observations.
    gyro_observed = CubicSpline(imu.t, imu[['gx','gy','gz']])(time-reference['imu_to_tracker_time_s'])
    gyro_residual = gyro_observed-omega_body-gyro_bias
    force_observed = CubicSpline(imu.t, imu[['ax','ay','az']])(time-reference['imu_to_tracker_time_s'])
    body_acceleration = savgol_filter(body_position,21,4,deriv=2,delta=.02,axis=0)
    g_tracker = np.asarray(force_cal['gravity_tracker_world_mps2'])
    force_expected = rot.inv().apply(body_acceleration-g_tracker) @ rotation_ti + acceleration_bias
    force_residual = force_observed-force_expected
    bias_radius = np.r_[1.5*np.max(np.abs(force_residual[train]),axis=0),
                        1.5*np.max(np.abs(gyro_residual[train]),axis=0)]
    output = dict(model_id='sc9-walk1-cal-empirical-v1',status='DECLARED_EMPIRICAL',
        statistical_qualified=False,publication_protected=False,
        source_sha=diagnostic['source_sha'],calibration_sha=sha(CACHE/'calibration-diagnostic.json'),
        R_model_from_anchor_world=rotate_world.as_matrix().tolist(),gravity_mps2=gravity,
        reference=reference,range_to_imu_time_s=reference['range_to_tracker_time_s']-reference['imu_to_tracker_time_s'],
        accel_bias_mps2=acceleration_bias.tolist(),gyro_bias_radps=gyro_bias.tolist(),
        initial_bias_radius=bias_radius.tolist(),initial_velocity_radius=domain['speed'],
        initial_rotation_domain='FULL_SO3_OUTER_BOX; numeric tilt/yaw is not a healthy prior',
        motion=domain,range_sigma_m=.15,accel_adc_sigma=float(max(adc[:3])),gyro_adc_sigma=float(max(adc[3:])),
        accel_rw_sigma=.001,gyro_rw_sigma=.0001,
        adc_diagnostic=dict(method='1.5*MAD(second difference)/sqrt(6), CAL training only; irregular time/motion and correlated noise remain empirical',axis_scales=adc.tolist()),
        motion_diagnostic=domains,
        caveats=['No hardware marginal-tail qualification','CAL reference/time/mounting uncertainty is material',
                 'Full rotation and declared velocity/bias domains; no independent bootstrap covariance',
                 'Domain/ADC margins are declared before SC9 TEST runs; hold metrics are descriptive CAL checks'])
    (CACHE/'empirical-model.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps({k:output[k] for k in ['model_id','gravity_mps2','range_to_imu_time_s','accel_adc_sigma','gyro_adc_sigma','initial_bias_radius','motion']},indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('command', choices=['legacy-freeze', 'legacy-parity', 'check', 'prepare', 'calibrate', 'empirical-model', 'inject'])
    args = parser.parse_args()
    if args.command == 'check':
        check()
    elif args.command == 'prepare':
        prepare()
    elif args.command == 'inject':
        inject_registered()
    elif args.command == 'empirical-model':
        empirical_model()
    elif args.command == 'calibrate':
        calibrate()
    else:
        legacy(args.command == 'legacy-freeze')
