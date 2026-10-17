#!/usr/bin/env python3
"""SC9 native integration checks. Intermediate evidence is owned by .cache/sc9."""
import argparse
import csv
import hashlib
import json
import struct
import subprocess
from pathlib import Path

from sc4_system_research import ROOT, BIN, BUILD, environment

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


def check(build=False):
    CACHE.mkdir(parents=True, exist_ok=True)
    targets=['sc9_input_checks', 'sc9_event_checks', 'sc9_time_checks', 'sc9_joint_checks', 'sc9_recovery_checks', 'sc9_boundary_checks', 'sc9_core_checks', 'sc9_bootstrap_checks', 'sc9_time_factor_checks', 'sc9_topology_checks', 'sc9_route_checks', 'sc9_stream_checks', 'sc9_native_time_checks']
    if build:
        with (CACHE/'check-build.log').open('w') as log:
            subprocess.run(['cmake','-S',str(ROOT),'-B',str(BUILD)],env=environment(),stdout=log,stderr=subprocess.STDOUT,check=True)
            subprocess.run(['cmake','--build',str(BUILD),'--target']+targets+['sc9_dataset_runner','epoch_raim_runner','-j2'],env=environment(),stdout=log,stderr=subprocess.STDOUT,check=True)
    for name in targets:
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


def score_walk(args):
    """Post-sink CAL-only empirical reference: never called by the estimator."""
    import numpy as np
    import pandas as pd
    from scipy.interpolate import CubicSpline
    from scipy.spatial.transform import Rotation, Slerp
    destination = CACHE/'walk'/args.run_id
    freeze = json.loads((destination/'freeze.json').read_text())
    model_path = CACHE/'empirical-model.json'
    if sha(model_path) != freeze['model']:
        raise RuntimeError('scoring model differs from run freeze')
    model = json.loads(model_path.read_text()); ref = model['reference']
    diagnostic_path=CACHE/'calibration-diagnostic.json'
    if sha(diagnostic_path)!=model['calibration_sha']:
        raise RuntimeError('CAL diagnostic changed after model freeze')
    stability=json.loads(diagnostic_path.read_text())['parameter_stability']
    world = Rotation.from_matrix(model['R_model_from_anchor_world'])
    external = Rotation.from_matrix(ref['R_anchor_from_tracker_world'])
    mount = Rotation.from_matrix(ref['R_tracker_from_imu'])
    def statistics(values):
        values = np.asarray(values)
        if not len(values):
            return dict(n=0)
        ordered = np.sort(values)
        return dict(n=len(values),rmse=float(np.sqrt(np.mean(values**2))),
                    median=float(np.median(values)),p95=float(ordered[int(np.ceil(.95*len(values)))-1]),
                    p99=float(ordered[int(np.ceil(.99*len(values)))-1]),maximum=float(ordered[-1]))
    output = dict(scope='CAL_ONLY_EMPIRICAL_REFERENCE; no TEST alignment or fit; bias truth NA',
                  model_sha=freeze['model'],scorer_sha=sha(__file__),runs={},pairs={},injection_causality={})
    frames = {}
    for case in freeze['cases']:
        walk_name = case.split('-')[0]
        gt_path = CACHE/f'data/ISAS-{walk_name}/gt.csv'
        gt = pd.read_csv(gt_path,float_precision='round_trip').drop_duplicates('t')
        pos = CubicSpline(gt.t,gt[['px','py','pz']])
        rot = Slerp(gt.t,Rotation.from_quat(gt[['qx','qy','qz','qw']]))
        for mode in ['off','on']:
            name = case+'-'+mode
            receipt = json.loads((destination/(name+'.json')).read_text())
            csv_path = destination/(name+'.csv')
            if receipt['output_sha'] != sha(csv_path):
                raise RuntimeError('output changed after receipt')
            data = pd.read_csv(csv_path,float_precision='round_trip')
            frames[name]=data
            time = data.stamp_ns.to_numpy()/1e9
            reference_time = time+ref['imu_to_tracker_time_s']
            valid = (reference_time>=gt.t.min()) & (reference_time<=gt.t.max())
            data = data[valid]; time = time[valid]; reference_time=reference_time[valid]
            reference_rotation = world*external*rot(reference_time)*mount
            tag = world.apply(external.apply(pos(reference_time)+rot(reference_time).apply(ref['tracker_to_tag_m']))+np.asarray(ref['t_anchor_from_tracker_world_m']))
            error = data[['tag_x','tag_y','tag_z']].to_numpy()-tag
            estimate_rotation = Rotation.from_quat(data[['qx','qy','qz','qw']])
            angle = (reference_rotation.inv()*estimate_rotation).magnitude()*180/np.pi
            report = dict(reference_sha=sha(gt_path),committed=len(frames[name]),reference_matched=int(valid.sum()),
                          reference_missing=int((~valid).sum()),returncode=receipt['returncode'],
                          ape_m=statistics(np.linalg.norm(error,axis=1)),horizontal_m=statistics(np.linalg.norm(error[:,:2],axis=1)),
                          vertical_m=statistics(np.abs(error[:,2])),are_deg=statistics(angle),
                          offline_core_ms=statistics(frames[name].core_ms),
                          diagnostic_h_exceedances=int((np.linalg.norm(error[:,:2],axis=1)>data.hpl).sum()),
                          diagnostic_v_exceedances=int((np.abs(error[:,2])>data.vpl).sum()),
                          diagnostic_not_statistical_protection=True,rpe={})
            # Fixed 20--26 s local window includes the registered injections
            # and subsequent recovery, independent of estimated errors.
            local=(time>=20)&(time<26)
            report['registered_window_20_26s']=dict(ape_m=statistics(np.linalg.norm(error[local],axis=1)),are_deg=statistics(angle[local]))
            # Perturb only the scoring reference with magnitudes already
            # measured between CAL training fractions. Do not pick a best fit
            # or change estimator parameters from these TEST scores.
            clock_delta=stability['clock_delta_s']
            safe=(reference_time-clock_delta>=gt.t.min())&(reference_time+clock_delta<=gt.t.max())
            tr=reference_time[safe];er=estimate_rotation[safe]
            estimated_tag=data[['tag_x','tag_y','tag_z']].to_numpy()[safe]
            tracker_tag=pos(tr)+rot(tr).apply(ref['tracker_to_tag_m'])
            family={key:[] for key in ['world_rotation','world_translation','tracker_mount','clock']}
            def variant(key,p,r):
                family[key].append(dict(ape_rmse_m=float(np.sqrt(np.mean(np.sum((estimated_tag-p)**2,axis=1)))),
                                        are_rmse_deg=float(np.sqrt(np.mean(((r.inv()*er).magnitude()*180/np.pi)**2)))))
            base_rotation=world*external*rot(tr)*mount
            base_tag=world.apply(external.apply(tracker_tag)+np.asarray(ref['t_anchor_from_tracker_world_m']))
            for axis in np.eye(3):
                for sign in [-1,1]:
                    changed=Rotation.from_rotvec(sign*axis*stability['world_rotation_delta_rad'])*external
                    variant('world_rotation',world.apply(changed.apply(tracker_tag)+np.asarray(ref['t_anchor_from_tracker_world_m'])),world*changed*rot(tr)*mount)
                    variant('world_translation',base_tag+world.apply(sign*axis*stability['world_translation_delta_m']),base_rotation)
                    changed_tag=pos(tr)+rot(tr).apply(np.asarray(ref['tracker_to_tag_m'])+sign*axis*stability['mount_delta_m'])
                    variant('tracker_mount',world.apply(external.apply(changed_tag)+np.asarray(ref['t_anchor_from_tracker_world_m'])),base_rotation)
            for sign in [-1,1]:
                changed_time=tr+sign*clock_delta
                changed_tag=pos(changed_time)+rot(changed_time).apply(ref['tracker_to_tag_m'])
                variant('clock',world.apply(external.apply(changed_tag)+np.asarray(ref['t_anchor_from_tracker_world_m'])),world*external*rot(changed_time)*mount)
            report['reference_sensitivity']=dict(scope='one CAL-stability perturbation at a time; descriptive, not a confidence bound',
                matched=int(safe.sum()),cal_parameter_stability=stability,
                families={key:{metric:[min(v[metric] for v in values),max(v[metric] for v in values)]
                          for metric in ['ape_rmse_m','are_rmse_deg']} for key,values in family.items()})
            # Relative displacement in the body frame and relative attitude;
            # interpolation only inside observed support, no trajectory alignment.
            if len(time)>2:
                est_pos=CubicSpline(time,data[['px','py','pz']])
                est_rot=Slerp(time,estimate_rotation)
                def body_reference(t):
                    tr=t+ref['imu_to_tracker_time_s']
                    return world.apply(external.apply(pos(tr)+rot(tr).apply(ref['tracker_to_imu_m']))+np.asarray(ref['t_anchor_from_tracker_world_m']))
                for interval in [.5,1.]:
                    t=time[time+interval<=time[-1]]; end=t+interval
                    er0=est_rot(t);er1=est_rot(end)
                    rr0=world*external*rot(t+ref['imu_to_tracker_time_s'])*mount
                    rr1=world*external*rot(end+ref['imu_to_tracker_time_s'])*mount
                    delta=er0.inv().apply(est_pos(end)-est_pos(t))-rr0.inv().apply(body_reference(end)-body_reference(t))
                    rotation_error=((rr0.inv()*rr1).inv()*(er0.inv()*er1)).magnitude()*180/np.pi
                    report['rpe'][str(interval)]=dict(position_m=statistics(np.linalg.norm(delta,axis=1)),angle_deg=statistics(rotation_error))
            output['runs'][name]=report
    for case in freeze['cases']:
        off,on=(frames[case+'-'+mode] for mode in ['off','on'])
        joined=off.merge(on,on=['epoch','stamp_ns','batch_id'],suffixes=('_off','_on'))
        output['pairs'][case]=dict(matched=len(joined),off_committed=len(off),on_committed=len(on),
            tag_change_m=statistics(np.linalg.norm(joined[['tag_x_on','tag_y_on','tag_z_on']].to_numpy()-joined[['tag_x_off','tag_y_off','tag_z_off']].to_numpy(),axis=1)),
            same_imu_alarm=bool((joined.imu_alarm_off==joined.imu_alarm_on).all()),
            same_source_prefix=bool((joined.source_prefix_off==joined.source_prefix_on).all()),
            same_risk=bool((joined.event_risk_off==joined.event_risk_on).all()))
    registration=json.loads((CACHE/'injection-registration.json').read_text())
    if sha(CACHE/'injection-registration.json') != freeze['registration']:
        raise RuntimeError('injection registration changed')
    for case in freeze['cases']:
        sequence,fault=case.split('-')
        if fault=='base' or sequence+'-base' not in freeze['cases']:
            continue
        declared=registration['ISAS-'+sequence]
        first=min(declared['range_times_s'])+model['range_to_imu_time_s'] if fault in ['U','J'] else declared['imu_begin_s']
        for mode in ['off','on']:
            overlay=frames[case+'-'+mode];base=frames[sequence+'-base-'+mode]
            common=base.merge(overlay,on=['epoch','batch_id','stamp_ns'],suffixes=('_base','_overlay'))
            cols=['tag_x','tag_y','tag_z','px','py','pz','vx','vy','vz','qw','qx','qy','qz',
                  'bax','bay','baz','bgx','bgy','bgz','status','imu_alarm','uwb_alarm','excluded_source',
                  'bridge','repair','rejoined','source_prefix','event_risk','boundary','hpl','vpl']
            # Original message identity avoids classifying the first changed
            # batch as pre-injection when its rounded ns lies <0.5ns below t.
            before=(common.batch_id.to_numpy()<declared['range_source'][0]['source_message']+1) if fault in ['U','J'] else (common.stamp_ns.to_numpy()<round(first*1e9))
            mismatches=[]
            for col in cols:
                a=common.loc[before,col+'_base'].reset_index(drop=True)
                b=common.loc[before,col+'_overlay'].reset_index(drop=True)
                if not a.equals(b):mismatches.append(col)
            extra_sources=common.loc[common.excluded_source_overlay.notna() &
                         (common.excluded_source_overlay!=declared['anchor_id']) &
                         (common.excluded_source_overlay!=common.excluded_source_base),'excluded_source_overlay']
            output['injection_causality'][case+'-'+mode]=dict(matched=len(common),
                pre_injection_epochs=int(before.sum()),pre_injection_mismatched_columns=mismatches,
                new_imu_alarm_epochs=int(((common.imu_alarm_overlay>0)&(common.imu_alarm_base==0)).sum()),
                new_uwb_alarm_epochs=int(((common.uwb_alarm_overlay>0)&(common.uwb_alarm_base==0)).sum()),
                new_bridge_epochs=int(((common.bridge_overlay>0)&(common.bridge_base==0)).sum()),
                nonregistered_extra_source_epochs=int(len(extra_sources)),
                excluded_sources=overlay.excluded_source.dropna().astype('int64').value_counts().to_dict())
    (destination/'scores.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps({name:{k:result[k] for k in ['committed','ape_m','are_deg','registered_window_20_26s']} for name,result in output['runs'].items()},indent=2))


def walk(args):
    """Freeze before execution; run both action modes with the same sensors."""
    if not args.run_id or not all(c.isalnum() or c in '-_' for c in args.run_id):
        raise ValueError('walk requires a simple --run-id; existing frozen runs are resumable')
    destination = CACHE/'walk'/args.run_id
    destination.mkdir(parents=True, exist_ok=True)
    cases = [(f'Walk{w}-base', CACHE/f'data/ISAS-Walk{w}') for w in [2,3]]
    cases += [(f'Walk{w}-{f}', CACHE/f'injected/ISAS-Walk{w}-{f}')
              for w, faults in [(2,['U','A','G','J']),(3,['J'])] for f in faults]
    selected = args.cases.split(',') if args.cases else [name for name, _ in cases]
    if set(selected)-{name for name,_ in cases}:
        raise ValueError('unknown case')
    cases = [(name,path) for name,path in cases if name in selected]
    model = CACHE/'empirical-model.json'
    freeze = dict(identity=identity(), runner=sha(BIN/'sc9_dataset_runner'),
                  source_diff_sha=hashlib.sha256(subprocess.check_output(['git','diff','HEAD'],cwd=ROOT)).hexdigest(),
                  launcher_sha=sha(__file__),
                  configuration={str(p.relative_to(ROOT)):sha(p) for p in sorted((ROOT/'config').glob('*.yaml'))},
                  model=sha(model), registration=sha(CACHE/'injection-registration.json'),
                  profile=args.profile, cases={name:{file:sha(path/file)
                  for file in ['imu.csv','uwb.csv','anchors.csv']} for name,path in cases})
    manifest = destination/'freeze.json'
    if manifest.exists():
        if json.loads(manifest.read_text()) != freeze:
            raise RuntimeError('frozen run identity changed; use a new --run-id')
    else:
        manifest.write_text(json.dumps(freeze,indent=2)+'\n')
    limit = '340' if args.profile == 'gate' else '0'
    results = {}
    for name,path in cases:
        for mode in ['off','on']:
            key = name+'-'+mode
            output = destination/(key+'.csv')
            receipt = destination/(key+'.json')
            if receipt.exists():
                result = json.loads(receipt.read_text())
                if result['returncode'] == 0 and result['output_sha'] == sha(output):
                    results[key] = result
                    continue
                raise RuntimeError(f'{key}: retained failure/changed output; use a new run ID')
            command = [str(BIN/'sc9_dataset_runner'),str(model),str(path),str(output),mode,limit]
            with output.with_suffix('.log').open('w') as log:
                completed = subprocess.run(command,cwd=ROOT,env=environment(),stdout=log,stderr=subprocess.STDOUT)
            rows = list(csv.DictReader(output.open())) if output.exists() else []
            from collections import Counter
            result = dict(command=command,returncode=completed.returncode,committed=len(rows),
                          output_sha=sha(output) if output.exists() else None,
                          statuses=dict(Counter(row['status'] for row in rows)),
                          imu_alarms=sum(int(r['imu_alarm']) for r in rows),
                          uwb_alarms=sum(int(r['uwb_alarm']) for r in rows),
                          exclusions=sum(bool(r['excluded_source']) for r in rows),
                          bridges=sum(int(r['bridge']) for r in rows),
                          repairs=sum(int(r['repair']) for r in rows),
                          polluted_prior=sum(int(r['polluted_prior']) for r in rows),
                          rejoins=sum(int(r['rejoined']) for r in rows))
            if rows and 'old_engine_calls' in rows[0]:
                result['runtime_route']={key:max(int(r[key]) for r in rows) for key in
                    ['old_pipeline_constructions','old_engine_calls','old_fde_calls','backend_constructions']}
                result['semantic_audit_rows']=sum(all(len(r[key])==64 for key in
                    ['source_digest','receipt_digest','ledger_digest','boundary_digest']) for r in rows)
            receipt.write_text(json.dumps(result,indent=2)+'\n')
            results[key]=result
            print(key,json.dumps({k:v for k,v in result.items() if k not in ['command','output_sha']}),flush=True)
    (destination/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
    if any(r['returncode'] for r in results.values()):
        raise RuntimeError('matrix contains failures; all completed prefixes retained')


def wire_prepare(args):
    """Recover original integer stamps/arrival order from sensor topics only."""
    from rosbags.highlevel import AnyReader
    import numpy as np
    if not args.run_id or not all(c.isalnum() or c in '-_' for c in args.run_id):
        raise ValueError('wire-prepare requires --run-id')
    cases=args.cases.split(',') if args.cases else ['Walk2-base','Walk2-J','Walk3-base','Walk3-J']
    allowed={'Walk2-base','Walk3-base','Walk2-U','Walk2-A','Walk2-G','Walk2-J','Walk3-J'}
    if set(cases)-allowed:raise ValueError('unknown case')
    for case in cases:
        sequence,fault=case.split('-')
        base=CACHE/f'data/ISAS-{sequence}'
        source=base if fault=='base' else CACHE/f'injected/ISAS-{case}'
        manifest=json.loads((base/'manifest.json').read_text())
        bag=Path(manifest['source'])
        expected=dict(source_bag_sha=sha(bag),model_sha=sha(CACHE/'empirical-model.json'),
                      sensor_sha={name:sha(source/name) for name in ['imu.csv','uwb.csv','anchors.csv']})
        if expected['source_bag_sha']!=manifest['source_sha256']:
            raise RuntimeError('original bag identity changed')
        destination=CACHE/'wire'/args.run_id/case
        output_manifest=destination/'manifest.json'
        if output_manifest.exists():
            old=json.loads(output_manifest.read_text())
            if old['identity']!=expected or sha(destination/'events.jsonl')!=old['events_sha']:
                raise RuntimeError('wire input changed; use a new run ID')
            print(case,'reused',old['events'],flush=True);continue
        destination.mkdir(parents=True,exist_ok=True)
        def rows(path):
            with path.open() as stream:return list(csv.DictReader(stream))
        base_imu=rows(base/'imu.csv');input_imu=rows(source/'imu.csv')
        base_range=rows(base/'uwb.csv');input_range=rows(source/'uwb.csv')
        if len(base_imu)!=len(input_imu) or len(base_range)!=len(input_range):
            raise RuntimeError('overlay row count changed')
        grouped={};original={}
        for collection,target in [(input_range,grouped),(base_range,original)]:
            for row in collection:target.setdefault(int(row['source_message']),[]).append(row)
        anchors=[dict(id=int(row['anchor_id']),position=[float(row[k]) for k in ['x','y','z']],calibration_version=1)
                 for row in rows(source/'anchors.csv')]
        (destination/'anchors.json').write_text(json.dumps(anchors,indent=2)+'\n')
        i=j=count=invalid=0;delays={'imu':[],'range':[]};first_receive=None;first_imu=None;last_receive=None
        topics={'/waveshare_sense_hat_b','/rtls_flares'}
        with AnyReader([bag]) as reader,(destination/'events.jsonl').open('w') as out:
            connections=[c for c in reader.connections if c.topic in topics]
            for connection,received,raw in reader.messages(connections=connections):
                message=reader.deserialize(raw,connection.msgtype)
                stamp=int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec)
                if received<stamp:raise RuntimeError('recorded receive precedes original header')
                count+=1;first_receive=received if first_receive is None else first_receive;last_receive=received
                event=dict(event_sequence=count,received_ns=received,stamp_ns=stamp)
                if connection.topic=='/waveshare_sense_hat_b':
                    fields=['ax','ay','az','gx','gy','gz']
                    actual=[message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z,
                            message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z]
                    if actual!=[float(base_imu[i][key]) for key in fields]:raise RuntimeError('bag/CSV ADC mismatch')
                    values=[float(input_imu[i][key]) for key in fields]
                    event.update(kind='imu',measurement_id=i+1,quality_flags=0,accel=values[:3],gyro=values[3:])
                    first_imu=stamp if first_imu is None else first_imu;i+=1;delays['imu'].append(received-stamp)
                else:
                    current=grouped[j];old=original[j]
                    if len(current)!=len(message.ranges) or len(old)!=len(current):raise RuntimeError('bag/CSV range membership changed')
                    members=[]
                    for member,row,base_row in zip(message.ranges,current,old):
                        index=int(row['source_range']);aid=int(row['anchor_id']);valid=bool(int(row['valid']))
                        if aid!=int(member.id) or int(row['tag_id'])!=int(message.id) or valid!=(int(member.ra)!=0) or float(base_row['range_raw'])!=float(member.range):
                            raise RuntimeError('bag/CSV range source mismatch')
                        if j>=2**46 or index>65535:raise RuntimeError('measurement identity capacity')
                        identity=2**62+(j<<16)+index
                        members.append(dict(measurement_id=identity,factor_id=identity,anchor_id=aid,tag_id=int(row['tag_id']),
                            source_sequence=j,stamp_ns=stamp,received_ns=received,calibration_version=1,quality_flags=0,valid=valid,
                            range_m=float(row['range']),raw_range_m=float(row['range_raw']),sigma_m=float(row['sigma']),
                            anchor=[float(row[k]) for k in ['anchor_x','anchor_y','anchor_z']],
                            lever=[float(row[k]) for k in ['lever_x','lever_y','lever_z']]))
                        invalid+=not valid
                    event.update(kind='range',batch_id=j+1,ranges=members);j+=1;delays['range'].append(received-stamp)
                out.write(json.dumps(event,separators=(',',':'),allow_nan=False)+'\n')
        if i!=len(input_imu) or j!=len(grouped):raise RuntimeError('wire source denominator mismatch')
        result=dict(schema='sc9-canonical-sensor-wire-v1',identity=expected,events_sha=sha(destination/'events.jsonl'),
                    anchors_sha=sha(destination/'anchors.json'),events=count,imu=i,batches=j,invalid_ranges=invalid,
                    first_received_ns=first_receive,last_received_ns=last_receive,first_imu_stamp_ns=first_imu,
                    legacy_first_imu_time_s=float(base_imu[0]['t']),source_topics=sorted(topics),
                    arrival_minus_header_ms={k:np.quantile(np.array(v)*1e-6,[0,.5,.95,1]).tolist() for k,v in delays.items()},
                    timing='original bag receive order and integer header/receive ns; no GT read; CAL mapping occurs in common adapter')
        output_manifest.write_text(json.dumps(result,indent=2)+'\n');print(case,count,'sensor events',flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('command', choices=['legacy-freeze', 'legacy-parity', 'check', 'prepare', 'calibrate', 'empirical-model', 'inject', 'walk', 'score', 'wire-prepare'])
    parser.add_argument('--profile',choices=['gate','full'],default='gate')
    parser.add_argument('--run-id')
    parser.add_argument('--cases',help='comma-separated Walk2-base, Walk3-base, Walk2-U/A/G/J, Walk3-J names')
    parser.add_argument('--build',action='store_true',help='configure/build all SC9 checks before running check')
    args = parser.parse_args()
    if args.command == 'wire-prepare':
        wire_prepare(args)
    elif args.command == 'score':
        score_walk(args)
    elif args.command == 'walk':
        walk(args)
    elif args.command == 'check':
        check(args.build)
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
