#!/usr/bin/env python3
"""Independent small-model observability and legal weak-fault overlap oracles.

No oracle output is an online input. An ADC coordinate is chosen once per raw
sample, including when a trapezoid later uses it twice. Source q is global.
"""
import json
from fractions import Fraction
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from sc6_closure import CACHE, ROOT, law, dump


def global_q(sample):
    k = (sample+9)//10
    delta = 1e-6/(2*k*(k+1)) if k else 1e-6/2
    sources = 128 if k else 21
    bits = 1
    while 2*sources*2.0**(-bits) > delta:
        bits += 1
    q = 1
    while 5*q*q < 7*bits:
        q += 1
    return q


def temporal_radius(sample, axis, cfg, domain):
    """Independent necessary healthy adjacent difference, no accumulated bias."""
    q, before = global_q(sample), global_q(sample-1)
    is_accel = axis < 3
    adc = .02 if is_accel else .001
    rw = .001 if is_accel else .0001
    derivative = domain['jerk']+domain['angular_rate']*(9.80665+domain['acceleration']) if is_accel else domain['angular_acceleration']
    return derivative*.005 + (q+before)*adc + q*rw*np.sqrt(.005) + 4e-10


def input_imu(path):
    samples = []
    for line in Path(path).read_text().splitlines():
        if not line.startswith('imu '):
            continue
        v = line.split()
        samples.append(dict(index=(int(v[2])//5000000),
                            measured=np.array([float.fromhex(x) for x in v[4:10]])))
    return samples


def diagnose_inputs():
    root = law()
    rows = []
    for path in sorted(CACHE.glob('P*.csv.input')):
        meta_path = Path(str(path).replace('.csv.input', '.json'))
        if not meta_path.exists():
            continue
        meta = json.loads(meta_path.read_text())
        if meta['entry']['fde'] != 'on':
            continue
        manifest = meta['manifest']
        d = root['sc6']['domains']['D0' if manifest['motion'] == 'M0' else 'D1']
        imu = input_imu(path)
        for onset, axis, duration, amplitude, _ in manifest['faults']:
            i = (onset-1)*10+1
            if i >= len(imu):
                continue
            observed = imu[i]['measured'][axis]-imu[i-1]['measured'][axis]
            radius = temporal_radius(i, axis, None, d)
            rows.append(dict(run=meta['entry'], onset=onset, axis=axis, amplitude=amplitude,
                             observed_difference=float(observed), healthy_difference_radius=radius,
                             violation=bool(abs(observed)>radius),
                             classification='DETECTABLE_BY_NEW_CERTIFIED_RELATION' if abs(observed)>radius else 'NUMERICAL_INDETERMINATE',
                             original_episode=next(e for e in meta['episodes'] if e['onset']==onset)))
    return rows


def integrate_state(x, times, lever):
    # Static measured accelerometer=(0,0,g), gyro=0, both arbitrary bias states.
    p, v = x[:3].copy(), x[3:6].copy()
    r = Rotation.from_rotvec(x[6:9])
    ba, bg = x[9:12], x[12:15]
    tags, orientations = [p+r.apply(lever)], [r.as_rotvec()]
    t = 0.
    for target in times[1:]:
        while t < target-1e-10:
            dt = min(.005, target-t)
            a = r.apply(np.array([0, 0, 9.80665])-ba)-np.array([0, 0, 9.80665])
            p = p+v*dt+.5*a*dt*dt
            v = v+a*dt
            r = r*Rotation.from_rotvec(-bg*dt)
            t += dt
        tags.append(p+r.apply(lever))
        orientations.append(r.as_rotvec())
    return np.array(tags), np.array(orientations)


def rank_oracle():
    times = np.arange(21)*.05
    lever = np.array([.2, -.1, .08])
    step = 1e-6
    tag_columns, full_columns = [], []
    for axis in range(15):
        pos = np.zeros(15);pos[axis] = step
        neg = -pos
        ap, ar = integrate_state(pos, times, lever)
        bp, br = integrate_state(neg, times, lever)
        tag_columns.append(((ap-bp)/(2*step)).ravel())
        full_columns.append(np.concatenate([((ap-bp)/(2*step)).ravel(), ((ar-br)/(2*step)).ravel()]))
    jt = np.array(tag_columns).T
    jf = np.array(full_columns).T
    st, sf = np.linalg.svd(jt, compute_uv=False), np.linalg.svd(jf, compute_uv=False)
    assert np.sum(st>1e-7) < 15
    assert np.sum(sf>1e-7) == 15
    # Exact gauge witnesses, including initial source support (q0=7,
    # attitude sigma=.1rad and translation sigma=.5m; not a GT-selected branch).
    tag = np.array([.2, -.1, 1.08])
    states = []
    for yaw in [-.1, .1]:
        r = Rotation.from_rotvec([0, 0, yaw])
        p = tag-r.apply(lever)
        states.append(dict(yaw=yaw, p=p.tolist(), tag=(p+r.apply(lever)).tolist(),
                           specific_force=r.inv().apply([0, 0, 9.80665]).tolist()))
        assert np.linalg.norm(p-np.array([0,0,1])) < 7*.5
        assert abs(yaw) < 7*.1
    assert np.allclose(states[0]['tag'], states[1]['tag'], atol=1e-14)
    assert np.allclose(states[0]['specific_force'], states[1]['specific_force'], atol=1e-14)
    return dict(tag_only_rank=int(np.sum(st>1e-7)), orientation_observed_rank=int(np.sum(sf>1e-7)),
                tag_singular_values=st.tolist(), full_singular_values=sf.tolist(),
                exact_static_yaw_gauge=states,
                scope='Static local full15D toy; observed orientations are independent positive-model observations, never assumed in main raw.')


def overlap_oracle():
    examples = []
    # Same truth, same initial observation, same bias and RW, same UWB, at most
    # one legal IMU episode. Two distinct ADC histories stay in the original E.
    # Faulty y=h+b+sigma*(-theta/2sigma)+theta equals
    # healthy y=h+b+sigma*(theta/2sigma) exactly in rational arithmetic.
    for sample, axis, theta, sigma, duration in [(11201, 0, Fraction(3,10), Fraction(1,50), 50),
                                                (51201, 3, Fraction(1,100), Fraction(1,1000), 500)]:
        draw = theta/(2*sigma)
        assert abs(draw) < global_q(sample)
        assert sigma*(-draw)+theta == sigma*draw
        examples.append(dict(sample=sample, onset_epoch=(sample-1)//10+1, axis=axis,
                             duration_ms=duration, theta=str(theta), sigma=str(sigma),
                             faulty_adc=str(-draw), healthy_adc=str(draw), q=global_q(sample),
                             equal_observable_offset=str(sigma*draw),
                             outside_episode='both ADC zero; RW coordinates equal (zero is legal); bias and physical states unchanged',
                             classification='NOT_IDENTIFIABLE_UNDER_DECLARED_LAW',
                             scope='Weak episode observation-support overlap only; not the original20m/s² or1rad/s strong episode.'))
    return examples



def geometry_oracle():
    # G2's frozen 1mm top/bottom separation is not exactly coplanar.
    anchors = np.array([[4 if a&1 else -4, 4 if a&2 else -4,
                         .001 if a&4 else 0] for a in range(8)], dtype=float)
    p = np.array([.2, -.1, 1.08])
    mirror = p.copy();mirror[2] = -p[2]
    ra, rb = np.linalg.norm(anchors-p, axis=1), np.linalg.norm(anchors-mirror, axis=1)
    midpoint = (ra+rb)/2
    # Same noisy ranges: independent normal coordinates (midpoint-range)/.1
    # are legal within the ORIGINAL q0/epoch Gaussian support for both truths.
    za, zb = (midpoint-ra)/.1, (midpoint-rb)/.1
    assert max(np.abs(za).max(), np.abs(zb).max()) < 7
    h = (p-anchors)/ra[:,None]
    singular = np.linalg.svd(h, compute_uv=False)
    design = 2*(anchors[1:]-anchors[0])
    squared = np.array([a@a-anchors[0]@anchors[0]-r*r+midpoint[0]**2
                        for a,r in zip(anchors[1:],midpoint[1:])])
    seed = np.linalg.lstsq(design, squared, rcond=None)[0]
    return dict(anchors=anchors.tolist(), positive=p.tolist(), mirror=mirror.tolist(),
                shared_ranges=midpoint.tolist(), positive_noise_standard=za.tolist(),
                mirror_noise_standard=zb.tolist(), jacobian_singular_values=singular.tolist(),
                squared_range_seed=seed.tolist(),
                conclusion='G2 geometry has full algebraic rank at this off-plane point, yet two mirror-like physical tags share legal noisy observation support. A functional seed/convergence rejection does not prove physical emptiness; no GT branch choice.')


def actual_g2_oracle(folder=CACHE/'w1/endpoint-fixed'):
    from scipy.optimize import least_squares
    from scipy.stats import chi2
    from sc6_closure import sha
    path = folder/'P3-S3-20261010-30-30-on-audit.csv.input'
    if not path.exists():
        return dict(status='NOT_RUN')
    lines = [line.split() for line in path.read_text().splitlines()
             if line.startswith('uwb 1 ')]
    anchors = np.array([[float.fromhex(x) for x in row[10:13]] for row in lines])
    ranges = np.array([float.fromhex(row[13]) for row in lines])
    evidence = []
    for excluded in [-1, *range(8)]:
        indices = [i for i in range(8) if i != excluded]
        a, y = anchors[indices], ranges[indices]
        d = 2*(a[1:]-a[0])
        b = np.sum(a[1:]**2,axis=1)-a[0]@a[0]-y[1:]**2+y[0]**2
        seed = np.linalg.lstsq(d,b,rcond=None)[0]
        h = (seed-a)/np.linalg.norm(seed-a,axis=1)[:,None]
        native_first_step = np.linalg.lstsq(h, y-np.linalg.norm(seed-a,axis=1), rcond=None)[0]
        solutions = []
        for start_z in [-2., -1., 1., 2.]:
            fitted = least_squares(lambda p:(np.linalg.norm(p-a,axis=1)-y)/.1,
                                   np.array([0.,0.,start_z]), max_nfev=200,
                                   xtol=1e-12,ftol=1e-12,gtol=1e-12)
            solutions.append(dict(start=[0,0,start_z], point=fitted.x.tolist(),
                                  objective=float(2*fitted.cost), success=bool(fitted.success),
                                  threshold=float(chi2.isf(1e-6,len(indices)-3)),
                                  post_pass=bool(2*fitted.cost<=chi2.isf(1e-6,len(indices)-3))))
        evidence.append(dict(excluded=excluded, anchor_design_singular=np.linalg.svd(d,compute_uv=False).tolist(),
                             squared_range_seed=seed.tolist(), native_first_step_norm=float(np.linalg.norm(native_first_step)),
                             native_domain_guard_rejects=bool(np.linalg.norm(native_first_step)>10),
                             observation_only_fixed_seed_solutions=solutions))
    return dict(input_sha=sha(path), candidates=evidence,
                scope='Actual first S3 ranges, all9 masks, fixed observation-only initial seeds; independent diagnostic, not a committed candidate or a raw inclusion proof. Native failure remains in denominator.')

def main():
    result = dict(rank=rank_oracle(), geometry=geometry_oracle(), actual_g2=actual_g2_oracle(), weak_overlap=overlap_oracle(),
                  temporal_diagnosis=diagnose_inputs(),
                  theorem='|d specific_force/dt| <= J+Omega*(A+g); |d body_omega/dt|<=alpha. Bias initial cancels in adjacent differences; one original RW increment and each original ADC coordinate remain. This is necessary in every declared healthy interval, not a new Gaussian draw.')
    dump(CACHE/'observability-prototype.json', result)
    print('OBSERVABILITY ORACLES PASS', result['rank']['tag_only_rank'], '/15 tag-only,',
          result['rank']['orientation_observed_rank'], '/15 positive; weak overlaps', len(result['weak_overlap']))
    print('Actual input onset diagnostics', len(result['temporal_diagnosis']))


if __name__ == '__main__':
    main()
