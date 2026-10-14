#!/usr/bin/env python3
"""SC6 Closure preregistered paired native runs and immutable output scoring.

GT/scoring/AL never enters the estimator. Input audits are pre-RUN_READY evidence;
privileged source certification remains inside the timed service.
"""
import argparse
import csv
import hashlib
import json
import math
import re
import subprocess
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from sc4_system_research import BIN, BUILD, ROOT, environment
from sc6_system_research import CORE_CHECKS
CLOSURE_CHECKS = CORE_CHECKS + [('sc6_closure_checks', [])]

CACHE = ROOT / '.cache/sc6-closure'
CONFIG = ROOT / 'config/sc6_closure.yaml'
SEEDS = [20261010, 20261011, 20261012, 20261013, 20261014]
ALS = [(2, 3), (5, 5), (10, 10)]
PHASES = ['healthy_before', 'fault_active', 'detected_not_isolated',
          'quarantine_bridge', 'post_rejoin', 'late_unrecovered']
METRICS = ['ape', 'hpe', 'vpe', 'velocity_error', 'rotation_error',
           'accel_bias_error', 'gyro_bias_error',
           'baerr_x', 'baerr_y', 'baerr_z', 'bgerr_x', 'bgerr_y', 'bgerr_z']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def law(path=CONFIG):
    v = yaml.safe_load(Path(path).read_text())
    if 'raw_base_law' in v:
        base = law(ROOT / v['raw_base_law'])
        base.update(v)
        return base
    return v


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def identity():
    sources = subprocess.check_output(['git', 'ls-files', 'research', 'src', 'include',
                                      'config', 'tools', 'CMakeLists.txt'], cwd=ROOT, text=True).splitlines()
    sources += ['config/sc6_closure.yaml', 'tools/sc6_closure.py', 'tools/test_sc6_closure.py',
                'tools/sc6_closure_observability.py', 'research/sc6_temporal_monitor.hpp', 'research/sc6_closure_checks.cpp']
    return {'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
            'elf': sha(BIN / 'epoch_raim_runner'), 'dso': sha(BIN.parent / 'libuwb_imu_pl.so'),
            'sources': {p: sha(ROOT / p) for p in sorted(set(sources)) if (ROOT / p).is_file()},
            'platform': subprocess.check_output(['uname', '-a'], text=True).strip(),
            'cpu': subprocess.check_output(['lscpu'], text=True),
            'environment': {k: environment()[k] for k in ['OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS']}}


def matrix():
    entries = []
    def add(group, case, seed=SEEDS[0], window='30/30', modes=('on',), positive=True, hook='audit'):
        for fde in modes:
            entries.append(dict(group=group, case=case, seed=seed, window=window,
                                fde=fde, mode='paced', profile='tight-optimized', positive=positive, hook=hook))
    for case in ['S0', 'S1', 'S2']:
        for seed in SEEDS:
            add('P0', case, seed, modes=('on', 'off'))
    for axis in range(6):
        add('P1', f'F{axis}')
    add('P1', 'F0', SEEDS[1])
    add('P1', 'V')
    for case in ['D1_A_E', 'D1_G_E', 'D1_A_L', 'D1_G_L', 'W0', 'W1', 'W2']:
        add('P2', case, modes=('on', 'off'))
    for window in ['20/20', '30/10', '40/40']:
        add('P3', 'V', window=window)
    add('P3', 'S3', positive=False)
    for case in ['L0', 'L1']:
        add('P4', case, modes=('on', 'off'))
    for hook in ['cancel:3', 'delay:3']:
        add('N', 'S0', positive=False, hook=hook)
    return entries


def label(entry):
    return '-'.join(str(entry[k]).replace('/', '-').replace(':', '-')
                    for k in ['group', 'case', 'seed', 'window', 'fde', 'hook'])


def quantile(values, probability):
    v = sorted(float(x) for x in values if math.isfinite(float(x)) and float(x) >= 0)
    return v[max(0, math.ceil(probability * len(v)) - 1)] if v else None


def stats(values):
    v = [abs(float(x)) for x in values if math.isfinite(float(x))]
    if not v:
        return dict(n=0, rmse=None, p50=None, p95=None, p99=None, max=None)
    return dict(n=len(v), rmse=float(np.sqrt(np.mean(np.square(v)))),
                p50=quantile(v, .5), p95=quantile(v, .95), p99=quantile(v, .99), max=max(v))


def partition(e, p, a):
    """Mutually exclusive numerical regions; unavailable underbound is retained."""
    if not all(math.isfinite(x) for x in (e, p, a)) or min(e, p) < 0 or a <= 0:
        raise ValueError('Stanford requires finite PE/PL>=0 and AL>0')
    if p > a:
        return 'AL_unavailable'
    if e <= p:
        return 'nominal_admissible'
    return 'MI_candidate' if e <= a else 'HMI_candidate'


def value(row, key, default=float('nan')):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return default


def committed(row):
    return value(row, 'updates') == 1


def spatial(row):
    k = value(row, 'epoch')
    return (committed(row) and value(row, 'qualified') == 1 and
            value(row, 'stamp_ns') == k * 50000000 and value(row, 'version') == k + 1 and
            value(row, 'generation') == k and value(row, 'parent') == k - 1 and
            value(row, 'source_prefix') == 21 + 128 * k and
            all(math.isfinite(value(row, key)) and value(row, key) >= 0
                for key in ['hpe', 'vpe', 'hpl', 'vpl']))


def current(row):
    return spatial(row) and value(row, 'timely') == 1 and 0 <= value(row, 'delivery_ms') <= 50


def stanford(rows, scheduled, axis, al):
    pe, pl = ('hpe', 'hpl') if axis == 'H' else ('vpe', 'vpl')
    counts = dict(scheduled=scheduled, received=len(rows),
                  attempted=sum(r.get('status') != 'SOURCE_GAP_OR_ABORT' for r in rows),
                  committed=sum(committed(r) for r in rows),
                  native_committed=sum(value(r, 'mutations', 0) > 0 for r in rows),
                  spatial_qualified=sum(spatial(r) for r in rows),
                  time_current=sum(current(r) for r in rows),
                  refused=scheduled-sum(committed(r) for r in rows),
                  expired=sum(spatial(r) and not current(r) for r in rows),
                  delayed=sum(value(r, 'delivery_ms') > 50 for r in rows),
                  plot_points=sum(current(r) for r in rows),
                  excluded_from_plot=scheduled-sum(current(r) for r in rows))
    for name in ['covered', 'underbound', 'AL_admissible', 'timely_AL_admissible',
                 'nominal_admissible', 'MI_candidate', 'HMI_candidate', 'AL_unavailable',
                 'spatial_underbound', 'spatial_AL_admissible']:
        counts[name] = 0
    for r in rows:
        if spatial(r):
            counts['spatial_underbound'] += value(r, pe) > value(r, pl)
            counts['spatial_AL_admissible'] += value(r, pl) <= al
        if not current(r):
            continue
        e, p = value(r, pe), value(r, pl)
        counts['covered' if e <= p else 'underbound'] += 1
        counts[partition(e, p, al)] += 1
        counts['AL_admissible'] += p <= al
        counts['timely_AL_admissible'] += p <= al
    assert counts['covered'] + counts['underbound'] == counts['plot_points']
    assert sum(counts[k] for k in ['nominal_admissible', 'MI_candidate', 'HMI_candidate', 'AL_unavailable']) == counts['plot_points']
    assert counts['plot_points'] + counts['excluded_from_plot'] == scheduled
    counts['numerical_availability'] = counts['spatial_AL_admissible'] / scheduled if scheduled else None
    counts['timely_numerical_availability'] = counts['timely_AL_admissible'] / scheduled if scheduled else None
    counts['operational_HMI'] = 'NOT_QUALIFIED'
    return counts


def vec(r, prefix):
    return np.array([value(r, prefix + j) for j in 'xyz'])


def rot(r, prefix='q'):
    return Rotation.from_quat([value(r, prefix+j) for j in 'xyzw'])


def verified_rows(rows, lever):
    """Recompute error from terminal states, never from aggregate summaries."""
    for r in rows:
        if not committed(r):
            continue
        assert all(np.isclose(np.linalg.norm([value(r, prefix+j) for j in 'xyzw']), 1,
                              rtol=0, atol=1e-10) for prefix in ['q', 'gt_q']), 'non-unit quaternion'
        tag = vec(r, 'tag_')
        gt_tag = vec(r, 'gt_p') + rot(r, 'gt_q').apply(lever)
        assert np.allclose(tag, vec(r, 'p') + rot(r).apply(lever), rtol=0, atol=1e-10)
        delta = tag-gt_tag
        errors = dict(ape=np.linalg.norm(delta), hpe=np.linalg.norm(delta[:2]), vpe=abs(delta[2]),
                      rotation_error=(rot(r, 'gt_q').inv()*rot(r)).magnitude(),
                      velocity_error=np.linalg.norm(vec(r, 'v')-vec(r, 'gt_v')),
                      accel_bias_error=np.linalg.norm(vec(r, 'baerr_')),
                      gyro_bias_error=np.linalg.norm(vec(r, 'bgerr_')))
        assert all(np.isclose(value(r, k), v, rtol=1e-10, atol=1e-10) for k, v in errors.items()), r['epoch']
        assert all(value(r, k) == 0 for k in ['formal_eligible', 'publication_protected', 'application_available'])
        assert 0 <= errors['rotation_error'] <= math.pi + 1e-12
    return rows


def episodes(rows, manifest):
    good = [r for r in rows if committed(r)]
    result = []
    for i, f in enumerate(manifest['faults']):
        onset, axis, duration, amplitude, uwb_onset = f
        begin_ns = (onset-1)*50000000+5000000
        end_ns = begin_ns+duration*1000000
        next_onset = manifest['faults'][i+1][0] if i+1 < len(manifest['faults']) else math.inf
        relevant = [r for r in good if onset <= int(value(r, 'epoch')) < next_onset]
        alarm = [r for r in relevant if value(r, 'imu_alarm') == 1 and value(r, 'first_imu_alarm_ns') >= begin_ns]
        bridge = [r for r in relevant if value(r, 'bridge') == 1]
        repairs = [r for r in bridge if value(r, 'repair') == 1 and value(r, 'removed_priors') > 0]
        unsafe_rejoin = [r for r in relevant if value(r, 'rejoined') == 1 and
                         (int(value(r, 'epoch'))-1)*50000000 < end_ns and bridge]
        rejoin = [r for r in relevant if value(r, 'rejoined') == 1 and
                  (int(value(r, 'epoch'))-1)*50000000 >= end_ns and bridge and
                  int(value(r, 'epoch')) > int(bridge[0]['epoch'])]
        first = lambda items: int(value(items[0], 'epoch')) if items else None
        fd, bi, re = first(alarm), first(bridge), first(rejoin)
        result.append(dict(onset=onset, axis=axis, duration_ms=duration, amplitude=amplitude,
                           begin_ns=begin_ns, end_ns=end_ns, first_fd=fd, first_bridge=bi,
                           first_prior_repair=first(repairs), first_rejoin=re,
                           rejoin4=[int(value(r, 'epoch')) for r in rejoin[:4]],
                           rejoin_on_fault_shared_endpoint=[int(value(r, 'epoch')) for r in unsafe_rejoin],
                           TTD_ms=(value(alarm[0], 'first_imu_alarm_ns')-begin_ns)/1e6 if alarm else None,
                           TTI_ms=(bi*50-begin_ns/1e6) if bi else None,
                           TTR_ms=(re*50-begin_ns/1e6) if re else None,
                           TTD_wall_ms=value(alarm[0], 'alarm_ms')-begin_ns/1e6 if alarm and math.isfinite(value(alarm[0], 'alarm_ms')) else None,
                           TTI_wall_ms=value(bridge[0], 'commit_time_ms')-begin_ns/1e6 if bridge else None,
                           TTR_wall_ms=value(rejoin[0], 'commit_time_ms')-begin_ns/1e6 if rejoin else None,
                           uwb_onset=uwb_onset,
                           first_uwb_fd=first([r for r in good if uwb_onset <= int(value(r, 'epoch')) < uwb_onset+3 and value(r, 'uwb_alarm') == 1]),
                           correct_uwb_exclusion=[int(value(r, 'epoch')) for r in good if uwb_onset <= int(value(r, 'epoch')) < uwb_onset+3 and value(r, 'excluded') == 0],
                           recovered=bool(repairs and len(rejoin) >= 4 and not unsafe_rejoin)))
    return result


def phase(r, eps):
    k = int(value(r, 'epoch'))
    if value(r, 'bridge') == 1:
        return 'quarantine_bridge'
    relevant = [e for e in eps if e['begin_ns'] <= k*50000000]
    if not relevant:
        return 'healthy_before'
    e = relevant[-1]
    active = k*50000000 < e['end_ns']
    if active and e['first_fd'] and k >= e['first_fd']:
        return 'detected_not_isolated'
    if active:
        return 'fault_active'
    if not e['first_rejoin'] or k < e['first_rejoin']:
        return 'late_unrecovered'
    return 'post_rejoin'


def metric_stats(rows):
    good = [r for r in rows if committed(r)]
    result = {key: stats([value(r, key) for r in good]) for key in METRICS}
    by_epoch = {int(value(r, 'epoch')): r for r in good}
    for d in [1, 20]:
        translation, angles = [], []
        for k, a in by_epoch.items():
            if k+d not in by_epoch:
                continue
            b = by_epoch[k+d]
            tr = rot(a, 'gt_q').inv()*rot(b, 'gt_q')
            er = rot(a).inv()*rot(b)
            tp = rot(a, 'gt_q').inv().apply(vec(b, 'gt_p')-vec(a, 'gt_p'))
            ep = rot(a).inv().apply(vec(b, 'p')-vec(a, 'p'))
            translation.append(np.linalg.norm(tr.inv().apply(ep-tp)))
            angles.append((tr.inv()*er).magnitude())
        result[f'rpe{d}_translation'] = stats(translation)
        result[f'rpe{d}_rotation'] = stats(angles)
    return result


def score(rows, manifest, scheduled):
    eps = episodes(rows, manifest)
    timing_rows = [r for r in rows if r.get('status') != 'SOURCE_GAP_OR_ABORT']
    result = dict(scheduled=scheduled, rows=len(rows), attempted=len(timing_rows),
                  backend_mutations=sum(int(value(r, 'mutations', 0)) for r in rows),
                  committed=sum(committed(r) for r in rows),
                  qualified=sum(spatial(r) for r in rows), timely=sum(current(r) for r in rows),
                  missing=scheduled-len(rows), refused=scheduled-sum(committed(r) for r in rows),
                  unattempted=sum(r.get('status') == 'SOURCE_GAP_OR_ABORT' for r in rows),
                  unknown_missing_service=scheduled-len(rows),
                  abort=sum(r.get('status') == 'ABORT' for r in rows),
                  metrics=metric_stats(rows), episodes=eps)
    result['timing'] = {key: stats([value(r, key) for r in timing_rows if value(r, key) >= 0])
                        for key in ['core_ms', 'delivery_ms']}
    result['core40'] = sum(value(r, 'core_ms') > 40 for r in timing_rows)
    result['sink50'] = sum(value(r, 'delivery_ms') > 50 for r in timing_rows)
    result['positive_realtime_pass'] = (result['timing']['core_ms']['p95'] is not None and
        result['timing']['core_ms']['p95'] <= 40 and result['timing']['delivery_ms']['p95'] <= 50 and
        result['timely']/scheduled >= .95)
    good = [r for r in rows if committed(r)]
    result['stages'] = {name: metric_stats([r for r in good if phase(r, eps) == name]) for name in PHASES}
    # Parallel channel labels; never add these to the unique overall partition.
    uwb_active = lambda r: any(e['uwb_onset'] <= int(value(r, 'epoch')) < e['uwb_onset']+3 for e in eps)
    result['uwb_stages'] = {name: metric_stats([r for r in good if uwb_active(r) == active])
                            for name, active in [('fault_active', True), ('healthy', False)]}
    assert sum(result['stages'][name]['ape']['n'] for name in PHASES) == len(good)
    result['false_uwb_exclusions'] = sum(value(r, 'excluded') >= 0 and
        not any(e['uwb_onset'] <= int(value(r, 'epoch')) < e['uwb_onset']+3 for e in eps) for r in good)
    result['stanford'] = {f'{h}/{v}': {a: stanford(rows, scheduled, a, limit)
                          for a, limit in [('H', h), ('V', v)]} for h, v in ALS}
    by_epoch = {int(value(r, 'epoch')): r for r in rows}
    result['stanford_by_stage'] = {}
    for stage_name in PHASES:
        # Allocate planned phase denominators using real action receipts when
        # present. A missing row contributes only its scheduled epoch, never PE/PL.
        stage_epochs = {k for k in range(1, scheduled+1)
                        if phase(by_epoch.get(k, {'epoch': str(k)}), eps) == stage_name}
        stage_rows = [r for r in rows if int(value(r, 'epoch')) in stage_epochs]
        result['stanford_by_stage'][stage_name] = {f'{h}/{v}': {
            a: stanford(stage_rows, len(stage_epochs), a, limit)
            for a, limit in [('H', h), ('V', v)]} for h, v in ALS}
    result['resource'] = {key: dict(stats([value(r, key) for r in good]),
                        first=value(good[0], key) if good else None, last=value(good[-1], key) if good else None)
                          for key in ['slots', 'live', 'ledger', 'rss', 'alerts', 'bias_width', 'q', 'risk', 'replay', 'capsules', 'hpl', 'vpl']}
    return result


def register():
    CACHE.mkdir(parents=True, exist_ok=True)
    p = CACHE/'registration.json'
    if p.exists():
        raise RuntimeError('registration already exists; never overwrite preregistration')
    dump(p, dict(version='SC6-CLOSURE-R1', initial_head=identity()['head'], matrix=matrix(),
                 protocol=law()['closure'], config_sha=sha(CONFIG), identity=identity(),
                 clock='steady_clock; IMU 5ms and UWB 50ms; source+native+raw+cleanup in core; sink includes queue',
                 phase_definition='Latest commenced IMU episode; bridge > detected active > active > unresolved after end > post rejoin > before. UWB episodes scored separately.',
                 negative_regressions=CLOSURE_CHECKS,
                 performance_comparisons=dict(cases=['S0', 'S2', 'D1_G_E'], count=90,
                     seed=SEEDS[0], fde='on', mode='offline',
                     profiles=['tight-reference', 'tight-optimized'],
                     purpose='same-model deterministic semantics and measured computation only; no paced deadline award'),
                 rotation_experiment=dict(case='S2', seed=SEEDS[0], count=400, mode='paced',
                     fde=['on', 'off'], metrics=['frozen_boundary', 'polar_prototype'],
                     purpose='controlled numerical-prior A diagnosis; original noise/raw/fault union; main policy remains frozen_boundary')))
    print('REGISTERED', sha(p), len(matrix()), 'native runs', flush=True)


def configured(entry):
    lf, li = map(int, entry['window'].split('/'))
    v = yaml.safe_load(CONFIG.read_text())
    v['fixed_lag_epochs'] = lf
    v['rolling_integrity'] = dict(law()['rolling_integrity'],
                                window_mode='follow_fgo' if lf == li else 'explicit',
                                explicit_epochs=None if lf == li else li)
    if entry.get('rotation_metric'):
        v['closure']['rotation_metric'] = entry['rotation_metric']
    p = CACHE/(label(entry)+'.yaml')
    p.write_text(yaml.safe_dump(v, sort_keys=True))
    return p


def run(entry):
    name = label(entry)
    summary = CACHE/(name+'.json')
    if summary.exists():
        previous = json.loads(summary.read_text())
        if previous['identity']['elf'] != identity()['elf']:
            raise RuntimeError('candidate differs from existing evidence: '+name)
        assert previous['trace_sha'] == sha(CACHE/(name+'.csv'))
        print('REUSE', name, flush=True)
        return previous
    config = configured(entry)
    trace = CACHE/(name+'.csv')
    log = CACHE/(name+'.log')
    command = [str(BIN/'epoch_raim_runner'), str(config), str(trace), entry['case'],
               'sc6', entry['fde'], str(entry.get('count', 16 if entry['group'] == 'N' else 0)),
               entry['profile'], entry['mode'], str(entry['seed']), entry['hook']]
    ident = identity()
    registration = json.loads((CACHE/'registration.json').read_text())
    assert all(ident[k] == registration['identity'][k] for k in ['elf', 'dso']), 'registered online candidate changed'
    assert sha(CONFIG) == registration['config_sha'], 'registered recipe changed'
    print('START', name, flush=True)
    with log.open('w') as sink:
        rc = subprocess.run(command, cwd=ROOT, env=environment(), stdout=sink, stderr=subprocess.STDOUT).returncode
    dump(CACHE/(name+'.execution.json'), dict(identity=ident, command=command, native_returncode=rc,
         registration_sha=sha(CACHE/'registration.json'), trace_sha=sha(trace) if trace.exists() else None,
         log_sha=sha(log), config_sha=sha(config)))
    rows = list(csv.DictReader(trace.open())) if trace.exists() else []
    resolved = law(config)
    manifest = resolved['sc6']['cases'].get(entry['case']) or resolved['closure']['additional_cases'][entry['case']]
    scheduled = entry.get('count', 16 if entry['group'] == 'N' else manifest['epochs'])
    verified_rows(rows, resolved['lever'])
    data = dict(entry=entry, identity=ident, command=command, returncode=rc,
                registration_sha=sha(CACHE/'registration.json'), config_sha=sha(config),
                trace_sha=sha(trace), log_sha=sha(log),
                input_sha=sha(str(trace)+'.input') if entry['hook'] == 'audit' else None,
                manifest=manifest, **score(rows, manifest, scheduled))
    data['input_counters'] = {key: int(v) for key, v in re.findall(r'(released_imu|enqueued_imu|released_uwb|enqueued_uwb|queue_peak)=(\d+)', log.read_text())}
    data['cold_start'] = {key: float(v) for key, v in re.findall(r'(initialized_ms|input_audit_ms|ready_ms)=([0-9.]+)', log.read_text())}
    data['profile_costs'] = {key: stats([value(r, key) for r in rows if committed(r)])
        for key in ['source_ms', 'prepare_ms', 'candidate_ms', 'proof_ms', 'commit_ms', 'cleanup_ms',
                    'envelope_cleanup_ms', 'post_sink_score_io_ms']}
    dump(summary, data)
    print('END', name, rc, f"{data['committed']}/{scheduled} committed; {data['timely']} timely",
          'P95', data['timing']['core_ms']['p95'], data['timing']['delivery_ms']['p95'], flush=True)
    return data


def paired(group):
    if not (CACHE/'registration.json').exists():
        raise RuntimeError('register before any experimental outcomes')
    for entry in json.loads((CACHE/'registration.json').read_text())['matrix']:
        if entry['group'] == group:
            run(entry)
    paired_scores()


def paired_scores():
    data = [json.loads(p.read_text()) for p in sorted(CACHE.glob('P*.json')) if p.name != 'pairs.json']
    data = [r for r in data if isinstance(r, dict) and 'entry' in r]
    pairs = []
    for on in data:
        if on['entry']['fde'] != 'on':
            continue
        off_entry = dict(on['entry'], fde='off')
        off_path = CACHE/(label(off_entry)+'.json')
        if not off_path.exists():
            continue
        off = json.loads(off_path.read_text())
        assert on['input_sha'] == off['input_sha'] and on['input_sha'] is not None
        # Cryptographic hash plus exact original-byte equality, not rounded CSV data.
        assert (CACHE/(label(on['entry'])+'.csv.input')).read_bytes() == (CACHE/(label(off_entry)+'.csv.input')).read_bytes()
        assert on['entry']['mode'] == off['entry']['mode'] == 'paced'
        assert on['identity']['elf'] == off['identity']['elf']
        on_rows = list(csv.DictReader((CACHE/(label(on['entry'])+'.csv')).open()))
        off_rows = list(csv.DictReader((CACHE/(label(off_entry)+'.csv')).open()))
        indices = {int(value(r, 'epoch')) for r in on_rows if committed(r)} & {int(value(r, 'epoch')) for r in off_rows if committed(r)}
        selected = [[r for r in rows if int(value(r, 'epoch')) in indices] for rows in [on_rows, off_rows]]
        metrics_on, metrics_off = map(metric_stats, selected)
        delta = {}
        for key in metrics_on:
            a, b = metrics_on[key]['rmse'], metrics_off[key]['rmse']
            delta[key] = dict(on=a, off=b, improvement=b-a if a is not None and b is not None else None,
                              percent=100*(b-a)/b if a is not None and b else None,
                              n=metrics_on[key]['n'])
        pairs.append(dict(entry=on['entry'], input_sha=on['input_sha'],
                          scheduled=on['scheduled'], paired=len(indices), paired_missing=on['scheduled']-len(indices),
                          metrics=delta))
    dump(CACHE/'pairs.json', pairs)
    return pairs


def compare():
    registration = json.loads((CACHE/'registration.json').read_text())
    protocol = registration['performance_comparisons']
    evidence = []
    for case in protocol['cases']:
        results = []
        for profile in protocol['profiles']:
            entry = dict(group='Pperf-'+profile, case=case, seed=protocol['seed'], window='30/30',
                         fde='on', mode='offline', profile=profile, positive=False, hook='audit',
                         count=protocol['count'])
            results.append(run(entry))
        a, b = results
        assert a['input_sha'] == b['input_sha']
        assert (CACHE/(label(a['entry'])+'.csv.input')).read_bytes() == (CACHE/(label(b['entry'])+'.csv.input')).read_bytes()
        ar = list(csv.DictReader((CACHE/(label(a['entry'])+'.csv')).open()))
        br = list(csv.DictReader((CACHE/(label(b['entry'])+'.csv')).open()))
        keys = [k for k in ar[0] if not k.endswith('_ms') and k not in ['timely', 'rss']]
        assert len(ar) == len(br) == protocol['count']
        for x, y in zip(ar, br):
            for key in keys:
                assert x[key] == y[key], (case, x['epoch'], key, x[key], y[key])
        evidence.append(dict(case=case, count=len(ar), semantic_fields=keys, input_sha=a['input_sha'],
                             reference_core=a['timing']['core_ms'], optimized_core=b['timing']['core_ms']))
        print('P exact semantic PASS', case, len(ar), flush=True)
    dump(CACHE/'performance-comparison.json', evidence)


def rotation_comparison():
    protocol = json.loads((CACHE/'registration.json').read_text())['rotation_experiment']
    evidence = []
    for metric in protocol['metrics']:
        for fde in protocol['fde']:
            entry = dict(group='A-'+metric, case=protocol['case'], seed=protocol['seed'],
                         count=protocol['count'], fde=fde, window='30/30', mode=protocol['mode'],
                         profile='tight-optimized', hook='audit', positive=False, rotation_metric=metric)
            r = run(entry)
            original = (CACHE/(label(entry)+'.csv.input')).read_bytes().splitlines(keepends=True)
            observable = b''.join(x for x in original if x.startswith((b'initial ', b'imu ', b'uwb ')))
            r['observation_history_sha'] = hashlib.sha256(observable).hexdigest()
            evidence.append(r)
    assert len({r['observation_history_sha'] for r in evidence}) == 1
    # In OFF no restoration metric is used: identical actual state and raw proof.
    off = [r for r in evidence if r['entry']['fde'] == 'off']
    a, b = [list(csv.DictReader((CACHE/(label(r['entry'])+'.csv')).open())) for r in off]
    keys = [k for k in a[0] if not k.endswith('_ms') and k not in ['timely', 'rss']]
    assert len(a) == len(b) == protocol['count']
    for x,y in zip(a,b):
        for key in keys:
            assert x[key] == y[key], (x['epoch'],key,x[key],y[key])
    dump(CACHE/'rotation-comparison.json', evidence)


def checks(build=False):
    CACHE.mkdir(parents=True, exist_ok=True)
    if build:
        with (CACHE/'build.log').open('w') as sink:
            subprocess.run(['cmake', '--build', str(BUILD), '--target', 'epoch_raim_runner',
                            *sorted({t for t, _ in CLOSURE_CHECKS}), '-j2'], cwd=ROOT,
                           env=environment(), stdout=sink, stderr=subprocess.STDOUT, check=True)
    subprocess.run(['python3', '-m', 'unittest', 'discover', '-s', 'tools', '-p', 'test_sc6_closure.py'], cwd=ROOT, check=True)
    evidence = []
    for target, args in CLOSURE_CHECKS:
        with (CACHE/('check-'+target+('-boundary' if args == ['boundary'] else '')+'.log')).open('w') as sink:
            rc = subprocess.run([str(BIN/target), *args], cwd=ROOT, env=environment(), stdout=sink, stderr=subprocess.STDOUT).returncode
        evidence.append(dict(target=target, args=args, returncode=rc))
        print('CHECK', target, args, rc, flush=True)
    dump(CACHE/'checks.json', evidence)
    assert all(x['returncode'] == 0 for x in evidence)


def seed_summary(pairs):
    rng = np.random.default_rng(61020)
    result = {}
    for case in ['S0', 'S1', 'S2']:
        selected = [p for p in pairs if p['entry']['group'] == 'P0' and p['entry']['case'] == case]
        result[case] = {}
        for metric in METRICS:
            values = [p['metrics'][metric]['improvement'] for p in selected
                      if p['metrics'][metric]['improvement'] is not None]
            percents = [p['metrics'][metric]['percent'] for p in selected
                        if p['metrics'][metric]['percent'] is not None]
            result[case][metric] = dict(N=len(values), mean=float(np.mean(values)) if values else None,
                median=float(np.median(values)) if values else None,
                minimum=min(values) if values else None, maximum=max(values) if values else None,
                percent_mean=float(np.mean(percents)) if percents else None,
                descriptive_seed_bootstrap95=list(map(float, np.quantile(
                    rng.choice(values, (10000, len(values)), replace=True).mean(axis=1), [.025, .975]))) if values else None)
    return result


def plots():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                         'savefig.dpi': 240, 'font.family': 'DejaVu Sans'})
    runs = [json.loads(p.read_text()) for p in sorted(CACHE.glob('P*.json')) if p.name != 'pairs.json']
    runs = [r for r in runs if 'entry' in r]
    p0 = [r for r in runs if r['entry']['group'] == 'P0']
    complete = len(p0) == 30
    pairs = paired_scores()
    phase_colors = dict(zip(PHASES, ['#377eb8', '#e41a1c', '#ff7f00', '#984ea3', '#4daf4a', '#a65628']))
    markers = ['o', 's', '^', 'D', 'v']
    prefix = 'Five-seed S0/S1/S2' if complete else 'INCOMPLETE PAIRS — provisional'
    for axis, suffix in [('H', 'horizontal'), ('V', 'vertical')]:
        pe, pl = ('hpe', 'hpl') if axis == 'H' else ('vpe', 'vpl')
        selected_rows = {}
        maximum = 5.0
        for run_data in p0:
            name = label(run_data['entry'])
            trace = CACHE/(name+'.csv')
            assert sha(trace) == run_data['trace_sha'], 'mutated trace'
            rows = list(csv.DictReader(trace.open()))
            selected_rows[name] = rows
            for r in rows:
                if spatial(r):
                    maximum = max(maximum, value(r, pe), value(r, pl))
        limit = maximum*1.06
        fig, axes = plt.subplots(1, 2, figsize=(12.5, 6.2), sharex=True, sharey=True)
        for ax, fde in zip(axes, ['off', 'on']):
            schedule = points = excluded = late = 0
            for run_data in p0:
                if run_data['entry']['fde'] != fde:
                    continue
                rows = selected_rows[label(run_data['entry'])]
                schedule += run_data['scheduled']
                counts = stanford(rows, run_data['scheduled'], axis, 5)
                points += counts['plot_points']
                excluded += counts['excluded_from_plot']
                late += counts['expired']
                for stage in PHASES:
                    subset = [r for r in rows if current(r) and phase(r, run_data['episodes']) == stage]
                    ax.scatter([value(r, pe) for r in subset], [value(r, pl) for r in subset],
                               s=10, alpha=.45, linewidths=0,
                               marker=markers[SEEDS.index(run_data['entry']['seed'])], c=phase_colors[stage])
                delayed = [r for r in rows if spatial(r) and not current(r)]
                ax.scatter([value(r, pe) for r in delayed], [value(r, pl) for r in delayed],
                           s=24, marker='x', c='gray', linewidths=.7)
            ax.plot([0, limit], [0, limit], '--', color='black', lw=1, label='PE = PL')
            ax.axhline(5, color='#d62728', ls=':', lw=1.2)
            ax.axvline(5, color='#d62728', ls=':', lw=1.2)
            ax.set(xlim=(0, limit), ylim=(0, limit), xlabel=f'{axis}PE (m)',
                   title=f'FDE {fde.upper()} | current N={points}/{schedule}\nexcluded={excluded}; late spatial reference={late}')
            ax.set_aspect('equal', adjustable='box')
            ax.grid(alpha=.15)
        axes[0].set_ylabel(f'{axis}PL (m)')
        handles = [Line2D([], [], marker='o', ls='', color=phase_colors[s], label=s.replace('_', ' ')) for s in PHASES]
        handles += [Line2D([], [], marker=m, ls='', color='black', label=str(seed)) for m, seed in zip(markers, SEEDS)]
        handles += [Line2D([], [], marker='x', ls='', color='gray', label='late: unavailable')]
        fig.legend(handles=handles, loc='lower center', ncol=6, fontsize=8, frameon=False)
        fig.suptitle(f'{prefix}: {suffix} Stanford plot | research AL=5 m')
        fig.text(.5, .15, 'Actual terminal tag, same-scale ON/OFF; no clipping or synthetic points.\n'
                 'Conditional source event epsilon=1e-6; production OFF; finite zero underbound is not a risk proof.',
                 ha='center', fontsize=8)
        fig.subplots_adjust(bottom=.25, top=.84, wspace=.2)
        fig.savefig(ROOT/f'docs/fde_next/sc6_closure_{suffix}_stanford.png')
        plt.close(fig)
    figure_metrics = ['ape', 'hpe', 'vpe', 'velocity_error', 'rotation_error', 'accel_bias_error',
                      'baerr_x', 'baerr_y', 'baerr_z', 'bgerr_x', 'bgerr_y', 'bgerr_z']
    fig, axes = plt.subplots(4, 3, figsize=(13, 12))
    colors = {'S0': '#377eb8', 'S1': '#ff7f00', 'S2': '#984ea3'}
    for ax, metric in zip(axes.flat, figure_metrics):
        for pair in pairs:
            if pair['entry']['group'] != 'P0':
                continue
            v = pair['metrics'][metric]
            if v['on'] is None or v['off'] is None:
                continue
            ax.plot([0, 1], [v['off'], v['on']], color=colors[pair['entry']['case']],
                    marker=markers[SEEDS.index(pair['entry']['seed'])], alpha=.8, lw=1, markersize=4)
        unit = 'm' if metric in ['ape', 'hpe', 'vpe'] else 'm/s' if metric == 'velocity_error' else 'rad' if metric == 'rotation_error' else 'rad/s' if metric.startswith('bg') else 'm/s²'
        ax.set_xticks([0, 1], ['OFF', 'ON'])
        ax.set_title(metric.replace('_', ' ')+' RMSE')
        ax.set_ylabel(unit)
        ax.grid(alpha=.18)
    fig.suptitle(f'{prefix}: paired terminal state accuracy (each line = one independent seed)')
    fig.legend(handles=[Line2D([], [], color=c, label=case) for case, c in colors.items()],
               loc='lower center', ncol=3, frameon=False)
    fig.text(.5, .035, 'Worsening is retained; no trajectory alignment. S0/S1/S2 equally weighted; N=5 per scene when complete.', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .06, 1, .96))
    fig.savefig(ROOT/'docs/fde_next/sc6_closure_fde_accuracy.png')
    plt.close(fig)
    report(runs, pairs)


def report(runs, pairs):
    f = lambda x: 'NA' if x is None else f'{x:.6g}' if isinstance(x, (int, float)) else str(x)
    registration = json.loads((CACHE/'registration.json').read_text())
    lines = ['# SC6 Closure results', '', '**IN PROGRESS — W0 paired evaluation. W1–W3 are not yet completed.**', '',
             'Historical `SC6_RESULTS.md` and its strict A18 OPEN remain unchanged. Production publication stays OFF.', '',
             f"Entry HEAD `{registration['initial_head']}`. Preregistration SHA256 `{sha(CACHE/'registration.json')}`.",
             'Protocol: `config/sc6_closure.yaml`; matrix/scoring/plot source: `tools/sc6_closure.py`.',
             'All runs below use the same actual Sc6Service and paced 200Hz/20Hz shell. FDE OFF retains nominal FGO, monitoring, raw PL and receiver gate.',
             'Input audit contains hex-float raw observations, IDs, stamps, initial observation/draws, shared ADC births, RW draws, bias path and injections; pairs assert SHA256 and exact byte equality.',
             'Timing uses nearest rank on every measurable admitted service (including abort and timeout); never-started latched refusals are counted separately.',
             'Phase precedence: quarantine/bridge, detected active, active, late unrecovered, post rejoin, healthy before. Latest commenced IMU episode wins; UWB active/healthy scores are parallel labels, not added to the overall RMSE denominator.',
             'GT scoring checks terminal tag=p+R*lever, world frame, independent SO(3) log/magnitude in [0,pi], velocity and bias errors. RPE is body relative pose at deltas 1/20, on common terminal indices.', '',
             '|Run|scheduled/rows/attempted/committed/raw/timely|P50/P95/P99/max core ms|P50/P95/P99/max sink ms|core40/sink50|queue|',
             '|---|---|---|---|---|---|']
    for r in runs:
        timing = lambda k: '/'.join(f(r['timing'][k][q]) for q in ['p50', 'p95', 'p99', 'max'])
        lines.append(f"|{label(r['entry'])}|"+'/'.join(str(r[k]) for k in ['scheduled', 'rows', 'attempted', 'committed', 'qualified', 'timely'])+
                     f"|{timing('core_ms')}|{timing('delivery_ms')}|{r['core40']}/{r['sink50']}|{r['input_counters'].get('queue_peak', 'NA')}|")
    lines += ['', '|Pair|scheduled/paired/missing|tag OFF/ON m; delta m;%|velocity OFF/ON m/s|SO3 OFF/ON rad|accel/gyro bias delta|', '|---|---|---|---|---|---|']
    for p in pairs:
        m = p['metrics']; a = m['ape']
        lines.append(f"|{label(p['entry'])}|{p['scheduled']}/{p['paired']}/{p['paired_missing']}|{f(a['off'])}/{f(a['on'])}; {f(a['improvement'])}; {f(a['percent'])}%|"+
                     f"{f(m['velocity_error']['off'])}/{f(m['velocity_error']['on'])}|{f(m['rotation_error']['off'])}/{f(m['rotation_error']['on'])}|{f(m['accel_bias_error']['improvement'])}/{f(m['gyro_bias_error']['improvement'])}|")
    lines += ['', 'Seed is the statistical unit; descriptive seed bootstrap uses 10000 resamples, fixed bootstrap seed61020. No independent-frame inference. N<5 is incomplete.', '',
              '|Scene/metric|N|delta mean/median/range|delta descriptive95 interval|mean %|', '|---|---|---|---|---|']
    seed_stats = seed_summary(pairs)
    for scene, metrics in seed_stats.items():
        for metric, s in metrics.items():
            lines.append(f"|{scene}/{metric}|{s['N']}|{f(s['mean'])}/{f(s['median'])}/[{f(s['minimum'])},{f(s['maximum'])}]|{s['descriptive_seed_bootstrap95']}|{f(s['percent_mean'])}|")
    lines += ['', 'Preregistered scene-equal aggregate (S0/S1/S2 each 1/3; no scenario selected by benefit):', '', '|metric|equal-weight mean delta|equal-weight mean seed %|', '|---|---|---|']
    for metric in METRICS:
        means = [seed_stats[c][metric]['mean'] for c in ['S0', 'S1', 'S2']]
        percents = [seed_stats[c][metric]['percent_mean'] for c in ['S0', 'S1', 'S2']]
        lines.append(f"|{metric}|{f(float(np.mean(means))) if all(x is not None for x in means) else 'NA'}|{f(float(np.mean(percents))) if all(x is not None for x in percents) else 'NA'}|")
    lines += ['', 'Stanford uses classic actual-output PE versus PL, not all-geometries Stanford–ESA enumeration. Region definitions follow the [ESA reference](https://gssc.esa.int/navipedia/index.php/The_Stanford_%E2%80%93_ESA_Integrity_Diagram:_Focusing_on_SBAS_Integrity).',
              'Only same-stamp/version/source, finite, spatial-qualified and current receipts enter the colored plot. Late spatial-qualified points are gray and unavailable. Missing outputs are never fabricated.',
              'MI/HMI below are counterfactual numerical candidates; operational HMI = NOT_QUALIFIED. Zero observed underbound does not establish epsilon=1e-6.', '',
              '|Run/AL/axis|scheduled/current/plot-excluded/late|covered/underbound|nominal/MI/HMI/unavailable|spatial/timely AL admissible|', '|---|---|---|---|---|']
    for r in runs:
        for al, axes in r['stanford'].items():
            for axis, c in axes.items():
                lines.append(f"|{label(r['entry'])}/{al}/{axis}|{c['scheduled']}/{c['time_current']}/{c['excluded_from_plot']}/{c['expired']}|{c['covered']}/{c['underbound']}|{c['nominal_admissible']}/{c['MI_candidate']}/{c['HMI_candidate']}/{c['AL_unavailable']}|{c['spatial_AL_admissible']}/{c['timely_AL_admissible']}|")
    lines += ['', 'TTD/TTI/TTR below are respectively source-stamp and actual steady-clock action/commit latencies from registered onset, not an operator-hold latency.', '', '|Run/episode axis|onset/duration/amplitude|FD/bridge/prior/rejoin|source TTD/TTI/TTR ms|wall TTD/TTI/TTR ms|rejoin4; UWB isolation|', '|---|---|---|---|---|---|---|']
    for r in runs:
        for e in r['episodes']:
            lines.append(f"|{label(r['entry'])}/{e['axis']}|{e['onset']}/{e['duration_ms']}/{e['amplitude']}|{e['first_fd']}/{e['first_bridge']}/{e['first_prior_repair']}/{e['first_rejoin']}|{f(e['TTD_ms'])}/{f(e['TTI_ms'])}/{f(e['TTR_ms'])}|{f(e.get('TTD_wall_ms'))}/{f(e.get('TTI_wall_ms'))}/{f(e.get('TTR_wall_ms'))}|{e['rejoin4']}; {e['correct_uwb_exclusion']}|")
    lines += ['', '<details><summary>Complete run and unique-phase metrics (n/RMSE/P95/max), paired differences and frozen identities</summary>', '']
    for r in runs:
        lines += [f"**{label(r['entry'])}**", '',
                  f"ELF `{r['identity']['elf']}`; input `{r['input_sha']}`; trace `{r['trace_sha']}`; rc={r['returncode']}; false healthy UWB exclusion epochs={r['false_uwb_exclusions']}.", '',
                  '|metric|all n/RMSE/P95/max|'+'|'.join(PHASES)+'|', '|---|---|'+'---|'*len(PHASES)]
        for key, s in r['metrics'].items():
            cell = lambda m: '/'.join(f(m[k]) for k in ['n', 'rmse', 'p95', 'max'])
            lines.append('|'+key+'|'+cell(s)+'|'+ '|'.join(cell(r['stages'][phase_name][key]) for phase_name in PHASES)+'|')
        if 'uwb_stages' in r:
            lines += ['', '|Parallel UWB label/metric|n/RMSE/P95/max|', '|---|---|']
            for stage_name, metrics in r['uwb_stages'].items():
                for key, s in metrics.items():
                    lines.append(f'|{stage_name}/{key}|'+cell(s)+'|')
    lines += ['', 'All pair metric differences (OFF−ON; improvement%; common valid n):', '', '|Pair/metric|OFF/ON|delta;%|n|', '|---|---|---|---|']
    for p in pairs:
        for key, s in p['metrics'].items():
            lines.append(f"|{label(p['entry'])}/{key}|{f(s['off'])}/{f(s['on'])}|{f(s['improvement'])};{f(s['percent'])}|{s['n']}|")
    lines += ['', '</details>', '', 'W0 tests: `python3 -m unittest discover -s tools -p test_sc6_closure.py`; native regression evidence in own `.cache/sc6-closure/checks.json`.',
              'Reproduce: `python3 tools/sc6_closure.py checks --build`; `register`; `paired --group P0`; `stanford`. Registration refuses overwrite; traces are checksum-validated before plotting.', '',
              '|Clause|Status|', '|---|---|']
    for i in range(1, 21):
        lines.append(f'|C{i:02d}|OPEN pending full W0→W3 audit|')
    for suffix in ['fde_accuracy', 'horizontal_stanford', 'vertical_stanford']:
        lines += ['', f'![Actual Closure {suffix}](sc6_closure_{suffix}.png)']
    (ROOT/'docs/fde_next/SC6_CLOSURE_RESULTS.md').write_text('\n'.join(lines)+'\n')


def main():
    global CACHE
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['register', 'paired', 'checks', 'stanford', 'compare', 'rotation'])
    p.add_argument('--group', choices=['P0', 'P1', 'P2', 'P3', 'P4', 'N'], default='P0')
    p.add_argument('--build', action='store_true')
    p.add_argument('--campaign', choices=['w0', 'w1', 'w2', 'final'], default='w0',
                   help='Keep earlier candidates and failures; never overwrite their frozen traces')
    p.add_argument('--case', help='Run only a case already in the preregistered matrix')
    p.add_argument('--revision', choices=['endpoint-fixed', 'rotation-study'], help='Preserve separate diagnostic candidates and trials')
    a = p.parse_args()
    if a.campaign != 'w0':
        CACHE = CACHE/a.campaign
    if a.revision:
        CACHE = CACHE/a.revision
    if a.command == 'register':
        register()
    elif a.command == 'paired':
        if a.case:
            entries = json.loads((CACHE/'registration.json').read_text())['matrix']
            selected = [e for e in entries if e['group'] == a.group and e['case'] == a.case]
            if not selected:
                raise ValueError('case not in registered group')
            for entry in selected:
                run(entry)
            paired_scores()
        else:
            paired(a.group)
    elif a.command == 'checks':
        checks(a.build)
    elif a.command == 'compare':
        compare()
    elif a.command == 'rotation':
        rotation_comparison()
    else:
        plots()


if __name__ == '__main__':
    main()
