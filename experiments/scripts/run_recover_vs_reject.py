#!/usr/bin/env python3
"""Locked STAR-loc admission/preparation and serial task accounting.

Unresolved source IMU semantics are a hard stop, including for SFUISE. Staging
is deliberately not published as an IMU-frame cache. No estimator is reimplemented.
"""
import argparse
import csv
import datetime as dt
import hashlib
import io
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import numpy as np
import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parents[2]
WS = ROOT.parents[1]
sys.path.insert(0, str(ROOT / 'tools/paper'))
from nlos_injection import cache_id, csv_bytes

RECORDINGS = ('zigzag_s4', 'loop-3d_s3', 'zigzag_s3')
METHODS = ('suppress_all', 'lcb_fixed_full', 'robust_cauchy', 'SFUISE-ToA')
TASKS = ('producer',) + METHODS
CONFIG = ROOT / 'config/paper/ie0911/sfuise_walk1_pl_bidirectional_clean.yaml'
PROTOCOL = ROOT / 'experiments/RECOVER_VS_REJECT_PROTOCOL.md'
GEOMETRY = ROOT / 'experiments/recover_vs_reject_geometry.json'
UWB_COLUMNS = ('time_s', 'range', 'from_id', 'to_id')
IMU_COLUMNS = ('time_s',) + tuple(p+a for p in ('linear_acceleration_', 'angular_velocity_') for a in 'xyz')
UWB_HEADER = ('obs_id,source_message_index,source_range_index,source_observation_index,'
              'sensor_time_s,tag_id,anchor_id,observed_range_m,fp_rssi_dbm,rx_rssi_dbm,'
              'source_valid,source_validity_reason_hex').split(',')
IMU_HEADER = ('source_index,sensor_time_s,acc_x_mps2,acc_y_mps2,acc_z_mps2,'
              'gyro_x_radps,gyro_y_radps,gyro_z_radps,has_orientation,qw,qx,qy,qz').split(',')
PAIR_KEYS = ('measurement_id', 'raw_uwb_sha256', 'imu_sha256', 'input_plan_hash',
             'nominal_sigma_hash', 'original_values_hash', 'common_preparation_id',
             'support_hash', 'stage2_cache_id', 'stage2_values_hash')
EVALUATION = dict(grid_hz=10, nearest_tolerance_s=.02, gt_max_bracket_s=.05,
                  alignment='SE3_SCALE_1_FULL_COMMON_INTERVAL', reference='tag1_antenna',
                  window_error_m=.5, window_min_duration_s=2., window_min_count=5,
                  window_max_gap_s=1., difference='recover_minus_reject', extrapolation=False)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''): h.update(chunk)
    return h.hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def write(path, value):
    path = Path(path)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def read(path):
    return json.loads(Path(path).read_text())


def stable_id(recording, row):
    h = 1469598103934665603
    for b in ('%s|source_message=%d|source_range=0' % (recording, row)).encode():
        h = ((h ^ b) * 1099511628211) & ((1 << 64)-1)
    return h


def projection(path, fields):
    # pandas never materializes other columns (including GT and fitted range).
    d = pd.read_csv(path, usecols=list(fields), dtype=str, keep_default_na=False)
    rows = [{'source_row': i, **dict(zip(fields, values))}
            for i, values in enumerate(d.loc[:, list(fields)].itertuples(index=False, name=None))]
    return rows


def rotation(value):
    r = np.asarray(value, dtype=float)
    if r.shape != (3, 3) or not np.isfinite(r).all() or not np.allclose(r.T @ r, np.eye(3), atol=1e-10, rtol=0) or abs(np.linalg.det(r)-1)>1e-10:
        raise ValueError('INVALID_ROTATION')
    return r


def compose_lever(T_rig_camera, T_camera_imu, tag_in_rig):
    """T_a_b maps b coordinates to a; lever returned in IMU coordinates."""
    transforms = []
    for value in (T_rig_camera, T_camera_imu):
        t = np.asarray(value, dtype=float)
        if t.shape != (4,4) or not np.isfinite(t).all() or not np.array_equal(t[3], [0,0,0,1]):
            raise ValueError('INVALID_TRANSFORM')
        rotation(t[:3,:3]); transforms.append(t)
    t = transforms[0] @ transforms[1]
    return t[:3,:3].T @ (np.asarray(tag_in_rig)-t[:3,3])


def require_imu_semantics(geometry):
    if geometry.get('status') not in ('ADMITTED_INDEPENDENT_APPROXIMATE','ADMITTED_GT_ASSISTED_APPROXIMATE'):
        raise ValueError('BLOCKED_GEOMETRY_OR_IMU')
    s = geometry.get('imu', {})
    # Origin changes need angular acceleration terms; never silently treat
    # acceleration at the rig origin as acceleration at the IMU origin.
    if (s.get('acceleration_origin') != 'IMU' or s.get('angular_velocity_origin') != 'IMU'
            or s.get('acceleration_semantics') != 'SPECIFIC_FORCE'
            or not s.get('upstream_processing_source') or not s.get('transform_source')):
        raise ValueError('BLOCKED_GEOMETRY_OR_IMU')
    return rotation(s['R_imu_acc']), rotation(s['R_imu_gyro'])


def cache_payloads(recording, uwb, imu, geometry):
    """Only callable with closed semantics; tests use explicit engineering geometry."""
    ra, rg = require_imu_semantics(geometry)
    out_u, out_i = [], []
    for row in uwb:
        if int(row['from_id']) != 1: continue
        i = int(row['source_row'])
        z, t = float(row['range']), float(row['time_s'])
        if not math.isfinite(t): raise ValueError('NONFINITE_TIME')
        valid = math.isfinite(z) and z > 0
        out_u.append(dict(zip(UWB_HEADER, [stable_id(recording, i), i, 0, i, row['time_s'],
            1, int(row['to_id']), row['range'], 0, 0, int(valid), '' if valid else 'INVALID_SOURCE_RANGE'.encode().hex()])))
    for i, row in enumerate(imu):
        acc = ra @ np.array([float(row['linear_acceleration_'+a]) for a in 'xyz'])
        gyro = rg @ np.array([float(row['angular_velocity_'+a]) for a in 'xyz'])
        if not np.isfinite([float(row['time_s']), *acc, *gyro]).all(): raise ValueError('NONFINITE_IMU')
        out_i.append(dict(zip(IMU_HEADER, [i, format(float(row['time_s'])+geometry.get('imu',{}).get('time_offset_s',0.),'.17g'), *acc, *gyro, 0, 1, 0, 0, 0])))
    for rows, field in ((out_i, 'sensor_time_s'), (out_u, 'sensor_time_s')):
        if any(float(b[field]) < float(a[field]) for a,b in zip(rows, rows[1:])):
            raise ValueError('NONMONOTONIC_SOURCE_TIME')
    u, im = csv_bytes(out_u, UWB_HEADER), csv_bytes(out_i, IMU_HEADER)
    projected = {'uwb': uwb, 'imu': imu}
    m = dict(schema='nlos_measurement_cache_v2', base_recording_id=recording,
             base_source_sha256='sha256:'+digest(projected), recording_time_origin_s=0.,
             time_basis='sensor_time_from_recording_origin',
             imu_units='acc_mps2,gyro_radps,orientation_quaternion_wxyz', uwb_units='time_s,range_m,rssi_dbm',
             uwb_message_grouping='source_message_index', imu_file='imu.csv', uwb_file='uwb_observations.csv',
             imu_sha256='sha256:'+hashlib.sha256(im).hexdigest(), uwb_sha256='sha256:'+hashlib.sha256(u).hexdigest(),
             imu_count=len(out_i), uwb_observation_count=len(out_u), uwb_message_count=len(out_u),
             transform_sha256='sha256:'+digest({'version':'STARLOC_WHITELIST_IMU_V1','geometry':geometry,
                 'orientation':'unused_identity_placeholder','rssi':'neutral_zero_placeholder','range':'RAW_UNCORRECTED'}))
    m['cache_id'] = cache_id(m)
    return m, u, im


def require_pair(a, b):
    for key in PAIR_KEYS:
        if not a.get(key) or not b.get(key) or a[key] != b[key]:
            raise ValueError('PAIR_IDENTITY_MISMATCH:'+key)


def science_process(command, directory, timeout=1800):
    """One process tree; no retry. Reserve ledger before launch, stream logs."""
    if not 0 < timeout <= 1800: raise ValueError('INVALID_BUDGET')
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=False)
    status = dict(command=list(map(str, command)), status='RUNNING', exit_code=None, timeout_s=timeout)
    write(directory/'process.json', status)
    start = time.monotonic()
    with (directory/'stdout.log').open('w') as out, (directory/'stderr.log').open('w') as err:
        try:
            process = subprocess.Popen(command, stdout=out, stderr=err, start_new_session=True)
        except OSError as exc:
            status.update(status='FAILURE',exit_code=127,reason=str(exc),wall_time_s=time.monotonic()-start)
            write(directory/'process.json',status)
            return status
        try:
            code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait(); code=124
        finally:
            # Clean surviving descendants even when the direct child exits early.
            try: os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError: pass
    status.update(exit_code=code, status='TIMEOUT' if code==124 else ('SUCCESS' if code==0 else 'FAILURE'),
                  wall_time_s=time.monotonic()-start)
    write(directory/'process.json', status)
    return status


def serial_tasks(recordings, call):
    """Testable ordering/one-shot accounting; independent tasks survive producer failure."""
    result = []
    for recording in recordings:
        producer_ok = False
        for task in TASKS:
            if recording['status'] != 'ADMITTED_INDEPENDENT_APPROXIMATE':
                row = dict(status='NOT_RUN', reason='BLOCKED_GEOMETRY_OR_IMU', exit_code=None)
            elif task in METHODS[:2] and not producer_ok:
                row = dict(status='NOT_RUN', reason='PRODUCER_FAILED', exit_code=None)
            else:
                try:
                    row = call(recording, task)
                except Exception as exc:
                    row = dict(status='FAILURE',reason=type(exc).__name__+': '+str(exc),exit_code=None)
                if task == 'producer': producer_ok = row['status']=='SUCCESS'
            result.append(dict(recording=recording['recording'], task=task, **row))
    return result


def frozen_files():
    files = list((ROOT/'src').glob('*.cpp')) + list((ROOT/'include/uifgo').glob('*.h'))
    files += [CONFIG, ROOT/'tools/run_ie_paper.cpp', ROOT/'tools/paper/run_experiments.py',
              ROOT/'doc/v2/paper_structure.tex', ROOT/'doc/v2/v2_roadmap.md']
    files += list((ROOT/'experiments/compare_algorithm/SFUISE/sfuise').rglob('*.cpp'))
    files += list((ROOT/'experiments/compare_algorithm/SFUISE/sfuise').rglob('*.h'))
    return {str(p.relative_to(ROOT)): sha(p) for p in files if p.is_file()}


def prepare(out):
    out = Path(out); out.mkdir(parents=True, exist_ok=False)
    cfg = yaml.safe_load(CONFIG.read_text())
    assert cfg['keyframe']['step']==4
    assert [cfg['nlos'][k] for k in ('cusum_forward_kappa','cusum_backward_kappa','cusum_forward_h','cusum_backward_h')] == [.5,.5,7.0234689587858723,7.0234689587858714]
    geometry = read(GEOMETRY)
    params = {p['name']:p for p in read(ROOT/'data/starloc/dataset_params.json')}
    lock = dict(schema='ICRA_RECOVER_REJECT_LOCK_V1', recordings=list(RECORDINGS), methods=list(METHODS),
                role='development', evaluation=EVALUATION, task_budget=15, per_tree_timeout_s=1800,
                protocol_sha256=sha(PROTOCOL), geometry_sha256=sha(GEOMETRY),
                git_commit=subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(),
                frozen_files=frozen_files(), implementation={str(p.relative_to(ROOT)):sha(p) for p in
                    [Path(__file__).resolve(), ROOT/'experiments/scripts/evaluate_recover_vs_reject.py',
                     ROOT/'experiments/scripts/test_recover_vs_reject.py']},
                rows=[], beta_status='MISSING_CALIBRATION_UNCORRECTED', config_template=cfg)
    for name in RECORDINGS:
        src = ROOT/'data/starloc/data'/name
        d = out/name; d.mkdir()
        assert params[name]['landmarks']=='v2'
        uwb = projection(src/'uwb.csv', UWB_COLUMNS)
        imu = projection(src/'imu.csv', IMU_COLUMNS)
        (d/'uwb_source.csv').write_bytes(csv_bytes(uwb, ['source_row',*UWB_COLUMNS]))
        (d/'imu_source.csv').write_bytes(csv_bytes(imu, ['source_row',*IMU_COLUMNS]))
        selected = [r for r in uwb if int(r['from_id'])==1]
        ids = [{'source_row':r['source_row'], 'csv_line':r['source_row']+2,
                'obs_id':stable_id(name,r['source_row'])} for r in selected]
        (d/'obs_id_map.csv').write_bytes(csv_bytes(ids))
        times = [float(r['time_s']) for r in selected]
        imut = np.array([float(r['time_s']) for r in imu])
        status = geometry['recordings'][name]['status']
        if status != 'BLOCKED_GEOMETRY_OR_IMU':
            # This release has no admitted real IMU transform. A manually edited
            # status must not bypass engineering/ROS/C++ admission on a later run.
            raise ValueError('UNREVIEWED_GEOMETRY_CHANGE_REQUIRES_ADMISSION_IMPLEMENTATION')
        row = dict(recording=name, status=status, reasons=geometry['recordings'][name]['reasons'],
                   measurement_id=digest({'uwb':uwb,'imu':imu}), source_row_count=len(uwb),
                   tag1_rows=len(selected), imu_rows=len(imu), anchor_ids=sorted({int(r['to_id']) for r in selected}),
                   interval_s=[min(times),max(times)], imu_interval_s=[float(imut.min()),float(imut.max())],
                   uwb_nonmonotonic=int(np.sum(np.diff(times)<0)), imu_nonmonotonic=int(np.sum(np.diff(imut)<0)),
                   imu_max_gap_s=float(np.max(np.diff(imut))), cache_status='NOT_PUBLISHED_UNRESOLVED_IMU',
                   source_sha256={f:sha(src/f) for f in ('uwb.csv','imu.csv','calib.json')},
                   staging_sha256={f:sha(d/f) for f in ('uwb_source.csv','imu_source.csv','obs_id_map.csv')})
        write(d/'preparation.json', row); lock['rows'].append(row)
    lock['anchors_sha256']=sha(ROOT/'data/starloc/mocap/uwb_markers_v2.csv')
    lock['lock_id']=digest(lock); write(out/'lock.json',lock)
    print(json.dumps({'output':str(out.resolve()), 'status':'BLOCKED_GEOMETRY_OR_IMU','recordings':3}))
    return 2


def verify(out):
    lock=read(out/'lock.json'); content=dict(lock); identity=content.pop('lock_id')
    if digest(content)!=identity: raise ValueError('LOCK_TAMPERED')
    if lock['recordings']!=list(RECORDINGS) or lock['methods']!=list(METHODS) or lock['evaluation']!=EVALUATION:
        raise ValueError('PROTOCOL_CHANGED')
    for f,h in {**lock['frozen_files'], **lock['implementation'],
                str(PROTOCOL.relative_to(ROOT)):lock['protocol_sha256'],
                str(GEOMETRY.relative_to(ROOT)):lock['geometry_sha256']}.items():
        if sha(ROOT/f)!=h: raise ValueError('LOCKED_FILE_CHANGED:'+f)
    for row in lock['rows']:
        for f,h in row['staging_sha256'].items():
            if sha(out/row['recording']/f)!=h: raise ValueError('STAGING_CHANGED:'+f)
        for f,h in row['source_sha256'].items():
            if sha(ROOT/'data/starloc/data'/row['recording']/f)!=h: raise ValueError('SOURCE_CHANGED:'+f)
    if sha(ROOT/'data/starloc/mocap/uwb_markers_v2.csv')!=lock['anchors_sha256']: raise ValueError('ANCHORS_CHANGED')
    return lock


def execute(out):
    lock=verify(out)
    ledger=out/'execution.json'
    if ledger.exists(): raise ValueError('EXECUTION_ALREADY_ACCOUNTED_NO_RETRY')
    def no_admitted_backend(recording, task):
        raise ValueError('REAL_BACKEND_NOT_ADMITTED')
    rows=serial_tasks(lock['rows'],no_admitted_backend)
    write(ledger,dict(lock_id=lock['lock_id'], tasks=rows, scientific_processes_started=0,
                     status='BLOCKED_GEOMETRY_OR_IMU'))
    return 2


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage',choices=['prepare','preflight','execute','evaluate','verify'])
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--gt-assisted',action='store_true')
    a=p.parse_args(); out=a.run.resolve()
    if a.gt_assisted:
        import rr_admitted
        result=getattr(rr_admitted,a.stage)(out)
        if a.stage=='verify': print('LOCK_AND_SOURCE_HASHES_PASS'); return 0
        return result
    if a.stage=='prepare': return prepare(out)
    if a.stage=='execute': return execute(out)
    if a.stage=='verify': verify(out); print('LOCK_AND_SOURCE_HASHES_PASS'); return 0
    from evaluate_recover_vs_reject import evaluate
    return evaluate(out)

if __name__=='__main__': raise SystemExit(main())
