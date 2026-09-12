#!/usr/bin/env python3
"""Evaluator-only raw absolute range audit; no estimator imports or execution."""
import argparse
import ast
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import traceback
import uuid

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag
import requests
import yaml

from evaluate_range_gt import associate, sha

ROOT = Path(__file__).resolve().parents[2]
PRIVATE = ROOT.parents[1] / 'evaluator_private/icra/obstacle_audit'
THRESHOLDS = (0., .2, .5, 1.)
MILUV_COMMIT = '399ad46f4e6e88c3c98b40aae33fe8e2c6719cff'


def clean_json(x):
    if isinstance(x, dict): return {str(k): clean_json(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)): return [clean_json(v) for v in x]
    if isinstance(x, np.ndarray): return clean_json(x.tolist())
    if isinstance(x, np.generic): return clean_json(x.item())
    if isinstance(x, float) and not np.isfinite(x): return None
    return x


def dump(path, obj):
    path.write_text(json.dumps(clean_json(obj), indent=2, allow_nan=False) + '\n')


def geometric_range(tag, anchor):
    """One physical antenna-distance implementation for all adapters."""
    return np.linalg.norm(np.asarray(tag) - np.asarray(anchor), axis=-1)


def interpolate(query, times, positions, quaternions=None, max_gap=.05):
    times = np.asarray(times, float)
    positions = np.asarray(positions, float)
    if not len(times) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('GT time must be finite and strictly increasing; no silent sort/dedup')
    if quaternions is None: quaternions = np.tile([0., 0., 0., 1.], (len(times), 1))
    quaternions = np.asarray(quaternions, float)
    valid = np.isfinite(positions).all(axis=1) & np.isfinite(quaternions).all(axis=1)
    valid &= np.abs(np.linalg.norm(quaternions, axis=1) - 1) <= 1e-3
    gt = list(zip(times, positions, quaternions))
    out = np.full((len(query), 3), np.nan)
    rotations = np.full((len(query), 4), np.nan)
    status = []
    for i, t in enumerate(query):
        if not np.isfinite(t): status.append('INVALID_TIMESTAMP'); continue
        j = np.searchsorted(times, t)
        endpoints = [j] if j < len(times) and times[j] == t else [j-1, j]
        if any(k < 0 or k >= len(times) for k in endpoints):
            status.append('OUTSIDE_GT_SUPPORT'); continue
        if not all(valid[k] for k in endpoints):
            status.append('INVALID_GT_ENDPOINT'); continue
        p, q, _, _, s = associate(t, gt, times, max_gap)
        status.append(s)
        if s == 'AVAILABLE': out[i], rotations[i] = p, q
    return out, rotations, np.array(status)


def longest_episode(times, errors, threshold, max_gap=1.):
    """Every invalid/nonpositive row breaks a run, including unmatched GT rows."""
    best = {'duration_s': 0., 'count': 0, 'start_s': None, 'end_s': None, 'mean_m': None}
    start = None
    count = 0; total = 0.
    previous = None
    for t, e in zip(times, errors):
        good = np.isfinite(t) and np.isfinite(e) and e > threshold
        if not good:
            start, count, total, previous = None, 0, 0., None
            continue
        if start is None or previous is None or t <= previous or t-previous > max_gap:
            start, count, total = float(t), 0, 0.
        count += 1; total += float(e); previous = float(t)
        candidate = {'duration_s': float(t-start), 'count': count,
                     'start_s': start, 'end_s': float(t), 'mean_m': total/count}
        if (candidate['duration_s'], candidate['count']) > (best['duration_s'], best['count']):
            best = candidate
    return best


def time_stats(t):
    t = np.asarray(t, float); t = t[np.isfinite(t)]
    d = np.diff(t); positive = d[d > 0]
    median = float(np.median(positive)) if len(positive) else np.nan
    gaps = positive[positive > 1.]
    return {'time_count': len(t), 'start_s': min(t) if len(t) else None,
            'end_s': max(t) if len(t) else None,
            'backward_steps': int(sum(d < 0)), 'duplicate_steps': int(sum(d == 0)),
            'interarrival_median_s': median,
            'interarrival_p95_s': np.quantile(positive, .95) if len(positive) else None,
            'interarrival_max_s': max(positive) if len(positive) else None,
            'gaps_gt_1s_count': len(gaps), 'gaps_gt_1s_total_s': sum(gaps),
            'gaps_gt_3median_count': int(sum(positive > 3*median)),
            'cadence_missing_estimate': int(sum(np.maximum(np.rint(positive/median)-1, 0)))
                if np.isfinite(median) else None}


def summarize(df):
    e = df.error_m.to_numpy(float); valid = np.isfinite(e); v = e[valid]; pos = v[v > 0]
    result = {'sample_count': len(df), 'valid_range_count': int(sum(np.isfinite(df.raw_m) & (df.raw_m > 0))),
              'gt_matched_count': int(sum(np.isfinite(df.gt_range_m))),
              'error_sample_count': len(v), 'unavailable_error_count': int(sum(~valid)),
              'raw_range_error_median_m': np.median(v) if len(v) else None,
              'raw_range_error_mean_m': np.mean(v) if len(v) else None,
              'raw_range_error_rmse_m': np.sqrt(np.mean(v*v)) if len(v) else None,
              'raw_range_error_absolute_p95_m': np.quantile(abs(v), .95) if len(v) else None,
              'raw_range_error_signed_p95_m': np.quantile(v, .95) if len(v) else None,
              'positive_tail_mean_m': np.mean(pos) if len(pos) else None,
              'positive_tail_p95_m': np.quantile(pos, .95) if len(pos) else None}
    for threshold in THRESHOLDS:
        key = 'positive_gt_%g' % threshold
        result[key+'_count'] = int(sum(v > threshold))
        result[key+'_fraction'] = float(np.mean(v > threshold)) if len(v) else None
        best = longest_episode(df.time_s, e, threshold)
        result.update({key+'_longest_'+k: value for k, value in best.items()})
    return result


def canonical(times, tags, anchors, ranges, origin):
    n = len(times)
    return pd.DataFrame({'obs_id': [origin+':'+str(i) for i in range(n)],
                         'time_s': np.asarray(times, float), 'tag_id': np.asarray(tags).astype(str),
                         'anchor_id': np.asarray(anchors).astype(str), 'raw_m': np.asarray(ranges, float),
                         'gt_range_m': np.nan, 'association_status': 'UNAVAILABLE_REFERENCE'})


def starloc_anchor_ids(radio_ids, layout):
    mapping = {'4': '12', '5': '15', '6': '16', '7': '11', '9': '9', '10': '13', '11': '10', '12': '6'}
    return [mapping.get(str(x), 'UNKNOWN') if layout == 'v1' else str(x) for x in radio_ids]


def validate_published_geometry(computed, published):
    delta = np.asarray(computed)-np.asarray(published)
    if not np.isfinite(delta).all() or np.max(abs(delta)) > 1e-9:
        raise ValueError('STARLOC_GEOMETRY_ID_MISMATCH: reference not admissible')


def static_rows(u):
    footer_names = ['Distance Mean', 'Distance Std', 'RSSI(dBm) Mean', 'RSSI(dBm) Std',
                    'RSSI_fp(dBm) Mean', 'RSSI_fp(dBm) Std']
    footer = u.timestamp.astype(str).str.strip().isin(footer_names)
    rows = u.loc[~footer].copy()
    rows['source_row'] = rows.index
    rows['anchor_id_raw'] = rows.anchor_id
    ids = rows.anchor_id.astype(str).str.split('anchor_id:').str[-1].str.strip()
    numeric_ids = pd.to_numeric(ids, errors='coerce')
    rows['anchor_id'] = [str(int(x)) if np.isfinite(x) and x == int(x) else 'UNPARSED_SOURCE_ID' for x in numeric_ids]
    for key in ['timestamp', 'Distance', 'Transmission #', 'Reception #']:
        if key in rows: rows[key] = pd.to_numeric(rows[key], errors='coerce')
    return rows, int(sum(footer))


class Audit:
    def __init__(self, out):
        self.out = out
        self.sources = {}; self.links = []; self.anchors = []; self.recordings = []
        self.geometry = []; self.figures = []

    def source(self, path):
        path = Path(path).resolve()
        if str(path) not in self.sources:
            self.sources[str(path)] = {'sha256': sha(path), 'bytes': path.stat().st_size}
        return path

    def csv(self, path, **kwargs):
        return pd.read_csv(self.source(path), **kwargs)

    def finish(self, family, name, condition, reference, df, notes, plot=False):
        identity = {'family': family, 'recording': name, 'condition': condition, 'reference_status': reference}
        df['error_m'] = df.raw_m - df.gt_range_m
        df.loc[~np.isfinite(df.raw_m) | (df.raw_m <= 0), 'error_m'] = np.nan
        rdir = self.out/'recordings'/family/name
        rdir.mkdir(parents=True, exist_ok=False)
        df.to_csv(rdir/'observations.csv.gz', index=False, compression='gzip')
        perlink = []
        finite_t = df.time_s[np.isfinite(df.time_s)]
        beginning = min(finite_t) if len(finite_t) else np.nan
        ending = max(finite_t) if len(finite_t) else np.nan
        for (tag, anchor), group in df.groupby(['tag_id', 'anchor_id'], sort=True):
            timing = time_stats(group.time_s)
            order = group.sort_values('time_s', kind='stable')
            row = dict(identity, tag_id=tag, anchor_id=anchor, **summarize(order), **timing)
            row['leading_gap_s'] = timing['start_s']-beginning if timing['start_s'] is not None else None
            row['trailing_gap_s'] = ending-timing['end_s'] if timing['end_s'] is not None else None
            row['persistent_gt_0.5'] = (row['positive_gt_0.5_longest_duration_s'] >= 2 and
                                         row['positive_gt_0.5_longest_count'] >= 5)
            perlink.append(row); self.links.append(row)
        for anchor, group in df.groupby('anchor_id', sort=True):
            row = dict(identity, anchor_id=anchor, **summarize(group))
            sub = [x for x in perlink if x['anchor_id'] == anchor]
            for threshold in THRESHOLDS:
                key = 'positive_gt_%g_longest_' % threshold
                best = max(sub, key=lambda x: (x[key+'duration_s'], x[key+'count']))
                for k in ['duration_s', 'count', 'start_s', 'end_s', 'mean_m']: row[key+k] = best[key+k]
                row[key+'tag_id'] = best['tag_id']
            row['dropout_link_count'] = len(sub)
            row['gaps_gt_1s_count_sum_links'] = sum(x['gaps_gt_1s_count'] for x in sub)
            row['cadence_missing_estimate_sum_links'] = sum(x['cadence_missing_estimate'] or 0 for x in sub)
            row['max_link_interarrival_s'] = max((x['interarrival_max_s'] or 0) for x in sub)
            self.anchors.append(row)
        recording_stats = summarize(df)
        for threshold in THRESHOLDS:
            key = 'positive_gt_%g_longest_' % threshold
            if perlink:
                best = max(perlink, key=lambda x: (x[key+'duration_s'], x[key+'count']))
                for k in ['duration_s', 'count', 'start_s', 'end_s', 'mean_m']: recording_stats[key+k] = best[key+k]
                recording_stats[key+'tag_id'] = best['tag_id']; recording_stats[key+'anchor_id'] = best['anchor_id']
        self.recordings.append(dict(identity, **recording_stats, link_count=len(perlink),
                                    finite_timestamp_count=int(np.isfinite(df.time_s).sum()),
                                    status='AUDITED' if np.isfinite(df.error_m).any() else 'UNAVAILABLE_GT',
                                    observation_path=str(rdir/'observations.csv.gz'), notes=notes))
        if plot and np.isfinite(df.error_m).any(): self.plot(identity, df, rdir)
        if family != 'HUEC_static' or len(self.recordings) % 100 == 0:
            print(f"{family}/{name}: raw={len(df)} evaluated={np.isfinite(df.error_m).sum()} [{reference}]", flush=True)

    def plot(self, identity, df, rdir):
        anchors = sorted(df.anchor_id.unique())
        fig, axes = plt.subplots(len(anchors), 1, figsize=(11, 2.1*len(anchors)), squeeze=False, sharex=True)
        t0 = df.time_s.min()
        for ax, anchor in zip(axes[:, 0], anchors):
            for tag, g in df[df.anchor_id == anchor].groupby('tag_id'):
                ax.scatter(g.time_s-t0, g.error_m, s=2, alpha=.55, label='tag '+tag, rasterized=True)
            ax.axhline(0, color='black', lw=.6); ax.axhline(.5, color='red', ls='--', lw=.7)
            ax.set_ylabel('anchor '+anchor+'\nerror [m]'); ax.grid(alpha=.2); ax.legend(loc='upper right', fontsize=7)
        axes[-1, 0].set_xlabel('seconds since first raw UWB observation')
        fig.suptitle(identity['family']+'/'+identity['recording']+'\n'+identity['reference_status'], fontsize=10)
        fig.tight_layout(rect=(0, 0, 1, .97)); fig.savefig(rdir/'range_error.png', dpi=120); plt.close(fig)
        self.figures.append(str(rdir/'range_error.png'))

    def miluv(self, sequence, sources):
        name = sequence.name; base = sequence/'ifo001'
        u = self.csv(base/'uwb_range.csv', usecols=['timestamp', 'from_id', 'to_id', 'range_raw', 'gt_range'])
        g = self.csv(base/'mocap.csv'); self.source(base/'imu_px4.csv'); self.source(sequence/'timeshift.yaml')
        exp = pd.read_csv(sources/'experiments.csv', dtype={'anchor_constellation': str})
        constellation = exp.loc[exp.experiment == name, 'anchor_constellation'].iloc[0]
        aa = yaml.safe_load((sources/'anchors.yaml').read_text())[constellation]
        aa = {str(k): ast.literal_eval(v) for k, v in aa.items()}
        levers = yaml.safe_load((sources/'tags.yaml').read_text())['ifo001']
        levers = {str(k): ast.literal_eval(v) for k, v in levers.items()}
        df = canonical(u.timestamp, u.from_id, u.to_id, u.range_raw, 'uwb_range.csv')
        p, q, status = interpolate(df.time_s, g.timestamp, g[['pose.position.'+a for a in 'xyz']],
                                  g[['pose.orientation.'+a for a in 'xyzw']])
        ok = status == 'AVAILABLE'
        tag_pos = np.full_like(p, np.nan)
        tag_pos[ok] = p[ok] + Rotation.from_quat(q[ok]).apply(np.array([levers[x] for x in df.tag_id[ok]]))
        known = df.anchor_id.isin(aa)
        coords = np.array([aa.get(x, [np.nan]*3) for x in df.anchor_id])
        df['gt_range_m'] = geometric_range(tag_pos, coords); df['association_status'] = status
        df.loc[~known, 'association_status'] = 'UNKNOWN_ANCHOR'
        df['published_gt_range_m'] = u.gt_range
        discrepancy = df.gt_range_m-u.gt_range
        self.geometry.append({'family': 'MILUV', 'recording': name, 'constellation': constellation,
                              'anchors_m': aa, 'body_to_tags_m': levers,
                              'recomputed_minus_published_gt_rmse_m': np.sqrt(np.nanmean(discrepancy**2)),
                              'recomputed_minus_published_gt_max_abs_m': np.nanmax(abs(discrepancy)),
                              'gt_time': time_stats(g.timestamp), 'imu_time': time_stats(self.csv(base/'imu_px4.csv', usecols=['timestamp']).timestamp)})
        self.finish('MILUV', name, 'obstacle' if name.startswith('cirObstacles') else 'nominal',
                    'PUBLISHED_VICON_GEOMETRY', df,
                    'Raw range_raw; constellation 0; author body-to-tag levers; shared relative seconds; timeshift is epoch origin, not a second offset.', True)

    def starloc(self, sequence, params):
        name = sequence.name
        cols = ['time_s', 'range', 'from_id', 'to_id', 'tag_pos_x', 'tag_pos_y', 'tag_pos_z', 'gt_range',
                'x', 'y', 'z', 'w', 'rot_x', 'rot_y', 'rot_z']
        u = self.csv(sequence/'uwb.csv', usecols=cols)
        aa = self.csv(sequence.parents[1]/'mocap'/('uwb_markers_'+params['landmarks']+'.csv'), index_col=0)
        aa.index = aa.index.astype(str)
        self.source(sequence/'calib.json'); self.source(sequence/'imu.csv')
        df = canonical(u.time_s, u.from_id, u.to_id, u.range, 'uwb.csv')
        marker_ids = starloc_anchor_ids(df.anchor_id, params['landmarks'])
        coords = aa.reindex(marker_ids)[['x', 'y', 'z']].to_numpy()
        df['gt_range_m'] = geometric_range(u[['tag_pos_'+a for a in 'xyz']], coords)
        validate_published_geometry(df.gt_range_m, u.gt_range)
        df['anchor_marker_id'] = marker_ids
        df['association_status'] = np.where(np.isfinite(df.gt_range_m), 'PUBLISHED_TAG_POSITION_AT_RANGE_TIME', 'UNKNOWN_ANCHOR_OR_GT')
        df['published_gt_range_m'] = u.gt_range
        diff = df.gt_range_m-u.gt_range
        body_lever = Rotation.from_quat(u[['rot_x', 'rot_y', 'rot_z', 'w']]).inv().apply(
            u[['tag_pos_'+a for a in 'xyz']].to_numpy()-u[['x', 'y', 'z']].to_numpy())
        lever_check = {str(tag): {'median_m': np.median(body_lever[u.from_id == tag], axis=0),
                                  'max_component_span_m': np.max(np.ptp(body_lever[u.from_id == tag], axis=0))}
                       for tag in u.from_id.unique()}
        self.geometry.append({'family': 'starloc', 'recording': name, 'setup': params['setup'],
                              'landmarks': params['landmarks'], 'published_body_lever_consistency_only': lever_check,
                              'recomputed_minus_published_gt_max_abs_m': np.nanmax(abs(diff)),
                              'anchors_m': aa.to_dict('index')})
        reference = 'PUBLISHED_VICON_REFERENCE_ID_RESOLVED' if params['landmarks'] == 'v1' else 'PUBLISHED_VICON_TAG_REFERENCE'
        self.finish('starloc', name, 'setup_'+params['setup'], reference, df,
                    'Published timestamp-matched Vicon tag position; no IMU/camera-origin substitution; raw range before calibration; no packet NLOS label.',
                    name in ['loop-2d_s1', 'loop-2d_s2', 'loop-2d_s3', 'loop-2d_s4', 'zigzag_s4'])

    def huec_dynamic(self, sequence):
        condition, trajectory, case = sequence.parts[-3:]
        name = '_'.join([condition, trajectory, case]); frames = []
        gt = self.csv(sequence/'trajectory.csv')
        times = gt.timestamp.to_numpy(float)*1e-9
        pos = gt[['x', 'y', 'z']].to_numpy(); pos[:, 2] += 1.0
        self.source(sequence/'imu.csv'); self.source(sequence/'gnss.csv')
        anchor_geometry = {}
        for anchor in [3, 5, 9, 12]:
            u = self.csv(sequence/('A%d.csv' % anchor))
            df = canonical(u['field.stamp']*1e-9, ['UNSPECIFIED_SINGLE_TAG']*len(u),
                           u['field.id'].astype(int), u['field.distanceFromTag'], 'A%d.csv' % anchor)
            p, _, status = interpolate(df.time_s, times, pos, max_gap=.25)
            coords = u[['field.'+a for a in 'xyz']].to_numpy()
            df['gt_range_m'] = geometric_range(p, coords); df['association_status'] = status
            anchor_geometry[str(anchor)] = {'first_m': coords[0], 'max_component_span_m': np.max(np.ptp(coords, axis=0))}
            frames.append(df)
        self.geometry.append({'family': 'HUEC_dynamic', 'recording': name, 'anchors': anchor_geometry,
                              'gt_time': time_stats(times), 'gt_z_unfiltered_minmax_m': [gt.z.min(), gt.z.max()],
                              'gt_position_step_max_m': max(np.linalg.norm(np.diff(pos, axis=0), axis=1)),
                              'gt_abs_z_ge_0.5_count_retained': int(sum(abs(gt.z) >= .5)),
                              'gt_evaluation_rule': 'author trajectory.csv x,y,z+1; no abs(z)<0.5 filtering'})
        self.finish('HUEC_dynamic', name, condition, 'PUBLISHED_GNSS_REFERENCE', pd.concat(frames, ignore_index=True),
                    'GNSS-derived author reference, z+1m from author evaluator; exact lever/heading error budget unclosed; raw nanosecond clock /1e9; 0.25s GT gap.',
                    case == 'Case_1')

    def huec_static(self, path):
        condition, height, filename = path.parts[-3:]
        u, footer_count = static_rows(self.csv(path, usecols=lambda c: c in ['timestamp', 'Distance', 'anchor_id', 'Transmission #', 'Reception #']))
        df = canonical(u.timestamp, ['UNSPECIFIED_SINGLE_TAG']*len(u), u.anchor_id, u.Distance, path.name)
        df['obs_id'] = [path.name+':'+str(i) for i in u.source_row]
        df['anchor_id_raw'] = u.anchor_id_raw.to_numpy()
        counters = {}
        for key in ['Transmission #', 'Reception #']:
            if key in u and len(u):
                d = np.diff(u[key]); counters[key] = {'first': u[key].iloc[0], 'last': u[key].iloc[-1],
                                                       'positive_increments': sum(d[d > 0]), 'resets': sum(d < 0)}
        self.geometry.append({'family': 'HUEC_static', 'recording': condition+'/'+height+'/'+path.stem,
                              'counters': counters, 'footer_rows_not_measurements': footer_count,
                              'reference': 'LASER_NOT_ALLOWED_BY_USER_GT_CONSTRAINT'})
        self.finish('HUEC_static', condition+'/'+height+'/'+path.stem, condition, 'UNAVAILABLE_GT_TYPE_LASER', df,
                    'Static laser reference excluded by Vicon/GNSS-only scope; no IMU/allowed GT; no residual or persistence claim.')

    def own(self, path):
        self.source(path); rows = []; poses = {str(i): [] for i in range(5)}; latency = {}; imu_times = []
        with readable_bag(path, self.out) as bag:
            selected = ['/nlink_linktrack_nodeframe3', '/livox/imu'] + ['/vrpn_client_node/tas_uwb_%d/pose' % i for i in range(5)]
            for topic, msg, bt in bag.read_messages(topics=selected):
                t = msg.header.stamp.to_sec()
                latency.setdefault(topic, []).append(bt.to_sec()-t)
                if topic == selected[0]:
                    for j, node in enumerate(msg.nodes): rows.append((t, str(msg.id), str(node.id), node.dis))
                elif topic == selected[1]: imu_times.append(t)
                else:
                    i = topic.split('tas_uwb_')[1].split('/')[0]; p = msg.pose.position
                    poses[i].append((t, p.x, p.y, p.z))
        u = pd.DataFrame(rows, columns=['t', 'tag', 'anchor', 'raw'])
        df = canonical(u.t, u.tag, u.anchor, u.raw, 'nodeframe3_range_ordinal')
        g = np.asarray(poses['0'])
        p, _, status = interpolate(df.time_s, g[:, 0], g[:, 1:])
        anchors = {}; surveys = {}
        for key in ['1', '2', '3', '4']:
            a = np.asarray(poses[key]); anchors[key] = a
            surveys[key] = {'median_m': np.median(a[:, 1:], axis=0),
                            'max_component_span_m': np.max(np.ptp(a[:, 1:], axis=0)), 'time': time_stats(a[:, 0])}
            mask = df.anchor_id == key
            ap, _, astat = interpolate(df.time_s[mask], a[:, 0], a[:, 1:])
            df.loc[mask, 'gt_range_m'] = geometric_range(p[mask], ap)
            df.loc[mask, 'association_status'] = np.where(status[mask] != 'AVAILABLE', status[mask], astat)
        self.geometry.append({'family': 'own_vicon', 'recording': path.stem, 'anchors_vicon': surveys,
                              'gt_time': time_stats(g[:, 0]), 'imu_time': time_stats(imu_times),
                              'bag_minus_header_s': {k: {'median': np.median(v), 'p95': np.quantile(v, .95), 'min': min(v), 'max': max(v)} for k,v in latency.items()}})
        self.finish('own_vicon', path.stem, 'nominal' if 'no_obstacle' in path.name else 'obstacle',
                    'USER_APPROXIMATE_COLOCATION', df,
                    'User-declared approximate common origin; moving Vicon tas_uwb_0 and same-bag tas_uwb_1..4; common header clock not independently calibrated.', True)

    def sfuise(self, path):
        self.source(path); rows = []
        with rosbag.Bag(str(path)) as bag:
            for _, m, _ in bag.read_messages(topics=['/rtls_flares']):
                for v in m.ranges: rows.append((m.header.stamp.to_sec(), str(m.id), str(v.id), v.range))
        u = pd.DataFrame(rows, columns=['t', 'tag', 'anchor', 'raw'])
        df = canonical(u.t, u.tag, u.anchor, u.raw, 'rtls_flares_range_ordinal')
        self.finish('SFUISE', path.stem, 'nominal_unlabelled', 'UNAVAILABLE_GT_TYPE_VIVE', df,
                    'Absolute ToA counted; Vive tracker is outside Vicon/GNSS-only scope; no reuse of historical identity-frame range diagnostic.')


@contextmanager
def readable_bag(path, out):
    try:
        bag = rosbag.Bag(str(path)); yield bag; bag.close()
    except rosbag.bag.ROSBagUnindexedException:
        with tempfile.TemporaryDirectory(prefix='obstacle_audit_reindex_') as td:
            copy = Path(td)/'recording.bag'; shutil.copy2(path, copy)
            result = subprocess.run(['rosbag', 'reindex', str(copy)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            (out/(path.stem+'_reindex.log')).write_text(result.stdout)
            dump(out/(path.stem+'_reindex.json'), {'command': ['rosbag','reindex',str(copy)], 'exit_code': result.returncode,
                                                  'original_path': str(path), 'original_modified': False})
            if result.returncode: raise RuntimeError('temporary-copy reindex failed')
            with rosbag.Bag(str(copy)) as bag: yield bag


def get_sources(audit):
    dest = audit.out/'sources'; dest.mkdir()
    ledger = []
    for name, rel in [('anchors.yaml', 'config/uwb/anchors.yaml'), ('tags.yaml', 'config/uwb/tags.yaml'),
                      ('experiments.csv', 'config/experiments.csv'), ('process_uwb.py', 'preprocess/process_uwb.py'),
                      ('cleanup_csv.py', 'preprocess/cleanup_csv.py'), ('miluv_README.md', 'README.md')]:
        url = 'https://raw.githubusercontent.com/decargroup/miluv/'+MILUV_COMMIT+'/'+rel
        r = requests.get(url, timeout=30); r.raise_for_status()
        target = dest/name; target.write_bytes(r.content)
        audit.source(target); ledger.append({'url': url, 'sha256': sha(target), 'path': str(target)})
    for rel in ['data/HUEC/README.md', 'data/HUEC/Technical_validation/localization_error_analysis.py',
                'data/HUEC/Technical_validation/ranging_error_analysis.py', 'data/starloc/README.md',
                'data/starloc/dataset_params.json', 'data/starloc/reader.py']:
        audit.source(ROOT/rel)
    dump(dest/'ledger.json', ledger)
    return dest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--families', nargs='+', default=['MILUV', 'own_vicon', 'HUEC_dynamic', 'starloc', 'SFUISE', 'HUEC_static'])
    args = parser.parse_args()
    out = PRIVATE/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True, exist_ok=False); print('OUTPUT='+str(out), flush=True)
    audit = Audit(out); audit.source(Path(__file__)); audit.source(ROOT/'experiments/scripts/evaluate_range_gt.py')
    audit.source(ROOT/'experiments/OBSTACLE_RANGE_AUDIT_PROTOCOL.md')
    for source in [Path(__file__), ROOT/'experiments/scripts/evaluate_range_gt.py', ROOT/'experiments/OBSTACLE_RANGE_AUDIT_PROTOCOL.md']:
        shutil.copy2(source, out/source.name)
    started = datetime.now(timezone.utc).isoformat(); failures = []
    sources = get_sources(audit)
    work = []
    if 'MILUV' in args.families:
        work += [('MILUV', p.name, lambda p=p: audit.miluv(p, sources)) for p in sorted((ROOT/'data/MILUV').iterdir()) if p.is_dir()]
    if 'own_vicon' in args.families:
        work += [('own_vicon', p.stem, lambda p=p: audit.own(p)) for p in sorted((ROOT/'data/own_vicon').glob('*.bag'))]
    if 'HUEC_dynamic' in args.families:
        work += [('HUEC_dynamic', str(p.parent), lambda p=p: audit.huec_dynamic(p.parent)) for p in sorted((ROOT/'data/HUEC/Dynamic_measurements').glob('**/combined.bag'))]
    if 'starloc' in args.families:
        params = json.loads((ROOT/'data/starloc/dataset_params.json').read_text())
        work += [('starloc', p['name'], lambda p=p: audit.starloc(ROOT/'data/starloc/data'/p['name'], p)) for p in params]
    if 'SFUISE' in args.families:
        work += [('SFUISE', p.stem, lambda p=p: audit.sfuise(p)) for p in sorted((ROOT/'data/SFUISE').glob('ISAS-Walk*.bag'))]
    if 'HUEC_static' in args.families:
        work += [('HUEC_static', str(p), lambda p=p: audit.huec_static(p)) for p in sorted((ROOT/'data/HUEC/Static_measurements').glob('*/*/*.csv'))]
    for family, name, fn in work:
        try: fn()
        except Exception as e:
            failure = {'family': family, 'recording': name, 'status': 'FAILED', 'error': str(e), 'traceback': traceback.format_exc()}
            failures.append(failure); print('FAILED '+family+'/'+name+': '+str(e), flush=True)
    changed = [p for p, s in audit.sources.items() if sha(p) != s['sha256']]
    pd.DataFrame(audit.links).to_csv(out/'link_summary.csv', index=False)
    pd.DataFrame(audit.anchors).to_csv(out/'anchor_summary.csv', index=False)
    pd.DataFrame(audit.recordings+failures).to_csv(out/'recording_summary.csv', index=False)
    dump(out/'recordings.json', audit.recordings+failures); dump(out/'geometry_audit.json', audit.geometry)
    dump(out/'source_hashes.json', audit.sources)
    dump(out/'execution.json', {'command': [sys.executable]+sys.argv, 'started_utc': started,
                              'ended_utc': datetime.now(timezone.utc).isoformat(), 'planned_recordings': len(work),
                              'completed_recordings': len(audit.recordings), 'failures': failures,
                              'input_changes': changed, 'figures': audit.figures, 'estimator': 'NOT_RUN',
                              'recovery': 'NOT_RUN', 'localization_benchmark': 'NOT_RUN',
                              'exit_code': 1 if failures or changed else 0,
                              'head': subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip()})
    print('OUTPUT='+str(out), flush=True)
    return 1 if failures or changed else 0


if __name__ == '__main__': sys.exit(main())
