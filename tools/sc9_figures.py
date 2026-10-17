#!/usr/bin/env python3
"""Redraw the three SC9 figures from frozen traces; never runs an estimator."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / '.cache/sc9'
OUT = ROOT / 'docs/sc9'

def data(name):
    return json.loads((CACHE / name).read_text())

def rows(name):
    with (CACHE / name).open() as f:
        return list(csv.DictReader(f))

def injected_span(ax, trace, case, relative=False):
    walk, fault = case.split('-')
    registration = data('injection-registration.json')['ISAS-' + walk]
    if fault in ('U', 'J'):
        batches = {v['source_message'] + 1 for v in registration['range_source']}
        epochs = [int(r['epoch']) for r in trace if int(r['batch_id']) in batches]
        for epoch in epochs:
            ax.axvspan(epoch - .18, epoch + .18, color='#ccaa33', alpha=.18)
    if fault in ('A', 'G', 'J'):
        if relative:
            times = [int(r['stamp_ns']) / 1e9 for r in trace]
        else:
            meta = data('wire/canonical-v1/' + walk + '-base/manifest.json')
            times = [(int(r['stamp_ns']) - meta['first_imu_stamp_ns']) / 1e9 + meta['legacy_first_imu_time_s'] for r in trace]
        limits = np.interp([registration['imu_begin_s'], registration['imu_end_s']], times, [int(r['epoch']) for r in trace])
        ax.axvspan(*limits, color='#bb5533', alpha=.10)

def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / name, dpi=160)
    plt.close(fig)

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False})
    scores = data('walk/canonical-final-v2/scores.json')['runs']
    cases = ['Walk2-base', 'Walk3-base', 'Walk2-U', 'Walk2-A', 'Walk2-G', 'Walk2-J', 'Walk3-J']
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    x = np.arange(len(cases))
    for ax, metric, label in zip(axes, ['ape_m', 'are_deg'], ['Tag APE RMSE (m)', 'Attitude RMSE (deg)']):
        for dx, mode, color in [(-.18, 'off', '#777777'), (.18, 'on', '#167d9a')]:
            ax.bar(x + dx, [scores[c + '-' + mode][metric]['rmse'] for c in cases], .36, label=mode.upper(), color=color)
        ax.set_xticks(x, cases, rotation=35, ha='right'); ax.set_ylabel(label); ax.legend()
    fig.suptitle('Fixed Walk1 CAL reference; full recordings, no TEST alignment\nPosition benefits coexist with A/G attitude degradation; hardware risk unqualified', fontsize=11)
    save(fig, 'accuracy.png')

    fig, axes = plt.subplots(6, 1, figsize=(11, 9), sharex=False)
    columns = [('imu_alarm', 'IMU alarm', '#bb5533'), ('uwb_alarm', 'UWB alarm', '#ccaa33'),
               ('excluded_source', 'UWB exclusion', '#573c8f'), ('bridge', 'Bridge', '#167d9a'),
               ('repair', 'Prior repair', '#222222'), ('rejoined', 'Rejoin', '#49944b')]
    for ax, case in zip(axes[:5], ['Walk2-U', 'Walk2-A', 'Walk2-G', 'Walk2-J', 'Walk3-J']):
        trace = rows('walk/canonical-final-v2/' + case + '-on.csv')
        injected_span(ax, trace, case)
        for y, (key, label, color) in enumerate(columns):
            times = [int(r['epoch']) for r in trace if (bool(r[key]) if key == 'excluded_source' else r[key] == '1')]
            ax.scatter(times, [y] * len(times), marker='|', s=45, color=color)
        ax.set_yticks(range(6), [c[1] for c in columns]); ax.set_title(case, loc='left'); ax.set_xlim(270, 335)
    trace = rows('shared-range-real-held.csv')
    injected_span(axes[5], trace, 'Walk2-J', relative=True)
    for y, (key, label, color) in enumerate(columns):
        times = [int(r['epoch']) for r in trace if (bool(r[key]) if key == 'excluded_source' else r[key] == '1')]
        axes[5].scatter(times, [y] * len(times), marker='|', s=50, color=color)
    axes[5].set_yticks(range(6), [c[1] for c in columns]); axes[5].set_xlim(270, 340)
    axes[5].set_title('Walk2 J, held actions: polluted prior repaired at 313; rejoin 314; 26 further commits', loc='left')
    axes[5].set_xlabel('Logical epoch; shaded bands = registered UWB samples (gold) / IMU injection (red)')
    save(fig, 'recovery.png')

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    trace = rows('simulation/nondegenerate-long-final-v2/L-repeated.csv')
    for ax, error, bound, label in zip(axes[0, :2], ['herror', 'verror'], ['hpl', 'vpl'], ['Horizontal', 'Vertical']):
        e = np.array([float(r[error]) for r in trace]); b = np.array([float(r[bound]) for r in trace])
        ax.scatter(e, b, s=3, alpha=.12, rasterized=True)
        top = max(e.max(), b.max()); ax.plot([0, top], [0, top], '--', color='#bb5533')
        ax.set_xlabel(label + ' actual tag error (m)'); ax.set_ylabel(label + ' conditional bound (m)')
        ax.set_title('Declared simulation model: 10000 / 10000 contained')
    ax = axes[0, 2]
    names = ['Walk2-base', 'Walk2-J', 'Walk3-base', 'Walk3-J']
    summaries = [data('ros/final-v2-' + n + '/summary.json') for n in names]
    for dx, key, label in [(-.18, 'core_ms', 'Core P95'), (.18, 'measurement_to_sink_ms', 'Original measurement to sink P95')]:
        ax.bar(np.arange(4) + dx, [s[key]['p95'] for s in summaries], .36, label=label)
    ax.axhline(40, color='orange', ls='--'); ax.axhline(50, color='red', ls='--')
    ax.set_xticks(range(4), names, rotation=30); ax.set_ylabel('ms, all results'); ax.legend(fontsize=7)
    sim = data('simulation/nondegenerate-paced-final-v1/summary.json')
    ax = axes[1, 0]; names = list(sim)
    ax.bar(range(len(names)), [100 * sim[n]['delivery']['timely_spatial'] / sim[n]['planned'] for n in names])
    ax.axhline(95, color='red', ls='--'); ax.set_ylim(90, 100.5)
    ax.set_xticks(range(len(names)), names, rotation=60, ha='right'); ax.set_ylabel('Actual timely + spatial (%)')
    ax.set_title('Independent paced virtual sensors, all planned rows')
    for label, filename in [('Healthy', 'L-healthy'), ('Repeated', 'L-repeated')]:
        r = rows('simulation/nondegenerate-long-final-v2/' + filename + '.csv')
        x = [int(v['epoch']) for v in r]
        axes[1, 1].plot(x, [int(v['rss']) / 1024 for v in r], label=label)
        axes[1, 2].plot(x, [int(v['prefix']) for v in r], label=label)
    axes[1, 1].set_ylabel('Whole-process RSS (MiB)'); axes[1, 1].set_title('Includes preloaded schedule; finite-scope storage')
    axes[1, 2].set_ylabel('Registered source coordinates'); axes[1, 2].set_title('Risk never reset; O(T) identity directory')
    for ax in axes[1, 1:]: ax.set_xlabel('Epoch'); ax.legend()
    save(fig, 'bounds_delivery_resources.png')

if __name__ == '__main__':
    main()
