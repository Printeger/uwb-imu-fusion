#!/usr/bin/env python3
"""Fixed research A/B/C architecture comparison; transient artifacts auto-cleaned."""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import tempfile
from collections import Counter
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT.parents[1] / 'build/uwb_imu_pl'
BIN = ROOT.parents[1] / 'devel/.private/uwb_imu_pl/lib/uwb_imu_pl'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['check', 'compare'])
    parser.add_argument('--build', action='store_true')
    parser.add_argument('--historical', action='store_true', help='execute existing C once per U0/U1, epochs14-20')
    parser.add_argument('--scene', choices=['U0','U1','U2','U3','I2'], help='run just the selected fixed scene')
    args = parser.parse_args()
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    env['LD_LIBRARY_PATH'] = str(BIN.parent) + ':' + env.get('LD_LIBRARY_PATH', '')
    for name in list(env):
        if name.startswith('FDE_'):
            env.pop(name)
    targets = ['epoch_raim_checks', 'fused_epoch_checks', 'epoch_raim_runner']
    if args.historical:
        targets.append('fde_next_runner')
    if args.build:
        subprocess.run(['cmake', '-S', str(ROOT), '-B', str(BUILD)], cwd=ROOT, env=env, check=True)
        subprocess.run(['cmake', '--build', str(BUILD), '--target', *targets, '-j2'], env=env, check=True)
    config = 'config/fde_next.yaml'
    if args.command == 'check':
        for target in targets[:2]:
            subprocess.run([str(BIN / target), config], cwd=ROOT, env=env, check=True)
        return
    with tempfile.TemporaryDirectory(prefix='epoch-architecture-') as temp:
        tmp = Path(temp)
        def run(name, cmd, custom_env=env):
            result = subprocess.run(cmd, cwd=ROOT, env=custom_env, capture_output=True, text=True)
            (tmp / (name + '.log')).write_text(result.stdout + result.stderr)
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
            print(result.stdout.strip(), flush=True)
            with (tmp / (name + '.csv')).open() as stream:
                return list(csv.DictReader(stream))
        def summary(name, rows, historical=False):
            def values(key):
                return np.array([float(r[key]) for r in rows])
            def stats(key):
                x = values(key)
                x = x[np.isfinite(x)]
                return '/'.join(f'{v:.6f}' for v in [np.nanmean(x), np.nanmedian(x), np.nanpercentile(x, 95), np.nanmax(x)]) if x.size else 'UNAVAILABLE'
            def rmse(key):
                x = values(key)
                x = x[np.isfinite(x)]
                return np.sqrt(np.mean(x*x)) if x.size else float('nan')
            available = 'selected' if historical else 'available'
            timing = 'end_to_end_ms' if historical else 'full_ms'
            print(f'{name}: n={len(rows)} commits={sum(int(r["updates"]) for r in rows)} '
                  f'valid={sum(r[available]=="1" for r in rows)} alarm={sum(r["alarm"]=="1" for r in rows)} '
                  f'correct={sum(r["correct"]=="1" for r in rows)} '
                  f'APE_RMSE={rmse("error"):.6f} '
                  f'APE_mean/median/P95/max={stats("error")} '
                  f'HPL={stats("hpl")} VPL={stats("vpl")} '
                  f'core_ms={stats("core_ms")} full_ms={stats(timing)} '
                  f'full>50={sum(values(timing)>50)} statuses={dict(Counter(r["status"] for r in rows))}', flush=True)
            if not historical:
                print(f'{name}: increment_RMSE={rmse("increment_error"):.6f} '
                      f'translation_RPE_RMSE={rmse("translation_rpe"):.6f} '
                      f'rotation_RPE_RMSE={rmse("rotation_rpe"):.6f} '
                      f'healthy_excluded={sum(r["healthy_excluded"]=="1" for r in rows)} '
                      f'within_AL={sum(r["within_al"]=="1" for r in rows)} '
                      f'prediction_ms={stats("prediction_ms")} center_shift={stats("center_shift")} '
                      f'rows={min(values("rows"))}:{max(values("rows"))} cols={set(r["cols"] for r in rows)} '
                      f'candidates={min(values("candidates"))}:{max(values("candidates"))}', flush=True)
                for r in rows:
                    if r['alarm'] == '1':
                        print(name, 'ALARM', {k:r[k] for k in ['epoch','excluded_anchor','excluded_measurement','retained','statistic','threshold','post_statistic','dof','hpl','vpl','core_ms','full_ms']})
        fused_common = {}
        for algorithm in ['uwb_epoch_raim', 'fused_epoch_pl']:
            for scene, fde, count, extra in [('U0','on',50,[]), ('U1','on',50,[]), ('U2','on',21,[]),
                                             ('U3','on',20,[]), ('I2','on',20,[]), ('U1','off',50,[]),
                                             ('U0','on',20,['history-invalid'])]:
                if args.scene and (scene != args.scene or fde != 'on' or extra):
                    continue
                name = f'{algorithm}-{scene}-{fde}' + ('-history-invalid' if extra else '')
                rows = run(name, [str(BIN/'epoch_raim_runner'), config, str(tmp/(name+'.csv')), scene, algorithm, fde, str(count), *extra])
                if len(rows) != count or any(r['updates']!='1' or r['formal_eligible']!='0' or r['publication_protected']!='0' for r in rows):
                    raise RuntimeError('commit count or research-only qualification regression')
                if scene in ['U1','U2'] and fde=='on':
                    expected = 1 if scene=='U1' else 3
                    if sum(r['correct']=='1' for r in rows)!=expected or (scene=='U2' and (len(rows[17:21])!=4 or any(r['available']!='1' for r in rows[17:21]))):
                        raise RuntimeError('recovery/four-followup regression')
                if scene=='U3' and any(r['available']=='1' for r in rows):
                    raise RuntimeError('mirror branch must refuse')
                if algorithm=='fused_epoch_pl' and (scene=='I2' or extra) and any(r['available']=='1' for r in rows):
                    raise RuntimeError('known contaminated prediction must refuse')
                summary(name, rows)
                if scene=='U1' and fde=='on':
                    if any(r['available']!='1' for r in rows[15:19]):
                        raise RuntimeError('four post-U1 epochs must remain valid')
                if scene in ['U0','U1'] and fde=='on' and not extra:
                    summary(name+'-common14-20', rows[13:20])
                    if algorithm=='fused_epoch_pl':
                        fused_common[scene] = rows[13:20]
            if algorithm=='fused_epoch_pl' and not args.scene:
                # U2 OFF contaminates history; separately provided invalid-history
                # evidence demonstrates refusal, independent of fault selection.
                name='fused_epoch_pl-U2-off-history-invalid'
                rows=run(name,[str(BIN/'epoch_raim_runner'),config,str(tmp/(name+'.csv')),'U2',algorithm,'off','20','history-invalid'])
                if any(r['available']=='1' for r in rows):
                    raise RuntimeError('persistent carry-in must refuse')
                summary(name,rows)
        if args.historical:
            for scene in ['U0','U1']:
                if args.scene and args.scene!=scene:
                    continue
                name='historical_reference-'+scene
                current = dict(env, FDE_NEXT_ONLINE_SUMMARY='1')
                rows=run(name,[str(BIN/'fde_next_runner'),config,str(tmp/(name+'.csv')),scene,'uwb','20261010','20','optimized','1','continuous'],current)
                summary(name,rows,True)
                for left, right in zip(fused_common[scene], rows):
                    if left['timestamp_ns']!=right['timestamp_ns'] or not np.allclose([float(left['tag_'+k]) for k in ['x','y','z']], [float(right['commit_'+k]) for k in ['x','y','z']], rtol=0, atol=1e-9):
                        raise RuntimeError('B/C native committed tag mismatch')
                print(name, 'B/C native means/time equality PASS; PL scopes/covariances differ')
                print(name,'DIMENSIONS', [{k:r[k] for k in ['epoch','rows','cols','hypotheses','reference_faults','detector_dof','summary_bytes']} for r in [rows[0],rows[-1]]])


if __name__ == '__main__':
    main()
