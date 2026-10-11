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
    parser.add_argument('command', choices=['check', 'compare', 'check-hybrid', 'hybrid', 'lifecycle'])
    parser.add_argument('--build', action='store_true')
    parser.add_argument('--historical', action='store_true', help='execute existing C once per U0/U1, epochs14-20')
    parser.add_argument('--scene', choices=['U0','U1','U2','U3','I2','H1'], help='run just the selected fixed scene')
    args = parser.parse_args()
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    env['LD_LIBRARY_PATH'] = str(BIN.parent) + ':' + env.get('LD_LIBRARY_PATH', '')
    for name in list(env):
        if name.startswith('FDE_'):
            env.pop(name)
    targets = ['epoch_raim_checks', 'fused_epoch_checks', 'epoch_raim_runner']
    if args.command in ('hybrid','check-hybrid','lifecycle'):
        targets = ['hybrid_checks', 'epoch_raim_runner']
    if args.historical:
        targets.append('fde_next_runner')
    if args.build:
        subprocess.run(['cmake', '-S', str(ROOT), '-B', str(BUILD)], cwd=ROOT, env=env, check=True)
        subprocess.run(['cmake', '--build', str(BUILD), '--target', *targets, '-j2'], env=env, check=True)
    config = 'config/fde_next.yaml'
    if args.command in ('check-hybrid','lifecycle'):
        subprocess.run([str(BIN / 'hybrid_checks'), config], cwd=ROOT, env=env, check=True)
        return
    if args.command == 'hybrid':
        cache = ROOT / '.cache/hybrid'
        cache.mkdir(parents=True, exist_ok=True)
        scenarios = [args.scene] if args.scene else ['U0','U1','U2','U3','H1','I2']
        with tempfile.TemporaryDirectory(prefix='final-', dir=cache) as temp:
            for scenario in scenarios:
                variants = [''] if scenario != 'I2' else ['', 'imu-invalid']
                for evidence in variants:
                    count = 50 if scenario in ('U0','U1') else 21 if scenario=='U2' else 20
                    output = Path(temp) / (scenario + evidence + '.csv')
                    cmd = [str(BIN/'epoch_raim_runner'), config, str(output), scenario, 'hybrid', 'on', str(count)]
                    if evidence:
                        cmd.append(evidence)
                    subprocess.run(cmd, cwd=ROOT, env=env, check=True)
                    with output.open() as f:
                        rows = list(csv.DictReader(f))
                    assert len(rows)==count and all(r['updates']=='1' and r['formal_eligible']=='0' and r['publication_protected']=='0' and r['fused_conditional_available']=='0' and r['raw_total_risk']=='UNKNOWN' and r['full_C_calls']=='0' for r in rows)
                    assert all(r['evidence_level']=='RAW_UNQUALIFIED' for r in rows)
                    if scenario=='U1':
                        assert rows[14]['excluded']=='0' and rows[14]['correct']=='1' and rows[14]['candidates']=='9' and rows[14]['committed_retained']=='7' and '1501' not in rows[14]['retained_ids'].split('|')
                        assert all(r['updates']=='1' for r in rows[15:19])
                    if scenario=='H1':
                        assert rows[2]['excluded']=='-1' and rows[2]['alarm']=='0'
                        assert float(rows[3]['history_bound'])>0
                    if scenario=='U2':
                        assert sum(r['correct']=='1' for r in rows)==3
                        assert all(float(r['history_bound'])>0 for r in rows[17:21])
                    if scenario=='U3':
                        assert all(r['status']=='GEOMETRY_OR_MIRROR_UNQUALIFIED' for r in rows)
                    if evidence:
                        assert all(r['status']=='IMU_EVIDENCE_MISSING' and r['available']=='0' for r in rows)
                    def stats(key):
                        values=np.array([float(r[key]) for r in rows]);values=values[np.isfinite(values)]
                        return '/'.join(f'{x:.6f}' for x in [values.mean(),np.median(values),np.percentile(values,95),values.max()]) if values.size else 'UNKNOWN'
                    print(f'HYBRID {scenario} {evidence or "unlabelled"}: outputs={count} H_bounded={sum(r["history_status"]=="BOUNDED" for r in rows)} nonzero_H={sum(float(r["history_bound"])>0 for r in rows)} exclusions={sum(int(r["excluded"])>=0 for r in rows)} correct={sum(r["correct"]=="1" for r in rows)} AL_numeric={sum(r["within_al"]=="1" for r in rows)} max_bytes={max(int(r["history_bytes"]) for r in rows)} full_C_calls=0 status={dict(Counter(r["status"] for r in rows))}', flush=True)
                    print(f'  denominators B_finite={sum(r["numerical_status"]=="FINITE" for r in rows)}/{count} A_available={sum(r["a_available"]=="1" for r in rows)}/{count} A_alarms={sum(r["a_alarm"]=="1" for r in rows)}/{count} B_alarms={sum(int(r["dof"])>0 and float(r["statistic"])>float(r["threshold"]) for r in rows)}/{count} healthy_exclusions={sum(r["healthy_excluded"]=="1" for r in rows)}/{count} on_time={sum(r["on_time"]=="1" for r in rows)}/{count} qualified=0/{count}', flush=True)
                    for key in ['history_bound','history_eta','hpl','vpl','error','h_ms','prediction_ms','a_ms','b_ms','core_ms','verification_ms','commit_ms','h_update_ms','finalization_ms','cleanup_ms','full_ms']:
                        print(f'  {key} mean/median/P95/max={stats(key)}', flush=True)
        return
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
                  f'valid={sum(r[available]=="1" for r in rows)} alarm={sum(int(r["dof"])>0 and float(r["statistic"])>float(r["threshold"]) for r in rows)} '
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
