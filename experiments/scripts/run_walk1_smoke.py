#!/usr/bin/env python3
"""One locked ISAS Walk1 clean batch; orchestration only, no estimator math."""
import argparse
import csv
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/paper'))
METHODS = {'all_range': 'M0', 'robust_cauchy': 'M1',
           'suppress_all': 'M3', 'lcb_fixed_full': 'M4'}
CONFIG = ROOT / 'config/paper/ie0911/sfuise_walk1_pl_bidirectional_clean.yaml'
RUNNER = ROOT.parents[1] / 'devel/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner'
GT = ROOT.parents[1] / 'res/ie0911_step2_truth_20260911_01/sfuise_walk1/ground_truth.tum'
PROTOCOL = {'time_association': {'policy': 'nearest_within_tolerance', 'tolerance_s': 0.02},
            'rpe_horizon_s': 1.0, 'evaluation_interval_s': [1664959676.9893188,1664959736.082964],
            'alignment_mode': 'SE3_SCALE_1_PER_TRAJECTORY',
            'gt_reference_point': 'vive/tracker_1 origin; tracker-to-IMU extrinsics UNKNOWN',
            'metric_scope': 'DEVELOPMENT_TRACKER_PROXY_NOT_CALIBRATED_BODY_ERROR'}


def now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''):
            h.update(b)
    return h.hexdigest()


def digest(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, allow_nan=False).encode()).hexdigest()


def write(path, obj):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(obj, indent=2, sort_keys=True, allow_nan=False)+'\n')
    temp.replace(path)


def read(path):
    return json.loads(Path(path).read_text()) if Path(path).is_file() else {}


def worker(batch, output, runner):
    import run_experiments as scheduler
    from estimator_isolation import runner_call as original

    def logged(*args):
        binary, config, runroot, rid, method, execution, cache, point, anchors, env = args
        identity = {'effective_yaml': yaml.safe_load(config.read_text()),
                    'method': method or 'structured_debias', 'execution_type': execution,
                    'operating_point_id': point, 'anchor_ids': anchors,
                    'environment_overrides': env,
                    'parent_cache_sha256': sha(cache) if cache else None}
        doc = {'run_id': rid, 'start_time': now(), 'complete_config_hash': digest(identity),
               'configuration': identity, 'config_file_sha256': sha(config), 'end_time': None}
        path = output / (rid + '.invocation.json')
        write(path, doc)
        start = time.monotonic()
        try:
            result = original(*args)
            doc['exit_code'] = result[0]
            return result
        finally:
            doc.update(end_time=now(), wall_time_s=time.monotonic()-start)
            write(path, doc)
    scheduler.runner_call = logged
    sys.argv = ['run_experiments.py', '--manifest', str(batch), '--output-root', str(output),
                '--runner', str(runner)]
    return scheduler.main()


def save_csv(path, rows):
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader(); w.writerows(rows)


def classify(status, cell_status):
    s = str(status.get('status', ''))
    if 'FALLBACK' in s and status.get('valid_estimate_exported', False):
        return 'fallback'
    if status.get('exit_code') == 0 and (s in ('OK', 'NO_CANDIDATES', 'CONVERGED') or
                                        status.get('valid_estimate_exported', False)):
        return 'success'
    return 'failure'


def aggregate(out, batch_start, batch_end, code):
    import evaluate_runs as ev
    batch = read(out/'batch/batch_manifest.json')
    cells = batch.get('cells', [])
    rows, metrics = [], []
    expected = [(k, 'BASELINE_TRAJECTORY' if k in ('all_range','robust_cauchy') else 'FINAL_TRAJECTORY')
                for k in METHODS] + [('structured_debias','CACHE_PRODUCER')]
    lock = read(out/'lock.json')
    lo, hi = PROTOCOL['evaluation_interval_s']
    for mode, execution in expected:
        cell = next((c for c in cells if c['canonical_mode']==mode and c['execution_type']==execution), {})
        rid = cell.get('run_id', 'not-run-'+uuid.uuid4().hex)
        run = Path(cell['run_directory']) if cell.get('run_directory') else None
        invocation = read(out/'batch'/f'{rid}.invocation.json')
        status = read(run/'run_status.json') if run else {}
        outcome = classify(status, cell.get('status'))
        trajectory = run/'trajectory.tum' if run else None
        if outcome != 'failure' and not trajectory.is_file():
            outcome = 'failure'; status['reason'] = 'FINAL_TRAJECTORY_MISSING'
        row = dict(run_id=rid, git_commit=lock['git_commit'], dataset_family='ISAS', recording_id='ISAS-Walk1',
                   method=METHODS.get(mode, 'AUX_STAGE2_PRODUCER'), canonical_method=mode,
                   complete_config_hash=invocation.get('complete_config_hash', ''),
                   detector_provider='DISABLED' if mode in ('all_range','robust_cauchy') else 'PL_BIDIRECTIONAL_CUSUM_V1',
                   recovery_policy=mode if mode in ('suppress_all','lcb_fixed_full') else 'NONE',
                   start_time=invocation.get('start_time', batch_start), end_time=invocation.get('end_time', batch_end),
                   status=outcome, failure_reason='' if outcome=='success' else
                   status.get('reason', status.get('failure_reason', cell.get('reason', read(out/'infrastructure_failure.json').get('reason', f'BATCH_EXIT_{code}')))),
                   backend_status=status.get('status', cell.get('status','NOT_RUN')),
                   wall_time_s=invocation.get('wall_time_s',0),
                   output_trajectory_path=str(trajectory) if trajectory and outcome!='failure' else '',
                   parent_cache_id=cell.get('cache_id',''), execution_type=execution,
                   evaluator_protocol_hash=lock['evaluator_protocol_hash'])
        # Planned requests without backend execution still have a reproducible request identity.
        if not row['complete_config_hash']:
            row['complete_config_hash'] = digest({'locked_config_sha256': lock['config_sha256'],
                                                  'method':mode,'execution':execution,'executed':False})
        rows.append(row)
        if mode not in METHODS:
            continue
        m = dict(run_id=rid, method=METHODS[mode], ATE_RMSE_m=None, ATE_median_m=None, ATE_P95_m=None,
                 translation_RPE_1s_RMSE_m=None, evaluated_GT_samples=0,
                 evaluation_start_s=lo, evaluation_end_s=hi,
                 actual_matched_start_s=None, actual_matched_end_s=None,
                 alignment_mode=PROTOCOL['alignment_mode'], gt_reference_point=PROTOCOL['gt_reference_point'],
                 evaluation_status='UNAVAILABLE', failure_reason=row['failure_reason'],
                 metric_scope=PROTOCOL['metric_scope'], matched_GT_identity_hash='', rpe_status='NOT_RUN')
        if outcome!='failure':
            try:
                est=ev.load_tum(trajectory)
                clipped=[e for e in est if lo<=e[0]<=hi]
                evaluation=ROOT.parents[1]/'evaluator_private/icra/trajectory'/out.name/rid
                evaluation.mkdir(parents=True)
                with (evaluation/'trajectory.tum').open('w') as f:
                    for t,p,q in clipped:
                        f.write(' '.join(format(float(v),'.17g') for v in [t,*p,*q])+'\n')
                scenario={**PROTOCOL, 'ground_truth': str(GT)}
                result=ev.trajectory_metrics(evaluation,scenario)
                write(evaluation/'metrics.json',result)
                matches,_=ev.match_trajectories(clipped,ev.load_tum(GT),PROTOCOL['time_association'])
                timestamps=[g[0] for _,g in matches]
                write(evaluation/'matched_gt_timestamps.json',timestamps)
                for dst,src in [('ATE_RMSE_m','aligned_ATE_rmse_m'),('ATE_median_m','aligned_ATE_p50_m'),
                                ('ATE_P95_m','aligned_ATE_p95_m'),('translation_RPE_1s_RMSE_m','RPE_rmse_m')]:
                    m[dst]=result.get(src,{}).get('value')
                interval=result.get('evaluation_interval_s',{}).get('value') or [None,None]
                m.update(evaluated_GT_samples=len(set(timestamps)),actual_matched_start_s=interval[0],actual_matched_end_s=interval[1],
                         matched_GT_identity_hash=digest(timestamps), rpe_status=result.get('RPE_rmse_m',{}).get('status'),
                         evaluation_status='AVAILABLE' if m['ATE_RMSE_m'] is not None else 'UNAVAILABLE',
                         failure_reason=result.get('aligned_ATE_rmse_m',{}).get('reason',''))
            except Exception as e:
                m['failure_reason']=f'EVALUATOR_ERROR: {e}'
        metrics.append(m)
    available=[m for m in metrics if m['evaluation_status']=='AVAILABLE']
    comparable=len(available)==4 and len({m['matched_GT_identity_hash'] for m in available})==1
    for m in metrics:
        m['comparison_status']='IDENTICAL_GT_SET' if comparable else 'INCOMPLETE_OR_MISMATCHED_GT_SET'
    save_csv(out/'runs.csv',rows);save_csv(out/'trajectory_metrics.csv',metrics)
    verdict='SMOKE_PASS' if comparable and all(r['status']!='failure' for r in rows) else 'SMOKE_FAIL_OR_INCOMPLETE'
    write(out/'summary.json',{'verdict':verdict,'scheduler_exit_code':code,'runs':rows,'metrics':metrics,
                              'estimator_math_modified':False})
    print(json.dumps({'verdict':verdict,'output':str(out),'methods':[(m['method'],m['ATE_RMSE_m']) for m in metrics]}))
    return 0 if verdict=='SMOKE_PASS' else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,default=ROOT/'experiments/results')
    parser.add_argument('--worker',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--runner',type=Path,default=RUNNER)
    a=parser.parse_args()
    if a.worker:
        return worker(a.worker/'batch.yaml',a.worker/'batch',a.runner)
    out=a.output_root.resolve()/('walk1-clean-'+dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:12])
    out.mkdir(parents=True,exist_ok=False)
    try:
        return run_batch(out, a)
    except Exception as exc:
        message=f'INFRASTRUCTURE_ERROR: {type(exc).__name__}: {exc}'
        write(out/'infrastructure_failure.json', {'reason':message,'time':now()})
        if not (out/'lock.json').exists():
            write(out/'lock.json', {'git_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                                   'config_sha256':sha(CONFIG),'evaluator_protocol_hash':digest(PROTOCOL)})
        print(message, file=sys.stderr)
        return aggregate(out,now(),now(),2)


def run_batch(out, a, config=CONFIG, run_unit_id="isas_walk1_clean"):
    cfg=yaml.safe_load(config.read_text())
    cache=Path(cfg['dataset']['cache_manifest']); doc=read(cache)
    for field in ('imu','uwb'):
        if sha(cache.parent/doc[field+'_file'])!=doc[field+'_sha256'].split(':')[-1]:
            raise ValueError('clean input payload identity mismatch')
    if sha(cache)!='7af404c8580ecf0b9f9d2742bbc56d03ed9a592172603d3df0e8a0d569e22c0e':
        from canonical_injection import validate_child
        validate_child(cache)
    if sha(GT)!='2bd90465cc6f9445e94c75087a293c36cfc25a4ac1165654a9765431c8816c33':
        raise ValueError('GT identity mismatch')
    # Explicit thresholds preserve the source values; the scheduler otherwise fills absent values with zero.
    cells=[{'mode':'all_range','execution_type':'BASELINE_TRAJECTORY','path':'DIRECT_COMMON_PREPARATION'},
           {'mode':'robust_cauchy','robust_scale':2.3849,'execution_type':'BASELINE_TRAJECTORY','path':'DIRECT_COMMON_PREPARATION'},
           {'mode':'structured_debias','execution_type':'CACHE_PRODUCER','path':'PL_BIDIRECTIONAL_CUSUM','producer_id':'shared'}]
    for mode in ('suppress_all','lcb_fixed_full'):
        cells.append({'mode':mode,'execution_type':'FINAL_TRAJECTORY','path':'PL_BIDIRECTIONAL_CUSUM','producer_id':'shared',
                      'thresholds':{k:cfg['nlos'][k] for k in ('tau_eta','tau_s_m','tau_gamma')}})
    batch={'schema':'uifgo_t09_batch_v2','role':'development','parameter_provenance':'IE0912_PL_BIDIRECTIONAL_LOCKED_PENDING_VALIDATION',
           'run_units':[{'run_unit_id':run_unit_id,'recording_id':'ISAS-Walk1','base_trajectory_id':'ISAS-Walk1',
                         'seed':911,'prefix_identity':'full_original_crop','config':str(config),'cells':cells}]}
    (out/'batch.yaml').write_text(yaml.safe_dump(batch,sort_keys=False))
    write(out/'lock.json',{'git_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                           'tracked_diff_hash':hashlib.sha256(subprocess.check_output(['git','diff','HEAD'],cwd=ROOT)).hexdigest(),
                           'config_sha256':sha(config),'runner_sha256':sha(a.runner),'input_manifest_sha256':sha(cache),
                           'evaluator_sha256':sha(ROOT/'tools/paper/evaluate_runs.py'),'evaluator_protocol':PROTOCOL,
                           'evaluator_protocol_hash':digest(PROTOCOL),'harness_sha256':sha(__file__),'gt_sha256':sha(GT)})
    start=now()
    env=os.environ.copy()
    for k in list(env):
        if k.startswith('UIFGO_'): del env[k]
    env['UIFGO_EXPERIMENT_WALL_LIMIT_S']='1800'
    command=[sys.executable,str(Path(__file__).resolve()),'--worker',str(out),'--runner',str(a.runner)]
    write(out/'command.json',{'argv':command,'cwd':str(ROOT),'start_time':start})
    with (out/'scheduler.log').open('w') as log:
        code=subprocess.call(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
    end=now()
    write(out/'command.json',{'argv':command,'cwd':str(ROOT),'start_time':start,'end_time':end,'exit_code':code})
    return aggregate(out,start,end,code)


if __name__=='__main__':
    raise SystemExit(main())
