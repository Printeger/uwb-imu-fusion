#!/usr/bin/env python3
"""Frozen R3–R6 orchestration of the existing native estimator, with mount isolation."""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml
from fingerprint import HERE, ROOT, WS, RUNNER, sha, digest
from evaluate_clean import read, rows, save

sys.path.insert(0, str(ROOT / 'experiments/scripts'))
sys.path.insert(0, str(ROOT / 'tools/paper'))
FROZEN = HERE / 'configs/formal_lock.json'
METHODS = {'all_range': 'FGO_BASE', 'robust_cauchy': 'FGO_ROBUST',
           'suppress_all': 'FGO_REJECT', 'lcb_fixed_full': 'FGO_RECOVER_FULL',
           'structured_debias': 'AUX_PRODUCER'}


def write(p, data):
    Path(p).parent.mkdir(parents=True, exist_ok=True)
    Path(p).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def freeze():
    if FROZEN.exists():
        raise ValueError('formal freeze already exists')
    version = yaml.safe_load((HERE / 'VERSION.yaml').read_text())
    protected = {str(ROOT / k): v for k, v in version['source_files'].items()}
    protected.update(version['linked_libraries'])
    protected[str(RUNNER)] = version['runner_sha256']
    protected[str(ROOT / 'tools/run_ie_paper.cpp')] = sha(ROOT / 'tools/run_ie_paper.cpp')
    for p in (HERE / 'metrics').glob('*.csv'):
        protected[str(p)] = sha(p)
    for p in [HERE/'evaluate_clean.py', ROOT/'tools/paper/evaluate_runs.py', HERE/'audits/frame_fits_evaluator_only.json']:
        protected[str(p)] = sha(p)
    for inp in version['inputs']:
        protected.update(inp['files'])
    for p, h in protected.items():
        if sha(p) != h: raise ValueError('pre-freeze identity mismatch: '+p)
    starting = yaml.safe_load((HERE/'configs/detector/starting_pipeline.yaml').read_text())
    assert starting['calibration']['fixed_beta_by_link'] == {}
    backbone = copy.deepcopy(starting)
    detector = {k: v for k,v in starting['nlos'].items() if k.startswith('cusum') or k in ('mode','gap_threshold_s')}
    recovery = {k: v for k,v in starting['nlos'].items() if k not in detector}
    evaluator = {'schema':'R1_R2_FROZEN_COMMON_GT_V1', 'common_gt_sets':read(HERE/'audits/frame_fits_evaluator_only.json'),
                 'association':'nearest_within_tolerance', 'tolerance_s':.02,
                 'alignment':'SE3_SCALE_1_PER_TRAJECTORY_COMMON_GT', 'reference_point':'TRACKER_BODY_COLOCATION_PROXY',
                 'raw_frame_ATE':'UNAVAILABLE_FRAME_AND_POINT_PROVENANCE', 'rpe_horizon_s':1.,
                 'clean_equivalence_position_m':.001,'clean_equivalence_rotation_rad':.001,'clean_equivalence_ate_m':.001,
                 'RR_definition':'recover_minus_reject',
                 'stable_improvement_rule':{'min_valid_pairs':12,'min_strict_ATE_improvements_of_15':10,
                                            'min_strict_fault_improvements_of_15':10,'both_medians_strictly_negative':True},
                 'figure_primary':['Walk1',10548],'figure_fallback':['Walk2',10548]}
    configs = {'backbone/FROZEN_BACKBONE.yaml':backbone,'detector/FROZEN_DETECTOR.yaml':detector,
               'methods/FROZEN_RECOVERY.yaml':dict(recovery,primary='lcb_fixed_full',reject='suppress_all',lcb_kappa=2),
               'FROZEN_EVALUATOR.yaml':evaluator}
    for rel,doc in configs.items():
        p=HERE/'configs'/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(yaml.safe_dump(doc,sort_keys=True))
    hashes={str(HERE/'configs'/p):sha(HERE/'configs'/p) for p in configs}
    for n in (1,2,3):
        p=HERE/f'configs/sfuse/config_test_isas-walk{n}.yaml';hashes[str(p)]=sha(p)
        gt=WS/f'res/ie0911_step2_truth_20260911_01/sfuise_walk{n}/ground_truth.tum';protected[str(gt)]=sha(gt)
    doc={'schema':'ICRA_R3_R6_FORMAL_FROZEN_V1','git_hash':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
         'parent_fingerprint':version['fingerprint'],'config_hashes':hashes,'protected':protected,'inputs':version['inputs'],
         'backbone':'B0_CURRENT','data_role':'PREREGISTERED_CONTROLLED_BENCHMARK_ON_EXPOSED_DEVELOPMENT_RECORDINGS',
         'stable_improvement_rule':evaluator['stable_improvement_rule']}
    doc['experiment_fingerprint']=digest(doc);write(FROZEN,doc)
    return doc


def verify():
    lock=read(FROZEN);original=lock.copy();fp=original.pop('experiment_fingerprint')
    if digest(original)!=fp:raise ValueError('formal fingerprint mismatch')
    for p,h in {**lock['protected'],**lock['config_hashes']}.items():
        if sha(p)!=h:raise ValueError('frozen input/config/source changed: '+p)
    return lock


def worker(out):
    import run_experiments as scheduler
    import estimator_isolation as isolation
    allowed=read(out/'measurement_allowlist.json')
    def files(config):
        cfg=yaml.safe_load(Path(config).read_text())
        if cfg['dataset']['interface']!='t07_cache':raise ValueError('cache only')
        expected=allowed[cfg['dataset']['cache_manifest']]
        for p,h in expected.items():
            if sha(p)!=h:raise ValueError('input hash mismatch')
        return [Path(config)]+[Path(p) for p in expected]
    isolation.measurement_files=files
    scheduler.runner_call=isolation.runner_call
    sys.argv=['run_experiments.py','--manifest',str(out/'batch.yaml'),'--output-root',str(out/'batch'),'--runner',str(RUNNER)]
    return scheduler.main()


def run_native(case_key, sequence, manifest, stage, baselines=True):
    lock=verify();out=HERE/'runs'/stage/case_key
    if out.exists():raise ValueError('refuse duplicate scientific native run '+str(out))
    out.mkdir(parents=True)
    cfg=yaml.safe_load((HERE/'configs/backbone/FROZEN_BACKBONE.yaml').read_text())
    cfg['dataset']['cache_manifest']=str(manifest)
    config=out/'config.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=True))
    inputs=read(manifest);files={str(manifest):sha(manifest)}
    for kind in ('uwb','imu'):
        p=manifest.parent/inputs[kind+'_file']
        assert 'sha256:'+sha(p)==inputs[kind+'_sha256'];files[str(p)]=sha(p)
    write(out/'measurement_allowlist.json',{str(manifest):files})
    cells=[]
    if baselines:
        cells=[{'mode':'all_range','execution_type':'BASELINE_TRAJECTORY','path':'DIRECT_COMMON_PREPARATION'},
               {'mode':'robust_cauchy','robust_scale':2.3849,'execution_type':'BASELINE_TRAJECTORY','path':'DIRECT_COMMON_PREPARATION'}]
    cells.append({'mode':'structured_debias','execution_type':'CACHE_PRODUCER','path':'PL_BIDIRECTIONAL_CUSUM','producer_id':'shared'})
    for method in ('suppress_all','lcb_fixed_full'):
        cells.append({'mode':method,'execution_type':'FINAL_TRAJECTORY','path':'PL_BIDIRECTIONAL_CUSUM','producer_id':'shared',
                      'thresholds':{k:cfg['nlos'][k] for k in ('tau_eta','tau_s_m','tau_gamma')}})
    batch={'schema':'uifgo_t09_batch_v2','role':'development','parameter_provenance':'IE0912_PL_BIDIRECTIONAL_LOCKED_PENDING_VALIDATION',
           'run_units':[{'run_unit_id':case_key,'recording_id':inputs['base_recording_id'],
                         'base_trajectory_id':inputs['base_recording_id'],'seed':911,'prefix_identity':'full',
                         'config':str(config),'cells':cells}]}
    (out/'batch.yaml').write_text(yaml.safe_dump(batch,sort_keys=False))
    cmd=[sys.executable,str(Path(__file__).resolve()),'--worker',str(out)]
    meta={'git_hash':lock['git_hash'],'config_hash':sha(config),'experiment_fingerprint':lock['experiment_fingerprint'],
          'input_hash':sha(manifest),'case_key':case_key,'sequence':sequence,'stage':stage,'argv':cmd,
          'implementation_hash':sha(__file__),'status':'running'};write(out/'execution.json',meta)
    env={k:v for k,v in os.environ.items() if not k.startswith('UIFGO_')};env['UIFGO_EXPERIMENT_WALL_LIMIT_S']='1800'
    start=time.monotonic()
    with (out/'scheduler.log').open('w') as log:
        code=subprocess.call(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
    meta.update(exit_code=code,runtime=time.monotonic()-start,status='scheduler_finished');write(out/'execution.json',meta)
    result={}
    for cell in read(out/'batch/batch_manifest.json')['cells']:
        path=Path(cell['run_directory']) if cell.get('run_directory') else None
        state=read(path/'run_status.json') if path and (path/'run_status.json').exists() else {}
        result[METHODS[cell['canonical_mode']]]={'run':str(path) if path else None,'cell':cell,'backend':state,
                                               'identity':dict(meta,method_id=METHODS[cell['canonical_mode']])}
        if path:write(path/'formal_identity.json',result[METHODS[cell['canonical_mode']]]['identity'])
    write(out/'native_results.json',result);verify();return result


def clean_summary(results):
    output=[]
    for sequence,methods in results.items():
        producer=Path(methods['AUX_PRODUCER']['run'])
        state=read(producer/'production_detector_status.json') if (producer/'production_detector_status.json').exists() else {}
        trace=rows(producer/'production_cusum_trace.csv') if (producer/'production_cusum_trace.csv').exists() else []
        for aid in (7475,9524,10548,15155,20276):
            a=[r for r in trace if int(r['anchor_id'])==aid];valid=[r for r in a if r['diagnostic_valid']=='1']
            z=[float(r['conditional_z']) for r in valid]
            output.append({'sequence':sequence,'anchor_id':aid,'signal_count':len(a),'valid_signal_count':len(z),
                           'clean_exposure_s':float(a[-1]['timestamp'])-float(a[0]['timestamp']) if a else None,
                           'conditional_z_mean':float(np.mean(z)) if z else None,'conditional_z_std':float(np.std(z)) if z else None,
                           'conditional_z_min':min(z) if z else None,'conditional_z_max':max(z) if z else None,
                           'candidate_count':sum(r['final_candidate']=='1' for r in a),
                           'segment_count':len({r['segment_id'] for r in a if r['segment_id']}),
                           'total_candidate_count':state.get('candidate_observation_count'),
                           'status':'no_candidates' if state.get('valid') and state.get('candidate_observation_count')==0 else
                                    'completed' if state.get('valid') else 'failed',
                           'source_run':str(producer),**{k:methods['AUX_PRODUCER']['identity'][k] for k in
                                                       ('git_hash','config_hash','experiment_fingerprint','input_hash','method_id')}})
    save(HERE/'metrics/clean_detector_calibration.csv',output)
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--worker',type=Path);p.add_argument('--r3',action='store_true');a=p.parse_args()
    if a.worker:raise SystemExit(worker(a.worker))
    if a.r3:
        lock=verify() if FROZEN.exists() else freeze();result={}
        for n in (1,2,3):
            key=f'clean{n}'
            if (HERE/'runs/R3'/key).exists():
                assert not (HERE/'runs/R3'/key/'batch').exists(), 'do not rerun a scientific batch'
                key+='-admission-fixed'
            result[f'Walk{n}']=run_native(key,f'Walk{n}',Path(lock['inputs'][n-1]['manifest']),'R3',False)
            print('R3 completed Walk'+str(n),flush=True)
        write(HERE/'runs/R3/results.json',result);summary=clean_summary(result)
        report=['# R3 formal freeze','', 'Backbone B0_CURRENT. No parameter recalibration or optimization.',
                'Formal denotes preregistered controlled benchmark, not unseen held-out or physical NLOS.',
                '', 'Fingerprint: '+lock['experiment_fingerprint'], '',
                'Configs and protected source/input hashes: `../configs/formal_lock.json`.',
                'Clean per-anchor signals/counts: `../metrics/clean_detector_calibration.csv`.',
                'Actual per-run commands, exit codes and native artifacts: `../runs/R3/`.',
                '', 'Stable-improvement classification is fixed before injections: at least 12 valid pairs, '
                'at least 10/15 strict improvements in each of ATE and fault RMSE, and both paired medians <0. '
                'Missing/failure/no-candidate cases do not count as improvements. This descriptive rule is not statistical significance.',
                '', 'Clean statuses: '+json.dumps({s:r['AUX_PRODUCER']['backend'] for s,r in result.items()})]
        valid=all(r['status'] in ('completed','no_candidates') for r in summary)
        report+=['','R3 '+('PASS' if valid else 'FAIL_DETECTOR_VERIFICATION')]
        (HERE/'audits/formal_freeze_report.md').write_text('\n'.join(report)+'\n')
        write(HERE/'runs/R3/stage_status.json',{'status':'PASS' if valid else 'FAIL','fingerprint':lock['experiment_fingerprint']})
