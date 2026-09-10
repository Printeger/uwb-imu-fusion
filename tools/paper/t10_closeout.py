#!/usr/bin/env python3
"""Bounded T10 closeout orchestration; reuses the C++ producer and evaluator."""
import argparse
import csv
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time

import numpy as np
from generate_synthetic_input import cache_id, canonical
from a19_r08_prepare import LIBS

REPO = Path(__file__).resolve().parents[2]
OLD = REPO / 'doc/ie_sprint/evidence/t10_a19_r08_validation_compare_20260910T115109Z'
MASTER = REPO / 'doc/ie_sprint/T10_CLOSEOUT_MANIFEST.json'
RUNNER = Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/a19_r08_pipeline')
POLICY = 'PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1'


def obj(p):
    return json.loads(Path(p).read_text())


def sha(p):
    return 'sha256:' + hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write(p, data):
    p = Path(p)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(p.suffix + '.tmp')
    with tmp.open('w') as stream:
        stream.write(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    tmp.replace(p)
    directory_fd = os.open(str(p.parent), os.O_DIRECTORY)
    try: os.fsync(directory_fd)
    finally: os.close(directory_fd)


def read_rows(p):
    with Path(p).open(newline='') as f:
        return list(csv.DictReader(f))


def write_rows(p, data, fields=None):
    with Path(p).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields or list(data[0]), lineterminator='\n')
        w.writeheader()
        w.writerows(data)


def derive(source, target, condition, seed):
    """Generation-only transform; no estimator output or labels select a condition."""
    if condition not in ('B', 'C'):
        raise ValueError(condition)
    target.mkdir(parents=True, exist_ok=False)
    raw = target / 'raw/step2'
    raw.mkdir(parents=True)
    original = source / 'raw/step2'
    shutil.copy2(original / 'imu.csv', raw / 'imu.csv')
    rows = read_rows(original / 'uwb_observations.csv')
    original_ids = [r['obs_id'] for r in rows]
    noise = np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed, 0]))).normal(0., .05, (41, 8))
    if condition == 'B':
        for r in rows:
            k, j = int(r['source_message_index']), int(r['source_range_index'])
            # Add the same frozen noise draw: z_B = z_A + epsilon_A.
            r['observed_range_m'] = format(float(r['observed_range_m']) + noise[k, j], '.17g')
    else:
        rows = [r for r in rows if int(r['source_message_index']) % 2 == 0]
    assert all(r['obs_id'] in original_ids for r in rows)
    write_rows(raw / 'uwb_observations.csv', rows)
    desc = {'condition': condition, 'parent_input_sha256': sha(original / 'input_manifest.json'),
            'sigma_m': .10 if condition == 'B' else .05,
            'selection': 'ALL' if condition == 'B' else 'source_message_index % 2 == 0',
            'noise_transform': 'z_B=z_A+epsilon_A; PCG64 SeedSequence([seed,0])' if condition == 'B' else 'UNCHANGED',
            'obs_id_policy': 'PRESERVE_PARENT_ID_AND_SOURCE_ORDINALS', 'seed': seed}
    manifest = obj(original / 'input_manifest.json')
    manifest.update(base_source_sha256='sha256:' + hashlib.sha256(canonical(desc)).hexdigest(),
                    uwb_sha256=sha(raw / 'uwb_observations.csv'),
                    uwb_observation_count=len(rows), uwb_message_count=len({r['source_message_index'] for r in rows}))
    manifest['cache_id'] = cache_id(manifest)
    write(raw / 'input_manifest.json', manifest)
    assumptions = dict(obj(source / 'generation_manifest.json')['base']['sensor_assumptions'])
    assumptions.update(range_sigma_m=desc['sigma_m'], uwb_hz=5 if condition=='B' else 2.5)
    context = obj(original / 'validation_context.json')
    context.update(cache_id=manifest['cache_id'], raw_input_manifest_sha256=sha(raw / 'input_manifest.json'),
                   closeout_condition=desc,
                   synthetic_assumptions_sha256='sha256:' + hashlib.sha256(canonical(assumptions)).hexdigest(),
                   nominal_noise_sha256='sha256:' + hashlib.sha256(canonical({'range_sigma_m':desc['sigma_m'],'accel_noise_density':.002,'gyro_noise_density':.0002})).hexdigest())
    write(raw / 'validation_context.json', context)
    # Isolated generation sidecars remain inaccessible to the estimator.
    (target / 'evaluation').mkdir()
    shutil.copy2(source / 'evaluation/motion.csv', target / 'evaluation/motion.csv')
    truth = read_rows(source / 'evaluation/step2_range_truth.csv')
    kept = {r['obs_id'] for r in rows}
    truth = [r for r in truth if r['obs_id'] in kept]
    if condition == 'B':
        for r in truth:
            r['range_noise_m'] = format(2 * float(r['range_noise_m']), '.17g')
    write_rows(target / 'evaluation/step2_range_truth.csv', truth)
    generation = obj(source / 'generation_manifest.json')
    generation['scenarios'] = {'step2': dict(generation['scenarios']['step2'], cache_id=manifest['cache_id'])}
    generation['base']['sensor_assumptions'] = assumptions
    generation['base_source_sha256'] = manifest['base_source_sha256']
    generation['closeout_derivation'] = dict(desc, implementation_sha256=sha(__file__), parent_generation_sha256=sha(source/'generation_manifest.json'))
    generation['payload_sha256'] = {str(p.relative_to(target)):sha(p) for p in target.rglob('*') if p.is_file()}
    write(target / 'generation_manifest.json', generation)
    return target


def prepare(root, phase, seed, scenario, condition='A'):
    base = f'a10_val_turn_{1 if seed==20101 else 2:02d}_seed{seed}'
    row = next(r for r in obj(OLD/'RUN_MATRIX.json')['order'] if r['seed']==seed and r['scenario']==scenario)
    case = root / phase / f'{base}_{scenario}_{condition}'
    case.mkdir(parents=True, exist_ok=False)
    generation = OLD / 'inputs' / base
    if condition in ('B','C'):
        generation = derive(generation, case/'input', condition, seed)
    raw = generation / 'raw' / scenario / 'input_manifest.json'
    config = case / 'config.yaml'
    text = Path(row['config']).read_text()
    text, n = re.subn(r'(?m)^(\s*cache_manifest:).+$',r'\1 '+str(raw),text)
    assert n==1
    if condition=='B':
        text,n = re.subn(r'(?m)^(\s*sigma_range:) 0.05$',r'\1 0.10',text)
        assert n==1
    config.write_text(text)
    context = obj(raw.parent/'validation_context.json')
    context.update(config_sha256=sha(config), runner_sha256=sha(RUNNER),
                   source_config_sha256=sha(row['config']), closeout_manifest_sha256=sha(MASTER),
                   source_config_semantics='CLOSEOUT_EXPLICIT_CONDITION_AND_INPUT_PATH',
                   pipeline_source_sha256=sha(REPO/'tools/paper/a19_r08_pipeline.cpp'),
                   metric_implementation_sha256=sha(REPO/'tools/paper/a19_r08_evaluate.py'),
                   parent_cache_id=context['cache_id'],
                   **{k:sha(p) for k,p in LIBS.items()})
    write(case/'context.json', context)
    write(case/'case.json', {'phase':phase,'seed':seed,'scenario':scenario,'condition':condition,
          'generation_manifest':str(generation/'generation_manifest.json'),'parent_run':row,
          'config_sha256':sha(config),'context_sha256':sha(case/'context.json')})
    return case


def run(case):
    # No optimizer calls: reject input wiring before consuming a science attempt.
    prepare_command = [str(RUNNER), '--policy', POLICY, 'validation-prepare-only',
                       str(case/'config.yaml'), 'ORIGINAL_RAW_NO_CHECKPOINT',
                       str(case/'context.json'), str(case/'prepare')]
    with (case/'prepare.log').open('w') as log:
        prepared = subprocess.run(prepare_command, cwd=REPO, stdout=log, stderr=subprocess.STDOUT, timeout=120)
    write(case/'PREPARE_RUN.json', {'argv':prepare_command, 'exit_code':prepared.returncode})
    if prepared.returncode: raise RuntimeError('PREPARE_CORRECTNESS_FAILED')
    command = ['/usr/bin/strace','-f','-e','trace=openat,open,execve','-o',str(case/'file_access.trace'),str(RUNNER),
               '--policy',POLICY,'validation-stage1-stage2-score',str(case/'config.yaml'),
               'ORIGINAL_RAW_NO_CHECKPOINT',str(case/'context.json'),str(case/'output')]
    write(case/'COMMAND.json', {'argv':command,'cwd':str(REPO),'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()})
    start = time.monotonic()
    with (case/'stdout.log').open('wb') as out, (case/'stderr.log').open('wb') as err:
        process = subprocess.Popen(command,cwd=REPO,stdout=out,stderr=err,start_new_session=True)
        try:
            code=process.wait(timeout=700)
        except subprocess.TimeoutExpired:
            # All forks (including their own process groups) stay in this session.
            for line in subprocess.check_output(['ps','-eo','pid=,sid='],text=True).splitlines():
                pid,sid=map(int,line.split())
                if sid==process.pid:
                    try: os.kill(pid,signal.SIGKILL)
                    except ProcessLookupError: pass
            code=process.wait()
    trace=(case/'file_access.trace').read_text(errors='replace')
    forbidden=[line for line in trace.splitlines() if any(x in line.lower() for x in ('/evaluation/','range_truth','motion.csv','ground_truth'))]
    result={'exit_code':code,'elapsed_s':time.monotonic()-start,'truth_open_count':len(forbidden),'truth_open_lines':forbidden,
            'status':'COMPLETE' if code==0 else 'FAILED','attempt_count':1}
    write(case/'RUN_RESULT.json',result)
    print(case.name,json.dumps(result),flush=True)
    if forbidden: raise RuntimeError('TRUTH_EXPOSURE')


def evaluate(case):
    meta=obj(case/'case.json')
    run_root=Path(meta.get('output_root',str(case/'output')))
    files = [p for p in case.rglob('*') if p.is_file() and p.name not in ('FREEZE.json','evaluation.json','evaluation.log','EVALUATION_RUN.json') and '/evaluation/' not in str(p) and 'range_truth' not in p.name and p.name!='generation_manifest.json']
    if meta.get('output_root'):
        previous=obj(OLD/'PRE_EVALUATION_FREEZE.json')
        for item in previous['payloads']:
            path=Path(item['path'])
            if str(path).startswith(str(run_root)+'/'):
                if not path.exists():
                    ledger=REPO/'doc/ie_sprint/evidence/retention_20260910/REMOVED.jsonl'
                    removed={str(REPO/json.loads(line)['path']) for line in ledger.read_text().splitlines()}
                    if str(path) in removed: continue
                    raise ValueError('MISSING_REUSE_PAYLOAD:'+str(path))
                if sha(path)[7:] != item['sha256']: raise ValueError('REUSE_PAYLOAD_MISMATCH:'+str(path))
                files.append(path)
    write(case/'FREEZE.json', {'evaluation_label':'LIMITED_SYNTHETIC_VALIDATION','decisions_and_finals_frozen':True,
          'freeze_scope':'RECONSTRUCTED_POST_CRASH_RETAINED_PAYLOADS',
          'original_new_run_pre_evaluation_hash_lists':'LOST_ZERO_LENGTH_AFTER_REBOOT',
          'evaluator_source_sha256':sha(REPO/'tools/paper/a19_r08_evaluate.py'),
          'retention_policy':str(REPO/'doc/ie_sprint/evidence/retention_20260910/POLICY.json'),
          'payloads':[{'path':str(p),'sha256':sha(p)[7:]} for p in sorted(files)]})
    meta=obj(case/'case.json')
    cmd=['python3',str(REPO/'tools/paper/a19_r08_evaluate.py'),'--run-root',str(run_root),
         '--generation-manifest',meta['generation_manifest'],'--freeze-manifest',str(case/'FREEZE.json'),
         '--output',str(case/'evaluation.json'),'--scenario',meta['scenario'],'--validation-context',str(case/'context.json')]
    with (case/'evaluation.log').open('w') as f:
        code=subprocess.call(cmd,cwd=REPO,stdout=f,stderr=subprocess.STDOUT)
    write(case/'EVALUATION_RUN.json',{'argv':cmd,'exit_code':code})
    print(case.name,'evaluator',code,flush=True)
    return code


def main():
    p=argparse.ArgumentParser()
    p.add_argument('phase',choices=['diagnostic','los','science','evaluate'])
    p.add_argument('--case',type=Path)
    a=p.parse_args()
    root=Path(obj(MASTER)['evidence_root'])
    if a.phase=='evaluate':
        raise SystemExit(evaluate(a.case.resolve()))
    rows = [(20102,'ramp_steep','A')] if a.phase=='diagnostic' else ([(s,'los','A') for s in (20101,20102)] if a.phase=='los' else [(s,'step2',c) for s in (20101,20102) for c in ('B','C')])
    for seed,scenario,condition in rows:
        case=prepare(root,a.phase,seed,scenario,condition)
        run(case)


if __name__=='__main__':
    main()
