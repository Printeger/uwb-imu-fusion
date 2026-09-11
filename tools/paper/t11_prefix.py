#!/usr/bin/env python3
"""Frozen T11 physical prefixes and serial tickets; never opens evaluation truth."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
from t10_closeout import REPO, OLD, RUNNER, POLICY, LIBS, obj, sha, write, read_rows
from generate_synthetic_input import cache_id, canonical

MASTER = REPO / 'doc/ie_sprint/T11_MANIFEST.json'
POLICIES = ('full_gate', 'structured_debias')


def crop(source, target, cutoff):
    """Keep original bytes, order, IDs and source ordinals, including exact cutoff."""
    lines = Path(source).read_bytes().splitlines(keepends=True)
    column = lines[0].decode().strip().split(',').index('sensor_time_s')
    kept = [line for line in lines[1:] if float(line.split(b',')[column]) <= cutoff]
    if any(not 0 <= float(line.split(b',')[column]) <= cutoff for line in kept):
        raise ValueError('INVALID_PREFIX_TIME')
    Path(target).write_bytes(lines[0] + b''.join(kept))
    return len(kept)


def prepare(seed, h, replay=False, destination=None):
    manifest = obj(MASTER)
    row = next(x['row'] for x in manifest['inputs'] if x['row']['seed'] == seed)
    case = destination or Path(manifest['evidence_root']) / 'runs' / f'{seed}_{"U13" if replay else "H"+str(h)}'
    case.mkdir(parents=True, exist_ok=False)
    original = Path(row['raw']).parent
    source = original
    cutoff = 6 + h
    if replay:
        # Independently delete all strictly future rows in a full-source copy,
        # then invoke the SAME prefix builder. The copy never reaches estimator.
        source = case / 'mutated_full_source'
        source.mkdir()
        for name in ('uwb_observations.csv', 'imu.csv'):
            crop(original / name, source / name, 6)
        write(case / 'U13_RECIPE.json', {'operation': 'delete rows with sensor_time_s > 6 in independent full-source copy',
              'parent': {n: sha(original/n) for n in ('uwb_observations.csv','imu.csv')},
              'mutated': {n: sha(source/n) for n in ('uwb_observations.csv','imu.csv')}})
    raw = case / 'input'
    raw.mkdir()
    counts = {name: crop(source/name, raw/name, cutoff) for name in ('uwb_observations.csv','imu.csv')}
    assert list(counts.values()) == manifest['expected_counts'][str(h)]
    m = obj(original/'input_manifest.json')
    m.update(imu_count=counts['imu.csv'], uwb_observation_count=counts['uwb_observations.csv'],
             uwb_message_count=len({r['source_message_index'] for r in read_rows(raw/'uwb_observations.csv')}),
             imu_sha256=sha(raw/'imu.csv'), uwb_sha256=sha(raw/'uwb_observations.csv'))
    # recording identity retains ID namespace; source content identity is prefix-only.
    m['base_source_sha256'] = 'sha256:' + hashlib.sha256(canonical({
        'recording':m['base_recording_id'], 'cutoff':cutoff,
        'imu':m['imu_sha256'], 'uwb':m['uwb_sha256'], 'schema':'T11_PREFIX_CONTENT_V1'})).hexdigest()
    m['cache_id'] = cache_id(m)
    write(raw/'input_manifest.json',m)
    config = yaml.safe_load(Path(row['config']).read_text())
    config['dataset'].update(cache_manifest=str(raw/'input_manifest.json'),cache_duration_s=float(cutoff))
    (case/'config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    context = obj(original/'validation_context.json')
    context.update(cache_id=m['cache_id'],raw_input_manifest_sha256=sha(raw/'input_manifest.json'),
        config_sha256=sha(case/'config.yaml'),runner_sha256=sha(RUNNER), **{k:sha(p) for k,p in LIBS.items()},
        t11={'manifest_sha256':sha(MASTER),'cutoff_s':cutoff,'compact_output':True,
             'seen_validation':True,'final_policies':list(POLICIES)})
    write(case/'context.json',context)
    write(case/'lineage.json',{'seed':seed,'H':h,'replay':replay,'source':str(original),
          'source_manifest_sha256':sha(original/'input_manifest.json'), 'source_config_sha256':sha(row['config']),
          'prefix_cache_id':m['cache_id'],'recording_id':m['base_recording_id'],
          'full_source_is_lineage_only':True,'master_sha256':sha(MASTER)})
    if replay:
        import shutil
        shutil.rmtree(source)  # only this invocation's regenerable temporary copy
    return case


def command(case, prepare_only=False):
    return [str(RUNNER),'--policy',POLICY,
            'validation-prepare-only' if prepare_only else 'validation-stage1-stage2-score',
            str(case/'config.yaml'),'ORIGINAL_RAW_NO_CHECKPOINT',str(case/'context.json'),
            str(case/('prepare' if prepare_only else 'output'))]


def execute(argv, root, timeout):
    root.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    write(root/'COMMAND.json',{'argv':argv,'cwd':str(REPO),'hard_limit_s':timeout,
        'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()})
    timed_out = False
    with (root/'stdout.log').open('wb') as out, (root/'stderr.log').open('wb') as err:
        p = subprocess.Popen(argv,cwd=REPO,stdout=out,stderr=err,start_new_session=True)
        try: code=p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out=True
            for line in subprocess.check_output(['ps','-eo','pid=,sid='],text=True).splitlines():
                pid,sid=map(int,line.split())
                if sid==p.pid:
                    try: os.kill(pid,signal.SIGKILL)
                    except ProcessLookupError: pass
            code=p.wait()
    result={'exit_code':code,'timed_out':timed_out,'elapsed_s':time.monotonic()-started}
    write(root/'RESULT.json',result)
    return result


def access_audit(case):
    """Positive input allowlist plus forbidden truth/old artifacts for all descendants."""
    import re
    trace=(case/'file_access.trace').read_text(errors='replace')
    opened=[]; forbidden=[]
    allowed=[case/'input',case/'output',case/'config.yaml',case/'context.json']
    for line in trace.splitlines():
        if not re.search(r'\bopen(?:at)?\(',line): continue
        matches=re.findall(r'"([^"\n]+)"',line)
        if not matches: continue
        path=matches[0]
        opened.append(path)
        if (str(OLD) in path or any(x in path.lower() for x in ('/evaluation/','range_truth','motion.csv','ground_truth','diagnostic_fixed_model'))
            or (str(Path(obj(MASTER)['evidence_root'])) in path and not any(path==str(a) or path.startswith(str(a)+'/') for a in allowed))):
            # Diagnostic exports are allowed writes only, never inputs.
            if 'O_WRONLY' not in line: forbidden.append(line)
    result={'status':'PASS' if not forbidden else 'FAIL','forbidden':forbidden,'opened_paths':sorted(set(opened))}
    write(case/'ACCESS_AUDIT.json',result)
    return result


def run(case):
    if (case/'TICKET.json').exists(): raise RuntimeError('TICKET_ALREADY_CONSUMED')
    prep=execute(command(case,True),case/'prepare_execution',120)
    if prep['exit_code']: raise RuntimeError('PREFIX_PREPARE_FAILED')
    write(case/'TICKET.json',{'attempt':1,'master_sha256':sha(MASTER),'runner':sha(RUNNER)})
    argv=['/usr/bin/strace','-f','-e','trace=openat,open,execve','-o',str(case/'file_access.trace')]+command(case)
    # Internal alarm limits automatic chain to 900s. Each forked final has its
    # own 900s WaitFinalChild limit. Outer bound only guards orchestration.
    result=execute(argv,case/'execution',2730)
    audit=access_audit(case)
    print(case.name,result,audit['status'],flush=True)
    if audit['status']!='PASS': raise RuntimeError('T11_INPUT_ACCESS_VIOLATION')


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','run','all'])
    p.add_argument('--seed',type=int,choices=[20101,20102]);p.add_argument('--h',type=int,choices=[0,1,2],default=0)
    p.add_argument('--replay',action='store_true');p.add_argument('--case',type=Path)
    a=p.parse_args()
    if a.action=='run': run(a.case.resolve()); return
    if a.action=='prepare': print(prepare(a.seed,a.h,a.replay));return
    # The fixed manifest has no restart/retry path. Existing cases are rejected.
    for h,replay in [(0,False),(0,True),(1,False),(2,False)]:
        for seed in (20101,20102): run(prepare(seed,h,replay))

if __name__=='__main__': main()
