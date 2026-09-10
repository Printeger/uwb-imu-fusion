#!/usr/bin/env python3
"""Execute exactly the nine preregistered A10 development producers, without retries.

Uses existing C++ estimator and T09 cache publisher. No gate, validation, test or solver changes.
"""
import argparse
import copy
import csv
import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

import yaml

import generate_synthetic_input as gen
import run_experiments as scheduler

CANDIDATES = [dict(id=f'P{i+1}', lambda_l1=l1, lambda_tv=tv, active_bias_min_m=active,
                   change_point_min_m=change, merge_max_difference_m=merge)
              for i, (l1,tv,active,change,merge) in enumerate([
                  (4.,20.,.10,.15,.10), (8.,40.,.15,.20,.15), (12.,80.,.20,.25,.20)])]
REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO.parents[1]/'devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner'
TEMPLATE = REPO/'doc/ie_sprint/evidence/t10_a09_stage2_budget_diagnostic_20260909T111641Z/configs/step_v2_outer500_refit200.yaml'


def timestamp():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def identity(path):
    p=Path(path).resolve()
    elf=subprocess.run(['readelf','-n',str(p)],capture_output=True,text=True)
    return dict(path=str(path),realpath=str(p),sha256=gen.file_sha(p),bytes=p.stat().st_size,
                build_id=next((line.strip() for line in elf.stdout.splitlines() if 'Build ID:' in line),None))


def binary_identity():
    ldd=subprocess.run(['ldd',str(RUNNER)],capture_output=True,text=True,check=True)
    paths={RUNNER.resolve()}
    for line in ldd.stdout.splitlines():
        for word in line.replace('=>',' ').split():
            if word.startswith('/') and Path(word).is_file():
                paths.add(Path(word).resolve())
    return dict(utc=timestamp(),ldd_raw=ldd.stdout,
                ldd_normalized=scheduler.normalize_ldd_output(ldd.stdout),
                files=[identity(p) for p in sorted(paths)])


def descendants(pid):
    todo=[pid]
    seen=set()
    while todo:
        p=todo.pop()
        if p in seen:
            continue
        seen.add(p)
        try:
            todo.extend(int(i) for i in Path(f'/proc/{p}/task/{p}/children').read_text().split())
        except (FileNotFoundError,ProcessLookupError,PermissionError):
            pass
    return seen


def run_one(root, cell, truth_path, hidden):
    cid=cell['run_id']
    record=root/'commands'/('pilot_'+cid)
    record.mkdir(parents=True,exist_ok=False)
    argv=['/usr/bin/time','-v','-o',str(record/'resources.txt'),
          '/usr/bin/timeout','--signal=KILL','120s',
          '/usr/bin/strace','-f','-e','trace=openat,open,execve','-o',str(record/'file_access.trace'),
          str(RUNNER),'--config',cell['config'],'--output-root',str(root/'runs'),'--run-id',cid]
    meta=dict(argv=argv,cwd=str(REPO),started_utc=timestamp(),role='development',run_id=cid,
              timeout_s=120,attempt=1,status='RESERVED_BEFORE_SPAWN')
    gen.write_json(record/'command.json',meta)
    t=time.monotonic()
    maps=[]
    polls=0
    with (record/'stdout.log').open('w') as stdout, (record/'stderr.log').open('w') as stderr:
        process=subprocess.Popen(argv,cwd=REPO,stdout=stdout,stderr=stderr)
        while process.poll() is None:
            assert not truth_path.exists() and not (truth_path.parent/'generation_manifest.json').exists()
            polls+=1
            for pid in descendants(process.pid):
                try:
                    if Path(f'/proc/{pid}/exe').resolve() == RUNNER.resolve():
                        content=Path(f'/proc/{pid}/maps').read_text()
                        if 'libgtsam.so' in content and 'libuwb_imu_fgo.so' in content:
                            maps.append((pid,content))
                except (FileNotFoundError,ProcessLookupError,PermissionError):
                    pass
            time.sleep(.02)
    wall=time.monotonic()-t
    meta.update(exit_code=process.returncode,finished_utc=timestamp(),external_wall_s=wall,
                status='TIMEOUT' if process.returncode in (124,137,-9) else 'TERMINAL',
                truth_original_path_absent_polls=polls)
    trace=(record/'file_access.trace').read_text()
    forbidden=[str(truth_path),str(hidden),str(truth_path.parent/'generation_manifest.json')]
    meta['truth_open_trace_matches']=[x for x in forbidden if x in trace]
    meta['runner_maps_captured']=bool(maps)
    if maps:
        pid,content=maps[-1]
        (record/'runner_maps.txt').write_text(content)
        mapped_paths=sorted({line.split()[-1] for line in content.splitlines()
                             if len(line.split())>=6 and line.split()[-1].startswith('/')})
        meta['mapped_elf_identity']=[identity(p) for p in mapped_paths if Path(p).is_file()]
    gen.write_json(record/'command.json',meta)
    print(json.dumps({k:meta[k] for k in ('run_id','exit_code','external_wall_s','runner_maps_captured')}),flush=True)
    return meta


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-root',type=Path,required=True)
    args=parser.parse_args()
    root=args.evidence_root.resolve()
    manifest_path=root/'PILOT_MANIFEST.json'
    if manifest_path.exists():
        raise RuntimeError('pilot already reserved; refusing rerun/retry')
    assert (root/'GENERATOR_CHECKS.json').is_file()
    assert json.loads((root/'GENERATOR_CHECKS.json').read_text())['consistency']['status']=='PASS'
    generation=json.loads((root/'inputs/generation_manifest.json').read_text())
    assert generation['base']['seed']==10101 and generation['base']['role']=='development'
    assert gen.file_sha(gen.__file__)==generation['base']['generator_sha256']
    before=binary_identity()
    gen.write_json(root/'BINARY_IDENTITY_BEFORE.json',before)
    base=yaml.safe_load(TEMPLATE.read_text())
    base['anchors']=[dict(id=i+1,pos=p,prior_sigma=.1) for i,p in enumerate(gen.ANCHORS)]
    base['calibration']['fixed_beta_by_link']={f'0:{i+1}':b for i,b in enumerate(gen.BETA)}
    base['imu'].update(sigma_a=.002,sigma_g=.0002)
    base['initialization'] = dict(use_imu_orientation=False)
    base['uwb'].update(sigma_range=.05,v_max=0.)
    base['keyframe'].update(step=1)
    base['dataset'].update(cache_start_s=0.,cache_duration_s=8.)
    assert base['nlos']['discovery_short_min_count']==2
    assert base['nlos']['discovery_short_min_duration_s']==.01
    assert base['nlos']['discovery_max_outer_iterations']==500 and base['nlos']['max_refit_iterations']==200
    (root/'configs').mkdir(exist_ok=False)
    cells=[]
    for candidate in CANDIDATES:
        for scenario in ('los','step','ramp'):
            cid=candidate['id']+'_'+scenario+'_seed10101'
            cfg=copy.deepcopy(base)
            cfg['nlos'].update({k:v for k,v in candidate.items() if k!='id'})
            cfg['dataset']['cache_manifest']=str(root/'inputs/raw'/scenario/'input_manifest.json')
            path=root/'configs'/f'{cid}.yaml'
            path.write_text(yaml.safe_dump(cfg,sort_keys=False))
            cells.append(dict(run_id=cid,scenario=scenario,support_id=candidate['id'],seed=10101,
                              config=str(path),config_sha256=gen.file_sha(path),role='development',
                              input_cache_id=generation['scenarios'][scenario]['cache_id']))
    gen.write_json(manifest_path,dict(schema='t10_a10_bounded_development_v1',role='development',
        created_utc=timestamp(),protocol_sha256=gen.file_sha(root/'PREREGISTRATION.md'),
        generation_manifest_sha256=gen.file_sha(root/'inputs/generation_manifest.json'),
        template_sha256=gen.file_sha(TEMPLATE),candidates=CANDIDATES,cells=cells,
        planned_estimator_processes=9,hard_max_estimator_processes=9,per_process_limit_s=120,
        B_total_s=1440,B_scheduled_max_s=1080,B_reserve_s=360,retries=0,
        final_gate_runs='NOT_RUN',held_out_validation='NOT_RUN',test='NOT_RUN'))
    # Snapshot actual estimator and generation sources before launch; no rebuild needed (C++ unchanged).
    for p in [TEMPLATE,REPO/'CMakeLists.txt',REPO/'tools/run_ie_paper.cpp',
              *sorted((REPO/'src').glob('*.cpp')),*sorted((REPO/'include/uifgo').glob('*.h')),
              *[REPO/'tools/paper'/n for n in ('generate_synthetic_input.py','check_synthetic_input.py',
                    'run_a10_development_pilot.py','run_experiments.py','reference_recoverability.py')]]:
        dest=root/'source_snapshot'/p.relative_to(REPO)
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(p,dest)
    gen.write_json(root/'SOURCE_IDENTITY_BEFORE.json',{
        str(p.relative_to(root/'source_snapshot')):gen.file_sha(p)
        for p in sorted((root/'source_snapshot').rglob('*')) if p.is_file()})
    results=[]
    truth=root/'inputs/evaluation'
    with tempfile.TemporaryDirectory(prefix='a10-evaluation-custody-') as tmp:
        hidden=Path(tmp)
        shutil.move(str(truth),hidden/'evaluation')
        shutil.move(str(root/'inputs/generation_manifest.json'),hidden/'generation_manifest.json')
        try:
            for cell in cells:
                results.append(run_one(root,cell,truth,hidden))
                gen.write_json(root/'PILOT_EXECUTIONS.json',results)
        finally:
            shutil.move(str(hidden/'evaluation'),truth)
            shutil.move(str(hidden/'generation_manifest.json'),root/'inputs/generation_manifest.json')
    after=binary_identity()
    gen.write_json(root/'BINARY_IDENTITY_AFTER.json',after)
    assert before['files']==after['files'], 'runtime dependency changed during pilot'
    assert all(r['runner_maps_captured'] and not r['truth_open_trace_matches'] for r in results)
    producer=scheduler.producer_provenance(RUNNER)
    published={}
    for cell in cells:
        rd=root/'runs'/cell['run_id']
        if not (rd/'stage2_content_identity.json').is_file():
            published[cell['run_id']]=dict(status='UNAVAILABLE_STAGE2_NOT_CONVERGED')
            continue
        cfg=yaml.safe_load(Path(cell['config']).read_text())
        rm=scheduler.read_json(rd/'run_manifest.json')
        try:
            cache=scheduler.publish_cache(rd,root/'caches','AUTO_DISCOVERY',rm['common_preparation_id'],
                scheduler.common_config_identity(cfg),scheduler.stage2_producer_config_identity(cfg,root/'configs'),False,producer)
            published[cell['run_id']]=dict(cache_id=cache['cache_id'],stage2_status=cache['stage2_status'],
                score_status=cache['score_status'],manifest=str(root/'caches'/cache['cache_id'].split(':')[1]/'stage2_cache_manifest.json'))
        except Exception as error:
            published[cell['run_id']]=dict(status='PUBLISH_FAILED',error=repr(error))
    gen.write_json(root/'PUBLISHED_CACHES.json',published)
    print(json.dumps(published,indent=2))


if __name__ == '__main__':
    main()
