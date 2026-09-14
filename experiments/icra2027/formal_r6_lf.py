#!/usr/bin/env python3
"""Authorized serialization-only R6 retry. Old frozen artifacts are immutable."""
import argparse
import copy
import json
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import yaml
from formal_native import HERE, ROOT, WS, verify, read, rows, write, sha, digest, run_native
from formal_treatment import export, make_bag, run_sf
from nlos_injection import cache_id
import formal_injection

STAGE = 'R6-LF-v2'
MANIFEST = HERE/'manifests/injection_manifest_canonical_v2.yaml'
ATTEMPT = HERE/'attempts'/STAGE


def normalize(payload):
    """Do not parse/reformat numbers: only canonicalize CRLF record endings."""
    result = payload.replace(b'\r\n', b'\n')
    if b'\r' in result: raise ValueError('BARE_CR_NOT_A_RECORD_ENDING')
    return result


def archive():
    out = HERE/'attempts/R6-CRLF-invalid'
    if out.exists(): raise ValueError('old-attempt archive already exists')
    paths = set()
    for pattern in ('metrics/E2*', 'audits/E2*', 'audits/R3_R6*', 'figures/**/FIG_2*',
                    'tables/**/TABLE_II*', 'manifests/injection_manifest_canonical*', 'formal_*.py'):
        paths.update(p for p in HERE.glob(pattern) if p.is_file())
    for p in sorted(paths):
        target = out/p.relative_to(HERE)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, target)
    immutable = paths | {p for p in (HERE/'runs/R6').rglob('*') if p.is_file()}
    for c in formal_injection.load()['cases']:
        immutable.update(p for p in Path(c['input_manifest']).parent.iterdir() if p.is_file())
    write(out/'preservation_hashes.json', {str(p): sha(p) for p in sorted(immutable)})


def freeze():
    lock = verify()
    assert not MANIFEST.exists()
    old = formal_injection.load()
    archive()
    cases = []; checks = []
    for previous in old['cases']:
        c = copy.deepcopy(previous)
        source = Path(c['input_manifest']); m = read(source)
        key = 'case-'+digest({'parent_key': c['case_key'], 'encoding': 'LF-v2'})[:16]
        target = HERE/'inputs'/key
        target.mkdir(parents=True, exist_ok=False)
        before = (source.parent/m['uwb_file']).read_bytes()
        after = normalize(before)
        assert before != after and before.count(b'\r\n') == after.count(b'\n')
        (target/m['uwb_file']).write_bytes(after)
        shutil.copy2(source.parent/m['imu_file'], target/m['imu_file'])
        assert rows(source.parent/m['uwb_file']) == rows(target/m['uwb_file'])
        assert sha(source.parent/m['imu_file']) == sha(target/m['imu_file'])
        child = copy.deepcopy(m)
        child['uwb_sha256'] = 'sha256:'+sha(target/m['uwb_file'])
        child['cache_id'] = cache_id(child)
        write(target/'input_manifest.json', child)
        c.update(case_key=key, input_manifest=str(target/'input_manifest.json'),
                 corrupted_input_hash=sha(target/'input_manifest.json'), corrupted_uwb_hash=sha(target/m['uwb_file']))
        changed = {k for k in c if c[k] != previous[k]}
        assert changed == {'case_key','input_manifest','corrupted_input_hash','corrupted_uwb_hash'}
        checks.append({'case_id': c['case_id'], 'changed_manifest_fields': sorted(changed),
                       'decoded_rows_equal': True, 'IMU_byte_equal': True, 'only_CRLF_to_LF': True,
                       'parent_input_hash': previous['corrupted_input_hash'], 'new_input_hash': c['corrupted_input_hash']})
        cases.append(c)
    protocol = {'parent_experiment_fingerprint': lock['experiment_fingerprint'],
                'old_manifest_sha256': sha(formal_injection.MANIFEST),
                'operation': 'CRLF_TO_LF_ONLY; same scientific cases and parameters; fresh all R6 methods',
                'cases': cases, 'source_hashes': {str(p):sha(p) for p in
                    [Path(__file__), HERE/'cache_parser_probe.cpp', HERE/'formal_assets.py']}}
    fp = digest(protocol)
    write(ATTEMPT/'protocol.json', dict(protocol, experiment_fingerprint=fp))
    doc = copy.deepcopy(old)
    doc.update(schema='ICRA_CANONICAL_15_CASE_LF_V2', cases=cases, experiment_fingerprint=fp,
               parent_experiment_fingerprint=lock['experiment_fingerprint'],
               supersedes_invalid_manifest_sha256=sha(formal_injection.MANIFEST))
    MANIFEST.write_text(yaml.safe_dump(doc, sort_keys=False))
    write(MANIFEST.with_suffix('.lock.json'), {'sha256':sha(MANIFEST), 'frozen_before_any_R6_method':True,
          'input_hashes':{c['input_manifest']:c['corrupted_input_hash'] for c in cases}})
    write(ATTEMPT/'format_integrity.json', checks)
    return doc


def load():
    verify()
    guard = read(MANIFEST.with_suffix('.lock.json'))
    assert sha(MANIFEST) == guard['sha256']
    for p,h in guard['input_hashes'].items():
        assert sha(p) == h
        m = read(p)
        for kind in ('imu','uwb'):
            assert 'sha256:'+sha(Path(p).parent/m[kind+'_file']) == m[kind+'_sha256']
    return yaml.safe_load(MANIFEST.read_text())


def preflight():
    doc = load(); probe = ATTEMPT/'cache_parser_probe'
    lib = WS/'devel/.private/uwb_imu_fgo/lib'
    cmd = ['g++','-std=c++17','-O2','-march=native','-I'+str(ROOT/'include'),'-I/usr/include/eigen3',
           str(HERE/'cache_parser_probe.cpp'),'-L'+str(lib),'-luwb_imu_fgo',
           '-Wl,-rpath,'+str(lib)+':/opt/ros/noetic/lib:/usr/local/lib','-o',str(probe)]
    build = subprocess.run(cmd, capture_output=True, text=True)
    write(ATTEMPT/'parser_build.json', {'argv':cmd,'exit_code':build.returncode,'stdout':build.stdout,'stderr':build.stderr})
    assert build.returncode == 0
    checks = []
    for c in doc['cases']:
        command = [str(probe),c['input_manifest']]
        result = subprocess.run(command, capture_output=True, text=True)
        checks.append({'case_id':c['case_id'],'argv':command,'exit_code':result.returncode,
                       'stdout':result.stdout,'stderr':result.stderr})
    old = formal_injection.load()['cases'][0]
    regression = subprocess.run([str(probe),old['input_manifest']],capture_output=True,text=True)
    write(ATTEMPT/'parser_preflight.json', {'checks':checks,'probe_hash':sha(probe),
          'old_CRLF_regression_exit_code':regression.returncode,'old_CRLF_error':regression.stderr,
          'actual_frozen_library_loader':True,'estimator_optimizer_calls':0})
    assert all(c['exit_code']==0 for c in checks)
    assert regression.returncode == 1 and 'invalid T07 UWB cache header' in regression.stderr
    print('PARSER PASS 15/15; original invalid CRLF regression confirmed',flush=True)


def run_case(case):
    doc = load(); key=case['case_key']; seq=case['sequence']; manifest=Path(case['input_manifest'])
    native=run_native(key,seq,manifest,STAGE)
    # Parse failures invalidate the comparison; ordinary solver failures do not.
    for log in (HERE/'runs'/STAGE/key).rglob('*.log'):
        if 'invalid T07 UWB cache header' in log.read_text(errors='replace'):
            raise ValueError('NATIVE_INPUT_PARSE_FAILURE')
    t=export(key,manifest,native);results=dict(native)
    write(HERE/'exports/corrected_ranges'/f'{key}_treatment.json',t)
    for method in ('SF_NATIVE','SF_REJECT','SF_RECOVER'):
        bag=make_bag(key,seq,manifest,t,method)
        results[method]=run_sf(key,seq,bag,method,STAGE,ready=True)
        if results[method]['status']=='invalid_input':raise ValueError('SF_TRANSPORT_INTEGRITY_FAILURE')
        print(case['case_id'],method,results[method]['status'],flush=True)
    assert len({results[m]['identity']['config_hash'] for m in ('SF_NATIVE','SF_REJECT','SF_RECOVER')})==1
    for result in results.values():
        result['identity']['parent_experiment_fingerprint']=result['identity']['experiment_fingerprint']
        result['identity']['experiment_fingerprint']=doc['experiment_fingerprint']
    write(HERE/'runs'/STAGE/key/'results.json',results)
    return results


def run():
    load(); preflight()
    assert read(HERE/'runs/R5/stage_status.json')['status']=='completed'
    complete={}
    with ThreadPoolExecutor(max_workers=3) as pool:
        jobs={pool.submit(run_case,c):c for c in load()['cases']}
        for future in as_completed(jobs):
            c=jobs[future]
            try:complete[c['case_id']]=future.result()
            except Exception as exc:
                write(ATTEMPT/'integrity_stop.json',{'case_id':c['case_id'],'error':repr(exc)})
                for job in jobs:job.cancel()
                raise
            write(HERE/'runs'/STAGE/'results.json',complete)
            print('R6 LF CASE FINISHED',c['case_id'],len(complete),'/15',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--freeze',action='store_true');p.add_argument('--run',action='store_true');p.add_argument('--preflight',action='store_true')
    a=p.parse_args()
    if a.freeze:freeze()
    if a.run:run()
    elif a.preflight:preflight()
