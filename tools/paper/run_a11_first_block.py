#!/usr/bin/env python3
"""Frozen A11 three-process development diagnostic; no generation or retries."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

from run_a10_development_pilot import REPO, RUNNER, binary_identity, descendants, identity, timestamp

A10 = REPO/'doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, obj):
    path.write_text(json.dumps(obj, indent=2, sort_keys=True)+'\n')


def frozen_check(root):
    identities = json.loads((root/'FROZEN_IDENTITIES.json').read_text())
    assert all(sha(REPO/p) == h for p,h in identities.items())
    return identities


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-root', required=True, type=Path)
    args = parser.parse_args()
    root = args.evidence_root.resolve()
    assert not (root/'EXECUTION_RESERVATION.json').exists(), 'no retry or rerun'
    frozen_check(root)
    protocol = REPO/'doc/ie_sprint/T10_A11_PROTOCOL.md'
    shutil.copy2(protocol, root/'PREREGISTERED_PROTOCOL.md')
    cells = []
    for scene in ('los','step','ramp'):
        rid = f'P1_{scene}_seed10101'
        config = A10/'configs'/f'{rid}.yaml'
        cells.append(dict(run_id=rid, scene=scene, seed=10101, role='development',
                          config=str(config), config_sha256=sha(config)))
        target=root/'frozen_a10'/'configs'/config.name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(config,target)
        shutil.copytree(A10/'inputs/raw'/scene,root/'frozen_a10/inputs/raw'/scene)
        base=root/'frozen_a10/baseline'/rid
        base.mkdir(parents=True)
        for name in ('discovery_failure_diagnostics.json','common_preparation.json','input_manifest.json',
                     'config_effective.yaml','observations.csv'):
            shutil.copy2(A10/'runs'/rid/name,base/name)
    write(root/'BINARY_BEFORE.json',binary_identity())
    write(root/'EXECUTION_RESERVATION.json',dict(utc=timestamp(),protocol_sha256=sha(protocol),
          estimator_process_limit=3,per_process_limit_s=120,cells=cells,retry_allowed=False))
    executions=[]
    for cell in cells:
        frozen_check(root)
        rec=root/'commands'/('run_'+cell['run_id'])
        rec.mkdir(parents=True,exist_ok=False)
        argv=['/usr/bin/time','-v','-o',str(rec/'resources.txt'),
              '/usr/bin/timeout','--signal=KILL','120s',
              '/usr/bin/strace','-f','-e','trace=openat,open,execve','-o',str(rec/'file_access.trace'),
              str(RUNNER),'--config',cell['config'],'--output-root',str(root/'runs'),
              '--run-id',cell['run_id'],'--diagnostic-conditional-lm-outer','1',
              '--diagnostic-first-block-budget']
        meta=dict(argv=argv,cwd=str(REPO),started_utc=timestamp(),run_id=cell['run_id'],
                  role='development',attempt=1,timeout_s=120,status='RESERVED_BEFORE_SPAWN')
        write(rec/'command.json',meta)
        t=time.monotonic()
        maps=None
        with (rec/'stdout.log').open('w') as stdout,(rec/'stderr.log').open('w') as stderr:
            proc=subprocess.Popen(argv,cwd=REPO,stdout=stdout,stderr=stderr)
            while proc.poll() is None:
                if maps is None:
                    for pid in descendants(proc.pid):
                        try:
                            if Path(f'/proc/{pid}/exe').resolve() == RUNNER.resolve():
                                content=Path(f'/proc/{pid}/maps').read_text()
                                if 'libgtsam.so' in content and 'libuwb_imu_fgo.so' in content:
                                    maps=content
                        except (FileNotFoundError,ProcessLookupError,PermissionError):
                            pass
                time.sleep(.02)
        meta.update(exit_code=proc.returncode,finished_utc=timestamp(),external_wall_s=time.monotonic()-t,
                    status='TIMEOUT' if proc.returncode in (124,137,-9) else 'TERMINAL',
                    runner_maps_captured=maps is not None)
        if maps:
            (rec/'runner_maps.txt').write_text(maps)
            paths=sorted({line.split()[-1] for line in maps.splitlines()
                          if len(line.split())>=6 and line.split()[-1].startswith('/')})
            meta['mapped_elf_identity']=[identity(p) for p in paths if Path(p).is_file()]
        trace=(rec/'file_access.trace').read_text()
        meta['forbidden_truth_open_lines']=[line for line in trace.splitlines()
            if '/inputs/evaluation/' in line or 'generation_manifest.json' in line
            or '/inputs/evaluation"' in line]
        write(rec/'command.json',meta)
        executions.append(meta)
        write(root/'EXECUTIONS.json',executions)
        frozen_check(root)
        print(json.dumps({k:meta[k] for k in ('run_id','exit_code','external_wall_s','runner_maps_captured')}),flush=True)
    write(root/'BINARY_AFTER.json',binary_identity())
    write(root/'FROZEN_AFTER.json',frozen_check(root))


if __name__ == '__main__':
    main()
