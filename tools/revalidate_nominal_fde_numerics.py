#!/usr/bin/env python3
"""Revalidate current numerical rows without overwriting frozen FDE evidence.

C++ production emits raw H/z/C and an actual protocol. An independent Python
SVD/Cholesky oracle reconstructs the served and post-exclusion systems in a
separate process. Frozen statistical recipe and comparator tolerances remain
unchanged. Observed old numerical winners are explicitly excluded, because the
common correctness fixes intentionally change those observations.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--probe',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();root=Path(__file__).resolve().parents[1]
    evidence=root/'docs/evidence/p0-07-corrected-exhaustive'
    inputs=dict(raw=evidence/'authoritative-replay/frozen-input.json',
                truth=evidence/'authoritative-replay/frozen-truth.json',
                config=root/'config/fde_uwb_order1.yaml',manifest=evidence/'oracle-manifest-v1.json')
    a.output.mkdir(parents=True,exist_ok=True)
    actual=a.output/'actual.tsv';capture=a.output/'raw_capture.tsv';oracle=a.output/'oracle.tsv'
    env=dict(os.environ,LD_LIBRARY_PATH=str(a.probe.parent.parent)+':'+os.environ.get('LD_LIBRARY_PATH',''),
             OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    commands=[[str(a.probe),str(inputs['config']),str(inputs['raw']),str(inputs['truth']),str(actual)]]
    with (a.output/'probe.log').open('w') as log:
        subprocess.run(commands[-1],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    hashes={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in inputs.items()}
    capture.write_text(''.join(f'{key}_sha256\t{value}\n' for key,value in hashes.items())+actual.read_text())
    commands.append([sys.executable,str(root/'tools/p0_07_offline_oracle.py'),'--capture',str(capture),
                     '--raw',str(inputs['raw']),'--truth',str(inputs['truth']),'--config',str(inputs['config']),
                     '--manifest',str(inputs['manifest']),'--canonical-manifest',str(inputs['manifest']),
                     '--output',str(oracle)])
    commands.append([sys.executable,str(root/'tools/p0_07_compare_protocols.py'),
                     '--oracle',str(oracle),'--actual',str(actual),'--skip-observed'])
    status='PASS';reason=''
    for index,command in enumerate(commands[1:]):
        with (a.output/f'oracle_{index}.log').open('w') as log:
            result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
        if result.returncode:status='FAIL';reason=f'process {index+1} exit {result.returncode}';break
    record=dict(schema='nominal-common-fde-numerical-revalidation/v1',status=status,reason=reason,
                immutable_inputs=hashes,commands=commands,
                probe_sha256=hashlib.sha256(a.probe.read_bytes()).hexdigest(),
                library_sha256=hashlib.sha256((a.probe.parent.parent/'libuwb_imu_pl.so').read_bytes()).hexdigest(),
                statistical_recipe_changed=False,comparison_tolerances_changed=False,
                frozen_observed_winner_compared=False,old_evidence_overwritten=False)
    (a.output/'result.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record,indent=2))
    return 0 if status=='PASS' else 1

if __name__=='__main__':raise SystemExit(main())
