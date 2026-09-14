#!/usr/bin/env python3
"""Reproduce the existing clean Walk1 pipeline under the R0 fingerprint."""
import datetime
import json
import subprocess
import sys
import time
import uuid

import yaml
from fingerprint import HERE, ROOT, digest, sha


def main():
    version = yaml.safe_load((HERE / 'VERSION.yaml').read_text())
    fingerprint = version.pop('fingerprint')
    assert digest(version) == fingerprint
    assert sha(version['runner']) == version['runner_sha256']
    out = HERE / 'runs' / ('R0-' + datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
                          + '-' + uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    cmd = [sys.executable, str(ROOT / 'experiments/scripts/run_walk1_smoke.py'), '--output-root', str(out)]
    doc = {'fingerprint': fingerprint, 'argv': cmd, 'cwd': str(ROOT), 'status': 'RUNNING'}
    path = out / 'run_metadata.json'
    path.write_text(json.dumps(doc, indent=2) + '\n')
    start = time.monotonic()
    with (out / 'command.log').open('w') as f:
        code = subprocess.call(cmd, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT)
    doc.update(exit_code=code, runtime_s=time.monotonic()-start,
               status='completed' if code == 0 else 'failed')
    path.write_text(json.dumps(doc, indent=2) + '\n')
    for run in out.glob('*/batch/runs/*'):
        if run.is_dir():
            (run / 'experiment_fingerprint.json').write_text(json.dumps({'fingerprint': fingerprint}) + '\n')
    print(json.dumps(dict(doc, output=str(out))))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
