#!/usr/bin/env python3
"""Remove only audited, owned SC8 process artifacts; default is a dry run."""
import argparse
import hashlib
import json
import os
from collections import Counter
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
CACHE=ROOT/'.cache/sc8'
OUT=ROOT/'docs/sc8'

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main(apply=False):
    audit=json.loads((OUT/'SC8_AUDIT.json').read_text())
    metrics=json.loads((OUT/'SC8_METRICS.json').read_text())
    assert audit['status']=='PASS' and metrics['status']!='IN_PROGRESS','Finish the registered matrix and audit before cleanup'
    assert audit['metrics_sha256']==sha(OUT/'SC8_METRICS.json'),'Audit/metrics identity changed'
    assert audit['registry_sha256']==sha(ROOT/'config/sc8_experiments.yaml'),'Audit/registry identity changed'
    assert audit['plotdata_sha256']==sha(OUT/'SC8_PLOTDATA.json'),'Audit/plotdata identity changed'
    assert CACHE.resolve()==CACHE and not CACHE.is_symlink(),'Owned cache path must not redirect elsewhere'
    keep=set()
    for path in (CACHE/'runs').glob('*/execution.json'):
        assert json.loads(path.read_text())['status']!='RUNNING','An experiment is still running'
        keep.add(path)
    keep.update((CACHE/'sfuise_observer/src').glob('*.cpp'))
    keep.update([CACHE/'sfuise_observer/include',CACHE/'sfuise_observer/patch_identity.json'])
    removed=Counter();removed_bytes=Counter();retained=[];listing=[]
    for directory,dirs,files in os.walk(CACHE,topdown=False,followlinks=False):
        parent=Path(directory)
        for name in files+[name for name in dirs if (parent/name).is_symlink()]:
            path=parent/name;rel=path.relative_to(CACHE);size=0 if path.is_symlink() else path.stat().st_size
            listing.append((str(rel),size))
            if path in keep:retained.append(str(rel));continue
            category=rel.parts[0] if len(rel.parts)>1 else 'root_process_files'
            removed[category]+=1;removed_bytes[category]+=size
            if apply:path.unlink()
        if apply and parent!=CACHE:
            try:parent.rmdir()
            except OSError:pass # retained journals or build-support files
    receipt=dict(status='COMPLETE' if apply else 'DRY_RUN',scope=str(CACHE.relative_to(ROOT)),
        removed_files=dict(removed),removed_bytes=dict(removed_bytes),total_removed_files=sum(removed.values()),total_removed_bytes=sum(removed_bytes.values()),
        retained=sorted(retained),inventory_before_sha256=hashlib.sha256(json.dumps(sorted(listing)).encode()).hexdigest(),
        audit_sha256=sha(OUT/'SC8_AUDIT.json'),metrics_sha256=sha(OUT/'SC8_METRICS.json'),
        originals='Only .cache/sc8 derivatives removed; original input data, user pre-existing evidence, source and binaries untouched',
        retained_reason='Compact execution journals prevent accidental repeat solves; generated SFUISE observer sources/include keep the isolated build workspace valid',
        reproducibility='Permanent metrics + plotdata reproduce figures without solvers; raw re-evaluation requires an explicitly versioned fresh execution, not replay of deleted outputs',
        pushed=False)
    if apply:(OUT/'SC8_CLEANUP.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(receipt,ensure_ascii=False,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(__doc__);parser.add_argument('--apply',action='store_true')
    main(parser.parse_args().apply)
