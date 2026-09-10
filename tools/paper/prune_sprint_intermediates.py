#!/usr/bin/env python3
"""User-authorized retention cleanup; never traverses symlinks or raw data."""
import datetime
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
EVIDENCE=ROOT/'doc/ie_sprint/evidence'
AUDIT=EVIDENCE/'retention_20260910'


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    AUDIT.mkdir(exist_ok=True)
    if (AUDIT/"REMOVED.jsonl").exists():
        raise RuntimeError("Refuse to overwrite an existing deletion ledger")
    candidates={}
    parents=set()
    for p in EVIDENCE.rglob('*'):
        if p.is_symlink() or not p.is_file() or AUDIT in p.parents:continue
        if p.name in ('exact_binary64.csv','linear.csv'):
            candidates[p]='REGENERABLE_PER_TRIAL_NUMERICAL_DUMP'
            parents.add(p.parent)
        elif p.suffixes[-2:]==['.tar','.gz'] and p.with_suffix('').with_suffix('').is_dir():
            candidates[p]='DUPLICATE_ARCHIVE_EXPANDED_DIRECTORY_RETAINED'
        elif p.name=='file_access.trace' and any(x.startswith('build_') for x in p.parts):
            candidates[p]='REGENERABLE_BUILD_SYSTEM_CALL_TRACE'
        elif p.stat().st_size>1024*1024:
            with p.open('rb') as f:magic=f.read(8)
            if magic.startswith(b'\x7fELF') or magic==b'!<arch>\n':
                candidates[p]='REBUILDABLE_BINARY_OR_OBJECT_COPY'
    for parent in parents:
        p=parent/'factors.csv'
        if p.is_file() and not p.is_symlink():candidates[p]='PER_TRIAL_FACTOR_MAPPING_FINAL_GRAPHS_RETAINED'
    # Preserve original manifests. A separate retained subset records the
    # post-cleanup audit scope rather than rewriting old freeze history.
    freeze_files=list(EVIDENCE.glob('t10_closeout_*/**/FREEZE.json'))+[EVIDENCE/'t10_a19_r08_validation_compare_20260910T115109Z/PRE_EVALUATION_FREEZE.json']
    expected={}; retained=[]
    for freeze in freeze_files:
        if not freeze.is_file() or freeze.is_symlink():continue
        try:
            data=json.loads(freeze.read_text())
        except json.JSONDecodeError:
            retained.append({'original_manifest':str(freeze),'original_manifest_sha256':digest(freeze),'status':'PREEXISTING_EMPTY_OR_INVALID_JSON_PRESERVED'})
            continue
        keep=[];removed=[]
        for item in data.get('payloads',[]):
            p=Path(item['path'])
            if not p.is_absolute():p=(freeze.parent/p).resolve()
            if p in candidates:
                expected.setdefault(p,set()).add(item['sha256'].removeprefix('sha256:') if hasattr(str,'removeprefix') else item['sha256'].replace('sha256:',''))
                removed.append(str(p))
            else:keep.append(item)
        retained.append({'original_manifest':str(freeze),'original_manifest_sha256':digest(freeze),
                         'retained_payloads':keep,'removed_paths':removed})
    total=sum(p.stat().st_size for p in candidates)
    (AUDIT/'POLICY.json').write_text(json.dumps({'authorization':'User explicitly requested keeping only essential results and deleting regenerable/rebuildable intermediates after disk-full reboot.',
        'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'retained':'Raw inputs, source/configs, commands, status/failed attempts, certificates, final graph/Values, N/R/F/G, trajectories, evaluation, provenance and original manifests.',
        'candidate_files':len(candidates),'candidate_bytes':total},indent=2)+'\n')
    count=0;freed=0
    with (AUDIT/'REMOVED.jsonl').open('w') as log:
        for p,reason in sorted(candidates.items()):
            size=p.stat().st_size;hash_value=digest(p)
            if p in expected and hash_value not in expected[p]:
                raise RuntimeError('FROZEN_REMOVAL_HASH_MISMATCH:'+str(p))
            log.write(json.dumps({'path':str(p.relative_to(ROOT)),'bytes':size,'sha256':hash_value,'reason':reason})+'\n');log.flush()
            p.unlink();count+=1;freed+=size
            if count%3000==0:print(count,'files',round(freed/1024**3,2),'GiB removed',flush=True)
    (AUDIT/'RETAINED_FREEZES.json').write_text(json.dumps({'scope':'SLIMMED_USER_AUTHORIZED_NOT_ORIGINAL_COMPLETE_ARCHIVE','manifests':retained},indent=2)+'\n')
    (AUDIT/'RESULT.json').write_text(json.dumps({'removed_files':count,'removed_bytes':freed,'status':'COMPLETE'},indent=2)+'\n')
    print(count,'files',round(freed/1024**3,2),'GiB removed',flush=True)


if __name__=='__main__':main()
