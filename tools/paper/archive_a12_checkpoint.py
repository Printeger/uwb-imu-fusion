#!/usr/bin/env python3
"""Seal and verify A12 artifacts without any estimator or truth access."""
import hashlib,json,sys,tarfile,tempfile,shutil,datetime
from pathlib import Path

def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
r=Path(sys.argv[1]).resolve();archive=r.with_suffix('.tar.gz');verify=r.parent/(r.name+'_archive_verification');verify.mkdir(exist_ok=False)
assert not archive.exists()
shutil.copy2(__file__,r/'implementation'/Path(__file__).name)
files={str(p.relative_to(r)):sha(p) for p in sorted(r.rglob('*')) if p.is_file()}
(r/'FILES_SHA256.json').write_text(json.dumps(files,indent=2)+'\n')
with tarfile.open(archive,'w:gz',compresslevel=6) as t:t.add(r,arcname=r.name)
with tempfile.TemporaryDirectory(prefix='a12_archive_verify_') as td:
 with tarfile.open(archive,'r:gz') as t:
  for m in t.getmembers():
   assert not Path(m.name).is_absolute() and '..' not in Path(m.name).parts
  t.extractall(td)
 base=Path(td)/r.name;manifest=json.loads((base/'FILES_SHA256.json').read_text());mismatch=[p for p,h in manifest.items() if sha(base/p)!=h]
 assert not mismatch
 result={'schema':'a12_archive_content_verification_v1','utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'archive':str(archive),'archive_sha256':sha(archive),'archive_bytes':archive.stat().st_size,'content_files_verified':len(manifest),'hash_mismatches':mismatch,'estimator_replay':'NOT_RUN_BUDGET_CLOSED','truth_in_archive':False,'passed':True}
(verify/'VERIFICATION.json').write_text(json.dumps(result,indent=2)+'\n')
(archive.parent/(archive.name+'.sha256')).write_text(result['archive_sha256']+'  '+archive.name+'\n')
print(json.dumps(result,indent=2))
