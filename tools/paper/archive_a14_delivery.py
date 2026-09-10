#!/usr/bin/env python3
"""Seal A14 byte inventory, archive, extract and verify without estimator replay."""
import hashlib,json,shutil,sys,tarfile,tempfile,time
from pathlib import Path
r=Path(sys.argv[1]).resolve();archive=r.with_suffix('.tar.gz');verification=r.parent/(r.name+'_archive_verification');verification.mkdir(exist_ok=False)
def h(p):
 d=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):d.update(b)
 return d.hexdigest()
assert not archive.exists()
rows=[{'path':str(p.relative_to(r)),'bytes':p.stat().st_size,'sha256':h(p)} for p in sorted(r.rglob('*')) if p.is_file() and p.name!='FILES_SHA256.json']
(r/'FILES_SHA256.json').write_text(json.dumps(rows,indent=2)+'\n');t=time.monotonic()
with tarfile.open(archive,'w:gz',compresslevel=1) as f:f.add(r,arcname=r.name)
with tempfile.TemporaryDirectory(prefix='a14-extract-check-') as tmp:
 with tarfile.open(archive,'r:gz') as f:
  for member in f.getmembers():
   assert not member.name.startswith('/') and '..' not in Path(member.name).parts
  f.extractall(tmp)
 extracted=Path(tmp)/r.name
 for row in rows:
  p=extracted/row['path'];assert p.stat().st_size==row['bytes'] and h(p)==row['sha256'],row['path']
 assert h(extracted/'FILES_SHA256.json')==h(r/'FILES_SHA256.json')
result={'archive':str(archive),'sha256':h(archive),'bytes':archive.stat().st_size,'verified_payload_files':len(rows),'inventory_verified':True,'archive_elapsed_s':time.monotonic()-t,'estimator_replay':'NOT_RUN','truth_GT_inclusion':'NONE; only source/engineering-reference/raw/config/runtime evidence'}
(verification/'VERIFICATION.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
