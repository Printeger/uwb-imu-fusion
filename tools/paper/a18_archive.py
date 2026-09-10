#!/usr/bin/env python3
import sys,json,hashlib,tarfile,tempfile,time,shutil
from pathlib import Path
E=Path(sys.argv[1]).resolve();start=time.monotonic();assert json.loads((E/'FINAL_CHECKS.json').read_text())['passed']
files=[]
for p in sorted(E.rglob('*')):
 if p.is_file() and p.name!='PAYLOAD_INVENTORY.json':files.append({'path':str(p.relative_to(E)),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
inv=E/'PAYLOAD_INVENTORY.json';inv.write_text(json.dumps(files,indent=2)+'\n');archive=E.with_suffix('.tar.gz');assert not archive.exists()
with tarfile.open(archive,'w:gz',compresslevel=1) as tf:tf.add(E,arcname=E.name)
archive_sha=hashlib.sha256(archive.read_bytes()).hexdigest();verify=E.parent/(E.name+'_archive_verification');verify.mkdir()
with tempfile.TemporaryDirectory(prefix='t10-a18-independent-archive-') as temp:
 with tarfile.open(archive,'r:gz') as tf:
  assert all(not x.name.startswith('/') and '..' not in Path(x.name).parts for x in tf.getmembers());tf.extractall(temp)
 extracted=Path(temp)/E.name;records=json.loads((extracted/'PAYLOAD_INVENTORY.json').read_text());assert (extracted/'PAYLOAD_INVENTORY.json').read_bytes()==inv.read_bytes()
 seen={str(p.relative_to(extracted)) for p in extracted.rglob('*') if p.is_file()};assert seen=={x['path'] for x in records}|{'PAYLOAD_INVENTORY.json'}
 for r in records:
  p=extracted/r['path'];assert p.stat().st_size==r['bytes'] and hashlib.sha256(p.read_bytes()).hexdigest()==r['sha256'],r['path']
result={'passed':True,'archive':str(archive),'sha256':archive_sha,'archive_bytes':archive.stat().st_size,'payload_files':len(files),'inventory_sha256':hashlib.sha256(inv.read_bytes()).hexdigest(),'independent_extraction_all_hashes_verified':True,'wall_s':time.monotonic()-start}
(verify/'VERIFICATION.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
