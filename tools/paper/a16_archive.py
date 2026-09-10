#!/usr/bin/env python3
"""Archive finalized A16 payload, then independently extract/hash every file."""
from pathlib import Path
import hashlib,json,tarfile,tempfile,datetime
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'doc/ie_sprint/evidence/t10_a16_endpoint_precision_20260909T160252Z'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def main():
 assert json.loads((E/'FINAL_CHECKS.json').read_text())['pass']
 entries=[]
 for p in sorted(E.rglob('*')):
  if p.is_file() and p.name!='PAYLOAD_INVENTORY.json':entries.append({'path':str(p.relative_to(E)),'bytes':p.stat().st_size,'sha256':sha(p)})
 inventory={'schema':'t10_a16_payload_inventory_v1','entries':entries}
 (E/'PAYLOAD_INVENTORY.json').write_text(json.dumps(inventory,indent=2)+'\n')
 archive=E.with_suffix('.tar.gz')
 if archive.exists():raise RuntimeError('archive already exists')
 with tarfile.open(archive,'w:gz',compresslevel=1) as t:t.add(E,arcname=E.name)
 failures=[]
 with tempfile.TemporaryDirectory(prefix='a16_archive_verify_') as d:
  d=Path(d)
  with tarfile.open(archive,'r:gz') as t:t.extractall(d)
  unpack=d/E.name
  for e in entries:
   p=unpack/e['path']
   if not p.is_file() or p.stat().st_size!=e['bytes'] or sha(p)!=e['sha256']:failures.append(e['path'])
  assert sha(unpack/'PAYLOAD_INVENTORY.json')==sha(E/'PAYLOAD_INVENTORY.json')
 result={'archive':str(archive),'sha256':sha(archive),'bytes':archive.stat().st_size,'payload_files':len(entries),'inventory_verified':True,'failed_payloads':failures,'status':'PASS' if not failures else 'FAIL','estimator_replay':'NOT_RUN','verified_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
 target=E.with_name(E.name+'_archive_verification');target.mkdir(exist_ok=False);(target/'VERIFICATION.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result));assert not failures
if __name__=='__main__':main()
