#!/usr/bin/env python3
"""Resume the explicitly user-paused matrix without repeating completed cells."""
import argparse,datetime
from pathlib import Path
import nlos_experiment_common as c
import nlos_injection as n

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);a=p.parse_args();r=Path(a.root).resolve()
 if not (r/'PAUSED.json').exists():raise RuntimeError('user pause ledger required')
 if (r/'RESUME.json').exists():raise RuntimeError('resume already attempted; inspect ledger instead of retrying')
 d=c.read_json(r/'execution_manifest.json');locked=r/'locked_manifest.json'
 assert d['locked_manifest_sha256']==n.sha(locked.read_bytes())
 # Check every sealed artifact before continuation.
 for sc in d['scenarios']:
  for key in ('baseline_batch','e2e_batch'):
   if not sc.get(key):continue
   b=Path(sc[key]);hashes=c.read_json(b/'sealed_hashes.json')
   if not hashes:raise RuntimeError('missing seal '+str(b))
   for rel,h in hashes.items():
    if n.sha((b/rel).read_bytes())!=h:raise RuntimeError('seal mismatch '+str(b/rel))
 note=dict(status='RUNNING',authorized_by='USER_CONTINUE_PAUSED_EXPERIMENT',started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),algorithm_retries=False,interruption_restart=True,completed_cells_repeated=False)
 n.write_json(r/'RESUME.json',note)
 sid='sfuise_walk3_normal_injected';sc=next(x for x in d['scenarios'] if x['scenario_id']==sid)
 old=r/'batches'/(sid+'_e2e');bm=c.read_json(old/'batch_manifest.json')
 for cell in bm['cells']:
  if cell['status']=='INTERRUPTED_USER_PAUSE':
   cell['run_directory']=str(old/'runs/t09-c4b912ef0533459ea6b544b8f8333199');cell['run_id']='t09-c4b912ef0533459ea6b544b8f8333199'
  elif cell['status']=='PREREGISTERED':cell['status']='NOT_RUN_USER_PAUSE_CONTINUED_IN_RESUME_BATCH'
 n.write_json(old/'batch_manifest.json',bm);sc['e2e_batch']=str(old);sc['artifact_seal']=c.seal(old)
 n.write_json(r/'execution_manifest.json',d)
 for sid,tag,modes,key in [('sfuise_walk3_normal_injected','resume_producer',n.METHODS[2:],'resumed_batch'),('sfuise_walk3_low_clean','e2e',n.METHODS[2:],'e2e_batch'),('sfuise_walk3_low_injected','e2e',n.METHODS,'e2e_batch')]:
  sc=next(x for x in d['scenarios'] if x['scenario_id']==sid);out=r/'batches'/(sid+'_'+tag)
  if out.exists():raise RuntimeError('attempt directory already exists')
  unit=c.manifest_unit(sc['dataset'],sid,sc['config'],sc['subset'],modes=modes)
  code=c.batch(unit,r/'batch_specs'/(sid+'_'+tag+'.yaml'),out)
  sc[key]=str(out);sc[key+'_exit_code']=code;sc[key+'_seal']=c.seal(out)
  n.write_json(r/'execution_manifest.json',d)
  if code:raise RuntimeError('scheduler infrastructure failure; inspect without algorithm retry')
 note['status']='COMPLETE';note['completed_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat();n.write_json(r/'RESUME.json',note)
 print('RESUMED_MATRIX_COMPLETE',flush=True)
if __name__=='__main__':main()
