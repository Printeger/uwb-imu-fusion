#!/usr/bin/env python3
"""Single preregistered STAR-loc startup-coverage control; old calibration unchanged."""
import argparse,copy,hashlib,json
from pathlib import Path
import pandas as pd,yaml
import run_recover_vs_reject as r
import rr_admitted as b
NAME='zigzag_s4'
PARENT=r.ROOT/'experiments/results/recover-vs-reject-gt-20260912C'

def prepare(out):
 b.verify(PARENT);out.mkdir(parents=True,exist_ok=False);old=r.read(PARENT/'lock.json');d=out/NAME;d.mkdir();inp=d/'input';inp.mkdir();source=PARENT/NAME/'input'
 u=pd.read_csv(source/'uwb_observations.csv',dtype=str,keep_default_na=False);im=pd.read_csv(source/'imu.csv',dtype=str,keep_default_na=False);start=max(float(u.sensor_time_s.iloc[0]),float(im.sensor_time_s.iloc[0]));keep=u.sensor_time_s.astype(float)>=start;crop=u[keep];ub=r.csv_bytes(crop.to_dict('records'),r.UWB_HEADER);ib=(source/'imu.csv').read_bytes();m=r.read(source/'input_manifest.json');m.update(uwb_sha256='sha256:'+hashlib.sha256(ub).hexdigest(),uwb_observation_count=len(crop),uwb_message_count=len(crop),transform_sha256='sha256:'+r.digest(dict(parent_cache=m['cache_id'],crop_start_s=start,rule='DROP_UWB_BEFORE_FIRST_CORRECTED_IMU_KEEP_IMU_AND_SOURCE_IDS')));m['cache_id']=r.cache_id(m)
 (inp/'uwb_observations.csv').write_bytes(ub);(inp/'imu.csv').write_bytes(ib);r.write(inp/'input_manifest.json',m)
 for fn in ['config.yaml','cauchy.yaml','sfuise.yaml']:
  c=yaml.safe_load((PARENT/NAME/fn).read_text())
  if fn!='sfuise.yaml':c['dataset']['cache_manifest']=str(inp/'input_manifest.json')
  (d/fn).write_text(yaml.safe_dump(c,sort_keys=False))
 row=dict(recording=NAME,interval_s=[float(crop.sensor_time_s.iloc[0]),float(crop.sensor_time_s.iloc[-1])],measurement_id=m['cache_id'],removed_uwb_rows=int(sum(~keep)),crop_threshold_s=start)
 lock=dict(schema='STARLOC_CROP_DIAGNOSTIC_V1',geometry=old['geometry'],rows=[row],parent=str(PARENT),parent_lock_id=old['lock_id'],source_hashes=b.filemap(source.iterdir()),core_files=b.core_files(),implementation=b.filemap([Path(__file__).resolve(),r.ROOT/'experiments/STARLOC_SIGNAL_DIAGNOSTIC_PROTOCOL.md',Path(b.__file__),Path(r.__file__)]),prepared_hashes=b.filemap([*inp.iterdir(),d/'config.yaml',d/'cauchy.yaml',d/'sfuise.yaml']),scientific_budget=dict(max_tasks=3,seconds_per_tree=1800,retries=0))
 lock['lock_id']=r.digest(lock);r.write(out/'lock.json',lock);print(row)

def execute(out):
 lock=b.verify(out);pre=r.read(out/'preflight.json');assert pre['status']=='PASS' and pre['lock_id']==lock['lock_id'];assert not (out/'execution.json').exists();ledger=dict(status='RUNNING',lock_id=lock['lock_id'],tasks=[]);r.write(out/'execution.json',ledger)
 for task in ['producer','robust_cauchy','SFUISE-ToA']:
  try:result=b.call_sfuise(out,NAME) if task=='SFUISE-ToA' else b.call_backend(out,NAME,task)
  except Exception as ex:
   inv=out/NAME/(task+'_invocation.json');result=r.read(inv) if inv.exists() else dict(task=task,recording=NAME);result.update(status='FAILURE',reason=str(ex))
  if result.get('run_directory'):result['artifact_hashes']=b.filemap(p for p in Path(result['run_directory']).rglob('*') if p.is_file())
  ledger['tasks'].append(result);r.write(out/'execution.json',ledger);print(task,result['status'],result.get('reason'),flush=True)
 ledger['status']='COMPLETE_WITH_FAILURES_RETAINED' if any(t['status']!='SUCCESS' for t in ledger['tasks']) else 'SUCCESS';r.write(out/'execution.json',ledger)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('stage',choices=['prepare','preflight','execute','verify']);p.add_argument('--run',type=Path,required=True);a=p.parse_args();out=a.run.resolve()
 if a.stage=='prepare':prepare(out)
 elif a.stage=='preflight':raise SystemExit(b.preflight(out))
 elif a.stage=='execute':execute(out)
 else:b.verify(out);print('PASS')
