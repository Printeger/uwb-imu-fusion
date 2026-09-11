#!/usr/bin/env python3
"""Six-input/four-method loading and initialization checks; zero optimization."""
import argparse,json,subprocess,time
from pathlib import Path
import yaml
p=argparse.ArgumentParser();p.add_argument('--manifest',required=True);p.add_argument('--runner',required=True);p.add_argument('--output',required=True);a=p.parse_args()
root=Path(a.output).resolve();root.mkdir(parents=True,exist_ok=False)
ledger=[]
for unit in yaml.safe_load(Path(a.manifest).read_text())['run_units']:
 for method in ['robust_cauchy','suppress_all','structured_debias','lcb_partial']:
  name=unit['run_unit_id']+'-'+method
  command=[a.runner,'--config',unit['config'],'--output-root',str(root/'runs'),'--run-id',name,'--method',method,'--prepare-only']
  t=time.monotonic()
  with (root/(name+'.log')).open('x') as log:
   try: code=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=120).returncode
   except subprocess.TimeoutExpired: code=124
  status_path=root/'runs'/name/'run_status.json'
  status=json.loads(status_path.read_text()) if status_path.is_file() else {}
  ledger.append(dict(input=unit['run_unit_id'],method=method,command=command,exit_code=code,wall_seconds=time.monotonic()-t,status=status,log=str(root/(name+'.log'))))
  (root/'ledger.json').write_text(json.dumps(ledger,indent=2))
  print(name,code,status.get('status'),flush=True)
