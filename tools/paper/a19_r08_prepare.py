#!/usr/bin/env python3
"""Materialize the closed R08 truth-free run contexts and derived P1 configs."""
import argparse, hashlib, json, re
from pathlib import Path

SCENARIOS=('los','step1','step2','step3','ramp_gentle','ramp_steep')
BASES=(('a10_val_turn_01_seed20101',20101),('a10_val_turn_02_seed20102',20102))
LIBS={
 'core_sha256':Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so'),
 'gtsam_sha256':Path('/usr/local/lib/libgtsam.so.4.2.0'),
 'mpfr_sha256':Path('/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2'),
 'gmp_sha256':Path('/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0')}
def sha(p): return 'sha256:'+hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):
 p=Path(p); p.parent.mkdir(parents=True,exist_ok=True)
 t=p.with_suffix(p.suffix+'.tmp'); t.write_text(json.dumps(v,indent=2,sort_keys=True)+'\n'); t.replace(p)
def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--evidence-root',type=Path,required=True)
 ap.add_argument('--runner',type=Path,required=True); ap.add_argument('--base-config',type=Path,required=True)
 a=ap.parse_args(); root=a.evidence_root.resolve(); runner=a.runner.resolve(); base=a.base_config.resolve()
 text=base.read_text(); rows=[]
 for base_id,seed in BASES:
  for scenario in SCENARIOS:
   raw=(root/'inputs'/base_id/'raw'/scenario/'input_manifest.json').resolve()
   source_context=root/'inputs'/base_id/'raw'/scenario/'validation_context.json'
   context=json.loads(source_context.read_text())
   if context['seed']!=seed or context['scenario_id']!=scenario or context['role']!='validation': raise ValueError('source context mismatch')
   config=root/'configs'/f'{base_id}_{scenario}.yaml'
   changed,n=re.subn(r'(?m)^(\s*cache_manifest:\s*).+$',r'\1'+str(raw),text)
   if n!=1: raise ValueError('config cache_manifest replacement count')
   config.write_text(changed)
   context.update({
    'config_sha256':sha(config),'runner_sha256':sha(runner),
    'source_config_sha256':sha(base),'source_config_semantics':'ONLY_CACHE_MANIFEST_PATH_CHANGED',
    'generator_sha256':sha('tools/paper/generate_synthetic_input.py'),
    'pipeline_source_sha256':sha('tools/paper/a19_r08_pipeline.cpp'),
    'metric_implementation_sha256':sha('tools/paper/a19_r08_evaluate.py'),
    'parent_cache_id':context['cache_id'],
    **{key:sha(path) for key,path in LIBS.items()}})
   target=root/'contexts'/f'{base_id}_{scenario}.json'; write(target,context)
   rows.append({'base':base_id,'seed':seed,'scenario':scenario,'config':str(config),'context':str(target),'raw':str(raw)})
 write(root/'RUN_MATRIX.json',{'schema':'T10_A19_R08_FIXED_MATRIX_V1','role':'validation','order':rows,
  'science_support':'P1','scheduled_wall_limit_s':10800,'total_allocation_s':14400,'reserve_not_consumed_s':3600})
 print(json.dumps({'status':'PASS','cells':len(rows),'runner_sha256':sha(runner)}))
if __name__=='__main__': main()
