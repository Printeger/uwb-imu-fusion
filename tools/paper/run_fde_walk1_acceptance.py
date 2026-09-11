#!/usr/bin/env python3
"""Run the locked Walk1 pair once; no selection or algorithm retries."""
import argparse,json,os
from pathlib import Path
import yaml
import nlos_experiment_common as common

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
r=a.output.resolve();r.mkdir(exist_ok=False,parents=True)
old=Path('/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01')
locked=json.loads((old/'locked_manifest.json').read_text())
manifest={'baseline':'c254c6acdce48a1be66febaaa935e0f2846e9992','branch':'C','scenarios':[],'matrix_status':'PENDING_WALK1_GATE'}
for condition in ['clean','injected']:
 scenario=next(x for x in locked['scenarios'] if x['scenario_id']==f'sfuise_walk1_normal_{condition}')
 cfg=yaml.safe_load(Path(scenario['config']).read_text());cfg['nlos']['fde_grouped_test']=True
 path=r/(condition+'.yaml');path.write_text(yaml.safe_dump(cfg,sort_keys=True))
 unit=common.manifest_unit('sfuise_walk1',f'walk1_{condition}',path,scenario.get('subset'))
 spec=r/(condition+'_batch.yaml');spec.write_text(yaml.safe_dump(dict(schema='uifgo_t09_batch_v2',role='development',parameter_provenance='IE0911_DEVELOPMENT_TEST_ONLY_PENDING_VALIDATION',run_units=[unit]),sort_keys=False))
 out=r/(condition+'_batch');trace=r/(condition+'_file_access.trace')
 cmd=['bwrap','--bind','/','/','--dev-bind','/dev','/dev','--tmpfs',str(old/'truth'),'--tmpfs',str(common.ROOT/'data'),'--tmpfs','/home/mint/ws_fusion_uwb/res/ie0911_step2_truth_20260911_01','--','strace','-f','-e','trace=%file','-o',str(trace),'python3',str(common.ROOT/'tools/paper/run_experiments.py'),'--manifest',str(spec),'--runner',str(common.BIN),'--output-root',str(out)]
 env=os.environ.copy();env['UIFGO_EXPERIMENT_WALL_LIMIT_S']='1800'
 code=common.execute(cmd,r/(condition+'.log'),limit=1800*8+120,env=env)
 manifest['scenarios'].append({'scenario_id':scenario['scenario_id'],'condition':condition,'config':str(path),'batch':str(out),'exit_code':code,'original_scenario':scenario})
 (r/'acceptance_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 # Exit codes are outcomes. Do not retry an algorithm failure.
