#!/usr/bin/env python3
"""Fresh frozen-selection matrix. Existing locked experiments require explicit continuation."""
import argparse,itertools,hashlib,json,sys
from pathlib import Path
import yaml
import nlos_injection as n
import nlos_experiment_common as c


def run(args):
 r=Path(args.output).resolve();r.mkdir(parents=True,exist_ok=True)
 if (r/'locked_manifest.json').exists():raise RuntimeError('locked experiment exists; use explicit continuation, never skip an interrupted batch')
 selection=[];scenarios=[]
 def save():n.write_json(r/'selection_manifest.json',dict(schema='nlos_selection_v1',baseline='46d37f6709aeedb4958e484683fd51e5fd8ec0cc',selections=selection,scenarios=scenarios))
 def cache_config(dataset,base,label,spec):
  cache=r/'inputs'/label;truth=r/'truth'/label;cfgpath=r/'configs'/(label+'.yaml')
  if not cache.exists():n.generate(base,cache,truth,spec)
  if not cfgpath.exists():c.config_for(dataset,base,cache/'input_manifest.json',cfgpath)
  return cfgpath
 def screening(cfg,name,subset=None):
  out=r/'selection_runs';log=out/(name+'.log.command.json')
  if not log.exists():c.screen(cfg,out,name,subset)
  state=c.read_json(out/name/'fde_status.json');return state,out/name
 def run_batch(unit,label):
  out=r/'batches'/label;log=Path(str(out)+'.log.command.json')
  if not log.exists():
   (r/'batch_specs').mkdir(exist_ok=True);c.batch(unit,r/'batch_specs'/(label+'.yaml'),out)
  return out
 for index in (1,2,3):
  dataset=f'sfuise_walk{index}';base=r/'base_v2'/f'walk{index}'
  anchors=sorted(a['id'] for a in yaml.safe_load((base/'anchors.yaml').read_text()))
  cfg=r/'configs'/(dataset+'_screen.yaml')
  if not cfg.exists():c.config_for(dataset,base,base/'input_manifest.yaml',cfg)
  state,screen=screening(cfg,dataset+'_normal')
  chosen,audit=n.select_window(dataset,n.rows(screen/'fde_observations.csv') if (screen/'fde_observations.csv').exists() else [],n.rows(base/'uwb_observations.csv')) if state.get('status')=='SUCCESS' else (None,{'status':'RAW_OR_FDE_FAILED','fde_status':state})
  entry=dict(dataset=dataset,normal_selection=chosen,normal_audit=audit,low_candidates=[]);selection.append(entry);save()
  if chosen is None:
   # Explicit clean scenario remains, but no injection can be admitted or low subset selected.
   chosen=((-1,-1),0.,0.);entry['injected_status']='SKIPPED_NO_QUALIFIED_NORMAL_WINDOW'
  link,start,end=chosen
  subsets=[('normal',anchors)]
  if link[0]!=-1:
   locked=None
   for size in (3,4):
    candidates=[list(s) for s in itertools.combinations(anchors,size) if link[1] in s]
    candidates.sort(key=lambda s:hashlib.sha256(f'{n.VERSION}|911|{dataset}|{",".join(map(str,s))}'.encode()).hexdigest())
    for subset in candidates:
     label=dataset+'_low_'+'_'.join(map(str,subset));spec=n.specification(False,*link,start,end,subset);subcfg=cache_config(dataset,base,label,spec)
     state,sr=screening(subcfg,label,subset)
     eligible=False;why='RAW_OR_FDE_FAILED'
     if state.get('status')=='SUCCESS':
      rr=n.rows(sr/'fde_observations.csv');planned=[x for x in rr if x['valid']=='1' and x['planned']=='1']
      if planned and all(x['tested']=='1' for x in planned):
       k=sorted({int(x['keyframe_id']) for x in planned});protected=max(float(x['sensor_time']) for x in planned if int(x['keyframe_id'])<=k[min(4,len(k)-1)])
       times=sorted(float(x['sensor_time']) for x in planned if (int(x['tag_id']),int(x['anchor_id']))==tuple(link))
       choices=n.windows(times,protected,1.,2,.01,end-start,recording_end=max(float(x["sensor_time"]) for x in planned))
       eligible=tuple(link) not in n.persistent_faults(planned,1.,2,.01) and (start,end) in choices and len(k)>=5
       why='PASS' if eligible else 'FIXED_LINK_WINDOW_OR_INITIALIZATION_INELIGIBLE'
     entry['low_candidates'].append(dict(subset=subset,status=why,screen=str(sr),config=str(subcfg)));save()
     if eligible:locked=subset;break
    if locked:break
   if locked:subsets.append(('low',locked));entry['low_selected']=locked
   else:entry['low_status']='SKIPPED_NO_QUALIFIED_SUBSET'
  else:entry['low_status']='SKIPPED_NORMAL_SELECTION_UNAVAILABLE'
  if len(subsets)==1:
   for condition in ('clean','injected'):
    scenarios.append(dict(dataset=dataset,regime='low',condition=condition,scenario_id=dataset+'_low_'+condition,status=entry.get('low_status','SKIPPED_NO_QUALIFIED_SUBSET'),link=link,window=[start,end]))
   save()
  for regime,subset in subsets:
   label=dataset+'_'+regime+'_clean';spec=n.specification(False,*link,start,end,subset);cfg=cache_config(dataset,base,label,spec)
   cell=dict(dataset=dataset,regime=regime,condition='clean',scenario_id=label,subset=subset,link=link,window=[start,end],config=str(cfg),truth=str(r/'truth'/label),status='SELECTED')
   # Selection is locked before any GT-consuming evaluator is launched.
   scenarios.append(cell);save()
   out=run_batch(c.manifest_unit(dataset,label,cfg,subset,modes=('all_range','robust_cauchy'),producer=False),label+'_baselines');cell['baseline_batch']=str(out);c.seal(out);save()
   evalpath=r/'clean_gates'/(label+'.json');evalpath.parent.mkdir(exist_ok=True)
   if not evalpath.exists():
    c.execute(['python3',str(c.ROOT/'tools/paper/evaluate_nlos_clean_gate.py'),'--batch',str(out/'batch_manifest.json'),'--dataset',dataset,'--output',str(evalpath)],str(evalpath)+'.log')
   gate=c.read_json(evalpath);cell['gate']=gate;save()
   if gate.get('passed') and link[0]!=-1:
    ilabel=dataset+'_'+regime+'_injected';ispec=n.specification(True,*link,start,end,subset);icfg=cache_config(dataset,base,ilabel,ispec)
    scenarios.append(dict(dataset=dataset,regime=regime,condition='injected',scenario_id=ilabel,subset=subset,link=link,window=[start,end],config=str(icfg),truth=str(r/'truth'/ilabel),status='ADMITTED'))
   else:
    scenarios.append(dict(dataset=dataset,regime=regime,condition='injected',scenario_id=dataset+'_'+regime+'_injected',subset=subset,link=link,window=[start,end],status='SKIPPED_CLEAN_GATE' if link[0]!=-1 else 'SKIPPED_NO_QUALIFIED_WINDOW',gate=gate))
   save()
 # Manifest frozen once all selection and clean admission outcomes exist.
 manifest=dict(schema='nlos_locked_matrix_v1',baseline='46d37f6709aeedb4958e484683fd51e5fd8ec0cc',selections=selection,scenarios=scenarios)
 locked=r/'locked_manifest.json'
 if locked.exists():
  if c.read_json(locked)!=manifest:raise RuntimeError('locked manifest differs; refuse reselection')
 else:n.write_json(locked,manifest)
 for cell in scenarios:
  if cell['status'].startswith('SKIPPED'):continue
  modes=n.METHODS if cell['condition']=='injected' else n.METHODS[2:]
  label=cell['scenario_id'];out=run_batch(c.manifest_unit(cell['dataset'],label,cell['config'],cell['subset'],modes=modes),label+'_e2e')
  cell['e2e_batch']=str(out);cell['artifact_seal']=c.seal(out)
  n.write_json(r/'execution_manifest.json',dict(locked_manifest_sha256=n.sha(locked.read_bytes()),scenarios=scenarios,selections=selection))
 print('Frozen matrix execution finished',flush=True)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);run(p.parse_args())
