#!/usr/bin/env python3
"""Post-seal Walk1 evaluation using the unchanged locked metric functions."""
import argparse,csv,hashlib,json
from pathlib import Path
import yaml
import nlos_experiment_common as common
import nlos_injection_metrics as metrics
from nlos_injection import METHODS


def load(p):return json.loads(p.read_text()) if p.is_file() else {}
def rows(p):return list(csv.DictReader(p.open())) if p.is_file() else []
def val(m,k):return m.get(k,{}).get('value')
def csv_out(path,data):
 fields=list(dict.fromkeys(k for row in data for k in row))
 with path.open('w') as f:
  w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
  for row in data:w.writerow({k:json.dumps(v,sort_keys=True) if isinstance(v,(dict,list)) else v for k,v in row.items()})

def main():
 p=argparse.ArgumentParser();p.add_argument('acceptance',type=Path);p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 manifest=load(a.acceptance/'acceptance_manifest.json')
 protocol=yaml.safe_load((common.ROOT/'config/paper/ie0911/step2_evaluation.yaml').read_text())['run_units']['sfuise_walk1']
 assert 'sha256:'+hashlib.sha256(Path(protocol['ground_truth']).read_bytes()).hexdigest()==protocol['ground_truth_sha256']
 result=[];table_a=[];table_b=[];access=[]
 for scenario in manifest['scenarios']:
  batch=Path(scenario['batch']);seal=common.seal(batch)
  cells=load(batch/'batch_manifest.json').get('cells',[]);original=scenario['original_scenario']
  producer=next((c for c in cells if c['execution_type']=='CACHE_PRODUCER'),{})
  run=Path(producer['run_directory']) if producer.get('run_directory') else batch/'UNAVAILABLE'
  fde=load(run/'fde_status.json');fde_rows=rows(run/'fde_observations.csv');partition=load(run/'support_partition.json')
  truth=load(Path(original['truth'])/'nlos_injection_truth.json')
  ids=set(truth.get('affected_obs_ids',[]));window=original['window'];planned={r['obs_id'] for r in fde_rows if r['planned']=='1' and r['valid']=='1'}
  d=metrics.detection(fde_rows,ids,window[0],fde.get('status')=='SUCCESS',planned)
  points=[dict(r,nlos_candidate=str(int(r['fault_detected']=='1' and r['positive_excess']=='1'))) for r in fde_rows]
  point=metrics.detection(points,ids,window[0],fde.get('status')=='SUCCESS',planned)
  fp=point['positive_excess_candidate'].get('fp');negative=len(planned-(ids&planned))
  d['pointwise_original_positive_excess']=point['positive_excess_candidate'];d['pointwise_fpr']=metrics.ratio(fp,negative) if fp is not None else {'status':'UNAVAILABLE'}
  groups=rows(run/'fde_group_tests.csv')
  item={'condition':scenario['condition'],'scenario_id':scenario['scenario_id'],'seal':seal,'producer':producer,'fde':fde,'detection':d,'group_tests':groups,'partition':partition,'stage2_status':load(run/'stage2_refit_status.json'),'stage2_content_identity':load(run/'stage2_content_identity.json'),'scores':rows(run/'scores_decision.csv'),'segments':rows(run/'segments.csv'),'finals':{}}
  for method in METHODS:
   cell=next((c for c in cells if c['canonical_mode']==method and c['execution_type']!='CACHE_PRODUCER'),{})
   final=Path(cell['run_directory']) if cell.get('run_directory') else batch/'UNAVAILABLE'
   m=metrics.localization(final,protocol,tuple(window)) if cell.get('status')=='COMPLETE' else {}
   fixed=rows(final/'fixed_compensations.csv');status=load(final/'run_status.json')
   record={'condition':scenario['condition'],'method':method,'cell_status':cell.get('status','NOT_RUN'),'reason':cell.get('reason'),'run_directory':str(final),'cache_id':cell.get('cache_id'),'rmse_m':val(m,'aligned_ATE_rmse_m'),'p95_m':val(m,'aligned_ATE_p95_m'),'horizontal_rmse_m':val(m,'aligned_horizontal_rmse_m'),'vertical_rmse_m':val(m,'aligned_height_rmse_m'),'complete_planned_trajectory':m.get('complete_planned_trajectory'),'metrics':m,'status':status,'covariance':load(final/'covariance_status.json'),'fallback':load(final/'fallback_attempt.json'),'fixed_compensations':fixed}
   table_b.append(record);item['finals'][method]=record
   if method in ('all_range','robust_cauchy'):continue
   segments=partition.get('segments',[])
   for seg in segments or [None]:
    sid=seg.get('segment_id') if seg else None;rr=[r for r in fde_rows if r['segment_id']==sid] if sid else []
    amp=next((float(s['amplitude_m']) for s in item['segments'] if s['segment_id']==sid),None)
    correction=next((f for f in fixed if f['segment_id']==sid),{})
    table_a.append({'condition':scenario['condition'],'method':method,'segment_id':sid,'segment_metrics':metrics.segment_metric(rr,ids&planned,*window,amp),'detection':d,'stage2_status':item['stage2_status'],'scores':item['scores'],'fixed_compensation':correction,'final_status':status,'availability':'AVAILABLE' if seg else 'NO_CANDIDATES_LOCAL_SIGMA_RC_LCB_NOT_APPLICABLE'})
  item['clean_gate']=all(x['cell_status']=='COMPLETE' and x['complete_planned_trajectory'] is True and x['rmse_m'] is not None and x['rmse_m']<1 for x in item['finals'].values()) and fde.get('retained_segment_count')==0
  item['injected_gate']=d.get('temporal_support',{}).get('tp',0)>0 and fde.get('retained_segment_count',0)>0 and producer.get('status')=='COMPLETE' and all(item['finals'][k]['cell_status']=='COMPLETE' for k in METHODS[2:])
  # Original truth is not mutated by this evaluator.
  text=(a.acceptance/(scenario['condition']+'_file_access.trace')).read_text()
  forbidden=[l for l in text.splitlines() if ('open(' in l or 'openat(' in l) and any(s in l for s in ['nlos_injection_truth','nlos_injected_observations','ground_truth.tum','ground_truth.csv','.bag'])]
  access.append({'condition':scenario['condition'],'trace_sha256':hashlib.sha256(text.encode()).hexdigest(),'forbidden_opens':forbidden,'hidden_truth_directory':True})
  assert not forbidden
  result.append(item)
 clean=next(x for x in result if x['condition']=='clean');injected=next(x for x in result if x['condition']=='injected')
 gate=clean['clean_gate'] and injected['injected_gate']
 report={'schema':'fde_walk1_acceptance_v1','gate_passed':gate,'matrix_status':'ADMITTED' if gate else 'NOT_RUN_WALK1_GATE_FAILED','results':result,'access':access,'table_a':table_a,'table_b':table_b,'claims':'DEVELOPMENT_ONLY; T10=C2-C, T11=C, C1-C3 unchanged'}
 (a.output/'walk1_acceptance.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');csv_out(a.output/'walk1_table_a.csv',table_a);csv_out(a.output/'walk1_table_b.csv',table_b)
 print(json.dumps({'gate_passed':gate,'matrix_status':report['matrix_status'],'methods':[(x['condition'],x['method'],x['cell_status'],x['rmse_m']) for x in table_b]},indent=2))
if __name__=='__main__':main()
