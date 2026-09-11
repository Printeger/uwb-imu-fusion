#!/usr/bin/env python3
"""Generate both tables from sealed runs; truth is evaluation-only."""
import argparse,json,math
from collections import Counter
from pathlib import Path
import numpy as np
import yaml
import nlos_injection as n
import nlos_experiment_common as c
import nlos_injection_metrics as met

def readrows(p):return n.rows(p) if Path(p).is_file() else []
def finite(v):
 try:
  x=float(v);return x if math.isfinite(x) else None
 except (ValueError,TypeError):return None

def flatcsv(path,rows):
 keys=sorted(set().union(*(set(r) for r in rows))) if rows else ['status']
 normalized=[{k:json.dumps(v,sort_keys=True) if isinstance(v,(dict,list,tuple)) else v for k,v in r.items()} for r in rows]
 Path(path).write_bytes(n.csv_bytes(normalized,keys))

def value(metrics,key):
 x=metrics.get(key,{});return x.get('value') if x.get('status')=='AVAILABLE' else None

def artifacts(run):
 if run is None:return {}
 names=['run_status.json','observations.csv','common_preparation.json','fde_status.json','fde_observations.csv','support_partition.json','stage2_content_identity.json','stage2_refit_status.json','stage2_values.csv','fixed_compensations.csv','final_masks.csv','inference_identity.json','trajectory.tum','residuals.csv','covariance_status.json','fallback_attempt.json']
 return {f:n.sha((run/f).read_bytes()) for f in names if (run/f).is_file()}

def merged_length(intervals):
 merged=[]
 for s,e in sorted(intervals):
  if merged and s<=merged[-1][1]:merged[-1][1]=max(e,merged[-1][1])
  else:merged.append([s,e])
 return sum(e-s for s,e in merged),merged

def run_report(root,report):
 root=Path(root);execution=c.read_json(root/'execution_manifest.json');locked=c.read_json(root/'locked_manifest.json')
 assert execution['locked_manifest_sha256']==n.sha((root/'locked_manifest.json').read_bytes())
 if any(not x['status'].startswith('SKIPPED') and not x.get('e2e_batch') for x in execution['scenarios']):
  raise RuntimeError('matrix is not sealed; evaluator must wait for every estimator')
 if (root/'RESUME.json').exists() and c.read_json(root/'RESUME.json').get('status')!='COMPLETE':
  raise RuntimeError('resume is not complete')
 protocols=yaml.safe_load((c.ROOT/'config/paper/ie0911/step2_evaluation.yaml').read_text())['run_units']
 for dataset in ('sfuise_walk1','sfuise_walk2','sfuise_walk3'):
  p=protocols[dataset];assert n.sha(Path(p['ground_truth']).read_bytes())==p['ground_truth_sha256']
 table_a=[];table_b=[];results=[];audits=[]
 for scenario in execution['scenarios']:
  ident={k:scenario.get(k) for k in ('scenario_id','dataset','regime','condition')};cells=[]
  for key in ('baseline_batch','e2e_batch','resumed_batch'):
   if key not in scenario:continue
   b=Path(scenario[key]);sealed=c.read_json(b/'sealed_hashes.json')
   for rel,hash_ in sealed.items():
    if n.sha((b/rel).read_bytes())!=hash_:raise RuntimeError('sealed artifact changed: '+str(b/rel))
   cells+=c.read_json(b/'batch_manifest.json').get('cells',[])
  by_mode={x['canonical_mode']:x for x in cells if x['execution_type']!='CACHE_PRODUCER'}
  producer=next((x for x in reversed(cells) if x['execution_type']=='CACHE_PRODUCER'),{})
  pr=Path(producer['run_directory']) if producer.get('run_directory') else None
  fde=c.read_json(pr/'fde_status.json') if pr else {};fde_rows=readrows(pr/'fde_observations.csv') if pr else []
  pst=c.read_json(pr/'run_status.json') if pr else {};parts=c.read_json(pr/'support_partition.json') if pr else {}
  truth=c.read_json(Path(scenario['truth'])/'nlos_injection_truth.json') if scenario.get('truth') else {}
  truthids=set(truth.get('affected_obs_ids',[]));start,end=scenario.get('window',[0,0]);link=tuple(scenario.get('link',[-1,-1]));injected=scenario['condition']=='injected'
  planned={r['obs_id'] for r in fde_rows if r['valid']=='1' and r['planned']=='1'}
  if not planned:
   some=next((Path(x['run_directory']) for x in cells if x.get('run_directory') and (Path(x['run_directory'])/'observations.csv').is_file()),None)
   if some:planned={r['obs_id'] for r in readrows(some/'observations.csv') if r['planned']=='1' and r['valid']=='1'}
  if truth:
   # Post-estimation sidecar enrichment, never part of an estimator input.
   truth['planned_affected_obs_ids']=sorted(planned&truthids,key=int);truth['planned_status']='ACTUAL_PLAN' if planned else 'UNAVAILABLE_NO_PLAN'
   n.write_json(Path(scenario['truth'])/'nlos_injection_truth.json',truth)
  detect=met.detection(fde_rows,truthids,start,fde.get('status')=='SUCCESS',planned_ids=planned)
  segments={}
  for r in fde_rows:
   if r['segment_id']:segments.setdefault(r['segment_id'],[]).append(r)
  stage2ok=pst.get('segment_refit_status') in ('CONVERGED','SUCCESS_EMPTY') or c.read_json(pr/'stage2_refit_status.json').get('status') in ('CONVERGED','SUCCESS_EMPTY') if pr else False
  # Successful producer / published cache is also an authoritative successful Stage2 gate.
  stage2ok=bool(stage2ok or producer.get('status')=='COMPLETE')
  amps={x['segment_id']:finite(x.get('amplitude_m')) for x in readrows(pr/'segments.csv')} if pr and stage2ok else {}
  scores=readrows(pr/'scores_decision.csv') if pr else [];groups=readrows(pr/'groups.csv') if pr else []
  ordinal_group={o:g['group_id'] for g in groups for o in g['segment_ordinals'].split(';')};scoremap={x['group_id']:x for x in scores}
  finaldetails={};fixedsets={}
  for mode in n.METHODS:
   cell=by_mode.get(mode,{});run=Path(cell['run_directory']) if cell.get('run_directory') else None
   status=c.read_json(run/'run_status.json') if run else {};fallback=c.read_json(run/'fallback_attempt.json') if run else {}
   metrics=met.localization(run,protocols[scenario['dataset']],(start,end)) if run and cell.get('status')=='COMPLETE' else {}
   masks=readrows(run/'final_masks.csv') if run else [];obs=readrows(run/'observations.csv') if run else []
   denominator=sum(x['planned']=='1' and x['valid']=='1' for x in obs)
   valid_raw=sum(x['valid']=='1' for x in obs)
   used=sum(x['final_use']=='1' for x in masks) if masks else (denominator if cell.get('status')=='COMPLETE' and mode in ('all_range','robust_cauchy') else None)
   fixed=readrows(run/'fixed_compensations.csv') if run else []
   if mode in ('lcb_partial','lcb_fixed_full') and cell.get('status')=='COMPLETE':fixedsets[mode]={x['segment_id'] for x in fixed if x['decision_use']=='1'}
   b=dict(**ident,method=mode,status=cell.get('status',scenario.get('status','NOT_RUN')),reason=cell.get('reason',status.get('reason')),final_status=status.get('status'),
      rmse_m=value(metrics,'aligned_ATE_rmse_m'),p95_m=value(metrics,'aligned_ATE_p95_m'),horizontal_rmse_m=value(metrics,'aligned_horizontal_rmse_m'),vertical_rmse_m=value(metrics,'aligned_height_rmse_m'),trajectory_coverage=metrics.get('trajectory_coverage'),
      retained_fraction=met.ratio(used,valid_raw) if used is not None else {'status':'UNAVAILABLE'},planned_retained_fraction=met.ratio(used,denominator) if used is not None else {'status':'UNAVAILABLE'},covariance_status=c.read_json(run/'covariance_status.json') if run else {},matched=value(metrics,'matched_count'),unmatched=value(metrics,'unmatched_count'),
      fallback=fallback,window_metrics=metrics.get('window',{'status':'UNAVAILABLE'}),run_directory=str(run) if run else None,cache_id=cell.get('cache_id'),artifacts=artifacts(run),
      range_error_status='UNAVAILABLE_FRAME_ANCHOR_LEVER_TIME_PROVENANCE',trajectory_metrics=metrics)
   table_b.append(b);finaldetails[mode]=b
   if mode in ('all_range','robust_cauchy'):continue
   fixmap={x['segment_id']:x for x in fixed}
   for sid,rr in (segments.items() if segments else [('NONE',[])]):
    fx=fixmap.get(sid,{});sigma=finite(fx.get('sigma_c_local_m')) if fx.get('sigma_available')=='1' else None;delta=finite(fx.get('delta_c_fixed_m'));amp=amps.get(sid)
    sm=met.segment_metric(rr,truthids&planned,start,end,amp)
    if rr and (int(rr[0]['tag_id']),int(rr[0]['anchor_id']))!=link:sm['interval_overlap_s']=0.
    if not injected:sm['signed_error_vs_added_half_m']=None;sm['absolute_error_vs_added_half_m']=None
    ordinal=rr[0]['segment_ordinal'] if rr else '';group=ordinal_group.get(ordinal);sc=scoremap.get(group,{'status':'UNAVAILABLE_NO_SUCCESSFUL_STAGE2_OR_NO_SEGMENT'})
    formula=None; sigma_audit=None
    if fx and sigma is not None and finite(fx.get('c_hat_stage2_m')) is not None:
     ah=float(fx['c_hat_stage2_m']);partial=max(0.,ah-2*sigma);expected=partial if mode=='lcb_partial' else (ah if partial>0 else 0.)
     formula=delta is not None and abs(delta-expected)<=1e-9
     rp=pr/group/'R.mtx' if group else None
     if rp and rp.is_file():
      lines=[x for x in rp.read_text().splitlines() if x and not x.startswith('%')];dims=list(map(int,lines[0].split()));matrix=np.array(list(map(float,lines[1:]))).reshape(dims,order='F')
      cols=sorted([x for x in readrows(pr/group/'key_columns.csv') if x['role']=='GROUP_AMPLITUDE'],key=lambda x:int(x['offset']))
      ix=next((i for i,x in enumerate(cols) if x['key']=='c'+ordinal),None)
      if ix is not None:
       vec=np.zeros(len(cols));vec[ix]=1.;local=math.sqrt(np.linalg.solve(matrix,vec)[ix]);sigma_audit=abs(local-sigma)<=1e-10+1e-7*abs(sigma)
    delivery=met.correction_delivery(delta,fx.get('decision_use')=='1',fx.get('final_use')=='1',fallback.get('attempted',False),cell.get('status')=='COMPLETE')
    final_used=delivery['corrected']
    segment_masks=[x for x in masks if x['segment_id']==sid]
    a=dict(**ident,method=mode,segment_id=sid,segment_ordinal=ordinal,stage1_status=fde.get('status','UNAVAILABLE'),stage2_status=pst.get('segment_refit_status','SUCCESS' if stage2ok else 'UNAVAILABLE'),
      status=b['status'],segment_metrics=sm,detection=detect,Rc_status=sc,group_id=group,sigma_c_local_m=sigma,sigma_status='AVAILABLE_LOCAL_DIAGNOSTIC_ONLY' if sigma is not None else 'UNAVAILABLE',
      proposed_offset_m=delta,formula_verified=formula,sigma_R_solve_verified=sigma_audit,decision_use=fx.get('decision_use'),actual_final_use=fx.get('final_use'),
      actual_fixed_offset_m=delta if final_used else None,proposed_correction_classification=met.correction(delta) if fx else {'status':'UNAVAILABLE'},actual_correction_classification=delivery['actual'],
      proposed_reason=fx.get('reason'),final_used_obs_count=sum(x['final_use']=='1' for x in segment_masks) if masks else None,
      fallback=fallback,run_directory=b['run_directory'],cache_id=b['cache_id'],artifact_identity=b['artifacts'])
    table_a.append(a)
  paired={}
  sup=finaldetails['suppress_all']['rmse_m'];lcb=finaldetails['lcb_partial']['rmse_m']
  if sup is not None and lcb is not None:paired=dict(status='AVAILABLE',delta_rmse_lcb_minus_suppress_m=lcb-sup,improvement_pct=(sup-lcb)/sup*100 if sup else None)
  else:paired=dict(status='UNAVAILABLE_PAIRED_FAILURE',delta_rmse_lcb_minus_suppress_m=None,improvement_pct=None)
  for x in table_b:
   if x['scenario_id']==scenario['scenario_id']:x['paired_lcb_vs_suppress']=paired
  shared={x.get('cache_id') for x in by_mode.values() if x['execution_type']=='FINAL_TRAJECTORY' and x.get('cache_id')}
  paired_fixed=fixedsets.get('lcb_partial')==fixedsets.get('lcb_fixed_full') if len(fixedsets)==2 else None
  intervals={}
  for rr in segments.values():
   lk=(int(rr[0]['tag_id']),int(rr[0]['anchor_id']));ts=[float(x['sensor_time']) for x in rr];intervals.setdefault(lk,[]).append((min(ts),max(ts)))
  total=0;intersection=0
  for lk,ii in intervals.items():
   length,merged=merged_length(ii);total+=length
   if lk==link and injected:intersection=sum(met.overlap(s,e,start,end) for s,e in merged)
  union=total+(end-start if injected else 0)-intersection
  detect['merged_temporal_overlap']={'intersection_s':intersection,'support_duration_s_across_links':total,'truth_duration_s':end-start if injected else 0,'iou':met.ratio(intersection,union)} if fde.get('status')=='SUCCESS' else {'status':'UNAVAILABLE'}
  audit=dict(**ident,same_fixed_recovery_set=paired_fixed,shared_stage2_cache_count=len(shared),shared_stage2_cache_ids=sorted(shared),stage2_content_identity=c.read_json(pr/'stage2_content_identity.json') if pr else {},producer_artifacts=artifacts(pr))
  if paired_fixed is False or len(shared)>1:raise RuntimeError('upstream/fixed-set invariant failed')
  audits.append(audit);results.append(dict(**ident,scenario=scenario,producer=producer,producer_status=pst,fde=fde,detection=detect,paired_lcb_vs_suppress=paired,upstream_audit=audit))
 # Actual clean/injected plan, noise and pre-reference Values audits for every executed pair.
 for item in results:
  if item['condition']!='injected':continue
  clean_id=item['scenario_id'].replace('_injected','_clean')
  a=next((x for x in table_b if x['scenario_id']==clean_id and x['method']=='all_range'),None)
  b=next((x for x in table_b if x['scenario_id']==item['scenario_id'] and x['method']=='all_range'),None)
  if a and b and a.get('run_directory') and b.get('run_directory'):
   check=c.same_initialization(a['run_directory'],b['run_directory']);item['input_pair_audit']=check
   if not check['plan_and_noise_equal'] or not check['pre_raw_lm_values_equal']:raise RuntimeError('injected initial condition differs')
 out=root/'report';out.mkdir(exist_ok=True)
 n.write_json(out/'results.json',dict(schema='nlos_experiment_metrics_v1',baseline=locked['baseline'],locked_manifest_sha256=execution['locked_manifest_sha256'],results=results,table_a=table_a,table_b=table_b,audits=audits))
 flatcsv(out/'table_a.csv',table_a);flatcsv(out/'table_b.csv',table_b)
 # Pair low-v-normal benefits only for identical recording/link/window.
 low_pairs=[]
 for dataset in ('sfuise_walk1','sfuise_walk2','sfuise_walk3'):
  normal=next(x for x in results if x['dataset']==dataset and x['regime']=='normal' and x['condition']=='injected');low=next(x for x in results if x['dataset']==dataset and x['regime']=='low' and x['condition']=='injected')
  nr=normal['paired_lcb_vs_suppress'];lr=low['paired_lcb_vs_suppress'];valid=nr['status']==lr['status']=='AVAILABLE' and normal['scenario']['link']==low['scenario']['link'] and normal['scenario']['window']==low['scenario']['window']
  low_pairs.append(dict(dataset=dataset,status='AVAILABLE' if valid else 'UNAVAILABLE_PAIRED_FAILURE_OR_SKIPPED',normal=nr,low=lr,low_minus_normal_improvement_m=(-lr['delta_rmse_lcb_minus_suppress_m']+nr['delta_rmse_lcb_minus_suppress_m']) if valid else None))
 n.write_json(out/'low_redundancy_pairs.json',dict(pairs=low_pairs))
 def fmt(x):return 'NA' if x is None else f'{x:.6f}'
 text=['# 冻结算法真实数据 NLOS 注入 E2E 结果','',f'实际基线 `{locked["baseline"]}`。本轮仅修改实验输入、调度和评价接口；算法和科学参数冻结。semi-synthetic persistent NLOS-on-real-data，B=0.5 m、目标8 s、seed=911、闭区间；truth只覆盖新增分量，天然总bias未知。','',f'完整证据目录：`{root}`。机器长表：[Table A]({out}/table_a.csv)、[Table B]({out}/table_b.csv)、[完整JSON]({out}/results.json)、[锁定manifest]({root}/locked_manifest.json)。','', '## 选择与准入','']
 for sel in execution['selections']:
  text.append(f'- {sel["dataset"]}: normal link/window `{sel.get("normal_selection")}`；low subset `{sel.get("low_selected",sel.get("low_status"))}`。所有排序候选、失败及排除原因见锁定manifest。')
 for x in results:
  sc=x['scenario']
  if sc['condition']=='clean':text.append(f'- {sc["scenario_id"]}: clean gate `{sc.get("gate",{}).get("passed","SKIPPED")}`；状态 `{sc["status"]}`。')
 text+=['','## Table A：逐段 Detection / Recovery','', '每个scene与候选方法保留零段、失败和跳过行。P/R为正 excess candidate的全planned检测指标；完整双边、temporal、overlap、分母、bias误差、Rc与fallback见长表。FP只相对新增注入分量。','', '| scenario | method | segment | Stage2 | P / R | c_hat (m) | sigma local (m) | proposed offset (m) | final-use | status |','|---|---|---|---|---|---:|---:|---:|---|---|']
 for a in table_a:
  d=a['detection'].get('positive_excess_candidate',{});p=d.get('precision',{}).get('value');rec=d.get('recall',{}).get('value')
  text.append(f'| {a["scenario_id"]} | {a["method"]} | {a["segment_ordinal"] or "NONE"} | {a["stage2_status"]} | {fmt(p)} / {fmt(rec)} | {fmt(a["segment_metrics"]["c_hat_stage2_m"])} | {fmt(a["sigma_c_local_m"])} | {fmt(a["proposed_offset_m"])} | {a["actual_final_use"] or "NA"} | {a["status"]} |')
 text+=['','## Table B：六方法 aligned RMSE (m)','', '| dataset / regime / condition | all_range | robust_cauchy | suppress_all | structured_debias | lcb_fixed_full | lcb_partial | ΔRMSE LCB−suppress | improvement % |','|---|---:|---:|---:|---:|---:|---:|---:|---:|']
 for x in results:
  bb={b['method']:b for b in table_b if b['scenario_id']==x['scenario_id']};p=x['paired_lcb_vs_suppress']
  text.append('| '+x['scenario_id']+' | '+' | '.join(fmt(bb[m]['rmse_m']) if bb[m]['rmse_m'] is not None else 'NA ('+bb[m]['status']+')' for m in n.METHODS)+' | '+fmt(p['delta_rmse_lcb_minus_suppress_m'])+' | '+fmt(p['improvement_pct'])+' |')
 text+=['','定位采用原20ms最近邻、scale=1 SE(3)；窗口指标使用全轨迹alignment。P95、horizontal/vertical、coverage、observations retained、failure/fallback及窗口数值均在Table B长表。','', '## 裁决与限制','']
 detected=[x for x in results if x['condition']=='injected' and x['fde'].get('retained_segment_count',0)>0]
 paired=[x for x in detected if x['paired_lcb_vs_suppress']['status']=='AVAILABLE']
 if not paired:text.append('“同一persistent bias被检测后，LCB是否优于suppression”：证据不足；没有同时满足实际非空temporal support与有效两方法定位输出的配对。')
 else:
  wins=sum(x['paired_lcb_vs_suppress']['delta_rmse_lcb_minus_suppress_m']<0 for x in paired);loss=sum(x['paired_lcb_vs_suppress']['delta_rmse_lcb_minus_suppress_m']>0 for x in paired)
  text.append(f'“同一persistent bias被检测后，LCB是否优于suppression”：{len(paired)}个有效非空support配对，LCB {wins}好/{loss}差/{len(paired)-wins-loss}相同；不据此推断普遍或统计显著收益。')
 valid=[p for p in low_pairs if p['status']=='AVAILABLE']
 if not valid:text.append('“降低冗余是否放大收益”：证据不足；normal/low同recording、link/window的有效收益配对缺失。')
 else:text.append('“降低冗余是否放大收益”：逐recording差值（low收益−normal收益，m）为 '+', '.join(p['dataset']+'='+fmt(p['low_minus_normal_improvement_m']) for p in valid)+'；失败和负收益见完整配对JSON，不从三个recordings推断统计显著性。')
 text.append('本轮所有10个已执行scenario的temporal support均为空；4个injected的正excess检测TP均为0。LCB与suppression的相同轨迹来自原有空support行为，没有实际恢复补偿。唯一有效normal/low配对Walk3的收益均为0，未观察到降低冗余放大收益；检测后恢复优势仍证据不足。')
 text+=['','local sigma只是同Stage2共同Rc的局部诊断，不是校准置信保证或最终bias后验。suppressed/fallback观测不被记为实际补偿；混合段单列support加权新增分量。Range error因原frame/anchor/lever/time provenance缺口为UNAVAILABLE，不以post-fit residual或aligned GT补齐。T10=C2-C、T11=C、C1–C3不升级。sensitivity、新算法、新phase、自动commit/push均NOT_RUN。','', '## 实现、测试与命令证据','', '新增SFUISE原loader导出、闭区间step生成器、v2输入envelope、FDE-only筛选、1800s进程树限制、确定性选择和独立评价/表格生成。原T07半开语义及默认路径保持。','', '修改前baseline.tar.gz/baseline_hashes.json保存源码、配置、runner及已链接依赖；engineering/frozen_algorithm_audit.json核对算法和科学参数。构建与完整CTest（30/30）日志见engineering/。禁用注入与原loader、clean/injected初值/计划/噪声、C++往返、truth字段拒绝、跨scenario replay拒绝与共享上游已实际检查。bwrap隐藏源bag/生成truth目录后，strace筛选通过且未读取truth、未执行Stage2。','', '所有实际命令、退出码、墙钟和日志汇总到report/commands.json；各进程原始.command.json、batch中command.json和run_status均保留。工程首次导出因YAML origin精度损失拒绝，修为17位往返；首次批处理provenance白名单拒绝，未运行estimator；首次新target直接build失败，随后catkin重新配置通过。旧失败目录均保留，不属于算法重试。']
 counts=Counter(b['status'] for b in table_b)
 text+=['','## 暂停恢复与最终计数','',f'用户低电量暂停的producer记录为INTERRUPTED_USER_PAUSE；随后按明确续跑指令从原始输入新建producer，旧尝试保留，不作为算法失败或超时。PAUSED.json、RESUME.json与interrupted_command.json保存完整记录。最终{len(results)}个scenario条目，{sum(bool(x["producer"]) for x in results)}个执行；Table B共{len(table_b)}行，状态计数为{dict(counts)}。另有1次用户中断；方法TIMEOUT计数为{counts.get("TIMEOUT",0)}。所有已完成与算法失败项未重跑。','', '最新独立指标工程测试7/7通过；完整CTest30/30已通过。后补精确planned时间覆盖检查与窗口边界审计均未改变锁定选择、准入或缓存；见engineering/final_results_audit.json、endpoint_selection_audit.json、endpoint_low_audit.json和locked_manifest_regeneration.json。']
 Path(report).write_text('\n'.join(text)+'\n')
 commands=[]
 for p in sorted(root.rglob('*.command.json')):commands.append(dict(artifact=str(p),**c.read_json(p)))
 for p in sorted(root.rglob('command.json')):commands.append(dict(artifact=str(p),**c.read_json(p)))
 for name in ('interrupted_command.json','engineering/engineering_commands.json'):
  p=root/name
  if p.exists():commands.append(dict(artifact=str(p),**c.read_json(p)))
 n.write_json(out/'commands.json',dict(commands=commands))
 print('Wrote',report,len(table_a),'A rows',len(table_b),'B rows')

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--report',required=True);a=p.parse_args();run_report(a.root,a.report)
