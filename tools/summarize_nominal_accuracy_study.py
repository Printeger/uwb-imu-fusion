#!/usr/bin/env python3
"""Produce acceptance gates from online outputs, keeping every failure visible."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def eligible(r):
    return r.get('status')=='SUCCESS' and float(r.get('coverage',0))>=.8


def summarize(evaluation, baseline, runs):
    records=json.loads(evaluation.read_text())['records']
    with baseline.open() as stream:
        old={(r['dataset'],r['sequence']):r for r in csv.DictReader(stream) if r['method']=='current'}
    current={(r['dataset'],r['sequence']):r for r in records}
    newly_failed=[list(k) for k,r in old.items() if eligible(r) and not eligible(current.get(k,{}))]
    valid=[r for r in records if eligible(r)]
    families=[]
    for family in sorted({k[0] for k in old}):
        keys=[k for k in old if k[0]==family]
        paired=[k for k in keys if eligible(old[k]) and eligible(current.get(k,{}))]
        row=dict(dataset=family,total=len(keys),valid=sum(eligible(current.get(k,{})) for k in keys),
                 baseline_valid=sum(eligible(old[k]) for k in keys),paired_sequences=len(paired))
        for metric in ('ape_translation_m_rmse','ape_translation_m_p95','ape_translation_m_max'):
            for name,source in (('before',old),('after',current)):
                row[name+'_'+metric]=float(np.mean([float(source[k][metric]) for k in paired])) if paired else None
        families.append(row)
    walks=[r for r in records if r['dataset']=='SFUISE']
    walk_mean=float(np.mean([r['ape_translation_m_rmse'] for r in walks])) if len(walks)==3 and all(eligible(r) for r in walks) else None
    simulation=[r for r in records if r['dataset']=='simulation']
    star=next(x for x in families if x['dataset']=='starloc')
    totals=[]
    for r in records:
        timing=runs/r['dataset']/r['sequence']/'timing.csv'
        if not timing.exists():continue
        with timing.open() as stream:
            for row in csv.DictReader(stream):
                totals.append(float(row['epoch_total_ms']) if 'epoch_total_ms' in row else
                    float(row['prepare_wall_ms'])+float(row['commit_wall_ms'])+float(row['state_query_wall_ms']))
    p99=float(np.percentile(totals,99)) if totals else None
    gates=dict(full_inventory=len(records)==40 and len(old)==40,
               walks_success_coverage=len(walks)==3 and all(eligible(r) for r in walks),
               walks_equal_weight_ape_le_020=walk_mean is not None and walk_mean<=.2,
               walks_max_le_1=len(walks)==3 and all(eligible(r) and r['ape_translation_m_max']<=1 for r in walks),
               valid_sequences_ge_36=len(valid)>=36,no_new_failure=not newly_failed,
               simulation_ape_le_011=len(simulation)==1 and eligible(simulation[0]) and simulation[0]['ape_translation_m_rmse']<=.11,
               starloc_paired_degradation_le_5pct=star['paired_sequences']==star['baseline_valid'] and
                    star['after_ape_translation_m_rmse']<=1.05*star['before_ape_translation_m_rmse'],
               pooled_epoch_p99_le_40ms=p99 is not None and p99<=40)
    return dict(schema='discrete-nominal-accuracy-acceptance/v1',semantics='online_current_state_raw_frame',
                baseline_sequences=len(old),evaluated_sequences=len(records),valid_sequences=len(valid),
                walk_equal_weight_ape_rmse_m=walk_mean,gates=gates,accuracy_performance_pass=all(gates.values()),
                newly_failed=newly_failed,failures=[r for r in records if not eligible(r)],
                families=families,epoch_count=len(totals),epoch_total_p99_ms=p99,
                epoch_total_max_ms=float(max(totals)) if totals else None,
                deadline_exceedances=sum(t>40 for t in totals))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evaluation',type=Path,required=True)
    p.add_argument('--baseline',type=Path,default=Path('docs/benchmark/sequence_metrics.csv'))
    p.add_argument('--runs',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();summary=summarize(a.evaluation,a.baseline,a.runs)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
