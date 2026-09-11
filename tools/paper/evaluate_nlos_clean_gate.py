#!/usr/bin/env python3
"""Independent post-seal clean admission evaluator; selection never imports this."""
import argparse
from pathlib import Path
import yaml
import nlos_injection as n
import nlos_experiment_common as c
import nlos_injection_metrics as m

def main():
 p=argparse.ArgumentParser();p.add_argument('--batch',required=True);p.add_argument('--dataset',required=True);p.add_argument('--output',required=True);a=p.parse_args()
 batch=c.read_json(a.batch);protocol=yaml.safe_load((c.ROOT/'config/paper/ie0911/step2_evaluation.yaml').read_text())['run_units'][a.dataset]
 assert n.sha(Path(protocol['ground_truth']).read_bytes())==protocol['ground_truth_sha256']
 result={};passed=True
 for cell in batch['cells']:
  run=Path(cell['run_directory']) if cell.get('run_directory') else None
  metrics=m.localization(run,protocol) if run else {};rmse=metrics.get('aligned_ATE_rmse_m',{});coverage=metrics.get('trajectory_coverage',{})
  ok=cell['status']=='COMPLETE' and rmse.get('status')=='AVAILABLE' and rmse['value']<1 and coverage.get('value')==1 and metrics.get('complete_planned_trajectory') is True
  result[cell['canonical_mode']]=dict(passed=ok,metrics=metrics,run_directory=str(run),cell_status=cell['status']);passed &= ok
 n.write_json(a.output,dict(passed=passed and set(result)=={'all_range','robust_cauchy'},methods=result,ground_truth_sha256=protocol['ground_truth_sha256'],rule='full planned trajectory and both aligned RMSE < 1 m',batch_sha256=n.sha(Path(a.batch).read_bytes())))
if __name__=='__main__':main()
