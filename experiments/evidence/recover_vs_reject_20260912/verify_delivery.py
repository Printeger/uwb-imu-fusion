"""Read-only independent audit of published development evidence."""
import csv
import json
from pathlib import Path
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[3]
EVIDENCE=Path(__file__).resolve().parent
RUN=Path((EVIDENCE/'run_path.txt').read_text().strip())
PRIVATE=ROOT.parents[1]/'evaluator_private/icra/recover_vs_reject'/RUN.name
OLD=ROOT.parents[1]/'evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/starloc'
metrics=pd.read_csv(RUN/'metrics.csv')
pairs=pd.read_csv(RUN/'paired_differences.csv')
assert len(metrics)==12 and len(pairs)==3
assert metrics[['ATE_RMSE_m','Window_RMSE_m','candidate_segments','coverage','fallback','exit_code']].isna().all().all()
assert pairs[['delta_RR_m','delta_RR_NLOS_m']].isna().all().all()
assert (metrics.status=='NOT_RUN').all()
ledger=json.loads((RUN/'execution.json').read_text())
assert len(ledger['tasks'])==15 and ledger['scientific_processes_started']==0
checks=[]
for name in ('zigzag_s4','loop-3d_s3','zigzag_s3'):
    old=pd.read_csv(OLD/name/'observations.csv.gz')
    new=pd.read_csv(PRIVATE/name/'range_reference.csv')
    assert len(old)==len(new)
    assert np.array_equal(old.time_s,new.time_s)
    assert np.array_equal(old.anchor_id,new.anchor_id)
    maxdiff=float(np.max(np.abs(old.error_m-new.error_m)))
    assert maxdiff<1e-12
    found=[]
    # Independent run-length implementation with pandas group IDs, no events() call.
    for link,d in new.groupby(['tag_id','anchor_id'],sort=True):
        d=d.sort_values('source_row')
        good=np.isfinite(d.error_m)&(d.error_m>.5)
        reset=(~good)|(d.time_s.diff()>1)|(d.time_s.diff()<0)|(~good.shift(1,fill_value=False))
        segment=reset.cumsum()
        for _,v in d[good].groupby(segment[good]):
            if len(v)>=5 and v.time_s.iloc[-1]-v.time_s.iloc[0]>=2:
                found.append((int(link[0]),int(link[1]),float(v.time_s.iloc[0]),float(v.time_s.iloc[-1]),len(v)))
    stored=pd.read_csv(PRIVATE/name/'all_positive_error_events.csv')
    actual=[tuple(v) for v in stored[['tag_id','anchor_id','start_s','end_s','count']].to_numpy()]
    assert found==actual
    checks.append(dict(recording=name,reference_rows=len(new),prior_audit_max_abs_difference_m=maxdiff,all_events=len(found)))
print(json.dumps(dict(status='PASS',method_cells=12,paired_rows=3,accounted_tasks=15,scientific_processes=0,checks=checks),indent=2))
