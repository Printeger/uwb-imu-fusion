#!/usr/bin/env python3
"""Canonical single-case smoke; no amplitude/duration/anchor sweep arguments."""
import argparse
import datetime as dt
from pathlib import Path
from types import SimpleNamespace
import uuid
import yaml
import run_walk1_smoke as h
import canonical_injection as inj


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--estimate-only',action='store_true',help='Produce sealed artifacts; evaluate separately')
    a=p.parse_args()
    identity='controlled-'+dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:12]
    out=h.ROOT/'experiments/results'/identity;out.mkdir()
    private=h.ROOT.parents[1]/'evaluator_private/icra/controlled'/identity
    child=inj.generate(out/'inputs/child',private)
    h.write(out/'experiment.json',{'schema':'ICRA_CONTROLLED_SMOKE_V1','identity':identity,'private_dir':str(private),
                                'start_time':h.now(),'status':'RUNNING','estimator_core_modified':False})
    codes={}
    for name,manifest in [('clean',inj.PARENT),('corrupted',child)]:
        batch=out/name;batch.mkdir()
        cfg=yaml.safe_load(h.CONFIG.read_text());cfg['dataset']['cache_manifest']=str(manifest)
        config=batch/'input.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=False))
        try:codes[name]=h.run_batch(batch,SimpleNamespace(runner=h.RUNNER),config,'isas_walk1_'+name)
        except Exception as exc:
            h.write(batch/'infrastructure_failure.json',{'reason':f'{type(exc).__name__}: {exc}','time':h.now()})
            if not (batch/'lock.json').exists():
                h.write(batch/'lock.json',{'git_commit':'UNAVAILABLE','config_sha256':h.sha(config),'evaluator_protocol_hash':h.digest(h.PROTOCOL)})
            codes[name]=h.aggregate(batch,h.now(),h.now(),2)
    inj.validate_child(child)
    truth=h.read(private/'injection_truth.json')
    if h.sha(h.GT)!=truth['gt_sha256']:raise ValueError('GT changed')
    d=h.read(out/'experiment.json');d.update(status='ESTIMATOR_ARTIFACTS_SEALED',end_time=h.now(),batch_exit_codes=codes)
    h.write(out/'experiment.json',d)
    rows=[];metrics=[]
    for name in codes:
        for filename,target in [('runs.csv',rows),('trajectory_metrics.csv',metrics)]:
            import csv
            for row in csv.DictReader((out/name/filename).open()):
                row.update(condition=name,recording_variant_id='ISAS-Walk1' if name=='clean' else truth['child_recording_id']);target.append(row)
    h.save_csv(out/'runs.csv',rows);h.save_csv(out/'trajectory_metrics.csv',metrics)
    print('CONTROLLED_OUTPUT='+str(out),flush=True)
    if not a.estimate_only:
        from evaluate_controlled_smoke import evaluate
        return evaluate(out)
    return int(any(codes.values()))


if __name__=='__main__':raise SystemExit(main())
