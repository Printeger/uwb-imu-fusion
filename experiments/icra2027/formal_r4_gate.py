#!/usr/bin/env python3
from pathlib import Path
from formal_native import HERE,read,write,sha,verify,save
from formal_treatment import SF_BINS,TOPICS
from formal_evaluate import evaluate
import rosbag


def main():
    lock=verify();stage=HERE/'runs/R4-transport-ready';state=read(stage/'stage_status.json')
    assert state['status']=='PASS','CLEAN_EQUIVALENCE_FAILED'
    results=read(stage/'results.json');failures=[];metrics=[];sourcehashes={str(p):sha(p) for p in SF_BINS}
    for seq,methods in results.items():
        assert len({r['identity']['config_hash'] for r in methods.values()})==1
        for method,r in methods.items():
            out=Path(r['run'])
            if r['status']=='invalid_input':failures.append((seq,method,'SENSOR_TRANSPORT_LOSS'))
            if r['identity']['input_hash']:
                bag=HERE/'exports/sfuse_inputs'/('clean'+seq[-1])/(method+'.bag')
                assert sha(bag)==r['identity']['input_hash']
                with rosbag.Bag(str(bag)) as b:assert set(b.get_type_and_topic_info().topics)==set(TOPICS)
                iso=read(out/'isolation.json')
                assert not iso['gt_mounts'] and not iso['truth_mounts'] and not iso['support_mounts']
                for p,h in sourcehashes.items():assert iso['readonly'][p]==h
            metric,*_=evaluate(seq,method,r)
            metrics.append(metric)
            if metric['status']=='invalid_input':failures.append((seq,method,metric['failure_reason']))
    save(HERE/'metrics/R4_clean_adapter_metrics.csv',metrics)
    write(stage/'integrity_gate.json',{'status':'FAIL' if failures else 'PASS','failures':failures,'sf_binary_hashes':sourcehashes,
                                      'same_config_per_sequence':True,'measurement_only_topics':True,'frozen_evaluator_coverage_checked':True})
    if failures:raise RuntimeError(str(failures))


if __name__=='__main__':main()
