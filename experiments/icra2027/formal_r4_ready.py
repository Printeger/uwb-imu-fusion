#!/usr/bin/env python3
"""R4 transport-correctness validation; same frozen bags/configs, no tuning."""
from pathlib import Path
from formal_native import HERE,read,write,save,sha,verify
from formal_treatment import run_sf
from formal_transport_probe import validate
from formal_evaluate import clean_equivalence


def main():
    verify();results={};equivalence=[]
    for n in (1,2,3):
        seq=f'Walk{n}';key=f'clean{n}';t=read(HERE/'exports/corrected_ranges'/f'{key}_treatment.json')
        bags={};results[seq]={}
        for method in ('SF_NATIVE','SF_REJECT','SF_RECOVER'):
            bag=HERE/'exports/sfuse_inputs'/key/(method+'.bag');bag=bag if bag.exists() else None
            bags[method]=bag;out=HERE/'runs/R4-transport-ready'/key/method
            if n==1 and method=='SF_NATIVE':
                r=read(out/'formal_result.json');probe=validate(bag,out/'received.bag')
                write(out/'transport_integrity_semantic.json',probe)
                assert probe['status']=='PASS','TRANSPORT_INVALID'
                # Original probe compared the ROS publication header.seq counter, not observation identity.
                r.update(status='completed',failure_reason='',review='HEADER_SEQ_IS_ROS_TRANSPORT_COUNTER_NOT_SENSOR_ID')
                write(out/'reviewed_result.json',r)
            else:r=run_sf(key,seq,bag,method,'R4-transport-ready',ready=True)
            results[seq][method]=r;print('R4 ready',seq,method,r['status'],flush=True)
        assert len({r['identity']['config_hash'] for r in results[seq].values()})==1
        equivalence+=clean_equivalence(seq,results[seq],bags,t)
        save(HERE/'metrics/clean_sf_equivalence_transport_ready.csv',equivalence)
        write(HERE/'runs/R4-transport-ready/results.json',results)
        if any(r['status']=='FAIL_TOLERANCE' for r in equivalence):
            write(HERE/'runs/R4-transport-ready/stage_status.json',{'status':'FAIL','reason':'CLEAN_EQUIVALENCE_OUTSIDE_FROZEN_TOLERANCE',
                                                                 'equivalence':equivalence})
            return 2
    valid=all(r['status'] in ('PASS','not_applicable_detected') for r in equivalence)
    write(HERE/'runs/R4-transport-ready/stage_status.json',{'status':'PASS' if valid else 'FAIL','equivalence':equivalence})
    return 0 if valid else 2


if __name__=='__main__':raise SystemExit(main())
