#!/usr/bin/env python3
"""Independent immutable raw-input checks; no FGO solve or truth feedback."""
import json
from pathlib import Path
import sys
import numpy as np
import yaml
from sc8_experiments import CACHE,ROOT,BIN,command,registry_design,sim_config,save


def raw(path):
    result={'imu':[],'uwb':[],'initial':[]}
    for line in path.read_text().splitlines():
        a=line.split()
        if a and a[0] in result:result[a[0]].append(a)
    return result


def check():
    registry=yaml.safe_load((ROOT/'config/sc8_experiments.yaml').read_text())
    dest=CACHE/'input-checks';dest.mkdir(parents=True,exist_ok=True)
    cases={}
    for case in ['SC8-H','SC8-U','SC8-A','SC8-G','SC8-J']:
        entry=dict(stream=case,window=[30,30],requested_method='O-FULL')
        config=sim_config(entry,registry);path=dest/(case+'.yaml');path.write_text(yaml.safe_dump(config))
        audit=dest/(case+'.input')
        assert command([BIN/'sc7_checks','inputs',path,case,140,20261020,audit],dest/(case+'.log'))==0
        cases[case]=raw(audit)
    base=cases['SC8-H']
    for case,data in cases.items():
        assert data['initial']==base['initial']
        assert len(data['imu'])==len(base['imu'])==1401
        assert len(data['uwb'])==len(base['uwb'])==1120
        for i,(a,b) in enumerate(zip(base['imu'],data['imu'])):
            assert a[:4]==b[:4] and a[10:16]==b[10:16] and a[-6:]==b[-6:]
            delta=np.array(list(map(float.fromhex,b[4:10])))-np.array(list(map(float.fromhex,a[4:10])))
            want=np.zeros(6)
            if 1201<=i<1301:
                if case in ['SC8-A','SC8-J']:want[0]=20.
                if case=='SC8-G':want[3]=1.
            np.testing.assert_allclose(delta,want,rtol=0,atol=1e-13)
        for a,b in zip(base['uwb'],data['uwb']):
            assert a[:13]==b[:13] and a[14:16]==b[14:16]
            e,anchor=int(a[1]),int(a[5])
            want=2.25 if case in ['SC8-U','SC8-J'] and 121<=e<124 and anchor==int(base['uwb'][0][5]) else 0.
            np.testing.assert_allclose(float.fromhex(b[13])-float.fromhex(a[13]),want,rtol=0,atol=2e-15)
    # Explicit SC8 UWB representation must preserve the original SC7 fixture.
    for explicit in [False,True]:
        cfg={'raw_base_law':'config/sc7.yaml'}
        if explicit:
            cfg['closure']={'additional_cases':{'SC8-LEGACY':dict(epochs=140,motion='M0',geometry='G0',faults=[[61,0,500,20.,61]],uwb_faults=[[61,0,3,2.25]])}}
        case='SC8-LEGACY' if explicit else 'F0'
        path=dest/(case+'.yaml');path.write_text(yaml.safe_dump(cfg))
        assert command([BIN/'sc7_checks','inputs',path,case,140,20261020,dest/(case+'.input')],dest/(case+'.log'))==0
    assert raw(dest/'F0.input')==raw(dest/'SC8-LEGACY.input')
    save(dest/'result.json',dict(status='PASS',same_standard_draws=True,initial_observation_equal=True,
         cases=list(cases),imu_samples_each=1401,ranges_each=1120,legacy_explicit_raw_bytes_equal=True,
         expected_onset_raw_sample=1201,GT_used_by_consumer=False))
    print('SC8 raw input equivalence and exact overlay checks PASS')

if __name__=='__main__':check()
