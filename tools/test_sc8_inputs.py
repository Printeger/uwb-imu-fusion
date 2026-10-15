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
        assert command([BIN/'epoch_raim_runner',path,audit,case,'sc6','on',140,'tight-optimized','offline',20261020,'inputs'],dest/(case+'.log'))==0
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
        assert command([BIN/'epoch_raim_runner',path,dest/(case+'.input'),case,'sc6','on',140,'tight-optimized','offline',20261020,'inputs'],dest/(case+'.log'))==0
    assert raw(dest/'F0.input')==raw(dest/'SC8-LEGACY.input')
    save(dest/'result.json',dict(status='PASS',same_standard_draws=True,initial_observation_equal=True,
         cases=list(cases),imu_samples_each=1401,ranges_each=1120,legacy_explicit_raw_bytes_equal=True,
         expected_onset_raw_sample=1201,GT_used_by_consumer=False))
    print('SC8 raw input equivalence and exact overlay checks PASS')



def bag_parity():
    import csv
    from rosbags.highlevel import AnyReader
    from sc8_experiments import common,sha
    summaries=[]
    for rid in ['R1','R2','R3','R4','R5','R6']:
        paths=list((CACHE/'runs').glob('E00-'+rid+'-SFUISE-*/execution.json'))
        paths=[p for p in paths if json.loads(p.read_text())['status']=='SUCCESS']
        assert paths,rid
        path=max(paths,key=lambda p:json.loads(p.read_text()).get('attempt',1))
        execution=json.loads(path.read_text());run=path.parent;cache=ROOT/execution['inputs']
        doc=json.loads((cache/'manifest.json').read_text());primary=doc['primary_tag']
        imu=common.read_csv(cache/'imu.csv');ranges=[r for r in common.read_csv(cache/'uwb.csv') if int(r['tag_id'])==primary]
        groups=[];i=0;window=doc['epoch_batch_window_s']
        while i<len(ranges):
            j=i+1
            while j<len(ranges) and (ranges[j]['source_message']==ranges[i]['source_message'] or float(ranges[j]['t'])-float(ranges[i]['t'])<=window+1e-12):j+=1
            groups.append(ranges[i:j]);i=j
        cfg=yaml.safe_load((run/'sfuise.yaml').read_text());assert all(v==0 for v in cfg['toa_offset'])
        def ns(t):
            v=float(t)*1e9
            return 1000000000000+(int(np.floor(v+.5)) if v>=0 else int(np.ceil(v-.5)))
        actual_imu=[];actual_toa=[];topics=set()
        with AnyReader([run/'input.bag']) as reader:
            for connection,stamp,rawdata in reader.messages():
                topics.add(connection.topic);m=reader.deserialize(rawdata,connection.msgtype)
                if connection.topic.endswith('/imu_ds'):
                    timestamp=int(m.header.stamp.sec)*1000000000+int(m.header.stamp.nanosec)
                    actual_imu.append((timestamp,[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]))
                if connection.topic.endswith('/toa_ds64'):actual_toa.append(list(m.data))
        assert not any('gt' in t or 'truth' in t for t in topics)
        assert len(actual_imu)==len(imu) and len(actual_toa)==len(groups)
        for r,(timestamp,v) in zip(imu,actual_imu):
            assert ns(r['t'])==timestamp
            assert [float(r[k]) for k in common.IMU_FIELDS[1:]]==v
        for group,payload in zip(groups,actual_toa):
            assert int(payload[0])==ns(group[-1]['t']) and int(payload[1])==len(group)
            for i,r in enumerate(group):
                assert payload[2+3*i:5+3*i]==[float(r['anchor_id']),float(r['range']),float(r['valid'])]
        summaries.append(dict(id=rid,imu=len(imu),ranges=len(ranges),groups=len(groups),
                              input_digest=execution['input_digest'],bag_sha256=sha(run/'input.bag'),
                              doubles_equal_exactly=True,canonical_ns_equal_exactly=True,offset_applied_once=True))
    save(CACHE/'input-checks/bag-parity.json',dict(status='PASS',records=summaries,
        async_group_original_timestamps='preserved in canonical input; group-end model is shared, unqualified on real data'))
    print('SC8 six-family lossless TOA/IMU/timestamp/offset physical parity PASS')

if __name__=='__main__':
    if sys.argv[1:]==['bags']:bag_parity()
    else:check()
