#!/usr/bin/env python3
"""Evaluator-side deterministic manifest creation; no estimator execution."""
import copy
from pathlib import Path
import shutil
import yaml
from formal_native import HERE,WS,verify,read,rows,save,write,sha,digest
from evaluate_clean import ev
from nlos_injection import cache_id

MANIFEST=HERE/'manifests/injection_manifest_canonical.yaml'


def freeze():
    if MANIFEST.exists():raise ValueError('canonical manifest already frozen')
    lock=verify();clean=read(HERE/'runs/R3/results.json');cases=[];checks=[]
    for n in (1,2,3):
        seq=f'Walk{n}';parent=Path(lock['inputs'][n-1]['manifest']);m=read(parent)
        raw=rows(parent.parent/m['uwb_file']);origin=float(m['recording_time_origin_s'])
        anchors=sorted({int(r['anchor_id']) for r in raw});assert anchors==[7475,9524,10548,15155,20276]
        producer=Path(clean[seq]['AUX_PRODUCER']['run']);ledger=rows(producer/'observations.csv')
        byid={o['obs_id']:o for o in ledger};assert set(byid)=={r['obs_id'] for r in raw}
        gt=ev.load_tum(WS/f'res/ie0911_step2_truth_20260911_01/sfuise_walk{n}/ground_truth.tum')
        planned=sorted({float(o['raw_time']) for o in ledger if o['planned']=='1'})
        common=read(HERE/'audits/frame_fits_evaluator_only.json')[seq]['common_gt_times']
        bootstrap_end=planned[4]
        chosen=None
        for offset in range(8,int(max(float(r['sensor_time_s']) for r in raw)-origin)-10):
            start=origin+offset;end=start+10
            g=[g[0] for g in gt if start<=g[0]<end]
            counts={a:sum(o['planned']=='1' and int(o['anchor_id'])==a and start<=float(o['raw_time'])<end for o in ledger) for a in anchors}
            valid=(start>bootstrap_end and len(g)>10 and g[0]-start<.1 and end-g[-1]<.1
                   and max(b-a for a,b in zip(g,g[1:]))<.1 and min(counts.values())>=5
                   and sum(start<=t<end for t in common)>=10)
            checks.append({'sequence':seq,'offset_s':offset,'start':start,'end':end,'bootstrap_end':bootstrap_end,
                           'planned_counts':counts,'gt_samples':len(g),'valid':valid})
            if valid:chosen=(start,end);break
        if chosen is None:raise ValueError('NO_DETERMINISTIC_VALID_WINDOW_'+seq)
        start,end=chosen
        for anchor in anchors:
            spec={'sequence':seq,'anchor_id':anchor,'start_time':start,'end_time':end,'bias_m':1.,'bias_shape':'constant',
                  'interval':'left_closed_right_open','domain':'valid parent ToA observations; protocol-invalid rows unchanged'}
            key='case-'+digest(spec)[:16];directory=HERE/'inputs'/key
            if directory.exists():raise ValueError('input directory exists')
            directory.mkdir(parents=True)
            corrupted=copy.deepcopy(raw);affected=[]
            for old,new in zip(raw,corrupted):
                target=int(old['anchor_id'])==anchor and start<=float(old['sensor_time_s'])<end and byid[old['obs_id']]['valid']=='1'
                if target:
                    new['observed_range_m']=format(float(old['observed_range_m'])+1.,'.17g');affected.append(old['obs_id'])
                    assert .3<=float(new['observed_range_m'])<=60.
                assert {k:v for k,v in old.items() if k!='observed_range_m'}=={k:v for k,v in new.items() if k!='observed_range_m'}
                assert target or old==new
            assert affected
            save(directory/m['uwb_file'],corrupted);shutil.copyfile(parent.parent/m['imu_file'],directory/m['imu_file'])
            child=copy.deepcopy(m);child.update(uwb_sha256='sha256:'+sha(directory/m['uwb_file']),transform_sha256='sha256:'+digest(spec))
            child['cache_id']=cache_id(child);write(directory/'input_manifest.json',child)
            assert sha(directory/m['imu_file'])==sha(parent.parent/m['imu_file'])
            cases.append(dict(spec,case_id=f'{seq}_A{anchor}',case_key=key,affected_obs_ids=affected,
                              clean_parent_hash=sha(parent),corrupted_input_hash=sha(directory/'input_manifest.json'),
                              corrupted_uwb_hash=sha(directory/m['uwb_file']),input_manifest=str(directory/'input_manifest.json'),
                              planned_affected_obs_ids=[oid for oid in affected if byid[oid]['planned']=='1']))
    doc={'schema':'ICRA_CANONICAL_15_CASE_V1','experiment_fingerprint':lock['experiment_fingerprint'],
         'selection':'first integer offset >=8 s passing bootstrap/GT/density checks; no estimator outcomes used',
         'truth_access':'EVALUATOR_ONLY; no recipe/support/GT fields in child measurement cache',
         'primary_case':'Walk1_A10548','fallback_case':'Walk2_A10548','planned_cases':15,'cases':cases}
    MANIFEST.parent.mkdir(parents=True,exist_ok=True);MANIFEST.write_text(yaml.safe_dump(doc,sort_keys=False))
    write(MANIFEST.with_suffix('.lock.json'),{'sha256':sha(MANIFEST),'input_hashes':{c['input_manifest']:c['corrupted_input_hash'] for c in cases},
                                            'window_checks':checks,'frozen_before_any_R6_method':True})
    return doc


def load():
    guard=read(MANIFEST.with_suffix('.lock.json'));assert sha(MANIFEST)==guard['sha256']
    doc=yaml.safe_load(MANIFEST.read_text())
    for p,h in guard['input_hashes'].items():assert sha(p)==h
    return doc
