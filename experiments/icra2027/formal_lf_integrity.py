#!/usr/bin/env python3
"""Post-seal evaluator-side independent raw/treatment/provenance verification."""
from pathlib import Path
import yaml
from formal_native import HERE, read, rows, sha, write, verify
from formal_r6_lf import ATTEMPT, STAGE, load


def main():
    lock=verify();manifest=load();checks=[]
    old=read(HERE/'attempts/R6-CRLF-invalid/audits/R3_R6_artifact_hashes.json')
    clean={name:sha(HERE/name) for name in ('metrics/E1_clean_metrics.csv',
           'tables/csv/TABLE_I_clean_backbone.csv','tables/latex/TABLE_I_clean_backbone.tex')}
    assert all(value==old[name] for name,value in clean.items())
    frozen=yaml.safe_load((HERE/'configs/backbone/FROZEN_BACKBONE.yaml').read_text())
    for case in manifest['cases']:
        parent=Path(lock['inputs'][int(case['sequence'][-1])-1]['manifest'])
        source=Path(case['input_manifest']);pm=read(parent);sm=read(source)
        assert sha(parent)==case['clean_parent_hash']
        assert sha(parent.parent/pm['imu_file'])==sha(source.parent/sm['imu_file'])
        raw=rows(parent.parent/pm['uwb_file']);corrupted=rows(source.parent/sm['uwb_file'])
        assert len(raw)==len(corrupted)
        truth=set(case['affected_obs_ids']);changed=set()
        for a,b in zip(raw,corrupted):
            assert {k:v for k,v in a.items() if k!='observed_range_m'}=={k:v for k,v in b.items() if k!='observed_range_m'}
            if a['observed_range_m']!=b['observed_range_m']:
                changed.add(a['obs_id']);assert int(a['anchor_id'])==case['anchor_id']
                assert case['start_time']<=float(a['sensor_time_s'])<case['end_time']
                assert abs(float(b['observed_range_m'])-float(a['observed_range_m'])-1.)<1e-12
        assert changed==truth
        native=HERE/'runs'/STAGE/case['case_key']
        cfg=yaml.safe_load((native/'config.yaml').read_text())
        assert cfg['dataset']['cache_manifest']==str(source)
        cfg['dataset']['cache_manifest']=frozen['dataset']['cache_manifest']
        assert cfg==frozen
        isolation=list((native/'batch').glob('*.isolation.json'));assert isolation
        for p in isolation:
            i=read(p);assert not i['gt_mounts'] and i['host_home']=='ABSENT' and i['network']=='UNSHARED'
            assert not any('truth' in name.lower() or 'injection_manifest' in name.lower() or 'ground_truth' in name.lower()
                           for name in i['read_only_files'])
        treatment=read(HERE/'exports/corrected_ranges'/(case['case_key']+'_treatment.json'))
        byid={b['obs_id']:b for b in corrupted}
        assert set(byid)=={r['obs_id'] for r in treatment['rows']}
        for r in treatment['rows']:
            assert r['raw_range']==float(byid[r['obs_id']]['observed_range_m'])
            assert r['corrected_range']==r['raw_range']-r['applied_correction']
            if not r['recovery_accepted']:assert r['applied_correction']==0 and r['corrected_range']==r['raw_range']
            else:assert r['candidate'] and r['lcb']>0 and r['applied_correction']==r['estimated_bias']
        checks.append({'case_id':case['case_id'],'status':'PASS','affected_observations':len(changed),
                       'native_isolation_records':len(isolation),'only_cache_path_changed_in_backbone':True,
                       'measurement_treatment_exact':True})
    write(ATTEMPT/'independent_integrity.json',{'status':'PASS','clean_asset_hashes':clean,'cases':checks,
          'evaluator_only_truth_check':True,'source_hash':sha(__file__)})
    print('Independent raw/treatment/native-isolation checks PASS 15/15; clean hashes unchanged')


if __name__=='__main__':main()
