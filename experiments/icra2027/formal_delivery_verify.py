#!/usr/bin/env python3
"""Verify delivery bookkeeping, NOT scientific validity of the failed R6 inputs."""
from collections import Counter
from pathlib import Path
from formal_native import HERE,read,rows,write,verify,sha
from formal_injection import load,MANIFEST


def main():
    lock=verify();manifest=load();counts={}
    for name,n in [('E1_clean_metrics',21),('E2_canonical_runs',105),('E2_canonical_pairwise',15),('E2_canonical_detection',15)]:
        data=rows(HERE/'metrics'/(name+'.csv'));assert len(data)==n
        for r in data:
            for k in ('git_hash','config_hash','experiment_fingerprint','input_hash','method_id'):assert r[k]
        counts[name]=dict(Counter(r.get('status') for r in data))
    for p,h in read(HERE/'audits/R3_R6_artifact_hashes.json').items():assert sha(HERE/p)==h
    for c in manifest['cases']:
        p=Path(c['input_manifest']);assert sha(p)==c['corrupted_input_hash']
        assert sha(p.parent/read(p)['uwb_file'])==c['corrupted_uwb_hash']
    for p,h in read(HERE/'configs/FORMAL_METRIC_PROTOCOL.json')['source_hashes'].items():assert sha(p)==h
    assert len(manifest['cases'])==15
    assert read(HERE/'runs/R6/stage_status.json')['formal_status']=='invalid_input'
    write(HERE/'audits/R3_R6_delivery_verification.json',{'delivery_integrity':'PASS','R6_scientific_validity':'FAIL_INVALID_INPUT',
          'canonical_manifest_sha256':sha(MANIFEST),'all_frozen_inputs_unchanged':True,'protected_source_configs_unchanged':True,
          'frozen_R6_helper_sources_unchanged':True,'metric_status_counts':counts,
          'summary_sha256':sha(HERE/'audits/R3_R6_EXECUTION_SUMMARY.md'),
          'unit_test_scope':'four existing trajectory-evaluator fixtures and three treatment-export fixtures; not native input-format admission',
          'all_required_identity_fields_present':True,'no_scientific_R6_pass_claimed':True})


if __name__=='__main__':main()
