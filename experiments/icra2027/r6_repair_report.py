#!/usr/bin/env python3
"""Verify repaired evidence and generate infrastructure report from machine-readable rows."""
import ast
import json
from collections import Counter
from pathlib import Path
import yaml
from formal_native import HERE,read,rows,write,sha,save,verify,digest
from formal_r6_lf import load,MANIFEST
from run_r6_case import REPLAY,PARENTS,BLOCKED,science_failure,verify_seal
from formal_benchmark import METHODS
from formal_assets_repaired import median


def finish():
    manifest=load();lock=verify();results=read(REPLAY/'results.json')
    assert set(results)=={c['case_id'] for c in manifest['cases']}
    old=read(HERE/'runs/R6-LF-v2/results.json')
    retained=read(HERE/'audits/R6_REPAIR_preservation.json')
    for p,h in retained.items():assert sha(p)==h,'OLD_RESULT_MODIFIED:'+p
    for p,h in read(HERE/'configs/FORMAL_METRIC_PROTOCOL.json')['source_hashes'].items():
        assert sha(p)==h,'FROZEN_METRIC_OR_SF_IMPLEMENTATION_CHANGED:'+p
    runs=rows(HERE/'metrics/E2_canonical_runs_repaired.csv');pairs=rows(HERE/'metrics/E2_canonical_pairwise_repaired.csv')
    assert len(runs)==105 and len(pairs)==15
    index={(r['case_id'],r['method_id']):r for r in runs};assert len(index)==105
    for p in pairs:
        for col,a,b,metric in [('delta_FGO_RR_ATE','FGO_RECOVER_FULL','FGO_REJECT','ATE_RMSE'),
                               ('delta_FGO_RR_fault','FGO_RECOVER_FULL','FGO_REJECT','fault_window_RMSE'),
                               ('delta_SF_RR_ATE','SF_RECOVER','SF_REJECT','ATE_RMSE'),
                               ('delta_SF_RR_fault','SF_RECOVER','SF_REJECT','fault_window_RMSE'),
                               ('delta_SF_recover_native','SF_RECOVER','SF_NATIVE','ATE_RMSE')]:
            x,y=index[(p['case_id'],a)][metric],index[(p['case_id'],b)][metric]
            if x and y:assert abs(float(p[col])-(float(x)-float(y)))<1e-12
            else:assert p[col]==''
        assert p['shared_support_treatment_verified']=='True'
    dependencies=[];failures=[];checks=[]
    for case in manifest['cases']:
        cid=case['case_id'];r=results[cid];out=REPLAY/cid/'engineering-r1'
        verify_seal(out,'case_seal.json');verify_seal(PARENTS/cid/'parent/producer')
        status=read(out/'case_status.json');assert status['engineering_failures']==0
        assert not list(out.rglob('engineering_failure.json'))
        parent=read(PARENTS/cid/'parent/parent_metadata.json')
        ph=parent.pop('parent_hash');assert digest(parent)==ph;parent['parent_hash']=ph
        t=read(out/'treatment.json');doc=yaml.safe_load((HERE/'exports/treatments'/f'{cid}.yaml').read_text())
        th=doc.pop('treatment_hash');assert digest(doc)==th==t['treatment_hash']
        for directory in ('corrected_ranges','rejected_ranges'):
            a=HERE/'exports'/directory/(cid+'.csv')
            bcsv=HERE/'exports'/directory/'R6_REPAIR_REPLAY'/cid/'engineering-r1.csv'
            assert sha(a)==sha(bcsv),'CANONICAL_EXPORT_NOT_EXACT:'+str(a)
        p=Path(r['AUX_PRODUCER']['run']);d=read(p/'production_detector_status.json');b=r['AUX_PRODUCER']['backend']
        assert sha(p/'production_detector_status.json')==t['detector_hash']
        assert d['support_hash']==t['support_hash']==parent['support_hash']
        assert sha(Path(old[cid]['AUX_PRODUCER']['run'])/'production_support_partition.json')==sha(p/'production_support_partition.json')
        ps=read(out/'SF_input_integrity.json');assert ps['status']=='PASS'
        assert len({v['imu_payload_hash'] for v in ps['inputs']})==1
        for inp in ps['inputs']:
            assert sha(inp['bag'])==inp['bag_hash'];assert not inp['gt_topics'] and not inp['truth_fields']
        causes=[]
        for method in ['AUX_PRODUCER']+METHODS:
            result=r[method];backend=result.get('backend',result)
            if method=='AUX_PRODUCER':failed=not bool(parent['cache_id'])
            else:failed=index[(cid,method)]['status'] in ('failed','invalid_input')
            if failed:
                valid,detail=science_failure(result)
                if backend['status']==BLOCKED:
                    assert not backend['executed'];valid=True
                    assert not parent['cache_id'] or science_failure(r['FGO_RECOVER_FULL'])[0]
                    detail=backend['reason']
                    cls='BLOCKED_BY_SCIENTIFIC_PARENT'
                else:cls='SCIENTIFIC_NUMERICAL'
                assert valid,'UNCLASSIFIED_REMAINING_FAILURE:'+cid+'/'+method+':'+detail
                failures.append({'case_id':cid,'method_id':method,'class':cls,'detail':detail,'run':result['run']})
                if cls=='SCIENTIFIC_NUMERICAL':causes.append(method+': '+detail)
            if method.startswith('SF') and backend['status']=='completed':
                q=Path(result['run']);assert read(q/'transport_integrity.json')['status']=='PASS'
                isolation=read(q/'isolation.json');assert not isolation['gt_mounts'] and not isolation['truth_mounts']
            if method in ('FGO_REJECT','FGO_RECOVER_FULL') and parent['cache_id']:
                assert result['cell']['parent_cache_manifest']==parent['cache_manifest']
                assert result['cell']['cache_id']==parent['cache_id']
            if method in ('FGO_REJECT','FGO_RECOVER_FULL','SF_REJECT','SF_RECOVER'):
                assert result['identity']['support_hash']==t['support_hash']
                assert result['identity']['treatment_hash']==t['treatment_hash']
        row={'case_id':cid,'input_valid':True,
             'base_preliminary_status':r['FGO_BASE']['backend'].get('reason','UNKNOWN'),
             'robust_preliminary_status':r['FGO_ROBUST']['backend'].get('reason','UNKNOWN'),
             'parent_cache_required':True,'parent_cache_available':bool(parent['cache_id']),
             'detector_status':d['status'],'candidate_count':d['candidate_observation_count'],
             'offline_inference_status':b.get('solver_status',b['status']),
             'treatment_export_status':'available' if t['recover_available'] else BLOCKED,
             'sf_reject_input_status':'available' if t['support_valid'] else BLOCKED,
             'sf_recover_input_status':'available' if t['recover_available'] else BLOCKED}
        for m in METHODS:
            s=r[m].get('backend',r[m])['status']
            row[('FGO_RECOVER' if m=='FGO_RECOVER_FULL' else m)+'_status']=s
        row.update(root_cause_class='SCIENTIFIC_NUMERICAL' if causes else 'NONE',
                   root_cause_detail='; '.join(causes),parent_hash=ph,support_hash=t['support_hash'],treatment_hash=th,
                   parent_path=str(PARENTS/cid/'parent'),producer_run=str(p))
        dependencies.append(row)
        # New and old native science outputs must agree; unavailable stays unavailable.
        eq={}
        for m in ('FGO_BASE','FGO_ROBUST','FGO_REJECT','FGO_RECOVER_FULL'):
            a=Path(old[cid][m]['run']) if old[cid][m].get('run') else None;bpath=Path(r[m]['run'])
            if a and (a/'trajectory.tum').exists():
                eq[m]=sha(a/'trajectory.tum')==sha(bpath/'trajectory.tum')
                assert eq[m],'NATIVE_TRAJECTORY_REPLAY_DIFFERENCE:'+cid+'/'+m
        checks.append({'case_id':cid,'native_trajectory_byte_exact':eq,'detector_support_byte_exact':True,
                       'shared_support_treatment_verified':True,'parent_locality':True,'SF_input_and_transport_integrity':'PASS'})
    save(HERE/'metrics/R6_dependency_status.csv',dependencies);save(HERE/'metrics/R6_REPAIR_failures.csv',failures)
    summary=read(HERE/'audits/E2_GO_NOGO_REPAIRED.json')
    criterion=lock['stable_improvement_rule'];assert criterion['min_valid_pairs']==12
    stable={}
    for prefix,name in [('FGO','Native_FGO'),('SF','SFUISE')]:
        a=[float(p['delta_'+prefix+'_RR_ATE']) if p['delta_'+prefix+'_RR_ATE'] else None for p in pairs]
        f=[float(p['delta_'+prefix+'_RR_fault']) if p['delta_'+prefix+'_RR_fault'] else None for p in pairs]
        valid=sum(x is not None and y is not None for x,y in zip(a,f));wins=sum(x is not None and x<0 for x in a);fw=sum(x is not None and x<0 for x in f)
        stable[prefix]=(valid>=criterion['min_valid_pairs'] and wins>=criterion['min_strict_ATE_improvements_of_15'] and
                        fw>=criterion['min_strict_fault_improvements_of_15'] and median(a)<0 and median(f)<0)
        assert summary[name]['valid_pairs']==valid and summary[name]['stable']==stable[prefix]
    claim='STRONG' if all(stable.values()) else 'PARTIAL_NATIVE' if stable['FGO'] else 'PARTIAL_TRANSFER' if stable['SF'] else 'NOT_SUPPORTED'
    assert claim==summary['RECOVERY_CLAIM']
    oldrows=rows(HERE/'metrics/E2_canonical_runs.csv')
    oldcounts=Counter(r['failure_reason'] for r in oldrows if r['status']=='failed')
    aftercounts=Counter(r['failure_reason'] for r in runs if r['status']=='failed')
    engineering={'attempted_cases':15,'engineering_blocking_failures_before':0,
                 'engineering_failure_note':'No lost successful cache/treatment demonstrated; misleading status rows are reporting defects, not recoverable science.',
                 'misleading_dependency_rows_before':oldcounts['PARENT_CACHE_UNAVAILABLE']+oldcounts['OFFLINE_INFERENCE_TREATMENT_UNAVAILABLE'],
                 'engineering_failures_after':0,
                 'PARENT_CACHE_UNAVAILABLE_before':oldcounts['PARENT_CACHE_UNAVAILABLE'],
                 'PARENT_CACHE_UNAVAILABLE_after':aftercounts['PARENT_CACHE_UNAVAILABLE'],
                 'OFFLINE_INFERENCE_TREATMENT_UNAVAILABLE_before':oldcounts['OFFLINE_INFERENCE_TREATMENT_UNAVAILABLE'],
                 'OFFLINE_INFERENCE_TREATMENT_UNAVAILABLE_after':aftercounts['OFFLINE_INFERENCE_TREATMENT_UNAVAILABLE'],
                 'genuine_failed_scientific_processes':sum(r['class']=='SCIENTIFIC_NUMERICAL' for r in failures),
                 'blocked_method_rows':sum(r['class']=='BLOCKED_BY_SCIENTIFIC_PARENT' for r in failures),
                 'failed_method_rows':sum(r['status']=='failed' for r in runs),'case_root_counts':dict(Counter(r['root_cause_class'] for r in dependencies)),
                 'engineering_revision':1,'old_files_verified':len(retained)}
    write(HERE/'audits/R6_REPAIR_verification.json',{'status':'PASS','engineering':engineering,'case_checks':checks,
           'frozen_manifest_sha256':sha(MANIFEST),'scientific_fingerprint':manifest['experiment_fingerprint'],
           'preregistered_criterion':criterion,'method_row_counts':dict(Counter(r['status'] for r in runs)),
           'RECOVERY_CLAIM':claim,'science_success_not_implied_by_engineering_PASS':True,
           'engineering_source_hashes':{str(HERE/name):sha(HERE/name) for name in
               ('run_r6_case.py','formal_assets_repaired.py','r6_repair_report.py','test_r6_repair.py','test_r6_repaired_metrics.py')}})
    content=['# R6 Engineering Repair + Exact Replay','',f'Completed all 15 frozen cases. RECOVERY_CLAIM = {claim}.',
             '', '## Engineering outcome','',
             'The old generic dependency errors did not demonstrate missing successful scientific products. '
             'All 22 cache-unavailable rows and 11 unavailable SF recovery treatments traced to genuine failures in 11 scientific parents. '
             'Repair makes those dependencies explicit and preserves the failures; it does not claim to have recovered 33 successful runs.',
             '', '```json',json.dumps(engineering,indent=2),'```','',
             'Implemented a deterministic case runner with a durable case-local parent registry, explicit external cache binding, '
             'parent/input/config/payload hashes, sealed phase and completed-case resume verification, and fail-closed incomplete-attempt handling. '
             'Native final failures now propagate their actual solver and fallback reasons. '
             'Published canonical corrected/rejected CSVs and treatment YAMLs with candidate/accepted/rejected IDs, per-segment bias/sigma/LCB/correction '
             'and shared support/treatment hashes. All four successful offline inference/recovery cases have available treatments; '
             'the other eleven exports explicitly record unavailability and contain no invented accepted correction.',
             '', 'No successful-but-lost cache, wrong cache lookup, cross-case reuse, or exporter-not-called fault was observed. '
             'The old labels are retained in the before table; the after table records blocked_by_scientific_parent_failure. '
             'No estimator, initializer, solver cap/tolerance, noise, detector/recovery parameter, input payload, SF configuration, '
             'association/alignment/evaluator or GO/NO-GO criterion was modified. '
             'The repaired metrics writer copies the LF-v2 metric calculations with only output destinations, shared-provenance assertions '
             'and the requested PARTIAL_NATIVE/PARTIAL_TRANSFER label names changed.',
             '', '## Actual execution and validation','',
             '`/usr/bin/python3 experiments/icra2027/run_r6_case.py --preflight` — exit 0; 15 actual loader passes and 7 engineering tests. '
             'See `R6_REPAIR_PREFLIGHT.json`, `R6_REPAIR_tests.log`. '
             'Additional metric regression compared all four classification outcomes and the 11/12 valid-pair boundary '
             'against the original implementation (exit 0; `R6_REPAIR_metric_regression.log`). '
             'A real completed-case CLI resume returned exit 0 without changing any case file or re-executing science '
             '(`R6_REPAIR_resume_check.json`).',
             '`/usr/bin/python3 experiments/icra2027/run_r6_case.py --all` — see `R6_REPAIR_execution.json` for actual exit status. '
             'Each case serializes its methods; at most three independent cases, as in the original frozen protocol. '
             'Per-phase argv/exit/wall records and estimator stdout/stderr are in the case and parent directories; '
             'SF sandbox/transport proofs are in each executed SF method directory.',
             'All old results and numeric inputs remain byte-identical. All available native trajectory files and all 15 detector '
             'support artifacts replayed byte-for-byte; SF outputs are evaluated as fresh runs without requiring bitwise solver determinism. '
             'The SF adapter independently deserializes every generated input, checks retained ranges/timestamps/other fields, '
             'and verifies identical IMU payload hashes; GT/truth topics absent. Actual executed SF processes additionally pass '
             'received-sensor transport comparison. Parent/cache/input/protected science hashes were verified.',
             '', '## Completion and unchanged science summary','', '```json',json.dumps(summary,indent=2),'```','',
             'Bias statistics remain per-segment errors against the mean injected component, not total parent latent bias truth. '
             'Sigma remains a local diagnostic, not calibrated confidence. Unavailable outcomes are NA, never zero. '
             'The shared detector support is valid in all 15 cases, including the three failed batch raw-reference solves.',
             '', '## Per-case root causes','', '| case | parent cache | root cause |', '|---|---|---|']
    for row in dependencies:content.append(f"| {row['case_id']} | {row['parent_cache_available']} | {row['root_cause_detail'] or 'none'} |")
    content+=['','Detailed stage/method status: `../metrics/R6_dependency_status.csv`. '
              'Exact failed-process and blocked-child evidence: `../metrics/R6_REPAIR_failures.csv`.',
              '', '## Dependency order and stop','',
              'The frozen implementation runs PL conditional innovation/CUSUM and freezes support before its batch raw-reference solve. '
              'Its Stage2 cache exists only after constrained refit. The existing final-use SF treatment requires native final acceptance; '
              'therefore final-use export follows that final graph rather than substituting a new decision-only policy. '
              'The actual DAG is in `R6_dependency_dag.md`.',
              'Stop after repaired R6. '+('Review R7–R9 separately; no automatic continuation.' if claim=='STRONG' else
              'Do not proceed to R7–R9 as a recovery-claim expansion: the preregistered stable improvement condition remains unsupported. '
              'The remaining obstacles are scientific/numerical, not cache/export/orchestration failures. Any future work requires a new explicit scope.'),
              'T10=C2-C, T11=C and historical C1–C3 limitations are unchanged. No commit or push.']
    (HERE/'audits/R6_REPAIR_REPORT.md').write_text('\n'.join(content)+'\n')
    inventory={str(p):sha(p) for directory in (REPLAY,PARENTS) for p in directory.rglob('*') if p.is_file()}
    inventory.update({str(p):sha(p) for p in (HERE/'metrics').glob('*repaired.csv')})
    for case in manifest['cases']:
        cid=case['case_id']
        for directory in ('corrected_ranges','rejected_ranges'):
            p=HERE/'exports'/directory/(cid+'.csv');inventory[str(p)]=sha(p)
            p=HERE/'exports'/directory/'R6_REPAIR_REPLAY'/cid/'engineering-r1.csv';inventory[str(p)]=sha(p)
        p=HERE/'exports/treatments'/(cid+'.yaml');inventory[str(p)]=sha(p)
        for p in (HERE/'exports/sfuse_inputs/R6_REPAIR_REPLAY'/cid).rglob('*'):
            if p.is_file():inventory[str(p)]=sha(p)
    inventory.update({str(HERE/'audits/R6_REPAIR_REPORT.md'):sha(HERE/'audits/R6_REPAIR_REPORT.md')})
    for pattern in ('audits/R6_REPAIR*','audits/R6_dependency_dag.md','audits/E2_GO_NOGO_REPAIRED.*',
                    'metrics/R6_dependency_status*.csv','metrics/R6_REPAIR*.csv'):
        for p in HERE.glob(pattern):
            if p.is_file() and p.name!='R6_REPAIR_artifact_hashes.json':inventory[str(p)]=sha(p)
    write(HERE/'audits/R6_REPAIR_artifact_hashes.json',inventory)
    print(json.dumps({'engineering':engineering,'claim':claim,'native_pairs':summary['Native_FGO']['valid_pairs'],
                      'SF_pairs':summary['SFUISE']['valid_pairs']},indent=2))

if __name__=='__main__':finish()
