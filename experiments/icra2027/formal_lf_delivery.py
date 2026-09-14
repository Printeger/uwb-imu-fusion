#!/usr/bin/env python3
"""Read-only scientific verification and result-derived delivery for the LF retry."""
import argparse
import json
import subprocess
from collections import Counter
from pathlib import Path
from formal_native import HERE, ROOT, verify, read, rows, write, sha
from formal_r6_lf import MANIFEST, ATTEMPT, STAGE, load


def preserve_check():
    archive=HERE/'attempts/R6-CRLF-invalid'
    ledger=read(archive/'preservation_hashes.json')
    for name, expected in ledger.items():
        original=Path(name)
        preserved=archive/original.relative_to(HERE)
        # Published E2/report paths are versioned copies; old inputs and runs stay in place.
        actual=preserved if preserved.exists() else original
        assert sha(actual)==expected, str(actual)
    return len(ledger)


def seal():
    verify(); load();count=preserve_check()
    command=['/usr/bin/python3','-m','unittest','discover','-s','experiments/icra2027','-p','test_*py']
    result=subprocess.run(command,cwd=ROOT,text=True,capture_output=True)
    (ATTEMPT/'engineering_tests.log').write_text(result.stdout+result.stderr)
    record={'old_preserved_files_verified':count,'test_command':command,'test_exit_code':result.returncode,
            'actual_probe_source_hash':sha(HERE/'cache_parser_probe.cpp'),
            'probe_change_after_manifest':'load-only diagnostic duration 0 -> -1 (full recording); no input/config or estimator modification',
            'initial_failed_probe':'parser_preflight_zero_window_failed.json',
            'parser_pass':'parser_preflight.json','manifest_sha256':sha(MANIFEST),
            'source_hashes':{str(p):sha(p) for p in [HERE/'formal_r6_lf.py',HERE/'formal_assets_lf.py',Path(__file__)]}}
    write(ATTEMPT/'delivery_protocol.json',record)
    assert result.returncode==0


def finish():
    lock=verify();manifest=load();preserved=preserve_check()
    protocol=read(ATTEMPT/'delivery_protocol.json')
    for p,h in protocol['source_hashes'].items():assert sha(p)==h
    results=read(HERE/'runs'/STAGE/'results.json')
    assert set(results)=={c['case_id'] for c in manifest['cases']}
    checks=[]
    for c in manifest['cases']:
        r=results[c['case_id']]
        hashes={r[m]['identity']['config_hash'] for m in ('SF_NATIVE','SF_REJECT','SF_RECOVER')}
        assert hashes=={sha(HERE/f"configs/sfuse/config_test_isas-walk{c['sequence'][-1]}.yaml")}
        for m in ('SF_NATIVE','SF_REJECT','SF_RECOVER'):
            directory=Path(r[m]['run']); inp=HERE/'exports/sfuse_inputs'/c['case_key']/(m+'_input.json')
            if inp.exists():
                metadata=read(inp);assert metadata['integrity']=='PASS'
                assert not metadata['gt_topics'] and not metadata['truth_fields']
                assert sha(metadata['bag'])==metadata['bag_hash']
                isolated=read(directory/'isolation.json')
                assert not isolated['gt_mounts'] and not isolated['truth_mounts']
                if r[m]['status']=='completed':assert read(directory/'transport_integrity.json')['status']=='PASS'
            checks.append({'case_id':c['case_id'],'method_id':m,'status':r[m]['status'],'input_available':inp.exists(),
                           'official_config_hash':next(iter(hashes))})
    data=rows(HERE/'metrics/E2_canonical_runs.csv');assert len(data)==105
    allowed={'completed','no_candidates','fallback','failed','invalid_input'}
    assert all(r['status'] in allowed for r in data)
    for r in data:
        for field in ('git_hash','config_hash','experiment_fingerprint','input_hash','method_id'):assert r[field]
        assert r['experiment_fingerprint']==manifest['experiment_fingerprint']
    pairs=rows(HERE/'metrics/E2_canonical_pairwise.csv');assert len(pairs)==15
    index={(r['case_id'],r['method_id']):r for r in data}
    for p in pairs:
        for col,a,b,metric in [('delta_FGO_RR_ATE','FGO_RECOVER_FULL','FGO_REJECT','ATE_RMSE'),
                               ('delta_FGO_RR_fault','FGO_RECOVER_FULL','FGO_REJECT','fault_window_RMSE'),
                               ('delta_SF_RR_ATE','SF_RECOVER','SF_REJECT','ATE_RMSE'),
                               ('delta_SF_RR_fault','SF_RECOVER','SF_REJECT','fault_window_RMSE'),
                               ('delta_SF_recover_native','SF_RECOVER','SF_NATIVE','ATE_RMSE')]:
            x,y=index[(p['case_id'],a)][metric],index[(p['case_id'],b)][metric]
            if x and y:assert abs(float(p[col])-(float(x)-float(y)))<1e-12
            else:assert p[col]==''
    clean=rows(HERE/'metrics/E1_clean_metrics.csv'); summary=read(HERE/'audits/E2_GO_NOGO.json')
    failure_rows=[r for r in clean+data if r['status'] in ('failed','invalid_input','fallback','no_candidates')]
    claim=summary['RECOVERY_CLAIM']
    next_stage=('R7 robustness sweeps, only after separate user authorization' if claim=='STRONG' else
                'R6 GO/NO-GO claim/evidence review; do not start R7 while stable transferable recovery remains unsupported')
    assets=[]
    for pattern in ('metrics/E*.csv','tables/**/TABLE_I*','figures/**/FIG_2*','configs/**/FROZEN*',
                    'configs/formal_lock.json','configs/FORMAL_METRIC_PROTOCOL.json','manifests/injection_manifest_canonical*',
                    'audits/E2_GO_NOGO*'):
        assets.extend(p for p in HERE.glob(pattern) if p.is_file())
    content=['# R3–R6 execution summary — LF serialization repair and fresh R6', '',
             'R3 PASS; R4 PASS; R5 completed with retained failures; R6 all 15 cases attempted and materialized.', '',
             'RECOVERY_CLAIM = '+claim, '', '## Identity and scope','',
             'Git: `'+lock['git_hash']+'` (dirty snapshot hash-locked).',
             'Frozen scientific parent fingerprint: `'+lock['experiment_fingerprint']+'`.',
             'R6 LF-v2 fingerprint: `'+manifest['experiment_fingerprint']+'`.',
             'Active manifest: `../manifests/injection_manifest_canonical_v2.yaml`, SHA256 `'+sha(MANIFEST)+'`.',
             'The original canonical filename intentionally remains the immutable invalid v1 manifest; v2 supersedes it for this retry.',
             'B0_CURRENT, beta=0, existing PL bidirectional CUSUM, suppress_all/lcb_fixed_full, solver, state/range scheduling, '
             'IMU, evaluator and alignment are unchanged. No B1/B2/B3, tuning, new dataset, TDoA, core math changes, R7–R10, commit or push.',
             '', '## Serialization repair and actual verification','',
             'Only CRLF record endings were changed to LF; numeric spelling, IDs, all sensor timestamps, anchor/window/bias/affected IDs, '
             'and IMU bytes are identical to the old 15 inputs. New cache hashes/IDs and manifest were frozen before methods. '
             'Actual existing C++ library loader passed 15/15; old CRLF regression failed at the expected header. '
             'Initial probe mistakenly requested a zero-duration window; the full-recording diagnostic corrected this without changing inputs. '
             'Both preflight attempts and their build records remain under `../attempts/R6-LF-v2/`.',
             'Previous invalid inputs and runs remain in place; previous published artifacts are preserved under '
             '`../attempts/R6-CRLF-invalid/`. Verified preserved files: '+str(preserved)+'.',
             '', '## R3 and R4','',
             'R3 clean detector candidates: Walk1/2/3 = 0/344/89. Walk2 Stage2 failure is retained; not all clean runs are no-candidates. '
             'Four scientific YAML configs and their hashes remain frozen. R4 measurement-only adapter PASS; identical official per-sequence '
             'SFUISE configs for Native/Reject/Recover. Walk1 clean input equivalence and trajectory tolerance PASS '
             '(position 1 mm, orientation 1 mrad, ATE 1 mm). Previous transport-startup failures are retained. '
             'R6 checks independently verify export integrity and complete forwarded sensor payloads for successful SF runs.',
             'SFUISE static ToA offsets are author-provided dataset-specific values. Repository/paper does not establish independent '
             'calibration-recording provenance; these comparisons use input-parity analysis, not independent calibration provenance.',
             '', '## R5 clean summary (reused, no scientific rerun)','',
             'Common GT samples remain Walk1/2/3 = 227/292/313, same association and scale-1 SE3 alignment. '
             'SFUISE clean is more accurate than frozen native B0; Table I shows this explicitly.',
             '```json',json.dumps(rows(HERE/'tables/csv/TABLE_I_clean_backbone.csv'),indent=2),'```',
             'SF internal used-range/state counts are unavailable and remain NA, not replaced by input or trajectory counts.',
             '', '## R6 canonical summary','', '```json',json.dumps(summary,indent=2),'```',
             'Solver failures are preserved, never retried/tuned. Missing paired outcomes remain NA; zero support is not a substitute for failed inference. '
             'Classification follows the unchanged preregistered >=12 valid pairs, >=10/15 improvements in both ATE/fault, both medians <0 rule. '
             'This is a controlled injected-component benchmark on exposed recordings, not physical NLOS or independent held-out generalization. '
             'Local sigma is a diagnostic, not a calibrated uncertainty guarantee.',
             '', '## All failed / fallback / no-candidate formal rows','', '```json',
             json.dumps([{k:r.get(k) for k in ('case_id','sequence','method_id','status','failure_reason','run')} for r in failure_rows],indent=2),'```',
             '', '## Tables and figures','',
             'Table I: `../tables/csv/TABLE_I_clean_backbone.csv` and LaTeX. '
             'Table II: `../tables/csv/TABLE_II_persistent_faults.csv` and LaTeX. '
             'Figure 2: PDF/PNG plus all CSV source data under `../figures/`. Unavailable methods/signals are labeled rather than invented.',
             '```json',json.dumps(read(HERE/'figures/data/FIG_2_selection.json'),indent=2),'```',
             '', '## Exact next recommended Roadmap stage','',next_stage+'. R6 execution stops here.']
    (HERE/'audits/R3_R6_EXECUTION_SUMMARY.md').write_text('\n'.join(content)+'\n')
    assets.append(HERE/'audits/R3_R6_EXECUTION_SUMMARY.md')
    # Inventory includes isolated logs/configs/hashes and intermediate outputs, not only paper assets.
    for directory in (HERE/'runs'/STAGE, ATTEMPT):
        assets.extend(p for p in directory.rglob('*') if p.is_file())
    for c in manifest['cases']:
        assets.extend(p for p in Path(c['input_manifest']).parent.iterdir() if p.is_file())
        assets.extend(p for p in (HERE/'exports/sfuse_inputs'/c['case_key']).rglob('*') if p.is_file())
        for directory in ('corrected_ranges','rejected_ranges'):
            assets.extend(p for p in (HERE/'exports'/directory).glob(c['case_key']+'*') if p.is_file())
    hashes={str(p.relative_to(HERE)):sha(p) for p in sorted(set(assets))}
    write(HERE/'audits/R3_R6_artifact_hashes.json',hashes)
    write(HERE/'audits/R3_R6_delivery_verification.json',{'status':'PASS','rows':105,'cases':15,
          'formal_status_counts':dict(Counter(r['status'] for r in data)), 'preserved_old_files':preserved,
          'artifact_count':len(hashes),'experiment_fingerprint':manifest['experiment_fingerprint'],'SF_checks':checks,
          'pairwise_exact':True,'protected_scientific_sources_configs_inputs':'PASS','parser_preflight':'PASS_15_OF_15',
          'scientific_success_not_implied_by_delivery_PASS':True})
    print(claim,dict(Counter(r['status'] for r in data)))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--seal',action='store_true');args=parser.parse_args()
    if args.seal:seal()
    else:finish()
