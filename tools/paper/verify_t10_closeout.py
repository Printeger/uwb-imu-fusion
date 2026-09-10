#!/usr/bin/env python3
"""Read-only checks of retained artifacts and independent numerical recomputation."""
from pathlib import Path
import json
import numpy as np
import t10_closeout as c


def main():
    root=Path(c.obj(c.MASTER)['evidence_root'])
    result=c.obj(root/'RESULTS.json')
    checked=0;score_checks=[];trajectory_checks=[]
    for phase in ('science','los','metadata_regression'):
        for case in sorted((root/phase).iterdir()):
            freeze=c.obj(case/'FREEZE.json')
            for item in freeze['payloads']:
                p=Path(item['path'])
                assert c.sha(p)[7:]==item['sha256'],str(p)
                checked+=1
            meta=c.obj(case/'case.json')
            output=Path(meta.get('output_root',str(case/'output')))
            generation=Path(meta['generation_manifest']).parent
            # Estimator inputs and executable/library identities remain intact.
            if meta['condition']!='A' or phase!='science':
                context=c.obj(case/'context.json')
                for k,p in c.LIBS.items():assert context[k]==c.sha(p),(case,k)
                assert context['runner_sha256']==c.sha(c.RUNNER)
                assert c.obj(case/'RUN_RESULT.json')['truth_open_count']==0
                trace=(case/'file_access.trace').read_text(errors='replace')
                assert not any(x in trace.lower() for x in ('/evaluation/','range_truth','motion.csv','ground_truth'))
            motion=np.genfromtxt(generation/'evaluation/motion.csv',delimiter=',',names=True)
            evaluation=c.obj(case/'evaluation.json')
            for policy,record in evaluation['policies'].items():
                if not record['status']['valid_estimate']:continue
                x=np.loadtxt(output/'final'/policy/'trajectory.tum')
                summary=c.obj(output/'final'/policy/'final_inference_summary.json')
                assert summary['corrected_pseudo_range_count']==0
                assert summary['factor_audit_status']=='OK'
                assert summary['fallback_attempt_count']<=1
                for interval,field in [(None,'trajectory_full'),((3,6),'trajectory_historical_3_6_s')]:
                    selected=x if interval is None else x[(x[:,0]>=3)&(x[:,0]<=6)]
                    truth=np.column_stack([np.interp(selected[:,0],motion['time_s'],motion[name]) for name in ('px_m','py_m','pz_m')])
                    err=np.linalg.norm(selected[:,1:4]-truth,axis=1)
                    values=[float(np.sqrt(np.mean(err**2))),float(np.percentile(err,95))]
                    expected=[record[field][name]['value'] for name in ('rmse_m','p95_m')]
                    assert np.allclose(values,expected,rtol=1e-12,atol=1e-14),(case,policy,field)
                    trajectory_checks.append({'case':case.name,'policy':policy,'field':field,'matches':len(err)})
    for group in result['groups']:
        n=np.array(group['N']);nominal={'A':6400.,'B':1600.,'C':3200.}[group['condition']]
        assert np.array_equal(n,nominal*np.eye(n.shape[0]))
        sigma={'A':.05,'B':.1,'C':.05}[group['condition']]
        assert np.allclose(np.diag(n),np.array(group['segment_observation_counts'])/(sigma*sigma),rtol=1e-14)
        assert np.isclose(group['s_m'],1/np.sqrt(group['lambda_min_R_c']),rtol=1e-12)
        assert np.isclose(group['eta'],group['lambda_min_R_c']/nominal,rtol=1e-12)
        assert group['s_fit_full_gate_differ'] is False
        score_checks.append({'seed':group['seed'],'condition':group['condition'],'N_scale':nominal,
                             'fixed_6400_mapping_error':group['old_fixed_N_eta_mapping_error']})
    rows=result['inputs']
    assert len(rows)==6 and all(r['status']=='SCORED' for r in rows)
    assert sum(r['structured_delta_rmse_m']<0 for r in rows)==1
    assert all(r['structured_delta_p95_m']>0 for r in rows)
    c.write(root/'VERIFIED_RESULTS.json',{'status':'PASS','retained_frozen_payload_checks':checked,
        'freeze_scope':'POST_CRASH_RECONSTRUCTED_RETAINED_ARTIFACTS_NOT_ORIGINAL_COMPLETE_FREEZE',
        'score_checks':score_checks,'trajectory_checks':trajectory_checks,
        'limitations':['No complete GTSAM graph reoptimization/reconstruction in this independent check.','Lost original new-run freeze sidecars disclosed in CRASH_RECOVERY.json.'],
        'C2_disposition':'C2-C','reason':'No repeated trajectory benefit: 1/6 RMSE improvement, 0/6 P95 improvement; full_gate and s_fit decisions identical on all six inputs.'})
    print(checked,'retained hashes;',len(score_checks),'score checks;',len(trajectory_checks),'trajectory metric pairs; C2-C')


if __name__=='__main__':main()
