#!/usr/bin/env python3
"""A12 restoration gate and static linear algebra; never solves navigation."""
import csv, json, math, sys
from pathlib import Path
import numpy as np

def rows(p):
    with Path(p).open() as f: return list(csv.DictReader(f))
def mapping(p): return {r['name']:r['value'] for r in rows(p)}
def close(a,b,at=2e-12,rt=2e-12): return math.isclose(float(a),float(b),abs_tol=at,rel_tol=rt)
def write(p,obj): Path(p).write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
def main(root):
    root=Path(root); ref=root/'reference'; out=root/'static'; failures=[]; checks={}
    def check(name,ok):
        checks[name]=bool(ok)
        if not ok: failures.append(name)
    meta=mapping(out/'restoration_identity.csv'); common=json.loads((ref/'common_preparation.json').read_text())
    check('initial_graph_content_exact',meta['initial_graph_sha']==common['graph_linearization_sha256'])
    check('initial_values_content_exact',meta['initial_values_sha']==common['values_sha256'])
    check('counts',int(meta['keys'])==123 and int(meta['factors'])==371 and int(meta['uwb_factors'])==328)
    call=rows(ref/'conditional_lm_calls.csv')[49]; sg=rows(ref/'first_block_stationarity.csv')[49]; actual=rows(out/'checkpoint_stationarity.csv')[0]
    check('checkpoint_error',close(meta['checkpoint_error'],call['error_after']))
    check('base_error',close(meta['base_error'],call['error_before']))
    check('stationarity',all(close(actual[k],sg[k],2e-10,2e-12) for k in actual))
    check('native_retract',float(meta['accepted_retract_max_local'])<=1e-12)
    archived=rows(ref/'conditional_lm_direction_factor_errors.csv'); current=rows(out/'factor_identity.csv')
    check('all_factor_identity_and_error',len(archived)==len(current)==371 and all(a['factor_index']==b['factor_index'] and a['dynamic_type']==b['dynamic_type'] and a['keys']==b['keys'] and close(a['error_at_base'],b['error_at_base']) and close(a['error_at_tentative'],b['error_at_checkpoint']) for a,b in zip(archived,current)))
    previous={(int(r['factor_index']),float(r['step'])):r for r in rows(ref/'conditional_lm_direction_factor_derivatives.csv')}
    previous.update({(-1,float(r['step'])):r for r in rows(ref/'conditional_lm_direction_finite_difference.csv')})
    fd=rows(out/'direction_fd.csv')
    check('actual_direction_complete',len(fd)==1116 and len(previous)==1116)
    check('actual_direction_evidence_reproduced',all(close(r['analytic'],previous[(int(r['factor_index']),float(r['step']))]['gradient_dot_unit_direction'],2e-9,2e-9) and close(r['central'],previous[(int(r['factor_index']),float(r['step']))]['central_derivative'],2e-9,2e-9) for r in fd))
    check('original_fd_criterion',all(r['agrees']=='1' and abs(float(r['central'])-float(r['analytic']))<=5e-9+.005*abs(float(r['analytic'])) for r in fd))
    check('zero_iterate',meta['iterate_calls']=='0')
    restoration={'passed':not failures,'checks':checks,'failures':failures,'meta':meta,'stationarity':actual,'factor_count':len(current),'factor_fd_pass':sum(r['agrees']=='1' for r in fd if r['factor_index']!='-1'),'graph_fd_pass':sum(r['agrees']=='1' for r in fd if r['factor_index']=='-1')}
    write(root/'RESTORATION.json',restoration)
    if failures:
        write(root/'STATIC_AUDIT.json',{'status':'STOP_RESTORATION_NOT_PROVEN','failures':failures,'A':'NOT_RUN','B':'NOT_RUN'})
        return 1
    J=np.loadtxt(out/'whitened_J.csv',delimiter=','); rhs=np.loadtxt(out/'whitened_rhs.csv',delimiter=','); cols=rows(out/'columns.csv'); g=np.array([float(r['gradient']) for r in cols]); delta=np.array([float(r['actual_call50_delta']) for r in cols]); d=np.array([float(r['hessian_diagonal']) for r in cols]);norms=np.linalg.norm(J,axis=0);lam=float(call['lambda_after'])
    assert J.shape==(943,615) and len(cols)==615 and np.all(np.isfinite(J))
    # No normal-equation eigenvalues used for rank: SVD of the physical Jacobian.
    assert np.allclose(norms**2,d,rtol=1e-12,atol=1e-10)
    grad_discrep=float(np.max(np.abs(-J.T@rhs-g)))
    assert grad_discrep<2e-8, grad_discrep
    damp=rows(out/'linked_damping.csv')
    assert len(damp)==1230 and all(close(r['added_diagonal_from_linked_damping_factors'],r['expected'],1e-12,1e-12) for r in damp)
    spectrum={}; weakrows=[]; spectralrows=[]
    for name,field in [('stationarity_physical','stationarity_scale'),('prior_1sigma_physical','prior_scale')]:
        S=np.array([float(r[field]) for r in cols]);Q=J*S;_,sv,Vh=np.linalg.svd(Q,full_matrices=False);threshold=max(1e-12,1e-10*sv[0]);rank=int(np.sum(sv>threshold));cond=float(sv[0]/sv[-1]);modes=[]
        for idx in range(1,6):
            v=Vh[-idx];curv=float(sv[-idx]**2);iso=float(lam*np.sum((S*v)**2));diagonal=float(lam*np.sum(np.clip(d,1e-6,1e32)*(S*v)**2));gp=float((g*S)@v)
            group={k:float(np.sum(v[np.array([r['group']==k for r in cols])]**2)) for k in sorted(set(r['group'] for r in cols))}
            top=[{'key':cols[j]['key'],'coordinate':int(cols[j]['coordinate']),'group':cols[j]['group'],'q_component':float(v[j])} for j in np.argsort(np.abs(v))[-8:][::-1]]
            mode={'weak_index':idx,'singular_value':float(sv[-idx]),'physical_curvature':curv,'isotropic_damping_over_curvature':iso/curv,'diagonal_damping_over_curvature':diagonal/curv,'scaled_gradient_projection':gp,'group_squared_norm':group,'top_coordinates':top,'actual_call50_delta_projection':float((delta/S)@v)};modes.append(mode)
            for j in range(615):weakrows.append([name,idx,j,cols[j]['key'],cols[j]['coordinate'],cols[j]['group'],float(v[j])])
        strongest_iso=float(lam*np.sum((S*Vh[0])**2)/(sv[0]**2))
        spectrum[name]={'scale_by_group':{r['group']:{'value':float(r[field]),'unit':r['unit']} for r in cols},'rank':rank,'rank_threshold':float(threshold),'rank_sensitivity':{str(t):int(np.sum(sv>max(1e-12,t*sv[0]))) for t in (1e-11,1e-10,1e-9)},'sigma_min':float(sv[-1]),'sigma_max':float(sv[0]),'condition_J':cond,'condition_information':cond**2,'column_norm_min':float(np.linalg.norm(Q,axis=0).min()),'column_norm_max':float(np.linalg.norm(Q,axis=0).max()),'column_norm_ratio':float(np.linalg.norm(Q,axis=0).max()/np.linalg.norm(Q,axis=0).min()),'strongest_isotropic_over_curvature':strongest_iso,'weak_modes':modes}
        spectralrows.extend([[name,i,float(s),float(s*s)] for i,s in enumerate(sv)])
    with (out/'physical_spectrum.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['scaling','index_descending','singular_value','physical_information_eigenvalue']);w.writerows(spectralrows)
    with (out/'weak_directions.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['scaling','weak_index','column','key','coordinate','group','dimensionless_component']);w.writerows(weakrows)
    with (out/'column_norms.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['column','key','coordinate','group','native_norm','stationarity_physical_norm','prior_physical_norm','isotropic_damping_over_column_curvature','diagonal_damping_over_column_curvature'])
        for j,r in enumerate(cols):w.writerow([j,r['key'],r['coordinate'],r['group'],norms[j],norms[j],norms[j]*float(r['prior_scale']),lam/d[j],lam*np.clip(d[j],1e-6,1e32)/d[j]])
    grouping={}
    for name in ['X','V','B']+sorted(set(r['group'] for r in cols)):
        mask=np.array([r['key'][0].upper()==name if name in ('X','V','B') else r['group']==name for r in cols]);v=norms[mask]
        grouping[name]={'count':int(mask.sum()),'min':float(v.min()),'median':float(np.median(v)),'max':float(v.max())}
    sp=spectrum['stationarity_physical']; ratios=lam/d
    evidence={'physical_column_ratio_ge100':sp['column_norm_ratio']>=100,'isotropic_column_ratio_spread_ge1e4':float(ratios.max()/ratios.min())>=1e4,'weak_mode_isotropic_ratio_ge0_1':any(m['isotropic_damping_over_curvature']>=.1 for m in sp['weak_modes']),'strong_mode_isotropic_ratio_le1e_4':sp['strongest_isotropic_over_curvature']<=1e-4,'weak_gradient_projection_identifiable':any(abs(m['scaled_gradient_projection'])>1e-8 for m in sp['weak_modes'])}
    result={'status':'STATIC_COMPLETE','restoration_passed':True,'shape':list(J.shape),'gradient_assembly_max_difference':grad_discrep,'column_norm_groups':grouping,'physical_spectra':spectrum,'lambda':lam,'isotropic_column_ratio_min':float(ratios.min()),'isotropic_column_ratio_max':float(ratios.max()),'isotropic_column_ratio_spread':float(ratios.max()/ratios.min()),'linked_damping_matches_formula':True,'clipping_low_count':sum(r['clipped_low']=='1' for r in damp if r['arm']=='B'),'clipping_high_count':sum(r['clipped_high']=='1' for r in damp if r['arm']=='B'),'hypothesis_evidence':evidence,'supports_single_comparison':all(evidence.values()),'information_excludes_damping':True,'A':'NOT_RUN_PENDING_PHASE2_REGISTRATION' if all(evidence.values()) else 'NOT_RUN_STATIC_PREREQUISITE_FAILED','B':'NOT_RUN_PENDING_A_REPRODUCTION' if all(evidence.values()) else 'NOT_RUN_STATIC_PREREQUISITE_FAILED'}
    write(root/'STATIC_AUDIT.json',result)
    print(json.dumps({'restoration_passed':True,'shape':J.shape,'condition_J':sp['condition_J'],'column_ratio':sp['column_norm_ratio'],'hypothesis_evidence':evidence,'supports_single_comparison':all(evidence.values())},indent=2))
    return 0
if __name__=='__main__':sys.exit(main(sys.argv[1]))
