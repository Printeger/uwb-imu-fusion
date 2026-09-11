#!/usr/bin/env python3
"""Exact U13 and fixed-linearization verification, no evaluation truth access."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from t11_prefix import MASTER, POLICIES
from t10_closeout import obj, sha, write, read_rows

# Only these provenance fields and measured times are excluded. Numerical
# state/graph hashes remain compared. ID correspondence is recorded separately.
PROVENANCE = {'segment_id','parent_ids','partition_hash','snapshot_hash','discovery_context_hash',
 'input_plan_hash','solver_config_hash','support_partition_sha256','solver_config_sha256',
 'validation_context_sha256','context_sha256','config_sha256','inference_id',
 'linearization_id','decision_linearization_id','recovery_linearization_id','final_linearization_id',
 'implementation_identity','strategy_identity','policy_identity',
 'factor_metadata_sha256','content_identity_sha256'}
TIMING = {'stage1_elapsed_seconds','stage2_elapsed_seconds','elapsed_s','elapsed_seconds','certificate_s','certificate_seconds','certificate_wall_s',
          'timing_seconds','score_elapsed_seconds','total_elapsed_seconds','wall_seconds','added_diagnostics_seconds'}


def normalize(data, ids):
    if isinstance(data,dict):
        return {k:normalize(v,ids) for k,v in data.items() if k not in PROVENANCE|TIMING}
    if isinstance(data,list): return [normalize(v,ids) for v in data]
    if isinstance(data,str):
        for old,new in ids.items(): data=data.replace(old,new)
    return data


def read_normal(path, ids):
    if path.suffix=='.json': return normalize(obj(path),ids)
    if path.suffix=='.csv':
        # Matrix sparse headers are not DictReader records.
        text=path.read_text()
        if text.startswith('rows,'): return text
        return normalize(read_rows(path),ids)
    return path.read_text()


def compare_pair(left,right):
    lp=read_rows(left/'stage1/partition.csv') if (left/'stage1/partition.csv').exists() else []
    rp=read_rows(right/'stage1/partition.csv') if (right/'stage1/partition.csv').exists() else []
    def mapping(rows):
        return {r['segment_id']:'support:'+hashlib.sha256(r['obs_ids'].encode()).hexdigest() for r in rows}
    li,ri=mapping(lp),mapping(rp)
    checks=[]
    # Compare all non-repeated scientific outputs and every call/trial/certificate
    # summary. Large full-output details are checked separately in engineering.
    excluded={'diagnostic_manifest.json','partition_identity.json','manifest.json',
              'validation_status.json','final_content_identity.json'}
    files=set()
    for base in (left,right):
        files.update(str(p.relative_to(base)) for p in base.rglob('*') if p.is_file()
            and p.suffix in ('.json','.csv','.tum','.yaml')
            and p.name not in excluded)
    # Keep certificates, omit full-mode matrix dumps only in compact/full fixture.
    files={f for f in files if not any(x in f for x in ('/certificate_details/','exact_binary64.csv','linear.csv','factors.csv','branches.csv','constants.csv'))}
    for name in sorted(files):
        a,b=left/name,right/name
        if not a.exists() or not b.exists():
            checks.append({'file':name,'pass':False,'reason':'MISSING'});continue
        try:
            av,bv=read_normal(a,li),read_normal(b,ri)
            checks.append({'file':name,'pass':av==bv,'normalized_left_sha256':hashlib.sha256(repr(av).encode()).hexdigest(),
                           'normalized_right_sha256':hashlib.sha256(repr(bv).encode()).hexdigest()})
        except Exception as e: checks.append({'file':name,'pass':False,'reason':str(e)})
    identities=[]
    for policy in POLICIES:
        a,b=left/'final'/policy/'final_content_identity.json',right/'final'/policy/'final_content_identity.json'
        if a.exists() and b.exists():
            x,y=obj(a),obj(b)
            equal=all(x[k]==y[k] for k in ('graph_linearization_sha256','values_sha256','calibration_sha256'))
            checks.append({'file':str(a.relative_to(left)),'pass':equal,'reason':'EXACT_PHYSICAL_GRAPH_VALUES_CALIBRATION'})
            identities.append({'policy':policy,'original':x,'replay':y})
    complete=all((root/'final'/policy/'validation_status.json').exists() and
                 obj(root/'final'/policy/'validation_status.json').get('valid_estimate') is True
                 for root in (left,right) for policy in POLICIES)
    return {'status':'PASS' if complete and checks and all(c['pass'] for c in checks) else 'FAIL' if any(not c['pass'] for c in checks) else 'NOT_RUN_INCOMPLETE_CHAIN',
        'complete_chain':complete,'checks':checks,'segment_identity_correspondence':{'original':li,'replay':ri},
        'final_identities':identities,'excluded_provenance_fields':sorted(PROVENANCE),'excluded_time_fields':sorted(TIMING)}


def u13(root):
    result={'schema':'T11_U13_V1','pairs':[]}
    for seed in (20101,20102):
        left,right=root/'runs'/f'{seed}_H0',root/'runs'/f'{seed}_U13'
        r=compare_pair(left/'output',right/'output');r['seed']=seed
        r['input_bytes_equal']={n:sha(left/'input'/n)==sha(right/'input'/n) for n in ('imu.csv','uwb_observations.csv')}
        r['access_pass']=all(obj(x/'ACCESS_AUDIT.json')['status']=='PASS' for x in (left,right))
        if not all(r['input_bytes_equal'].values()) or not r['access_pass']:r['status']='FAIL'
        result['pairs'].append(r)
    result['status']='PASS' if all(x['status']=='PASS' for x in result['pairs']) else 'NOT_PASSED'
    write(root/'U13_RESULT.json',result)
    print('U13',result['status'],[(p['seed'],p['status'],[c['file'] for c in p['checks'] if not c['pass']][:15]) for p in result['pairs']])
    return result


def matrix(path,shape=None):
    rows=read_rows(path)
    if not rows:return np.zeros(shape or (0,0))
    a=np.zeros(shape or (max(int(r['row']) for r in rows)+1,max(int(r['column']) for r in rows)+1))
    for r in rows:a[int(r['row']),int(r['column'])]=float(r['value'])
    return a


def sparse(path):
    lines=path.read_text().splitlines();shape=tuple(int(x.split(',')[1]) for x in lines[:2]);a=np.zeros(shape)
    for row in csv.DictReader(lines[2:]):a[int(row['row']),int(row['column'])]=float(row['value'])
    return a


def fixed(root):
    details=[];rtol=1e-7;atol=1e-10
    for seed in (20101,20102):
        output=root/'runs'/f'{seed}_H2'/'output';base=output/'diagnostic_fixed_model'
        if not (base/'results.csv').exists():
            details.append({'seed':seed,'status':'UNAVAILABLE','reason':obj(base/'status.json') if (base/'status.json').exists() else 'H2_STAGE2_NOT_REACHED'});continue
        records=read_rows(base/'results.csv')
        if not records:details.append({'seed':seed,'status':'UNAVAILABLE','reason':'NO_APPLICABLE_HISTORY_GROUP'});continue
        for group in sorted({x['group'] for x in records}):
            F=sparse(output/'scoring'/group/'F_whitened.csv');G=matrix(output/'scoring'/group/'G_whitened.csv')
            deps=read_rows(base/group/'factor_dependencies.csv')
            previous=None;checks=[];series=[]
            for h in (0,1,2):
                p=base/group/f'H{h}';r=next(x for x in records if x['group']==group and int(x['H'])==h)
                mapping=[int(x['common_row']) for x in read_rows(p/'row_mapping.csv')]
                expected=[i for d in deps if float(d['end_s'])<=6+h and float(d['sample_max_s'])<=6+h
                          for i in range(int(d['row_offset']),int(d['row_offset'])+int(d['row_count']))]
                f=sparse(p/'F.csv');g=matrix(p/'G.csv');n=matrix(p/'N.csv');rc=matrix(p/'R.csv')
                checks += [mapping==expected,np.array_equal(f,F[mapping]),np.array_equal(g,G[mapping]),r['status'] in ('OK','RANK_DEFICIENT')]
                item={'H':h,'result':r,'N':n.tolist(),'Rc':rc.tolist(),'row_count':len(mapping)}
                if previous is not None and rc.size and previous['R'].shape==rc.shape:
                    delta=rc-previous['R'];tolerance=atol+rtol*max(np.linalg.norm(rc,2),np.linalg.norm(previous['R'],2))
                    eig=float(np.linalg.eigvalsh(delta)[0]);checks += [np.array_equal(n,previous['N']),set(previous['mapping']).issubset(mapping),eig>=-tolerance]
                    item.update(increment_min_eigenvalue=eig,increment_tolerance=tolerance)
                    if r['status']=='OK' and previous['status']=='OK':
                        cov_delta=np.linalg.inv(previous['R'])-np.linalg.inv(rc)
                        cov_tol=atol+rtol*max(np.linalg.norm(np.linalg.inv(previous['R']),2),np.linalg.norm(np.linalg.inv(rc),2))
                        item['covariance_order_min_eigenvalue']=float(np.linalg.eigvalsh(cov_delta)[0]);checks.append(item['covariance_order_min_eigenvalue']>=-cov_tol)
                previous={'R':rc,'N':n,'mapping':mapping,'status':r['status']};series.append(item)
            if all(checks):
                a,b=series[0],series[-1];ra=np.array(a['Rc']);rb=np.array(b['Rc']);lo,hi=np.linalg.eigvalsh(ra)[0],np.linalg.eigvalsh(rb)[0]
                effect=bool(hi-lo>atol+rtol*max(abs(lo),abs(hi)))
            else:effect=False
            details.append({'seed':seed,'group':group,'status':'PASS' if all(checks) else 'FAIL','resolved_effect':effect,'checks':[bool(c) for c in checks],'series':series})
    write(root/'FIXED_MODEL_AUDIT.json',{'groups':details,'status':'PASS' if details and all(x['status']=='PASS' and x['resolved_effect'] for x in details) else 'NOT_PASSED'})
    return details

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['u13','fixed']);a=p.parse_args()
    root=Path(obj(MASTER)['evidence_root']);u13(root) if a.action=='u13' else fixed(root)
