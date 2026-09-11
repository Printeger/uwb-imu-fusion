#!/usr/bin/env python3
"""Independent SVD check of the six saved T11 fixed-model sparse scores."""
from pathlib import Path
import numpy as np
from t11_prefix import MASTER
from t10_closeout import obj, sha, write, read_rows
from t11_verify import sparse, matrix
from reference_recoverability import compute_recoverability


def main():
    root=Path(obj(MASTER)['evidence_root']);items=[]
    for seed in (20101,20102):
        base=root/'runs'/f'{seed}_H2/output/diagnostic_fixed_model'
        if not (base/'results.csv').exists():continue
        for row in read_rows(base/'results.csv'):
            p=base/row['group']/('H'+row['H']);F=sparse(p/'F.csv');G=matrix(p/'G.csv')
            golden=compute_recoverability(F,G)
            checks={'status':golden['status']==row['status'],
                'N':bool(np.allclose(golden['N'],matrix(p/'N.csv'),rtol=1e-7,atol=1e-10)),
                'Rc':bool(np.allclose(golden['R'],matrix(p/'R.csv'),rtol=1e-7,atol=1e-10)),
                'F_rank':golden['rank']['F_scaled']==int(row['F_rank'])}
            if row['status']=='OK':
                checks.update(eta=bool(np.isclose(golden['eta'],float(row['eta']),rtol=1e-7,atol=1e-10)),
                              s=bool(np.isclose(golden['s_m'],float(row['s_m']),rtol=1e-7,atol=1e-10)))
            items.append({'seed':seed,'H':int(row['H']),'group':row['group'],'checks':checks,
                'golden':{k:golden[k] for k in ('status','N','R','rank','spectrum','eta','s_m')},
                'F_sha256':sha(p/'F.csv'),'G_sha256':sha(p/'G.csv')})
    write(root/'FIXED_GOLDEN_AUDIT.json',{'status':'PASS' if len(items)==6 and all(all(x['checks'].values()) for x in items) else 'NOT_PASSED',
          'scope':'NUMERICAL_VERIFICATION_ONLY_NOT_E2E_OR_CLAIM','cases':items})
    print('fixed golden cases',len(items),[x['checks'] for x in items])

if __name__=='__main__':main()
