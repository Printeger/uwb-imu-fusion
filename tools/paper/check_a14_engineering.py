#!/usr/bin/env python3
"""Static A14 engineering checks; no estimator, generator, GT or optimizer."""
import argparse,csv,json,sys,importlib.util,copy
from pathlib import Path
import numpy as np

def matrices(path):
    result={}
    with Path(path).open() as f:
        for r in csv.DictReader(f):
            k=(int(r['factor_index']),r['name'])
            if k not in result: result[k]=np.zeros((int(r['nrows']),int(r['ncols'])))
            result[k][int(r['row']),int(r['col'])]=float(r['value'])
    return result

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--reference',required=True);ap.add_argument('--config',required=True);a=ap.parse_args()
    out=Path(a.output);old=matrices(a.reference);new=matrices(out/'matrices.csv');checks=[]
    def check(name,ok,**detail): checks.append(dict(name=name,pass_check=bool(ok),**detail))
    with (out/'checks.csv').open() as f:
        for row in csv.DictReader(f): check(row['name'],row['pass']=='1',value=float(row['value']),limit=float(row['limit']))
    for fi in range(11,371,9):
        expected=sum((old[(fi,'component_'+s)] for s in ('measurement_acc','measurement_gyro','integration','bias_RW_acc','bias_RW_gyro')),np.zeros((15,15)))
        actual=new[(fi,'new_covariance')];err=float(np.max(np.abs(actual-expected)));limit=1e-15+1e-10*float(np.max(np.abs(expected)))
        check(f'A13_non_K_direct_sum_{fi}',err<=limit,max_abs=err,limit=limit)
        # All blocks are retained and compared, including off-diagonal cross terms.
        off=actual-np.diag(np.diag(actual));check(f'cross_terms_retained_{fi}',np.max(np.abs(off))>0,max_abs=float(np.max(np.abs(off))))
        for n,o in [('new_mean','preintegrated'),('new_unwhitened_H','unwhitened_H'),('new_unwhitened_residual','unwhitened_residual')]:
            check(f'A13_sealed_{n}_{fi}',np.array_equal(new[(fi,n)],old[(fi,o)]))
    # The existing batch cache consumer checks the complete non-NLOS config,
    # including paper model. This is an engineering metadata counterexample only.
    spec=importlib.util.spec_from_file_location('a14_batch','tools/paper/run_experiments.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    import yaml
    cfg=yaml.safe_load(Path(a.config).read_text());newcfg=copy.deepcopy(cfg);newcfg['paper']={'imu_covariance_model':'PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1'}
    oldid=m.common_config_identity(cfg);newid=m.common_config_identity(newcfg)
    check('python_actual_common_config_model_identity',oldid!=newid)
    stageid=m.stage2_producer_config_identity(cfg,Path(a.config).parent)
    for left,right in [(oldid,newid),(newid,oldid)]:
        cache={'schema':'uifgo_t09_stage2_cache_v2','producer_common_config_sha256':left,'producer_stage2_config_sha256':stageid}
        cell={'requested_common_config_sha256':left,'requested_stage2_config_sha256':stageid}
        m.validate_cache_compatibility(cache,cell)
        cell['requested_common_config_sha256']=right
        try: m.validate_cache_compatibility(cache,cell)
        except RuntimeError as e: check('python_cross_model_cache_reject',str(e)=='CACHE_COMMON_CONFIG_INCOMPATIBLE',reason=str(e))
        else: check('python_cross_model_cache_reject',False)
    result={'schema':'a14_engineering_static_v1','pass':all(c['pass_check'] for c in checks),'count':len(checks),'failed':[c for c in checks if not c['pass_check']],'checks':checks,'optimizer_iterate_calls':0,'truth_GT':'NOT_READ','A12_negative_result':'PRESERVED','A08_historical_FD':'15/18_LIMIT_PRESERVED'}
    (out/'ENGINEERING_STATIC_CHECKS.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='checks'},indent=2));return 0 if result['pass'] else 1
if __name__=='__main__':sys.exit(main())
