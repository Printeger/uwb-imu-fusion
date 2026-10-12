#!/usr/bin/env python3
"""Frozen SC3 native joint witnesses; no seed or amplitude search."""
import argparse,csv,os,subprocess,tempfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
BUILD=ROOT.parents[1]/'build/uwb_imu_pl'
BIN=ROOT.parents[1]/'devel/.private/uwb_imu_pl/lib/uwb_imu_pl'
def environment():
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    env['LD_LIBRARY_PATH']=str(BIN.parent)+':'+env.get('LD_LIBRARY_PATH','')
    for name in list(env):
        if name.startswith(('SC1_','SC2_','SC3_')) or name=='HYBRID_SHARED_EPISODE':env.pop(name)
    return env

def native(scene,mode,count,directory,redact=False,compat=False):
    env=environment()
    if compat:env.update(SC1_RANGE_CONTRACTION='1',HYBRID_SHARED_EPISODE='1')
    else:env['SC3_JOINT_RECOVERY']='1'
    if redact:env['SC3_REDACT_SCORING']='1'
    path=directory/(scene+'-'+mode+('-redacted' if redact else '')+'.csv')
    config='config/hybrid_raw.yaml' if compat else 'config/sc3_joint.yaml'
    result=subprocess.run([str(BIN/'epoch_raim_runner'),config,str(path),scene,'hybrid',mode,str(count)],cwd=ROOT,env=env)
    with path.open() as f:rows=list(csv.DictReader(f))
    if result.returncode:
        print('FAIL',scene,mode,'scheduled',count,'committed',len(rows),'attempted',min(count,len(rows)+1),'aborted',1,'unattempted',max(0,count-len(rows)-1),flush=True)
        result.check_returncode()
    assert len(rows)==count and all(r['updates']=='1' for r in rows)
    if compat:
        assert all(r['sc1_valid']=='1' and float(r['sc1_hpl'])<=float(r['sc1_old_hpl']) and float(r['sc1_vpl'])<=float(r['sc1_old_vpl']) for r in rows)
    else:
        assert all(r['qualified']=='1' and r['range_masks']=='9' and r['physical_expected']==r['physical_represented']==r['physical_evaluated']=='481401' and r['formal_eligible']==r['publication_protected']==r['application_available']=='0' for r in rows)
        assert all(float(r['risk_upper'])==2*6421/2**35 for r in rows)
        if not redact:assert all(float(r['horizontal_pe'])<=float(r['hpl']) and float(r['vertical_pe'])<=float(r['vpl']) for r in rows)
        for key in ['hpl','vpl','horizontal_pe','vertical_pe','velocity_error','rotation_error','candidate_ms','bound_ms','full_ms','source_init_ms']:
            vals=np.array([float(r[key]) for r in rows]);vals=vals[np.isfinite(vals)]
            print(scene,mode,'redacted' if redact else '',key,'mean/median/P95/max',('/'.join(f'{x:.8g}' for x in [vals.mean(),np.median(vals),np.percentile(vals,95),vals.max()]) if len(vals) else 'NOT_SCORED'),flush=True)
        for e in [12,15,17,25,26,30]:
            if e<=count:print('SNAPSHOT',scene,mode,{k:rows[e-1][k] for k in ['epoch','status','excluded','imu_bridge','imu_rejoined','history_uwb_removed','hpl','vpl','horizontal_pe','vertical_pe','actions_evaluated','rebuilds']},flush=True)
        for hal,val in [(2,3),(5,5),(10,10)]:print('AL',scene,mode,hal,val,'numeric_pass',sum(float(r['hpl'])<=hal and float(r['vpl'])<=val for r in rows),'application_available=0',flush=True)
    print('BOUND_COUNTS',scene,mode,'redacted' if redact else '',{status:sum(r.get('raw_bound_status',r.get('sc1_status',''))==status for r in rows) for status in sorted({r.get('raw_bound_status',r.get('sc1_status','')) for r in rows})},flush=True)
    print('DENOM',scene,mode,'redacted' if redact else '', 'scheduled=attempted=committed=finite=qualified='+str(count),'aborted=refused=0','late='+str(sum(r['on_time']=='0' for r in rows)),flush=True)
    return rows

def streams(directory):
    controls={}
    for scene in ['SC3_A','SC3_G']:
        for mode in ['off','on']:
            rows=native(scene,mode,30,directory);controls[scene,mode]=rows
            assert rows[14]['imu_alarm']=='1'
            if mode=='on':
                assert all(r['excluded']=='0' and r['imu_bridge']=='1' for r in rows[14:17])
                assert all(r['imu_bridge']=='1' for r in rows[14:25])
                assert all(r['imu_rejoined']=='1' for r in rows[25:])
            else:assert all(r['imu_bridge']=='0' for r in rows)
        keys=['tag_x','tag_y','tag_z','velocity_x','velocity_y','velocity_z','qw','qx','qy','qz']
        assert all(all(a[k]==b[k] for k in keys) for a,b in zip(controls[scene,'off'][:14],controls[scene,'on'][:14]))
    for scene in ['SC3_IA','SC3_IG']:native(scene,'on',30,directory)
    native('SC3_STAGGERED','on',30,directory)
    redacted=native('SC3_A','on',30,directory,redact=True)
    skip={'scene','horizontal_pe','vertical_pe','velocity_error','rotation_error','candidate_ms','bound_ms','full_ms','on_time','source_init_ms'}
    assert all(all(a[k]==b[k] for k in a if k not in skip) for a,b in zip(controls['SC3_A','on'],redacted))
    for scene,count in [('SC3_U1',19),('SC3_U2',21)]:
        rows=native(scene,'on',count,directory)
        assert rows[14]['excluded']=='0' and all(r['imu_bridge']=='0' for r in rows)
    for scene,count in [('U1',19),('U2',21)]:native(scene,'on',count,directory,compat=True)
    print('FROZEN MATRIX PASS: scheduled=attempted=committed=finite=qualified=320; aborted/refused=0; scored=290 redacted=30; exact redacted online invariance',flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['checks','streams']);p.add_argument('--build',action='store_true');args=p.parse_args();env=environment()
    if args.build:subprocess.run(['cmake','--build',str(BUILD),'--target','sc3_joint_checks','epoch_raim_runner','-j2'],env=env,check=True)
    if args.command=='checks':subprocess.run([str(BIN/'sc3_joint_checks')],cwd=ROOT,env=env,check=True)
    else:
        cache=ROOT/'.cache/sc3-joint';cache.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='frozen-',dir=cache) as directory:streams(Path(directory))
if __name__=='__main__':main()
