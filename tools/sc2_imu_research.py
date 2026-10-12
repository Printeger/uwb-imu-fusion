#!/usr/bin/env python3
"""SC2 native IMU-only research, fixed witnesses and isolated temporary traces."""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import tempfile
from collections import Counter
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
BUILD=ROOT.parents[1]/'build/uwb_imu_pl'
BIN=ROOT.parents[1]/'devel/.private/uwb_imu_pl/lib/uwb_imu_pl'

def environment():
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    env['LD_LIBRARY_PATH']=str(BIN.parent)+':'+env.get('LD_LIBRARY_PATH','')
    for name in ['SC2_IMU_RECOVERY','SC2_FDE_ENABLE_EPOCH','SC2_REDACT_SCORING','SC1_RANGE_CONTRACTION','HYBRID_SHARED_EPISODE']:
        env.pop(name,None)
    return env

def native(scene,path,fde='on',count=30,enable_epoch=1,redact=False):
    env=environment();env['SC2_IMU_RECOVERY']='1';env['SC2_FDE_ENABLE_EPOCH']=str(enable_epoch)
    if redact:env['SC2_REDACT_SCORING']='1'
    result=subprocess.run([str(BIN/'epoch_raim_runner'),'config/sc2_imu.yaml',str(path),scene,'hybrid',fde,str(count)],cwd=ROOT,env=env)
    if result.returncode:
        with path.open() as f: partial=list(csv.DictReader(f))
        print(f'FAILED {scene}-{fde}: scheduled={count} committed={len(partial)} attempted={min(count,len(partial)+1)} aborted=1 unattempted={max(0,count-len(partial)-1)}',flush=True)
        result.check_returncode()
    with path.open() as f:rows=list(csv.DictReader(f))
    assert len(rows)==count
    assert all(r['updates']=='1' and r['qualified']=='1' and r['formal_eligible']=='0' and r['publication_protected']=='0' and r['application_available']=='0' and r['retained_current_ids'].count('|')==7 for r in rows)
    assert all(r['risk_upper']==rows[0]['risk_upper'] and float(r['risk_upper'])==2*6421/(2**35) and float(r['bias_box_x'])>0 and float(r['bias_box_gx'])>0 for r in rows)
    if not redact:assert all(float(r['horizontal_pe'])<=float(r['hpl']) and float(r['vertical_pe'])<=float(r['vpl']) for r in rows)
    return rows

def summarize(label,rows):
    n=len(rows);print(label,f'scheduled={n} attempted={n} committed={n} aborted=0 finite={sum(np.isfinite(float(r["hpl"])) and np.isfinite(float(r["vpl"])) for r in rows)} qualified={sum(r["qualified"]=="1" for r in rows)} contracted={sum(r["raw_bound_status"]=="CONTRACTED_VERIFIED" for r in rows)} fallback={sum(r["raw_bound_status"]=="FALLBACK_INERTIAL_VERIFIED" for r in rows)} refused={sum(r["qualified"]=="0" for r in rows)} late={sum(r["on_time"]=="0" for r in rows)}',flush=True)
    print(label,'status',dict(Counter(r['status'] for r in rows)))
    for key in ['hpl','vpl','horizontal_pe','vertical_pe','velocity_error','rotation_error','candidate_ms','range_contraction_ms','full_ms','source_init_ms','prefix_groups']:
        values=np.array([float(r[key]) for r in rows]);values=values[np.isfinite(values)]
        print(label,key,'mean/median/P95/max','/'.join(f'{v:.8g}' for v in [values.mean(),np.median(values),np.percentile(values,95),values.max()]) if len(values) else 'NOT_SCORED')
    for row in rows:
        if row['epoch'] in ['15','17','25','26','27','28','29','30']:
            print(label,'SNAPSHOT',{k:row[k] for k in ['epoch','alarm','possible','status','first_dependency','history_purge','current_bridge','imu_rejoined','qualified','hpl','vpl','horizontal_pe','vertical_pe','velocity_error','rotation_error','rebuilds','old_local_rot','old_local_pos','old_local_vel']})
    for hal,val in [(2,3),(5,5),(10,10)]:
        before=[dict(row) for row in rows]
        count=sum(float(r['hpl'])<=hal and float(r['vpl'])<=val for r in rows)
        assert rows==before
        print(label,f'AL={hal}/{val} numeric_pass={count}/{n} application_available=0/{n}; immutable PL/state/FDE/receipt/risk')

def streams(temp):
    controls={}
    for scene in ['SC2_A','SC2_G']:
        for mode in ['off','on']:
            rows=native(scene,temp/(scene+'-'+mode+'.csv'),mode)
            controls[(scene,mode)]=rows
            assert rows[14]['alarm']=='1' and rows[14]['first_alarm_ns']=='705000000'
            if mode=='on':
                assert all(r['current_bridge']=='1' and r['rebuilds']=='2' for r in rows[14:25])
                assert all(r['imu_rejoined']=='1' and r['qualified']=='1' for r in rows[25:30])
            else:assert all(r['current_bridge']=='0' for r in rows)
            summarize(scene+'-'+mode,rows)
        keys=['tag_x','tag_y','tag_z','velocity_x','velocity_y','velocity_z','qw','qx','qy','qz']
        assert all(all(a[k]==b[k] for k in keys) for a,b in zip(controls[(scene,'off')][:14],controls[(scene,'on')][:14]))
    short=native('SC2_S',temp/'short.csv')
    assert all(r['alarm']=='0' and r['current_bridge']=='0' and r['nominal_possible']=='1' for r in short)
    summarize('SC2_S-weak-negative',short)
    late=native('SC2_A',temp/'late-isolation.csv',enable_epoch=17)
    assert late[14]['alarm']=='1' and late[14]['current_bridge']=='0' and late[15]['current_bridge']=='0' and late[16]['history_purge']=='1'
    assert all(r['imu_rejoined']=='1' for r in late[25:])
    summarize('SC2_A-delayed17',late)
    redacted=native('SC2_A',temp/'redacted.csv',count=30,redact=True)
    exclude={'scene','horizontal_pe','vertical_pe','velocity_error','rotation_error','candidate_ms','range_contraction_ms','full_ms','on_time','source_init_ms'}
    assert all(all(a[k]==b[k] for k in a if k not in exclude) for a,b in zip(controls[('SC2_A','on')],redacted))
    print('REDACTED same input: 30 native decisions/states/PL/candidates/receipts/risk exactly invariant; truth never read for scoring',flush=True)
    env=environment();env['HYBRID_SHARED_EPISODE']='1';env['SC1_RANGE_CONTRACTION']='1'
    path=temp/'U1-compat.csv'
    subprocess.run([str(BIN/'epoch_raim_runner'),'config/hybrid_raw.yaml',str(path),'U1','hybrid','on','19'],cwd=ROOT,env=env,check=True)
    with path.open() as f:rows=list(csv.DictReader(f))
    assert len(rows)==19 and all(r['updates']=='1' and r['sc1_valid']=='1' and float(r['sc1_hpl'])<=float(r['sc1_old_hpl']) and float(r['sc1_vpl'])<=float(r['sc1_old_vpl']) for r in rows)
    assert rows[14]['excluded_measurement']=='1501' and rows[14]['committed_retained']=='7' and all(r['sc1_valid']=='1' for r in rows[15:19])
    print('SC1 U1 compatibility scheduled=19 attempted=19 actual_commits=19 aborted=0 finite=19 qualified=19 contracted=19 fallback=0 refused=0 late='+str(sum(r['on_time']=='0' for r in rows))+' actual_exclusion=1501 four-followup PASS',flush=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['checks','streams'])
    parser.add_argument('--build',action='store_true')
    args=parser.parse_args()
    env=environment()
    if args.build:
        subprocess.run(['cmake','-S',str(ROOT),'-B',str(BUILD)],env=env,check=True)
        subprocess.run(['cmake','--build',str(BUILD),'--target','sc2_imu_checks','epoch_raim_runner','-j2'],env=env,check=True)
    if args.command=='checks':subprocess.run([str(BIN/'sc2_imu_checks')],cwd=ROOT,env=env,check=True)
    else:
        cache=ROOT/'.cache/sc2-imu';cache.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='frozen-',dir=cache) as directory:streams(Path(directory))
if __name__=='__main__':main()
