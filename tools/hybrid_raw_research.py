#!/usr/bin/env python3
"""Isolated raw-source/inclusion experiments; no production qualification."""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import tempfile
from collections import Counter
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT.parents[1] / 'build/uwb_imu_pl'
BIN = ROOT.parents[1] / 'devel/.private/uwb_imu_pl/lib/uwb_imu_pl'


def summarize(scene, rows):
    n = len(rows)
    qualified = [r for r in rows if r['finite_bound_qualified'] == '1']
    numeric = [r for r in rows if np.isfinite(float(r['hpl'])) and np.isfinite(float(r['vpl']))]
    print(f'{scene} epochs={n} actual_commit={sum(r["updates"]=="1" for r in rows)} '
          f'finite={len(numeric)} qualified={len(qualified)} rejected_or_unknown={n-len(qualified)} '
          f'healthy_exclusions={sum(r["healthy_excluded"]=="1" for r in rows)} '
          f'correct_exclusions={sum(r["correct"]=="1" for r in rows)} '
          f'timely={sum(r["on_time"]=="1" for r in rows)} '
          f'qualified_PE_over_PL={sum(float(r["horizontal_pe"])>float(r["hpl"]) or float(r["vertical_pe"])>float(r["vpl"]) for r in qualified)}/{len(qualified)}', flush=True)
    print(scene,'status_denominators',dict(Counter(r['status'] for r in rows)))
    for key in ['hpl', 'vpl', 'frozen_hpl', 'frozen_vpl', 'h_ms', 'prediction_ms', 'a_ms', 'b_ms', 'verification_ms', 'commit_ms', 'finalization_ms', 'h_update_ms', 'cleanup_ms', 'full_ms']:
        values = np.array([float(r[key]) for r in rows])
        values = values[np.isfinite(values)]
        print(scene, key, 'mean/p50/p95/max', '/'.join(f'{v:.6g}' for v in [values.mean(),np.median(values),np.percentile(values,95),values.max()]) if len(values) else 'NOT_COMPUTED')
    if rows and rows[0].get('sc1_status')!='DISABLED':
        print(scene,'SC1_status_denominators',dict(Counter(r['sc1_status'] for r in rows)))
        for key in ['sc1_old_hpl','sc1_old_vpl','sc1_hpl','sc1_vpl','range_contraction_ms','sc1_visited','sc1_retained','sc1_frontier','sc1_peak_boxes','history_bytes']:
            values=np.array([float(r[key]) for r in rows]);values=values[np.isfinite(values)]
            print(scene,key,'mean/p50/p95/max','/'.join(f'{v:.8g}' for v in [values.mean(),np.median(values),np.percentile(values,95),values.max()]) if len(values) else 'NOT_COMPUTED')
        for r in rows:
            if r['epoch'] in ['31','50']:
                print(scene,'SC1_SNAPSHOT',{k:r[k] for k in ['epoch','sc1_old_hpl','sc1_old_vpl','sc1_hpl','sc1_vpl','sc1_old_x','sc1_old_y','sc1_old_z','sc1_x','sc1_y','sc1_z','sc1_old_width_x','sc1_old_width_y','sc1_old_width_z','sc1_width_x','sc1_width_y','sc1_width_z','range_contraction_ms']})
    # One immutable trace is evaluated at all pairs: this function has no
    # estimator reference and cannot change retained rows or next H.
    for hal, val in [(2,3),(5,5),(10,10)]:
        before=[dict(r) for r in rows]
        count = sum(float(r['hpl'])<=hal and float(r['vpl'])<=val for r in numeric)
        qcount = sum(float(r['hpl'])<=hal and float(r['vpl'])<=val for r in qualified)
        assert rows==before  # all PL/state/selection/risk/H fields stay fixed
        print(scene, f'diagnostic_AL={hal}/{val} numeric_pass={count}/{n} qualified_pass={qcount}/{n} application_available=0/{n}')
    for r in rows:
        if r['epoch'] in ['15','16','17','18','19','50']:
            print(scene, 'SNAPSHOT', {k:r[k] for k in ['epoch','excluded','committed_retained','finite_bound_qualified','hpl','vpl','bound_risk_upper','initial_support_z','adc_support','initial_bias_support','bias_rw_support','rotation_discretization_support','lever_support','inclusion_center_z','retained_ids']})


def run_streams(scenes, temporary, contract_ranges=False):
    env=dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    env['LD_LIBRARY_PATH']=str(BIN.parent)+':'+env.get('LD_LIBRARY_PATH','')
    env['HYBRID_SHARED_EPISODE']='1'
    env.pop('SC1_RANGE_CONTRACTION', None)
    if contract_ranges: env['SC1_RANGE_CONTRACTION']='1'
    for scene in scenes:
        count=50 if scene in ['U0','U1'] else 21 if scene=='U2' else 20 if scene=='I2' else 5 if scene=='H1' else 3
        path=Path(temporary)/(scene+'.csv')
        subprocess.run([str(BIN/'epoch_raim_runner'),'config/hybrid_raw.yaml',str(path),scene,'hybrid','on',str(count)],cwd=ROOT,env=env,check=True)
        with path.open() as f: rows=list(csv.DictReader(f))
        assert len(rows)==count and all(r['updates']=='1' and r['formal_eligible']=='0' and r['publication_protected']=='0' and r['application_available']=='0' for r in rows)
        if scene in ['U0','U1','U2','H1']:
            assert all(r['finite_bound_qualified']=='1' and r['bound_scope']=='healthy-IMU-single-UWB-episode-50-v1' and float(r['bound_risk_upper'])<=1e-6 for r in rows)
        else:
            assert all(r['finite_bound_qualified']=='0' for r in rows)
        if scene=='U1':
            assert rows[14]['excluded']=='0' and rows[14]['correct']=='1' and rows[14]['candidates']=='9' and rows[14]['committed_retained']=='7' and '1501' not in rows[14]['retained_ids'].split('|')
            assert len(rows[15:19])==4 and all(r['finite_bound_qualified']=='1' for r in rows[15:19])
        if scene=='H1':
            assert rows[2]['excluded']=='-1' and rows[2]['alarm']=='0' and float(rows[3]['history_bound'])>0
        if scene=='U2': assert sum(r['correct']=='1' for r in rows)==3
        if contract_ranges:
            paired = [r for r in rows if r['sc1_valid']=='1']
            if scene in ['U0','U1','U2','H1']:
                assert len(paired)==count
            else: assert not paired
            for r in paired:
                assert r['sc1_source']=='raw-imu-uwb-contracted-v1'
                assert r['sc1_event_risk']==r['bound_risk_upper']
                assert int(r['sc1_considered'])==9 and len(r['sc1_input_ids'].split('|'))==8
                assert float(r['sc1_hpl'])<=float(r['sc1_old_hpl']) and float(r['sc1_vpl'])<=float(r['sc1_old_vpl'])
                assert float(r['horizontal_pe'])<=float(r['sc1_hpl']) and float(r['vertical_pe'])<=float(r['sc1_vpl'])
                for axis in ['x','y','z']:
                    assert float(r['sc1_'+axis])<=float(r['sc1_old_'+axis])
                assert r['hpl']==r['sc1_hpl'] and r['vpl']==r['sc1_vpl']
            if scene in ['U0','U1']:
                for epoch in [31,50]:
                    r=rows[epoch-1]
                    assert float(r['sc1_old_hpl'])-float(r['sc1_hpl'])>1e-8 and float(r['sc1_old_vpl'])-float(r['sc1_vpl'])>1e-8
            if scene=='U2': assert all(r['sc1_valid']=='1' for r in rows[17:21])
        summarize(scene,rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['noise-model','range-check','raw-bound','summarize','evaluate-al'])
    parser.add_argument('--build',action='store_true')
    parser.add_argument('--scene',choices=['U0','U1','U2','U3','I2','H1'])
    parser.add_argument('--contract-ranges',action='store_true',help='SC1 same-commit noisy range contraction, research only')
    parser.add_argument('--csv',type=Path)
    parser.add_argument('--policy',type=Path,default=ROOT/'config/application_al_policy.yaml')
    args=parser.parse_args()
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    env['LD_LIBRARY_PATH']=str(BIN.parent)+':'+env.get('LD_LIBRARY_PATH','')
    if args.build:
        subprocess.run(['cmake','-S',str(ROOT),'-B',str(BUILD)],env=env,check=True)
        subprocess.run(['cmake','--build',str(BUILD),'--target','hybrid_raw_checks','raw_range_checks','epoch_raim_runner','-j2'],env=env,check=True)
    if args.command=='evaluate-al':
        import json
        import yaml
        from application_al_policy import validate,evaluate
        policy=yaml.safe_load(args.policy.read_text())
        print(json.dumps(validate(policy),indent=2))
        if args.csv:
            with args.csv.open() as f: rows=list(csv.DictReader(f))
            available=0
            for original in rows:
                row=dict(original)
                for key in ['hpl','vpl','bound_risk_upper']: row[key]=float(row.get(key,'nan'))
                row['risk_horizon_s']=float(row.get('risk_horizon','nan'))*.05 if row.get('source_law_id')=='raw-source-rw-v1' else None
                row['timeliness_status']=row.get('timeliness_status','UNKNOWN')
                available+=evaluate(policy,row)['application_available']
            print(f'fixed_trace_application_available={available}/{len(rows)}; estimator/retained/risk/H unchanged')
    elif args.command=='range-check':
        subprocess.run([str(BIN/'raw_range_checks')],cwd=ROOT,env=env,check=True)
    elif args.command=='noise-model':
        subprocess.run([str(BIN/'hybrid_raw_checks'),'config/hybrid_raw.yaml'],cwd=ROOT,env=env,check=True)
    elif args.command=='summarize':
        if not args.csv: parser.error('--csv required for summarize')
        with args.csv.open() as f: summarize(args.scene or 'trace',list(csv.DictReader(f)))
    else:
        cache=ROOT/('.cache/sc1-range-contraction' if args.contract_ranges else '.cache/hybrid-raw');cache.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='raw-',dir=cache) as temp:
            run_streams([args.scene] if args.scene else ['U0','U1','U2','H1','U3','I2'],temp,args.contract_ranges)

if __name__=='__main__': main()
