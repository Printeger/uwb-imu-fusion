#!/usr/bin/env python3
import copy
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest
import numpy as np
import evaluate_range_gt as e
from estimator_isolation import sandbox_command, measurement_files


def observation(t=0.01):
    return dict(obs_id='9007199254740993',raw_time=str(t),raw_z_m='3.5',raw_tag_id='27956',anchor_id='7475',planned='1',valid='1')


def geometry():
    I=np.eye(4)
    gt=[(0.,np.array([1.,0.,0.]),np.array([0.,0.,0.,1.])),
        (0.02,np.array([1.,0.,0.]),np.array([0.,0.,0.,1.]))]
    d={'tag_id':'27956','anchors_m':{'7475':[0,0,0]},'static_bias_m':{'7475':0.5}}
    return d,(I.copy(),I.copy(),np.array([1.,0.,0.]),gt)


class RangeTests(unittest.TestCase):
    def test_raw_sign_and_tag_lever(self):
        d,g=geometry();r=e.evaluate_rows([observation()],'M0','run',d,g,[])[0]
        self.assertEqual(r['gt_range'],2.)
        self.assertEqual(r['raw_range_error'],1.)
        self.assertEqual(r['corrected_range_error'],1.)
        self.assertEqual(r['obs_id'],'9007199254740993')
        self.assertIsNone(r['detector_candidate'])

    def test_rotating_marker_and_frame_chain(self):
        d,g=geometry();A,M,l,gt=g
        A[:3,3]=[2,0,0];M[:3,3]=[0,1,0]
        q=np.array([0,0,math.sin(math.pi/4),math.cos(math.pi/4)])
        gt=[(t,p,q) for t,p,_ in gt]
        r=e.evaluate_rows([observation()],'M0','run',d,(A,M,l,gt),[])[0]
        # marker p=(1,0); rotation maps IMU translation (0,1) to (-1,0)
        # and tag lever (1,0) to (0,1); anchor translation (2,0) -> (2,1)
        self.assertAlmostEqual(r['gt_range'],math.sqrt(5))

    def test_gap_exact_and_no_extrapolation(self):
        gt=[(0.,np.zeros(3),[0,0,0,1]),(0.1,np.ones(3),[0,0,0,1])]
        self.assertEqual(e.associate(0.05,gt,[0,.1])[-1],'GT_GAP_EXCEEDED')
        self.assertEqual(e.associate(0.,gt,[0,.1])[-1],'AVAILABLE')
        self.assertEqual(e.associate(-.01,gt,[0,.1])[-1],'OUTSIDE_GT_SUPPORT')
        gt[1]=(0.05,np.ones(3),[0,0,0,-1])
        self.assertEqual(e.associate(.025,gt,[0,.05])[-1],'AVAILABLE')

    def candidate(self):
        obs=observation();mask={'candidate':'1','segment_id':'s','decision_use':'1','final_use':'1','fallback_use':'0'}
        segment={'tag_id':'27956','anchor_id':'7475','amplitude_m':'1'}
        comp={'tag_id':'27956','anchor_id':'7475','c_hat_stage2_m':'1','delta_c_fixed_m':'1','decision_use':'1','final_use':'1'}
        return obs,{obs['obs_id']:mask},{'s':comp},{'s':segment}

    def test_actual_compensation_sign(self):
        d,g=geometry();o,m,c,s=self.candidate()
        r=e.evaluate_rows([o],'M4','run',d,g,[],m,c,s)[0]
        self.assertEqual(r['corrected_range'],2.5)
        self.assertEqual(r['corrected_range_error'],0.)
        self.assertTrue(r['recovery_accepted'])

    def test_fallback_disables_correction(self):
        d,g=geometry();o,m,c,s=self.candidate();m[o['obs_id']]['fallback_use']='1'
        r=e.evaluate_rows([o],'M4','run',d,g,[],m,c,s)[0]
        self.assertEqual(r['applied_correction'],0.)
        self.assertFalse(r['recovery_accepted'])
        self.assertEqual(r['estimated_bias'],1.)

    def test_suppression_keeps_measurement(self):
        d,g=geometry();o,m,c,s=self.candidate();m[o['obs_id']].update(decision_use='0',final_use='0')
        r=e.evaluate_rows([o],'M3','run',d,g,[],m,c,s)[0]
        self.assertEqual(r['measured_range'],3.5)
        self.assertEqual(r['applied_correction'],0.)

    def test_missing_source_never_zero_filled(self):
        d,g=geometry();r=e.evaluate_rows([observation()],'M0','run',d,None,['T_marker_IMU'])[0]
        self.assertIsNone(r['gt_range']);self.assertIsNone(r['raw_range_error'])
        summary=e.summarize([r]);self.assertTrue(all(x['sample_count']==0 and x['RMSE'] is None for x in summary))

    def test_duplicate_and_missing_mask_rejected(self):
        d,g=geometry();o=observation()
        with self.assertRaises(ValueError):e.evaluate_rows([o,o],'M0','run',d,g,[])
        with self.assertRaises(ValueError):e.evaluate_rows([o],'M4','run',d,g,[])

    def test_cross_link_and_nan_rejected(self):
        d,g=geometry();o,m,c,s=self.candidate();s['s']['anchor_id']='other'
        with self.assertRaises(ValueError):e.evaluate_rows([o],'M4','run',d,g,[],m,c,s)
        with self.assertRaises(ValueError):e.number('NaN')
        with self.assertRaises(ValueError):e.quaternion([0,0,0,0])

    def test_signed_median_absolute_p95(self):
        rows=[]
        for v in [-3,-1,2]:rows.append({'run_id':'r','method':'M0','anchor_id':'a','segment_id':'',
                                      'raw_range_error':v,'corrected_range_error':v})
        m=e.summarize(rows)[0]
        self.assertEqual(m['median'],-1);self.assertEqual(m['MAE'],2)
        self.assertAlmostEqual(m['P95'],2.9);self.assertEqual(m['sample_count'],3)

    def test_tdoa_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'calibration.json'
            p.write_text(json.dumps({'schema':'isas_walk1_range_calibration_v1','recording_id':'ISAS-Walk1','uwb_measurement_type':'TDoA'}))
            with self.assertRaises(ValueError):e.calibration(p)

    def test_calibration_full_sources_and_no_test_fit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'survey.txt';source.write_text('Independent calibration fixture')
            gt=root/'gt.tum';gt.write_text('0 0 0 0 0 0 0 1\n0.02 1 0 0 0 0 0 1\n')
            d={'schema':'isas_walk1_range_calibration_v1','recording_id':'ISAS-Walk1',
               'uwb_measurement_type':'ABSOLUTE_TOA','association':e.ASSOCIATION,
               'T_anchor_GT':np.eye(4).tolist(),'T_marker_IMU':np.eye(4).tolist(),
               'lever_IMU_tag_m':[1,0,0],'anchors_m':{'7475':[0,0,0]},'static_bias_m':{'7475':0.2},
               'GT_time_to_sensor':{'scale':1,'offset_s':0},'gt_path':str(gt),'gt_sha256':e.sha(gt)}
            d['sources']={k:{'independent':True,'recording_id':'calibration-fixture',
                              'path':str(source),'sha256':e.sha(source)} for k in
                          ['T_anchor_GT','T_marker_IMU','lever_IMU_tag_m','anchors_m','static_bias_m','GT_time_to_sensor']}
            p=root/'cal.json';p.write_text(json.dumps(d));self.assertEqual(e.calibration(p)[2],[])
            assumed=copy.deepcopy(d)
            assumed['sources']['T_anchor_GT'].update(origin='USER_AUTHORIZED_ASSUMPTION',independent=False,fit_on_evaluation_data=False)
            p.write_text(json.dumps(assumed))
            with self.assertRaises(ValueError):e.calibration(p)
            loaded,geo,missing=e.calibration(p,True)
            self.assertEqual(missing,[])
            self.assertEqual(loaded['reference_status'],'ASSUMED_GEOMETRY_AND_CLOCK')
            loaded['tag_id']='27956'
            row=e.evaluate_rows([observation()],'M0','r',loaded,geo,[])[0]
            self.assertEqual(row['reference_status'],'ASSUMED_GEOMETRY_AND_CLOCK')
            self.assertEqual(e.summarize([row])[0]['assumed_fields'],'T_anchor_GT')
            assumed['T_anchor_GT'][0][3]=1
            p.write_text(json.dumps(assumed))
            with self.assertRaises(ValueError):e.calibration(p,True)
            assumed=copy.deepcopy(d)
            assumed['sources']['static_bias_m'].update(origin='USER_AUTHORIZED_ASSUMPTION',independent=False,fit_on_evaluation_data=False)
            p.write_text(json.dumps(assumed))
            with self.assertRaises(ValueError):e.calibration(p,True)
            d['sources']['static_bias_m']['recording_id']='ISAS-Walk1';p.write_text(json.dumps(d))
            with self.assertRaises(ValueError):e.calibration(p)

    def test_uncovered_anchor_bias_not_zero(self):
        d,g=geometry();d['static_bias_m']={}
        r=e.evaluate_rows([observation()],'M0','r',d,g,[])[0]
        self.assertEqual(r['evaluation_status'],'MISSING_STATIC_BIAS');self.assertIsNone(r['raw_range_error'])


class IsolationTests(unittest.TestCase):
    def test_gt_and_derived_output_not_visible_and_mutation_invariant(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);measurement=root/'measurement.csv';measurement.write_text('range\n3.5\n')
            gt=root/'gt.tum';derived=root/'range_metrics.csv';out=root/'estimator';out.mkdir()
            probe='''import pathlib,hashlib,sys
measurement,gt,derived=map(pathlib.Path,sys.argv[1:])
assert not gt.exists() and not derived.exists()
for forbidden in (gt,derived):
 try: forbidden.open('rb')
 except (FileNotFoundError,PermissionError): pass
 else: raise AssertionError('GT file opened')
print(hashlib.sha256(measurement.read_bytes()).hexdigest())
'''
            cmd=sandbox_command(['/usr/bin/python3','-c',probe,str(measurement),str(gt),str(derived)],[measurement],out)
            results=[]
            for val in ['GT_ONE','GT_CHANGED_WITH_ORACLE_LABEL']:
                gt.write_text(val);derived.write_text('gt_range,range_error,oracle_nlos\n'+val)
                run=subprocess.run(cmd,text=True,capture_output=True)
                self.assertEqual(run.returncode,0,run.stderr);results.append(run.stdout)
            self.assertEqual(results[0],results[1])

    def test_readonly_inputs_and_hidden_host_home(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'measurement';source.write_text('3.5');out=root/'out';out.mkdir()
            probe='''import pathlib,sys
p=pathlib.Path(sys.argv[1])
try:p.write_text('gt_range')
except OSError:pass
else:raise AssertionError('input writable')
assert not pathlib.Path('/home/mint/ws_fusion_uwb/evaluator_private').exists()
'''
            r=subprocess.run(sandbox_command(['/usr/bin/python3','-c',probe,str(source)],[source],out),capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr);self.assertEqual(source.read_text(),'3.5')


if __name__=='__main__':unittest.main()
