#!/usr/bin/env python3
"""Regression for demonstrated closeout failures and matched-input wiring."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/paper'))
import t10_closeout as closeout
import a19_r08_evaluate as evaluator
import numpy as np


class CloseoutRegression(unittest.TestCase):
    def test_policy_reason_strings_do_not_change_use_suppress_identity(self):
        gates = {'s_fit': [('group_0', 'USE', 'S_AND_FIT_PASS')],
                 'full_gate': [('group_0', 'USE', 'ETA_S_AND_FIT_PASS')]}
        self.assertTrue(evaluator.same_gate_decisions(gates))
        gates['full_gate'][0] = ('group_0', 'SUPPRESS', 'ETA_FAIL')
        self.assertFalse(evaluator.same_gate_decisions(gates))

    def test_changed_conditions_preserve_parent_observations_and_imu(self):
        base=closeout.OLD/'inputs/a10_val_turn_01_seed20101'
        original=closeout.read_rows(base/'raw/step2/uwb_observations.csv')
        with tempfile.TemporaryDirectory() as d:
            b=closeout.derive(base,Path(d)/'B','B',20101)
            c=closeout.derive(base,Path(d)/'C','C',20101)
            br=closeout.read_rows(b/'raw/step2/uwb_observations.csv')
            cr=closeout.read_rows(c/'raw/step2/uwb_observations.csv')
            self.assertEqual([r['obs_id'] for r in original],[r['obs_id'] for r in br])
            self.assertEqual([r for r in original if int(r['source_message_index'])%2==0],cr)
            noise=np.random.Generator(np.random.PCG64(np.random.SeedSequence([20101,0]))).normal(0.,.05,(41,8))
            for a,z in zip(original,br):
                self.assertEqual(float(z['observed_range_m']),float(a['observed_range_m'])+noise[int(a['source_message_index']),int(a['source_range_index'])])
            for x in (b,c):
                self.assertEqual((base/'raw/step2/imu.csv').read_bytes(),(x/'raw/step2/imu.csv').read_bytes())
                self.assertNotEqual(closeout.obj(x/'raw/step2/input_manifest.json')['cache_id'],closeout.obj(base/'raw/step2/input_manifest.json')['cache_id'])

    def test_gate_reason_labels_are_not_decision_differences(self):
        self.assertTrue(evaluator.same_gate_decisions({
            "s_fit": [("g0", "USE", "S_AND_FIT_PASS")],
            "full_gate": [("g0", "USE", "ALL_THREE_PASS")]}))
        self.assertFalse(evaluator.same_gate_decisions({
            "s_fit": [("g0", "USE", "S_AND_FIT_PASS")],
            "full_gate": [("g0", "SUPPRESS", "ETA_FAIL")]}))

    def test_failed_producer_evaluation_does_not_read_truth_or_invent_zero(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            closeout.write(root/'pipeline_status.json',{'status':'FAILED','reason':'CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED'})
            closeout.write(root/'freeze.json',{'evaluation_label':'LIMITED_SYNTHETIC_VALIDATION','decisions_and_finals_frozen':True,
                'payloads':[{'path':str(root/'pipeline_status.json'),'sha256':closeout.sha(root/'pipeline_status.json')[7:]}]})
            result=subprocess.run([sys.executable,str(ROOT/'tools/paper/a19_r08_evaluate.py'),
                '--run-root',str(root),'--generation-manifest',str(root/'DOES_NOT_EXIST_TRUTH'),
                '--freeze-manifest',str(root/'freeze.json'),'--output',str(root/'result.json'),
                '--scenario','step2','--validation-context',str(root/'DOES_NOT_EXIST_CONTEXT')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            data=closeout.obj(root/'result.json')
            self.assertFalse(data['truth_read'])
            self.assertIsNone(data['candidate_observations'])
            self.assertIsNone(data['scores'])
            self.assertEqual(data['reason'],'CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED')
            # Corrupted frozen content remains a hard rejection.
            closeout.write(root/'pipeline_status.json',{'status':'SCORED'})
            result=subprocess.run(result.args,capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('frozen payload mismatch',result.stderr)


def verify_los(case):
    """Assert real same-producer artifacts; no mocked optimizer or stdout PASS."""
    case=Path(case)
    run=closeout.obj(case/'RUN_RESULT.json')
    assert run['exit_code']==0 and run['truth_open_count']==0
    root=case/'output'
    status=closeout.obj(root/'scoring/status.json')
    assert status=={'status':'not_applicable','reason':'NOT_APPLICABLE_NO_CANDIDATES','computed':False,'eta':None,'s_m':None,'gamma':None}
    assert closeout.read_rows(root/'stage1/partition.csv')==[]
    assert closeout.read_rows(root/'scores.csv')==[]
    evaluation=closeout.obj(case/'evaluation.json')
    assert evaluation['recoverability']==status
    assert evaluation['candidate_observations']==0
    for policy in evaluator.POLICIES:
        record=evaluation['policies'][policy]
        assert record['status']['valid_estimate'] is True
        assert record['trajectory_full']['rmse_m']['status']=='AVAILABLE'
        assert record['coverage']['candidate_use_coverage']['status']=='UNDEFINED'
        assert record['decision_time']['accepted_bias_rmse_m']['value'] is None
        assert record['final_time']['applied_bias_field_rmse_m']['value'] is None
        assert record['final_time']['accepted_bias_rmse_m']['value'] is None
    assert evaluation['los_all_range_reference']['status']['valid_estimate'] is True
    assert (root/'final/all_range/trajectory.tum').is_file()
    assert list((root/'final_solver/all_range/empty_recovery').glob('outer*'))
    return {'status':'PASS','case':str(case),'path':'RAW_AUTOMATIC_EMPTY_STAGE2_SCORE_NA_FINAL_INDEPENDENT_ALL_RANGE_EVALUATOR'}


if __name__=='__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--verify-los':
        print(json.dumps(verify_los(sys.argv[2])))
    else:
        unittest.main()
