import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/paper'))
from t11_prefix import crop
from t11_verify import normalize, compare_pair

class PrefixTest(unittest.TestCase):
    def test_exact_cutoff_and_raw_bytes_and_source_ids(self):
        with tempfile.TemporaryDirectory() as d:
            a,b=Path(d)/'a',Path(d)/'b'
            data=b'obs_id,source_index,sensor_time_s,x\n99,8,5.9,0.12345678901234567\n42,9,6,2\n17,10,6.000000000000001,3\n'
            a.write_bytes(data);self.assertEqual(crop(a,b,6),2)
            self.assertEqual(b.read_bytes(),b'\n'.join(data.split(b'\n')[:3])+b'\n')
            # Future deletion or future numerical change cannot alter prefix.
            a.write_bytes(data.replace(b'17,10,6.000000000000001,3',b'17,10,8,999'))
            c=Path(d)/'c';crop(a,c,6);self.assertEqual(b.read_bytes(),c.read_bytes())
    def test_normalization_does_not_hide_scientific_changes(self):
        a={'stage1_elapsed_seconds':1,'elapsed_s':1,'bias_m':.1,'values_sha256':'original','graph_linearization_sha256':'g'}
        b=dict(a,elapsed_s=2,stage1_elapsed_seconds=2);self.assertEqual(normalize(a,{}),normalize(b,{}))
        for k,v in [('bias_m',.10000000000000002),('values_sha256','changed'),('graph_linearization_sha256','changed')]:
            self.assertNotEqual(normalize(a,{}),normalize(dict(b,**{k:v}),{}))
    def test_common_failure_is_not_u13_pass(self):
        with tempfile.TemporaryDirectory() as d:
            a,b=Path(d)/'a',Path(d)/'b';a.mkdir();b.mkdir()
            for p in (a,b):(p/'pipeline_status.json').write_text('{"status":"FAILED"}')
            self.assertEqual(compare_pair(a,b)['status'],'NOT_RUN_INCOMPLETE_CHAIN')

class EvaluationAccountingTest(unittest.TestCase):
    def test_zero_accepted_retains_history_universe_and_undefined_error(self):
        import json
        import numpy as np
        from t11_evaluate import final_metrics
        with tempfile.TemporaryDirectory() as d:
            case=Path(d);p=case/'output/final/full_gate';p.mkdir(parents=True)
            (p/'validation_status.json').write_text('{"valid_estimate":true,"status":"OK"}')
            (p/'final_masks.csv').write_text('obs_id,candidate,final_use,segment_id\n1,1,0,segment\n2,0,1,\n')
            (p/'segment_bias.csv').write_text('segment_id,amplitude_m\n')
            (p/'trajectory.tum').write_text('3 0 0 0 0 0 0 1\n6 0 0 0 0 0 0 1\n')
            (p/'fallback_attempt.json').write_text('{"attempted":false}')
            result=final_metrics(case,'full_gate',['1','2'],[(3.,np.zeros(3)),(6.,np.zeros(3))],{'1':2.,'2':0.})
            self.assertAlmostEqual(result['bias_field_rmse_m'],2**.5)
            self.assertEqual(result['history_count'],2)
            self.assertEqual(result['coverage'],0.)
            self.assertIsNone(result['accepted_only_rmse_m'])
            self.assertEqual(result['accepted_only_status'],'UNDEFINED')
    def test_failure_does_not_become_zero_coverage(self):
        from t11_evaluate import final_metrics
        with tempfile.TemporaryDirectory() as d:
            result=final_metrics(Path(d),'full_gate',['1'],None,None)
            self.assertIsNone(result['coverage'])
            self.assertIsNone(result['bias_field_rmse_m'])
            self.assertFalse(result['status']['valid_estimate'])

if __name__=='__main__':unittest.main()
