import unittest
from unittest.mock import patch
from pathlib import Path
import formal_treatment as adapter


class TreatmentTests(unittest.TestCase):
    def test_no_candidates_identity(self):
        raw=[{'obs_id':'18446744073709551600','sensor_time_s':'2','anchor_id':'7475','observed_range_m':'3.125','source_valid':'1'}]
        native={'AUX_PRODUCER':{'run':'/producer'},'FGO_RECOVER_FULL':{'run':None,'backend':{}}}
        def read(p):
            if str(p).endswith('input_manifest.json'):return {'uwb_file':'raw.csv'}
            if str(p).endswith('production_detector_status.json'):return {'valid':True}
            return {'segments':[]}
        with patch.object(adapter,'read',side_effect=read),patch.object(adapter,'rows',return_value=raw),\
             patch.object(adapter,'save'),patch.object(Path,'exists',return_value=True):
            result=adapter.export('fixture',Path('/input/input_manifest.json'),native)
        self.assertTrue(result['recover_available'])
        self.assertEqual(result['rows'][0]['corrected_range'],3.125)
        self.assertEqual(result['rows'][0]['obs_id'],'18446744073709551600')

    def test_failed_support_not_oracle_identity(self):
        raw=[{'obs_id':'9','sensor_time_s':'2','anchor_id':'7475','observed_range_m':'3.125','source_valid':'1'}]
        native={'AUX_PRODUCER':{'run':None},'FGO_RECOVER_FULL':{'run':None,'backend':{}}}
        with patch.object(adapter,'read',return_value={'uwb_file':'raw.csv'}),patch.object(adapter,'rows',return_value=raw),patch.object(adapter,'save'):
            result=adapter.export('fixture',Path('/input/input_manifest.json'),native)
        self.assertFalse(result['support_valid']);self.assertFalse(result['recover_available'])
        self.assertEqual(result['rows'][0]['treatment_status'],'unavailable')

    def test_only_final_accepted_corrected(self):
        raw=[{'obs_id':str(i),'sensor_time_s':str(i),'anchor_id':'7475','observed_range_m':'4','source_valid':'1'} for i in (1,2,3)]
        native={'AUX_PRODUCER':{'run':'/producer'},'FGO_RECOVER_FULL':{'run':'/final','backend':{'valid_estimate_exported':True}}}
        def read(p):
            if str(p).endswith('input_manifest.json'):return {'uwb_file':'raw.csv'}
            if str(p).endswith('production_detector_status.json'):return {'valid':True}
            return {'segments':[{'segment_id':'s1','obs_ids':[1]},{'segment_id':'s2','obs_ids':[2]}]}
        def rows(p):
            if str(p).endswith('fixed_compensations.csv'):
                return [dict(segment_id='s'+str(i),final_use=str(i==1 and 1 or 0),delta_c_fixed_m='0.8',sigma_available='1',
                             sigma_c_local_m='0.1',c_hat_stage2_m='0.8') for i in (1,2)]
            return raw
        with patch.object(adapter,'read',side_effect=read),patch.object(adapter,'rows',side_effect=rows),\
             patch.object(adapter,'save'),patch.object(Path,'exists',return_value=True):
            result=adapter.export('fixture',Path('/input/input_manifest.json'),native)
        self.assertEqual([r['corrected_range'] for r in result['rows']],[3.2,4.,4.])
        self.assertAlmostEqual(result['rows'][0]['lcb'],.6)


if __name__=='__main__':unittest.main()
