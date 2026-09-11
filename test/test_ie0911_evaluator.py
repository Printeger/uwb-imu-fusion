import importlib.util,json,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('evaluate',ROOT/'tools/paper/evaluate_runs.py')
e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e)
class FixedReferenceTest(unittest.TestCase):
 def test_reference_is_independent_and_zero_coverage_is_not_zero_error(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp)
   (p/'reference.csv').write_text('obs_id,geometric_range_m,fixed_beta_m\n1,2,0.1\n')
   (p/'observations.csv').write_text('obs_id,raw_z_m\n1,2.6\n')
   (p/'final_masks.csv').write_text('obs_id,candidate,final_use,segment_id\n1,1,1,s0\n')
   (p/'fixed_compensations.csv').write_text('segment_id,delta_c_fixed_m\ns0,0.2\n')
   scenario={'range_reference_csv':str(p/'reference.csv'),'range_reference_sha256':e.file_sha256(p/'reference.csv'),'range_reference_provenance':dict.fromkeys(['independent_geometry','frame','point','time','fixed_beta'],'fixture')}
   result=e.range_reference_metrics(p,scenario)
   self.assertAlmostEqual(result['restored_before_rmse_m']['value'],0.5)
   self.assertAlmostEqual(result['restored_after_rmse_m']['value'],0.3)
   (p/'final_masks.csv').write_text('obs_id,candidate,final_use,segment_id\n1,1,0,s0\n')
   self.assertIsNone(e.range_reference_metrics(p,scenario)['restored_after_rmse_m']['value'])
   scenario['range_reference_sha256']='bad'
   with self.assertRaises(ValueError): e.range_reference_metrics(p,scenario)
if __name__=='__main__':unittest.main()
