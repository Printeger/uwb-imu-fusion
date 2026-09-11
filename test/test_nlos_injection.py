import sys,unittest,tempfile,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/paper'))
import nlos_injection as n
import nlos_injection_metrics as m

class InjectionTests(unittest.TestCase):
 def test_closed_and_disabled_identity(self):
  with tempfile.TemporaryDirectory() as t:
   root=Path(t);base=root/'base';base.mkdir()
   data=[]
   for i in range(6):
    data.append(dict(obs_id=str(i),source_message_index=str(i),source_range_index='0',source_observation_index=str(i),sensor_time_s=str(i),tag_id='1',anchor_id='2' if i!=3 else '3',observed_range_m='2.125',fp_rssi_dbm='-70',rx_rssi_dbm='-70',source_valid='0' if i==2 else '1',source_validity_reason_hex='626164' if i==2 else ''))
   (base/'uwb_observations.csv').write_bytes(n.csv_bytes(data));(base/'imu.csv').write_bytes(b'immutable IMU\n');(base/'anchors.yaml').write_text('- {id: 2}\n- {id: 3}\n')
   manifest=dict(base_recording_id='original',base_source_sha256='sha256:base',recording_time_origin_s=0.,imu_sha256=n.sha((base/'imu.csv').read_bytes()),uwb_sha256=n.sha((base/'uwb_observations.csv').read_bytes()),imu_count=1,uwb_observation_count=6,uwb_message_count=6)
   manifest['cache_id']=n.v1_id(manifest);n.write_json(base/'input_manifest.yaml',manifest)
   spec=n.specification(False,1,2,1,4,[2,3]);a=n.generate(base,root/'off',root/'off_truth',spec)
   self.assertEqual((root/'off/uwb_observations.csv').read_bytes(),(base/'uwb_observations.csv').read_bytes())
   spec['enabled']=True;b=n.generate(base,root/'on',root/'on_truth',spec);c=n.generate(base,root/'again',root/'again_truth',spec)
   self.assertEqual(b,c);self.assertNotEqual(a['cache_id'],b['cache_id'])
   actual=n.rows(root/'on/uwb_observations.csv')
   for i,(old,new) in enumerate(zip(data,actual)):
    expect=dict(old)
    if i in (1,4):expect['observed_range_m']='2.625'
    self.assertEqual(expect,new)
   self.assertEqual((root/'on/imu.csv').read_bytes(),(base/'imu.csv').read_bytes())
   spec['anchor_subset']=[2];d=n.generate(base,root/'sub',root/'sub_truth',spec);self.assertNotEqual(d['cache_id'],b['cache_id'])
   spec['amplitude_m']=1
   different=n.generate(base,root/'different_amplitude',root/'different_amplitude_truth',spec);self.assertNotEqual(different['cache_id'],d['cache_id'])
   spec['end_s']=3
   changed=n.generate(base,root/'different_interval',root/'different_interval_truth',spec);self.assertNotEqual(changed['cache_id'],different['cache_id'])
   spec['anchor_id']=3
   changed_link=n.generate(base,root/'different_link',root/'different_link_truth',spec);self.assertNotEqual(changed_link['cache_id'],changed['cache_id'])
   spec['shape']='ramp'
   with self.assertRaises(ValueError):n.generate(base,root/'bad',root/'bad_truth',spec)
   data[1]['observed_range_m']='59.75';data[4]['observed_range_m']='70'
   (base/'uwb_observations.csv').write_bytes(n.csv_bytes(data));manifest['uwb_sha256']=n.sha((base/'uwb_observations.csv').read_bytes());manifest['cache_id']=n.v1_id(manifest);n.write_json(base/'input_manifest.yaml',manifest)
   spec=n.specification(True,1,2,1,4,[2,3])
   with self.assertRaisesRegex(ValueError,'OUT_OF_VALID_BOUNDS'):n.generate(base,root/'overflow',root/'overflow_truth',spec)
   data[1]['observed_range_m']='2.125';(base/'uwb_observations.csv').write_bytes(n.csv_bytes(data));manifest['uwb_sha256']=n.sha((base/'uwb_observations.csv').read_bytes());manifest['cache_id']=n.v1_id(manifest);n.write_json(base/'input_manifest.yaml',manifest)
   n.generate(base,root/'invalid',root/'invalid_truth',spec)
   self.assertEqual(n.rows(root/'invalid/uwb_observations.csv')[4]['observed_range_m'],'70')
   self.assertEqual(n.rows(root/'invalid_truth/nlos_injected_observations.csv')[4]['validity_reason'],'INVALID_RANGE')

 def test_windows_and_persistence(self):
  self.assertEqual(n.windows(list(range(20)),4,1,2,.01)[0],(5,13))
  self.assertEqual(n.windows([5+i*.2 for i in range(40)],4,1,2,.01,recording_end=14)[0],(5.,13.))
  self.assertEqual(n.windows([5,6,8,9],4,1,2,.01),[(5,6),(8,9)])
  rr=[dict(tag_id='1',anchor_id='2',sensor_time=str(i),obs_id=str(i),fault_detected='1' if i<2 else '0') for i in range(4)]
  self.assertEqual(n.persistent_faults(rr,1,2,.01),{(1,2)})
 def test_metrics_partial_zero_overlap(self):
  rr=[dict(obs_id=str(i),sensor_time=str(i),valid='1',planned='1',tested='1',fault_detected='1',nlos_candidate='1' if i<2 else '0',segment_id='s' if i<2 else '') for i in range(4)]
  result=m.detection(rr,['1','2'],1)['positive_excess_candidate'];self.assertEqual((result['tp'],result['fp'],result['fn']),(1,1,1))
  rr[0]['tested']='0';self.assertIsNone(m.detection(rr,['1'],1)['two_sided_fault']['tp'])
  self.assertIsNone(m.ratio(0,0)['value']);self.assertEqual(m.correction(0)['bin'],'zero');self.assertEqual(m.correction(.45)['bin'],'near-full');self.assertEqual(m.correction(.6)['relation'],'over');self.assertEqual(m.correction(None)['status'],'UNAVAILABLE')
  s=m.segment_metric(rr,['1','2'],1,2,.4);self.assertEqual(s['weighted_injected_component_m'],.25);self.assertTrue(s['mixed'])
  self.assertIsNone(m.segment_metric([],[],1,2,None)['c_hat_stage2_m'])
  for r in rr:r['tested']='1';r['nlos_candidate']='0';r['segment_id']=''
  self.assertEqual(m.detection(rr,['1'],1)['temporal_support']['fn'],1)
class MetricEdgeTests(unittest.TestCase):
 def test_zero_candidates_and_fragmented_support(self):
  rr=[dict(obs_id=str(i),sensor_time=str(i),valid='1',planned='1',tested='1',fault_detected='0',nlos_candidate='0',segment_id='') for i in range(6)]
  z=m.detection(rr,[],0);self.assertIsNone(z['positive_excess_candidate']['precision']['value']);self.assertEqual(z['positive_excess_candidate']['tp'],0)
  for i in [1,2,4]:rr[i].update(fault_detected='1',nlos_candidate='1',segment_id='a' if i<3 else 'b')
  d=m.detection(rr,['1','2','3','4'],1)['temporal_support'];self.assertEqual((d['tp'],d['fn']),(3,1))
  self.assertEqual(m.detection(rr,['1'],1,False)['status'],'UNAVAILABLE_FDE_INCOMPLETE')
  failed=m.detection([],['1'],1,False,planned_ids=['1','2']);self.assertEqual(failed['planned_count'],2);self.assertIsNone(failed['positive_excess_candidate']['fn'])
 def test_weighted_mixed_truth(self):
  rr=[dict(obs_id='1',sensor_time='1',ledger_nominal_sigma_m='1'),dict(obs_id='2',sensor_time='2',ledger_nominal_sigma_m='2')]
  out=m.segment_metric(rr,['1'],1,1,.3);self.assertAlmostEqual(out['weighted_injected_component_m'],.4);self.assertAlmostEqual(out['signed_error_vs_support_mean_m'],-.1)
 def test_suppress_failure_and_fallback_delivery(self):
  for decision,used,fallback,success in [(False,False,False,True),(True,False,True,True),(True,True,False,False)]:
   row=m.correction_delivery(.3,decision,used,fallback,success)
   self.assertIsNone(row['actual_offset_m']);self.assertEqual(row['actual']['status'],'UNAVAILABLE_NOT_CORRECTED')
  self.assertEqual(m.correction_delivery(.3,True,True,False,True)['actual_offset_m'],.3)
 def test_exact_planned_timestamps_and_window_alignment(self):
  with tempfile.TemporaryDirectory() as t:
   run=Path(t);tum='0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n2 1 1 0 0 0 0 1\n4 0 1 0 0 0 0 1\n'
   (run/'trajectory.tum').write_text(tum);(run/'gt.tum').write_text(tum)
   (run/'observations.csv').write_bytes(n.csv_bytes([dict(raw_time=i,planned='1',keyframe_id=i) for i in range(4)]))
   out=m.localization(run,dict(ground_truth=str(run/'gt.tum'),time_association={'policy':'nearest_within_tolerance','tolerance_s':.02}),(0,1))
   self.assertEqual(out['trajectory_coverage']['value'],.75);self.assertFalse(out['complete_planned_trajectory'])
   self.assertEqual(out['window']['alignment'],'FULL_TRAJECTORY_SCALE1_SE3');self.assertEqual(out['window']['count'],2)
if __name__=='__main__':unittest.main()
