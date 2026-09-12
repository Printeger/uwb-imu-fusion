"""Additional development admission/actual final-artifact engineering checks."""
import csv,io,tempfile,unittest
from pathlib import Path
import numpy as np
import run_recover_vs_reject as r
import rr_admitted as a
import rr_evaluate as ev
from test_recover_vs_reject import measurements

class Admitted(unittest.TestCase):
    def test_fixed_transform_clock_and_raw_rows(self):
        geo=r.read(a.GEOMETRY)
        for name,g in geo['recordings'].items():
            u,im=measurements();m,ub,ib=r.cache_payloads(name,u,im,g)
            z=list(csv.DictReader(io.StringIO(ub.decode())));v=list(csv.DictReader(io.StringIO(ib.decode())))
            self.assertEqual([x['sensor_time_s'] for x in z],[x['time_s'] for x in u])
            self.assertEqual([x['observed_range_m'] for x in z],[x['range'] for x in u])
            self.assertAlmostEqual(float(v[0]['sensor_time_s']),float(im[0]['time_s'])+g['clock']['imu_to_uwb_offset_s'])
            np.testing.assert_equal([float(v[0]['acc_'+k+'_mps2']) for k in 'xyz'],[0,-1,-2])
            np.testing.assert_equal([float(v[0]['gyro_'+k+'_radps']) for k in 'xyz'],[3,-4,-5])
            for row in im:row['x']=999;row['gt_range']=-900
            # Whitelisted source projection never supplies these evaluator fields.
            im=[{k:v for k,v in row.items() if k in (*r.IMU_COLUMNS,'source_row')} for row in im]
            self.assertEqual((m,ub,ib),r.cache_payloads(name,u,im,g))

    def test_common_body_lever(self):
        g=r.read(a.GEOMETRY);c=g['common_geometry'];T=np.array(c['T_rig_imu'])
        np.testing.assert_allclose(np.array(g['lever_body_m'])+T[:3,3],c['tag_in_rig_m'],atol=1e-14)
        np.testing.assert_allclose(T,np.linalg.inv(c['T_camera_rig'])@c['T_camera_imu'],atol=1e-14)

    def test_actual_final_masks_and_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            (p/'final_masks.csv').write_text('obs_id,candidate,decision_use,final_use,fallback_use,segment_id\n12,1,1,1,0,s1\n13,1,0,0,0,s2\n14,0,1,1,0,\n')
            (p/'fixed_compensations.csv').write_text('segment_id,final_use,delta_c_fixed_m\ns1,1,0.8\ns2,0,0\n')
            r.write(p/'final_inference_summary.json',{'fallback_attempt_count':0})
            offsets,accepted,counts=ev.final_offsets(p,[12,13,14])
            self.assertEqual(offsets,{12:.8});self.assertEqual(accepted,{12})
            self.assertEqual(counts['candidate_segments'],2);self.assertEqual(counts['final_accepted_segments'],1)
            r.write(p/'final_inference_summary.json',{'fallback_attempt_count':1})
            with self.assertRaisesRegex(ValueError,'FALLBACK_WITH_RECOVERY'):ev.final_offsets(p,[12,13,14])
            text=(p/'final_masks.csv').read_text().replace('12,1,1,1,0','12,1,1,0,1')
            (p/'final_masks.csv').write_text(text)
            offsets,accepted,counts=ev.final_offsets(p,[12,13,14]);self.assertEqual(offsets,{})
            self.assertEqual(counts['recovery_status'],'FALLBACK')

if __name__=='__main__':unittest.main()
