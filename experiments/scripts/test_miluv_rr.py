"""MILUV source projection and published geometry admission tests."""
import csv,io,tempfile,unittest
from pathlib import Path
import numpy as np
import run_miluv_recover_vs_reject as m

class Miluv(unittest.TestCase):
    def test_raw_projection_identity_and_tag(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'u.csv'
            p.write_text('timestamp,from_id,to_id,range_raw,range,range_gt\n1,11,0,4,3,2\n1.013,10,3,5.1200,3,2\n')
            u=m.r.projection(p,m.UWB_FIELDS)
            im=[dict(source_row=0,**dict(zip(m.IMU_FIELDS,['.99','1','2','9.8','.1','.2','.3'])))]
            result=m.cache_payloads('fixture',u,im,m.geometry());manifest,ub,ib=result
            rows=list(csv.DictReader(io.StringIO(ub.decode())));v=list(csv.DictReader(io.StringIO(ib.decode())))[0]
            self.assertEqual(len(rows),1);self.assertEqual(rows[0]['source_message_index'],'1')
            self.assertEqual(rows[0]['obs_id'],str(m.r.stable_id('fixture',1)))
            self.assertEqual(rows[0]['observed_range_m'],'5.1200');self.assertEqual(rows[0]['sensor_time_s'],'1.013')
            self.assertEqual([v['acc_'+a+'_mps2'] for a in 'xyz'],['1','2','9.8'])
            self.assertEqual([v['gyro_'+a+'_radps'] for a in 'xyz'],['.1','.2','.3']);self.assertEqual(v['sensor_time_s'],'.99')
            p.write_text(p.read_text().replace(',3,2',',900,-800'))
            self.assertEqual(result,m.cache_payloads('fixture',m.r.projection(p,m.UWB_FIELDS),im,m.geometry()))
    def test_geometry_no_fitted_transform(self):
        g=m.geometry();np.testing.assert_equal(g['axis_transform'],np.eye(3))
        np.testing.assert_allclose(g['lever_body_m'],[.13189,-.17245,-.05249],atol=0)
        self.assertEqual(set(g['anchors']),set(range(6)));self.assertFalse(g['imu_bias_removal'])
    def test_bad_input_rejected(self):
        im=[dict(source_row=0,**dict(zip(m.IMU_FIELDS,['0','0','0','9.8','0','0','0'])))]
        u=[dict(source_row=0,timestamp='0',from_id='10',to_id='99',range_raw='2')]
        with self.assertRaisesRegex(ValueError,'UNKNOWN_ANCHOR'):m.cache_payloads('x',u,im,m.geometry())

if __name__=='__main__':unittest.main()
