#!/usr/bin/env python3
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import sc8_metrics as metrics

class MetricsTests(unittest.TestCase):
    def setUp(self):
        self.t=np.arange(61)*.05
        self.p=np.column_stack([self.t,self.t**2*.1,np.sin(self.t)])
        self.q=Rotation.from_euler('z',self.t*.2).as_quat()
        self.gt=(self.t,self.p,self.q)
    def test_zero_and_quaternion_sign(self):
        m,_=metrics.evaluate((self.t,self.p,-self.q),self.gt)
        for key in ['ape_m','are_rad','rpe_tag_0.5s_m','rpe_body_1s_rad','rpe_body_1s_m']:
            self.assertAlmostEqual(m[key]['max'],0,places=12)
    def test_offset_and_units(self):
        p=self.p+np.array([3.,4.,12.])
        m,_=metrics.evaluate((self.t,p,self.q),self.gt)
        self.assertAlmostEqual(m['ape_m']['rmse'],13)
        self.assertAlmostEqual(m['hpe_m']['rmse'],5)
        self.assertAlmostEqual(m['vpe_m']['rmse'],12)
        self.assertAlmostEqual(m['rpe_tag_1s_m']['max'],0,places=12)
    def test_rotation_and_body_rpe(self):
        q=(Rotation.from_euler('x',.3)*Rotation.from_quat(self.q)).as_quat()
        m,_=metrics.evaluate((self.t,self.p,q),self.gt)
        self.assertAlmostEqual(m['are_rad']['rmse'],.3)
        self.assertGreater(m['rpe_body_1s_m']['rmse'],.01)
        self.assertAlmostEqual(m['rpe_tag_1s_m']['max'],0,places=12)
    def test_lever(self):
        lever=np.array([.2,-.1,.05]);r=Rotation.from_quat(self.q)
        tag=self.p+r.apply(lever)
        m,_=metrics.evaluate((self.t,tag,self.q),self.gt)
        self.assertAlmostEqual(m['ape_m']['rmse'],np.linalg.norm(lever))
        m,_=metrics.evaluate((self.t,tag,self.q),(self.t,tag,self.q),lever)
        self.assertAlmostEqual(m['rpe_body_1s_m']['max'],0,places=12)
    def test_timestamp_offset_and_gaps(self):
        m,_=metrics.evaluate((self.t+.02,self.p,self.q),self.gt)
        self.assertGreater(m['ape_m']['rmse'],.015)
        mask=(self.t<1)|(self.t>2)
        m,_=metrics.evaluate((self.t[mask],self.p[mask],self.q[mask]),self.gt,times=self.t)
        self.assertLess(m['coverage'],1)
        self.assertLess(m['rpe_tag_1s_m']['n'],20)
    def test_duplicates_order_nan(self):
        for t in [self.t[::-1],np.r_[self.t[0],self.t[:-1]],self.t*np.nan]:
            with self.assertRaises(ValueError): metrics.validate(t,self.p,self.q)
    def test_world_rigid_transform_invariance(self):
        r=Rotation.from_rotvec([.3,-.1,.2]);tr=[5,2,1]
        ep=self.p+np.array([.1,-.05,.08])
        m,_=metrics.evaluate((self.t,ep,self.q),self.gt)
        q=(r*Rotation.from_quat(self.q)).as_quat()
        n,_=metrics.evaluate((self.t,r.apply(ep)+tr,q),(self.t,r.apply(self.p)+tr,q))
        for k in ['ape_m','are_rad','rpe_tag_1s_m','rpe_body_1s_m']:
            self.assertAlmostEqual(m[k]['rmse'],n[k]['rmse'],places=11)
    def test_nearest_rank_slow_values(self):
        self.assertEqual(metrics.summary([1]*19+[100])['p95'],1)
        self.assertEqual(metrics.summary([1]*18+[100,200])['p95'],100)
    def test_evo_1290_primary_metrics(self):
        import evo
        from evo.core.metrics import APE, RPE, PoseRelation, Unit
        from evo.core.trajectory import PoseTrajectory3D
        self.assertEqual(evo.__version__,'v1.29.0')
        ep=self.p+np.column_stack([.1*self.t,.2*self.t,np.zeros_like(self.t)])
        eq=(Rotation.from_quat(self.q)*Rotation.from_euler('y',.12*self.t)).as_quat()
        ours,_=metrics.evaluate((self.t,ep,eq),self.gt)
        def traj(p,q):return PoseTrajectory3D(positions_xyz=p,orientations_quat_wxyz=q[:,[3,0,1,2]],timestamps=self.t)
        ref,est=traj(self.p,self.q),traj(ep,eq)
        for relation,key in [(PoseRelation.translation_part,'ape_m'),(PoseRelation.rotation_angle_rad,'are_rad')]:
            metric=APE(relation);metric.process_data((ref,est))
            np.testing.assert_allclose(ours[key]['rmse'],np.sqrt(np.mean(metric.error**2)),atol=1e-10)
        # Uniform .05s stamps: 20-frame pairs exactly match 1s pairs.
        for relation,key in [(PoseRelation.translation_part,'rpe_body_1s_m'),(PoseRelation.rotation_angle_rad,'rpe_body_1s_rad')]:
            metric=RPE(relation,delta=20,delta_unit=Unit.frames,all_pairs=True)
            metric.process_data((ref,est))
            np.testing.assert_allclose(ours[key]['rmse'],np.sqrt(np.mean(metric.error**2)),atol=1e-10)

if __name__=='__main__':unittest.main(verbosity=2)
