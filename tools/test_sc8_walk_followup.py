#!/usr/bin/env python3
"""Deterministic reference tests, independent of real TEST truth."""
import unittest
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import minimize_scalar
from sc8_walk_followup import gyro_pairs

class ReferenceTests(unittest.TestCase):
    def test_composition_and_inverse(self):
        wt=np.eye(4);ti=np.eye(4)
        wt[:3,:3]=Rotation.from_rotvec([.2,-.3,.5]).as_matrix();wt[:3,3]=[2,3,4]
        ti[:3,:3]=Rotation.from_rotvec([-.7,.2,.1]).as_matrix();ti[:3,3]=[.1,.2,-.3]
        tag=np.array([.1,-.025,0,1.])
        wi=wt@ti
        np.testing.assert_allclose(wi@tag,wt@(ti@tag),atol=1e-14)
        np.testing.assert_allclose(np.linalg.inv(wt)@wi,ti,atol=1e-14)
        np.testing.assert_allclose((wi@tag)[:3],wi[:3,3]+wi[:3,:3]@tag[:3])
        self.assertGreater(np.linalg.norm(wt@ti-ti@wt),.1)

    def test_gravity_units_and_axis(self):
        r=Rotation.from_euler('x',180,degrees=True)
        np.testing.assert_allclose(r.apply([0,0,9.80665]),[0,0,-9.80665],atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(r.apply([0,0,9.80665])),9.80665)
        self.assertAlmostEqual(Rotation.from_rotvec([0,0,np.pi/2]).magnitude(),np.pi/2)

    def test_known_time_and_rotation(self):
        t=np.arange(0,10,.002)
        angle=.3*np.sin(2*t)+.2*np.cos(.7*t)
        omega=.6*np.cos(2*t)-.14*np.sin(.7*t)
        r=Rotation.from_rotvec([.5,-.2,.3])
        # R_WI=R_WT R_TI, so omega_T=R_TI omega_I.
        q=(Rotation.from_rotvec(np.c_[angle,angle*0,angle*0])*r).as_quat()
        imu=pd.DataFrame(dict(t=t,gx=r.inv().apply(np.c_[omega,omega*0,omega*0])[:,0],
            gy=r.inv().apply(np.c_[omega,omega*0,omega*0])[:,1],gz=r.inv().apply(np.c_[omega,omega*0,omega*0])[:,2]))
        gt=pd.DataFrame(np.c_[t+.037,Rotation.from_rotvec(np.c_[angle,angle*0,angle*0]).as_quat()],columns=['t','qx','qy','qz','qw'])
        def loss(dt):
            a,b=gyro_pairs(imu,gt,dt)
            return np.mean((r.apply(a)-b)**2)
        result=minimize_scalar(loss,bounds=(-.1,.1),method='bounded')
        self.assertLess(abs(result.x-.037),.001)
        self.assertLess(loss(result.x),1e-6)
        self.assertGreater(loss(-.037),loss(result.x)*100)

if __name__=='__main__':unittest.main()
