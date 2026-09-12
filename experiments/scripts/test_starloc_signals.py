import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from diagnose_starloc_signals import Signal,propagate,intervals_ok
class Signals(unittest.TestCase):
 def test_linear_signal_mean_and_shift_sign(self):
  t=np.linspace(0,4,401);s=Signal(t,np.column_stack([t,2*t,3*t]));np.testing.assert_allclose(s.mean(np.array([1.]),.2),[[1,2,3]],atol=1e-13)
  # Physical gt at t=1.3 corresponds to source t=1.0 when offset=+.3.
  np.testing.assert_allclose(s.mean(np.array([1.3])-.3,.2),[[1,2,3]],atol=1e-13)
 def test_gravity_and_rad_units(self):
  t=np.linspace(0,1,101);a=np.tile([0,0,9.81],(len(t),1));w=np.tile([0,0,1.],(len(t),1));rot,p,v=propagate(t,a,w,Rotation.identity())
  np.testing.assert_allclose(rot.as_rotvec(),[0,0,1],atol=1e-12);np.testing.assert_allclose(p,[0,0,4.905],atol=1e-12);np.testing.assert_allclose(v,[0,0,9.81],atol=1e-12)
  rot,_,_=propagate(t,a,w*np.pi/180,Rotation.identity());self.assertAlmostEqual(rot.magnitude(),np.pi/180)
 def test_no_gap_crossing(self):
  self.assertFalse(intervals_ok([0,.01,.2],[.005],[.15])[0]);self.assertTrue(intervals_ok([0,.01,.02],[0],[.02])[0]);self.assertFalse(intervals_ok([0,.01,.02],[-.01],[.01])[0])
if __name__=='__main__':unittest.main()
