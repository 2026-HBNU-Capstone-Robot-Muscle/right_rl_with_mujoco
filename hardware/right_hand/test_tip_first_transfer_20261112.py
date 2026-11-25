"""Regression checks for physical motor ordering and bounded sequence proposals."""
import unittest
import numpy as np
from tip_first_transfer_20261112 import propose,rearward_side_offset,cap_ring_next_for_candidate

class Tests(unittest.TestCase):
    def test_physical_tip_precedes_next_link_and_other_motors_are_held(self):
        start=np.full(16,2100);p=propose(start,np.full(16,1000),np.full(16,3100),fraction=.65)
        tips=np.asarray(p.phases)=='tips_first';nexts=np.asarray(p.phases)=='next_links_second'
        self.assertTrue(np.all(p.goals[tips][:,[1,10]]==2100))
        self.assertTrue(np.all(p.goals[nexts][:,[0,8]]==2750))
        held=np.setdiff1d(np.arange(16),[0,1,8,10]);self.assertTrue(np.all(p.goals[:,held]==2100))
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([start,p.goals]),axis=0))),2)
    def test_limits_cannot_be_bypassed_by_fraction_or_bad_start(self):
        lo=np.full(16,1000);hi=np.full(16,3000)
        for start,fraction in ((np.full(16,3100),1),(np.full(16,2100),1.01),(np.full(16,2100),0)):
            with self.assertRaises(ValueError):propose(start,lo,hi,fraction=fraction)
    def test_rearward_shift_stays_tangent_and_preserves_registered_inputs(self):
        front=np.array([0.,0.,2.]);side=np.array([3.,0.,0.]);offset=rearward_side_offset(front,side,3.)
        np.testing.assert_allclose(offset,[0,0,-.003]);self.assertEqual(float(offset@side),0.)
        np.testing.assert_array_equal(front,[0,0,2]);np.testing.assert_array_equal(side,[3,0,0])
        with self.assertRaises(ValueError):rearward_side_offset(front,front,3)
    def test_ring_cap_is_candidate_only_and_does_not_modify_other_motors(self):
        q=np.full((6,16),3100);copy=q.copy();capped=cap_ring_next_for_candidate(q,2880)
        np.testing.assert_array_equal(q,copy);self.assertTrue(np.all(capped[:,14]==2880))
        np.testing.assert_array_equal(capped[:,:14],q[:,:14]);np.testing.assert_array_equal(capped[:,15],q[:,15])

if __name__=='__main__':unittest.main()
