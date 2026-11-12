"""Reject broad tolerances, moving-thumb exceptions and planned penetration."""
import copy,unittest
from types import SimpleNamespace
import numpy as np
import horizontal_thumb_book_numeric_20261027 as policy

class HeldThumbNumericTests(unittest.TestCase):
    def setUp(self):
        self.raw=np.array([2542,2012,2518,1953,1973,1887,1761,1982,2030,1988,1824,1979,1826,2040,1841,1974])
        self.plan=SimpleNamespace(source_position=lambda:self.raw.copy(),plan=lambda a:[('stage_1_roots_clear',a.copy())])
        self.empty=dict(static_cases=[dict(commanded_thumb_book_mm=0.)],dynamic_cases=[dict(raw_held_thumb_book_mm=.014445)])
    def test_only_current_family_fixed_thumb_and_book_pair(self):
        self.assertEqual(policy.allowance(self.raw,'stage_1_roots_clear',policy.PAIR),.02)
        self.assertEqual(policy.allowance(self.raw,'stage_1_roots_clear','support_glass|th_ds_collision_1'),0)
        self.assertEqual(policy.allowance(self.raw,'stage_1_roots_clear','support_book|mf_ds_collision_1'),0)
        self.assertEqual(policy.allowance(self.raw,'stage_7_horizontal_yaw',policy.PAIR),0)
    def test_other_postures_have_no_new_allowance(self):
        curled=self.raw.copy();curled[8]=3128
        below=self.raw.copy();below[3]=1871
        self.assertEqual(policy.allowance(curled,'initial_hold',policy.PAIR),0)
        self.assertEqual(policy.allowance(below,'initial_hold',policy.PAIR),0)
    def test_observed_depth_is_reported_without_suppressing_it(self):
        result=policy.review(self.plan,self.empty)
        self.assertEqual(result['maximum_raw_held_depth_mm'],.014445)
        self.assertTrue(result['policy']['contact_forces_preserved'])
    def test_dynamic_upper_boundary_is_enforced(self):
        for depth in (.020001,float('nan'),float('inf'),-1,False):
            changed=copy.deepcopy(self.empty);changed['dynamic_cases'][0]['raw_held_thumb_book_mm']=depth
            with self.assertRaises(RuntimeError):policy.review(self.plan,changed)
    def test_even_small_planned_penetration_is_rejected(self):
        changed=copy.deepcopy(self.empty);changed['static_cases'][0]['commanded_thumb_book_mm']=.0001
        with self.assertRaises(RuntimeError):policy.review(self.plan,changed)
    def test_thumb_moving_during_roots_cannot_use_the_allowance(self):
        changed=self.raw.copy();changed[1]+=1
        plan=SimpleNamespace(source_position=lambda:self.raw.copy(),plan=lambda a:[('stage_1_roots_clear',changed)])
        with self.assertRaises(RuntimeError):policy.review(plan,self.empty)

if __name__=='__main__':unittest.main()
