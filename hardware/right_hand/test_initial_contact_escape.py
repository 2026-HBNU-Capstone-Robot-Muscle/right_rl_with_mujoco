"""Geometric escape rules; no simulator or motor interfaces."""
import unittest
from initial_contact_escape import ContactEscape

PAIR='palm_collision_6|rf_tip'

class Tests(unittest.TestCase):
    def make(self,*,initial=None,monotonic=True):
        return ContactEscape({PAIR:1.3} if initial is None else initial,[PAIR],2.,40,monotonic=monotonic)

    def test_existing_contact_decreases_and_separates(self):
        rule=self.make()
        rule.observe({PAIR:.6},10);rule.observe({},20);rule.observe({},100)
        r=rule.result();self.assertTrue(r['self_escape_complete'])
        self.assertEqual(r['self_escape_complete_at'],20)
        self.assertEqual(r['self_escape_max_unapproved_overlap_mm'],0)

    def test_unlisted_or_too_deep_initial_contact_is_not_excused(self):
        for initial in ({'thumb|palm':1.},{PAIR:2.1}):
            rule=self.make(initial=initial);rule.observe({},10)
            self.assertGreater(rule.result()['self_escape_max_unapproved_overlap_mm'],0)

    def test_static_increase_new_contact_and_recontact_are_rejected(self):
        for mode in ('increase','new','recontact'):
            rule=self.make();rule.observe({PAIR:.5},1)
            rule.observe({PAIR:.6} if mode=='increase' else {'thumb|palm':.1} if mode=='new' else {},2)
            if mode=='recontact':rule.observe({PAIR:.2},3)
            self.assertGreater(rule.result()['self_escape_max_unapproved_overlap_mm'],0)

    def test_dynamics_can_rebound_below_initial_but_must_clear_by_deadline(self):
        rule=self.make(monotonic=False)
        rule.observe({PAIR:.2},1);rule.observe({PAIR:.3},2);rule.observe({},3)
        self.assertEqual(rule.result()['self_escape_max_unapproved_overlap_mm'],0)
        rule.observe({PAIR:.1},41)
        self.assertGreater(rule.result()['self_escape_max_unapproved_overlap_mm'],0)
        rule=self.make(monotonic=False);rule.observe({PAIR:1.31},1)
        self.assertGreater(rule.result()['self_escape_max_unapproved_overlap_mm'],0)

    def test_no_completion_or_nonfinite_read_cannot_pass(self):
        rule=self.make();self.assertFalse(rule.result()['self_escape_complete'])
        rule.observe({PAIR:.1},41)
        self.assertFalse(rule.result()['self_escape_complete'])
        self.assertGreater(rule.result()['self_escape_max_unapproved_overlap_mm'],0)
        with self.assertRaises(ValueError):self.make(initial={PAIR:float('nan')})
        with self.assertRaises(ValueError):self.make().observe({PAIR:float('inf')},1)

    def test_approximate_depth_can_be_diagnostic_in_short_certificate_mode(self):
        rule=ContactEscape({PAIR:2.17},[PAIR],None,40,monotonic=True)
        rule.observe({PAIR:1.},10);rule.observe({},30)
        self.assertEqual(rule.result()['self_escape_max_unapproved_overlap_mm'],0)
        self.assertEqual(rule.result()['initial_self_contacts_mm'][PAIR],2.17)
        rule.observe({PAIR:.01},31)
        self.assertGreater(rule.result()['self_escape_max_unapproved_overlap_mm'],0)

if __name__=='__main__':unittest.main()
