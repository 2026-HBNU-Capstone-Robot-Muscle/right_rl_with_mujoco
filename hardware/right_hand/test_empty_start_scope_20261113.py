import json
import unittest
from pathlib import Path
from empty_start_thumb_index_scope_20261113 import EmptyInitialScope,PAIR,validate_report
from new_pair_grasp_plan_20261027 import ROOTS,OPEN,SETTLE
ROOT=Path(__file__).resolve().parent

class Tests(unittest.TestCase):
    def test_actual_saved_path_initial_overlap_clears(self):
        rows=json.loads((ROOT/'right_hand_rotation_20261112/tip_first_empty_initial_scope/trace.json').read_bytes())['rows']
        s=EmptyInitialScope({PAIR:rows[0]['depth_mm']},False,mode='static')
        for row in rows:
            pairs={PAIR:row['depth_mm']} if row['depth_mm']>0 else {}
            self.assertFalse(s.filtered(row['phase'],pairs))
        self.assertTrue(s.valid);self.assertLessEqual(s.last_positive_open,48)
    def test_deeper_contact_rejected(self):
        s=EmptyInitialScope({PAIR:.94},False,mode='static')
        self.assertIn(PAIR,s.filtered(ROOTS,{PAIR:.95}));s.filtered(SETTLE,{})
        self.assertFalse(s.valid)
    def test_new_contact_is_not_admitted(self):
        s=EmptyInitialScope({},False,mode='static')
        self.assertIn(PAIR,s.filtered(ROOTS,{PAIR:.1}));s.filtered(SETTLE,{})
        self.assertFalse(s.valid)
    def test_other_pair_never_filtered(self):
        s=EmptyInitialScope({PAIR:.94},False,mode='static')
        self.assertEqual(s.filtered(ROOTS,{PAIR:.9,'different_pair':.01}),{'different_pair':.01})
    def test_out_of_scope_loaded_and_late_open_rejected(self):
        for mode,count in [('static',49),('dynamic',721)]:
            s=EmptyInitialScope({PAIR:.94},False,mode=mode)
            for _ in range(count):last=s.filtered(OPEN,{PAIR:.1})
            self.assertIn(PAIR,last);s.filtered(SETTLE,{})
            self.assertFalse(s.valid)
        s=EmptyInitialScope({PAIR:.94},False,mode='static')
        self.assertIn(PAIR,s.filtered('stage_5_loaded_grasp',{PAIR:.1}))
    def test_oversize_and_missing_clearance_rejected(self):
        s=EmptyInitialScope({PAIR:1.76},False,mode='static');s.filtered(SETTLE,{})
        self.assertFalse(s.valid)
        s=EmptyInitialScope({PAIR:.94},False,mode='static');s.filtered(ROOTS,{PAIR:.9})
        self.assertFalse(s.valid)
    def test_fabricated_or_partial_scope_report_rejected(self):
        with self.assertRaises(RuntimeError):validate_report({'static_cases':[{'offset_ticks':[0],'thumb_index_initial_scope':{}},{}]})

if __name__=='__main__':
    r=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    p=ROOT/'right_hand_rotation_20261113/initial_empty_scope/software_scope_tests.json'
    p.write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,failures=len(r.failures),errors=len(r.errors),devices_opened=0),indent=2),encoding='utf-8')
    raise SystemExit(not r.wasSuccessful())
