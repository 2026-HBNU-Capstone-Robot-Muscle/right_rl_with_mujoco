"""Mechanical scope and initial-only separation, with no native/device API."""
import copy
from types import SimpleNamespace
import unittest
import numpy as np
from book_support_contact_scope import derive_scope,allowed_initial_pairs,load_scope,validate_scope
from initial_contact_escape import ContactEscape
from book_empty_preparation_gate import screen_failure_details
import json
from pathlib import Path
import tempfile


def model():
    names=['support_book','support_glass','support_wood','index_root_proxy','if_md_collision_5',
           'middle_capsule','rf_tip','th_tip','if_misleading_on_thumb','palm_collision',None,'if_self_only']
    bodies=['world','palm','index_root','index_tip','middle_root','middle_tip','ring_root','ring_tip','thumb_root','thumb_tip']
    m=SimpleNamespace(ngeom=len(names),jnt_bodyid=np.array([2,4,6]),
        geom_bodyid=np.array([0,0,0,2,3,4,7,9,9,1,3,3]),
        body_parentid=np.array([0,0,1,2,1,4,1,6,1,8]),
        geom_contype=np.array([4,4,4,1,1,1,1,1,1,1,0,16]),
        geom_conaffinity=np.array([11,11,11,4,4,4,4,4,4,4,0,32]))
    m.joint=lambda name:SimpleNamespace(id=['if_mcp','mf_mcp','rf_mcp'].index(name))
    m.geom=lambda key:SimpleNamespace(id=names.index(key),name=key) if isinstance(key,str) else SimpleNamespace(id=key,name=names[key])
    m.body=lambda key:SimpleNamespace(name=bodies[key])
    return m


class Tests(unittest.TestCase):
    def test_body_ancestry_and_collision_masks_determine_scope(self):
        scope=derive_scope(model());pairs=allowed_initial_pairs(scope)
        self.assertIn('index_root_proxy|support_glass',pairs)
        self.assertIn('if_md_collision_5|support_glass',pairs)
        self.assertIn('rf_tip|support_book',pairs)
        self.assertNotIn('support_glass|th_tip',pairs)
        self.assertNotIn('if_misleading_on_thumb|support_glass',pairs)
        self.assertNotIn('palm_collision|support_book',pairs)
        self.assertNotIn('if_self_only|support_glass',pairs)

    def test_actual_native_table_contains_three_clearing_fingers_only(self):
        scope=load_scope();pairs=allowed_initial_pairs(scope)
        self.assertEqual(set(scope['root_groups']),{'8','12','16'})
        self.assertIn('if_md_collision_5|support_glass',pairs)
        self.assertFalse(any('th_' in pair or 'palm' in pair for pair in pairs))

    def test_small_initial_contact_must_decrease_and_separate(self):
        pair='if_md_collision_5|support_glass'
        escape=ContactEscape({pair:.038164},allowed_initial_pairs(),4.,40,monotonic=True)
        escape.observe({pair:.01},1);escape.observe({},2)
        self.assertEqual(escape.unapproved,0.)
        self.assertTrue(escape.result()['self_escape_complete'])

    def test_new_deeper_late_or_thumb_contacts_are_still_rejected(self):
        pair='if_md_collision_5|support_glass';old='if_bs_visual_support_proxy|support_glass'
        cases=[({old:.5},{pair:.001},1),({pair:.01},{pair:.02},1),
               ({pair:.01},{pair:.001},41),({'support_glass|th_tip':.001},{},1),
               ({pair:4.01},{},1)]
        for initial,observed,at in cases:
            escape=ContactEscape(initial,allowed_initial_pairs(),4.,40,monotonic=True)
            escape.observe(observed,at)
            with self.subTest(initial=initial,observed=observed,at=at):self.assertGreater(escape.unapproved,0)

    def test_inconsistent_scope_and_unnamed_active_colliders_fail(self):
        scope=derive_scope(model());scope['root_groups']['8']['root_joint']='th_cmc'
        with self.assertRaises(ValueError):validate_scope(scope)
        m=model();m.geom_contype[10]=1;m.geom_conaffinity[10]=4
        with self.assertRaises(ValueError):derive_scope(m)

    def test_failure_message_explains_actual_contact_and_range_failure(self):
        r=dict(code_unchanged=True,static_screens=[dict(initial_support_contacts_mm={'support_glass|th_tip':.01},
            support_escape_max_unapproved_overlap_mm=.01,native_range_violation_rad=.01)],dynamic_sensitivity=[])
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'screen.json';p.write_text(json.dumps(r))
            text=screen_failure_details(p)
        self.assertIn('outside the lifted fingers',text)
        self.assertIn('native joint range',text)


if __name__=='__main__':unittest.main()
