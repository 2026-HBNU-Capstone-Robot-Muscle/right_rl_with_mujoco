"""Verify the tiny escape prefix and strict scope of its initial contact."""
import unittest
import numpy as np
from thumb_clear_root_empty_plan_20261026 import (ThumbRootEmptyPlan,SlowForwardPlan,FIXTURE,RING_RELEASE,
    OLD_TURN,OLD_FINISH,OLD_HOLD,TURN_PHASE,FINISH_PHASE)
from confirmed_thumb_rest_policy_20261026 import (prefix,metadata,REFERENCE,PAIR,MAX_INITIAL_MM,
    classify_initial_palm,verify_palm_body)
from initial_contact_escape import ContactEscape

class ConfirmedEscapeChecks(unittest.TestCase):
    def test_small_prefix_joins_the_original_release_endpoint(self):
        p=ThumbRootEmptyPlan(FIXTURE/'request.json');a=p.source_position();escape=prefix(a)
        self.assertEqual(len(escape),6)
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([a]+escape),axis=0))),2)
        self.assertLessEqual(np.max(abs(np.asarray(escape)-a)),12)
        rows=p.plan(a);old=SlowForwardPlan.plan(p,a)
        old_count=sum(phase==RING_RELEASE for phase,q in old)
        new_count=sum(phase==RING_RELEASE for phase,q in rows)
        np.testing.assert_array_equal([q for phase,q in rows[:6]],escape)
        np.testing.assert_array_equal(rows[new_count-1][1],old[old_count-1][1])
        stop=next(i for i,(phase,q) in enumerate(old) if phase==OLD_HOLD)
        rename={OLD_TURN:TURN_PHASE,OLD_FINISH:FINISH_PHASE}
        expected=[(rename.get(phase,phase),q) for phase,q in old[old_count:stop]]
        self.assertEqual([phase for phase,q in rows[new_count:new_count+len(expected)]],[phase for phase,q in expected])
        np.testing.assert_array_equal([q for phase,q in rows[new_count:new_count+len(expected)]],[q for phase,q in expected])
        self.assertTrue({0,1,2,3}.issubset(set(p.release_indices())))
    def test_confirmed_face_scope_is_limited_to_this_rest_family(self):
        current=metadata(REFERENCE);other=REFERENCE.copy();other[0]+=33
        self.assertIn(PAIR,current['additional_initial_pairs'])
        self.assertNotIn(PAIR,metadata(other)['additional_initial_pairs'])
        self.assertEqual(prefix(other),[])
        self.assertEqual(current['confirmed_initial_palm_body']['maximum_initial_mm'],MAX_INITIAL_MM)
    def test_even_confirmed_initial_contact_cannot_deepen(self):
        guard=ContactEscape({PAIR:.2},{PAIR},.35,10,monotonic=True)
        guard.observe({PAIR:.21},1)
        self.assertGreater(guard.unapproved,1e-6)
    def test_contact_cannot_reappear_or_move_to_an_unknown_pair(self):
        guard=ContactEscape({PAIR:.2},{PAIR},.35,10,monotonic=True)
        guard.observe({},1);guard.observe({PAIR:.001,'palm_collision_2|th_tip':.001},2)
        self.assertGreater(guard.unapproved,1e-6)

class PalmBodyChecks(unittest.TestCase):
    def classify(self,pairs,**kwargs):
        return classify_initial_palm(pairs,enabled=True,initial_release=True,**kwargs)
    def test_sliding_between_same_palm_faces_is_one_shallowing_contact(self):
        initial={'palm_collision_6|th_tip':3.8,'palm_collision_4|th_tip':.3}
        after={'palm_collision_6|th_tip':3.2,'palm_collision_4|th_tip':.47}
        saved=initial.copy();a=self.classify(initial);b=self.classify(after)
        self.assertEqual(initial,saved);self.assertEqual(a,{PAIR:3.8});self.assertEqual(b,{PAIR:3.2})
        guard=ContactEscape(a,{PAIR},4.,10,monotonic=True);guard.observe(b,1);guard.observe({},2)
        self.assertEqual(guard.unapproved,0);self.assertTrue(guard.result()['self_escape_complete'])
    def test_deeper_palm_contact_is_still_rejected(self):
        a=self.classify({'palm_collision_6|th_tip':3.8})
        b=self.classify({'palm_collision_4|th_tip':3.81})
        guard=ContactEscape(a,{PAIR},4.,10,monotonic=True);guard.observe(b,1)
        self.assertGreater(guard.unapproved,0)
    def test_other_finger_contacts_are_retained_and_rejected(self):
        a=self.classify({'palm_collision_6|th_tip':3.8})
        b=self.classify({'palm_collision_4|th_tip':3.2,'mf_tip|th_tip':.001})
        self.assertEqual(b['mf_tip|th_tip'],.001)
        guard=ContactEscape(a,{PAIR},4.,10,monotonic=True);guard.observe(b,1)
        self.assertGreater(guard.unapproved,0)
    def test_after_release_and_outside_scope_faces_are_not_grouped(self):
        raw={'palm_collision_4|th_tip':.001}
        self.assertEqual(classify_initial_palm(raw,enabled=True,phase='stage_3_empty_return'),raw)
        self.assertEqual(classify_initial_palm(raw,enabled=False,initial_release=True),raw)
    def test_same_palm_contact_cannot_start_after_an_empty_initial_state(self):
        guard=ContactEscape({}, {PAIR},4.,10,monotonic=True)
        guard.observe(self.classify({'palm_collision_4|th_tip':.001}),1)
        self.assertGreater(guard.unapproved,0)
    def test_a_mesh_piece_on_another_body_cannot_be_grouped(self):
        from types import SimpleNamespace
        class Model:
            ngeom=4
            names=['palm_collision_3','palm_collision_4','palm_collision_6','palm_collision_7']
            def geom(self,index):
                name=self.names[index] if isinstance(index,int) else index
                return SimpleNamespace(name=name,bodyid=1 if name=='palm_collision_4' else 0)
            def body(self,name):return SimpleNamespace(id=0)
        with self.assertRaisesRegex(ValueError,'different rigid bodies'):verify_palm_body(Model())
    def test_native_one_element_body_arrays_are_verified(self):
        from types import SimpleNamespace
        class Model:
            ngeom=4
            names=['palm_collision_3','palm_collision_4','palm_collision_6','palm_collision_7']
            def geom(self,index):
                name=self.names[index] if isinstance(index,int) else index
                return SimpleNamespace(name=name,bodyid=np.array([7],dtype=np.int32))
            def body(self,name):return SimpleNamespace(id=7)
        identity=verify_palm_body(Model())
        self.assertTrue(identity['verified_same_rigid_body']);self.assertEqual(len(identity['geom_names']),4)
if __name__=='__main__':unittest.main()
