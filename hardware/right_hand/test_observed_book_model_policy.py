"""Model corrections remain scoped; no extension is a drive-deeper goal."""
import json
import unittest
import tempfile
import numpy as np
from book_grasp_trial_plan import ROOT,BookGraspPlan
from observed_book_model_policy import (admission,classify_self_contacts,policy,
    CONTACT_PAIR,bounds)


class Tests(unittest.TestCase):
    def state(self):
        return json.loads((ROOT/'book_settled_off_readonly_20261024_174424_563949/request.json').read_bytes())['off_state']

    def test_real_passive_pose_is_admitted_but_other_joint_limits_remain(self):
        raw=np.array(self.state()['position']);np.testing.assert_array_equal(admission(raw),raw)
        for motor,value in ((4,int(np.floor(bounds()[0][3]))-1),(8,1843),(1,3274)):
            q=raw.copy();q[motor-1]=value
            with self.subTest(motor=motor),self.assertRaises(ValueError):admission(q)

    def test_small_ring_pair_only_is_classified_without_hiding_larger_or_other_contacts(self):
        other='mf_ds_offline_hull_0|th_ds_offline_hull_0'
        self.assertEqual(classify_self_contacts({CONTACT_PAIR:.0859,other:.00001}),{other:.00001})
        self.assertEqual(classify_self_contacts({CONTACT_PAIR:.10001}),{CONTACT_PAIR:.10001})

    def test_model_lower_bound_uses_the_new_source_instead_of_a_past_pose(self):
        raw=np.array(self.state()['position']);raw[3]=1759
        self.assertEqual(policy(raw)['extended_lower_ticks'],1755)
        self.assertTrue(policy(raw)['model_range_extended'])
        raw[3]=2000
        self.assertEqual(policy(raw)['extended_lower_ticks'],policy(raw)['native_ID4_lower_ticks'])
        self.assertFalse(policy(raw)['model_range_extended'])

    def test_reentry_never_drives_id4_below_the_actual_start(self):
        source=ROOT/'book_settled_off_readonly_20261024_174424_563949/request.json'
        request=json.loads(source.read_bytes());request['plan_version']=BookGraspPlan.PLAN_VERSION
        with tempfile.TemporaryDirectory() as tmp:
            path=__import__('pathlib').Path(tmp)/'request.json'
            path.write_text(json.dumps(request),encoding='utf-8')
            plan=BookGraspPlan(path);raw=plan.source_position()
            values=np.array([q[3] for _,q in plan.plan(raw)])
        self.assertTrue(np.all(values>=raw[3]))
        native_lower=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())['lower_ticks'][3]
        entry=next(i for i,v in enumerate(values) if v>=native_lower)
        self.assertTrue(np.all(np.diff(np.r_[raw[3],values[:entry+1]])>=0))
        self.assertEqual(policy()['extended_motor_id'],4)


if __name__=='__main__':unittest.main()
