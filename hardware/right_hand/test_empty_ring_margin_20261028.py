"""Verify the EMPTY inward waypoint without broadening native ranges."""
import copy,json,tempfile,unittest
from pathlib import Path
import numpy as np
import calibrated_quarter_plan_20261028 as p
import calibrated_empty_ring_margin_20261028 as margin
from loaded_quarter_reentry_20261025 import limits,validate_path
from new_pair_grasp_plan_20261027 import ENTRY,ROOTS,OPEN,SETTLE
REQUEST=p.ROOT/'right_hand_rotation_20261028/reconnected_170143_margin_v4/request.json'
class MarginTests(unittest.TestCase):
    def setUp(self):self.plan=p.HorizontalQuarterPlan(REQUEST);self.rows=self.plan.plan(self.plan.source_position())
    def test_current_near_limit_joint_gets_small_inward_margin(self):
        review=margin.review(self.plan)
        self.assertTrue(review['enabled']);self.assertEqual(review['source_ticks'],1724)
        self.assertEqual(review['target_ticks'],1735);self.assertEqual(review['maximum_command_travel_ticks'],11)
    def test_only_ring_pip_moves_during_margin(self):
        previous=self.plan.source_position();count=0
        for phase,q in self.rows:
            if phase==margin.PHASE:
                count+=1;self.assertTrue(np.array_equal(q[np.arange(16)!=14],previous[np.arange(16)!=14]));self.assertTrue(0<=q[14]-previous[14]<=2)
            previous=q
        self.assertEqual(count,6)
    def test_margin_precedes_all_empty_root_movement(self):
        phases=[phase for phase,q in self.rows]
        self.assertLess(max(i for i,v in enumerate(phases) if v==margin.PHASE),min(i for i,v in enumerate(phases) if v==ROOTS))
        self.assertTrue(all(q[14]>=1735 for phase,q in self.rows if phase==ROOTS))
    def test_native_goal_and_step_criteria_are_unchanged(self):
        values=validate_path(self.plan.source_position(),self.rows,ENTRY);low,high=limits()
        self.assertTrue(np.all(values[:,np.arange(16)!=3]>=low[np.arange(16)!=3]));self.assertTrue(np.all(values<=high))
        self.assertLessEqual(int(np.max(abs(np.diff(values,axis=0)))),2)
    def test_cmc_reentry_still_finishes_before_margin(self):
        low,_=limits();entry=[q[3] for phase,q in self.rows if phase==ENTRY]
        self.assertTrue(all(b>=a for a,b in zip(entry,entry[1:])))
        self.assertTrue(all(q[3]>=low[3] for phase,q in self.rows if phase==margin.PHASE))
    def test_loaded_goal_cycle_and_open_join_unchanged(self):
        empty=[q for phase,q in self.rows if phase==SETTLE]
        loaded=np.load(p.LOADED_PATH)
        self.assertTrue(np.array_equal(empty[-1],loaded['start']))
        self.assertTrue(np.array_equal(loaded['goals'][loaded['phases']=='yaw'][-1],loaded['yaw_goal']))
        self.assertEqual(sum(phase=='stage_7_horizontal_yaw' for phase,q in self.rows),638)
    def test_frozen_and_progress_sets_match_only_the_one_moving_joint(self):
        self.assertEqual(self.plan.progress_for_phase(margin.PHASE).tolist(),[14]);self.assertNotIn(14,self.plan.frozen_for_phase(margin.PHASE));self.assertEqual(len(self.plan.frozen_for_phase(margin.PHASE)),15)
    def test_inward_margin_is_not_added_when_already_inside(self):
        request=json.loads(REQUEST.read_bytes());request['off_state']['position'][14]=1800;request['off_state']['goal'][14]=1800
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'request.json';path.write_bytes(json.dumps(request).encode());plan=p.HorizontalQuarterPlan(path)
            self.assertFalse(margin.review(plan)['enabled']);self.assertFalse(any(phase==margin.PHASE for phase,q in plan.plan(plan.source_position())))
if __name__=='__main__':unittest.main()
