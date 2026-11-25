"""Passive OFF admission never permits deeper motion or later range escape."""
import json
import unittest
import numpy as np
from final_pose_start_scope_20261116 import SOURCE,admission,healthy_off,validate_path
from new_pair_grasp_plan_20261027 import ENTRY

class Tests(unittest.TestCase):
    def setUp(self):self.state=json.loads(SOURCE.read_bytes())['preflight_state'];self.raw=np.asarray(self.state['position'])
    def test_saved_actual_start_is_admitted(self):self.assertTrue(np.array_equal(admission(self.raw),self.raw))
    def test_other_joint_lower_bounds_do_not_change(self):
        from observed_book_model_policy import bounds
        lo,hi=bounds()
        for j in range(16):
            if j in (6,7,11,15):continue
            q=self.raw.copy();q[j]=int(np.floor(lo[j]))-1
            with self.assertRaises(ValueError):admission(q)
    def test_below_scoped_floor_rejected(self):
        for j,lower in ((6,1702),(7,1812),(11,1819),(15,1816)):
            q=self.raw.copy();q[j]=lower-1
            with self.assertRaises(ValueError):admission(q)
    def test_torque_or_error_never_admitted(self):
        for key in ('torque','error','watchdog'):
            state={k:list(v) for k,v in self.state.items()};state[key][0]=1
            with self.assertRaises(ValueError):healthy_off(state)
    def test_deeper_command_is_rejected(self):
        q=self.raw.copy();q[7]-=1
        with self.assertRaises(ValueError):validate_path(self.raw,[(ENTRY,q)],ENTRY)
    def test_phase_cannot_skip_reentry(self):
        with self.assertRaises(ValueError):validate_path(self.raw,[('final_grip_empty_move',self.raw.copy())],ENTRY)
    def test_latest_failed_OFF_start_and_review_radius(self):
        from pathlib import Path
        from book_empty_preparation_plan import offsets
        root=Path(__file__).resolve().parent
        state=json.loads((root/'cube_grasp_hold_20261116_200443_746252/result.json').read_bytes())['preflight_state']
        q=healthy_off(state)
        for delta in offsets():self.assertTrue(np.array_equal(admission(q+delta),q+delta))
    def test_ID7_cannot_move_deeper_or_leave_range_again(self):
        from loaded_quarter_reentry_20261025 import limits
        q=self.raw.copy();q[6]=1706
        deeper=q.copy();deeper[6]-=1
        with self.assertRaises(ValueError):validate_path(q,[(ENTRY,deeper)],ENTRY)
        inside=q.copy();inside[6]=int(np.ceil(limits()[0][6]))
        rows=[];current=q.copy()
        while current[6]<inside[6]:
            current=current.copy();current[6]=min(current[6]+2,inside[6]);rows.append((ENTRY,current))
        back=inside.copy();back[6]-=2;rows.append((ENTRY,back))
        with self.assertRaises(ValueError):validate_path(q,rows,ENTRY)

if __name__=='__main__':unittest.main(verbosity=1)
