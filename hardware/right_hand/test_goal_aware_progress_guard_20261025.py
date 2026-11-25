import json,unittest
from pathlib import Path
import numpy as np
from goal_aware_progress_guard_20261025 import GoalAwareProgress,GoalAwareProgressFault
ROOT=Path(__file__).resolve().parent

class Tests(unittest.TestCase):
    def test_saved_actual_trace_does_not_false_trip(self):
        rows=json.loads((ROOT/'reverse_quarter_path_empty_20261025_082749_355962/result.json').read_bytes())['samples']
        guard=GoalAwareProgress()
        for row in rows:guard.check(row['phase'],np.arange(16),row['goal'],row['actual'],row['t'])
    def test_real_block_with_advancing_target_trips(self):
        guard=GoalAwareProgress()
        with self.assertRaises(GoalAwareProgressFault):
            for i in range(100):
                goal=np.full(16,1064+2*i);actual=np.full(16,1000)
                guard.check('motion',[7],goal,actual,i*.05)
    def test_slow_real_block_still_trips(self):
        guard=GoalAwareProgress()
        with self.assertRaises(GoalAwareProgressFault):
            for i in range(110):guard.check('motion',[7],np.full(16,1064+int(i*.2)),np.full(16,1000),i*.05)
    def test_static_goal_error_is_not_a_motion_request(self):
        guard=GoalAwareProgress()
        for i in range(200):guard.check('hold',[7],np.full(16,1080),np.full(16,1000),i*.05)
    def test_slow_gravity_bias_with_measured_progress_is_not_a_stall(self):
        guard=GoalAwareProgress()
        for i in range(400):guard.check('motion',[7],np.full(16,1100+int(i*.2)),np.full(16,1000+int(i*.18)),i*.05)
    def test_goal_reversal_resets_direction(self):
        guard=GoalAwareProgress()
        for i in range(50):
            goal=1000+min(i,25) if i<=25 else 1025-(i-25)
            guard.check('motion',[7],np.full(16,goal),np.full(16,1000),i*.05)

if __name__=='__main__':unittest.main()
