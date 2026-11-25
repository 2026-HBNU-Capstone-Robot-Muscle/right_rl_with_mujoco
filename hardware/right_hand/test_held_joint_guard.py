"""Real stopped-run replay plus unexpected-movement cases; no motor access."""
import json
import unittest
from pathlib import Path
import numpy as np
from held_joint_guard import HeldJointGuard,HeldJointFault
from grasp_approach_plan import GraspApproachPlan,admission

ROOT=Path(__file__).resolve().parent


class Tests(unittest.TestCase):
    def test_recorded_stop_was_existing_offset_not_new_motion(self):
        r=json.loads((ROOT/'grasp_approach_current_empty_20261022_015038_969787/result.json').read_bytes())
        samples=r['samples'];enabled=r['enable_snapshots'][-1]
        guard=HeldJointGuard(enabled['position'],r['accepted_enable_goal'])
        phase_masks=object.__new__(GraspApproachPlan)
        old_failures=[]
        for i,s in enumerate(samples):
            frozen=phase_masks.frozen_for_phase(s['phase'])
            if np.any(abs(np.array(s['actual'])[frozen]-np.array(s['goal'])[frozen])>20):old_failures.append(i)
            guard.check(s['actual'],s['goal'],frozen,s['phase'])
            if i+1<len(samples):guard.advance(s['actual'],s['goal'],samples[i+1]['goal'])
        self.assertEqual(old_failures,[len(samples)-1])
        self.assertEqual(samples[-1]['phase'],'stage_3_ring_clearance')
        for index,error in ((7,-29),(11,-25)):
            self.assertEqual(samples[-1]['actual'][index]-samples[-1]['goal'][index],error)
            self.assertLessEqual(abs(samples[-1]['actual'][index]-samples[-2]['actual'][index]),1)

    def test_prior_lag_settling_allowed_but_21_tick_extra_drift_stops(self):
        command=np.full(16,2472);position=command.copy();position[7]-=29
        guard=HeldJointGuard(position,command)
        for value in (2443,2442,2460,2472,2492):
            actual=position.copy();actual[7]=value
            guard.check(actual,command,[7],'hold')
        for value in (2422,2493):
            actual=position.copy();actual[7]=value
            with self.assertRaises(HeldJointFault) as caught:guard.check(actual,command,[7],'hold')
            self.assertEqual(caught.exception.details['joints'][0]['id'],8)

    def test_fixed_joint_reference_not_reset_by_other_joints_or_phase(self):
        command=np.full(16,2500);guard=HeldJointGuard(command,command)
        for i in range(3):
            position=command.copy();position[7]+=i*10
            new=command.copy();new[0]+=2
            guard.advance(position,command,new);command=new
        position=command.copy();position[7]+=21
        with self.assertRaises(HeldJointFault):guard.check(position,command,[7],'new_phase')

    def test_new_command_uses_prewrite_measurement_and_rejects_wrong_order(self):
        command=np.full(16,2500);guard=HeldJointGuard(command,command)
        new=command.copy();new[7]+=2
        guard.advance(command,command,new)
        position=new.copy();position[7]=2523
        with self.assertRaises(HeldJointFault):guard.check(position,new,[7],'hold')
        with self.assertRaises(ValueError):guard.advance(new,command,new)

    def test_latest_read_only_off_pose_is_admitted_but_not_motion_approved(self):
        r=json.loads((ROOT/'grasp_approach_current_empty_20261022_015140_792784/result.json').read_bytes())
        raw=r['preflight_state']['position'];self.assertEqual(raw[14],2700)
        np.testing.assert_array_equal(admission(raw),raw)
        bad=np.array(raw);bad[14]=2625+513
        with self.assertRaises(ValueError):admission(bad)
        bad=np.array(raw);bad[2]-=4096
        with self.assertRaises(ValueError):admission(bad)


if __name__=='__main__':unittest.main()
