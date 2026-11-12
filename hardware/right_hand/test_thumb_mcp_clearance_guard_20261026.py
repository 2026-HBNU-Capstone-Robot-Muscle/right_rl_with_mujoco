"""Replay the actual22tick stop and retain freshness, clearance and health checks."""
import json,time,unittest
from unittest.mock import patch
import numpy as np
import sequence_flip_checked as checked
from thumb_mcp_clearance_guard_20261026 import verify
from leap_recovered_forward_placement_20261026 import RecoveredForwardBus,EmptyFinishBus,LoadedFinishBus
from recovered_forward_plan_20261026 import ROOT

class McpClearanceChecks(unittest.TestCase):
    def setUp(self):
        self.r=json.loads((ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/mcp_checkpoint_stop_0348/review.json').read_bytes())
        actual=json.loads((ROOT/self.r['source']/'result.json').read_bytes())['samples'][-1]
        self.state=dict(position=np.array(actual['actual']),goal=np.array(actual['goal']))
        self.command=np.array(actual['goal']);self.policy=self.r['new_clearance_policy']

    def test_actual22tick_lag_is_already_beyond_validated_release(self):
        self.assertEqual(self.state['position'][1]-self.command[1],22)
        report={};verify(self.state,self.command,self.policy,report,read_at=1.,now=lambda:1.01)
        self.assertTrue(report['thumb_mcp_release_measured_verified'])
        self.assertEqual(report['thumb_mcp_release_goal_error_ticks'],22)
        self.assertLessEqual(report['thumb_mcp_release_actual_ticks'],report['thumb_mcp_release_limit_ticks'])

    def test_not_cleared_position_still_blocks(self):
        self.state['position'][1]=self.policy['threshold_ticks']+1
        with self.assertRaisesRegex(RuntimeError,'validated release position'):
            verify(self.state,self.command,self.policy,{},read_at=1.,now=lambda:1.01)

    def test_stale_snapshot_still_blocks(self):
        with self.assertRaisesRegex(RuntimeError,'fresh measured'):
            verify(self.state,self.command,self.policy,{},read_at=1.,now=lambda:1.21)

    def test_unaccepted_goal_readback_still_blocks(self):
        self.state['goal'][1]+=1
        with self.assertRaisesRegex(RuntimeError,'accepted goal'):
            verify(self.state,self.command,self.policy,{},read_at=1.,now=lambda:1.01)

    def bus(self):
        b=RecoveredForwardBus.__new__(RecoveredForwardBus);b._ring_release_writes=32;b._goal_writes=34
        b.latest_release_state=self.state;b.latest_release_read_at=time.monotonic();b.last_goal_command=self.command
        b._mcp_release_policy=self.policy;b.report={};return b

    def test_phase_transition_skips_only_obsolete20tick_guard(self):
        b=self.bus()
        with patch.object(checked,'health') as health,patch.object(EmptyFinishBus,'write',return_value='accepted') as downstream,patch.object(LoadedFinishBus,'write',side_effect=AssertionError('obsolete guard called')):
            self.assertEqual(b.write(116,4,self.command),'accepted')
            health.assert_called_once();downstream.assert_called_once()

    def test_actual_health_fault_still_prevents_target(self):
        b=self.bus()
        with patch.object(checked,'health',side_effect=RuntimeError('tracking fault')),patch.object(EmptyFinishBus,'write') as downstream:
            with self.assertRaisesRegex(RuntimeError,'tracking fault'):b.write(116,4,self.command)
            downstream.assert_not_called()

    def test_off_is_not_blocked_by_release_condition(self):
        b=self.bus();b.latest_release_state=None
        with patch.object(EmptyFinishBus,'write',return_value='off') as downstream:
            self.assertEqual(b.write(64,1,np.zeros(16,dtype=int)),'off');downstream.assert_called_once()

if __name__=='__main__':unittest.main()
