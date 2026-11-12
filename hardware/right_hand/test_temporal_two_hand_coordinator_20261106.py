"""Behavioral regression tests for phase-specific upper-space access."""
import unittest
from dataclasses import replace
from temporal_two_hand_coordinator_20261106 import Coordinator,Observation,Phase,Plan

def plan(entry='A',exit='B'):
    return Plan('a'*64,entry,exit,'cube-work-frame',90,{p.value:20. for p in (Phase.ROTATE,Phase.REGRASP,Phase.APPROACH,Phase.TURN,Phase.RETREAT)})
def obs(now,ready=True):
    return Observation(now,frozenset(range(1,17)),True,ready,ready,ready,ready,1. if ready else -3.,'cube-work-frame')
def at_regrasp():
    c=Coordinator('A');first=c.start(plan(),0,left_clear_confirmed=True)[0];second=c.receive(first.token,'rotation_complete',1,obs(1,False),left_clear_confirmed=True)[0];return c,second
def left_active():
    c,item=at_regrasp();approach=c.receive(item.token,'lower_grip_and_upper_space_ready',2,obs(2),left_clear_confirmed=True)[0];return c,approach

class TemporalAccessTests(unittest.TestCase):
    def test_whole_rotation_can_use_upper_space(self):
        c,item=at_regrasp();self.assertEqual(c.phase,Phase.REGRASP);self.assertEqual(item.recipient,'right');self.assertFalse(c.history[0]['payload']['upper_clearance_required'])
    def test_no_left_access_until_space_and_grip_ready(self):
        c,item=at_regrasp();self.assertEqual(c.receive(item.token,'lower_grip_and_upper_space_ready',2,obs(2,False),left_clear_confirmed=True),[]);self.assertEqual(c.phase,Phase.REGRASP)
    def test_wrong_clearance_frame_cannot_grant_access(self):
        c,item=at_regrasp();bad=replace(obs(2),clearance_frame='camera-frame');self.assertEqual(c.receive(item.token,'lower_grip_and_upper_space_ready',2,bad,left_clear_confirmed=True),[]);self.assertTrue(c.left_clear)
    def test_stale_or_incomplete_health_cannot_grant_access(self):
        for bad in (obs(1),replace(obs(2),motor_ids=frozenset(range(1,16)))):
            c,item=at_regrasp();self.assertEqual(c.receive(item.token,'lower_grip_and_upper_space_ready',2,bad,left_clear_confirmed=True),[])
    def test_cube_pose_confirmation_required_even_with_clear_space(self):
        c,item=at_regrasp();self.assertEqual(c.receive(item.token,'lower_grip_and_upper_space_ready',2,replace(obs(2),target_cube_pose_confirmed=False),left_clear_confirmed=True),[])
    def test_clearance_loss_during_retreat_also_stops(self):
        c,item=left_active();turn=c.receive(item.token,'at_turn_start',3,obs(3))[0];retreat=c.receive(turn.token,'layer_turn_complete',4,obs(4))[0];stops=c.tick(4.1,replace(obs(4.1),upper_sweep_gap_mm=.2));self.assertEqual(c.phase,Phase.STOPPED);self.assertEqual({s.recipient for s in stops},{'right','left'});self.assertEqual(c.start(plan(),5,left_clear_confirmed=True),[])
    def test_next_right_motion_waits_for_left_retreat(self):
        c,item=left_active();self.assertEqual(c.start(plan('B','C'),2.1,left_clear_confirmed=True),[]);turn=c.receive(item.token,'at_turn_start',3,obs(3))[0];retreat=c.receive(turn.token,'layer_turn_complete',4,obs(4))[0];self.assertEqual(c.receive(retreat.token,'left_clear_of_cube',5,obs(5)),[]);self.assertEqual(c.phase,Phase.RETREAT);c.receive(retreat.token,'left_clear_of_cube',5.1,obs(5.1),left_clear_confirmed=True);self.assertEqual(c.phase,Phase.READY);self.assertTrue(c.start(plan('B','C'),6,left_clear_confirmed=True))
    def test_mismatched_next_entry_rejected(self):
        c=Coordinator('B');self.assertEqual(c.start(plan('A','C'),0,left_clear_confirmed=True),[]);self.assertEqual(c.phase,Phase.IDLE)
    def test_duplicate_and_out_of_order_ack_does_not_move(self):
        c,item=at_regrasp();self.assertEqual(c.receive(item.token,'layer_turn_complete',2,obs(2)),[]);approach=c.receive(item.token,'lower_grip_and_upper_space_ready',2,obs(2),left_clear_confirmed=True)[0];self.assertEqual(c.receive(item.token,'lower_grip_and_upper_space_ready',2.1,obs(2.1),left_clear_confirmed=True),[]);self.assertEqual(c.pending,approach.token)
    def test_left_entering_during_right_motion_latches_stop(self):
        c=Coordinator('A');item=c.start(plan(),0,left_clear_confirmed=True)[0];self.assertEqual(len(c.receive(item.token,'rotation_complete',1,obs(1,False))),2);self.assertEqual(c.phase,Phase.STOPPED)
    def test_nan_and_negative_gap_never_ready(self):
        for gap in (float('nan'),float('inf'),-.1,.3):self.assertFalse(replace(obs(1),upper_sweep_gap_mm=gap).handoff_ready(1,'cube-work-frame'))
    def test_timeout_not_converted_to_automatic_retry(self):
        c=Coordinator('A');c.start(plan(),0,left_clear_confirmed=True);stops=c.tick(21,obs(21));self.assertEqual(c.phase,Phase.STOPPED);self.assertTrue(all(s.payload['automatic_retry'] is False for s in stops));self.assertEqual(c.tick(22,obs(22)),[])

if __name__=='__main__':unittest.main()
