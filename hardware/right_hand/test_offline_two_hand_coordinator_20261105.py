import unittest
from offline_two_hand_coordinator_20261105 import Coordinator,CyclePlan,Phase,RightHealth,FutureLiveEvidence

def plan(entry='placed_open',exit='common_hold'):return CyclePlan('offline_source_only','a'*64,'common_cube_frame_unregistered',90,{p.value:5. for p in (Phase.RIGHT,Phase.LEFT_APPROACH,Phase.LEFT_TURN,Phase.LEFT_RETREAT)},entry,exit)
def health(t,**changes):
    values=dict(received_at_s=t,ids=frozenset(range(1,17)),healthy=True,targets_constant=True,settled=True,cube_held_confirmed=True);values.update(changes);return RightHealth(**values)

class CoordinatorTests(unittest.TestCase):
    def setUp(self):self.c=Coordinator(initial_grip_id='placed_open');self.first=self.c.start(plan(),0.,left_clear_confirmed=True)[0]
    def right_ready(self):return self.c.receive(self.c.pending,'rotation_complete_hold_ready',1.,right_health=health(1.))[0]
    def test_happy_cycle(self):
        first=self.right_ready();turn=self.c.receive(first.token,'at_turn_start',2.,right_health=health(2.))[0];retreat=self.c.receive(turn.token,'layer_turn_complete',3.,right_health=health(3.))[0];self.assertEqual(self.c.receive(retreat.token,'clear_of_cube',4.,left_clear_confirmed=True),[]);self.assertEqual(self.c.phase,Phase.COMPLETE);self.assertTrue(self.c.left_clear)
    def test_rotation_denied_while_left_present(self):
        self.right_ready();self.assertEqual(self.c.start(plan(),2.,left_clear_confirmed=False),[])
    def test_wrong_token_does_not_advance(self):
        self.assertEqual(self.c.receive('wrong','rotation_complete_hold_ready',1.,right_health=health(1.)),[]);self.assertEqual(self.c.phase,Phase.RIGHT)
    def test_wrong_phase_does_not_advance(self):
        self.assertEqual(self.c.receive(self.first.token,'layer_turn_complete',1.,right_health=health(1.)),[]);self.assertEqual(self.c.phase,Phase.RIGHT)
    def test_stale_health_blocks_left(self):
        self.assertEqual(self.c.receive(self.first.token,'rotation_complete_hold_ready',1.,right_health=health(.7)),[]);self.assertEqual(self.c.phase,Phase.RIGHT)
    def test_missing_motor_blocks_left(self):
        self.assertEqual(self.c.receive(self.first.token,'rotation_complete_hold_ready',1.,right_health=health(1.,ids=frozenset(range(1,16)))),[])
    def test_unhealthy_right_blocks_left(self):
        self.assertEqual(self.c.receive(self.first.token,'rotation_complete_hold_ready',1.,right_health=health(1.,healthy=False)),[])
    def test_unsettled_right_blocks_left(self):
        self.assertEqual(self.c.receive(self.first.token,'rotation_complete_hold_ready',1.,right_health=health(1.,settled=False)),[])
    def test_future_dated_health_blocks_left(self):
        self.assertEqual(self.c.receive(self.first.token,'rotation_complete_hold_ready',1.,right_health=health(1.01)),[])
    def test_duplicate_ack_does_not_resend(self):
        self.right_ready();self.assertEqual(self.c.receive(self.first.token,'rotation_complete_hold_ready',1.1,right_health=health(1.1)),[]);self.assertEqual(self.c.phase,Phase.LEFT_APPROACH)
    def test_layer_completion_only_orders_left_retreat(self):
        first=self.right_ready();turn=self.c.receive(first.token,'at_turn_start',2.,right_health=health(2.))[0];actions=self.c.receive(turn.token,'layer_turn_complete',3.,right_health=health(3.));self.assertEqual([a.operation for a in actions],['retreat_clear_of_cube']);self.assertTrue(all(a.recipient=='left' for a in actions))
    def test_clear_ack_needs_clearance_confirmation(self):
        first=self.right_ready();turn=self.c.receive(first.token,'at_turn_start',2.,right_health=health(2.))[0];retreat=self.c.receive(turn.token,'layer_turn_complete',3.,right_health=health(3.))[0];self.assertEqual(self.c.receive(retreat.token,'clear_of_cube',4.,left_clear_confirmed=False),[]);self.assertEqual(self.c.phase,Phase.LEFT_RETREAT)
    def test_fault_latches_and_no_retry(self):
        actions=self.c.fault('existing controller health fault');self.assertEqual([a.operation for a in actions],['stop_requested','stop_requested']);self.assertEqual(self.c.fault('again'),[]);self.assertEqual(self.c.start(plan(),2.,left_clear_confirmed=True),[])
    def test_reset_requires_explicit_OFF_and_clearance(self):
        self.c.fault('fault');self.assertFalse(self.c.reset(explicit_operator_reset=True,right_OFF_verified=True,left_OFF_verified=False,left_clear_confirmed=True));self.assertTrue(self.c.reset(explicit_operator_reset=True,right_OFF_verified=True,left_OFF_verified=True,left_clear_confirmed=True));self.assertEqual(self.c.phase,Phase.IDLE);self.assertIsNone(self.c.pending)
    def test_timeout_no_automatic_motion_retry(self):
        self.assertEqual(len(self.c.tick(6.)),2);self.assertEqual(self.c.phase,Phase.STOPPED);self.assertEqual(self.c.tick(7.),[])
    def test_offline_review_alone_is_not_live_admission(self):
        e=FutureLiveEvidence(offline_model_review_passed=True);self.assertFalse(e.live_ready());self.assertIn('physical_grip_verified',e.missing())
    def test_held_cube_is_not_proof_of_target_orientation(self):
        from dataclasses import asdict
        values={k:True for k in asdict(FutureLiveEvidence())};values['physical_target_orientation_verified']=False
        e=FutureLiveEvidence(**values);self.assertFalse(e.live_ready());self.assertEqual(e.missing(),['physical_target_orientation_verified'])
    def test_evidence_flags_do_not_create_hardware_transport(self):
        from dataclasses import asdict
        e=FutureLiveEvidence(**{k:True for k in asdict(FutureLiveEvidence())});self.assertTrue(e.live_ready());self.assertTrue(self.first.simulated)
    def test_intents_are_explicitly_simulated(self):self.assertTrue(self.first.simulated)
    def test_time_budgets_are_required(self):
        with self.assertRaises(ValueError):CyclePlan('source','a'*64,'frame',90,{})
    def test_no_implicit_entry_grip(self):
        c=Coordinator();self.assertEqual(c.start(plan(),0.,left_clear_confirmed=True),[])
    def test_completion_cannot_reuse_wrong_entry_path(self):
        first=self.right_ready();turn=self.c.receive(first.token,'at_turn_start',2.,right_health=health(2.))[0];retreat=self.c.receive(turn.token,'layer_turn_complete',3.,right_health=health(3.))[0];self.c.receive(retreat.token,'clear_of_cube',4.,left_clear_confirmed=True);self.assertEqual(self.c.current_grip_id,'common_hold');self.assertEqual(self.c.start(plan(),4.1,left_clear_confirmed=True),[])
    def test_matching_reviewed_entry_is_required_for_next_cycle(self):
        first=self.right_ready();turn=self.c.receive(first.token,'at_turn_start',2.,right_health=health(2.))[0];retreat=self.c.receive(turn.token,'layer_turn_complete',3.,right_health=health(3.))[0];self.c.receive(retreat.token,'clear_of_cube',4.,left_clear_confirmed=True);actions=self.c.start(plan(entry='common_hold'),4.1,left_clear_confirmed=True);self.assertEqual([a.operation for a in actions],['rotate_then_hold'])
    def test_late_ack_times_out_without_scheduler_tick(self):
        actions=self.c.receive(self.first.token,'rotation_complete_hold_ready',6.,right_health=health(6.));self.assertEqual([a.operation for a in actions],['stop_requested','stop_requested']);self.assertEqual(self.c.phase,Phase.STOPPED)
    def test_missing_entry_or_exit_is_rejected(self):
        with self.assertRaises(ValueError):CyclePlan('source','a'*64,'frame',90,plan().timeout_s)
    def test_reset_does_not_reuse_uncertain_old_grip(self):
        self.c.fault('fault');self.c.reset(explicit_operator_reset=True,right_OFF_verified=True,left_OFF_verified=True,left_clear_confirmed=True);self.assertIsNone(self.c.current_grip_id);self.assertEqual(self.c.start(plan(),1.,left_clear_confirmed=True),[])
if __name__=='__main__':unittest.main()
