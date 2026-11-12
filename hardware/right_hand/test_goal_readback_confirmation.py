"""Offline fault injection; no serial port, camera, or motor writes."""
import contextlib
import io
import unittest
from unittest.mock import patch
import numpy as np
import sequence_flip_checked as checked
import goal_readback_confirmation as confirmation
from test_sequence_voltage import Clock, ReadOnlyBus, state


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.report={'enabled_at':0.}
        self.command=state()['goal'].copy()
        self.previous=self.command.copy();self.previous[0]-=2;self.previous[3]+=2
        self.stack=contextlib.ExitStack()
        self.stack.enter_context(patch.object(checked.time,'monotonic',self.clock.monotonic))
        self.stack.enter_context(patch.object(checked.time,'sleep',self.clock.sleep))
        self.stack.enter_context(patch.object(checked,'check_cancel'))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.addCleanup(self.stack.close)

    def observed(self,goal=None):
        s=state();s['goal']=self.command.copy() if goal is None else np.array(goal)
        s['position']=self.command.copy()
        return s

    def bus(self,states,seconds=.02):
        b=ReadOnlyBus(self.clock,states,seconds)
        b.previous_goal_command=self.previous.copy();b.last_goal_command=self.command.copy()
        b.goal_readback_confirmation=True
        return b

    def read(self,b,**kw):
        return checked.sample(b,self.command,self.report,'loaded',**kw)

    def test_healthy_snapshot_does_not_add_reads(self):
        b=self.bus([self.observed()]);self.read(b)
        self.assertEqual(b.calls,1);self.assertNotIn('goal_readback_events',self.report)

    def test_old_packet_requires_two_exact_fresh_reads_without_writes_or_rebasing(self):
        original=self.command.copy()
        b=self.bus([self.observed(self.previous),self.observed(),self.observed()])
        s,_=self.read(b)
        self.assertEqual(b.calls,3)
        np.testing.assert_array_equal(self.command,original)
        np.testing.assert_array_equal(s['goal'],original)
        np.testing.assert_array_equal(b.last_goal_command,original)
        event=self.report['goal_readback_events'][0]
        self.assertEqual(event['status'],'recovered')
        self.assertEqual(event['confirmation_reads'],2)
        self.assertEqual((event['goal_writes'],event['goals_advanced']),(0,0))
        self.assertEqual([r['goal_readback'] for r in self.report['samples']],
                         [self.previous.tolist(),original.tolist(),original.tolist()])

    def test_one_more_old_read_then_two_exact_fresh_reads(self):
        b=self.bus([self.observed(self.previous)]*2+[self.observed()]*2)
        self.read(b);self.assertEqual(b.calls,4)

    def test_exact_reads_must_be_consecutive(self):
        b=self.bus([self.observed(self.previous),self.observed(),self.observed(self.previous),self.observed()])
        with self.assertRaisesRegex(RuntimeError,'twice within'):self.read(b)
        self.assertEqual(b.calls,4)
        self.assertEqual(self.report['goal_readback_events'][0]['status'],'stopped')

    def test_persistent_previous_goal_is_not_accepted(self):
        b=self.bus([self.observed(self.previous)])
        with self.assertRaisesRegex(RuntimeError,'twice within'):self.read(b)
        self.assertEqual(b.calls,4)

    def test_wrong_small_partial_wrap_or_large_packets_stop_immediately(self):
        for kind in ('wrong','partial','wrap','large'):
            with self.subTest(kind=kind):
                observed=self.previous.copy();b=self.bus([])
                if kind=='wrong':observed[0]+=1
                if kind=='partial':observed[3]=self.command[3]
                if kind=='wrap':observed[0]-=4096
                if kind=='large':
                    observed[0]-=1;b.previous_goal_command=observed.copy()
                b.states=[self.observed(observed),self.observed()]
                with self.assertRaisesRegex(RuntimeError,'exact preceding'):self.read(b)
                self.assertEqual(b.calls,1)

    def test_unexpected_goal_during_confirmation_stops(self):
        wrong=self.command.copy();wrong[0]+=1
        b=self.bus([self.observed(self.previous),self.observed(wrong),self.observed()])
        with self.assertRaisesRegex(RuntimeError,'outside the preceding'):self.read(b)
        self.assertEqual(b.calls,2)

    def test_opt_in_and_issued_packet_metadata_are_required(self):
        b=self.bus([self.observed(self.previous),self.observed()]);b.goal_readback_confirmation=False
        with self.assertRaises(confirmation.GoalReadbackMismatch):self.read(b)
        self.assertEqual(b.calls,1);self.assertNotIn('goal_readback_events',self.report)
        for key in ('previous_goal_command','last_goal_command'):
            b=self.bus([self.observed(self.previous),self.observed()]);delattr(b,key)
            with self.assertRaisesRegex(RuntimeError,'exact preceding'):self.read(b)
            self.assertEqual(b.calls,1)

    def test_all_other_faults_are_immediate_even_with_goal_mismatch(self):
        for key,value in (('error',1),('torque',0),('watchdog',255),('current',551),
                          ('temperature',46),('voltage',35),('voltage',66),
                          ('p_gain',1),('current_cap',1),('velocity',1),('position',0)):
            for during in (False,True):
                with self.subTest(key=key,during=during):
                    self.report={'enabled_at':0.}
                    bad=self.observed(self.previous);bad[key][0]=value
                    b=self.bus(([self.observed(self.previous)] if during else [])+[bad,self.observed()])
                    with self.assertRaises(RuntimeError):self.read(b)
                    self.assertEqual(b.calls,2 if during else 1)

    def test_voltage_and_goal_recoveries_cannot_nest(self):
        bad=self.observed(self.previous);bad['voltage'][0]=36
        b=self.bus([bad,self.observed()])
        with self.assertRaisesRegex(RuntimeError,'simultaneous undervoltage'):self.read(b)
        self.assertEqual(b.calls,1)
        low=self.observed();low['voltage'][0]=36
        b=self.bus([self.observed(self.previous),low,self.observed()])
        with self.assertRaises(checked.MildUndervoltage):self.read(b)
        self.assertEqual(b.calls,2)

    def test_slow_read_or_deadline_cannot_resume(self):
        for seconds in (.09,.201):
            b=self.bus([self.observed(self.previous),self.observed()],seconds)
            with self.assertRaises(RuntimeError):self.read(b)
            self.assertLessEqual(b.calls,3)

    def test_read_fault_and_user_cancel_propagate_without_writes(self):
        for error in (RuntimeError('read failed'),KeyboardInterrupt('Q')):
            b=self.bus([self.observed(self.previous),error,self.observed()])
            with self.assertRaises(type(error)):self.read(b)
            self.assertEqual(b.calls,2)
            self.assertEqual(self.report['goal_readback_events'][-1]['status'],'stopped')

    def test_every_snapshot_keeps_joint_guards_even_if_later_reads_would_recover(self):
        seen=[]
        def guard(s):
            seen.append(s['goal'].tolist())
            if len(seen)==2:raise RuntimeError('held joint drift')
        b=self.bus([self.observed(self.previous),self.observed(),self.observed()])
        with self.assertRaisesRegex(RuntimeError,'held joint drift'):self.read(b,snapshot_check=guard)
        self.assertEqual(b.calls,2);self.assertEqual(len(seen),2)

    def test_third_repeated_event_stops_instead_of_confirming_forever(self):
        for _ in range(2):
            self.read(self.bus([self.observed(self.previous),self.observed(),self.observed()]))
        b=self.bus([self.observed(self.previous),self.observed(),self.observed()])
        with self.assertRaisesRegex(RuntimeError,'Repeated goal'):self.read(b)
        self.assertEqual(b.calls,1)

    def test_health_itself_is_always_strict(self):
        with self.assertRaises(confirmation.GoalReadbackMismatch):
            checked.health(self.observed(self.previous),self.command)


if __name__=='__main__':unittest.main()
