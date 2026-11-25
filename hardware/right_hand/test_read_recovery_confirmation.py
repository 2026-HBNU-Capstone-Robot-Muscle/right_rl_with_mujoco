"""Fresh-read fault injection and exact saved lost-goal regression."""
import contextlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import sequence_flip_checked as checked
from grasped_yaw_plan import GraspedYawPlan
from test_sequence_voltage import Clock,ReadOnlyBus,state

ROOT=Path(__file__).resolve().parent


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.report={'enabled_at':0.};self.command=state()['goal'].copy()
        stack=contextlib.ExitStack();self.addCleanup(stack.close)
        stack.enter_context(patch.object(checked.time,'monotonic',self.clock.monotonic))
        stack.enter_context(patch.object(checked.time,'sleep',self.clock.sleep))
        stack.enter_context(patch.object(checked,'check_cancel'))
        stack.enter_context(contextlib.redirect_stdout(io.StringIO()))

    def bus(self,rows=None,seconds=.032,*,opted=True,recovery_calls=(1,)):
        bus=ReadOnlyBus(self.clock,rows or [state()],seconds)
        bus.plan_module=GraspedYawPlan if opted else SimpleNamespace(CONFIRM_FRESH_AFTER_READ_RECOVERY=False)
        original=bus.snapshot
        def snapshot():
            result=original()
            if bus.calls in recovery_calls:
                self.report.setdefault('read_failures',[]).append(dict(recovered=True))
            return result
        bus.snapshot=snapshot
        return bus

    def read(self,bus,**kwargs):return checked.sample(bus,self.command,self.report,'test',**kwargs)

    def test_no_read_failure_does_not_add_confirmation(self):
        b=self.bus(recovery_calls=());self.read(b)
        self.assertEqual(b.calls,1);self.assertNotIn('read_recovery_confirmation_events',self.report)

    def test_two_fresh_exact_healthy_observations_keep_all_snapshot_guards(self):
        seen=[];b=self.bus()
        with patch('importlib.machinery.SourceFileLoader.get_data',side_effect=AssertionError('ON source file access')):
            self.read(b,snapshot_check=lambda s:seen.append(s['goal'].copy()))
        self.assertEqual(b.calls,3);self.assertEqual(len(seen),3)
        e=self.report['read_recovery_confirmation_events'][0]
        self.assertEqual(e['status'],'confirmed');self.assertEqual(e['clean_reads'],2)
        self.assertEqual((e['goal_writes'],e['goals_advanced']),(0,0))
        self.assertEqual(sum(s.get('sample_role')=='post_read_recovery_confirmation' for s in self.report['samples']),2)

    def test_repeat_read_recovery_stops_before_any_new_goal(self):
        b=self.bus(recovery_calls=(1,2))
        with self.assertRaisesRegex(RuntimeError,'Another communication failure'):self.read(b)
        self.assertEqual(b.calls,2)

    def test_total_sample_budget_includes_initial_recovered_read(self):
        b=self.bus(seconds=.09)
        with self.assertRaisesRegex(RuntimeError,'200ms sample budget'):self.read(b)
        self.assertEqual(b.calls,3)
        self.assertEqual(self.report['read_recovery_confirmation_events'][0]['status'],'stopped')

    def test_all_health_and_held_joint_faults_stop_even_if_later_reads_are_good(self):
        for key,value in (('torque',0),('error',1),('watchdog',255),('current',551),
                          ('voltage',35),('voltage',36),('temperature',46),('position',0),('goal',1)):
            with self.subTest(key=key):
                self.report={'enabled_at':0.};bad=state();bad[key][0]=value
                b=self.bus([state(),bad,state()])
                with self.assertRaises(RuntimeError):self.read(b)
                self.assertEqual(b.calls,2)
        self.report={'enabled_at':0.};seen=[];b=self.bus()
        def guard(s):
            seen.append(1)
            if len(seen)==2:raise RuntimeError('held-joint fault')
        with self.assertRaisesRegex(RuntimeError,'held-joint'):self.read(b,snapshot_check=guard)
        self.assertEqual(b.calls,2)

    def test_q_and_camera_or_read_errors_stop_without_writes(self):
        for error in (KeyboardInterrupt('Q'),RuntimeError('camera error'),RuntimeError('read error')):
            self.report={'enabled_at':0.};b=self.bus([state(),error,state()])
            with self.assertRaises(type(error)):self.read(b)
            self.assertEqual(b.calls,2)

    def test_other_plans_keep_the_existing_single_snapshot_behavior(self):
        b=self.bus(opted=False);self.read(b)
        self.assertEqual(b.calls,1);self.assertNotIn('read_recovery_confirmation_events',self.report)

    def test_recorded_missing_goal_packet_still_stops_and_is_not_tolerated(self):
        r=json.loads((ROOT/'grasped_yaw_trial_20261023_222816_628099/result.json').read_bytes())
        e=r['goal_readback_events'][0];self.command=np.array(e['command'])
        row=state();row['goal']=np.array(e['initial_readback']);row['position']=self.command.copy()
        b=self.bus([row],recovery_calls=())
        b.goal_readback_confirmation=True;b.last_goal_command=self.command.copy()
        b.previous_goal_command=np.array(e['previous_command'])
        with self.assertRaisesRegex(RuntimeError,'twice within'):self.read(b)
        self.assertEqual(self.report['goal_readback_events'][0]['goal_writes'],0)
        self.assertEqual(b.calls,4)


if __name__=='__main__':unittest.main()
