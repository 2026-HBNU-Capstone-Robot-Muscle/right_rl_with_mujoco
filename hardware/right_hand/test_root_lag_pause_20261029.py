import contextlib,io,unittest
from unittest.mock import Mock
import numpy as np
from root_lag_pause_20261029 import RootLagPause,POLICY
class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,x):self.t+=x
class Checks(unittest.TestCase):
    def state(self,error):
        s=dict(position=np.zeros(16,dtype=int),current=np.zeros(16,dtype=int));e=np.zeros(16);e[11]=error;return s,e
    def test_no_wait_for_other_phase_or_small_error(self):
        c=Clock();p=RootLagPause(now=c.now);observe=Mock();s,e=self.state(.15)
        self.assertEqual(p.wait_if_needed('stage_5_loaded_grasp',s,e,7,observe=observe,command=np.zeros(16),report={},sleep=c.sleep,cycle_now=c.now)[2],7)
        s,e=self.state(.119);p.wait_if_needed(POLICY['phase'],s,e,7,observe=observe,command=np.zeros(16),report={},sleep=c.sleep,cycle_now=c.now);observe.assert_not_called()
    def test_two_consecutive_good_observations_required_and_goals_immutable(self):
        c=Clock();p=RootLagPause(now=c.now);report={'samples':[]};command=np.arange(16);command.setflags(write=False);source=[.11,.095,.105,.09,.08];calls=[]
        def observe():
            calls.append(c.now());c.sleep(.02);report['samples'].append({});s,e=self.state(source[len(calls)-1]);return s,e,c.now()
        s,e=self.state(.15)
        with contextlib.redirect_stdout(io.StringIO()):s,e,t=p.wait_if_needed(POLICY['phase'],s,e,0,observe=observe,command=command,report=report,sleep=c.sleep,cycle_now=c.now)
        self.assertEqual(len(calls),5);np.testing.assert_array_equal(command,np.arange(16));self.assertEqual(report['root_lag_wait_events'][0]['status'],'resumed')
        self.assertEqual(report['root_lag_wait_events'][0]['goal_writes'],0);self.assertEqual(e[11],.08)
        self.assertTrue(all(b-a>=.075 for a,b in zip(calls,calls[1:])))
    def test_persistent_lag_stops_within_single_bound(self):
        c=Clock();p=RootLagPause(now=c.now);report={}
        def observe():c.sleep(.02);s,e=self.state(.14);return s,e,c.now()
        s,e=self.state(.14)
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(RuntimeError):p.wait_if_needed(POLICY['phase'],s,e,0,observe=observe,command=np.zeros(16),report=report,sleep=c.sleep,cycle_now=c.now)
        self.assertLessEqual(c.now(),1.2);self.assertEqual(report['root_lag_wait_events'][0]['status'],'stopped')
    def test_original_health_error_propagates_immediately(self):
        c=Clock();p=RootLagPause(now=c.now);observe=Mock(side_effect=RuntimeError('ID12 tracking0.18'));s,e=self.state(.14);r={}
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(RuntimeError,'tracking'):p.wait_if_needed(POLICY['phase'],s,e,0,observe=observe,command=np.zeros(16),report=r,sleep=c.sleep,cycle_now=c.now)
        self.assertEqual(observe.call_count,1);self.assertEqual(r['root_lag_wait_events'][0]['goal_writes'],0)
    def test_cancel_propagates(self):
        c=Clock();p=RootLagPause(now=c.now);s,e=self.state(.14)
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(KeyboardInterrupt):p.wait_if_needed(POLICY['phase'],s,e,0,observe=Mock(side_effect=KeyboardInterrupt('Q')),command=np.zeros(16),report={},sleep=c.sleep,cycle_now=c.now)
    def test_total_wait_budget_is_not_reset(self):
        c=Clock();p=RootLagPause(now=c.now);p.total=19.9;s,e=self.state(.14);observe=Mock()
        with self.assertRaisesRegex(RuntimeError,'total budget'):p.wait_if_needed(POLICY['phase'],s,e,0,observe=observe,command=np.zeros(16),report={},sleep=c.sleep,cycle_now=c.now)
        observe.assert_not_called()
if __name__=='__main__':unittest.main(verbosity=2)
