"""Offline policy/servo/camera integration. No physical devices."""
import contextlib
import io
import unittest
import numpy as np
from test_hand_prepare_resume import fixture, RAW
import hand_prepare_resume as prep
import bounded_policy as policy


class Camera:
    def __init__(self,clock,*,gap=False):
        self.clock=clock
        self.start=clock.now()
        self.gap=gap
    def next(self,**kwargs): return self.clock.now(),np.zeros((1,1))
    def detect(self,frame):
        t=self.clock.now()-self.start
        if self.gap and .5<t<1.: return None
        return dict(n_cube_markers=2,palm_minus_cube=[0,0,.01],cube_quat_world=[1,0,0,0],reproj=1.)
    def key(self): pass
    def show(self,*args): pass


class Tests(unittest.TestCase):
    def run_trial(self,model,*,gap=False,fault=None,read_delay=0.,observe_clock=None):
        bus,report,clock=fixture()
        if observe_clock is not None: observe_clock.append(clock)
        original=bus.snapshot
        def snapshot():
            clock.sleep(read_delay)
            s=original()
            if report.get('phase')=='bounded_policy_motion' and report.get('policy_actions_sent',0):
                if fault=='current': s['current'][0]=1000
                if fault=='cancel': raise KeyboardInterrupt()
                if fault=='tracking': s['position'][0]+=80
                if fault=='goal': s['goal'][0]+=1
                if fault=='slow': clock.sleep(.21)
                if fault=='total_timeout': clock.sleep(61)
                if fault=='stall': s['position']=np.array(report['goal_hw_raw'])-30
            return s
        bus.snapshot=snapshot
        original_write=bus.write
        def write(addr,size,values,verify=True):
            original_write(addr,size,values,verify)
            if fault=='write_fail' and addr==116 and report.get('phase')=='bounded_policy_motion':
                raise RuntimeError('Policy write readback failed')
        bus.write=write
        def active(control):
            camera=Camera(clock,gap=gap)
            if fault=='camera_lost': camera.next=lambda **kwargs:None
            policy.run(control,camera,model,report,
                       now=clock.now,sleep=clock.sleep,cancel=lambda:None)
        with contextlib.redirect_stdout(io.StringIO()):
            if fault:
                with self.assertRaises((RuntimeError,KeyboardInterrupt,ValueError)):
                    prep.run_trial(bus,bus.snapshot(),report,active_check=active,
                                   cancel=lambda:None,now=clock.now,sleep=clock.sleep)
            else:
                prep.run_trial(bus,bus.snapshot(),report,active_check=active,
                               cancel=lambda:None,now=clock.now,sleep=clock.sleep)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertEqual(len([v for a,v in bus.writes if a==64]),2)
        np.testing.assert_array_equal([v for a,v in bus.writes if a==102][0],prep.CAPS)
        return report,clock

    def test_policy_period_includes_read_work_instead_of_extra_50ms_sleep(self):
        clocks=[]; inference_times=[]
        def model(obs):
            inference_times.append(clocks[0].now())
            return np.full(16,.2 if len(inference_times)%2 else -.2)
        report,_=self.run_trial(model,read_delay=.02,observe_clock=clocks)
        periods=np.diff(inference_times)
        self.assertGreater(len(periods),100)
        self.assertGreaterEqual(float(np.min(periods)),.05-1e-8)
        self.assertLessEqual(float(np.max(periods)),.05+1e-8)
        self.assertGreater(report['policy_actions_sent'],0)

    def test_real_npz_moves_with_bounds_and_no_intermediate_off(self):
        model=policy.shadow.Policy(policy.shadow.PROJECT/'policy/policy_np.npz')
        report,clock=self.run_trial(model)
        self.assertGreater(report['policy_actions_sent'],0)
        self.assertTrue(report['bounded_policy_trial_complete'])
        self.assertLess(clock.now(),60)
        anchor=np.array(report['goal_hw_raw'])
        commands=np.array([anchor]+[s['command_hw_raw'] for s in report['policy_trial_samples']])
        self.assertLessEqual(np.max(np.abs(commands-anchor)),policy.RADIUS_TICKS)
        self.assertLessEqual(np.max(np.abs(np.diff(commands,axis=0))),8)
        self.assertTrue(all(s['frame_age_ms']<=200 for s in report['policy_trial_samples']))

    def test_marker_gap_holds_and_resumes_without_restart_and_preserves_feedback(self):
        seen=[]
        def model(obs):
            seen.append(obs.copy())
            return np.full(16,.2 if len(seen)%2 else -.2)
        report,clock=self.run_trial(model,gap=True)
        self.assertGreater(report['policy_actions_sent'],10)
        self.assertGreater(report['policy_skip_counts']['no_pose'],0)
        actions=report['policy_trial_samples']
        np.testing.assert_allclose(seen[1][-17:-1],actions[0]['applied_action'])
        self.assertLess(np.max(np.abs(actions[0]['applied_action'])),.2)

    def test_policy_faults_release_without_retry(self):
        for fault in ('current','cancel','tracking','goal','slow','stall','total_timeout','write_fail','camera_lost'):
            with self.subTest(fault=fault):
                self.run_trial(lambda obs:np.full(16,.5),fault=fault)

    def test_bad_policy_action_releases_before_policy_write(self):
        report,_=self.run_trial(lambda obs:np.full(16,np.nan),fault='bad_action')
        self.assertEqual(report['policy_write_attempts'],0)

    def test_stale_frame_or_expired_trial_never_writes(self):
        from test_hand_start_pose import Clock
        clock=Clock()
        anchor=prep.plan(RAW)[2]
        def read(command,lo,hi): return dict(position=command.copy())
        writes=[]
        control=policy.Control(anchor,read=read,send=writes.append,report={},now=clock.now)
        clock.sleep(.3)
        self.assertIsNone(control.apply(np.ones(16),0.))
        control.deadline=.2
        self.assertIsNone(control.apply(np.ones(16),clock.now()))
        self.assertEqual(writes,[])

    def test_recorded_preload_jitter_no_longer_requires_identical_reads(self):
        bus,report,clock=fixture()
        original=bus.snapshot
        reads=[]
        def jitter():
            if report.get('phase')=='configure_preload_OFF':
                n=len(reads)
                bus.state['position'][0]=RAW[0]-(n%2)
                bus.state['position'][13]=RAW[13]+(2 if n%2 else 0)
                bus.state['goal']=bus.state['position'].copy()
                reads.append(n)
            return original()
        bus.snapshot=jitter
        with contextlib.redirect_stdout(io.StringIO()):
            prep.run_trial(bus,bus.snapshot(),report,cancel=lambda:None,now=clock.now,sleep=clock.sleep)
        self.assertEqual(len(reads),3)
        self.assertTrue(report['all_torque_off_verified'])
