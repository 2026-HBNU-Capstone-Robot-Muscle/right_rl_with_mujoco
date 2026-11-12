"""Loaded protocol integration with fake motors/camera/clock; no devices opened."""
import contextlib
import io
import unittest
import numpy as np
from test_hand_prepare_full import full_fixture
import hand_prepare_full as full
import encoder_policy_shadow as shadow
from fixed_cube_contact import ContactKeys
import loaded_policy_trial as loaded


class EmptyConsole:
    def kbhit(self): return False


class Camera:
    def __init__(self,clock,report,scenario):
        self.clock,self.report,self.scenario=clock,report,scenario
        self.began={}; self.ack=set()
    def next(self,**kwargs):
        if self.scenario=='camera_lost' and self.report.get('contact_phase')=='policy': return None
        return self.clock.now(),np.zeros((1,1,3),np.uint8)
    def detect(self,frame):
        phase=self.report.get('contact_phase')
        if self.scenario=='missing_pose' and phase=='policy': return None
        if self.scenario=='early_space' and phase=='place' and self.clock.now()-self.began.get(phase,self.clock.now())<.25:
            return None
        return dict(n_cube_markers=2,palm_minus_cube=[.024,.035,.012],cube_quat_world=[1,0,0,0],reproj=1.)
    def show(self,*args): pass
    def key(self):
        phase=self.report.get('contact_phase')
        if phase is None: return 255
        self.began.setdefault(phase,self.clock.now())
        elapsed=self.clock.now()-self.began[phase]
        if self.scenario=='cancel_before_place' and phase=='place': return ord('Q')
        if self.scenario=='cancel_policy' and phase=='policy' and elapsed>.5: return ord('Q')
        if self.scenario=='cancel_remove' and phase=='remove': return ord('Q')
        if self.scenario=='early_space' and phase=='place' and 'early' not in self.ack:
            self.ack.add('early'); return 32
        delay=.4 if self.scenario=='early_space' else .15
        if phase in ('place','remove') and phase not in self.ack and elapsed>delay:
            self.ack.add(phase); return 32
        return 255


class Tests(unittest.TestCase):
    def case(self,scenario='success',model=None,radius=32,joint_limits=False,duration_s=5):
        bus,report,clock=full_fixture()
        report['loaded_policy_mode']=True
        report['sac_deployment_mode']=joint_limits
        camera=Camera(clock,report,scenario)
        keys=ContactKeys(camera,report,EmptyConsole())
        events=[]
        normal_write=bus.write
        def write(addr,size,values,verify=True):
            normal_write(addr,size,values,verify)
            if addr==116 and report.get('phase')=='bounded_policy_motion':
                events.append(dict(phase=report.get('contact_phase'),time=clock.now(),goal=np.array(values).copy()))
                if scenario=='write_fail': raise RuntimeError('Policy write failed')
        bus.write=write
        normal_read=bus.snapshot
        def read():
            clock.sleep(.002)
            s=normal_read()
            if report.get('policy_actions_sent',0)>0:
                if scenario=='overcurrent': s['current'][7]=651
                if scenario=='communication': raise RuntimeError('Read communication failed')
            return s
        bus.snapshot=read
        if model is None:
            def model(obs):
                if scenario=='bad_action': return np.full(16,np.nan)
                return np.full(16,.15 if report.get('policy_inference_count',0)%2 else -.15)
        error=None
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                full.run_trial(bus,bus.snapshot(),report,now=clock.now,sleep=clock.sleep,cancel=keys.poll,
                    policy_radius_ticks=radius,
                    policy_joint_limits=joint_limits,
                    active_check=lambda control:loaded.run(control,camera,model,report,input_keys=keys,
                                                           now=clock.now,sleep=clock.sleep,duration_s=duration_s))
            except (RuntimeError,ValueError,KeyboardInterrupt) as exc: error=exc
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertEqual(len([v for a,v in bus.writes if a==64]),2)
        self.assertTrue(all(e['phase']=='policy' for e in events))
        if events:
            anchor=np.array(report['goal_hw_raw'])
            commands=np.array([anchor,*[e['goal'] for e in events]])
            if joint_limits:
                self.assertTrue(np.all(commands>=np.array(report['policy_command_low_hw_raw'])))
                self.assertTrue(np.all(commands<=np.array(report['policy_command_high_hw_raw'])))
            else:
                self.assertLessEqual(np.max(np.abs(commands-anchor)),radius)
            self.assertLessEqual(np.max(np.abs(np.diff(commands,axis=0))),32)
            began=next(e['time_s'] for e in report['loaded_stage_events'] if e['stage']=='policy' and e['event']=='begin')
            self.assertTrue(all(began<=e['time']<began+duration_s+.01 for e in events))
        return report,error,events

    def test_loaded_success_only_writes_between_placement_and_removal(self):
        report,error,events=self.case()
        self.assertIsNone(error)
        self.assertTrue(report['loaded_trial_complete'])
        self.assertTrue(report['cube_placement_confirmed'] and report['cube_removal_confirmed'])
        self.assertGreater(report['policy_actions_sent'],10)
        self.assertEqual(report['policy_duration_s'],5)
        self.assertTrue(report['policy_cube_loaded'])
        self.assertEqual([e['phase'] for e in report['contact_key_events']],['place','remove'])

    def test_real_policy_npz_in_loaded_protocol(self):
        report,error,_=self.case(model=shadow.Policy(shadow.PROJECT/'policy/policy_np.npz'))
        self.assertIsNone(error)
        self.assertGreater(report['policy_actions_sent'],0)

    def test_space_without_markers_does_not_queue_automatic_start(self):
        report,error,_=self.case('early_space')
        self.assertIsNone(error)
        self.assertEqual(report['placement_space_rejected'],1)
        places=[e for e in report['contact_key_events'] if e['phase']=='place']
        self.assertEqual(len(places),2)

    def test_missing_pose_allows_removal_then_reports_no_movement(self):
        report,error,events=self.case('missing_pose')
        self.assertIn('No policy movement in 5s',str(error))
        self.assertTrue(report['cube_removal_confirmed'])
        self.assertFalse(report['loaded_trial_complete'])
        self.assertEqual(events,[])

    def test_cancel_in_each_stage_and_hardware_faults_release(self):
        for scenario in ('cancel_before_place','cancel_policy','cancel_remove','camera_lost','overcurrent','communication','write_fail','bad_action'):
            with self.subTest(scenario=scenario):
                report,error,events=self.case(scenario)
                self.assertIsNotNone(error)
                self.assertFalse(report['loaded_trial_complete'])
                if scenario in ('cancel_before_place','bad_action'):
                    self.assertEqual(report.get('policy_actions_sent',0),0)
                if scenario=='write_fail':
                    self.assertEqual(report['policy_write_attempts'],1)
                    self.assertEqual(report['policy_actions_sent'],0)


if __name__=='__main__': unittest.main()
