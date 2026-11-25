"""Behavioral checks: no AUTO OPEN, OFF-only joint teaching, no stale handoff."""
import copy,json,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from manual_grasp_plan_20261106 import R,prefix,validate_candidate
from natural_cradle_grasp_plan_20261030 import validate_candidate as original
from manual_grasp_capture_20261106 import ManualGraspCapture
from manual_grasp_off_teach_20261106 import install_readonly_guard,validate_off_snapshot,require_completed_off_motion,capture_off_pair
from leap_manual_grasp_teach_20261106 import completed_source

def state(position=None):
    return dict(torque=np.zeros(16,int),error=np.zeros(16,int),watchdog=np.zeros(16,int),position=np.full(16,2048,int) if position is None else np.asarray(position,int),voltage=np.full(16,50,int),temperature=np.full(16,20,int))
def completed():
    return dict(status='manual_grasp_reached_torque_off',all_torque_off_verified=True,settings_restored=True,cube_support_confirmed_before_off=True,cube_placement_confirmed=True,automatic_open=False,manual_joint_correction_allowed=False)
class Clock:
    t=1.
    def now(self):return self.t
    def sleep(self,seconds):self.t+=seconds
class Camera:
    def __init__(self,clock):self.clock=clock
    def read_packet(self):self.clock.t+=.05;return self.clock.t,np.zeros((4,4,3),np.uint8)
class ManualTeachingTests(unittest.TestCase):
    def test_keeps_exact_executed_grasp_but_removes_all_opening(self):
        source=original();q,p=prefix(source);self.assertEqual(len(q),927);self.assertEqual(set(p),{'stage_5_loaded_grasp','stage_6_grasp_hold'});self.assertTrue(np.array_equal(q,source['goals'][:927]));self.assertFalse(np.array_equal(q[-1],source['goals'][-1]));self.assertTrue(np.array_equal(validate_candidate()['goals'],q))
    def test_interleaved_opening_cannot_be_hidden_in_prefix(self):
        source=dict(original());source['phases']=source['phases'].copy();source['phases'][10]='stage_9_loaded_open'
        with self.assertRaises(ValueError):prefix(source)
    def test_missing_hold_or_changed_hold_cannot_be_admitted(self):
        source=dict(original());source['phases']=source['phases'].copy();source['phases'][source['phases']=='stage_6_grasp_hold']='unexpected_phase'
        with self.assertRaises(ValueError):prefix(source)
        source=dict(original());source['goals']=source['goals'].copy();source['goals'][926,0]+=1
        with self.assertRaises(ValueError):prefix(source)
    def test_support_checkpoint_has_no_motor_write_and_never_allows_ON_reshaping(self):
        c=ManualGraspCapture(None,None,cancel=lambda:None);c.guard=object();s=state();bus=SimpleNamespace(pending=[]);q=validate_candidate()['goals'][-1];r=dict(cube_placement_confirmed=True);phases=[]
        c._observer=lambda *args:lambda phase:s;c._photo=lambda *args:(s,dict(image='before.jpg'))
        c._space=lambda phase,observe:(phases.append(phase) or s)
        c.finish(bus,q,r);self.assertEqual(phases,['support_cube_for_off']);self.assertTrue(r['cube_support_confirmed_before_off']);self.assertFalse(r['manual_joint_correction_allowed']);self.assertFalse(r['cube_removal_confirmed'])
    def test_unverified_OFF_or_wrong_scope_never_starts_joint_teaching(self):
        require_completed_off_motion(completed())
        for key,value in (('all_torque_off_verified',False),('settings_restored',False),('cube_support_confirmed_before_off',False),('automatic_open',True),('manual_joint_correction_allowed',True),('status','stopped')):
            d=completed();d[key]=value
            with self.assertRaises(RuntimeError):require_completed_off_motion(d)
    def test_off_reading_does_not_clamp_freely_taught_angles(self):
        positions=np.arange(16)*500-1000;self.assertTrue(np.array_equal(validate_off_snapshot(state(positions)),positions))
    def test_any_active_or_error_motor_prevents_OFF_teaching(self):
        for key in ('torque','error','watchdog'):
            s=state();s[key][10]=1
            with self.assertRaises(RuntimeError):validate_off_snapshot(s)
    def test_raw_write_instruction_guard_blocks_torque_settings_and_sync_write(self):
        calls=[];port=SimpleNamespace(writePort=lambda data:calls.append(data));report={};install_readonly_guard(port,report)
        for instruction in (3,4,5,6,8,0x83,0x93):
            with self.assertRaises(RuntimeError):port.writePort([0]*7+[instruction,64,0,1,0,0])
        self.assertEqual(calls,[])
        port.writePort([0]*7+[1]);port.writePort([0]*7+[2]);port.writePort([0]*7+[0x8a,64,0,83,0,1,2,3,4,0,0]);self.assertEqual(len(calls),3)
        with self.assertRaises(RuntimeError):port.writePort([0]*7+[0x8a,116,0,4,0,1,2,3,4,0,0])
    def test_photo_pair_rejects_movement_while_motor_remains_OFF(self):
        for drift,expected in ((2,True),(4,False)):
            clock=Clock();rows=iter([state(),state(np.r_[2048+drift,np.full(15,2048)])]);report={};images=[];ok=capture_off_pair(lambda:next(rows),Camera(clock),report,images.append,now=clock.now,sleep=clock.sleep)
            self.assertEqual(ok,expected);self.assertEqual(len(images),1);self.assertEqual(report['pair_check']['maximum_drift_ticks'],drift)
    def test_torque_change_after_photo_never_accepts_pair(self):
        clock=Clock();bad=state();bad['torque'][0]=1;rows=iter([state(),bad]);report={}
        with self.assertRaises(RuntimeError):capture_off_pair(lambda:next(rows),Camera(clock),report,lambda frame:None,now=clock.now,sleep=clock.sleep)
        self.assertNotEqual(report.get('status'),'photo_and_readings_saved')
    def test_stale_or_mismatched_motion_result_never_reaches_teaching(self):
        with tempfile.TemporaryDirectory(dir=R) as folder:
            folder=Path(folder).resolve();self.assertTrue(folder.is_relative_to(R));source=folder/'result.json';pointer=folder/'latest.json';request=folder/'request.json';d=completed();d['request_file']=str(request);data=json.dumps(d).encode();source.write_bytes(data);pointer.write_bytes(data)
            self.assertEqual(completed_source(pointer,0),source)
            with self.assertRaises(RuntimeError):completed_source(pointer,time.time()+100)
            source.write_bytes(b'{}')
            with self.assertRaises(RuntimeError):completed_source(pointer,0)
if __name__=='__main__':unittest.main()
