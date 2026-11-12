import unittest
import numpy as np
from unittest.mock import patch
import leap_sac_start_empty as runner

class FakeBus:
    def __init__(self):
        self.state={k:np.zeros(16,dtype=int) for k in ('torque','error','watchdog','acceleration','velocity','current','i_gain')}
        self.state.update(position=np.full(16,2048),goal=np.full(16,2048),current_cap=np.full(16,60),p_gain=np.full(16,2100),d_gain=np.full(16,1900),voltage=np.full(16,48),temperature=np.full(16,25))
        self.writes=[];self.off_called=False
    def write(self,addr,size,values,verify=True):
        keys={64:'torque',80:'d_gain',82:'i_gain',84:'p_gain',98:'watchdog',102:'current_cap',108:'acceleration',112:'velocity',116:'goal'}
        self.state[keys[addr]]=np.asarray(values).copy();self.writes.append((addr,np.asarray(values).copy()))
        if addr==116:self.state['position']=np.asarray(values).copy()
    def snapshot(self):return {k:v.copy() for k,v in self.state.items()}
    def off(self):self.off_called=True;self.state['torque'][:]=0

class StartPlanTest(unittest.TestCase):
    def test_mapping_round_trip(self):
        raw=runner.target_pose();sim=(raw[runner.SIM_TO_ID-1]*runner.checked.TICK)-np.pi
        import json
        doc=json.loads((runner.ROOT/'sac_supported_reset_result.json').read_text())
        np.testing.assert_allclose(sim,doc['candidates'][0]['ctrl'],atol=runner.checked.TICK/2)
    def test_relief_only_raises_root_joints(self):
        initial=np.full(16,2048);poses=runner.plan(initial,runner.target_pose());relief=poses[0][1]
        changed=np.flatnonzero(initial!=relief);np.testing.assert_array_equal(changed,[4,8,12])
        self.assertTrue(np.all(relief[changed]>initial[changed]))
    def test_roots_already_clear_are_not_lowered_in_relief(self):
        initial=runner.target_pose();initial[[4,8,12]]=3000
        np.testing.assert_array_equal(runner.plan(initial,runner.target_pose())[0][1],initial)
    def test_large_initial_jump_rejected_before_run(self):
        with self.assertRaises(ValueError):runner.plan(np.zeros(16),runner.target_pose())
    def test_wrong_reset_hash_refused(self):
        with patch.object(runner,'RESET_SHA','bad'):
            with self.assertRaises(ValueError):runner.target_pose()
    def test_inherited_runner_restores_settings_and_off_after_failure(self):
        bus=FakeBus();initial=bus.snapshot();report={}
        with patch.object(runner.checked,'check_cancel'),patch.object(runner.checked.time,'sleep'),patch.object(runner.checked,'move',side_effect=RuntimeError('simulated obstruction')):
            with self.assertRaises(RuntimeError):runner.checked.run(bus,initial,runner.plan(initial['position'],runner.target_pose()),report,empty_check=True)
        self.assertTrue(bus.off_called);self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])
        for key in ('torque','current_cap','p_gain','i_gain','d_gain','watchdog','acceleration','velocity'):
            np.testing.assert_array_equal(bus.state[key],initial[key])
    def test_capture_hook_only_after_all_empty_preparation_stages(self):
        bus=FakeBus();initial=bus.snapshot();report={};events=[]
        poses=runner.plan(initial['position'],runner.target_pose())
        def move(bus,command,target,report,phase):
            events.append(phase);return target.copy()
        def capture(bus,command,report):
            events.append('capture')
            np.testing.assert_array_equal(command,runner.target_pose())
        with patch.object(runner.checked,'check_cancel'),patch.object(runner.checked.time,'sleep'),patch.object(runner.checked,'move',side_effect=move):
            runner.checked.run(bus,initial,poses,report,empty_check=True,prepared_callback=capture)
        self.assertEqual(events,['stage_0','stage_1','stage_2','capture'])
        self.assertEqual(report['status'],'revised_start_capture_complete')
        self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])
    def test_camera_failure_after_preparation_still_turns_off(self):
        bus=FakeBus();initial=bus.snapshot();report={}
        def failed_capture(*args):raise RuntimeError('camera stale')
        with patch.object(runner.checked,'check_cancel'),patch.object(runner.checked.time,'sleep'),patch.object(runner.checked,'move',return_value=runner.target_pose()):
            with self.assertRaisesRegex(RuntimeError,'camera stale'):
                runner.checked.run(bus,initial,runner.plan(initial['position'],runner.target_pose()),report,
                                   empty_check=True,prepared_callback=failed_capture)
        self.assertTrue(bus.off_called);self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])

if __name__=='__main__':unittest.main()
