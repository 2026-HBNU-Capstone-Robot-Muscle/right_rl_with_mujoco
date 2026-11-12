import unittest
from unittest.mock import patch
import numpy as np
import sequence_flip_checked as checked
from id12_stiffness_profile import make_profile
from test_leap_sac_start_empty import FakeBus

class StiffnessTest(unittest.TestCase):
    def test_only_id12_p_and_current_limits_change(self):
        p=make_profile()
        np.testing.assert_array_equal(np.flatnonzero(p['caps']!=checked.CAP),[11])
        np.testing.assert_array_equal(np.flatnonzero(p['gains']['p_gain']!=checked.GAINS['p_gain']),[11])
        self.assertEqual(p['gains']['p_gain'][11],600);self.assertEqual(p['caps'][11],225)
        self.assertEqual(p['current_trip_ma'][11],250)
        for key in ('i_gain','d_gain'):np.testing.assert_array_equal(p['gains'][key],checked.GAINS[key])

    def test_settings_written_off_and_restored_on_completion(self):
        bus=FakeBus();initial=bus.snapshot();bus.checked_control_profile=make_profile();report={}
        writes=[];original=bus.write
        def write(addr,size,values,verify=True):
            writes.append((addr,bus.state['torque'].copy(),np.asarray(values).copy()))
            original(addr,size,values,verify)
        bus.write=write
        def callback(b,c,r):
            self.assertEqual(b.state['p_gain'][11],600);self.assertEqual(b.state['current_cap'][11],225)
            checked.health(b.snapshot(),c,**b.checked_control_profile)
        with patch.object(checked,'check_cancel'),patch.object(checked.time,'sleep'),patch.object(checked,'move',return_value=initial['position']):
            checked.run(bus,initial,[('hold',initial['position'])],report,empty_check=True,prepared_callback=callback)
        for addr,torque,_ in writes:
            if addr in (80,82,84,102):self.assertTrue(np.all(torque==0))
        for key in ('current_cap','p_gain','i_gain','d_gain','torque','watchdog'):
            np.testing.assert_array_equal(bus.state[key],initial[key])
        self.assertTrue(report['settings_restored']);self.assertTrue(report['all_torque_off_verified'])
        self.assertEqual(checked.GAINS['p_gain'][11],450);self.assertEqual(checked.CAP[11],450)

    def test_id12_lower_current_trip_turns_off_and_restores(self):
        bus=FakeBus();initial=bus.snapshot();bus.checked_control_profile=make_profile();report={}
        def callback(b,c,r):
            b.state['current'][11]=251;b.state['current'][0]=300
            checked.sample(b,c,r,'id12_fixed_goal_hold')
        with patch.object(checked,'check_cancel'),patch.object(checked.time,'sleep'),patch.object(checked,'move',return_value=initial['position']):
            with self.assertRaisesRegex(RuntimeError,'ID12 measured current 251mA exceeded 250mA'):
                checked.run(bus,initial,[('hold',initial['position'])],report,empty_check=True,prepared_callback=callback)
        self.assertTrue(bus.off_called);self.assertTrue(report['settings_restored'])
        np.testing.assert_array_equal(bus.state['p_gain'],initial['p_gain'])

    def test_invalid_profile_rejected_before_writes(self):
        for key in ('caps','gains','current_trip_ma'):
            bus=FakeBus();profile=make_profile()
            if key=='gains':profile[key]['p_gain'][11]=601
            else:profile[key][11]=1000
            bus.checked_control_profile=profile
            with self.assertRaises(ValueError):checked.run(bus,bus.snapshot(),[('hold',bus.state['position'])],{},empty_check=True)
            self.assertEqual(bus.writes,[])

    def test_wrong_readback_does_not_pass_profile_health(self):
        bus=FakeBus();p=make_profile();s=bus.snapshot()
        s.update(torque=np.ones(16,dtype=int),watchdog=np.full(16,25),current_cap=p['caps'].copy(),**p['gains'])
        checked.health(s,s['goal'],**p)
        s['current_cap'][11]=450
        with self.assertRaisesRegex(RuntimeError,'setting changed'):checked.health(s,s['goal'],**p)

if __name__=='__main__':unittest.main()
