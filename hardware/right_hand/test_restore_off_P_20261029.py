import contextlib,copy,io,sys,unittest
from unittest.mock import patch
import numpy as np
import restore_off_P_20261029 as m
class Checks(unittest.TestCase):
    def setUp(self):self.original=m.specification();self.state={k:np.asarray(v) for k,v in self.original.items()}
    def packet(self,address=84,values=None):
        payload=[]
        for motor,v in enumerate(values if values is not None else self.original['p_gain'],1):payload.extend([motor,v&255,v>>8])
        return [255,255,253,0,254,0,0,0x83,address,0,2,0]+payload+[0,0]
    def test_exact_saved_P_packet_requires_armed_state(self):
        self.assertTrue(m.validate_packet(self.packet(),self.original,True))
        with self.assertRaises(RuntimeError):m.validate_packet(self.packet(),self.original,False)
    def test_goal_torque_mode_EEPROM_addresses_are_blocked(self):
        for address in (7,8,11,64,80,82,102,116):
            with self.subTest(address=address),self.assertRaises(RuntimeError):m.validate_packet(self.packet(address),self.original,True)
    def test_different_values_or_missing_ID_are_blocked(self):
        values=list(self.original['p_gain']);values[0]-=1
        with self.assertRaises(RuntimeError):m.validate_packet(self.packet(values=values),self.original,True)
        p=self.packet();del p[-5:-2]
        with self.assertRaises(RuntimeError):m.validate_packet(p,self.original,True)
    def test_unicast_write_cannot_bypass_whitelist(self):
        p=self.packet();p[7]=3
        with self.assertRaises(RuntimeError):m.validate_packet(p,self.original,True)
    def test_torque_ON_rejected(self):
        self.state['torque'][0]=1
        with self.assertRaises(ValueError):m.validate_state(self.state,self.original)
    def test_unexpected_other_settings_rejected(self):
        self.state['d_gain'][0]=123
        with self.assertRaises(RuntimeError):m.validate_state(self.state,self.original)
    def test_original_or_known_trial_P_accepted_only(self):
        self.state['p_gain']=m.PROFILE.copy();m.validate_state(self.state,self.original)
        self.state['p_gain'][0]=700
        with self.assertRaises(RuntimeError):m.validate_state(self.state,self.original)
    def test_default_never_opens_port(self):
        import dynamixel_sdk as sdk
        with patch.object(sys,'argv',['restore_off_P_20261029.py']),patch.object(sdk,'PortHandler',side_effect=AssertionError('Port opened')),contextlib.redirect_stdout(io.StringIO()):self.assertEqual(m.main(),0)
if __name__=='__main__':unittest.main(verbosity=2)
