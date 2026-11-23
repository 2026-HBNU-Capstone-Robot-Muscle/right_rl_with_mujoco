"""Failure injection for the new reader, initial-contact scope and launch lock."""
import tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from final_grip_standard_read_20261116 import block_once
from final_grip_single_instance_20261116 import SingleInstance
from interrupted_finger_scope_20261116 import InitialScope,PAIR
from new_pair_grasp_plan_20261027 import ENTRY,SETTLE

class Tests(unittest.TestCase):
    def bus(self,fail=None,error=0,short=False):
        calls=[]
        def tx(*args):calls.append(('tx',args[3]));return 0
        def rx(port,motor,length):
            calls.append(('rx',motor))
            data=[0]*length;data[68]=motor
            return data[:-1] if short and motor==fail else data,(-3002 if motor==fail and not error and not short else 0),(error if motor==fail else 0)
        port=SimpleNamespace(is_using=True,setPacketTimeoutMillis=lambda _:None)
        return SimpleNamespace(port=port,packet=SimpleNamespace(syncReadTx=tx,readRx=rx),
            sdk=SimpleNamespace(COMM_SUCCESS=0,COMM_RX_CORRUPT=-3002),report={}),calls
    def test_complete_all16(self):
        b,c=self.bus();values,comm,error,_=block_once(b,64,83,[('position',132,4)])
        self.assertEqual(values['position'].tolist(),list(range(1,17)))
        self.assertEqual((comm,error),(0,0));self.assertEqual(sum(x[0]=='tx' for x in c),4)
        self.assertFalse(b.port.is_using)
    def test_partial_never_returns_snapshot(self):
        b,c=self.bus(fail=6);values,comm,error,d=block_once(b,64,83,[('p',132,4)])
        self.assertIsNone(values);self.assertEqual(d['completed_ids'],[1,2,3,4,5])
        self.assertEqual(c[-1],('rx',6));self.assertFalse(b.port.is_using)
    def test_servo_status_error_not_discarded(self):
        b,c=self.bus(fail=3,error=4);values,comm,error,d=block_once(b,64,83,[('p',132,4)])
        self.assertIsNone(values);self.assertEqual(error,4);self.assertEqual(d['device_error_id'],3)
    def test_short_success_packet_rejected(self):
        b,c=self.bus(fail=4,short=True);self.assertIsNone(block_once(b,64,83,[('p',132,4)])[0])
    def test_deadline_does_not_transmit(self):
        b,c=self.bus();b._horizontal_sample_deadline=0
        with self.assertRaisesRegex(RuntimeError,'expired'):block_once(b,64,83,[('p',132,4)])
        self.assertEqual(c,[])
    def test_scope_separates_initial_contact(self):
        s=InitialScope({PAIR:.51837},False,mode='static')
        self.assertEqual(s.filtered(ENTRY,{PAIR:.4}),{})
        s.filtered(ENTRY,{});s.filtered(SETTLE,{})
        self.assertTrue(s.valid)
    def test_scope_deepening_rejected(self):
        s=InitialScope({PAIR:.14},False,mode='static');s.filtered(ENTRY,{PAIR:.15});s.filtered(SETTLE,{})
        self.assertFalse(s.valid)
    def test_scope_reappearing_rejected(self):
        s=InitialScope({PAIR:.14},False,mode='static');s.filtered(ENTRY,{});s.filtered(ENTRY,{PAIR:.1});s.filtered(SETTLE,{})
        self.assertFalse(s.valid)
    def test_scope_new_contact_rejected(self):
        s=InitialScope({},False,mode='static');self.assertIn(PAIR,s.filtered(ENTRY,{PAIR:.1}));s.filtered(SETTLE,{})
        self.assertFalse(s.valid)
    def test_scope_late_contact_rejected(self):
        s=InitialScope({PAIR:.14},False,mode='static')
        for _ in range(33):s.filtered(ENTRY,{PAIR:.1})
        s.filtered(SETTLE,{});self.assertFalse(s.valid)
    def test_single_instance_and_exception_release(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'lock'
            with self.assertRaisesRegex(ValueError,'test'):
                with SingleInstance(p):
                    with self.assertRaisesRegex(RuntimeError,'already running'):
                        with SingleInstance(p):self.fail('duplicate entered')
                    raise ValueError('test')
            with SingleInstance(p):pass

if __name__=='__main__':unittest.main()
