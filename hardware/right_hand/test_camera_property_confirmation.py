"""Fault-injected property reads only, with no capture or motor devices."""
import math
import unittest
from types import SimpleNamespace
from camera_property_confirmation import PropertyReadConfirmation


class Tests(unittest.TestCase):
    def read(self,values,*,initial=None,clock=None,stop=None):
        remaining=iter(values);record={};count=[]
        def get(prop):count.append(prop);return next(remaining)
        confirmation=PropertyReadConfirmation(record,now=clock or (lambda:0.))
        return confirmation,SimpleNamespace(get=get),record,count

    def test_transient_unavailable_requires_two_fresh_matches_without_setting(self):
        c,cap,r,calls=self.read([-1,-5,-5])
        self.assertEqual(c.read(cap,15,-5),-5);self.assertEqual(len(calls),3)
        self.assertTrue(r['property_read_confirmations'][0]['recovered'])
        self.assertEqual(r['property_read_confirmations'][0]['settings_writes'],0)

    def test_applied_mismatch_is_not_lost_when_first_confirmation_is_good(self):
        c,cap,r,calls=self.read([-5,-5])
        c.read(cap,15,-5,initial=-1)
        self.assertEqual(len(calls),2);self.assertEqual(r['property_read_confirmations'][0]['readbacks'],[-1,-5,-5])

    def test_persistent_wrong_unavailable_nonfinite_or_nonconsecutive_stops(self):
        for values in ([-1]*4,[-6]*4,[float('nan')]*4,[-1,-5,-1,-5]):
            c,cap,r,calls=self.read(values)
            with self.subTest(values=values),self.assertRaises(RuntimeError):c.read(cap,15,-5)
            self.assertEqual(len(calls),4);self.assertFalse(r['property_read_confirmations'][0]['recovered'])

    def test_deadline_and_cancel_are_bounded(self):
        times=iter([0.,0.,.101]);c,cap,r,calls=self.read([-1,-5],clock=lambda:next(times))
        with self.assertRaises(RuntimeError):c.read(cap,15,-5)
        self.assertEqual(len(calls),2)
        c,cap,r,calls=self.read([-1])
        with self.assertRaises(KeyboardInterrupt):c.read(cap,15,-5,stop=SimpleNamespace(is_set=lambda:True))
        self.assertEqual(len(calls),1)

    def test_repeated_mismatch_limit_does_not_accept_flapping_forever(self):
        c,cap,r,calls=self.read([-1,-5,-5]*4)
        for _ in range(3):c.read(cap,15,-5)
        with self.assertRaisesRegex(RuntimeError,'limit'):c.read(cap,15,-5)
        self.assertEqual(len(calls),10)


if __name__=='__main__':unittest.main()
