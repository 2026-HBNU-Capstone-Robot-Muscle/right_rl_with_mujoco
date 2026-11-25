import unittest
from restore_known_off_settings_20261105 import restore,baseline,FIELDS

class FakeIO:
    def __init__(self):
        self.values={(m,a):v for m in range(1,17) for a,v in ((64,0),(70,0),(11,5))};self.writes=[]
        b=baseline()
        for a,size,key in FIELDS:
            for m in range(1,17):self.values[m,a]=b[key][m-1]
    def read(self,m,a,size):return self.values[m,a]
    def write(self,m,a,size,v):self.writes.append((m,a,size,v));self.values[m,a]=v

class Tests(unittest.TestCase):
    def test_already_restored_no_writes(self):
        io=FakeIO();r={};restore(io,baseline(),r);self.assertEqual(io.writes,[]);self.assertTrue(r['all_torque_off_verified'])
    def test_only_changed_RAM_settings_written(self):
        io=FakeIO();io.values[8,80]=150;io.values[8,84]=450;r={};restore(io,baseline(),r);self.assertEqual(io.writes,[(8,80,2,1900),(8,84,2,2100)]);self.assertTrue(r['original_settings_verified'])
    def test_torque_on_blocks_all_writes(self):
        io=FakeIO();io.values[9,64]=1;io.values[8,80]=150
        with self.assertRaises(RuntimeError):restore(io,baseline(),{})
        self.assertEqual(io.writes,[])
    def test_wrong_mode_blocks_all_writes(self):
        io=FakeIO();io.values[16,11]=3
        with self.assertRaises(RuntimeError):restore(io,baseline(),{})
        self.assertEqual(io.writes,[])
    def test_failed_readback_no_retry(self):
        io=FakeIO();io.values[1,80]=200
        def broken(m,a,size,v):io.writes.append((m,a,size,v))
        io.write=broken
        with self.assertRaises(RuntimeError):restore(io,baseline(),{})
        self.assertEqual(len(io.writes),1)
if __name__=='__main__':unittest.main()
