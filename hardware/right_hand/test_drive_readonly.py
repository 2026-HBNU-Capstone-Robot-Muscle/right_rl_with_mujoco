import unittest
import hand_readonly_check as check

class DriveReadTest(unittest.TestCase):
    def test_only_expected_read_addresses_twice(self):
        calls=[]
        def read(motor,address,size):
            calls.append((motor,address,size));return 885 if address in (36,100) else 0
        result=check.read_drive_settings(read,12)
        self.assertEqual(calls,[call for _,address,size in check.DRIVE_REGISTERS for call in [(12,address,size)]*2])
        self.assertAlmostEqual(result['pwm_limit_percent_approx'],100.005)

    def test_inconsistent_read_fails_without_correction(self):
        calls=[]
        def read(*args):calls.append(args);return len(calls)
        with self.assertRaisesRegex(RuntimeError,'unstable'):check.read_drive_settings(read,12)
        self.assertEqual(len(calls),2)

    def test_communication_error_propagates(self):
        def read(*args):raise RuntimeError('timeout')
        with self.assertRaisesRegex(RuntimeError,'timeout'):check.read_drive_settings(read,12)

if __name__=='__main__':unittest.main()
