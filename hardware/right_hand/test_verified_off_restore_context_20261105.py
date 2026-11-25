import unittest
from types import SimpleNamespace
from verified_off_restore_context_20261105 import post_off_restore

def failed_camera():raise RuntimeError('Camera failed')
class Tests(unittest.TestCase):
    def bus(self,off):return SimpleNamespace(report=dict(all_torque_off_verified=off,initial=dict(d_gain=[1900]*16)),cancel_callback=failed_camera)
    def test_ON_camera_failure_is_preserved(self):
        b=self.bus(False)
        with self.assertRaisesRegex(RuntimeError,'Camera failed'):
            with post_off_restore(b,80,[1900]*16,terminal_cancel=lambda:None):b.cancel_callback()
        self.assertIs(b.cancel_callback,failed_camera)
    def test_verified_OFF_original_restore_can_finish(self):
        b=self.bus(True)
        with post_off_restore(b,80,[1900]*16,terminal_cancel=lambda:None):b.cancel_callback()
        self.assertIs(b.cancel_callback,failed_camera);self.assertIn('post_off_restore_policy',b.report)
    def test_unexpected_target_is_rejected(self):
        b=self.bus(True)
        with self.assertRaises(RuntimeError):
            with post_off_restore(b,80,[200]*16,terminal_cancel=lambda:None):self.fail('Must not yield')
    def test_torque_enable_is_outside_scope(self):
        b=self.bus(True)
        with self.assertRaisesRegex(RuntimeError,'Camera failed'):
            with post_off_restore(b,64,[1]*16,terminal_cancel=lambda:None):b.cancel_callback()
    def test_original_callback_restored_after_write_failure(self):
        b=self.bus(True)
        with self.assertRaisesRegex(ValueError,'readback failed'):
            with post_off_restore(b,80,[1900]*16,terminal_cancel=lambda:None):raise ValueError('readback failed')
        self.assertIs(b.cancel_callback,failed_camera)
    def test_terminal_stop_is_preserved_after_OFF(self):
        b=self.bus(True)
        def stop():raise KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            with post_off_restore(b,80,[1900]*16,terminal_cancel=stop):b.cancel_callback()
        self.assertIs(b.cancel_callback,failed_camera)
if __name__=='__main__':unittest.main()
