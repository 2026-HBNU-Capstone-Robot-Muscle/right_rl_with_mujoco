"""Operator flow and stopped/faulted hold checks, without camera or serial."""
import unittest,contextlib,io,json
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import cube_grasp_hold_20261116 as c
from natural_cradle_grasp_plan_20261030 import validate_candidate,ALLOWED

class Tests(unittest.TestCase):
    def test_queue_is_original_close_only(self):
        z=c.candidate();old=validate_candidate();mask=np.isin(old['phases'],ALLOWED[:2])
        self.assertTrue(np.array_equal(z['goals'],old['goals'][mask]))
        p=c.Plan(c.REQUEST);rows=p.plan(p.source_position())
        self.assertEqual(p.CAPTURE_BEFORE_PHASE,ALLOWED[0]);self.assertFalse(p.EMPTY_RELEASE_ONLY)
        self.assertFalse(any(name in ALLOWED[2:] for name,_ in rows))
        self.assertTrue(np.array_equal(rows[-1][1],z['goals'][-1]))
    def hold(self,fail_at=None,pending=False,placed=True,timeout=.3):
        q=c.candidate()['goals'][-1];clock=[0.];checks=[]
        obj=object.__new__(c.Capture);obj.now=lambda:clock[0]
        obj.sleep=lambda n:clock.__setitem__(0,clock[0]+n);obj.cancel=lambda:None
        obj.keys=SimpleNamespace(arm=lambda:None,consume_space=lambda:len(checks)>=5)
        def sample(bus,goal,report,phase,**kw):
            checks.append(phase)
            if len(checks)==fail_at:raise RuntimeError('health fault')
            s={'position':q.copy(),'goal':q.copy()}
            if 'snapshot_check' in kw:kw['snapshot_check'](s)
            return s,np.zeros(16)
        bus=SimpleNamespace(pending=[1] if pending else [],checked_sample=sample,plan_module=SimpleNamespace(ON_TIMEOUT=timeout,SAMPLE_PERIOD=.075),write=lambda *a,**kw:self.fail('hold wrote goal'))
        return obj,bus,q,dict(cube_placement_confirmed=placed,enabled_at=0.),checks
    def test_hold_no_automatic_open_or_goal_writes(self):
        obj,bus,q,r,checks=self.hold(timeout=500);obj.finish(bus,q,r)
        self.assertEqual(r['hold_end_reason'],'operator_SPACE');self.assertEqual(r['final_hold_goal_writes'],0)
        self.assertFalse(r['automatic_open']);self.assertFalse(r['physical_grasp_verified'])
    def test_timeout(self):
        obj,bus,q,r,_=self.hold(timeout=.2);obj.finish(bus,q,r);self.assertEqual(r['hold_end_reason'],'500s_total_on_limit')
    def test_fault_stops_hold(self):
        obj,bus,q,r,checks=self.hold(fail_at=3)
        with self.assertRaisesRegex(RuntimeError,'health fault'):obj.finish(bus,q,r)
        self.assertEqual(len(checks),3)
    def test_no_placement_no_closing_complete_claim(self):
        obj,bus,q,r,checks=self.hold(placed=False)
        with self.assertRaises(RuntimeError):obj.finish(bus,q,r)
        self.assertEqual(checks,[])
    def test_unfinished_queue_rejected(self):
        obj,bus,q,r,checks=self.hold(pending=True)
        with self.assertRaises(RuntimeError):obj.finish(bus,q,r)
    def test_review_and_default_do_not_open_devices(self):
        c.validate_review(c.Plan(c.REQUEST),c.PROOF)
        with patch('sys.argv',['cube']),patch('dynamixel_sdk.PortHandler',side_effect=AssertionError('serial')),patch('camera_stream.CameraStream',side_effect=AssertionError('camera')),patch('subprocess.Popen',side_effect=AssertionError('process')),contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(c.main(),0)
        self.assertIn('CHECK ONLY',output.getvalue())

if __name__=='__main__':unittest.main()
