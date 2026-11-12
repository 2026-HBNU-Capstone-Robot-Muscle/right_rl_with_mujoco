"""Reject route changes, unfinished queues, and false removal claims."""
import copy,json,sys,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import forward_hold_plan_20261104 as plan
from forward_hold_capture_20261104 import ForwardHoldCapture

class ForwardHoldTests(unittest.TestCase):
    def setUp(self):self.z=plan.validate_candidate()
    def changed(self,data):
        original=np.load
        return patch.object(plan.np,'load',side_effect=lambda p,*a,**k:data if Path(p).resolve()==plan.LOADED.resolve() else original(p,*a,**k))
    def test_source_route_and_10s_fixed(self):
        z=self.z;old=plan.original_candidate();cut=int(np.flatnonzero(old['phases']=='stage_8_horizontal_hold')[0]);self.assertTrue(np.array_equal(z['goals'][:cut],old['goals'][:cut]));self.assertEqual(len(z['goals'])-cut,134);self.assertTrue(np.all(z['goals'][-134:]==z['goals'][-1]));self.assertAlmostEqual(134*.075,10.05)
    def test_reject_new_goal(self):
        d={k:self.z[k].copy() for k in self.z.files};d['goals'][5,0]+=1
        with self.changed(d),self.assertRaises(RuntimeError):plan.validate_candidate()
    def test_reject_open_phase(self):
        d={k:self.z[k].copy() for k in self.z.files};d['phases'][-1]='stage_9_loaded_open'
        with self.changed(d),self.assertRaises(RuntimeError):plan.validate_candidate()
    def test_reject_shorter_hold(self):
        d={k:self.z[k].copy() for k in self.z.files};d['goals']=d['goals'][:-1];d['phases']=d['phases'][:-1]
        with self.changed(d),self.assertRaises(RuntimeError):plan.validate_candidate()
    def test_preserves_native_boundaries(self):
        lo,hi=plan.limits();q=np.vstack([self.z['start'],self.z['goals']]);self.assertTrue(np.all(q>=lo));self.assertTrue(np.all(q<=hi));self.assertLessEqual(np.max(abs(np.diff(q,axis=0))),2)
    def protocol(self):
        c=ForwardHoldCapture.__new__(ForwardHoldCapture);c.guard=object();state={'position':self.z['goals'][-1].copy()};calls=[]
        c._observer=lambda *a:lambda phase:state;c._photo=lambda *a:(state,{'image':'test.jpg'});c._space=lambda phase,observe:(calls.append(phase) or state)
        return c,state,calls
    def test_finishing_requires_completed_queue(self):
        c,s,calls=self.protocol()
        with self.assertRaises(RuntimeError):c.finish(SimpleNamespace(pending=[1]),self.z['goals'][-1],{'cube_placement_confirmed':True})
        self.assertFalse(calls)
    def test_finishing_rejects_wrong_target(self):
        c,s,calls=self.protocol()
        with self.assertRaises(RuntimeError):c.finish(SimpleNamespace(pending=[]),self.z['goals'][-1]+1,{'cube_placement_confirmed':True})
        self.assertFalse(calls)
    def test_support_confirmation_not_removal(self):
        c,s,calls=self.protocol();r={'cube_placement_confirmed':True};c.finish(SimpleNamespace(pending=[]),self.z['goals'][-1],r)
        self.assertEqual(calls,['support_cube_for_off']);self.assertTrue(r['cube_support_confirmed_before_off']);self.assertFalse(r['cube_removal_confirmed']);self.assertTrue(r['cube_present']);self.assertFalse(r['physical90_verified']);self.assertFalse(r['physical_body_hold_verified'])
    def test_no_cube_requires_rejection(self):
        c,s,calls=self.protocol()
        with self.assertRaises(RuntimeError):c.finish(SimpleNamespace(pending=[]),self.z['goals'][-1],{})
        self.assertFalse(calls)
if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ForwardHoldTests));report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful(),devices_opened=False)
    (plan.OUT/'software_verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');raise SystemExit(0 if result.wasSuccessful() else 1)
