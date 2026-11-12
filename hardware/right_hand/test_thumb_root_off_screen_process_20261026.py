"""Verify the longer OFF wait, cancellation and stale-proof rejection."""
import contextlib,io,tempfile,unittest
from pathlib import Path
from unittest.mock import Mock,patch
import thumb_root_off_screen_process_20261026 as subject
class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,_):self.t+=40.
class Process:
    def __init__(self,output,finish_at=None,clock=None):self.output=output;self.finish_at=finish_at;self.clock=clock;self.returncode=None;self.terminated=False
    def poll(self):
        if self.returncode is None and self.finish_at is not None and self.clock.t>=self.finish_at:
            self.output.write_text('{}',encoding='utf-8');self.returncode=0
        return self.returncode
    def terminate(self):self.terminated=True;self.returncode=-15
    def wait(self,timeout):return self.returncode
class ProcessChecks(unittest.TestCase):
    def test_all_motor_goals_and_order_equal_pre_timeout_fix(self):
        import json
        from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,ThumbRootEmptyPlan
        basis=json.loads((ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454/thumb_root_empty_fixture_v5_corner_resolution/before_timeout_fix/basis.json').read_bytes())
        request=json.loads((ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454/thumb_root_empty_fixture_v5_corner_resolution/request.json').read_bytes())
        request['plan_version']=ThumbRootEmptyPlan.PLAN_VERSION
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'request.json';path.write_text(json.dumps(request),encoding='utf-8')
            p=ThumbRootEmptyPlan(path);rows=p.plan(p.source_position())
            self.assertEqual(p.source_position().tolist(),basis['start_ticks'])
            self.assertEqual(p.plan_digest(p.source_position()),basis['plan_digest'])
            self.assertEqual([[phase,q.tolist()] for phase,q in rows],basis['goal_sequence'])
    def run_with(self,folder,*,finish_at=None,cancel=lambda:None):
        clock=Clock();output=Path(folder)/'proof.json';process=Process(output,finish_at,clock)
        with patch.object(subject,'wsl_path',side_effect=lambda p:str(p)),contextlib.redirect_stdout(io.StringIO()):
            result=subject.run_screen(Path(folder)/'request.json',output,cancel=cancel,sleep=clock.sleep,now=clock.now,
                popen=lambda *a,**k:process,checker=Path(folder)/'checker.py',failure_details=lambda p:'failure')
        return result,process,clock
    def test_checker_finishing_at200s_is_not_killed_at_old120s(self):
        with tempfile.TemporaryDirectory() as d:
            result,p,clock=self.run_with(d,finish_at=200.)
            self.assertTrue(result.is_file());self.assertFalse(p.terminated);self.assertEqual(clock.t,200.)
    def test_wait_still_times_out_and_terminates_child(self):
        with tempfile.TemporaryDirectory() as d:
            clock=Clock();out=Path(d)/'proof.json';p=Process(out,None,clock)
            with patch.object(subject,'wsl_path',side_effect=str),contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError,'240s; no writes'):
                    subject.run_screen(Path(d)/'request.json',out,cancel=lambda:None,sleep=clock.sleep,now=clock.now,
                        popen=lambda *a,**k:p,checker=Path(d)/'checker.py',failure_details=lambda p:'failure')
            self.assertTrue(p.terminated);self.assertFalse(out.exists())
    def test_cancel_terminates_without_creating_proof(self):
        with tempfile.TemporaryDirectory() as d:
            clock=Clock();out=Path(d)/'proof.json';p=Process(out,None,clock)
            with patch.object(subject,'wsl_path',side_effect=str),contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(KeyboardInterrupt):
                    subject.run_screen(Path(d)/'request.json',out,cancel=lambda:(_ for _ in ()).throw(KeyboardInterrupt()),
                        sleep=clock.sleep,now=clock.now,popen=lambda *a,**k:p,checker=Path(d)/'checker.py',failure_details=lambda p:'failure')
            self.assertTrue(p.terminated);self.assertFalse(out.exists())
    def test_existing_proof_rejected_before_launch(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'proof.json';out.write_text('{}',encoding='utf-8');launch=Mock()
            with self.assertRaisesRegex(RuntimeError,'stale proof'):
                subject.run_screen(Path(d)/'request.json',out,cancel=lambda:None,popen=launch,checker=Path(d)/'checker.py',failure_details=lambda p:'failure')
            launch.assert_not_called()
if __name__=='__main__':unittest.main()
