"""Device-free safety/control-flow checks of the new prefix-only mode."""
import contextlib,copy,io,itertools,json,subprocess,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np
import leap_quarter_support_photo_20261025 as mode
import quarter_motion_runner_20261025 as runner
from loaded_quarter_trial_plan_20261025 import LoadedQuarterTrialPlan,TURN_PHASE,ROOT,motion_goals
from support_photo_path_plan_20261025 import validate_proof
F=mode.FIXTURE
def state(raw):
    s=dict(position=raw.copy(),goal=raw.copy())
    s.update({k:np.full(16,i+100) for i,k in enumerate(('current_cap','acceleration','velocity','watchdog','d_gain','i_gain','p_gain'))})
    return s
class FakeBus:
    def __init__(self,plan,report):
        self.plan_module=plan;self.report=report;self.state=state(plan.source_position());self.loaded=0;self.off_count=0;self.writes=[]
    def bind(self,a):pass
    def adopt(self,a):pass
    def snapshot(self):return copy.deepcopy(self.state)
    def write(self,addr,size,v,verify=True):
        self.writes.append((addr,np.asarray(v).copy()))
        if addr==116:
            if self.report.get('phase')==TURN_PHASE:self.loaded+=1;raise AssertionError('Support photo sent a loaded target')
            self.state['position']=np.asarray(v).copy();self.state['goal']=np.asarray(v).copy()
    def off(self):self.off_count+=1
def protocol_for(bus,fail=None):
    p=mode.SupportPhotoCapture.__new__(mode.SupportPhotoCapture);p._observer=Mock(return_value=Mock())
    def photo(name,phase,observe):
        if fail=='photo' and phase=='place_cube':raise RuntimeError('Synthetic camera failure')
        return bus.snapshot(),name
    def space(phase,observe):
        if fail=='remove' and phase=='remove_cube':raise KeyboardInterrupt('Synthetic user stop')
        return bus.snapshot()
    p._photo=Mock(side_effect=photo);p._space=Mock(side_effect=space);return p
class PhotoChecks(unittest.TestCase):
    def setUp(self):self.plan=mode.SupportPhotoPlan(F/'request.json')
    def run_prefix(self,fail=None):
        report={};bus=FakeBus(self.plan,report);initial=bus.snapshot();p=protocol_for(bus,fail);clock=itertools.count()
        def sample(b,c,r,phase,snapshot_check):
            s=b.snapshot();snapshot_check(s);return s,np.zeros(16)
        exc=mode.SupportPhotoComplete if fail is None else RuntimeError if fail=='photo' else KeyboardInterrupt
        with patch.object(runner,'check_off'),patch.object(runner.checked,'health'),patch.object(runner.checked,'sample',side_effect=sample),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(exc):runner.run(bus,initial,report,cancel=lambda:None,sleep=lambda x:None,now=lambda:next(clock)*.002,phase_callback=p.place,prepared_callback=p.finish)
        return report,bus,p,initial
    def test_photo_preparation_uses_unchanged_parent_goals_and_bounds(self):
        parent=LoadedQuarterTrialPlan.__new__(LoadedQuarterTrialPlan);parent.__dict__.update(self.plan.__dict__)
        rows=self.plan.plan(self.plan.source_position());original=parent.plan(parent.source_position())
        self.assertEqual([p for p,q in rows],[p for p,q in original]);np.testing.assert_array_equal([q for p,q in rows],[q for p,q in original])
        values=np.vstack([self.plan.source_position()]+[q for phase,q in rows]);self.assertLessEqual(abs(np.diff(values,axis=0)).max(),2)
        self.assertTrue(validate_proof(self.plan,F/'screen.json')['cube_disabled_entire_screen'])
    def test_checkpoint_does_not_send_any_goal(self):
        report={};bus=FakeBus(self.plan,report);p=protocol_for(bus)
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(mode.SupportPhotoComplete):p.place(bus,motion_goals()[0],report,Mock())
        self.assertEqual(bus.writes,[]);self.assertEqual([c.args[0] for c in p._space.call_args_list],['place_cube','remove_cube'])
        self.assertTrue(report['cube_removal_confirmed']);self.assertFalse(report['cube_present']);self.assertEqual(report['loaded_goal_count'],0)
    def test_runner_restores_and_sends_no_loaded_target(self):
        report,bus,p,initial=self.run_prefix();self.assertEqual(bus.loaded,0);self.assertEqual(bus.off_count,1)
        self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])
        self.assertEqual(report['last_issued_command'],motion_goals()[0].tolist())
        self.assertFalse(report['cube_rotation_requested']);self.assertTrue(report['diagnostic_completion_requested'])
        for addr,size,key in ((102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),(98,1,'watchdog'),(80,2,'d_gain'),(82,2,'i_gain'),(84,2,'p_gain')):
            value=next(v for a,v in reversed(bus.writes) if a==addr);np.testing.assert_array_equal(value,initial[key])
    def test_camera_failure_still_off_without_loaded_target(self):
        report,bus,p,_=self.run_prefix('photo');self.assertEqual(bus.loaded,0);self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored']);self.assertNotIn('cube_removal_confirmed',report)
    def test_stop_before_removal_still_off_and_not_recorded_removed(self):
        report,bus,p,_=self.run_prefix('remove');self.assertEqual(bus.loaded,0);self.assertTrue(report['settings_restored']);self.assertNotIn('cube_removal_confirmed',report)
    def test_default_does_not_open_devices(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_quarter_support_photo_20261025.py')],capture_output=True,text=True,encoding='utf-8',timeout=5)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)
    def test_main_persists_expected_completion_after_normal_shutdown(self):
        import msvcrt
        class TopBus:
            def __init__(self,sdk,r):self.report=r;self.port=Mock();self.port.openPort.return_value=True;self.port.setBaudRate.return_value=True
            def preflight(self):return state(self_plan.source_position())
            def block(self,*a):return {'torque':np.zeros(16)}
        self_plan=self.plan;camera=Mock();camera.finish.return_value={'status':'complete'}
        def prepare(bus,folder,report,**kwargs):
            report.update(all_torque_off_verified=True,settings_restored=True,cube_removal_confirmed=True,last_issued_command=motion_goals()[0].tolist(),samples=[])
            raise mode.SupportPhotoComplete()
        with tempfile.TemporaryDirectory() as folder,patch.object(mode,'ROOT',Path(folder)),patch.object(mode,'require_mode_review'),patch.object(mode,'Camera',return_value=camera),patch.object(mode,'LoadedQuarterBus',TopBus),patch.object(mode,'prepare_and_run',side_effect=prepare),patch.object(mode,'record_off_endpoint'),patch.object(msvcrt,'kbhit',return_value=True),patch.object(msvcrt,'getwch',return_value=' '),patch.object(sys,'argv',['photo','--execute']),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(mode.main(),0);r=json.loads((Path(folder)/'leap_quarter_support_photo_result.json').read_bytes())
            self.assertEqual(r['status'],'quarter_support_photo_complete');self.assertEqual(r['loaded_goal_count'],0);self.assertTrue(r['stopped_before_loaded_motion'])
if __name__=='__main__':unittest.main()
