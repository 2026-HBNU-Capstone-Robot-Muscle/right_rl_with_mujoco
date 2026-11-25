"""No-device regression checks for observed OFF preload failures."""
import copy,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from horizontal_preload_confirmation_20261027 import confirm_off_preload,POLICY
import horizontal_quarter_plan_20261027 as p
import horizontal_quarter_motion_runner_20261027 as runner
import sequence_flip_checked as checked

class PreloadTests(unittest.TestCase):
    def setUp(self):
        self.plan=p.HorizontalQuarterPlan(p.REQUEST);self.original=self.plan.source_position();source=p.ROOT/'horizontal_quarter_trial_20261027_141354_998533/result.json';sample=json.loads(source.read_bytes())['off_preload_snapshots'][0];self.state={k:np.array(v) for k,v in sample.items()};self.state['position']=self.original.copy();self.state['goal']=self.original.copy();self.clock=[0.];self.report={}
    def make_bus(self,shifts,mutate=None,read_seconds=.026):
        states=[]
        for shift in shifts:
            state=copy.deepcopy(self.state);state['position'][9]+=shift;state['goal'][9]+=shift
            if mutate:mutate(state)
            states.append(state)
        index=[0]
        def snapshot():
            self.clock[0]+=read_seconds;state=states[min(index[0],len(states)-1)];index[0]+=1;return copy.deepcopy(state)
        return SimpleNamespace(snapshot=snapshot,plan_module=self.plan,write=Mock(side_effect=AssertionError('unexpected write')))
    def run_confirmation(self,bus,cancel=lambda:None):
        return confirm_off_preload(bus,self.original,self.report,cancel=cancel,now=lambda:self.clock[0],sleep=lambda seconds:self.clock.__setitem__(0,self.clock[0]+seconds))
    def test_three_fresh_reads_required(self):
        bus=self.make_bus([0,0,0]);self.run_confirmation(bus);self.assertEqual(self.report['off_preload_confirmation']['reads'],3);bus.write.assert_not_called()
    def test_two_tick_bistability_is_bounded_and_accepted(self):
        bus=self.make_bus([0,2,0]);state=self.run_confirmation(bus);self.assertEqual(state['position'][9],self.original[9]);self.assertTrue(self.report['off_preload_confirmation']['accepted']);self.assertLess(self.clock[0],.25);bus.write.assert_not_called()
    def test_saved_two_failures_now_get_a_third_read(self):
        for name in ('horizontal_quarter_trial_20261027_141354_998533','horizontal_quarter_trial_20261027_141539_818724'):
            record=json.loads((p.ROOT/name/'result.json').read_bytes());states=[{k:np.array(v) for k,v in row.items()} for row in record['off_preload_snapshots']];states.append(copy.deepcopy(states[-1]));index=[0];self.clock=[0.];self.report={};original=np.array(record['initial']['position'])
            def snapshot():
                self.clock[0]+=.026;value=states[index[0]];index[0]+=1;return value
            # Replay each historical failure against its own actual source,
            # rather than today's different OFF posture.
            request=json.loads(p.REQUEST.read_bytes());request['off_state']=record['initial']
            with tempfile.TemporaryDirectory(prefix='historical_preload_replay_',dir=p.ROOT) as temp:
                path=Path(temp)/'request.json';path.write_text(json.dumps(request),encoding='utf-8');plan=p.HorizontalQuarterPlan(path)
                bus=SimpleNamespace(snapshot=snapshot,plan_module=plan,write=Mock(side_effect=AssertionError('unexpected write')))
                confirm_off_preload(bus,original,self.report,cancel=lambda:None,now=lambda:self.clock[0],sleep=lambda s:self.clock.__setitem__(0,self.clock[0]+s))
            self.assertEqual(index[0],3);self.assertTrue(self.report['off_preload_confirmation']['accepted']);bus.write.assert_not_called()
    def test_more_than_two_tick_span_does_not_settle(self):
        bus=self.make_bus([0,3,0,3,0,3,0])
        with self.assertRaisesRegex(RuntimeError,'did not settle'):self.run_confirmation(bus)
        self.assertFalse(self.report['off_preload_confirmation']['accepted']);bus.write.assert_not_called()
    def test_more_than_four_tick_start_change_stops(self):
        bus=self.make_bus([5])
        with self.assertRaisesRegex(RuntimeError,'OFF state'):self.run_confirmation(bus)
        bus.write.assert_not_called()
    def test_low_voltage_still_stops(self):
        bus=self.make_bus([0],lambda s:s['voltage'].__setitem__(0,37))
        with self.assertRaises(RuntimeError):self.run_confirmation(bus)
    def test_temperature_still_stops(self):
        bus=self.make_bus([0],lambda s:s['temperature'].__setitem__(0,46))
        with self.assertRaises(RuntimeError):self.run_confirmation(bus)
    def test_torque_on_still_stops(self):
        bus=self.make_bus([0],lambda s:s['torque'].__setitem__(0,1))
        with self.assertRaises(RuntimeError):self.run_confirmation(bus)
    def test_watchdog_error_still_stops(self):
        bus=self.make_bus([0],lambda s:s['watchdog'].__setitem__(0,255))
        with self.assertRaises(RuntimeError):self.run_confirmation(bus)
    def test_wrong_gain_still_stops(self):
        bus=self.make_bus([0],lambda s:s['p_gain'].__setitem__(0,601))
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):self.run_confirmation(bus)
    def test_wrong_current_cap_still_stops(self):
        bus=self.make_bus([0],lambda s:s['current_cap'].__setitem__(0,451))
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):self.run_confirmation(bus)
    def test_goal_outside_original_window_still_stops(self):
        bus=self.make_bus([0],lambda s:s['goal'].__setitem__(0,self.original[0]+5))
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):self.run_confirmation(bus)
    def test_slow_read_does_not_enable(self):
        bus=self.make_bus([0],read_seconds=.26)
        with self.assertRaisesRegex(RuntimeError,'did not settle'):self.run_confirmation(bus)
        bus.write.assert_not_called()
    def test_cancellation_propagates(self):
        bus=self.make_bus([0])
        with self.assertRaises(KeyboardInterrupt):self.run_confirmation(bus,lambda:(_ for _ in ()).throw(KeyboardInterrupt()))
        self.assertEqual(self.report['off_preload_confirmation']['reads'],0)
    def test_post_enable_motion_and_cleanup_unchanged_outside_placement_and_error_log(self):
        # v7 intentionally replaces only the placement settling block. Enable,
        # queue consumption, observation, cancellation and OFF remain identical.
        def outside_placement(name):
            text=(p.ROOT/name).read_text(encoding='utf-8')
            tail=text[text.index('        bus.write(98,1,np.full(16,25))'):]
            marker="                if i==1 or held_guard is None:raise RuntimeError('Invalid placement checkpoint; OFF')\n"
            before,after=tail.split(marker,1)
            result=before+marker+after[after.index('                began=now()'):]
            logging="    except BaseException as exc:\n        report['primary_motion_error']=str(exc) or type(exc).__name__\n        report['primary_failed_phase']=report.get('phase')\n        raise\n"
            return result.replace(logging,'')
        self.assertEqual(outside_placement('camera_gap_motion_runner_20261026.py'),outside_placement('horizontal_quarter_motion_runner_20261027.py'))
    def test_confirmation_failure_cleans_up_without_enabling(self):
        events=[];bus=SimpleNamespace(plan_module=self.plan,bind=lambda a:None,snapshot=lambda:copy.deepcopy(self.state),write=lambda addr,size,values,**kwargs:events.append((addr,np.asarray(values).copy())),off=lambda:events.append((64,np.zeros(16))));report={}
        with patch.object(runner,'confirm_off_preload',side_effect=RuntimeError('test unsettled')):
            with self.assertRaisesRegex(RuntimeError,'test unsettled'):runner.run(bus,copy.deepcopy(self.state),report,cancel=lambda:None,phase_callback=lambda *a:None,prepared_callback=lambda *a:None)
        self.assertFalse(any(addr==64 and np.any(v) for addr,v in events));self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored']);self.assertEqual(sum(addr==116 for addr,_ in events),1)
        self.assertEqual(report['primary_motion_error'],'test unsettled')

if __name__=='__main__':unittest.main(verbosity=2)
