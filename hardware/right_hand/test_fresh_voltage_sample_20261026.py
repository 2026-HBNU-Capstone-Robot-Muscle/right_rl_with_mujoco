"""Replay measured voltage recovery without devices; retain stop boundaries."""
import contextlib,io,json,types,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import sequence_flip_checked as checked
import fresh_voltage_sample_20261026 as fresh
import test_empty_camera_gap_hold_20261026 as gaptests
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan

ROOT=Path(__file__).resolve().parent
class Clock:
    def __init__(self):self.t=0.;self.sleeps=[]
    def now(self):return self.t
    def sleep(self,seconds):self.sleeps.append(seconds);self.t+=seconds
class VoltageChecks(unittest.TestCase):
    def replay(self,voltages,*,duration=.031,old=False,mutate=None,events=None,sampler=None,loaded=False):
        r=json.loads((ROOT/'thumb_clear_root_empty_20261026_115050_905503/result.json').read_bytes())
        last=r['samples'][-1];command=np.array(last['goal']);clock=Clock();report=dict(enabled_at=0.)
        if events:report['voltage_events']=events
        base={k:np.asarray(v).copy() for k,v in r['initial'].items()}
        base.update(torque=np.ones(16,dtype=int),watchdog=np.full(16,25),goal=command,position=np.array(last['actual']),
            current=np.array(last['current_ma']),current_cap=checked.CAP,velocity=np.zeros(16),acceleration=np.zeros(16),**checked.GAINS)
        bus=types.SimpleNamespace(plan_module=types.SimpleNamespace(IMMEDIATE_HEALTHY_VOLTAGE_RECHECK=True,CAPTURE_PLACEMENT=loaded,SUPERVISED_LOADED_THUMB_MODE=loaded),cancel_callback=lambda:None)
        calls=[]
        def read(bus,command,report,phase,snapshot_check=None):
            i=len(calls);clock.t+=duration
            state={k:np.asarray(v).copy() for k,v in base.items()};value=voltages[min(i,len(voltages)-1)]
            state['voltage']=np.full(16,value,dtype=int) if np.isscalar(value) else np.array(value)
            if mutate:mutate(state,i)
            if snapshot_check:snapshot_check(state)
            calls.append(state)
            return state,np.abs(state['position']-command)*checked.TICK
        fn=checked.sample if old else (sampler or fresh).sample
        with patch.object(checked,'_read_sample',side_effect=read),patch.object(fresh.time,'monotonic',side_effect=clock.now),patch.object(fresh.time,'sleep',side_effect=clock.sleep),contextlib.redirect_stdout(io.StringIO()):
            try:result=fn(bus,command,report,'replay');failure=None
            except BaseException as exc:result=None;failure=exc
        return report,clock,calls,result,failure
    def test_measured_recovery_gets_two_fresh_good_reads_within_150ms(self):
        data=json.loads((ROOT/'thumb_clear_root_empty_20261026_115050_905503/result.json').read_bytes())
        event=data['voltage_events'][0]
        samples=[s for s in data['samples'] if s['t']>=event['t']-1e-6][-5:]
        voltage=[np.rint(np.asarray(s['voltage_v'])*10).astype(int) for s in samples]
        old,_,_,_,failure=self.replay(voltage,old=True)
        self.assertIsInstance(failure,RuntimeError);self.assertGreaterEqual(old['voltage_events'][0]['elapsed_ms'],150)
        r,c,reads,result,failure=self.replay(voltage)
        self.assertIsNone(failure);self.assertEqual(len(reads),5)
        e=r['voltage_events'][0];self.assertEqual(e['healthy_confirmation_count'],2)
        self.assertLess(e['elapsed_ms'],150);self.assertEqual(c.sleeps,[.01,.01])
    def test_second_healthy_read_after_deadline_still_stops(self):
        r,c,reads,result,error=self.replay([36,37,37,38,38],duration=.034)
        self.assertIsInstance(error,RuntimeError);self.assertEqual(r['voltage_events'][0]['status'],'stopped')
    def test_3p5_stops_without_confirmation(self):
        r,c,reads,result,error=self.replay([35])
        self.assertRegex(str(error),'immediate stop');self.assertEqual(len(reads),1);self.assertNotIn('voltage_events',r)
    def test_persistent_low_voltage_does_not_resume(self):
        r,c,reads,result,error=self.replay([36])
        self.assertIsInstance(error,RuntimeError);self.assertLessEqual(len(reads),7)
    def test_one_recovered_read_is_insufficient(self):
        r,c,reads,result,error=self.replay([36,38,37,37,37,37,37])
        self.assertIsInstance(error,RuntimeError);self.assertIsNone(result)
    def test_tracking_fault_during_recovery_is_not_ignored(self):
        def mutate(s,i):
            if i==1:s['position'][0]=s['goal'][0]+200
        r,c,reads,result,error=self.replay([36,38,38],mutate=mutate)
        self.assertRegex(str(error),'tracking');self.assertEqual(len(reads),2)
    def test_q_during_confirmation_still_interrupts(self):
        def mutate(s,i):
            if i==1:raise KeyboardInterrupt('Synthetic Q')
        r,c,reads,result,error=self.replay([36,38,38],mutate=mutate)
        self.assertIsInstance(error,KeyboardInterrupt);self.assertEqual(r['voltage_events'][0]['status'],'stopped')
    def test_repeated_low_voltage_trip_is_preserved(self):
        r,c,reads,result,error=self.replay([36,38,38],events=[dict(t=0.),dict(t=0.)])
        self.assertRegex(str(error),'Repeated undervoltage');self.assertEqual(len(reads),1)

class SamplerHookChecks(unittest.TestCase):
    def test_real_runner_calls_bus_specific_sampler(self):
        calls=[]
        def scoped_sample(*args,**kwargs):
            calls.append(args[3]);return checked.sample(*args,**kwargs)
        with patch.object(gaptests.FakeBus,'checked_sample',staticmethod(scoped_sample),create=True):
            p,r,b,track=gaptests.RunnerChecks('test_resume_preserves_every_target_once_and_keeps_off_restore').run_case()
        self.assertGreater(len(calls),len(p.plan(p.source_position())))
        self.assertTrue(any(phase.startswith('camera_gap_hold:') for phase in calls))
        self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
        from leap_thumb_clear_root_empty_20261026 import ThumbRootEmptyBus
        self.assertIs(ThumbRootEmptyBus.checked_sample,fresh.sample)
        self.assertTrue(ThumbRootEmptyPlan.IMMEDIATE_HEALTHY_VOLTAGE_RECHECK)
if __name__=='__main__':unittest.main()
