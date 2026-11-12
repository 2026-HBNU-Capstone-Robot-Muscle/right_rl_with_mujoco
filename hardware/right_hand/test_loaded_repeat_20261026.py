"""Precise recovery deadline, read-paced polling, unchanged route and shutdown."""
import contextlib,io,json,subprocess,sys,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import sequence_flip_checked as checked
import loaded_repeat_voltage_20261026 as precise
import loaded_thumb_voltage_sample_20261026 as previous_sampler
import test_loaded_thumb_rotation_20261026 as previous_tests
from loaded_repeat_plan_20261026 import ROOT,FIXTURE,LoadedRepeatPlan
from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan
import repeat_support_model_resolution_20261026 as support_resolution
from loaded_repeat_gate_20261026 import validate_proof

class Clock:
    def __init__(self):self.elapsed=0.;self.sleeps=[]
    def precise(self):return 5000.+self.elapsed
    def coarse(self):return 100.+np.floor(self.elapsed/.015625)*.015625
    def sleep(self,seconds):self.sleeps.append(seconds);self.elapsed+=seconds

class RecoveryChecks(unittest.TestCase):
    def replay(self,values,duration=.0265,*,old=False,mutate=None,events=None,scoped=True,cancel=None):
        c=Clock();report=dict(enabled_at=100.)
        if events is not None:report['voltage_events']=events
        command=np.full(16,2500,dtype=int)
        state=dict(torque=np.ones(16,dtype=int),error=np.zeros(16,dtype=int),watchdog=np.full(16,25),
            current_cap=checked.CAP.copy(),velocity=np.zeros(16,dtype=int),acceleration=np.zeros(16,dtype=int),
            goal=command.copy(),position=command.copy(),voltage=np.full(16,38),temperature=np.full(16,23),current=np.zeros(16,dtype=int),
            **{k:v.copy() for k,v in checked.GAINS.items()})
        calls=[]
        bus=types.SimpleNamespace(plan_module=types.SimpleNamespace(PRECISE_READ_PACED_VOLTAGE=scoped,
            IMMEDIATE_HEALTHY_VOLTAGE_RECHECK=True,SUPERVISED_LOADED_THUMB_MODE=True,CAPTURE_PLACEMENT=True),
            cancel_callback=cancel or (lambda:None))
        def read(bus,command,report,phase,snapshot_check=None):
            i=len(calls);c.elapsed+=duration[i] if isinstance(duration,list) else duration
            fresh={k:v.copy() for k,v in state.items()};v=values[min(i,len(values)-1)]
            fresh['voltage']=np.full(16,v,dtype=int) if np.isscalar(v) else np.asarray(v).copy()
            if mutate:mutate(fresh,i)
            if snapshot_check:snapshot_check(fresh)
            calls.append(fresh);return fresh,np.abs(fresh['position']-command)*checked.TICK
        sampler=previous_sampler if old else precise
        with patch.object(checked,'_read_sample',side_effect=read),patch.object(precise.time,'perf_counter',side_effect=c.precise),\
             patch.object(precise.time,'monotonic',side_effect=c.coarse),patch.object(precise.time,'sleep',side_effect=c.sleep),\
             contextlib.redirect_stdout(io.StringIO()):
            try:value=sampler.sample(bus,command,report,'counterfactual');error=None
            except BaseException as exc:value=None;error=exc
        return report,c,calls,value,error
    def test_actual_failed_voltage_sequence_fits_without_extra_sleeps(self):
        r=json.loads((ROOT/'loaded_thumb_rotation_20261026_163144_150855/result.json').read_bytes());e=r['voltage_events'][-1]
        rows=[s for s in r['samples'] if e['t']-1e-6<=s['t']<=e['t']+e['elapsed_ms']/1000+1e-6]
        values=[np.rint(np.asarray(s['voltage_v'])*10).astype(int) for s in rows]
        durations=[s['read_ms']/1000 for s in rows]
        old,_,_,_,failure=self.replay(values,durations,old=True)
        self.assertIsInstance(failure,RuntimeError);self.assertEqual(old['voltage_events'][0]['status'],'stopped')
        new,c,calls,value,error=self.replay(values,durations)
        self.assertIsNone(error);self.assertLess(new['voltage_events'][0]['elapsed_ms'],150)
        self.assertEqual(new['voltage_events'][0]['healthy_confirmation_count'],2);self.assertEqual(c.sleeps,[])
        self.assertEqual(len(calls),6)
    def test_clock_epochs_are_not_mixed_in_event_timestamps(self):
        r,c,reads,value,error=self.replay([36,38,38])
        self.assertIsNone(error);self.assertLess(r['voltage_events'][0]['t'],1)
        self.assertAlmostEqual(r['voltage_events'][0]['elapsed_ms'],53.,places=5)
    def test_two_healthy_reads_after_true_deadline_still_stop(self):
        r,c,reads,value,error=self.replay([36,37,38,37,38,38],duration=.031)
        self.assertIsInstance(error,RuntimeError);self.assertEqual(r['voltage_events'][0]['status'],'stopped')
    def test_read_faster_than_poll_period_sleeps_only_remaining_time(self):
        r,c,reads,value,error=self.replay([36,37,38,38],duration=.002)
        self.assertIsNone(error);self.assertEqual(len(c.sleeps),1);self.assertAlmostEqual(c.sleeps[0],.008)
    def test_3p5_stops_immediately(self):
        r,c,reads,value,error=self.replay([35]);self.assertRegex(str(error),'immediate stop')
        self.assertEqual(len(reads),1);self.assertNotIn('voltage_events',r)
    def test_one_healthy_read_is_not_enough(self):
        r,c,reads,value,error=self.replay([36,38,37]);self.assertIsInstance(error,RuntimeError);self.assertIsNone(value)
    def test_persistent_3p6_and_read_limit_are_not_relaxed(self):
        r,c,reads,value,error=self.replay([36],duration=.002)
        self.assertIsInstance(error,RuntimeError);self.assertEqual(len(reads),7)
    def test_all_sixteen_ids_must_recover(self):
        partial=np.full(16,38);partial[15]=37
        r,c,reads,value,error=self.replay([36,partial]);self.assertIsInstance(error,RuntimeError)
    def test_repeated_dips_still_stop(self):
        r,c,reads,value,error=self.replay([36,38,38],events=[dict(t=0.),dict(t=0.)])
        self.assertRegex(str(error),'Repeated undervoltage');self.assertEqual(len(reads),1)
    def test_other_motor_health_faults_are_not_hidden(self):
        changes=(('torque',0),('error',1),('watchdog',255),('current',600),('temperature',46))
        for key,value in changes:
            def mutate(s,i):
                if i==1:s[key][0]=value
            with self.subTest(key=key):
                r,c,reads,result,error=self.replay([36,38,38],mutate=mutate)
                self.assertIsInstance(error,RuntimeError);self.assertEqual(len(reads),2)
    def test_tracking_fault_prevents_resume(self):
        def mutate(s,i):
            if i==1:s['position'][0]+=200
        r,c,reads,value,error=self.replay([36,38,38],mutate=mutate);self.assertRegex(str(error),'tracking')
    def test_q_in_confirmation_still_stops(self):
        def cancel():raise KeyboardInterrupt('Synthetic Q')
        r,c,reads,value,error=self.replay([36,38,38],cancel=cancel)
        self.assertIsInstance(error,KeyboardInterrupt);self.assertEqual(r['voltage_events'][0]['status'],'stopped')
    def test_sampler_is_opt_in(self):
        r,c,reads,value,error=self.replay([38],scoped=False);self.assertIsInstance(error,RuntimeError);self.assertEqual(reads,[])

class SameMotionChecks(unittest.TestCase):
    def test_live_bus_uses_the_new_sampler(self):
        from leap_loaded_repeat_20261026 import LoadedRepeatBus
        self.assertIs(LoadedRepeatBus.checked_sample,precise.sample)
        self.assertTrue(LoadedRepeatPlan.PRECISE_READ_PACED_VOLTAGE)
    def test_support_resolution_cannot_hide_unknown_or_larger_contacts(self):
        pair=support_resolution.metadata()['pairs'][0];limit=support_resolution.ZERO_MM
        self.assertEqual(support_resolution.classify({pair:limit}),{})
        self.assertEqual(support_resolution.classify({pair:limit*1.01}),{pair:limit*1.01})
        unknown='unknown_body|support_book';self.assertEqual(support_resolution.classify({unknown:limit}),{unknown:limit})
        self.assertFalse(support_resolution.accepted_discard(pair,float('nan')))
    def test_positions_order_phases_and_goal_pacing_match_successful_implementation(self):
        p=LoadedRepeatPlan(FIXTURE/'request.json');raw=p.source_position()
        new=p.plan(raw);old=LoadedThumbRotationPlan.plan(p,raw)
        self.assertEqual([s for s,q in new],[s for s,q in old]);np.testing.assert_array_equal([q for s,q in new],[q for s,q in old])
        self.assertEqual(p.SAMPLE_PERIOD,.075);self.assertEqual(p.ON_TIMEOUT,210)
    def test_new_default_does_not_open_devices(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_loaded_repeat_20261026.py')],capture_output=True,text=True,encoding='utf-8',timeout=8)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)

class RepeatRunnerChecks(previous_tests.LoadedRunnerChecks):
    def run_case(self,fault=None):
        with patch.object(previous_tests,'LoadedThumbRotationPlan',LoadedRepeatPlan),patch.object(previous_tests,'FIXTURE',FIXTURE):
            return super().run_case(fault)
    def test_default_opens_no_devices(self):SameMotionChecks().test_new_default_does_not_open_devices()

@unittest.skipUnless((FIXTURE/'screen.json').exists(),'Native screen not yet complete')
class RepeatProofChecks(unittest.TestCase):
    def test_complete_saved_proof_is_accepted(self):
        r=validate_proof(LoadedRepeatPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
        self.assertTrue(r['experimental_loaded_trial'])
    def reject(self,mutate):
        r=json.loads((FIXTURE/'screen.json').read_bytes());mutate(r)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'changed.json';path.write_text(json.dumps(r),encoding='utf-8')
            with self.assertRaises(RuntimeError):validate_proof(LoadedRepeatPlan(FIXTURE/'request.json'),path)
    def test_larger_support_edge_cannot_reuse_resolution(self):
        pair=support_resolution.metadata()['pairs'][0]
        self.reject(lambda r:r['support_resolution_discarded_maximums_by_front'][0].__setitem__(pair,support_resolution.ZERO_MM*1.01))
    def test_longer_voltage_confirmation_budget_is_rejected(self):
        self.reject(lambda r:r['fresh_voltage_read_policy'].__setitem__('confirmation_budget_s',.2))
if __name__=='__main__':unittest.main()
