"""Replay observed partial goal packets and preserve voltage/motion guards."""
import ast,contextlib,copy,io,json,unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import sequence_flip_checked as checked
import horizontal_goal_confirmation_20261027 as confirmation
import horizontal_quarter_health_20261027 as health
import horizontal_quarter_plan_20261027 as plan_module
from horizontal_empty_recovery_20261027 import serial_roots

ROOT=plan_module.ROOT

def active(command,goal=None):
    s=dict(position=np.array(command).copy(),goal=np.array(command if goal is None else goal).copy(),
        torque=np.ones(16,int),watchdog=np.full(16,25),error=np.zeros(16,int),
        current=np.zeros(16,int),current_cap=np.full(16,checked.CAP),
        velocity=np.zeros(16,int),acceleration=np.zeros(16,int),voltage=np.full(16,40),temperature=np.full(16,24))
    for k,v in checked.GAINS.items():s[k]=np.asarray(v).copy()
    return s

class GoalPacketTests(unittest.TestCase):
    def setUp(self):
        r=json.loads((ROOT/'horizontal_quarter_trial_20261027_143018_624423/result.json').read_bytes())
        event=r['goal_readback_events'][0];self.command=np.array(event['command']);self.previous=np.array(event['previous_command']);self.initial=active(self.command,event['initial_readback'])
        self.bus=SimpleNamespace(previous_goal_command=self.previous,last_goal_command=self.command,goal_readback_confirmation=True,
            plan_module=SimpleNamespace(NEW_PAIR_GRASP_MODE=True),write=Mock(side_effect=AssertionError('write forbidden')))
        self.report={'enabled_at':0.};self.clock=[0.];self.read_count=0
    def run_confirm(self,states,read_seconds=.026):
        states=list(states)
        def reader():
            self.clock[0]+=read_seconds;self.read_count+=1
            s=copy.deepcopy(states[min(self.read_count-1,len(states)-1)])
            return s,np.abs(s['position']-self.command)*checked.TICK
        with patch.object(confirmation.time,'perf_counter',side_effect=lambda:self.clock[0]),patch.object(confirmation.time,'monotonic',return_value=10.),contextlib.redirect_stdout(io.StringIO()):
            return confirmation.confirm_previous_goal(self.bus,self.command,self.initial,self.report,'roots',read_sample=reader,validate=lambda s:checked.health(s,self.command))
    def test_observed_partial_vector_waits_for_two_exact_snapshots(self):
        self.run_confirm([active(self.command),active(self.command)]);self.assertEqual(self.read_count,2)
        self.assertEqual(self.report['goal_readback_events'][-1]['status'],'recovered');self.bus.write.assert_not_called()
    def test_entire_previous_vector_also_requires_two_exact_snapshots(self):
        self.initial['goal']=self.previous.copy();self.run_confirm([active(self.command),active(self.command)]);self.assertEqual(self.read_count,2)
    def test_unknown_one_tick_value_never_gets_a_read_retry(self):
        self.initial['goal'][0]-=1
        with self.assertRaisesRegex(RuntimeError,'unknown'):self.run_confirm([active(self.command)])
        self.assertEqual(self.read_count,0);self.bus.write.assert_not_called()
    def test_future_packet_is_rejected(self):
        bad=active(self.command);bad['goal'][15]+=1
        with self.assertRaisesRegex(RuntimeError,'known packets'):self.run_confirm([bad])
    def test_partial_write_that_never_completes_still_stops(self):
        with self.assertRaisesRegex(RuntimeError,'not confirmed'):self.run_confirm([self.initial])
        self.assertEqual(self.read_count,3);self.bus.write.assert_not_called()
    def test_nonconsecutive_exact_reads_do_not_resume(self):
        with self.assertRaisesRegex(RuntimeError,'not confirmed'):self.run_confirm([active(self.command),self.initial,active(self.command)])
    def test_two_exact_reads_after_deadline_do_not_resume(self):
        with self.assertRaisesRegex(RuntimeError,'not confirmed'):self.run_confirm([active(self.command)],read_seconds=.08)
        self.bus.write.assert_not_called()
    def test_packet_step_over_two_is_rejected(self):
        self.bus.previous_goal_command=self.command.copy();self.bus.previous_goal_command[0]-=3;self.initial['goal'][0]-=3
        with self.assertRaisesRegex(RuntimeError,'unknown'):self.run_confirm([active(self.command)])
    def test_last_issued_command_must_match(self):
        self.bus.last_goal_command=self.previous.copy()
        with self.assertRaisesRegex(RuntimeError,'unknown'):self.run_confirm([active(self.command)])
    def test_missing_history_stops(self):
        self.bus.previous_goal_command=None
        with self.assertRaisesRegex(RuntimeError,'unknown'):self.run_confirm([active(self.command)])
    def test_malformed_snapshot_stops(self):
        self.initial['goal']=np.zeros(15)
        with self.assertRaisesRegex(RuntimeError,'unknown'):self.run_confirm([active(self.command)])
    def test_repeated_pending_packets_still_stop(self):
        self.report['goal_readback_events']=[{'t':9.},{'t':9.5}]
        with self.assertRaisesRegex(RuntimeError,'Repeated'):self.run_confirm([active(self.command)])
        self.assertEqual(self.read_count,0)
    def test_other_health_failure_never_resumes(self):
        cases=[('temperature',0,46),('voltage',0,35),('current',0,551),('watchdog',0,255),('torque',0,0),('position',0,int(self.command[0])+120)]
        for key,index,value in cases:
            with self.subTest(key=key):
                self.clock=[0.];self.read_count=0;self.report={'enabled_at':0.};s=active(self.command);s[key][index]=value
                with self.assertRaises(RuntimeError):self.run_confirm([s])
        self.bus.write.assert_not_called()
    def test_sampler_routes_observed_partial_read_to_new_confirmation(self):
        states=[self.initial,active(self.command),active(self.command)]
        def reader(*args,**kwargs):
            self.clock[0]+=.026;s=states.pop(0);return s,np.zeros(16)
        with patch.object(checked,'_read_sample',side_effect=reader),patch.object(confirmation.time,'perf_counter',side_effect=lambda:self.clock[0]),contextlib.redirect_stdout(io.StringIO()):
            s,_=health.sample(self.bus,self.command,self.report,'roots')
        self.assertTrue(np.array_equal(s['goal'],self.command));self.assertEqual(states,[]);self.bus.write.assert_not_called()

class VoltageAndQueueTests(unittest.TestCase):
    def test_voltage_confirmation_body_unchanged(self):
        def function(path,name):
            tree=ast.parse(path.read_text(encoding='utf-8'));return next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
        old=function(ROOT/'loaded_repeat_voltage_20261026.py','_healthy');new=function(ROOT/'horizontal_quarter_health_20261027.py','_healthy')
        for node in ast.walk(old):
            if isinstance(node,ast.ImportFrom) and node.module=='goal_readback_confirmation':node.module='horizontal_goal_confirmation_20261027'
        self.assertEqual(ast.dump(old),ast.dump(new))
    def test_recorded_persistent_voltage_dip_still_stops(self):
        r=json.loads((ROOT/'horizontal_quarter_trial_20261027_143147_188487/result.json').read_bytes());command=np.array(r['samples'][-1]['goal']);initial=active(command);initial['voltage'][0]=36
        states=[initial]
        for row in r['samples'][-6:]:
            s=active(command);s['voltage']=np.rint(np.array(row['voltage_v'])*10).astype(int);states.append(s)
        self.clock=0.;read_count=0
        def reader(*a,**kw):
            nonlocal read_count
            self.clock+=.027;s=states[min(read_count,len(states)-1)];read_count+=1;return s,np.zeros(16)
        bus=SimpleNamespace(cancel_callback=lambda:None,write=Mock(side_effect=AssertionError('write forbidden')));report={'enabled_at':0.}
        with patch.object(checked,'_read_sample',side_effect=reader),patch.object(health.time,'perf_counter',side_effect=lambda:self.clock),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'Undervoltage not recovered'):health._healthy(bus,command,report,'roots')
        self.assertEqual(report['voltage_events'][-1]['status'],'stopped');bus.write.assert_not_called()
    def test_after_short_support_clearance_only_one_root_is_commanded(self):
        plan=plan_module.HorizontalQuarterPlan(plan_module.REQUEST);raw=plan.source_position();rows=plan.plan(raw)
        first_root=next(i for i,(p,q) in enumerate(rows) if p==plan_module.ROOTS);previous=rows[first_root-1][1] if first_root else raw.copy();start=previous.copy();order=[]
        clearance=np.maximum(start[[7,11,15]],1980);in_common=not np.array_equal(start[[7,11,15]],clearance)
        for phase,q in rows[first_root:]:
            if phase!=plan_module.ROOTS:break
            changed=np.flatnonzero(q!=previous)
            self.assertTrue(set(changed)<=set((7,11,15)));self.assertLessEqual(max(abs(q-previous)),2)
            if in_common:
                self.assertTrue(np.all(q[[7,11,15]]<=clearance))
                if np.array_equal(q[[7,11,15]],clearance):in_common=False
            else:
                self.assertLessEqual(len(changed),1)
                if len(changed) and (not order or order[-1]!=int(changed[0])):order.append(int(changed[0]))
            previous=q
        self.assertEqual(order,[7,11,15]);self.assertTrue(np.array_equal(previous[[7,11,15]],[2064,2064,2053]))

if __name__=='__main__':unittest.main(verbosity=2)
