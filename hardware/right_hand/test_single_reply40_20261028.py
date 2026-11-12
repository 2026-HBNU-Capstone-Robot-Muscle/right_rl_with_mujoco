"""Timed replay of logged ID5 partial-response stop; no real devices."""
import copy,json,unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import horizontal_single_reply40_20261028 as window
import horizontal_sync_receive_window_20261028 as original
from horizontal_receive_resync_20261027 import receive
from horizontal_read_recovery_20261028 import sample_deadline,confirm_recovered_read
from test_horizontal_v3_health_20261027 import active
import sequence_flip_checked as checked
FIELDS=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
class Fake:
    def __init__(self,fault=True,persistent=False):
        self.t=0.;self.n=0;self.fault=fault;self.persistent=persistent;self.window=0.;self.packet_ids=[];self.windows=[];self.command=np.full(16,2200);self.bad_confirm=False;self.first_delay=None
        self.port=SimpleNamespace(is_using=False,getBytesAvailable=lambda:0,readPort=lambda n:bytes(n),setPacketTimeoutMillis=self.set_window)
        self.bus=SimpleNamespace(sdk=SimpleNamespace(COMM_SUCCESS=0,GroupSyncRead=self.group),port=self.port,packet=object(),report={'samples':[]},_read_recovery_times=[],write=Mock())
    def now(self):return self.t
    def sleep(self,s):self.t+=s
    def set_window(self,value):self.window=value;self.windows.append(value)
    def group(self,*args):
        outer=self
        class Group:
            def __init__(self):self.data_dict={}
            def addParam(self,i):self.data_dict[i]=[];return True
            def txPacket(self):outer.sleep(.0005);return 0
            def rxPacket(self):
                ids=list(self.data_dict);outer.packet_ids.append((outer.n,ids.copy()))
                if outer.fault and 5 in ids and (outer.n==1 or outer.persistent):outer.sleep(outer.window/1000);return -3002
                delay=outer.first_delay if outer.first_delay is not None and ids[0]==1 else .0015
                if delay*1000>outer.window:outer.sleep(outer.window/1000);return -3002
                outer.sleep(delay)
                for i in ids:self.data_dict[i]=[0]*83
                return 0
            def isAvailable(self,i,a,n):return len(self.data_dict[i])==83
            def getData(self,i,a,n):
                state=active(outer.command);state['position']+=outer.n
                if outer.bad_confirm and outer.n>=3:state['voltage'][8]=35
                name=next(name for name,addr,size in FIELDS if addr==a and size==n)
                return int(state[name][i-1])
            def clearParam(self):self.data_dict.clear()
        return Group()
    def operation(self,module=window,address=64,length=83):
        self.n+=1
        if self.n==1 and self.fault:self.sleep(.022)
        return module.block_once(self.bus,address,length,FIELDS if length==83 else [('torque',64,1)])
    def read(self,module=window):
        value=receive(self.bus,lambda:self.operation(module),'full health READ',dict(kind='sync_read',address=64,length=83),now=self.now,sleep=self.sleep)
        self.bus.report['samples'].append({'goal':value['goal'].tolist()})
        return value,np.abs(value['position']-self.command)*checked.TICK
    def confirmed(self,module=window):
        with sample_deadline(self.bus,0.):
            state,errors=self.read(module)
            if self.bus.report.get('read_failures'):
                return confirm_recovered_read(self.bus,self.command,state,errors,self.bus.report,'stage_8_horizontal_hold',began=0.,read_sample=lambda:self.read(module),validate=lambda s:checked.health(s,self.command),now=self.now)
            return state,errors
class Reply40Tests(unittest.TestCase):
    def run_fake(self,fake,fn):
        with patch.object(window.time,'monotonic',side_effect=fake.now),patch.object(window.time,'perf_counter',side_effect=fake.now):return fn()
    def test_logged_stop_shape_is_reproduced_with_original80ms(self):
        from pathlib import Path
        recorded=json.loads((Path(__file__).resolve().parent/'calibrated_quarter_trial_20261028_053806_737439/result.json').read_bytes())['read_failures'][0]
        self.assertEqual(recorded['first_incomplete_id'],5);self.assertFalse(recorded['retry_requested'])
        f=Fake()
        with self.assertRaises(RuntimeError):self.run_fake(f,lambda:f.confirmed(original))
        self.assertAlmostEqual(f.t*1000,recorded['elapsed_ms'],delta=3);self.assertEqual(f.n,1);f.bus.write.assert_not_called()
    def test_same_timed_fault_recovers_with_two_clean_reads_inside200ms(self):
        f=Fake();state,_=self.run_fake(f,f.confirmed)
        self.assertEqual(f.n,4);self.assertLess(f.t,.2)
        self.assertEqual(f.bus.report['read_recovery_confirmation_events'][-1]['clean_reads'],2)
        self.assertEqual(state['position'].tolist(),(f.command+4).tolist());f.bus.write.assert_not_called()
    def test_partial_motor_values_are_discarded_before_retry(self):
        f=Fake();state,_=self.run_fake(f,f.confirmed)
        self.assertEqual(f.bus.report['read_failures'][0]['completed_ids'],[1,2,3,4])
        self.assertTrue(np.all(state['position']==f.command+4));self.assertTrue(f.bus.report['read_failures'][0]['recovered'])
    def test_persistent_corruption_still_stops(self):
        f=Fake(persistent=True)
        with self.assertRaises(RuntimeError):self.run_fake(f,f.confirmed)
        self.assertLessEqual(f.n,3);f.bus.write.assert_not_called()
    def test_unsafe_confirmation_is_rejected(self):
        f=Fake();f.bad_confirm=True
        with self.assertRaises(RuntimeError):self.run_fake(f,f.confirmed)
        f.bus.write.assert_not_called()
    def test_health_reply_can_arrive_at35ms(self):
        f=Fake(fault=False);f.first_delay=.035
        state,_=self.run_fake(f,f.confirmed)
        self.assertEqual(len(state['position']),16);self.assertLess(f.t,.2);self.assertTrue(all(v<=40 for v in f.windows))
    def test_late_health_reply_is_never_accepted_as_complete(self):
        f=Fake(fault=False);f.first_delay=.05
        values,comm,_,_=self.run_fake(f,lambda:f.operation())
        self.assertIsNone(values);self.assertNotEqual(comm,0);f.bus.write.assert_not_called()
    def test_torque_off_readback_keeps80ms_and16_motor_group(self):
        f=Fake(fault=False);f.first_delay=.055
        value,comm,_,_=self.run_fake(f,lambda:f.operation(length=1))
        self.assertEqual(comm,0);self.assertEqual(len(value['torque']),16);self.assertEqual(f.windows,[80.]);self.assertEqual(f.packet_ids[0][1],list(range(1,17)))
    def test_remaining_outer_deadline_caps_reply_window(self):
        f=Fake(fault=False);f.bus._horizontal_sample_deadline=.001
        values,comm,_,_=self.run_fake(f,lambda:f.operation())
        self.assertIsNone(values);self.assertNotEqual(comm,0);self.assertLessEqual(f.windows[-1],1.)
    def test_calibrated_bus_uses40_only_for_bound_scopes(self):
        import leap_calibrated_quarter_20261028 as live
        bus=live.CalibratedQuarterBus.__new__(live.CalibratedQuarterBus)
        with patch.object(live,'reply40_block',return_value='new') as new,patch.object(live.HorizontalQuarterBus,'_block_once',return_value='old') as old:
            self.assertEqual(bus._block_once(64,83,[]),'new');self.assertEqual(bus._block_once(64,1,[]),'new');self.assertEqual(bus._block_once(98,1,[]),'old')
            self.assertEqual(new.call_count,2);self.assertEqual(old.call_count,1)
if __name__=='__main__':unittest.main()
