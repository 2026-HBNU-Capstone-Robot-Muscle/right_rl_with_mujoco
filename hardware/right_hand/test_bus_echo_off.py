"""OFF LED diagnostic faults and saved OFF-position drift regression."""
import contextlib
import copy
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from bus_echo_off import EchoAudit,main,write_off_led_packet,read_off_snapshot


class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,dt):self.t+=dt


def make_state():
    return dict(torque=np.zeros(16,dtype=int),led=np.arange(16)%2,
        error=np.zeros(16,dtype=int),watchdog=np.zeros(16,dtype=int),
        goal=np.arange(16)+2000,position=np.arange(16)+1900,
        voltage=np.full(16,48),temperature=np.full(16,24))


class Tests(unittest.TestCase):
    def exercise(self,fault=None,seconds=.2):
        clock=Clock();state=make_state();initial=copy.deepcopy(state);report={};writes=[];calls=[0]
        saved=[]
        if fault=='saved_goal_drift':
            source=Path(__file__).resolve().parent/'bus_echo_off_20261023_225847_477065/result.json'
            saved=json.loads(source.read_text(encoding='utf-8'))['reads']
        camera=SimpleNamespace(sequence=20,check_preview=lambda _:None,
            read_packet=lambda:None if fault=='camera_stale' else
                (clock.now()-3 if fault=='camera_old' else clock.now(),b'fake_frame'),
            release=lambda:report.update(camera_released=True))
        def read():
            calls[0]+=1;clock.sleep(.01)
            if saved:
                row=saved[(calls[0]-1)%len(saved)]
                for key in ('goal','position','voltage'):state[key]=np.array(row[key])
            if fault=='initial_ON':state['torque'][0]=1
            if fault=='later_ON' and len(writes)==1:state['torque'][0]=1
            if fault=='read_loss' and writes:raise RuntimeError('injected read loss')
            if fault=='slow':clock.sleep(.201)
            if fault=='malformed':state['led']=np.zeros(15)
            if fault=='error':state['error'][0]=1
            if fault=='watchdog':state['watchdog'][0]=255
            if fault=='low_voltage':state['voltage'][0]=37
            if fault=='hot':state['temperature'][0]=46
            if fault=='lost_restore' and len(writes)>1:state['led']=1-initial['led']
            return copy.deepcopy(state)
        def write(led):
            self.assertFalse(np.any(state['torque']))
            self.assertTrue(np.isin(led,(0,1)).all())
            writes.append(led.copy())
            if fault=='tx_error':raise RuntimeError('injected TX error')
            if fault=='partial_tx' and len(writes)==1:
                state['led'][:8]=led[:8]
                raise RuntimeError('injected partial TX')
            if fault!='lost_led':state['led']=led.copy()
        def cancel():
            if fault=='q' and writes:raise KeyboardInterrupt('Q')
        def camera_factory():
            if fault=='camera':raise RuntimeError('injected camera failure')
            return camera
        audit=EchoAudit(read,write,report,cancel=cancel,now=clock.now,sleep=clock.sleep)
        error=None
        with contextlib.redirect_stdout(io.StringIO()):
            try:audit.run(seconds=seconds,camera_factory=camera_factory)
            except BaseException as exc:error=exc
        return audit,report,writes,state,initial,error

    def test_normal_two_phases_restore_LEDs_without_writing_positions(self):
        audit,r,w,s,initial,error=self.exercise();self.assertIsNone(error)
        self.assertEqual(r['status'],'OFF_LED_echo_complete')
        self.assertEqual([x['phase'] for x in r['phases']],['camera_closed','camera_streaming'])
        self.assertTrue(all(x['status']=='complete' for x in r['phases']))
        for key in ('led','goal','position'):np.testing.assert_array_equal(s[key],initial[key])
        self.assertTrue(r['led_restoration_verified'] and r['all_torque_off_verified'])
        self.assertTrue(r['camera_released'])
        self.assertEqual((r['torque_writes'],r['goal_writes'],r['drive_setting_writes'],r['eeprom_writes']),(0,0,0,0))
        self.assertTrue(all(x['address']==65 and x['size']==1 for x in r['led_writes']))

    def test_actual_saved_OFF_goal_drift_does_not_block_LED_test(self):
        audit,r,w,s,initial,error=self.exercise('saved_goal_drift');self.assertIsNone(error)
        self.assertGreater(len(w),2);self.assertEqual(r['status'],'OFF_LED_echo_complete')
        self.assertTrue(all(x['goal']==x['position'] for x in r['reads']))
        self.assertGreater(len({tuple(x['goal']) for x in r['reads']}),1)
        self.assertEqual(r['goal_writes'],0);self.assertTrue(r['led_restoration_verified'])

    def test_initial_ON_or_unhealthy_read_cannot_send_a_probe(self):
        for fault in ('initial_ON','slow','malformed','error','watchdog','low_voltage','hot'):
            with self.subTest(fault=fault):
                audit,r,w,s,initial,error=self.exercise(fault)
                self.assertIsNotNone(error);self.assertEqual(w,[])
                self.assertFalse(r['all_torque_off_verified'])

    def test_unexpected_ON_after_first_probe_cannot_restore(self):
        audit,r,w,s,initial,error=self.exercise('later_ON')
        self.assertIsNotNone(error);self.assertEqual(len(w),1)
        self.assertFalse(r['led_restoration_verified']);self.assertFalse(r['all_torque_off_verified'])

    def test_lost_LED_is_reported_with_no_probe_resend(self):
        audit,r,w,s,initial,error=self.exercise('lost_led')
        self.assertIn('echo missing',str(error));self.assertEqual(len(w),1)
        self.assertEqual(r['echo_mismatches'][0]['probe_resends'],0)
        self.assertTrue(r['led_restoration_verified'])
        np.testing.assert_array_equal(s['led'],initial['led'])
        self.assertEqual(r['phases'][0]['status'],'stopped')

    def test_q_and_camera_error_restore_original_LEDs_while_OFF(self):
        for fault in ('q','camera'):
            audit,r,w,s,initial,error=self.exercise(fault)
            self.assertIsNotNone(error);self.assertTrue(r['led_restoration_verified'])
            np.testing.assert_array_equal(s['led'],initial['led'])
            self.assertFalse(np.any(s['torque']))

    def test_persistent_read_loss_cannot_blindly_restore(self):
        audit,r,w,s,initial,error=self.exercise('read_loss')
        self.assertIsNotNone(error);self.assertEqual(len(w),1)
        self.assertFalse(r['led_restoration_verified']);self.assertFalse(r['all_torque_off_verified'])

    def test_stale_or_invalid_LED_probe_is_rejected_without_write(self):
        clock=Clock();state=make_state();writes=[];r={}
        audit=EchoAudit(lambda:copy.deepcopy(state),lambda q:writes.append(q),r,now=clock.now,sleep=clock.sleep)
        audit.original=state['led'].copy();audit.fresh_off();clock.sleep(.051)
        with self.assertRaisesRegex(RuntimeError,'expired'):audit.send(audit.original)
        audit.fresh_off()
        for values in (np.full(16,2),np.full(16,-1),np.full(16,float('nan')),np.zeros(15)):
            with self.assertRaises(RuntimeError):audit.send(values)
        self.assertEqual(writes,[])

    def test_default_entry_constructs_no_camera_or_bus(self):
        with patch('camera_stream.CameraStream',side_effect=AssertionError('camera opened')), \
             patch('sequence_flip_checked.Bus',side_effect=AssertionError('bus opened')), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main([]),0)

    def test_camera_condition_requires_an_actual_fresh_stream(self):
        for fault in ('camera_stale','camera_old'):
            audit,r,w,s,initial,error=self.exercise(fault,seconds=2.2)
            self.assertIsNotNone(error);self.assertIn('camera',str(error).lower())
            self.assertTrue(r['led_restoration_verified']);self.assertTrue(r['camera_released'])
            np.testing.assert_array_equal(s['led'],initial['led'])

    def test_failed_restoration_prevents_success_status(self):
        audit,r,w,s,initial,error=self.exercise('lost_restore',seconds=.02)
        self.assertIsNotNone(error);self.assertFalse(r['led_restoration_verified'])
        self.assertEqual(r['status'],'stopped');self.assertEqual(len(w),2)

    def test_TX_error_is_recorded_before_return_without_probe_retry(self):
        audit,r,w,s,initial,error=self.exercise('tx_error')
        self.assertIsNotNone(error);self.assertEqual(len(w),1)
        self.assertFalse(r['led_writes'][0]['tx_returned_success'])
        self.assertTrue(r['led_restoration_verified'])

    def test_partial_TX_failure_restores_LED_without_resending_probe(self):
        audit,r,w,s,initial,error=self.exercise('partial_tx')
        self.assertIsNotNone(error);self.assertEqual(len(w),2)
        self.assertEqual([x['purpose'] for x in r['led_writes']],['probe','restore'])
        self.assertFalse(r['led_writes'][0]['tx_returned_success'])
        self.assertTrue(r['led_restoration_verified'])
        np.testing.assert_array_equal(s['led'],initial['led'])

    def test_sdk_packet_targets_only_LED_RAM_and_clears_on_TX_failure(self):
        events=[];params={}
        class Group:
            def addParam(self,ident,data):params[ident]=data;return True
            def txPacket(self):events.append('tx');return -1001
            def clearParam(self):events.append('clear')
        def group(port,packet,address,length):
            events.append((address,length));return Group()
        sdk=SimpleNamespace(GroupSyncWrite=group,COMM_SUCCESS=0)
        with self.assertRaisesRegex(RuntimeError,'TX failed'):
            write_off_led_packet(sdk,object(),object(),np.arange(16)%2)
        self.assertEqual(events,[(65,1),'tx','clear'])
        self.assertEqual(set(params),set(range(1,17)))
        for ident,value in params.items():self.assertEqual(value,[(ident-1)%2])

    def test_snapshot_distinguishes_LED_goal_and_present_position(self):
        import dynamixel_sdk as sdk
        import sequence_flip_checked as checked
        bus=checked.Bus.__new__(checked.Bus)
        bus.sdk=sdk;bus.port=object();bus.packet=SimpleNamespace(getProtocolVersion=lambda:2.)
        bus._receive=lambda operation,*args:operation()[0]
        group=sdk.GroupSyncRead(bus.port,bus.packet,64,83)
        for ident in range(1,17):
            group.addParam(ident);data=[0]*83;data[65-64]=ident%2
            for addr,value in ((116,2000+ident),(132,3000+ident),(144,48)):
                size=2 if addr==144 else 4
                data[addr-64:addr-64+size]=int(value).to_bytes(size,'little')
            data[146-64]=24;group.data_dict[ident]=data
        group.last_result=True;group.txRxPacket=lambda:0;group.addParam=lambda ident:True
        with patch.object(sdk,'GroupSyncRead',return_value=group):state=read_off_snapshot(bus)
        np.testing.assert_array_equal(state['led'],np.arange(1,17)%2)
        np.testing.assert_array_equal(state['goal'],np.arange(1,17)+2000)
        np.testing.assert_array_equal(state['position'],np.arange(1,17)+3000)


if __name__=='__main__':unittest.main()
