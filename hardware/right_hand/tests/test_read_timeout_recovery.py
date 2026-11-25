"""Read recovery exercises SDK-facing bus methods, with no physical ports."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import hand_start_pose as base
import hand_prepare_full as full
import sequence_flip_checked as sequence


class Clock:
    t=0.
    def now(self):return self.t


def fixture(replies, *, enabled=True, duration=.034, bus_class=None):
    clock=Clock(); made=[];writes=[];report={}
    queue=list(replies)
    def response():
        clock.t+=duration
        return queue.pop(0)
    def read(*args):
        row=response();return row.get('value',0),row['comm'],row.get('error',0)
    packet=SimpleNamespace(read1ByteTxRx=read,read2ByteTxRx=read,read4ByteTxRx=read)
    class Group:
        def __init__(self,port,ph,addr,length):
            self.length=length;self.data_dict={};self.cleared=False;made.append(self)
        def addParam(self,i):self.data_dict[i]=[];return True
        def txRxPacket(self):
            self.row=response()
            for i in range(1,self.row.get('received',16)+1):self.data_dict[i]=[0]*self.length
            return self.row['comm']
        def isAvailable(self,*args):return True
        def getData(self,i,*args):return self.row.get('value',0)
        def clearParam(self):self.cleared=True;self.data_dict.clear()
    class Writer:
        def __init__(self,*args):pass
        def addParam(self,*args):return True
        def txPacket(self):writes.append('write');return 0
        def clearParam(self):pass
    sdk=SimpleNamespace(COMM_SUCCESS=0,PortHandler=lambda p:object(),PacketHandler=lambda p:packet,
                        GroupSyncRead=Group,GroupSyncWrite=Writer)
    bus=(bus_class or (full.Bus if enabled else base.SyncBus))(sdk,report)
    return bus,report,clock,made,writes,queue


class Tests(unittest.TestCase):
    def test_partial_failed_sync_read_is_discarded_and_replaced_by_complete_fresh_group(self):
        bus,r,clock,made,writes,_=fixture([dict(comm=-3001,received=13,value=999),dict(comm=0,value=123)])
        with patch.object(base.time,'monotonic',clock.now):
            result=bus.block(132,4,[('position',132,4)])
        np.testing.assert_array_equal(result['position'],np.full(16,123))
        self.assertEqual(len(made),2)
        self.assertTrue(all(g.cleared for g in made))
        self.assertEqual(writes,[])
        self.assertEqual(r['read_failures'][0]['first_incomplete_id'],14)
        self.assertTrue(r['read_failures'][0]['recovered'])

    def test_single_register_timeout_gets_one_fresh_read(self):
        bus,r,clock,_,writes,_=fixture([dict(comm=-3001),dict(comm=0,value=25)])
        with patch.object(base.time,'monotonic',clock.now):self.assertEqual(bus.read(14,98),25)
        self.assertEqual(writes,[])
        self.assertTrue(r['read_failures'][0]['recovered'])

    def test_persistent_timeout_or_other_errors_do_not_loop(self):
        for replies,expected in [([dict(comm=-3001),dict(comm=-3001)],2),
                                  ([dict(comm=-3002)],1),([dict(comm=0,error=128)],1)]:
            bus,r,clock,_,writes,remaining=fixture(replies)
            with patch.object(base.time,'monotonic',clock.now),self.assertRaises(RuntimeError):bus.read(14,98)
            self.assertEqual(len(r['read_failures']),expected)
            self.assertFalse(any(x['recovered'] for x in r['read_failures']))
            self.assertEqual(writes,[])
            self.assertEqual(remaining,[])

    def test_successful_retry_after_200ms_is_not_returned(self):
        bus,r,clock,_,_,_=fixture([dict(comm=-3001),dict(comm=0,value=25)])
        original=bus.packet.read1ByteTxRx
        calls=[]
        def read(*args):
            value=original(*args);calls.append(1)
            if len(calls)==2:clock.t+=.2
            return value
        bus.packet.read1ByteTxRx=read
        with patch.object(base.time,'monotonic',clock.now),self.assertRaisesRegex(RuntimeError,'200ms'):
            bus.read(14,98)
        self.assertFalse(r['read_failures'][0]['recovered'])

    def test_slow_first_failure_and_excessive_recoveries_stop(self):
        bus,r,clock,_,_,_=fixture([dict(comm=-3001)],duration=.11)
        with patch.object(base.time,'monotonic',clock.now),self.assertRaises(RuntimeError):bus.read(14,98)
        self.assertFalse(r['read_failures'][0]['retry_requested'])
        bus,r,clock,_,_,_=fixture([dict(comm=-3001),dict(comm=0)]*3+[dict(comm=-3001)])
        with patch.object(base.time,'monotonic',clock.now):
            for _ in range(3):bus.read(14,98)
            with self.assertRaises(RuntimeError):bus.read(14,98)
        self.assertEqual(sum(x['recovered'] for x in r['read_failures']),3)
        self.assertFalse(r['read_failures'][-1]['retry_requested'])

    def test_readback_recovery_never_resends_goal(self):
        bus,r,clock,_,writes,_=fixture([dict(comm=-3001,received=13),dict(comm=0,value=2000)])
        with patch.object(base.time,'monotonic',clock.now):bus.write(116,4,np.full(16,2000))
        self.assertEqual(writes,['write'])
        self.assertEqual(len(r['write_events']),1)
        self.assertTrue(r['read_failures'][0]['recovered'])

    def test_legacy_bus_does_not_opt_in(self):
        bus,r,clock,_,_,_=fixture([dict(comm=-3001)],enabled=False)
        with patch.object(base.time,'monotonic',clock.now),self.assertRaises(RuntimeError):bus.read(14,98)
        self.assertFalse(r['read_failures'][0]['retry_requested'])

    def test_sequence_only_recovers_one_corrupt_read_from_fresh_group(self):
        replies=[dict(comm=-3002,received=15,value=999),dict(comm=0,value=123)]
        bus,r,clock,made,writes,_=fixture(replies,bus_class=sequence.Bus)
        with patch.object(base.time,'monotonic',clock.now):
            result=bus.block(64,83,[('position',132,4)])
        np.testing.assert_array_equal(result['position'],np.full(16,123))
        self.assertEqual(len(made),2)
        self.assertTrue(all(group.cleared for group in made))
        self.assertEqual(writes,[])
        self.assertEqual(r['read_failures'][0]['first_incomplete_id'],16)
        self.assertTrue(r['read_failures'][0]['recovered'])

    def test_sequence_stops_on_repeated_corrupt_read(self):
        bus,r,clock,_,writes,_=fixture([dict(comm=-3002),dict(comm=-3002)],
                                        bus_class=sequence.Bus)
        with patch.object(base.time,'monotonic',clock.now),self.assertRaises(RuntimeError):
            bus.read(16,98)
        self.assertEqual(len(r['read_failures']),2)
        self.assertEqual(writes,[])

    def test_other_full_profile_still_stops_on_corrupt_read(self):
        bus,r,clock,_,writes,_=fixture([dict(comm=-3002)])
        with patch.object(base.time,'monotonic',clock.now),self.assertRaises(RuntimeError):
            bus.read(16,98)
        self.assertFalse(r['read_failures'][0]['retry_requested'])
        self.assertEqual(writes,[])

    def test_sequence_reconfirms_a_static_preflight_anomaly_without_writes(self):
        for addr,size,values in ((6,1,(0,53,53)),(20,4,(17,0,0))):
            bus,r,clock,_,writes,_=fixture(
                [dict(comm=0,value=v) for v in values],bus_class=sequence.Bus)
            with patch.object(base.time,'monotonic',clock.now):
                self.assertEqual(bus.read(6,addr,size),values[-1])
            self.assertEqual(r['preflight_static_read_anomalies'][0]['values'],list(values))
            self.assertTrue(r['preflight_static_read_anomalies'][0]['recovered'])
            self.assertEqual(writes,[])

    def test_sequence_stops_on_persistent_static_preflight_mismatch(self):
        bus,r,clock,_,writes,_=fixture([dict(comm=0,value=0)]*3,
                                        bus_class=sequence.Bus)
        with patch.object(base.time,'monotonic',clock.now),self.assertRaisesRegex(RuntimeError,'firmware mismatch'):
            bus.read(6,6)
        self.assertEqual(r['preflight_static_read_anomalies'][0]['values'],[0,0,0])
        self.assertFalse(r['preflight_static_read_anomalies'][0]['recovered'])
        self.assertEqual(writes,[])
