"""Replay saved telemetry against unchanged voltage criteria; no devices."""
import json,unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch
import numpy as np
import sequence_flip_checked as checked
from loaded_repeat_voltage_20261026 import _healthy
ROOT=Path(__file__).resolve().parent
class Replay(unittest.TestCase):
    def state(self,record,voltage=None):
        return dict(torque=np.array(record['torque']),error=np.array(record['error']),watchdog=np.array(record['watchdog']),position=np.array(record['actual']),goal=np.array(record['goal_readback']),current=np.array(record['current_ma'])%65536,voltage=np.rint(10*np.array(record['voltage_v'] if voltage is None else voltage)).astype(int),temperature=np.array(record['temperature_c']),velocity=np.zeros(16,int),acceleration=np.zeros(16,int),current_cap=checked.CAP.copy(),**{k:v.copy() for k,v in checked.GAINS.items()})
    def replay(self,states):
        cursor=iter(states);clock=[0.];last=[None]
        def read(*args):
            clock[0]+=.025;last[0]=next(cursor,last[0]);return last[0],abs(last[0]['position']-last[0]['goal'])*checked.TICK
        bus=SimpleNamespace(checked_control_profile={},cancel_callback=lambda:None);report={'enabled_at':0.};command=states[0]['goal']
        with patch('loaded_repeat_voltage_20261026.checked._read_sample',side_effect=read),patch('loaded_repeat_voltage_20261026.time.perf_counter',side_effect=lambda:clock[0]),patch('loaded_repeat_voltage_20261026.time.sleep',side_effect=lambda seconds:clock.__setitem__(0,clock[0]+seconds)):
            return _healthy(bus,command,report,'saved_voltage_replay'),report
    def records(self):return json.loads((ROOT/'fresh_empty_grasp_20261027_024814_192218/result.json').read_bytes())['samples']
    def test_recorded_persistent_low_voltage_is_rejected(self):
        r=self.records();trigger=next(i for i,s in enumerate(r) if min(s['voltage_v'])==3.6)
        with self.assertRaisesRegex(RuntimeError,'Undervoltage not recovered'):self.replay([self.state(v) for v in r[trigger:]])
    def test_two_healthy_fresh_reads_resume_without_lowering_threshold(self):
        record=self.records()[-1];low=self.state(record,voltage=[3.6]+[4.0]*15);high=self.state(record,voltage=[3.8]*16);(_,errors),report=self.replay([low,high,high]);self.assertTrue(np.isfinite(errors).all());self.assertEqual(report['voltage_events'][0]['healthy_confirmation_count'],2);self.assertEqual(report['voltage_events'][0]['goals_advanced'],0)
    def test_three_point_five_still_stops_immediately(self):
        record=self.records()[-1]
        with self.assertRaisesRegex(RuntimeError,'Undervoltage'):self.replay([self.state(record,voltage=[3.5]+[4.0]*15)])
if __name__=='__main__':unittest.main()
