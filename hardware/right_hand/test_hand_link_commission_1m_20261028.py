"""Device-free checks for the user-started baud-only operation."""
import contextlib,copy,io,unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import hand_link_commission_1m_20261028 as setup

def state(baud):
    s={str(a):0 for a in setup.PRESERVED}
    s.update({'11':5,'38':2352,'80':1900,'82':0,'84':2100,'102':450,'116':2100,
        'baud_value':setup.RATES[baud],'voltage':50,'temperature':24})
    return s

class FakeLink:
    def __init__(self,mixed=False):
        self.rates={m:1000000 if mixed and m<=8 else 3000000 for m in range(1,17)}
        self.states={m:state(r) for m,r in self.rates.items()};self.current=None;self.located=[];self.writes=[]
        self.bad_before=None;self.failed_write=None;self.bad_after=None;self.final_mutation=None;self.state_calls={}
    def rate(self,value):self.current=value
    def locate(self,motor):
        self.located.append(motor)
        if motor==self.bad_before:raise RuntimeError('not healthy OFF')
        return dict(rate=self.rates[motor],state=copy.deepcopy(self.states[motor]))
    def write_baud_once(self,motor,address,value):
        if len(self.located)!=16:raise AssertionError('all16 checks must precede writes')
        if self.current!=3000000 or (address,value)!=(8,3):raise AssertionError('wrong write')
        self.writes.append((motor,address,value))
        if motor==self.failed_write:raise RuntimeError('transmission failure')
        self.rates[motor]=1000000;self.states[motor]['baud_value']=3
    def state(self,motor):
        if self.current!=self.rates[motor]:raise RuntimeError('wrong host speed')
        self.state_calls[motor]=self.state_calls.get(motor,0)+1
        s=copy.deepcopy(self.states[motor])
        if motor==self.bad_after:s['84']+=1
        if motor==self.final_mutation and self.state_calls[motor]>=3:s['116']+=1
        return s

class CommissionTests(unittest.TestCase):
    def run_change(self,link):
        report={}
        with contextlib.redirect_stdout(io.StringIO()):setup.migrate(link,report,sleep=lambda _:None)
        return report
    def test_default_opens_no_port_or_camera(self):
        with patch.object(setup,'Link',side_effect=AssertionError('port opened')),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(setup.main([]),0)
    def test_all16_migrate_once_and_preserve_other_registers(self):
        link=FakeLink();before=copy.deepcopy(link.states);r=self.run_change(link)
        self.assertEqual(r['confirmed_ids'],list(range(1,17)));self.assertEqual(len(link.writes),16)
        self.assertTrue(all(rate==1000000 for rate in link.rates.values()))
        for motor in range(1,17):
            self.assertEqual([link.states[motor][str(a)] for a in setup.PRESERVED],[before[motor][str(a)] for a in setup.PRESERVED])
    def test_mixed_partial_attempt_resumes_only_unconverted_devices(self):
        link=FakeLink(mixed=True);self.run_change(link)
        self.assertEqual([m for m,a,v in link.writes],list(range(9,17)))
    def test_already_converted_hand_does_no_eeprom_write(self):
        link=FakeLink();link.rates={m:1000000 for m in range(1,17)};link.states={m:state(1000000) for m in range(1,17)}
        self.run_change(link);self.assertEqual(link.writes,[])
    def test_unhealthy_last_motor_stops_before_any_write(self):
        link=FakeLink();link.bad_before=16
        with self.assertRaises(RuntimeError):self.run_change(link)
        self.assertEqual(link.writes,[])
    def test_failed_baud_transmission_is_not_retried_or_rolled_back(self):
        link=FakeLink();link.failed_write=4
        with self.assertRaises(RuntimeError):self.run_change(link)
        self.assertEqual(link.writes,[(m,8,3) for m in range(1,5)])
    def test_unexpected_other_setting_stops_the_sequence(self):
        link=FakeLink();link.bad_after=4
        with self.assertRaisesRegex(RuntimeError,'register'):self.run_change(link)
        self.assertEqual(len(link.writes),4)
    def test_final_comparison_detects_late_setting_change(self):
        link=FakeLink();link.final_mutation=1
        with self.assertRaisesRegex(RuntimeError,'final preserved'):self.run_change(link)
    def test_low_level_writer_rejects_every_other_register_and_wrong_baud(self):
        link=setup.Link.__new__(setup.Link);link.report={};link.port=object()
        link.packet=SimpleNamespace(write1ByteTxOnly=Mock(return_value=0));link.state=Mock(return_value=state(3000000))
        for motor,address,value in [(0,8,3),(17,8,3),(1,64,1),(1,116,2000),(1,84,600),(1,11,5),(1,8,5)]:
            with self.subTest(address=address):
                with self.assertRaises(RuntimeError):link.write_baud_once(motor,address,value)
        link.packet.write1ByteTxOnly.assert_not_called()
        link.write_baud_once(1,8,3);link.packet.write1ByteTxOnly.assert_called_once_with(link.port,1,8,3)
    def test_writer_rejects_already_converted_device(self):
        link=setup.Link.__new__(setup.Link);link.report={};link.port=object()
        link.packet=SimpleNamespace(write1ByteTxOnly=Mock());link.state=Mock(return_value=state(1000000))
        with self.assertRaises(RuntimeError):link.write_baud_once(1,8,3)
        link.packet.write1ByteTxOnly.assert_not_called()
    def test_locator_rejects_duplicate_responses(self):
        link=setup.Link.__new__(setup.Link);link.rate=Mock()
        link.packet=SimpleNamespace(ping=Mock(return_value=(1240,0,0)));link.port=object()
        link.state=Mock(side_effect=[state(3000000),state(1000000)])
        with self.assertRaisesRegex(RuntimeError,'exactly one'):link.locate(1)
    def test_state_rejects_torque_mode_error_voltage_before_writes(self):
        for address,value in [(64,1),(70,1),(98,25),(11,3),(20,10),(144,35),(146,46)]:
            with self.subTest(address=address):
                link=setup.Link.__new__(setup.Link)
                values={a:v for a,v in ((a,state(3000000)[str(a)]) for a in setup.PRESERVED)}
                values.update({8:5,144:50,146:24});values[address]=value
                link.read=lambda motor,addr,size=1:values[addr]
                with self.assertRaises(RuntimeError):link.state(1)

if __name__=='__main__':unittest.main()
