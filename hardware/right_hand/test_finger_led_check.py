import unittest
from types import SimpleNamespace
from leap_finger_led_check import LedBus


class FakePacket:
    def __init__(self):self.writes=[];self.values={(12,65):0,(16,65):1};self.fail_write=False
    def ping(self,p,i):return 1240,0,0
    def read1ByteTxRx(self,p,i,a):return self.values.get((i,a),255 if a==12 else 0),0,0
    def write1ByteTxRx(self,p,i,a,v):
        assert a==65 and i in (12,16) and v in (0,1)
        self.writes.append((i,a,v));self.values[i,a]=v
        if self.fail_write:self.fail_write=False;return -3001,0
        return 0,0


class LedTests(unittest.TestCase):
    def setup_bus(self):
        packet=FakePacket();closed=[]
        port=SimpleNamespace(openPort=lambda:True,setBaudRate=lambda b:True,closePort=lambda:closed.append(True))
        sdk=SimpleNamespace(PortHandler=lambda _:port,PacketHandler=lambda _:packet,COMM_SUCCESS=0)
        report={'led_write_attempts':0};return LedBus(sdk,'FAKE',report),packet,closed,report

    def test_only_led_writes_and_restore(self):
        b,p,c,r=self.setup_bus();b.open();b.select(12);b.select(16);b.close()
        self.assertEqual(p.values[12,65],0);self.assertEqual(p.values[16,65],1)
        self.assertTrue(r['led_restored']);self.assertEqual(c,[True])
        self.assertTrue(all(a==65 for i,a,v in p.writes))

    def test_torque_on_prevents_every_write(self):
        b,p,c,r=self.setup_bus();p.values[9,64]=1
        with self.assertRaisesRegex(RuntimeError,'torque ON'):b.open()
        b.close();self.assertEqual(p.writes,[])

    def test_secondary_alias_prevents_every_write(self):
        b,p,c,r=self.setup_bus();p.values[7,12]=12
        with self.assertRaisesRegex(RuntimeError,'aliases'):b.open()
        b.close();self.assertEqual(p.writes,[])

    def test_uncertain_write_restores_led_without_goal_retry(self):
        b,p,c,r=self.setup_bus();b.open();p.fail_write=True
        with self.assertRaisesRegex(RuntimeError,'write failed'):b.select(12)
        b.close();self.assertEqual(p.values[12,65],0);self.assertTrue(r['led_restored'])
        self.assertEqual(len(p.writes),2)

    def test_torque_change_before_select_and_invalid_requests(self):
        b,p,c,r=self.setup_bus();b.open();p.values[16,64]=1
        with self.assertRaisesRegex(RuntimeError,'torque ON'):b.select(12)
        for motor,value in [(1,1),(12,2),(12,True)]:
            with self.assertRaises(ValueError):b.led(motor,value)
        b.close();self.assertEqual(p.writes,[])

if __name__=='__main__':unittest.main()
