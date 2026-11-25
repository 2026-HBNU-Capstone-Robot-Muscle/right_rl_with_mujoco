import unittest
import dynamixel_sdk as sdk
from fast_health_read_20261029 import decode_status,DeviceStatusError
class Checks(unittest.TestCase):
    def setUp(self):self.p=sdk.PacketHandler(2.)
    def packet(self,ids=(1,2),length=4,error_index=None,payload=None):
        total=8+len(ids)*(length+4);raw=bytearray(b'\xff\xff\xfd\x00\xfe')+bytearray((total-7).to_bytes(2,'little'))+bytearray([0x55])
        for n,i in enumerate(ids):
            raw.extend([1 if n==error_index else 0,i]);raw.extend(payload if payload is not None else bytes([i])*length)
            crc=self.p.updateCRC(0,list(raw),len(raw));raw.extend(crc.to_bytes(2,'little'))
        return raw
    def recalc(self,raw):raw[-2:]=self.p.updateCRC(0,list(raw[:-2]),len(raw)-2).to_bytes(2,'little');return raw
    def test_official_three_device_golden_example(self):
        raw=bytes.fromhex('ff ff fd 00 fe 19 00 55 00 03 a6 00 00 00 84 08 00 07 1f 08 00 00 16 ca 00 04 ff 03 00 00 d1 9e')
        v=decode_status(self.p,raw,(3,7,4),4);self.assertEqual([int.from_bytes(v[i],'little') for i in (3,7,4)],[166,2079,1023])
    def test_eight_health_frames_fit_existing_sdk_limit(self):
        raw=self.packet(tuple(range(1,9)),83);self.assertEqual(len(raw),704)
        self.assertEqual(len(decode_status(self.p,raw,range(1,9),83)),8)
    def test_corrupted_payload_crc_rejected(self):
        raw=self.packet();raw[12]^=1
        with self.assertRaises(ValueError):decode_status(self.p,raw,(1,2),4)
    def test_crc_valid_duplicate_id_rejected(self):
        raw=self.packet((1,1))
        with self.assertRaises(ValueError):decode_status(self.p,raw,(1,2),4)
    def test_reordered_ids_rejected(self):
        with self.assertRaises(ValueError):decode_status(self.p,self.packet((2,1)),(1,2),4)
    def test_each_device_status_error_is_checked(self):
        for index in (0,1):
            with self.subTest(index=index),self.assertRaises(DeviceStatusError):decode_status(self.p,self.packet(error_index=index),(1,2),4)
    def test_incomplete_data_never_accepted(self):
        raw=self.packet()
        for n in (1,2,8,10):
            with self.subTest(n=n),self.assertRaises(ValueError):decode_status(self.p,raw[:-n],(1,2),4)
    def test_wrong_header_with_valid_crc_rejected(self):
        raw=self.packet();raw[4]=1;self.recalc(raw)
        with self.assertRaises(ValueError):decode_status(self.p,raw,(1,2),4)
    def test_no_byte_unstuffing_of_fast_payload(self):
        raw=self.packet(payload=b'\xff\xff\xfd\xfd');values=decode_status(self.p,raw,(1,2),4)
        self.assertEqual(values[1],b'\xff\xff\xfd\xfd')
    def test_oversized_group_rejected(self):
        with self.assertRaises(ValueError):decode_status(self.p,self.packet(tuple(range(1,17)),83),range(1,17),83)
if __name__=='__main__':unittest.main(verbosity=2)
