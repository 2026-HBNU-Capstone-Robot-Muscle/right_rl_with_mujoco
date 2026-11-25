"""Inspect installed SDK and official CRC examples without opening devices."""
import inspect
from pathlib import Path
import dynamixel_sdk as sdk
p=sdk.PacketHandler(2.)
print('PACKET FAST METHODS',[n for n in dir(p) if 'fast' in n.lower()])
source=Path(inspect.getfile(type(p)));lines=source.read_text(encoding='utf-8').splitlines()
for i,line in enumerate(lines):
    if 'fast' in line.lower() or 'PACKET_MAX' in line:
        print(i+1,line)
        if line.strip().startswith('def '):print('\n'.join(f'{j+1}: {lines[j]}' for j in range(i,min(i+50,len(lines)))))
raw=bytes.fromhex('ff ff fd 00 fe 19 00 55 00 03 a6 00 00 00 84 08 00 07 1f 08 00 00 16 ca 00 04 ff 03 00 00 d1 9e')
print('OFFICIAL_STATUS_CRC',hex(p.updateCRC(0,list(raw[:-2]),len(raw)-2)),hex(int.from_bytes(raw[-2:],'little')))
print('SDK_SOURCE',source)
