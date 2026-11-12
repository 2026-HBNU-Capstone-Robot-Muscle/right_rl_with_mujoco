"""Enumerate Windows serial devices without opening a port."""
import datetime as dt
import json
from pathlib import Path
from serial.tools.list_ports import comports

ROOT=Path(__file__).resolve().parent
def main():
    rows=[]
    for port in comports():
        rows.append({name:getattr(port,name,None) for name in (
            'device','description','hwid','vid','pid','serial_number',
            'manufacturer','product','interface','location')})
    result=dict(timestamp=dt.datetime.now().astimezone().isoformat(),
                status='serial_device_enumeration_only',ports=rows,
                ports_opened=0,motor_instructions_sent=0)
    folder=ROOT/('usb_presence_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir()
    (folder/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=folder.name,**result),ensure_ascii=False))

if __name__=='__main__':main()
