"""Identify ID12/16 visually using LED(65) only. No motion-register writes.

XC330-M288 manufacturer reference:
https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#led65
"""
import argparse
import datetime as dt
import json
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parent
TARGETS=(12,16)


class LedBus:
    def __init__(self,sdk,device,report):
        self.sdk=sdk;self.port=sdk.PortHandler(device);self.packet=sdk.PacketHandler(2.0)
        self.report=report;self.original={};self.changed=set()

    def read(self,motor,address):
        value,comm,error=self.packet.read1ByteTxRx(self.port,motor,address)
        if comm!=self.sdk.COMM_SUCCESS or error:raise RuntimeError(f'ID{motor} read{address}: comm={comm}, error={error}')
        return int(value)

    def verify_off(self):
        for motor in range(1,17):
            if self.read(motor,64)!=0:raise RuntimeError(f'ID{motor} torque ON; no torque change will be sent')
            if self.read(motor,70)!=0:raise RuntimeError(f'ID{motor} hardware error; stop identification')

    def open(self):
        if not self.port.openPort():raise RuntimeError('Cannot open serial port')
        if not self.port.setBaudRate(3000000):raise RuntimeError('Cannot set host baud')
        for motor in range(1,17):
            model,comm,error=self.packet.ping(self.port,motor)
            if comm!=self.sdk.COMM_SUCCESS or error or model!=1240:
                raise RuntimeError(f'ID{motor} unexpected model or communication failure')
            secondary=self.read(motor,12)
            if secondary in TARGETS:raise RuntimeError(f'ID{motor} secondary ID aliases a test ID; no writes')
        self.verify_off()
        for motor in TARGETS:
            value=self.read(motor,65)
            if value not in (0,1):raise RuntimeError(f'ID{motor} unexpected LED value')
            self.original[motor]=value
        self.report['original_led']=dict(self.original)

    def led(self,motor,value):
        if motor not in TARGETS or type(value) is not int or value not in (0,1):
            raise ValueError('Only ID12/16 LED values0/1 allowed')
        self.changed.add(motor)  # A timed-out write may still have reached the LED.
        self.report['led_write_attempts']+=1
        comm,error=self.packet.write1ByteTxRx(self.port,motor,65,value)
        if comm!=self.sdk.COMM_SUCCESS or error:raise RuntimeError(f'ID{motor} LED write failed: {comm}/{error}')
        if self.read(motor,65)!=value:raise RuntimeError(f'ID{motor} LED readback mismatch')

    def select(self,motor):
        if motor not in TARGETS:raise ValueError('Unknown test ID')
        self.verify_off()
        for other in TARGETS:
            self.led(other,int(other==motor))

    def close(self):
        errors=[]
        try:
            for motor in sorted(self.changed):
                try:self.led(motor,self.original[motor])
                except Exception as exc:errors.append(str(exc))
        finally:
            try:self.port.closePort()
            except Exception as exc:errors.append('Host port close: '+str(exc))
            self.report['led_restore_errors']=errors
            self.report['led_restored']=not errors


def main(run=False,port='COM7'):
    if not run:
        print('CHECK ONLY: IDs12/16 LED(65) identification. No devices opened.');return 0
    import msvcrt
    import dynamixel_sdk as sdk
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='started',
                led_write_attempts=0,goal_writes=0,torque_writes=0,responses=[],port=port)
    bus=LedBus(sdk,port,report)
    print('LED IDENTIFICATION ONLY: ID12 then ID16. No motor movement or torque writes.',flush=True)
    print('USB + motor power connected; torque already OFF. Close other motor programs.',flush=True)
    print('Leave fingers as they are. Identify the finger from its lit end-motor LED.',flush=True)
    try:
        bus.open()
        for motor in TARGETS:
            while msvcrt.kbhit():msvcrt.getwch()
            bus.select(motor)
            print(f'ID{motor} LED ON. Does this finger touch the thumb end-motor housing?',flush=True)
            print('Y = yes; N = no; U = hidden/unsure; Q = exit. Answer in this terminal.',flush=True)
            last_check=time.monotonic()
            while True:
                if time.monotonic()-last_check>=1:
                    bus.verify_off();last_check=time.monotonic()
                if msvcrt.kbhit():
                    key=msvcrt.getwch().lower()
                    if key in ('q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                    if key in ('y','n','u'):
                        report['responses'].append(dict(motor_id=motor,touches_thumb={'y':True,'n':False,'u':None}[key]))
                        break
                time.sleep(.02)
        bus.verify_off();report['status']='identification_recorded'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted');print('STOP:',report['error'],flush=True)
    finally:
        bus.close()
        if not report['led_restored']:report['status']='led_restore_unverified'
        text=json.dumps(report,ensure_ascii=False,indent=2)+'\n'
        archive=ROOT/('finger_led_check_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
        archive.write_text(text,encoding='utf-8');(ROOT/'leap_finger_led_check_result.json').write_text(text,encoding='utf-8')
        print('Responses:',report['responses'],flush=True)
        print('LED restored:',report['led_restored'],'| No goal or torque writes.',flush=True)
        print('Saved:',archive,flush=True)
    return 0 if report['status']=='identification_recorded' else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',action='store_true');parser.add_argument('--port',default='COM7')
    args=parser.parse_args();raise SystemExit(main(args.run,args.port))
