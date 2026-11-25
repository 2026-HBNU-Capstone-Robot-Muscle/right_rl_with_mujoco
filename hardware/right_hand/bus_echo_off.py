"""OFF LED packet diagnostic v2. Only LED RAM 65/1 is written.

Goal Position is observed, never written or expected to stay fixed while OFF.
No torque, drive or EEPROM writes. This does not certify loaded goal delivery.
"""
import argparse
import datetime as dt
import json
from pathlib import Path
import time
import numpy as np

ROOT=Path(__file__).resolve().parent
PHASE_SECONDS=15.


def write_off_led_packet(sdk,port,packet,led):
    values=np.asarray(led)
    if values.shape!=(16,) or not np.isin(values,(0,1)).all():
        raise ValueError('LED packet must contain sixteen 0/1 values')
    group=sdk.GroupSyncWrite(port,packet,65,1)
    try:
        for ident,value in enumerate(values,1):
            if not group.addParam(ident,[int(value)]):
                raise RuntimeError('Cannot build OFF LED packet')
        comm=group.txPacket()
        if comm!=sdk.COMM_SUCCESS:raise RuntimeError(f'OFF LED packet TX failed: {comm}')
    finally:group.clearParam()


def read_off_snapshot(bus):
    # Same 83-byte response traffic as motion, with LED also decoded.
    return bus.block(64,83,[('torque',64,1),('led',65,1),('error',70,1),
        ('watchdog',98,1),('goal',116,4),('position',132,4),
        ('voltage',144,2),('temperature',146,1)])


class EchoAudit:
    def __init__(self,read,write,report,*,cancel=lambda:None,now=time.monotonic,sleep=time.sleep):
        self.read=read;self.write=write;self.report=report;self.cancel=cancel
        self.now=now;self.sleep=sleep;self.original=None;self.last_read=None
        self.restore_attempted=False

    def fresh_off(self,*,poll=True):
        if poll:self.cancel()
        self.report['all_torque_off_verified']=False
        began=self.now();state=self.read();ended=self.now()
        if ended-began>.2:raise RuntimeError('OFF read exceeded 200ms; no LED write')
        for key in ('torque','led','error','watchdog','goal','position','voltage','temperature'):
            a=np.asarray(state[key])
            if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
                raise RuntimeError('Malformed OFF state; no LED write')
        if any(np.any(state[key]) for key in ('torque','error','watchdog')):
            raise RuntimeError('Torque/error/watchdog is not OFF/clear; no LED write')
        if not np.isin(state['led'],(0,1)).all():
            raise RuntimeError('Invalid LED register; no LED write')
        if (np.any(np.array(state['voltage'])<38) or np.any(np.array(state['voltage'])>65)
                or np.any(np.array(state['temperature'])>45)):
            raise RuntimeError('OFF voltage/temperature outside checked range; no LED write')
        self.last_read=ended;self.report['all_torque_off_verified']=True
        self.report.setdefault('reads',[]).append(dict(phase=self.report.get('phase'),
            t=ended,read_ms=(ended-began)*1000,
            **{k:np.asarray(state[k]).tolist() for k in
               ('torque','led','error','watchdog','goal','position','voltage','temperature')}))
        return state

    def send(self,led,*,restoring=False):
        a=np.asarray(led)
        if self.original is None or a.shape!=(16,) or not np.isin(a,(0,1)).all():
            raise RuntimeError('Invalid OFF LED target; no write')
        if self.last_read is None or self.now()-self.last_read>.05:
            raise RuntimeError('Fresh OFF verification expired; no LED write')
        if restoring and self.restore_attempted:
            raise RuntimeError('LED restoration already attempted; no resend')
        self.restore_attempted=bool(restoring)
        # Record the attempt first: even a TX error might mean partial delivery.
        row=dict(address=65,size=1,phase=self.report.get('phase'),t=self.now(),led=a.tolist(),
            purpose='restore' if restoring else 'probe',tx_returned_success=False)
        self.report.setdefault('led_writes',[]).append(row)
        self.report['led_restoration_verified']=False
        self.write(a.astype(np.int64));row['tx_returned_success']=True

    def confirm(self,expected,*,poll=True):
        state=self.fresh_off(poll=poll)
        if np.array_equal(state['led'],expected):return state
        event=dict(phase=self.report.get('phase'),expected=np.asarray(expected).tolist(),
            observed=[np.asarray(state['led']).tolist()],probe_resends=0)
        self.report.setdefault('echo_mismatches',[]).append(event)
        began=self.now();good=0
        for _ in range(3):
            if self.now()-began>=.15:break
            state=self.fresh_off(poll=poll)
            event['observed'].append(np.asarray(state['led']).tolist())
            good=good+1 if np.array_equal(state['led'],expected) else 0
            if self.now()-began>=.15:break
            if good>=2:
                event['recovered_by_reads_only']=True;return state
        event['recovered_by_reads_only']=False
        raise RuntimeError('OFF LED echo missing after fresh reads; no further probe')

    def restore(self):
        if self.original is None:return
        self.report['phase']='restore_original_LEDs_OFF'
        state=self.fresh_off(poll=False)
        if not np.array_equal(state['led'],self.original):
            self.send(self.original,restoring=True)
            self.confirm(self.original,poll=False)
        self.report['led_restoration_verified']=True

    def phase(self,name,seconds,camera=None):
        self.report['phase']=name;began=self.now();last=self.original.copy();count=0
        last_camera=None;first_camera=None;camera_packets=0;last_notice=began
        row=dict(phase=name,status='running',probes=0)
        self.report.setdefault('phases',[]).append(row)
        print(name,f': starting {seconds:g}s OFF LED packet check',flush=True)
        try:
            while self.now()-began<seconds:
                cycle=self.now();self.confirm(last)
                if camera is not None:
                    camera.check_preview('TORQUE OFF LED packet test; Q stops')
                    packet=camera.read_packet()
                    if packet is not None:
                        stamp=packet[0]
                        if not 0<=self.now()-stamp<=2.:
                            raise RuntimeError('Stale camera data; OFF comparison incomplete')
                        first_camera=stamp if first_camera is None else first_camera
                        last_camera=stamp;camera_packets+=1
                    if self.now()-began>2. and (last_camera is None or self.now()-last_camera>2.):
                        raise RuntimeError('Camera not streaming; OFF comparison incomplete')
                self.cancel()
                target=1-self.original if count%2==0 else self.original
                self.send(target);last=target.copy();count+=1
                self.sleep(max(0.,.05-(self.now()-cycle)))
                if self.now()-last_notice>=5.:
                    print(name,':',count,'LED packets sent; torque remains OFF',flush=True)
                    last_notice=self.now()
            self.confirm(last)
            if camera is not None and not camera_packets:
                raise RuntimeError('Camera not streaming; OFF comparison incomplete')
            self.restore();row['status']='complete'
            print(name,':',count,'OFF LED packets verified',flush=True)
        except BaseException as exc:
            row.update(status='stopped',error=str(exc) or 'Interrupted');raise
        finally:
            row.update(probes=count,elapsed_s=self.now()-began,
                camera_frames=None if camera is None else camera.sequence,
                camera_packets_consumed=camera_packets,
                camera_timestamp_span_s=None if first_camera is None else last_camera-first_camera)

    def run(self,*,seconds=PHASE_SECONDS,camera_factory=None):
        self.report.update(diagnostic_revision=2,probe_register='LED',probe_address=65,probe_size=1,
            led_restoration_verified=False,torque_writes=0,goal_writes=0,
            drive_setting_writes=0,eeprom_writes=0,torque_enabled_by_audit=False,phase='initial_OFF',
            loaded_goal_delivery_verified=False,physical_rotation_verified=False,
            limitation='LED writes use one byte per motor, not four-byte Goal Position writes under load.')
        camera=None;error=None
        try:
            self.original=np.asarray(self.fresh_off()['led']).copy()
            self.report['original_led_values']=self.original.tolist()
            self.phase('camera_closed',seconds)
            if camera_factory is not None:
                self.report['phase']='camera_startup_OFF'
                camera=camera_factory()
                self.phase('camera_streaming',seconds,camera)
            self.report['status']='OFF_LED_echo_complete'
        except (Exception,KeyboardInterrupt) as exc:
            error=exc;self.report.update(status='stopped',error=str(exc) or 'Interrupted',failed_phase=self.report.get('phase'))
        finally:
            try:self.restore()
            except Exception as exc:
                self.report.update(led_restoration_verified=False,restoration_error=str(exc))
                if error is None:error=exc;self.report.update(status='stopped',error=str(exc))
            if camera is not None:
                try:camera.release()
                except Exception as exc:
                    self.report['camera_release_error']=str(exc)
                    if error is None:error=exc;self.report.update(status='stopped',error=str(exc))
        if error is not None:raise error
        if not self.report['led_restoration_verified']:raise RuntimeError('Original LED restoration unverified')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');args=parser.parse_args(argv)
    print('OFF BUS ECHO v2: LEDs ONLY. Torque must already be OFF. Cube removed.',flush=True)
    print('15s camera closed + 15s streaming. LEDs may blink; original LED values restored.',flush=True)
    print('No goal, torque, current, mode, gain or EEPROM writes. No hand motion commands.',flush=True)
    if not args.execute:
        print('CHECK ONLY: no camera, serial port or settings opened.');return 0
    folder=ROOT/('bus_echo_off_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),diagnostic_revision=2,
        status='not_started',all_torque_off_verified=False,led_restoration_verified=False)
    bus=None;opened=False
    try:
        import msvcrt
        def cancel():
            if msvcrt.kbhit() and msvcrt.getwch() in ('q','Q','\x1b','\x03'):
                raise KeyboardInterrupt('User stop')
        print('Close other motor/camera apps. Connect USB + motor power. Leave fingers as they are.',flush=True)
        print('SPACE here starts OFF checks; camera starts automatically later. Q cancels.',flush=True)
        while True:
            if msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                if key==' ':break
            time.sleep(.02)
        import dynamixel_sdk as sdk
        import sequence_flip_checked as checked
        bus=checked.Bus(sdk,report)
        if not bus.port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True
        if not bus.port.setBaudRate(3000000):raise RuntimeError('Cannot set baud')
        for ident in range(1,17):
            cancel();model,comm,error=bus.packet.ping(bus.port,ident)
            if model!=1240 or comm!=sdk.COMM_SUCCESS or error:
                raise RuntimeError(f'ID{ident} model/communication check failed')
            if bus.read(ident,11)!=5:
                raise RuntimeError(f'ID{ident} is not in expected mode; no LED write')
        from camera_stream import CameraStream
        audit=EchoAudit(lambda:read_off_snapshot(bus),
            lambda led:write_off_led_packet(sdk,bus.port,bus.packet,led),report,cancel=cancel)
        audit.run(camera_factory=lambda:CameraStream(report,preview_title='OFF LED bus echo | Q stops'))
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
        print('STOP:',report['error'],flush=True)
    finally:
        if bus is not None and opened:bus.port.closePort()
        payload=json.dumps(report,ensure_ascii=False,indent=2)
        (folder/'result.json').write_text(payload,encoding='utf-8')
        (ROOT/'leap_bus_echo_off_result.json').write_text(payload,encoding='utf-8')
        print('Saved:',folder,flush=True)
        print('Original LEDs restored:',report['led_restoration_verified'],flush=True)
        print('Capture complete. Send just done; saved results will be reviewed.',flush=True)
    return 0 if report.get('status')=='OFF_LED_echo_complete' and report['led_restoration_verified'] else 1


if __name__=='__main__':raise SystemExit(main())
