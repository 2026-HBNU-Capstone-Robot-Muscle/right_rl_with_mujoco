"""Operator-triggered camera/control timing check using PING/READ only.

No servo WRITE API, target, torque, mode, gain, or watchdog-setting capability
is supplied to the monitoring loop. Without --execute no device is opened.
"""
import argparse
import datetime as dt
import json
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parent


class ReadonlyBus:
    def __init__(self,sdk,device):
        self.sdk=sdk;self.port=sdk.PortHandler(device);self.packet=sdk.PacketHandler(2.)
        self.group=sdk.GroupSyncRead(self.port,self.packet,64,83)
        self.opened=False
        for ident in range(1,17):
            if not self.group.addParam(ident):raise RuntimeError('Cannot configure host READ group')

    def open(self):
        if not self.port.openPort():raise RuntimeError('Cannot open motor port; close other motor programs')
        self.opened=True
        if not self.port.setBaudRate(3000000):raise RuntimeError('Cannot set host serial baud rate')
        for ident in range(1,17):
            model,comm,error=self.packet.ping(self.port,ident)
            if comm!=self.sdk.COMM_SUCCESS or error or model not in (1220,1240):
                raise RuntimeError(f'ID{ident}: model/PING check failed (model={model}, comm={comm}, error={error}); no servo changes sent')

    def snapshot(self):
        comm=self.group.txRxPacket()
        if comm!=self.sdk.COMM_SUCCESS:raise RuntimeError(f'Sync READ failed: {comm}; no retry or writes')
        for ident in range(1,17):
            if not self.group.isAvailable(ident,64,83):raise RuntimeError(f'ID{ident}: incomplete fresh READ')
        def values(address,size):return [int(self.group.getData(i,address,size)) for i in range(1,17)]
        positions=values(132,4)
        return dict(torque=values(64,1),hardware_error=values(70,1),watchdog=values(98,1),
            position_ticks=[v-(1<<32) if v&(1<<31) else v for v in positions],
            voltage_v=[v/10 for v in values(144,2)],temperature_c=values(146,1))

    def close(self):
        if self.opened or getattr(self.port,'ser',None) is not None:self.port.closePort()


def validate_off(snapshot):
    fields=('torque','hardware_error','watchdog','position_ticks','voltage_v','temperature_c')
    if any(len(snapshot.get(k,[]))!=16 for k in fields):raise RuntimeError('Incomplete motor snapshot')
    if any(snapshot['torque']):raise RuntimeError('Torque is ON. Disconnect motor power; this READ ONLY check does not change torque.')
    if any(snapshot['hardware_error']):raise RuntimeError('Motor hardware error; no servo setting changes sent')
    if 255 in snapshot['watchdog']:raise RuntimeError('Motor watchdog error remains; no watchdog reset is sent by this READ ONLY check')


def monitor(read_snapshot,check_camera,report,*,cancel,seconds=12.,period=.05,now=time.monotonic,sleep=time.sleep):
    began=now();last_read=None;report['samples']=[]
    report['phase']='readonly_camera_timing'
    report['measurement_started_monotonic']=began
    while now()-began<seconds:
        cycle=now();cancel();before=now();check_camera();poll_ms=(now()-before)*1000
        read_began=now();snapshot=read_snapshot();read_ended=now()
        row=dict(phase=report['phase'],t=read_ended-began,camera_poll_ms=poll_ms,read_ms=(read_ended-read_began)*1000,
            read_gap_ms=None if last_read is None else (read_ended-last_read)*1000,**snapshot)
        report['samples'].append(row);last_read=read_ended
        validate_off(snapshot)
        if poll_ms>200 or row['read_ms']>200 or (row['read_gap_ms'] or 0)>200:
            raise RuntimeError('READ ONLY timing exceeded the 200ms checked budget; no motion or setting writes occurred')
        sleep(max(0.,period-(now()-cycle)))
    rows=report['samples']
    if not rows:raise RuntimeError('No motor snapshots recorded')
    report.update(status='readonly_camera_timing_complete',timing_passed=True,
        all_observed_torque_off=True,
        max_camera_poll_ms=max(x['camera_poll_ms'] for x in rows),
        max_read_ms=max(x['read_ms'] for x in rows),
        max_read_gap_ms=max(x['read_gap_ms'] or 0 for x in rows),
        min_voltage_v=min(min(x['voltage_v']) for x in rows),
        elapsed_seconds=now()-began)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--port',default='COM7')
    args=parser.parse_args()
    print('CAMERA + READ ONLY TIMING CHECK: empty hand, torque must already be OFF.',flush=True)
    print('No target, torque, gain, mode, EEPROM or watchdog-setting writes. No SAC or motion.',flush=True)
    if not args.execute:
        print('CHECK ONLY: no camera, serial port, or simulator process opened.');return 0
    folder=ROOT/('camera_readonly_check_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='started',phase='preview',
        servo_write_commands_sent=0,torque_changes_requested=0,policy_actions_sent=0,
        recording_requested=False,hardware_authorized=False)
    camera=bus=None
    try:
        from hand_readonly_check import require_serial_port
        from empty_trial_camera import EmptyTrialCamera
        from camera_frame_signal import CameraImageUnavailable
        from prepared_placement_capture import TerminalKeys
        import dynamixel_sdk as sdk
        import msvcrt
        require_serial_port(args.port,report)
        print('Cube OUT; close other motor programs. USB + motor power connected; leave fingers as they are.',flush=True)
        print('Aim at the whole hand. SPACE in THIS TERMINAL starts a 12s READ-only check. Q exits.',flush=True)
        camera=EmptyTrialCamera(folder/'camera',report)
        while True:
            camera.check()
            if msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                if key==' ':
                    try:camera.require_fresh()
                    except CameraImageUnavailable as exc:print(exc,flush=True);continue
                    break
            time.sleep(.01)
        report.update(hardware_authorized=True,recording_requested=True,phase='readonly_preflight')
        bus=ReadonlyBus(sdk,args.port);bus.open()
        keys=TerminalKeys()
        report['camera_instruction']='READ ONLY: briefly drag this window 2-3 times; fingers stay still; Q exits'
        print('READING NOW, 12s: briefly move the camera WINDOW by its title bar 2-3 times.',flush=True)
        print('Do not reposition fingers or place a cube. No motor will be enabled.',flush=True)
        monitor(bus.snapshot,camera.check,report,cancel=keys.poll)
        print(f"PASS: 12s READ-only check complete; {len(report['samples'])} fresh reads; max camera check {report['max_camera_poll_ms']:.1f}ms; max read gap {report['max_read_gap_ms']:.1f}ms.",flush=True)
        print('This check is finished. No hand movement is expected; do not repeat it just because the hand stayed still.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted',failed_phase=report.get('phase'),timing_passed=False)
        print('STOP:',report['error'],flush=True)
    finally:
        if bus is not None:
            try:bus.close()
            except Exception as exc:report['port_close_error']=str(exc)
        if camera is not None:
            try:report['camera_recording']=camera.finish()
            except Exception as exc:report['camera_save_error']=str(exc)
        if report.get('samples'):
            report['all_observed_torque_off']=all(not any(s['torque']) for s in report['samples'])
        payload=json.dumps(report,ensure_ascii=False,indent=2)
        (folder/'result.json').write_text(payload,encoding='utf-8')
        (ROOT/'leap_camera_readonly_check_result.json').write_text(payload,encoding='utf-8')
        print('Saved:',folder,flush=True)
        print('No motor goals, servo writes, or torque changes sent.',flush=True)
    return 0 if report['status']=='readonly_camera_timing_complete' and report.get('camera_recording',{}).get('status')=='complete' else 1


if __name__=='__main__':raise SystemExit(main())
