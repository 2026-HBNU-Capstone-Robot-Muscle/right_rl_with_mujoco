"""Camera-closed PING/READ diagnosis of all sixteen motors. No servo writes."""
import argparse
import datetime as dt
import json
from pathlib import Path
import time
from leap_camera_readonly_check import ReadonlyBus,validate_off

ROOT=Path(__file__).resolve().parent
IDS=tuple(range(1,17))
ROUNDS=3
STREAM_SECONDS=5.


class DiagnosticBus(ReadonlyBus):
    def __init__(self,sdk,device,report):
        super().__init__(sdk,device);self.report=report

    def open(self):
        # Model checks deliberately happen in scan_all, so one missing ID
        # does not hide the other fifteen. No automatic replacement port.
        if not self.port.openPort():raise RuntimeError('Cannot open COM port; close other motor programs')
        self.opened=True
        if not self.port.setBaudRate(3000000):raise RuntimeError('Cannot set host baud rate')

    def install_receive_trace(self):
        """Copy only failed receive bytes. Keep the SDK result unchanged."""
        original=self.packet.rxPacket
        def receive(port,fast_option):
            raw,result=original(port,fast_option)
            if result!=self.sdk.COMM_SUCCESS:
                events=self.report.setdefault('failed_receive_packets',[])
                if len(events)<64:
                    data=bytes(raw)
                    declared=(7+data[5]+(data[6]<<8) if len(data)>=7 and data[:4]==bytes([255,255,253,0]) else None)
                    events.append(dict(comm=int(result),received_bytes=len(data),
                        declared_packet_bytes=declared,raw_prefix_hex=data[:128].hex(),
                        phase=self.report.get('phase'),
                        length_complete=bool(declared is not None and len(data)>=declared)))
            return raw,result
        self.packet.rxPacket=receive


def scan_all(bus,report,*,cancel,now=time.monotonic,sleep=time.sleep,rounds=ROUNDS):
    began=now();rows=report.setdefault('motor_scans',[])
    for round_index in range(rounds):
        report['phase']=f'camera_closed_ping_round_{round_index+1}'
        for ident in IDS:
            cancel()
            if now()-began>30.:raise RuntimeError('Read-only scan exceeded 30s; no changes sent')
            row=dict(round=round_index+1,id=ident,model_verified=False,health_complete=False)
            rows.append(row);before=now()
            model,comm,error=bus.packet.ping(bus.port,ident)
            row.update(model=int(model),ping_comm=int(comm),ping_device_error=int(error),ping_ms=(now()-before)*1000)
            if comm!=bus.sdk.COMM_SUCCESS or error:
                row['status']='ping_failed'
                continue
            if model!=1240:
                row['status']='unsupported_model_no_control_table_reads'
                continue
            row['model_verified']=True;row['status']='reading_health'
            for key,address,size in (('torque',64,1),('hardware_error',70,1),('watchdog',98,1),
                                     ('voltage_tenths',144,2),('temperature_c',146,1)):
                cancel();before=now()
                method=bus.packet.read1ByteTxRx if size==1 else bus.packet.read2ByteTxRx
                value,comm,error=method(bus.port,ident,address)
                row.setdefault('reads',[]).append(dict(address=address,size=size,value=int(value),
                    comm=int(comm),device_error=int(error),read_ms=(now()-before)*1000))
                if comm!=bus.sdk.COMM_SUCCESS or error:
                    row['status']='health_read_failed';break
                row[key]=int(value)
                if key in ('torque','hardware_error','watchdog') and value:
                    raise RuntimeError(f'ID{ident} {key}={value}; stop and disconnect motor power. This diagnostic sends no reset or OFF write.')
                if key=='voltage_tenths' and not 38<=value<=65:
                    raise RuntimeError(f'ID{ident} voltage {value/10:.1f}V outside checked range; no changes sent')
                if key=='temperature_c' and value>45:
                    raise RuntimeError(f'ID{ident} temperature {value}C outside checked range; no changes sent')
            else:
                row.update(health_complete=True,status='OFF_health_read_complete')
            sleep(.005)
        present=[r['id'] for r in rows if r['round']==round_index+1 and r['model_verified']]
        print(f'PING round {round_index+1}/{rounds}: responding expected motors {present}',flush=True)
        sleep(.15)
    expected=len(IDS)*rounds
    timing=all(r['ping_ms']<=200 and all(v['read_ms']<=200 for v in r.get('reads',[])) for r in rows)
    complete=len(rows)==expected and all(r['health_complete'] for r in rows) and timing
    report.update(scan_all_16_complete=complete,
        scan_timing_within_200ms=timing,
        responding_ids=sorted({r['id'] for r in rows if r['model_verified']}),
        ping_success_count_by_id={str(i):sum(r['model_verified'] for r in rows if r['id']==i) for i in IDS},
        all_16_torque_OFF_verified=complete)
    return complete


def stream_check(bus,report,*,cancel,now=time.monotonic,sleep=time.sleep,seconds=STREAM_SECONDS):
    began=now();rows=report.setdefault('stream_samples',[]);report['phase']='camera_closed_83_byte_stream'
    while now()-began<seconds:
        cycle=now();cancel();report['all_16_torque_OFF_verified']=False
        before=now();state=bus.snapshot();ended=now()
        row=dict(t=ended-began,read_ms=(ended-before)*1000,**state);rows.append(row)
        validate_off(state)
        if any(state['watchdog']):raise RuntimeError('Watchdog is not zero; no reset is sent')
        if any(not 3.8<=v<=6.5 for v in state['voltage_v']) or any(v>45 for v in state['temperature_c']):
            raise RuntimeError('Read-only stream health outside checked range')
        if row['read_ms']>200:raise RuntimeError('Read-only stream exceeded existing 200ms read budget')
        report['all_16_torque_OFF_verified']=True
        sleep(max(0.,.05-(now()-cycle)))
    if not rows:raise RuntimeError('No stream snapshots saved')
    report.update(stream_complete=True,maximum_stream_read_ms=max(r['read_ms'] for r in rows))


def diagnose(bus,report,*,cancel,now=time.monotonic,sleep=time.sleep):
    report.update(servo_write_commands_sent=0,torque_changes_requested=0,
        camera_opened=False,stream_complete=False,scan_all_16_complete=False,
        all_16_torque_OFF_verified=False,read_requests_are_motion_approval=False)
    if not scan_all(bus,report,cancel=cancel,now=now,sleep=sleep):
        report['status']='read_only_scan_found_missing_or_unhealthy_response'
        return False
    stream_check(bus,report,cancel=cancel,now=now,sleep=sleep)
    report['status']='camera_closed_all_16_read_only_check_complete'
    return True


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true')
    args=parser.parse_args(argv)
    print('READ ONLY BUS DIAGNOSIS. Camera CLOSED. All 16 IDs checked individually, then 5s of the same 83-byte READ traffic.',flush=True)
    print('No goal, torque, mode, gain, current, watchdog or EEPROM writes. No hand movement commands.',flush=True)
    if not args.execute:
        print('CHECK ONLY. No camera or serial port opened.');return 0
    folder=ROOT/('bus_readonly_diagnose_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='not_started',
        servo_write_commands_sent=0,torque_changes_requested=0,camera_opened=False)
    bus=None;passed=False
    try:
        import msvcrt
        from hand_readonly_check import require_serial_port
        require_serial_port('COM7',report)
        def cancel():
            if msvcrt.kbhit() and msvcrt.getwch() in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
        print('Cube OUT; motor power ON; torque must already be OFF. Close every motor/camera program.',flush=True)
        print('Leave fingers as they are. SPACE in this TERMINAL starts the READ-only check. Q exits.',flush=True)
        while True:
            if msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                if key==' ':break
            time.sleep(.02)
        import dynamixel_sdk as sdk
        bus=DiagnosticBus(sdk,'COM7',report);bus.install_receive_trace();bus.open()
        passed=diagnose(bus,report,cancel=cancel)
        print('Read-only check finished. Responding IDs:',report['responding_ids'],flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted',failed_phase=report.get('phase'))
        print('STOP:',report['error'],flush=True)
    finally:
        if bus is not None:
            try:bus.close()
            except Exception as exc:
                report.update(port_close_error=str(exc));passed=False
        report['hardware_response_check_passed']=passed
        payload=json.dumps(report,ensure_ascii=False,indent=2)
        (folder/'result.json').write_text(payload,encoding='utf-8')
        (ROOT/'leap_bus_readonly_diagnose_result.json').write_text(payload,encoding='utf-8')
        print('Saved:',folder,flush=True)
        print('Send just done. No motor settings were written.',flush=True)
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
