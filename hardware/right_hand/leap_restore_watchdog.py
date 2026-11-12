"""Restore idle XC330 Bus Watchdog(98) to zero; never enable torque or send goals."""
import datetime
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
IDS=range(1,17)


def run(sdk,report):
    port=sdk.PortHandler('COM7')
    packet=sdk.PacketHandler(2.0)
    opened=False
    report.update(preflight=[],write_events=[],postflight=[],all_torque_off_verified=False)

    def read(motor,address,size=1):
        fn={1:packet.read1ByteTxRx,2:packet.read2ByteTxRx}[size]
        value,comm,error=fn(port,motor,address)
        if comm!=sdk.COMM_SUCCESS or error:
            raise RuntimeError(f'ID {motor} READ {address}: comm={comm}, error={error}')
        return int(value)

    def state(motor):
        model,comm,error=packet.ping(port,motor)
        if model!=1240 or comm!=sdk.COMM_SUCCESS or error:
            raise RuntimeError(f'ID {motor} model/communication mismatch: model={model}, comm={comm}, error={error}')
        torque=read(motor,64)
        if torque!=0:
            raise RuntimeError(f'ID {motor} torque ON; cut motor power; no register writes sent')
        error_status=read(motor,70)
        if error_status:
            raise RuntimeError(f'ID {motor} hardware error {error_status}; no register writes sent')
        watchdog=read(motor,98)
        voltage=read(motor,144,2)
        temperature=read(motor,146)
        if not 35<=voltage<=65 or temperature>45:
            raise RuntimeError(f'ID {motor} voltage/temperature outside known range; no register writes sent')
        return dict(id=motor,model_number=int(model),torque_enabled=torque,
                    hardware_error=error_status,bus_watchdog_raw=watchdog,
                    voltage_raw=voltage,temperature_c=temperature)

    try:
        if not port.openPort():
            raise RuntimeError('Cannot open COM7; close other motor programs')
        opened=True
        if not port.setBaudRate(3000000):
            raise RuntimeError('Cannot set 3 Mbps host baud rate')
        for motor in IDS:
            row=state(motor)
            report['preflight'].append(row)
            if row['bus_watchdog_raw'] not in (0,25):
                raise RuntimeError(f'ID {motor} watchdog={row["bus_watchdog_raw"]}; expected 0 or 25; no writes sent')
        restore=[row['id'] for row in report['preflight'] if row['bus_watchdog_raw']==25]
        report['restore_ids']=restore
        print('All 16 torque OFF. Restore watchdog 25 -> 0 on IDs:',restore,flush=True)
        for motor in restore:
            event=dict(id=motor,address=98,value=0)
            report['write_events'].append(event)
            comm,error=packet.write1ByteTxRx(port,motor,98,0)
            event.update(comm=int(comm),device_error=int(error))
            if comm!=sdk.COMM_SUCCESS or error:
                raise RuntimeError(f'ID {motor} watchdog restore uncertain: comm={comm}, error={error}')
            observed=read(motor,98)
            event['readback']=observed
            if observed!=0:
                raise RuntimeError(f'ID {motor} watchdog readback {observed}, expected 0')
        for motor in IDS:
            row=state(motor)
            report['postflight'].append(row)
            if row['bus_watchdog_raw']!=0:
                raise RuntimeError(f'ID {motor} watchdog remains {row["bus_watchdog_raw"]}')
        report.update(status='idle_watchdog_restored_verified',all_torque_off_verified=True)
        print('Complete: 16 torque OFF; all watchdog=0. No torque ON or goal write.',flush=True)
        return 0
    finally:
        if opened:
            port.closePort()


def main():
    report=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),
                status='not_started',hardware_motion_authorized=False,
                goal_writes_sent=0,torque_enable_writes_sent=0)
    code=1
    print('EMPTY HAND. PING/READ first; only ID watchdog(98)=25 may be written to 0 while all torque OFF.',flush=True)
    print('No motor goals or torque ON. Do not run another motor program at the same time.',flush=True)
    try:
        import dynamixel_sdk as sdk
        code=run(sdk,report)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
        print('STOP:',report['error'],flush=True)
        print('Motor state uncertain; cut motor power before inspection.',flush=True)
    finally:
        path=ROOT/'leap_restore_watchdog_result.json'
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print('Saved:',path,flush=True)
    return code


if __name__=='__main__':
    raise SystemExit(main())
