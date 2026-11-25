"""One PING/READ-only pass after reconnection. No servo write interface."""
import argparse
import datetime as dt
import json
from pathlib import Path
from serial.tools import list_ports
from bus_readonly_diagnose import DiagnosticBus,scan_all


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    if not args.execute:print('CHECK ONLY: no port opened');return 0
    root=Path(__file__).resolve().parent
    folder=root/('reconnected_bus_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    r=dict(timestamp=dt.datetime.now().astimezone().isoformat(),servo_write_commands_sent=0,
        torque_changes_requested=0,camera_opened=False,
        ports=[dict(device=p.device,description=p.description,hwid=p.hwid) for p in list_ports.comports()])
    bus=None;passed=False
    try:
        import dynamixel_sdk as sdk
        bus=DiagnosticBus(sdk,'COM7',r);bus.install_receive_trace();bus.open()
        passed=scan_all(bus,r,cancel=lambda:None,rounds=1)
        r['status']='all16_OFF_health_read' if passed else 'missing_or_unhealthy_response'
    except (Exception,KeyboardInterrupt) as exc:r.update(status='stopped',error=str(exc))
    finally:
        if bus is not None:bus.close()
        (folder/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(folder=str(folder),status=r['status'],error=r.get('error'),
        ports=r['ports'],responding_ids=r.get('responding_ids'),
        all16_OFF=r.get('all_16_torque_OFF_verified',False),servo_write_commands_sent=0),ensure_ascii=False))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
