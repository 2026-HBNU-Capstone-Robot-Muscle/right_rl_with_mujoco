"""Preflight snapshots only; no motor motion, settings, or torque writes.

Default: enumerate host ports without opening any port. --read explicitly
permits PING/READ on the known adapter at the recorded 1 Mbps host setting.
This result is not approval of any new motion sequence.
"""
import argparse
import datetime as dt
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ADAPTER = (1027, 24596, "FTBENPSAA")
SIM_IDS = (8, 6, 7, 5, 12, 10, 11, 9, 16, 14, 15, 13, 4, 3, 2, 1)
READ_WINDOWS = {(0, 64), (64, 83)}


def select_adapter(devices):
    matches = [d for d in devices if (d.vid, d.pid, d.serial_number) == ADAPTER]
    if len(matches) != 1:
        raise RuntimeError("Known robot USB adapter missing or ambiguous; no port opened")
    return matches[0]


def install_readonly_filter(port, counts):
    original = port.writePort

    def readonly(data):
        wire = bytes(data)
        if (len(wire) < 10 or wire[:4] != b"\xff\xff\xfd\x00"
                or wire[4] not in range(1, 17)
                or int.from_bytes(wire[5:7], "little") != len(wire) - 7):
            raise RuntimeError("Malformed or non-individual packet blocked")
        instruction = wire[7]
        if instruction == 1 and len(wire) == 10:
            name = "PING"
        elif instruction == 2 and len(wire) == 14:
            window = (int.from_bytes(wire[8:10], "little"),
                      int.from_bytes(wire[10:12], "little"))
            if window not in READ_WINDOWS:
                raise RuntimeError("Undocumented READ window blocked")
            name = "READ"
        else:
            raise RuntimeError("Servo instruction blocked; only individual PING/READ allowed")
        counts[name] = counts.get(name, 0) + 1
        return original(data)

    port.writePort = readonly


def unsigned(data, start, address, size):
    return int.from_bytes(bytes(data[address - start:address - start + size]), "little")


def signed(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def decode_config(data):
    if len(data) != 64:
        raise ValueError("Incomplete configuration read")
    value = lambda address, size=1: unsigned(data, 0, address, size)
    return {"drive_mode": value(10), "operating_mode": value(11),
            "homing_offset_raw": signed(value(20, 4), 32),
            "current_limit_raw": value(38, 2),
            "device_position_min_ticks": value(52, 4),
            "device_position_max_ticks": value(48, 4)}


def decode_health(data):
    if len(data) != 83:
        raise ValueError("Incomplete health/position read")
    value = lambda address, size=1: unsigned(data, 64, address, size)
    ticks = signed(value(132, 4), 32)
    return {"torque_enabled": value(64), "hardware_error": value(70),
            "watchdog_raw": value(98), "position_ticks": ticks,
            "qsim_rad": (ticks - 2048) * 2 * math.pi / 4096,
            "goal_ticks_readonly": signed(value(116, 4), 32),
            "present_current_raw": signed(value(126, 2), 16),
            "voltage_v": value(144, 2) / 10, "temperature_c": value(146),
            "position_pid_readonly": {"p": value(84, 2), "i": value(82, 2), "d": value(80, 2)},
            "goal_current_raw_readonly": signed(value(102, 2), 16)}


def collect(port, packet, report):
    supported = []
    for ident in range(1, 17):
        model, comm, error = packet.ping(port, ident)
        report["inventory"].append({"id": ident, "model": model, "communication": comm, "device_error": error})
        if comm or error or model != 1240:
            report["errors"].append(f"ID{ident}: PING/model check failed ({model}, {comm}, {error})")
        else:
            supported.append(ident)
    for ident in supported:
        data, comm, error = packet.readTxRx(port, ident, 0, 64)
        if comm or error or len(data) != 64:
            raise RuntimeError(f"ID{ident}: incomplete configuration read ({comm}, {error})")
        report["configuration"].append({"id": ident, **decode_config(data)})
    for sample in range(2):
        rows = []
        for ident in supported:
            data, comm, error = packet.readTxRx(port, ident, 64, 83)
            if comm or error or len(data) != 83:
                raise RuntimeError(f"ID{ident}: incomplete health read ({comm}, {error})")
            row = {"id": ident, **decode_health(data)}
            rows.append(row)
            if row["torque_enabled"] != 0 or row["hardware_error"] or row["watchdog_raw"] == 255:
                report["errors"].append(f"ID{ident}: torque/error/watchdog needs review; no setting changed")
        report["snapshots"].append({"sample": sample, "motors": rows})
    report["all_16_saved_twice"] = all(len(s["motors"]) == 16 for s in report["snapshots"])
    if report["all_16_saved_twice"]:
        first, last = [{r["id"]: r for r in s["motors"]} for s in report["snapshots"]]
        report["position_delta_ticks"] = {str(i): last[i]["position_ticks"] - first[i]["position_ticks"] for i in range(1, 17)}
        report["maximum_endpoint_delta_ticks"] = max(abs(v) for v in report["position_delta_ticks"].values())
        report["qsim_last_in_sim_order_rad"] = [last[i]["qsim_rad"] for i in SIM_IDS]
    report["status"] = "read_only_snapshot_saved" if report["all_16_saved_twice"] and not report["errors"] else "readback_requires_review"


def run_read(sdk, devices, report):
    port = None
    try:
        device = select_adapter(devices)
        report["port"] = device.device
        port = sdk.PortHandler(device.device)
        install_readonly_filter(port, report["instruction_counts"])
        if not port.openPort() or not port.setBaudRate(1000000):
            raise RuntimeError("Cannot open the known adapter at 1 Mbps")
        collect(port, sdk.PacketHandler(2.0), report)
    except (Exception, KeyboardInterrupt) as exc:
        report["status"] = "stopped"
        report["errors"].append(str(exc))
    finally:
        report["serial_port_closed"] = port is None
        if port is not None:
            try:
                port.closePort()
                report["serial_port_closed"] = True
            except Exception as exc:
                report["status"] = "stopped"
                report["errors"].append(f"Port close failed: {exc}")


def new_report():
    return {"status": "started", "baud": 1000000, "servo_writes": 0, "torque_changes": 0,
            "instruction_counts": {}, "inventory": [], "configuration": [], "snapshots": [], "errors": [],
            "all_16_saved_twice": False, "sim_motor_ids": list(SIM_IDS),
            "motion_sequence_approved_by_this_read": False,
            "position_deltas_are_endpoints_not_continuous_stationarity": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--read", action="store_true", help="PING/READ only; no servo writes")
    parser.add_argument("--confirm-space", action="store_true", help="Wait for the operator before opening a port")
    args = parser.parse_args()
    from serial.tools.list_ports import comports
    devices = list(comports())
    if not args.read:
        print(json.dumps({"status": "inventory_only", "ports": [d.device for d in devices],
                          "known_adapter_found": sum((d.vid, d.pid, d.serial_number) == ADAPTER for d in devices) == 1,
                          "serial_ports_opened": 0, "servo_writes": 0}, ensure_ascii=False))
        return 0
    if args.confirm_space:
        import msvcrt
        print("읽기 전용 점검입니다. 모터를 움직이거나 토크·설정을 바꾸지 않습니다. SPACE 시작 / Q 취소.", flush=True)
        while True:
            key = msvcrt.getwch()
            if key == " ":
                break
            if key in ("q", "Q", "\x1b", "\x03"):
                print("취소됨. 장치 접근 없음.")
                return 1
        devices = list(comports())
    import dynamixel_sdk as sdk
    now = dt.datetime.now(dt.timezone.utc).astimezone(dt.timezone(dt.timedelta(hours=9)))
    folder = ROOT / ("cycle_test_readonly_" + now.strftime("%Y%m%d_%H%M%S_%f"))
    folder.mkdir()
    report = new_report()
    report["timestamp_kst"] = now.isoformat()
    run_read(sdk, devices, report)
    (folder / "result.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"folder": folder.name, "status": report["status"],
                      "all_16_saved_twice": report["all_16_saved_twice"],
                      "errors": report["errors"], "instructions": report["instruction_counts"],
                      "serial_port_closed": report["serial_port_closed"], "servo_writes": 0,
                      "motion_sequence_approved_by_this_read": False}, ensure_ascii=False))
    return 0 if report["status"] == "read_only_snapshot_saved" and report["serial_port_closed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
