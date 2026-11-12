"""Exercise the new preflight against fake ports only, including write denials."""
from pathlib import Path
from types import SimpleNamespace
import hashlib
import json

import cycle_test_readonly_20261111 as preflight

ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "right_hand_rotation_20261109/prehardware_policy_cycle/physical_test_preparation_20261111"


def frame(instruction, params=b"", ident=1):
    return b"\xff\xff\xfd\x00" + bytes([ident]) + (len(params) + 3).to_bytes(2, "little") + bytes([instruction]) + params + b"\x00\x00"


class FakePort:
    def __init__(self, device="FAKE"):
        self.sent = []
        self.closed = False
        self.baud = None

    def writePort(self, data):
        self.sent.append(bytes(data))
        return len(data)

    def openPort(self):
        return True

    def setBaudRate(self, baud):
        self.baud = baud
        return True

    def closePort(self):
        self.closed = True


class FakePacket:
    def __init__(self, *, torque=0, missing=None, short=False, exception=False, device_error=False):
        self.torque, self.missing, self.short = torque, missing, short
        self.exception, self.device_error = exception, device_error

    def ping(self, port, ident):
        port.writePort(frame(1, ident=ident))
        return (1240, 0, 0) if ident != self.missing else (0, -3001, 0)

    def readTxRx(self, port, ident, start, size):
        port.writePort(frame(2, start.to_bytes(2, "little") + size.to_bytes(2, "little"), ident))
        if self.exception:
            raise OSError("synthetic transport failure")
        data = bytearray(size)
        if start == 64:
            data[0] = self.torque
            data[70 - start] = 1 if self.device_error else 0
            data[132 - start:136 - start] = (2048 + ident).to_bytes(4, "little")
            data[126 - start:128 - start] = (-17).to_bytes(2, "little", signed=True)
            data[144 - start:146 - start] = (48).to_bytes(2, "little")
        return list(data[:-1] if self.short else data), 0, 0


def fake_run(**kwargs):
    port = FakePort()
    sdk = SimpleNamespace(PortHandler=lambda _: port, PacketHandler=lambda _: FakePacket(**kwargs))
    devices = [SimpleNamespace(vid=preflight.ADAPTER[0], pid=preflight.ADAPTER[1],
                               serial_number=preflight.ADAPTER[2], device="FAKE")]
    report = preflight.new_report()
    preflight.run_read(sdk, devices, report)
    return port, report


def main():
    checks = {}
    counts = {}
    port = FakePort()
    preflight.install_readonly_filter(port, counts)
    valid = [frame(1), frame(2, b"\x00\x00\x40\x00"), frame(2, b"\x40\x00\x53\x00")]
    for wire in valid:
        port.writePort(wire)
    checks["documented_ping_and_read_pass"] = len(port.sent) == 3 and counts == {"PING": 1, "READ": 2}
    denied = [frame(i, b"\x40\x00\x01") for i in (3, 4, 5, 6, 8, 16, 131, 147)]
    denied += [frame(1, ident=254), frame(2, b"\x40\x00\x01\x00"), b"short", frame(2), frame(1)[:-1]]
    for wire in denied:
        try:
            port.writePort(wire)
        except RuntimeError:
            continue
        raise AssertionError("Non-read or malformed instruction passed")
    checks["writes_broadcast_and_undocumented_reads_blocked_before_transport"] = len(port.sent) == 3
    port, report = fake_run()
    checks["complete_readback_saved_and_port_closed"] = (report["status"] == "read_only_snapshot_saved"
        and report["all_16_saved_twice"] and port.closed and port.baud == 1000000
        and report["instruction_counts"] == {"PING": 16, "READ": 48})
    checks["signed_current_and_voltage_decode"] = (report["snapshots"][0]["motors"][0]["present_current_raw"] == -17
        and report["snapshots"][0]["motors"][0]["voltage_v"] == 4.8)
    checks["sim_order_and_endpoint_delta"] = (report["maximum_endpoint_delta_ticks"] == 0
        and report["qsim_last_in_sim_order_rad"][0] == 8 * 2 * preflight.math.pi / 4096)
    checks["readback_does_not_approve_motion"] = not report["motion_sequence_approved_by_this_read"]
    for name, options in [("torque_on", {"torque": 1}), ("missing_motor", {"missing": 16}),
                          ("device_error", {"device_error": True}), ("short_read", {"short": True}),
                          ("transport_exception", {"exception": True})]:
        port, report = fake_run(**options)
        checks[name + "_cannot_pass_and_port_closed"] = report["status"] != "read_only_snapshot_saved" and port.closed and bool(report["errors"])
    for name, devices in [("missing_adapter", []), ("bluetooth_only", [SimpleNamespace(vid=None, pid=None, serial_number=None)]),
                          ("ambiguous_adapter", [SimpleNamespace(vid=1027, pid=24596, serial_number="FTBENPSAA")] * 2)]:
        made_ports = []
        sdk = SimpleNamespace(PortHandler=lambda _: made_ports.append(FakePort()))
        report = preflight.new_report()
        preflight.run_read(sdk, devices, report)
        checks[name + "_opens_no_port"] = not made_ports and report["status"] == "stopped"
    assert all(checks.values()), checks
    sources = ["cycle_test_readonly_20261111.py", "leap_cycle_test_readonly.cmd", "verify_cycle_test_readonly_20261111.py"]
    proof = {"passed": True, "checks": checks, "physical_devices_opened": 0, "servo_writes": 0,
             "fake_transport_only": True, "source_sha256": {s: hashlib.sha256((ROOT / s).read_bytes()).hexdigest() for s in sources}}
    OUTPUT.mkdir(exist_ok=True, parents=True)
    (OUTPUT / "verification.json").write_text(json.dumps(proof, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": True, "checks": len(checks), "physical_devices_opened": 0, "servo_writes": 0}))


if __name__ == "__main__":
    main()
