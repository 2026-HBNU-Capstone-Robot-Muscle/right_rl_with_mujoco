"""Review a saved read-only snapshot and, optionally, capture camera-only frames.

No motor bus API is constructed. An existing forward/hold experiment is checked
as a first diagnostic option; this does not prepare the new two-turn cycle.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BASE = ROOT / "right_hand_rotation_20261109/prehardware_policy_cycle/physical_test_preparation_20261111"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(snapshot):
    import numpy as np
    from forward_hold_plan_20261104 import ForwardHoldPlan, PROOF, REQUEST
    from forward_hold_gate_20261104 import validate_review
    from loaded_quarter_joint_limits_20261025 import configured_limits
    from hand_motor_mapping import SIM_TO_MOTOR_IDS

    source = Path(snapshot).resolve()
    source.relative_to(ROOT)
    data = json.loads(source.read_bytes())
    if (data["status"] != "read_only_snapshot_saved" or not data["all_16_saved_twice"]
            or not data["serial_port_closed"] or data["servo_writes"] or data["torque_changes"]
            or data["maximum_endpoint_delta_ticks"] > 3):
        raise RuntimeError("Snapshot incomplete, unhealthy, moving, or not read-only")
    rows = sorted(data["snapshots"][-1]["motors"], key=lambda r: r["id"])
    if [r["id"] for r in rows] != list(range(1, 17)):
        raise RuntimeError("Snapshot motor IDs changed")
    state = {out: [row[key] for row in rows] for out, key in (
        ("position", "position_ticks"), ("torque", "torque_enabled"), ("error", "hardware_error"),
        ("watchdog", "watchdog_raw"), ("temperature", "temperature_c"))}
    state["voltage"] = [int(round(r["voltage_v"] * 10)) for r in rows]
    folder = BASE / ("connected_" + source.parent.name.removeprefix("cycle_test_readonly_"))
    folder.mkdir(parents=True, exist_ok=True)
    request = folder / "forward_hold_request.json"
    body = dict(plan_version=ForwardHoldPlan.PLAN_VERSION,
                motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS), off_state=state,
                source=source.relative_to(ROOT).as_posix(), source_sha256=sha(source),
                hardware_execution_allowed=False)
    text = json.dumps(body, ensure_ascii=False, indent=2) + "\n"
    if request.exists() and request.read_text(encoding="utf-8") != text:
        raise RuntimeError("Refuse to replace a different current request")
    request.write_text(text, encoding="utf-8")
    low, high = configured_limits()
    position = np.asarray(state["position"])
    outside = np.flatnonzero((position < low) | (position > high))
    result = dict(source=body["source"], source_sha256=body["source_sha256"],
                  actual_snapshot_status=data["status"], actual_port=data["port"], baud=data["baud"],
                  model_ids=[r["model"] for r in data["inventory"]],
                  all_torque_off=all(v == 0 for v in state["torque"]),
                  all_hardware_error_zero=all(v == 0 for v in state["error"]),
                  voltage_v=[min(r["voltage_v"] for r in rows), max(r["voltage_v"] for r in rows)],
                  temperature_c=[min(state["temperature"]), max(state["temperature"])],
                  maximum_endpoint_delta_ticks=data["maximum_endpoint_delta_ticks"],
                  position_ticks_by_id={str(r["id"]): r["position_ticks"] for r in rows},
                  outside_configured_goal_limits_ids=(outside + 1).tolist(),
                  motion_approved=False, motor_bus_constructions=0, motor_writes=0,
                  first_diagnostic_option="existing physically observed forward route with fixed 10s hold; not the new full cycle")
    try:
        proof = validate_review(ForwardHoldPlan(REQUEST), PROOF)
        result["existing_forward_hold_proof_valid"] = True
        plan = ForwardHoldPlan(request)
        goals = plan.plan(plan.source_position())
        phases = {}
        for phase, _ in goals:
            phases[phase] = phases.get(phase, 0) + 1
        result.update(current_request_accepted_by_existing_plan=True, current_total_goals=len(goals),
                      current_motion_seconds=len(goals) * plan.SAMPLE_PERIOD,
                      current_phase_counts=phases, current_empty_recovery_model_review_required=True,
                      request=request.relative_to(ROOT).as_posix(), unchanged_loaded_goals=proof["loaded_goals"])
        np.savez_compressed(folder / "offline_goals.npz", start=position,
                            goals=np.asarray([g for _, g in goals]), phases=np.asarray([p for p, _ in goals]))
    except Exception as exc:
        result.update(current_request_accepted_by_existing_plan=False, preparation_error=str(exc))
    (folder / "prepare.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return folder, result


def capture_camera(folder):
    from camera_stream import CameraStream
    report = {"motor_bus_constructions": 0, "servo_writes": 0, "camera_only": True}
    stream = None
    attempt = "camera" if not (folder / "camera.json").exists() else "camera_retry_" + str(time.time_ns())
    try:
        stream = CameraStream(report)
        began = time.monotonic()
        frame = None
        count = 0
        while time.monotonic() - began < 2.0:
            packet = stream.read_packet()
            if packet is not None:
                stamp, frame = packet
                count += 1
            time.sleep(.02)
        if frame is None or time.monotonic() - stamp > .3:
            raise RuntimeError("No fresh camera frame")
        import cv2
        path = folder / (attempt + ".jpg")
        ok, encoded = cv2.imencode(".jpg", frame)
        if not ok:
            raise RuntimeError("Cannot encode camera frame")
        path.write_bytes(encoded.tobytes())
        report.update(status="camera_frame_saved", fresh_frames=count, frame_shape=list(frame.shape),
                      photograph=path.relative_to(ROOT).as_posix(), photograph_sha256=sha(path),
                      geometric_registration_verified=False)
    except Exception as exc:
        report.update(status="camera_stopped", error=str(exc))
    finally:
        if stream is not None:
            stream.release()
        (folder / (attempt + ".json")).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot")
    parser.add_argument("--camera", action="store_true")
    args = parser.parse_args()
    folder, result = prepare(args.snapshot)
    if args.camera:
        camera = capture_camera(folder)
        result["camera_status"] = camera["status"]
    print(json.dumps({k: v for k, v in result.items() if k not in ("model_ids", "position_ticks_by_id", "current_phase_counts")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
