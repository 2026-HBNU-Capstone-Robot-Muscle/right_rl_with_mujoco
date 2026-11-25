"""Archive and summarize a user-run root trial. Offline; no device imports."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "right_hand_root_success_20261020"

def main():
    content = (ROOT / "leap_roots_empty_result.json").read_bytes()
    r = json.loads(content)
    assert r["status"] == "empty_root_lift_complete"
    assert r["all_torque_off_verified"] and r["settings_restored"]
    OUT.mkdir(exist_ok=True)
    dest = OUT / "source_result.json"
    if dest.exists() and dest.read_bytes() != content:
        raise RuntimeError("Archive already contains a different trial")
    dest.write_bytes(content)
    a, b = r["initial"]["position"], r["final_on_state"]["position"]
    goals = r["final_on_state"]["goal"]
    summary = dict(
        timestamp=r["timestamp"],
        source_sha256=hashlib.sha256(content).hexdigest(),
        status=r["status"],
        torque_off_verified=r["all_torque_off_verified"],
        settings_restored=r["settings_restored"],
        roots=[dict(id=i+1, initial_ticks=a[i], final_on_ticks=b[i],
                    goal_ticks=goals[i], measured_travel_deg=(b[i]-a[i])*360/4096,
                    final_error_deg=(goals[i]-b[i])*360/4096)
               for i in (7,11,15)],
        max_final_held_joint_drift_ticks=max(abs(b[i]-a[i]) for i in range(16) if i not in (7,11,15)),
        final_min_voltage_v=min(r["final_on_state"]["voltage"])/10,
        trial_min_voltage_v=min(min(s["voltage_v"]) for s in r["samples"]),
        trial_peak_current_ma=max(max(abs(v) for v in s["current_ma"]) for s in r["samples"]),
        trial_max_tracking_rad=max(max(s["tracking_rad"]) for s in r["samples"]),
        physical_clearance_verified=False,
        cube_rotation_verified=False,
        current_off_pose_known=False,
        motor_commands_sent_by_this_script=0,
    )
    # Keep full samples in the immutable source; inspect field names before
    # deriving current units or extrema from their raw values.
    (OUT / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
