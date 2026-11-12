"""Read saved stage-2 runs without opening camera or motor APIs."""
import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "right_hand_rotation_20261112/stage2_actual_review"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(path):
    raw = json.loads(path.read_bytes())
    samples = raw.get("samples", [])
    phases = {}
    for phase in dict.fromkeys(s["phase"] for s in samples):
        ss = [s for s in samples if s["phase"] == phase]
        phases[phase] = {"samples": len(ss), "first_t": ss[0]["t"], "last_t": ss[-1]["t"]}
    report = {"folder": path.parent.name, "source_result_sha256": sha(path),
              **{k: raw.get(k) for k in ("timestamp", "status", "error", "failed_phase", "primary_motion_error",
                  "all_torque_off_verified", "settings_restored", "work_grip_transition_attempted",
                  "cube_support_confirmed_before_off", "cube_removal_confirmed", "current_start_screen_accepted")},
              "samples": len(samples), "phase_ranges": phases,
              "read_failures": len(raw.get("read_failures", [])),
              "all_recorded_read_failures_recovered": all(r.get("recovered") for r in raw.get("read_failures", [])),
              "camera_recording": raw.get("camera_recording"),
              "named_photos": [p.name for p in path.parent.glob("*.jpg")],
              "assistant_motor_writes": 0, "devices_opened_for_review": False}
    if samples:
        report.update(minimum_voltage_v=min(min(s["voltage_v"]) for s in samples),
                      maximum_absolute_current_ma=max(max(abs(v) for v in s["current_ma"]) for s in samples),
                      maximum_tracking_rad=max(max(s["tracking_rad"]) for s in samples),
                      maximum_temperature_c=max(max(s["temperature_c"]) for s in samples))
    return report


def analyze(folder, summary):
    import cv2 as cv
    import numpy as np
    from center_marker_cube_pose_20261026 import estimate
    from work_grip_stage2_plan_20261112 import validate_candidate, HOLD, MOVE

    raw = json.loads((folder / "result.json").read_bytes())
    hold = [s for s in raw["samples"] if s["phase"] == HOLD]
    if not hold:
        raise RuntimeError("No actual work-grip hold samples")
    actual = np.asarray([s["actual"] for s in hold])
    goals = np.asarray([s["goal"] for s in hold])
    candidate = validate_candidate()
    writes = np.asarray([e["requested"] for e in raw["write_events"] if e["address"] == 116])
    goal_match = len(writes) >= len(candidate["goals"]) and np.array_equal(writes[-len(candidate["goals"]):], candidate["goals"])
    frames_path = folder / "camera/frames.json"
    frames = json.loads(frames_path.read_bytes())["frames"]
    enabled = raw["enabled_at"]
    times = np.asarray([f["camera_read_monotonic"] - enabled for f in frames])
    detector = cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50), cv.aruco.DetectorParameters())
    calibration_path = ROOT / "camera1_720.npz"
    cal = np.load(calibration_path)

    def markers(path):
        frame = cv.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("Cannot read saved source frame")
        cs, ids, _ = detector.detectMarkers(frame)
        return {} if ids is None else {int(i): c.reshape(4, 2) for c, i in zip(cs, ids.ravel())}

    photographs = []
    for name in ("cube_before_grasp.jpg", "cube_before_work_grip.jpg", "cube_after_work_grip_hold.jpg"):
        path = folder / name
        found = markers(path)
        photographs.append({"image": name, "sha256": sha(path), "detected_ids": sorted(found),
                            "corners_px": {str(i): v.tolist() for i, v in found.items()},
                            "pose_attempt": estimate(found, cal["K"], cal["dist"], marker_m=.0107)})
    hold_frames = [f for f, t in zip(frames, times) if hold[0]["t"] <= t <= hold[-1]["t"]]
    observations = []
    for f in hold_frames:
        found = markers(folder / "camera" / f["image"])
        observations.append({"image": f["image"], "t": f["camera_read_monotonic"] - enabled,
                             "detected_ids": sorted(found), "centers_px": {str(i): p.mean(0).tolist() for i, p in found.items()}})
    image_stats = {"saved_frames_in_hold_window": len(hold_frames),
                   "frames_with_any_cube_marker": sum(any(i in range(6) for i in r["detected_ids"]) for r in observations),
                   "pixels_not_mm": True}
    for ident in (0, 3, 5, 40, 41):
        centers = [r["centers_px"][str(ident)] for r in observations if str(ident) in r["centers_px"]]
        if centers:
            image_stats[f"marker{ident}_center_range_px"] = np.ptp(centers, axis=0).tolist()
            image_stats[f"marker{ident}_frames"] = len(centers)
    for anchor in (40, 41):
        centered = [np.asarray(r["centers_px"]["0"]) - np.asarray(r["centers_px"][str(anchor)])
                    for r in observations if "0" in r["centers_px"] and str(anchor) in r["centers_px"]]
        if centered:
            image_stats[f"white0_minus_anchor{anchor}_frames"] = len(centered)
            image_stats[f"white0_minus_anchor{anchor}_range_px"] = np.ptp(centered, axis=0).tolist()
    move = summary["phase_ranges"][MOVE]
    points = [("before_transition", move["first_t"])]
    points += [(f"transition_{i}", float(t)) for i, t in enumerate(np.linspace(move["first_t"], move["last_t"], 9))]
    points += [(f"hold_{i}", float(t)) for i, t in enumerate(np.linspace(hold[0]["t"], hold[-1]["t"], 5))]
    selected = []
    for name, t in points:
        i = int(np.argmin(abs(times - t)))
        f = frames[i]
        selected.append({"label": name, "image": (folder / "camera" / f["image"]).relative_to(ROOT).as_posix(),
                         "phase": f["phase"], "t": float(times[i])})
    result = {**summary, "status": "saved_stage2_transition_and_hold_review",
              "actual_program_status": summary["status"], "all_loaded_goal_writes_exactly_match_candidate": bool(goal_match),
              "planned_transition_goals": int(np.count_nonzero(candidate["phases"] == MOVE)),
              "planned_work_hold_goals": int(np.count_nonzero(candidate["phases"] == HOLD)),
              "hold_read_samples": len(hold), "hold_sample_time_span_s": hold[-1]["t"] - hold[0]["t"],
              "hold_goals_fixed": bool(np.all(goals == goals[0])),
              "hold_goal_equals_reviewed_endpoint": np.array_equal(goals[0], candidate["goals"][-1]),
              "hold_joint_actual_peak_to_peak_ticks": np.ptp(actual, axis=0).tolist(),
              "hold_maximum_tracking_rad": max(max(s["tracking_rad"]) for s in hold),
              "hold_all_recorded_torque_on": all(all(v == 1 for v in s["torque"]) for s in hold),
              "hold_all_hardware_error_zero": all(all(v == 0 for v in s["error"]) for s in hold),
              "named_photographs": photographs, "selected_source_frames": selected,
              "hold_frame_observations": observations, "hold_image_statistics": image_stats,
              "frame_metadata_sha256": sha(frames_path), "intrinsic_sha256": sha(calibration_path),
              "every_saved_frame_visually_reviewed": False, "precise_cube_robot_registration_verified": False,
              "grip_force_measured": False, "actual_left_hand_layer_rotation_load_tested": False,
              "second_whole_cube_rotation_tested": False, "post_OFF_sag_not_an_ON_hold_failure": True}
    (OUT / "review.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    sim_ids = (8, 6, 7, 5, 12, 10, 11, 9, 16, 14, 15, 13, 4, 3, 2, 1)
    reference = {"status": "measured_work_grip_endpoint_not_a_new_motion_goal", "source": (folder / "result.json").relative_to(ROOT).as_posix(),
                 "source_result_sha256": summary["source_result_sha256"], "actual_ticks": hold[-1]["actual"],
                 "goal_ticks": hold[-1]["goal"], "sim_motor_ids": list(sim_ids),
                 "actual_qsim_rad": [(hold[-1]["actual"][i - 1] - 2048) * 2 * np.pi / 4096 for i in sim_ids],
                 "cube_pose_registered": False, "cube_velocity_observed": False, "contact_forces_observed": False,
                 "full_dynamic_model_state_reconstructed": False, "new_second_rotation_executable_ready": False}
    (OUT / "actual_work_grip_end_reference.json").write_text(json.dumps(reference, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: result[k] for k in ("all_loaded_goal_writes_exactly_match_candidate", "planned_transition_goals",
          "planned_work_hold_goals", "hold_read_samples", "hold_sample_time_span_s", "hold_goals_fixed",
          "hold_joint_actual_peak_to_peak_ticks", "hold_maximum_tracking_rad", "hold_image_statistics", "selected_source_frames")}, ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--analyze", action="store_true")
    parser.add_argument("--inspect", action="store_true")
    args = parser.parse_args()
    if args.inspect:
        saved = json.loads((OUT / "review.json").read_bytes())
        keys = ("actual_program_status", "all_loaded_goal_writes_exactly_match_candidate", "hold_read_samples",
                "hold_sample_time_span_s", "hold_goals_fixed", "hold_goal_equals_reviewed_endpoint",
                "hold_joint_actual_peak_to_peak_ticks", "hold_maximum_tracking_rad", "hold_all_recorded_torque_on",
                "hold_all_hardware_error_zero", "hold_image_statistics", "selected_source_frames")
        print(json.dumps({k: saved[k] for k in keys}, indent=2, ensure_ascii=False))
        print("PHOTO_POSE_ATTEMPTS", json.dumps([{k: p[k] for k in ("image", "detected_ids", "pose_attempt")}
                                                for p in saved["named_photographs"]], ensure_ascii=False))
        return
    paths = sorted(ROOT.glob("work_grip_stage2_20261112_*/result.json"))
    if not paths:
        raise RuntimeError("No saved stage-2 run found")
    OUT.mkdir(parents=True, exist_ok=True)
    reports = []
    for path in paths:
        report = summarize(path)
        reports.append(report)
        (OUT / (path.parent.name + ".json")).write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(json.dumps({k: v for k, v in report.items() if k != "camera_recording"}, ensure_ascii=False), flush=True)
    (OUT / "inventory.json").write_text(json.dumps({"runs": reports}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if args.analyze:
        analyze(paths[-1].parent, reports[-1])


if __name__ == "__main__":
    main()
