"""Read the two operator-run stage-1 trials. No live devices or control APIs."""
import argparse
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import cv2 as cv
import numpy as np
from center_marker_cube_pose_20261026 import estimate

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "right_hand_rotation_20261111/stage1_actual_review"
RUNS = ("forward_hold_observe_20261111_220404_103327", "forward_hold_observe_20261111_220908_830958")
DETECTOR = cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50), cv.aruco.DetectorParameters())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def image(path):
    frame = cv.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("Cannot decode saved image: " + str(path))
    return frame


def corners(path):
    cs, ids, _ = DETECTOR.detectMarkers(image(path))
    return {} if ids is None else {int(i): c.reshape(4, 2) for c, i in zip(cs, ids.ravel())}


def angle(before, after):
    B, E = np.asarray(before), np.asarray(after)
    rv = cv.Rodrigues(E @ B.T)[0].ravel()
    theta = float(np.linalg.norm(rv))
    vector = rv * (math.sin(theta / 2) / theta) if theta > 1e-12 else np.zeros(3)
    up = -B[:, 1]
    twist = math.degrees(2 * math.atan2(float(vector @ up), math.cos(theta / 2)))
    return {"twist_about_initial_up_deg": (twist + 180) % 360 - 180,
            "total_rotation_deg": math.degrees(theta),
            "up_axis_change_deg": math.degrees(math.acos(float(np.clip(B[:, 1] @ E[:, 1], -1, 1))))}


def compare_pose(a, b):
    accepted = a["pose"]["full_orientation_observable"] and b["pose"]["full_orientation_observable"]
    result = {"before": a["image"], "after": b["image"], "accepted": accepted,
              "frame": "camera", "approximate_only": True}
    if accepted:
        result.update(angle(a["pose"]["R_cube_camera"], b["pose"]["R_cube_camera"]))
    return result


def summarize(name, bad_placement, K, D):
    folder = ROOT / name
    source = folder / "result.json"
    raw = json.loads(source.read_bytes())
    samples = raw.get("samples", [])
    phases = {}
    for phase in dict.fromkeys(s["phase"] for s in samples):
        group = [s for s in samples if s["phase"] == phase]
        phases[phase] = {"samples": len(group), "first_t": group[0]["t"], "last_t": group[-1]["t"]}
    hold = [s for s in samples if s["phase"] == "stage_8_horizontal_hold"]
    report = {"run": name, "source_result_sha256": sha(source),
              "user_reported_bad_placement": bad_placement,
              "excluded_from_correct_placement_task_assessment": bad_placement,
              "status": raw.get("status"), "error": raw.get("error"), "failed_phase": raw.get("failed_phase"),
              "all_torque_off_verified": raw.get("all_torque_off_verified"), "settings_restored": raw.get("settings_restored"),
              "cube_support_confirmed_before_off": raw.get("cube_support_confirmed_before_off"),
              "cube_removal_after_exit_observed": False,
              "post_OFF_reference_stationary": raw.get("final_off_reference_valid"),
              "post_OFF_sag_is_not_an_ON_hold_failure": True,
              "sample_count": len(samples), "sample_fields": list(samples[0]) if samples else [],
              "phase_ranges": phases,
              "minimum_voltage_v": min(min(s["voltage_v"]) for s in samples),
              "peak_absolute_current_ma": max(max(abs(v) for v in s["current_ma"]) for s in samples),
              "maximum_temperature_c": max(max(s["temperature_c"]) for s in samples),
              "maximum_tracking_rad": max(max(s["tracking_rad"]) for s in samples),
              "read_failure_count": len(raw.get("read_failures", [])),
              "read_failures": raw.get("read_failures", []),
              "hold_samples": len(hold),
              "hold_sample_time_span_s": hold[-1]["t"] - hold[0]["t"] if hold else None,
              "hold_goals_fixed": bool(hold) and all(s["goal"] == hold[0]["goal"] for s in hold),
              "assistant_motor_writes": 0, "live_devices_opened": False,
              "physical_exact90_certified": False, "physical_grip_force_measured": False,
              "new_two_turn_cycle_tested": False, "named_photos": []}
    if hold:
        actual = np.asarray([s["actual"] for s in hold])
        report["hold_joint_actual_peak_to_peak_ticks_by_id"] = np.ptp(actual, axis=0).tolist()
        report["hold_maximum_tracking_rad"] = max(max(s["tracking_rad"]) for s in hold)
        report["hold_health_flags_normal"] = all(all(v == 0 for v in s["error"]) and all(v == 1 for v in s["torque"]) for s in hold)
    for filename in ("cube_before_grasp.jpg", "cube_grasp_hold.jpg", "cube_horizontal_turn_hold.jpg", "cube_after_fixed_hold.jpg"):
        path = folder / filename
        found = corners(path)
        report["named_photos"].append({"image": filename, "sha256": sha(path), "detected_ids": sorted(found),
            "marker_corners_px": {str(i): p.tolist() for i, p in found.items()},
            "pose": estimate(found, K, D, marker_m=.0107)})
    photos = report["named_photos"]
    report["named_pose_comparisons"] = [compare_pose(photos[i], photos[-1]) for i in range(3)]
    frames_path = folder / "camera/frames.json"
    metadata = json.loads(frames_path.read_bytes())
    frames = metadata["frames"]
    enabled = raw["enabled_at"]
    report["frame_metadata_sha256"] = sha(frames_path)
    report["saved_frame_count"] = len(frames)
    report["saved_frame_phase_counts"] = dict(Counter(f["phase"] for f in frames))
    report["camera_recording"] = raw.get("camera_recording")
    # Choose unaltered saved source frames at actual motor-phase times.
    times = np.asarray([f["camera_read_monotonic"] - enabled for f in frames])
    points = [("grasp_hold", phases["stage_6_grasp_hold"]["first_t"])]
    turn = phases["stage_7_horizontal_yaw"]
    points.extend((f"turn_{n}", float(t)) for n, t in enumerate(np.linspace(turn["first_t"], turn["last_t"], 7)))
    points.extend((f"hold_{n}", float(t)) for n, t in enumerate(np.linspace(hold[0]["t"], hold[-1]["t"], 5)))
    selected = []
    for label, t in points:
        index = int(np.argmin(abs(times - t)))
        f = frames[index]
        selected.append({"label": label, "image": (folder / "camera" / f["image"]).relative_to(ROOT).as_posix(),
                         "phase": f["phase"], "t": float(times[index]), "target_t": t})
    report["selected_source_frames"] = selected
    if not bad_placement and hold:
        hold_frames = [f for f in frames if hold[0]["t"] <= f["camera_read_monotonic"] - enabled <= hold[-1]["t"]]
        observations = []
        for f in hold_frames:
            found = corners(folder / "camera" / f["image"])
            row = {"image": f["image"], "t": f["camera_read_monotonic"] - enabled,
                   "detected_ids": sorted(found), "centers_px": {str(i): p.mean(0).tolist() for i, p in found.items()}}
            row["pose"] = estimate(found, K, D, marker_m=.0107)
            observations.append(row)
        report["hold_saved_frame_observations"] = observations
        good = [r for r in observations if r["pose"]["full_orientation_observable"]]
        stats = {"saved_frames_in_actual_hold_window": len(hold_frames),
                 "frames_with_any_cube_marker": sum(any(i in range(6) for i in r["detected_ids"]) for r in observations),
                 "accepted_multi_face_pose_frames": len(good), "image_units_are_pixels_not_mm": True}
        for ident in (0, 3, 5, 40, 41):
            values = [r["centers_px"][str(ident)] for r in observations if str(ident) in r["centers_px"]]
            if values:
                a = np.asarray(values)
                stats[f"marker{ident}_center_range_px"] = np.ptp(a, axis=0).tolist()
                stats[f"marker{ident}_frames"] = len(values)
        relative_centers = [np.asarray(r["centers_px"]["0"]) - np.asarray(r["centers_px"]["41"])
                            for r in observations if "0" in r["centers_px"] and "41" in r["centers_px"]]
        if relative_centers:
            stats["simultaneous_white0_wrist41_frames"] = len(relative_centers)
            stats["white0_minus_wrist41_center_range_px"] = np.ptp(relative_centers, axis=0).tolist()
        if good:
            reference = good[0]["pose"]["R_cube_camera"]
            deviations = [angle(reference, r["pose"]["R_cube_camera"])["total_rotation_deg"] for r in good]
            stats["maximum_approximate_camera_pose_change_from_first_deg"] = max(deviations)
        report["hold_image_statistics"] = stats
        sim_ids = (8, 6, 7, 5, 12, 10, 11, 9, 16, 14, 15, 13, 4, 3, 2, 1)
        reference = {"status": "measured_first_turn_hold_reference_not_a_new_motion_goal",
            "source": source.relative_to(ROOT).as_posix(), "source_result_sha256": report["source_result_sha256"],
            "phase": "stage_8_horizontal_hold", "sample_t": hold[-1]["t"],
            "motor_ids": list(range(1, 17)), "actual_ticks": hold[-1]["actual"],
            "goal_ticks": hold[-1]["goal"], "sim_motor_ids": list(sim_ids),
            "actual_qsim_rad": [float((hold[-1]["actual"][i - 1] - 2048) * 2 * math.pi / 4096) for i in sim_ids],
            "cube_3d_pose_registered": False, "cube_velocity_observed": False, "contact_force_observed": False,
            "full_mujoco_state_reconstructed": False, "directly_execute_as_new_target": False,
            "new_second_turn_executable_ready": False}
        (OUT / "actual_first_turn_end_reference.json").write_text(json.dumps(reference, indent=2) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inspect", action="store_true", help="Read the compact saved review only")
    args = parser.parse_args()
    if args.inspect:
        report = json.loads((OUT / "run2.json").read_bytes())
        raw = json.loads((ROOT / RUNS[1] / "result.json").read_bytes())
        print(json.dumps({"selected_source_frames": report["selected_source_frames"],
                          "named_photo_fits": [{"image": r["image"], "ids": r["detected_ids"], "pose_status": r["pose"]["status"], "rms_px": r["pose"].get("reprojection_rms_px")} for r in report["named_photos"]],
                          "read_failures": raw.get("read_failures"),
                          "recovery_fields": [k for k in raw if "recover" in k or "failure" in k]}, ensure_ascii=False))
        return
    OUT.mkdir(parents=True, exist_ok=True)
    cal = np.load(ROOT / "camera1_720.npz")
    reports = []
    for i, name in enumerate(RUNS):
        report = summarize(name, i == 0, cal["K"], cal["dist"])
        (OUT / ("run1.json" if i == 0 else "run2.json")).write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        reports.append(report)
        print(json.dumps({k: report[k] for k in ("run", "status", "error", "all_torque_off_verified", "settings_restored", "sample_count", "hold_samples", "hold_sample_time_span_s", "hold_goals_fixed", "hold_health_flags_normal", "hold_maximum_tracking_rad", "hold_joint_actual_peak_to_peak_ticks_by_id", "minimum_voltage_v", "peak_absolute_current_ma", "maximum_temperature_c", "maximum_tracking_rad", "read_failure_count", "saved_frame_count", "named_pose_comparisons")}, ensure_ascii=False), flush=True)
        if "hold_image_statistics" in report:
            print(json.dumps(report["hold_image_statistics"], ensure_ascii=False), flush=True)
    lines = ["# 2026-11-11 첫 회전·유지 실물 결과 검토", "", "첫 시행은 사용자가 배치 오류를 알렸으므로 올바른 배치의 동작 평가에서 제외한다. 원본은 보존한다. 두 시행 모두 실제 사용자 실행 결과다.", "", "두 번째 시행에서는 초록 윗면을 유지한 채 주황 앞면이 흰 앞면으로 바뀌는 큐브 전체 회전을 사진과 선정 프레임에서 확인했다. 실제 유지 구간134샘플의 시각 범위는 약10.094초이고, 그 사이 저장77프레임 모두에서 흰면 마커ID0이 검출됐다. 유지 시작·끝의 선택 프레임에서도 큐브가 손 위에 남아 있다. 프로그램 정상 종료와 토크OFF·설정 복구도 확인했다. 이는 첫 회전→짧은 유지의 실물 관찰이며 다음 회전과 윗단 작업은 아직 시험하지 않았다. 관찰 범위는 [visual_review.json](visual_review.json), 실제 종료 관절 참고는 [actual_first_turn_end_reference.json](actual_first_turn_end_reference.json)에 있다.", ""]
    for i, report in enumerate(reports):
        lines += [f"## 시행 {i + 1}: {report['run']}", "", f"실행 상태: `{report['status']}`, 오류: `{report['error']}`. 토크OFF 확인: {report['all_torque_off_verified']}, 설정 복구: {report['settings_restored']}.", "", f"고정 목표 유지 {report['hold_samples']}샘플, 실제 샘플 시각 범위 {report['hold_sample_time_span_s']:.3f}초. 전압 최저 {report['minimum_voltage_v']}V, 최대 절대 전류 {report['peak_absolute_current_ma']}mA, 최대 추종오차 {report['maximum_tracking_rad']:.6f}rad.", ""]
        for filename in ("cube_before_grasp.jpg", "cube_after_fixed_hold.jpg"):
            lines.append(f"[{filename}](../../{report['run']}/{filename})")
        lines.append("")
        for frame in report["selected_source_frames"]:
            lines.append(f"- [{frame['label']}: {frame['t']:.2f}s](../../{frame['image']}) ({frame['phase']})")
        lines.append("")
    lines += ["사진 기반 각도는 저장된 카메라 내부 파라미터와 큐브55mm·마커10.7mm 가정의 근사값이다. 이번 다면 마커 자세 적합은 수용되지 않았으므로 숫자 각도를 만들지 않는다. 마커 장착·로봇 좌표계 정합 및 실제 힘 측정이 없으므로 정밀90도·파지력·윗단 작업 하중·새 두 회전 연결 성공을 인증하지 않는다. 종료 후OFF 처짐은 구동 중 유지 실패로 세지 않는다.", "", "장치를 새로 열거나 모터에 출력하지 않았다. 원본 사진·로그는 변경하지 않았다."]
    (OUT / "REVIEW.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
