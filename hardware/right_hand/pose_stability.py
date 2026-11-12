"""Read-only 10-second hand/cube pose stability check with external webcam."""

import json
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from camera_calibrate import open_camera


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "work" / "Leap_Test" / "vision" / "rubik_center"))
import cube_board as cb  # noqa: E402
import track_cube as tc  # noqa: E402


def rotation_deviations(matrices, reference_mask=None):
    selected = matrices if reference_mask is None else matrices[reference_mask]
    if not len(selected):
        return None
    u, _, vt = np.linalg.svd(selected.mean(axis=0))
    reference = u @ vt
    if np.linalg.det(reference) < 0:
        u[:, -1] *= -1
        reference = u @ vt
    return np.degrees(np.arccos(np.clip(
        (np.einsum("ij,nij->n", reference, matrices) - 1.0) / 2.0, -1.0, 1.0)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hd', action='store_true')
    parser.add_argument('--mjpg', action='store_true', help='Require actual MJPG; apply codec after size and FPS')
    parser.add_argument('--profile', action='store_true', help='Start automatically and save separate timing diagnostics')
    args = parser.parse_args()
    if args.mjpg and not args.hd:
        parser.error('--mjpg requires --hd')
    calibration_path = ROOT / ('camera1_720.npz' if args.hd else 'camera1.npz')
    calibration = np.load(calibration_path)
    expected = (int(calibration['height']), int(calibration['width']))
    if args.hd and expected != (720, 1280):
        raise SystemExit('HD calibration must be 1280x720')
    rotations = tuple(json.loads((ROOT / "cube_face_rot_result.json").read_text())["face_rot"])
    points, ids = cb.object_points(cube_mm=54.5, marker_mm=10.6,
                                    layout="centers", face_rot=rotations)
    dictionary, _, detector = tc.build(marker_mm=10.6, grid=1)
    board = cv2.aruco.Board(points, dictionary, ids)
    if args.hd:
        from camera_720_check import open_hd
        cap, metadata, _ = open_hd(cv2, 1, require_mjpg=args.mjpg)
        actual = tuple(reversed(metadata['actual_size']))
    else:
        cap, first = open_camera(1)
        actual = first.shape[:2]
    if actual != expected:
        cap.release()
        raise SystemExit(f"Camera resolution does not match {calibration_path.name}")
    print(f"Using {calibration_path.name}: {expected[1]}x{expected[0]}; NO MOTORS")
    print("Keep hand and cube STILL with ID 40 and 2 cube faces visible.")
    print("Automatic 10-second timing check; Q exits." if args.profile else
          "SPACE begins a 10-second read-only pose check; Q exits.")
    started = time.monotonic() if args.profile else None
    total = 0
    failed_reads = 0
    positions, orientations, errors, samples = [], [], [], []
    timings = {'capture_read_ms': [], 'vision_step_ms': [], 'display_wait_ms': [], 'loop_ms': []}
    measured_elapsed = 0.0
    try:
        while True:
            if started is not None and time.monotonic() - started >= 10.0:
                break
            loop_start = time.perf_counter()
            measuring = started is not None
            ok, frame = cap.read()
            read_end = time.perf_counter()
            if not ok or frame is None:
                failed_reads += int(started is not None)
                if cv2.waitKey(10) & 255 in (ord('q'), 27):
                    break
                continue
            if frame.shape[:2] != expected:
                raise RuntimeError('Camera resolution changed during measurement')
            result = tc.step(frame, board, detector, dictionary,
                             calibration["K"], calibration["dist"])
            vision_end = time.perf_counter()
            valid = result is not None and "palm_minus_cube" in result
            if started is not None:
                total += 1
                if valid:
                    positions.append(result["palm_minus_cube"].copy())
                    orientations.append(result["R_cube_world"].copy())
                    errors.append(float(result.get("reproj", np.nan)))
                    samples.append({
                        "cube_markers": int(result["n_cube_markers"]),
                        "hand_camera_m": result["t_hand"].tolist(),
                        "cube_camera_m": result["t_cube"].tolist(),
                        "hand_minus_cube_m": result["palm_minus_cube"].tolist(),
                        "cube_rotation_in_uncalibrated_hand_frame": result["R_cube_world"].tolist(),
                        "reprojection_px": float(result.get("reproj", np.nan)),
                    })
                remaining = max(0.0, 10.0 - (time.monotonic() - started))
                n_cube = result.get('n_cube_markers', 0) if result else 0
                status = f"{remaining:.1f}s  pose {'YES' if valid else 'NO'}  cube markers:{n_cube}  {len(positions)}/{total}"
            else:
                status = f"pose {'YES' if valid else 'NO'}  SPACE start"
            display_start = time.perf_counter()
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0) if valid else (0, 0, 255), 2)
            cv2.imshow("LEAP pose stability - webcam 1", frame)
            key = cv2.waitKey(1) & 0xFF
            loop_end = time.perf_counter()
            if measuring:
                timings['capture_read_ms'].append((read_end - loop_start) * 1000)
                timings['vision_step_ms'].append((vision_end - read_end) * 1000)
                timings['display_wait_ms'].append((loop_end - display_start) * 1000)
                timings['loop_ms'].append((loop_end - loop_start) * 1000)
            if key in (ord("q"), 27):
                break
            if key == 32 and started is None:
                started = time.monotonic()
                print("Measuring for 10 seconds...", flush=True)
            if started is not None and time.monotonic() - started >= 10.0:
                break
    finally:
        measured_elapsed = time.monotonic() - started if started is not None else 0.0
        cap.release()
        cv2.destroyAllWindows()

    if not positions:
        print("No simultaneous hand/cube poses were measured")
        return 1
    p = np.asarray(positions)
    deviation_mm = np.linalg.norm(p - np.median(p, axis=0), axis=1) * 1000.0
    two_mask = np.array([s["cube_markers"] >= 2 for s in samples], bool)
    reference_p = np.median(p[two_mask], axis=0) if two_mask.any() else np.median(p, axis=0)
    reference_deviation_mm = np.linalg.norm(p - reference_p, axis=1) * 1000.0
    hand_p = np.asarray([s["hand_camera_m"] for s in samples])
    cube_p = np.asarray([s["cube_camera_m"] for s in samples])
    matrices = np.asarray(orientations)
    angles_deg = rotation_deviations(matrices)
    two_reference_angles = rotation_deviations(matrices, two_mask)
    elapsed = measured_elapsed
    report = {
        "camera": 1, "face_rot": rotations, "frames": total, "valid": len(p),
        "calibration_file": calibration_path.name,
        "capture": metadata if args.hd else None,
        "resolution": [expected[1], expected[0]],
        "elapsed_seconds": round(elapsed, 2), "processed_fps": round(total / elapsed, 1),
        "read_failures": failed_reads,
        "two_marker_percent_of_all_frames": round(100.0 * int(two_mask.sum()) / total, 1),
        "timing_ms": {k: {'mean': round(float(np.mean(v)), 2),
                          'p95': round(float(np.percentile(v, 95)), 2)} if v else None
                      for k, v in timings.items()},
        "timing_note": "read includes driver wait/transfer/conversion; not a sensor exposure measurement",
        "valid_percent": round(100.0 * len(p) / total, 1),
        "position_jitter_median_mm": round(float(np.median(deviation_mm)), 2),
        "position_jitter_p95_mm": round(float(np.percentile(deviation_mm, 95)), 2),
        "cube_markers_ge2_frames": int(two_mask.sum()),
        "cube_markers_1_frames": int((~two_mask).sum()),
        "position_p95_mm_ge2": round(float(np.percentile(reference_deviation_mm[two_mask], 95)), 2)
        if two_mask.any() else None,
        "position_p95_mm_1": round(float(np.percentile(reference_deviation_mm[~two_mask], 95)), 2)
        if (~two_mask).any() else None,
        "hand_camera_p95_mm": round(float(np.percentile(
            np.linalg.norm(hand_p - np.median(hand_p, axis=0), axis=1) * 1000, 95)), 2),
        "cube_camera_p95_mm": round(float(np.percentile(
            np.linalg.norm(cube_p - np.median(cube_p, axis=0), axis=1) * 1000, 95)), 2),
        "orientation_jitter_median_deg": round(float(np.median(angles_deg)), 2),
        "orientation_jitter_p95_deg": round(float(np.percentile(angles_deg, 95)), 2),
        "orientation_p95_deg_ge2": round(float(np.percentile(two_reference_angles[two_mask], 95)), 2)
        if two_reference_angles is not None else None,
        "cube_reprojection_mean_px": round(float(np.nanmean(errors)), 2),
        "marker_to_palm_offset_calibrated": False,
    }
    prefix = 'pose_stability_720' if args.hd else 'pose_stability'
    if args.mjpg:
        prefix += '_mjpg'
    if args.profile:
        prefix += '_timing'
    output = ROOT / f"{prefix}_result.json"
    output.write_text(json.dumps(report, indent=2))
    (ROOT / f"{prefix}_samples.json").write_text(json.dumps(samples))
    print(f"Saved: {output}")
    print(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
