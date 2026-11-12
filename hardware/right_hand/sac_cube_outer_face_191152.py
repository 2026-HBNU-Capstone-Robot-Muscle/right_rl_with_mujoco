"""Offline cuboid-face PnP from visible outer plastic rim plus ID5 on 19:11 frame."""

import json
import sys
from pathlib import Path

import cv2 as cv
import numpy as np


ROOT = Path(__file__).resolve().parent
FRAME = ROOT / "sequence_vision_20261008_191152_096085" / "placement_at_space.png"
sys.path.insert(0, str(ROOT.parent / "work" / "Leap_Test" / "vision" / "rubik_center"))
import cube_board


def main():
    with np.load(ROOT / "camera1_720.npz") as f:
        K, dist = f["K"].copy(), f["dist"].copy()
    rotations = tuple(json.loads((ROOT / "cube_face_rot_result.json").read_text(encoding="utf-8"))["face_rot"])
    board, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                         layout="centers", face_rot=rotations)
    inner_obj = np.asarray(board[5], dtype=np.float32)
    face_n = np.asarray(cube_board.FACES[5][0], dtype=float)
    face_center = face_n * (54.5 / 2000)
    outer_obj = (face_center + (inner_obj - face_center) * (54.5 / 10.6)).astype(np.float32)
    recovered = json.loads((ROOT / "sac_recover_191152_markers_result.json").read_text(encoding="utf-8"))
    found = [r for r in recovered["detections"] if r["image"] == FRAME.name and r["id"] == 5]
    found.sort(key=lambda r: (abs(r["scale"] - 1.0),
                               r["variant"] not in ("clahe", "raw")))
    if not found:
        raise RuntimeError("No recovered ID5 corners")
    inner_img = np.asarray(found[0]["corners_px"], dtype=np.float32)
    # Visible outer *plastic* top-face rim, manually read from the saved raw
    # image. Order corresponds to ID5 ArUco corners: bottom-right,
    # bottom-left, top-left, top-right. These are approximate (about +/-3 px).
    outer_img = np.asarray([[718, 260], [421, 259], [438, 76], [715, 78]], np.float32)
    marker_ok, marker_rv, marker_tv = cv.solvePnP(inner_obj, inner_img, K, dist)
    if not marker_ok:
        raise RuntimeError("Marker solve failed")
    marker_rv = marker_rv.copy()
    marker_tv = marker_tv.copy()
    rows = []
    center = outer_img.mean(axis=0)
    for pixel_scale in (.98, 1.0, 1.02):
        rim = center + (outer_img - center) * pixel_scale
        obj = np.concatenate((inner_obj, outer_obj))
        img = np.concatenate((inner_img, rim))
        ok, rv, tv = cv.solvePnP(obj, img, K, dist,
                                 marker_rv.copy(), marker_tv.copy(), True)
        if not ok:
            raise RuntimeError("Combined solve failed")
        pi, _ = cv.projectPoints(inner_obj, rv, tv, K, dist)
        po, _ = cv.projectPoints(outer_obj, rv, tv, K, dist)
        inner_error = float(np.mean(np.linalg.norm(pi.reshape(4, 2) - inner_img, axis=1)))
        outer_error = float(np.mean(np.linalg.norm(po.reshape(4, 2) - rim, axis=1)))
        rows.append({
            "rim_pixel_scale": pixel_scale,
            "center_camera_m": tv.reshape(3).tolist(),
            "rotation_cube_to_camera": cv.Rodrigues(rv)[0].tolist(),
            "center_shift_from_marker_only_camera_mm": (1000 * (tv - marker_tv).reshape(3)).tolist(),
            "marker_reprojection_px": inner_error,
            "rim_reprojection_px": outer_error,
            "rotation_delta_deg": float(np.degrees(np.arccos(np.clip(
                (np.trace(cv.Rodrigues(marker_rv)[0].T @ cv.Rodrigues(rv)[0]) - 1) / 2,
                -1, 1)))),
            "projected_inner_px": pi.reshape(4, 2).tolist(),
            "projected_rim_px": po.reshape(4, 2).tolist(),
        })
    report = {"source": str(FRAME), "hardware_authorized": False,
              "cube_side_mm": 54.5, "id5_corners_px": inner_img.tolist(),
              "hand_read_rim_corners_px": outer_img.tolist(),
              "marker_only_center_camera_m": marker_tv.reshape(3).tolist(),
              "cases": rows,
              "limitation": "Rim corners are hand-read from one angled photo, plastic edge may be rounded/occluded; cross-session hand registration still provisional."}
    (ROOT / "sac_cube_outer_face_191152_result.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    frame = cv.imdecode(np.frombuffer(FRAME.read_bytes(), np.uint8), cv.IMREAD_COLOR)
    cv.polylines(frame, [np.int32(outer_img)], True, (0, 255, 255), 2)
    cv.polylines(frame, [np.int32(rows[1]["projected_rim_px"])], True, (0, 0, 255), 2)
    cv.polylines(frame, [np.int32(rows[1]["projected_inner_px"])], True, (255, 0, 255), 2)
    (ROOT / "sac_cube_outer_face_191152_overlay.png").write_bytes(cv.imencode(".png", frame)[1].tobytes())
    print(json.dumps({"marker_only_center_camera_m": report["marker_only_center_camera_m"],
                      "cases": [{k: v for k, v in row.items() if not k.startswith("projected_")}
                                for row in rows]}, indent=2))


if __name__ == "__main__":
    main()
