"""Project tentative model palm geoms directly from ID40 in an empty-hand photo."""

import json
from pathlib import Path

import cv2 as cv
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
FRAME = ROOT / "leap_palm_photo_20261008_201116_472718.png"


def main():
    with np.load(ROOT / "camera1_720.npz") as f:
        K, dist = f["K"].copy(), f["dist"].copy()
    frame, corners, ids, _ = detect(FRAME, cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50))
    R_cm, t_cm, reproj = marker_pose(corners, ids, 40, .0282, K, dist)
    reg = candidate()
    R_wm = np.asarray(reg["hand_to_world_candidate"])
    g_m = np.asarray(reg["grasp_site_in_marker_m_candidate"])
    marker_world = np.array([.11, 0, .03]) - R_wm @ g_m
    boxes = json.loads((ROOT / "sac_palm_box_corners_result.json").read_text(encoding="utf-8"))
    colors = {"palm_collision_6": (255, 255, 0),
              "palm_collision_7": (255, 0, 255),
              "palm_collision_8": (0, 255, 255)}
    projected = {}
    for name, box in boxes.items():
        world = np.asarray(box["corners_world_m"])
        marker = (R_wm.T @ (world - marker_world).T).T
        camera = (R_cm @ marker.T).T + t_cm
        pixels, _ = cv.projectPoints(camera, np.zeros(3), np.zeros(3), K, dist)
        pixels = pixels.reshape(-1, 2)
        for i in range(8):
            for mask in (1, 2, 4):
                j = i ^ mask
                if i < j:
                    cv.line(frame, tuple(np.rint(pixels[i]).astype(int)),
                            tuple(np.rint(pixels[j]).astype(int)), colors[name], 2)
        center = pixels.mean(axis=0)
        cv.putText(frame, name[-1], tuple(np.rint(center).astype(int)),
                   cv.FONT_HERSHEY_SIMPLEX, 1, colors[name], 2)
        projected[name] = {"center_px": center.tolist(),
                           "bounds_xyxy": [float(pixels[:, 0].min()), float(pixels[:, 1].min()),
                                           float(pixels[:, 0].max()), float(pixels[:, 1].max())]}
    (ROOT / "sac_direct_id40_palm_overlay.png").write_bytes(cv.imencode(".png", frame)[1].tobytes())
    report = {"source": str(FRAME), "detected_ids": ids, "id40_reprojection_px": reproj,
              "projected_boxes": projected, "registration_verified": False,
              "hardware_authorized": False,
              "caveat": "This uses the provisional 47/50/14mm marker-to-model transform; the box projections are hypotheses, not measurements."}
    (ROOT / "sac_direct_id40_palm_overlay_result.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
