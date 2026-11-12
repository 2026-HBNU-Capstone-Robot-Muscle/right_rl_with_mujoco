"""Project measured-stage-0 model collision boxes into the saved empty-hand view."""

import json
from pathlib import Path

import cv2 as cv
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
FRAME = ROOT / "sequence_vision_20261006_203648_332236" / "anchor_empty_stage0.png"


def main():
    with np.load(ROOT / "camera1_720.npz") as f:
        K, dist = f["K"].copy(), f["dist"].copy()
    frame, corners, ids, _ = detect(FRAME, cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50))
    R_cm, t_cm, reproj = marker_pose(corners, ids, 40, .0282, K, dist)
    reg = candidate()
    R_wm = np.asarray(reg["hand_to_world_candidate"])
    g_m = np.asarray(reg["grasp_site_in_marker_m_candidate"])
    marker_world = np.array([.11, 0, .03]) - R_wm @ g_m
    source = json.loads((ROOT / "sac_stage0_geom_export_result.json").read_text(encoding="utf-8"))
    wanted = {"palm_collision_6", "palm_collision_7", "palm_collision_8"}
    wanted.update(name for name in source["geoms"] if name.startswith(
        ("if_bs_collision", "mf_bs_collision", "rf_bs_collision", "th_bs_collision",
         "if_px_collision", "mf_px_collision", "rf_px_collision", "th_px_collision")))
    projected = {}
    for name, box in source["geoms"].items():
        if name not in wanted:
            continue
        world = np.asarray(box["corners_world_m"])
        marker = (R_wm.T @ (world - marker_world).T).T
        camera = (R_cm @ marker.T).T + t_cm
        pixels, _ = cv.projectPoints(camera, np.zeros(3), np.zeros(3), K, dist)
        pixels = pixels.reshape(-1, 2)
        if name.startswith("palm_"):
            color = (0, 255, 255)
        elif name.startswith("th_"):
            color = (0, 0, 255)
        elif name.startswith("if_"):
            color = (255, 255, 0)
        elif name.startswith("mf_"):
            color = (0, 255, 0)
        else:
            color = (255, 0, 255)
        for i in range(8):
            for mask in (1, 2, 4):
                j = i ^ mask
                if i < j:
                    cv.line(frame, tuple(np.rint(pixels[i]).astype(int)),
                            tuple(np.rint(pixels[j]).astype(int)), color, 1)
        center = pixels.mean(axis=0)
        label = name.replace("_collision", "").replace("_1", "")
        cv.putText(frame, label, tuple(np.rint(center).astype(int)),
                   cv.FONT_HERSHEY_SIMPLEX, .4, color, 1)
        projected[name] = {"center_px": center.tolist(),
                           "bounds_xyxy": [float(pixels[:, 0].min()), float(pixels[:, 1].min()),
                                           float(pixels[:, 0].max()), float(pixels[:, 1].max())]}
    (ROOT / "sac_stage0_geom_overlay.png").write_bytes(cv.imencode(".png", frame)[1].tobytes())
    result = {"source": str(FRAME), "id40_reprojection_px": reproj,
              "projected_geoms": projected, "registration_verified": False,
              "hardware_authorized": False,
              "caveat": "Provisional marker-to-model transform and collision boxes; qualitative silhouette audit only."}
    (ROOT / "sac_stage0_geom_overlay_result.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"Projected {len(projected)} boxes; ID40 reprojection {reproj:.2f}px", flush=True)


if __name__ == "__main__":
    main()
