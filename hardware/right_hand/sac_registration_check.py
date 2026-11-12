"""Inspect the saved candidate marker-to-model transform without any devices."""

import json
from pathlib import Path

import numpy as np

from hand_registration_geometry import candidate

ROOT = Path(__file__).resolve().parent


def main():
    registration = candidate()
    R = np.asarray(registration["hand_to_world_candidate"])
    grasp_in_marker = np.asarray(registration["grasp_site_in_marker_m_candidate"])
    grasp_world = np.array([0.11, 0.0, 0.03])
    marker_world = grasp_world - R @ grasp_in_marker
    p9_world = marker_world + R @ np.asarray(registration["motor_axis_landmarks_in_marker_m"]["9"])
    images = json.loads((ROOT / "sac_success_cube_poses.json").read_text(encoding="utf-8"))
    cube_world = {
        image["image"]: (marker_world + R @ np.asarray(image["cube_minus_hand_marker_m"])).tolist()
        for image in images if image["cube_minus_hand_marker_m"] is not None
    }
    report = {
        "grasp_site_world_m": grasp_world.tolist(),
        "marker_center_world_candidate_m": marker_world.tolist(),
        "middle_motor_axis_world_candidate_m": p9_world.tolist(),
        "cube_world_candidate_by_image_m": cube_world,
        "first_cube_minus_default_reset_m": (
            np.asarray(next(iter(cube_world.values()))) - np.array([0.0902132, 0.0004209, 0.0590793])
        ).tolist(),
        "status": "candidate only; contact and external marker mounting are not validated",
    }
    (ROOT / "sac_registration_check_result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
