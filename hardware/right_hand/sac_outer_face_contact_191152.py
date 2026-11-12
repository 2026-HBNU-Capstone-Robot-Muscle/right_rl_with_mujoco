"""Offline contact test for cuboid-rim pose; uses only archived images/motor logs."""

import json
import sys
import argparse
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import probe


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent / "work" / "Leap_Test"
sys.path.insert(0, str(PROJECT / "deploy"))
from leap_hw import hw_to_sim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--registration-file", default="sac_outer_face_registration_191152_result.json")
    parser.add_argument("--output-file", default="sac_outer_face_contact_191152_result.json")
    args = parser.parse_args()
    cfg = registry.get_default_config("LeapCubeFaceUp")
    cfg.impl = "jax"
    env = registry.load("LeapCubeFaceUp", config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube")
    trial = json.loads((ROOT / "leap_sequence_flip_20261008_191152_267924.json").read_text(encoding="utf-8"))
    ticks = [r for r in trial["samples"] if r["phase"] == "place_cube"][-1]["actual"]
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)

    source = json.loads((ROOT / args.registration_file).read_text(encoding="utf-8"))
    rows = []
    for item in source["cases"]:
        cube_world = np.asarray(item["cube_center_in_sim_world_candidate_m"])
        cube_quat = np.asarray(item["cube_quat_world_candidate"])
        q[-7:-4] = cube_world
        q[-4:] = cube_quat
        rows.append({
            **item,
            **probe(model, q, cube_geom),
        })
    result = {"status": "offline_outer_face_contact_diagnostic", "hardware_authorized": False,
              "source": source["source"], "cases": rows,
              "caveat": "Outer rim hand-read and ID40/ID41 bridge crosses sessions; this is not a verified physical reset."}
    (ROOT / args.output_file).write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"cases": [{k: v for k, v in r.items() if k not in
                                   ("cube_quat_world_candidate", "cube_center_in_hand_marker_m")}
                                  for r in rows]}, indent=2))


if __name__ == "__main__":
    main()
