"""Search cube placements supported by the existing LEAP collision model.

Offline MuJoCo only: no camera, serial port, or motor command is opened.
This is a geometry diagnostic, not a calibrated real-to-sim transform.
"""

import json
import argparse
from pathlib import Path
import sys

import jax
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_cube_model import set_cube_physics

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "work" / "Leap_Test" / "deploy"))
from leap_hw import hw_to_sim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--real-cube", action="store_true", help="Use the measured 54.5 mm cube collision size in memory")
    parser.add_argument("--cube-mass-g", type=float, help="Approximate physical cube mass; requires --real-cube")
    parser.add_argument("--near-measured", action="store_true", help="Search a local grid around the saved camera position")
    parser.add_argument("--sequence-placement", action="store_true", help="Use same-run PLACE/SPACE frame with one cube marker")
    parser.add_argument("--staggered-placement", action="store_true", help="Use ID40-before / two-cube-marker-after provisional pose")
    parser.add_argument("--anchor-placement", action="store_true", help="Use ID40+41 empty / ID41+cube placed two-frame pose")
    parser.add_argument("--anchor-preflight", action="store_true", help="Use frontal preflight ID40+41 image for anchor calibration")
    parser.add_argument("--placement-json", type=Path,
                        help="Offline diagnostic from a specified saved registration JSON")
    parser.add_argument("--actual-stage0", action="store_true",
                        help="Use measured stage-0 motor angles from the saved anchor trial")
    parser.add_argument("--actual-placement", action="store_true",
                        help="Use measured motor angles nearest the saved cube-placement image")
    args = parser.parse_args()
    if args.actual_stage0 and args.actual_placement:
        parser.error("Select at most one measured joint source")
    if args.cube_mass_g is not None and (not args.real_cube or args.cube_mass_g <= 0):
        parser.error("--cube-mass-g requires --real-cube and a positive mass")
    config = registry.get_default_config("LeapCubeFaceUp")
    config.impl = "jax"
    env = registry.load("LeapCubeFaceUp", config=config)
    reset = env.reset(jax.random.PRNGKey(0))
    model = env.mj_model
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube")
    original_half_extent_m = model.geom_size[cube_geom_id, :3].copy()
    cube_physics = None
    if args.real_cube:
        if args.cube_mass_g is None:
            model.geom_size[cube_geom_id, :3] = 0.0545 / 2
            mujoco.mj_setConst(model, mujoco.MjData(model))
        else:
            cube_physics = set_cube_physics(model, mass_kg=args.cube_mass_g / 1000)
    body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
                  for i in range(model.nbody)]
    cube_body_ids = {i for i, name in enumerate(body_names) if "cube" in name.lower()}
    raw = json.loads((ROOT / "sequence_flip_poses.json").read_text(encoding="utf-8"))["poses"][0]["raw_ticks"]
    if args.actual_stage0:
        trial = json.loads((ROOT / "leap_anchor_capture_result.json").read_text(encoding="utf-8"))
        if trial.get("status") != "placement_capture_complete" or not trial.get("all_torque_off_verified"):
            raise RuntimeError("Saved anchor trial did not complete with torque OFF verified")
        raw = trial["stage_end_tracking"]["stage_0"]["actual"]
    if args.actual_placement:
        audit = json.loads((ROOT / "sac_stage0_alignment_audit_result.json").read_text(encoding="utf-8"))
        raw = audit["loaded_actual_ticks"]
    hand = hw_to_sim(np.asarray(raw, dtype=float) * 2 * np.pi / 4096)
    initial = np.asarray(reset.data.qpos, dtype=float)
    measured = json.loads((ROOT / "sac_success_cube_poses.json").read_text(encoding="utf-8"))[0]
    camera_mapped = -np.asarray(measured["palm_minus_cube_m"], dtype=float)
    cube_quat = np.asarray(measured["cube_quat_world"], dtype=float)
    measured_start = np.array([0.11, 0.0, 0.03]) + camera_mapped
    if args.sequence_placement:
        placement = json.loads((ROOT / "sac_sequence_placement_pose_result.json").read_text(encoding="utf-8"))[-1]
        measured_start = np.asarray(placement["cube_center_in_sim_world_candidate_m"], dtype=float)
        cube_quat = np.asarray(placement["cube_quat_world_candidate"], dtype=float)
        camera_mapped = measured_start - np.array([0.11, 0.0, 0.03])
    if args.staggered_placement:
        placement = json.loads((ROOT / "sac_registration_staggered_result.json").read_text(encoding="utf-8"))
        measured_start = np.asarray(placement["cube_center_in_sim_world_candidate_m"], dtype=float)
        cube_quat = np.asarray(placement["cube_quat_world_candidate"], dtype=float)
        camera_mapped = measured_start - np.array([0.11, 0.0, 0.03])
    if args.anchor_placement:
        placement = json.loads((ROOT / "sac_registration_anchor_result.json").read_text(encoding="utf-8"))
        measured_start = np.asarray(placement["cube_center_in_sim_world_candidate_m"], dtype=float)
        cube_quat = np.asarray(placement["cube_quat_world_candidate"], dtype=float)
        camera_mapped = measured_start - np.array([0.11, 0.0, 0.03])
    if args.anchor_preflight:
        placement = json.loads((ROOT / "sac_registration_anchor_preflight_result.json").read_text(encoding="utf-8"))
        measured_start = np.asarray(placement["cube_center_in_sim_world_candidate_m"], dtype=float)
        cube_quat = np.asarray(placement["cube_quat_world_candidate"], dtype=float)
        camera_mapped = measured_start - np.array([0.11, 0.0, 0.03])
    if args.placement_json:
        placement = json.loads(args.placement_json.read_text(encoding="utf-8"))
        measured_start = np.asarray(placement["cube_center_in_sim_world_candidate_m"], dtype=float)
        cube_quat = np.asarray(placement["cube_quat_world_candidate"], dtype=float)
        camera_mapped = measured_start - np.array([0.11, 0.0, 0.03])

    measured_data = mujoco.MjData(model)
    measured_data.qpos[:] = initial
    measured_data.qpos[:16] = hand
    measured_data.qpos[-7:-4] = measured_start
    measured_data.qpos[-4:] = cube_quat
    measured_data.ctrl[:] = hand
    mujoco.mj_forward(model, measured_data)
    measured_initial_contacts = [c.dist for c in measured_data.contact
                                 if int(model.geom_bodyid[c.geom1]) in cube_body_ids
                                 or int(model.geom_bodyid[c.geom2]) in cube_body_ids]
    for _ in range(30):
        mujoco.mj_step(model, measured_data)
    measured_final_contacts = [c.dist for c in measured_data.contact
                               if int(model.geom_bodyid[c.geom1]) in cube_body_ids
                               or int(model.geom_bodyid[c.geom2]) in cube_body_ids]
    measured_contact_diagnostic = {
        "start_m": measured_start.tolist(),
        "initial_cube_contacts": len(measured_initial_contacts),
        "initial_penetration_m": float(min(measured_initial_contacts, default=0.0)),
        "end_m": measured_data.qpos[-7:-4].tolist(),
        "drift_m": float(np.linalg.norm(measured_data.qpos[-7:-4] - measured_start)),
        "final_cube_contacts": len(measured_final_contacts),
    }

    results = []
    xs = np.linspace(measured_start[0] - 0.025, measured_start[0] + 0.015, 9) if args.near_measured else np.linspace(0.025, 0.125, 11)
    ys = np.linspace(measured_start[1] - 0.020, measured_start[1] + 0.020, 9) if args.near_measured else np.linspace(-0.025, 0.025, 6)
    zs = np.linspace(measured_start[2] - 0.005, measured_start[2] + 0.045, 11) if args.near_measured else np.linspace(0.015, 0.085, 8)
    for x in xs:
        for y in ys:
            for z in zs:
                data = mujoco.MjData(model)
                data.qpos[:] = initial
                data.qpos[:16] = hand
                data.qpos[-7:-4] = [x, y, z]
                data.qpos[-4:] = cube_quat
                data.ctrl[:] = hand
                mujoco.mj_forward(model, data)
                cube_contacts = [c.dist for c in data.contact
                                 if int(model.geom_bodyid[c.geom1]) in cube_body_ids
                                 or int(model.geom_bodyid[c.geom2]) in cube_body_ids]
                initial_penetration_m = min(cube_contacts, default=0.0)
                initial_cube_contacts = len(cube_contacts)
                for _ in range(30):
                    mujoco.mj_step(model, data)
                final_cube_contacts = sum(
                    int(model.geom_bodyid[c.geom1]) in cube_body_ids
                    or int(model.geom_bodyid[c.geom2]) in cube_body_ids
                    for c in data.contact
                )
                final = data.qpos[-7:-4].copy()
                drift = float(np.linalg.norm(final - [x, y, z]))
                results.append({
                    "start_m": [float(x), float(y), float(z)],
                    "end_m": final.tolist(),
                    "drift_m": drift,
                    "initial_penetration_m": float(min(0.0, initial_penetration_m)),
                    "initial_cube_contacts": initial_cube_contacts,
                    "final_cube_contacts": final_cube_contacts,
                    "cube_held_above_palm": bool(final[2] > 0.015),
                })
    nonpenetrating = [r for r in results if r["cube_held_above_palm"]
                      and r["initial_penetration_m"] > -0.004]
    stable = [r for r in nonpenetrating if r["drift_m"] < 0.010
              and r["initial_cube_contacts"] and r["final_cube_contacts"]]
    nonpenetrating.sort(key=lambda r: r["drift_m"])
    contact_retained = [r for r in nonpenetrating
                        if r["initial_cube_contacts"] and r["final_cube_contacts"]]
    report = {
        "cube_collision_half_extent_m": model.geom_size[cube_geom_id, :3].tolist(),
        "original_cube_collision_half_extent_m": original_half_extent_m.tolist(),
        "real_cube_size_applied": args.real_cube,
        "near_measured_search": args.near_measured,
        "sequence_placement_source": args.sequence_placement,
        "staggered_placement_source": args.staggered_placement,
        "anchor_placement_source": args.anchor_placement,
        "anchor_preflight_source": args.anchor_preflight,
        "placement_json_source": str(args.placement_json) if args.placement_json else None,
        "actual_stage0_source": args.actual_stage0,
        "actual_placement_source": args.actual_placement,
        "cube_physics": cube_physics,
        "samples": len(results),
        "cube_bodies": [body_names[i] for i in sorted(cube_body_ids)],
        "stable_candidates": len(stable),
        "contact_retained_candidates": sum(
            bool(r["initial_cube_contacts"] and r["final_cube_contacts"])
            for r in nonpenetrating
        ),
        "nonpenetrating_retained_candidates": len(nonpenetrating),
        "camera_mapped_start_m": camera_mapped.tolist(),
        "measured_contact_diagnostic": measured_contact_diagnostic,
        "top_candidates": nonpenetrating[:15],
        "contact_retained_top_candidates": contact_retained[:15],
        "lowest_drift": sorted(results, key=lambda r: r["drift_m"])[:8],
        "highest_final_z": sorted(results, key=lambda r: r["end_m"][2], reverse=True)[:8],
        "caveat": "Collision-only support search; camera-to-model transform and learned-policy success remain unverified.",
    }
    output = ROOT / ("sac_fit_contact_real_cube_result.json" if args.real_cube else "sac_fit_contact_result.json")
    if args.cube_mass_g is not None:
        output = ROOT / f"sac_fit_contact_real_cube_{args.cube_mass_g:g}g_result.json"
    if args.near_measured:
        output = output.with_name(output.stem.replace("_result", "_near_measured_result") + output.suffix)
    if args.sequence_placement:
        output = output.with_name(output.stem.replace("_result", "_sequence_placement_result") + output.suffix)
    if args.staggered_placement:
        output = output.with_name(output.stem.replace("_result", "_staggered_placement_result") + output.suffix)
    if args.anchor_placement:
        output = output.with_name(output.stem.replace("_result", "_anchor_placement_result") + output.suffix)
    if args.anchor_preflight:
        output = output.with_name(output.stem.replace("_result", "_anchor_preflight_result") + output.suffix)
    if args.placement_json:
        output = output.with_name(output.stem.replace("_result", "_candidate_diagnostic_result") + output.suffix)
    if args.actual_stage0:
        output = output.with_name(output.stem.replace("_result", "_actual_stage0_result") + output.suffix)
    if args.actual_placement:
        output = output.with_name(output.stem.replace("_result", "_actual_placement_result") + output.suffix)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
