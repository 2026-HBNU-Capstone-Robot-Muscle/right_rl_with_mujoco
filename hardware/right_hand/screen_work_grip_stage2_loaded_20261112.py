"""Finite native safety checks for the new bounded grip-transition experiment."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
WORK = ROOT / "right_hand_rotation_20261109/prehardware_policy_cycle"
sys.path.insert(0, str(WORK))
import mujoco
from native_cycle import assets, restore, Guard, BOUND, ticks_to_sim, sha, save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate
from work_grip_stage2_plan_20261112 import OUT, CANDIDATE, HOLD, validate_candidate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gain", required=True, type=float, choices=(.5, 1., 1.5))
    args = parser.parse_args()
    folder = OUT / "loaded_models" / f"gain{args.gain:g}"
    if (folder / "review.json").exists():
        raise RuntimeError("Preserve completed finite screen")
    folder.mkdir(parents=True, exist_ok=True)
    z = validate_candidate()
    rows = []
    for mu in (.2, .3, .5):
        model, initial, _, _, provenance = assets("active_ccd50")
        model.actuator_gainprm[:, 0] *= args.gain
        model.actuator_biasprm[:, 1] *= args.gain
        # This is the same declared sliding-friction hypothesis used in the
        # native cycle studies, not a measurement of hard plastic or 450mA.
        model.geom_friction[:, 0] = mu
        data = restore(model, initial)
        guard = Guard(model)
        cube = model.geom("cube").id
        cert = SweepCertificate(model, guard.hand, .3)
        samples = held = clear = 0
        minimum = None
        C0 = B0 = None
        drift = shift = 0.
        stop = []
        done = 0
        for index, (command, phase) in enumerate(zip(z["goals"], z["phases"])):
            data.ctrl[:] = ticks_to_sim(command)
            for _ in range(75):
                mujoco.mj_step(model, data)
                stop = guard.check(data)
                if np.linalg.norm(data.qpos[-7:-4] - initial["initial_cube_center"]) > .08:
                    stop = list(stop) + ["cube_translation_over80mm"]
                if phase == HOLD:
                    C, B = data.geom_xpos[cube], data.geom_xmat[cube].reshape(3, 3)
                    if C0 is None:
                        C0, B0 = C.copy(), B.copy()
                    samples += 1
                    held += sample_model(model, data, guard.hand, cube)["supported_sample"]
                    lower, bad = cert.query(data, C, B)
                    minimum = lower if minimum is None else min(minimum, lower)
                    clear += not bad and lower > .3
                    shift = max(shift, float(np.linalg.norm(C - C0) * 1000))
                    drift = max(drift, orientation_error_deg(B, B0))
                if stop:
                    break
            done = index + 1
            if done % 500 == 0:
                print(json.dumps({"gain": args.gain, "friction": mu, "goal": done, "stop": stop}), flush=True)
            if stop:
                break
        safe = bool(not stop and done == len(z["goals"]))
        observation = bool(safe and samples == 10050 and held / samples > .8 and shift < 2 and drift < 3)
        row = {"gain_scale": args.gain, "sliding_friction_hypothesis": mu, "executed_goals": done,
               "total_goals": len(z["goals"]), "stop": stop, "all_native_safety_passed": safe,
               "stable_supported_diagnostic_hold": observation, "hold_samples": samples,
               "supported_samples": held, "upper_certified_samples": clear, "minimum_upper_lower_mm": minimum,
               "hold_shift_mm": shift, "hold_drift_deg": drift,
               "model_target90_error_deg": orientation_error_deg(data.geom_xmat[cube].reshape(3, 3), upright_yaw_target(initial["initial_cube_rotation"], initial["up"], 90)),
               "tilt_deg": face_up_error_deg(data.geom_xmat[cube].reshape(3, 3), 5, initial["up"]),
               "worst": guard.worst, "terminal_guard": guard.last, "model_provenance": provenance}
        np.savez_compressed(folder / f"mu{mu:g}_end_state.npz", **{k: getattr(data, k) for k in BOUND}, time=data.time)
        rows.append(row)
        save_json(folder / "progress.json", {"rows": rows})
        print(json.dumps({k: v for k, v in row.items() if k not in ("terminal_guard", "model_provenance")}), flush=True)
    report = {"status": "bounded_work_grip_diagnostic_finite_native_screen", "candidate_sha256": sha(CANDIDATE),
              "source_script_sha256": sha(Path(__file__)), "rows": rows,
              "all_native_safety_passed": all(r["all_native_safety_passed"] for r in rows),
              "diagnostic_hold_passed": all(r["stable_supported_diagnostic_hold"] for r in rows),
              "not_a_full90_or_two_turn_task_certificate": True, "every_dynamic_sample_checked_ms": 1,
              "cube_initialized_only_before_first_motion": True, "joint_and_collision_guards_unchanged": True,
              "devices_opened": 0, "hardware_execution_allowed": False}
    save_json(folder / "review.json", report)


if __name__ == "__main__":
    main()
