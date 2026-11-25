"""Check the diagnostic's hand mechanics with the cube disabled, at every1ms."""
import argparse
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "right_hand_rotation_20261109/prehardware_policy_cycle"))
import mujoco
from native_cycle import assets, restore, Guard, ticks_to_sim, save_json, sha
from work_grip_stage2_plan_20261112 import OUT, CANDIDATE, validate_candidate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gain", required=True, type=float, choices=(.5, 1., 1.5))
    args = parser.parse_args()
    folder = OUT / "hand_models"
    folder.mkdir(parents=True, exist_ok=True)
    output = folder / f"gain{args.gain:g}.json"
    if output.exists():
        raise RuntimeError("Preserve the completed hand-only evidence")
    z = validate_candidate()
    model, initial, _, _, provenance = assets("active_ccd50")
    model.actuator_gainprm[:, 0] *= args.gain
    model.actuator_biasprm[:, 1] *= args.gain
    body = model.body("cube").id
    mask = model.geom_bodyid == body
    model.geom_contype[mask] = 0
    model.geom_conaffinity[mask] = 0
    data = restore(model, initial)
    guard = Guard(model)
    stop = []
    done = 0
    for index, command in enumerate(z["goals"]):
        data.ctrl[:] = ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(model, data)
            reasons = guard.check(data)
            # Cube support and cube travel are not applicable in this explicitly
            # separate hand-only test. All hand/contact/range/tracking stops stay.
            stop = [r for r in reasons if r in ("self_mm", "support_mm", "tracking_rad", "range_rad", "nonfinite_state")]
            if stop:
                break
        done = index + 1
        if done % 500 == 0:
            print({"gain": args.gain, "hand_only_goal": done, "stop": stop}, flush=True)
        if stop:
            break
    report = {"status": "stage2_hand_only_native_screen", "candidate_sha256": sha(CANDIDATE),
              "source_script_sha256": sha(Path(__file__)), "gain_scale": args.gain,
              "goals_completed": done, "total_goals": len(z["goals"]), "stop": stop,
              "passed": not stop and done == len(z["goals"]), "worst": guard.worst,
              "cube_disabled_for_hand_only_check": True, "all_hand_joints_dynamic": True,
              "every_dynamic_sample_checked_ms": 1, "model_provenance": provenance,
              "device_accesses": 0, "hardware_execution_allowed": False}
    save_json(output, report)
    print({k: v for k, v in report.items() if k != "model_provenance"}, flush=True)


if __name__ == "__main__":
    main()
