"""Find a bounded work-grip diagnostic following the exact real stage-1 path.

The cube is initialized once by the archived placement model; it is never
registered from a photo or pinned during a transition. All native guards apply.
"""
import argparse
import itertools
import json
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
WORK = ROOT / "right_hand_rotation_20261109/prehardware_policy_cycle"
sys.path.insert(0, str(WORK))
import mujoco
from native_cycle import assets, restore, Guard, BOUND, motor, ticks_to_sim, sha, save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate

BASE = WORK / "next_cycle/handoff_refinement_20261110"
OUT = ROOT / "right_hand_rotation_20261112/stage2_preparation"
GROUPS = {"TH": np.arange(4), "IF": np.arange(4, 8), "MF": np.arange(8, 12), "RF": np.arange(12, 16)}


def transition(start, target, order):
    previous = start.copy()
    rows = []
    groups = [np.arange(16)] if order == "all" else [GROUPS[n] for n in order.split("_")]
    for group in groups:
        goal = previous.copy()
        goal[group] = target[group]
        n = max(1, int(np.ceil(np.max(abs(goal - previous)) / 2)))
        rows.extend(np.rint(previous + (goal - previous) * k / n).astype(int) for k in range(1, n + 1))
        previous = goal
    return np.asarray(rows)


def state(data, initial):
    return {**{k: getattr(data, k).copy() for k in BOUND}, "time": data.time,
            "up": initial["up"], "initial_cube_rotation": initial["initial_cube_rotation"],
            "initial_cube_center": initial["initial_cube_center"]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag", default="direct_work_grip_v1")
    parser.add_argument("--orders", nargs="*", default=["all", "RF_IF_MF_TH", "MF_TH_RF_IF", "RF_MF_TH_IF", "TH_MF_RF_IF"])
    parser.add_argument("--targets", nargs="*", default=["reconfigured", "reached", "canonical"])
    parser.add_argument("--hold", type=int, default=54)
    args = parser.parse_args()
    folder = OUT / args.tag
    if folder.exists():
        raise RuntimeError("Preserve completed screen; use a new tag")
    folder.mkdir(parents=True)
    (folder / "source_screen_direct_work_grip_20261112.py").write_bytes(Path(__file__).read_bytes())
    source = ROOT / "right_hand_rotation_20261104/forward_hold/candidate.npz"
    reference_file = ROOT / "right_hand_rotation_20261111/stage1_actual_review/actual_first_turn_end_reference.json"
    real = json.loads(reference_file.read_bytes())
    old = dict(np.load(source, allow_pickle=False))
    assert np.array_equal(old["goals"][-1], real["goal_ticks"])
    model, initial, _, _, provenance = assets("active_ccd50")
    data = restore(model, initial)
    check = Guard(model)
    cube = model.geom("cube").id
    previous = motor(data.ctrl)
    for index, command in enumerate(old["goals"]):
        assert np.max(abs(command - previous)) <= 2
        data.ctrl[:] = ticks_to_sim(command)
        previous = command.copy()
        for _ in range(75):
            mujoco.mj_step(model, data)
            stop = check.check(data)
            if stop:
                save_json(folder / "prefix_failure.json", {"goal": index + 1, "stop": stop, "terminal_guard": check.last})
                raise RuntimeError(("Exact stage-1 model prefix failed", index + 1, check.last))
    start_state = state(data, initial)
    np.savez_compressed(folder / "prefix_end_state.npz", **start_state)
    target90 = upright_yaw_target(initial["initial_cube_rotation"], initial["up"], 90.)
    first = {"goals": len(old["goals"]), "orientation_error_deg": orientation_error_deg(data.geom_xmat[cube].reshape(3, 3), target90),
             "worst": check.worst, "source_goal_matches_real_stage1": True}
    print(json.dumps({"prefix": first}), flush=True)
    targets = {"reconfigured": BASE / "initial_to_reconfigured_first90/research_goals.npz",
               "reached": BASE / "initial_to_reached_workstation90/research_goals.npz",
               "canonical": ROOT / "right_hand_rotation_20261105/canonical_body_grip/continuous_handoff/candidate.npz"}
    reports = []
    for name in args.targets:
        path = targets[name]
        target = np.asarray(np.load(path, allow_pickle=False)["goals"][-1], dtype=int)
        for order in args.orders:
            sub = folder / (name + "_" + order)
            sub.mkdir()
            d = restore(model, start_state)
            g = Guard(model)
            moves = transition(old["goals"][-1], target, order)
            goals = np.vstack([moves, np.tile(target, (args.hold, 1))])
            cert = SweepCertificate(model, g.hand, .3)
            samples = supported = clear = 0
            C0 = B0 = None
            minimum = None
            shift = drift = 0.
            stop = []
            executed = []
            previous = old["goals"][-1].copy()
            for index, command in enumerate(goals):
                assert np.max(abs(command - previous)) <= 2 and np.all(command >= g.lo) and np.all(command <= g.hi)
                d.ctrl[:] = ticks_to_sim(command)
                previous = command.copy()
                executed.append(command)
                for _ in range(75):
                    mujoco.mj_step(model, d)
                    stop = g.check(d)
                    if np.linalg.norm(d.qpos[-7:-4] - initial["initial_cube_center"]) > .08:
                        stop = list(stop) + ["cube_translation_over80mm"]
                    if index >= len(moves):
                        C, B = d.geom_xpos[cube], d.geom_xmat[cube].reshape(3, 3)
                        if C0 is None:
                            C0, B0 = C.copy(), B.copy()
                        samples += 1
                        supported += sample_model(model, d, g.hand, cube)["supported_sample"]
                        lower, bad = cert.query(d, C, B)
                        minimum = lower if minimum is None else min(minimum, lower)
                        clear += not bad and lower > .3
                        shift = max(shift, float(np.linalg.norm(C - C0) * 1000))
                        drift = max(drift, orientation_error_deg(B, B0))
                    if stop:
                        break
                if stop:
                    break
            error = orientation_error_deg(d.geom_xmat[cube].reshape(3, 3), target90)
            tilt = face_up_error_deg(d.geom_xmat[cube].reshape(3, 3), 5, initial["up"])
            passed = bool(not stop and samples == clear == args.hold * 75 and supported / max(1, samples) > .8
                          and error < 5 and tilt < 5 and shift < 2 and drift < 3)
            np.savez_compressed(sub / "research_goals.npz", start=old["goals"][-1], goals=executed)
            np.savez_compressed(sub / "end_state.npz", **state(d, initial))
            row = {"target": name, "order": order, "steps": len(executed), "moving_steps": len(moves),
                   "stop": stop, "hold_samples": samples, "supported_samples": supported, "upper_certified_samples": clear,
                   "minimum_upper_lower_mm": minimum, "orientation_error_deg": error, "tilt_deg": tilt,
                   "hold_shift_mm": shift, "hold_drift_deg": drift, "passed": passed,
                   "worst": g.worst, "terminal_guard": g.last, "path": sub.relative_to(ROOT).as_posix()}
            save_json(sub / "review.json", row)
            reports.append(row)
            save_json(folder / "progress.json", {"rows": reports})
            print(json.dumps({k: v for k, v in row.items() if k not in ("terminal_guard", "worst")}), flush=True)
    save_json(folder / "review.json", {"status": "exact_stage1_to_work_grip_screen", "prefix": first,
              "source_sha256": sha(source), "actual_end_reference_sha256": sha(reference_file), "model_provenance": provenance,
              "rows": reports, "passed": sum(r["passed"] for r in reports), "devices_opened": 0,
              "physical_cube_state_reconstructed": False, "hardware_execution_allowed": False})


if __name__ == "__main__":
    main()
