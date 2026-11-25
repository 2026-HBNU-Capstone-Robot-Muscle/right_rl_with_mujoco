"""Build an immutable diagnostic only, without constructing device APIs."""
import json
from pathlib import Path
import numpy as np
from forward_hold_plan_20261104 import LOADED as FIRST, validate_candidate as first_candidate
from work_grip_stage2_plan_20261112 import ROOT, OUT, CANDIDATE, MANIFEST, MOVE, HOLD, HOLD_SAMPLES, sha, validate_candidate


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if CANDIDATE.exists() or MANIFEST.exists():
        raise RuntimeError("Preserve the frozen candidate")
    parent = ROOT / "right_hand_rotation_20261112/stage2_preparation/direct_work_grip_v1/reached_TH_MF_RF_IF"
    source = parent / "research_goals.npz"
    old = first_candidate()
    saved = dict(np.load(source, allow_pickle=False))
    review = json.loads((parent / "review.json").read_bytes())
    if review["stop"] or review["hold_samples"] != 4050 or review["upper_certified_samples"] != 4050:
        raise RuntimeError("No complete safe source diagnostic")
    moving = saved["goals"][:review["moving_steps"]]
    assert np.array_equal(saved["start"], old["goals"][-1])
    goals = np.vstack([old["goals"], moving, np.tile(moving[-1], (HOLD_SAMPLES, 1))]).astype(int)
    phases = np.r_[old["phases"], [MOVE] * len(moving), [HOLD] * HOLD_SAMPLES]
    np.savez_compressed(CANDIDATE, start=old["start"], goals=goals, phases=phases)
    source_paths = [FIRST, source, parent / "review.json", ROOT / "right_hand_rotation_20261111/stage1_actual_review/actual_first_turn_end_reference.json"]
    manifest = {"status": "observed_first_turn_and_work_grip_diagnostic_candidate",
                "candidate_sha256": sha(CANDIDATE), "source_files_sha256": {p.relative_to(ROOT).as_posix(): sha(p) for p in source_paths},
                "first_turn_goals_bitwise_unchanged": True, "first_turn_goals": len(old["goals"]),
                "transition_goals": len(moving), "final_hold_goals": HOLD_SAMPLES,
                "loaded_goals": len(goals), "control_seconds": .075, "maximum_goal_step_ticks": 2,
                "second_rotation_included": False, "new_live_SAC": False,
                "source_strict_full_work_grip_task_passed": review["passed"],
                "purpose": "Observe a bounded grip transition and its clearance; not a completed two-turn task",
                "model_first_turn_angle_does_not_reproduce_physical_angle": True,
                "model_target90_error_preserved_deg": review["orientation_error_deg"],
                "physical_cube_pose_reconstructed": False, "device_accesses": 0,
                "hardware_execution_allowed_by_candidate_build": False}
    MANIFEST.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    validate_candidate()
    print(json.dumps({"goals": len(goals), "loaded_seconds": len(goals) * .075, "transition_seconds": len(moving) * .075,
                      "first_turn_unchanged": True, "second_rotation_included": False, "devices_opened": 0}))


if __name__ == "__main__":
    main()
