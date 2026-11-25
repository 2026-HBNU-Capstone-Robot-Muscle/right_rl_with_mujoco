"""Bind a limited diagnostic to the original runtime and fresh-start review."""
import json
from pathlib import Path
import numpy as np
import forward_hold_gate_20261104 as original
from forward_hold_plan_20261104 import hashes as original_hashes
from calibrated_quarter_gate_20261028 import bounded
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
from work_grip_stage2_plan_20261112 import ROOT, OUT, REQUEST, PROOF, CANDIDATE, MANIFEST, WorkGripStage2Plan, hashes, sha, validate_candidate


def models():
    z = validate_candidate()
    rows, sources = [], []
    checker = ROOT / "screen_work_grip_stage2_loaded_20261112.py"
    for gain in (.5, 1., 1.5):
        path = OUT / "loaded_models" / f"gain{gain:g}" / "review.json"
        report = json.loads(path.read_bytes())
        if (report["candidate_sha256"] != sha(CANDIDATE) or report["source_script_sha256"] != sha(checker)
                or report["every_dynamic_sample_checked_ms"] != 1 or not report["cube_initialized_only_before_first_motion"]
                or not report["joint_and_collision_guards_unchanged"]):
            raise RuntimeError("New diagnostic model source/scope mismatch")
        rows.extend(report["rows"])
        sources.append({"path": path.relative_to(ROOT).as_posix(), "sha256": sha(path)})
    if len(rows) != 9 or {(r["gain_scale"], r["sliding_friction_hypothesis"]) for r in rows} != {(g, mu) for g in (.5, 1., 1.5) for mu in (.2, .3, .5)}:
        raise RuntimeError("Incomplete diagnostic finite conditions")
    for row in rows:
        if (row["stop"] or not row["all_native_safety_passed"] or row["executed_goals"] != len(z["goals"])
                or not row["stable_supported_diagnostic_hold"] or row["hold_samples"] != 10050
                or row["supported_samples"] / 10050 <= .8
                or not bounded(row, {"hold_shift_mm": 2., "hold_drift_deg": 3.})
                or not bounded(row["worst"], {"self_mm": 1e-6, "support_mm": .15, "cube_mm": 1., "tracking_rad": .18, "range_rad": 1e-6})):
            raise RuntimeError("New diagnostic safety/support/hold failed; do not execute")
    nominal = next(r for r in rows if (r["gain_scale"], r["sliding_friction_hypothesis"]) == (1., .3))
    if nominal["upper_certified_samples"] != 10050 or nominal["minimum_upper_lower_mm"] <= .3:
        raise RuntimeError("Nominal diagnostic does not reach the intended free upper area")
    hands = []
    for gain in (.5, 1., 1.5):
        path = OUT / "hand_models" / f"gain{gain:g}.json"
        hand = json.loads(path.read_bytes())
        if (hand["candidate_sha256"] != sha(CANDIDATE)
                or hand["source_script_sha256"] != sha(ROOT / "screen_work_grip_stage2_hand_20261112.py")
                or hand["gain_scale"] != gain or not hand["cube_disabled_for_hand_only_check"]
                or not hand["all_hand_joints_dynamic"] or hand["every_dynamic_sample_checked_ms"] != 1
                or not hand["passed"] or hand["stop"] or hand["goals_completed"] != len(z["goals"])
                or not bounded(hand["worst"], {"self_mm": 1e-6, "support_mm": .15, "tracking_rad": .18, "range_rad": 1e-6})):
            raise RuntimeError("New hand-only mechanical safety screen failed")
        hands.append({"path": path.relative_to(ROOT).as_posix(), "sha256": sha(path)})
    return {"sources": sources, "finite_cases": 9, "all_native_safety_passed": True,
            "hand_only_sources": hands, "hand_only_cases": 3,
            "all_diagnostic_holds_supported": True, "nominal_upper_samples": 10050,
            "nominal_upper_lower_mm": nominal["minimum_upper_lower_mm"],
            "model_target90_error_preserved_deg": nominal["model_target90_error_deg"],
            "precise_orientation_or_full_task_not_certified": True,
            "sliding_friction_hypotheses_not_measured": True, "physical_support_force_not_measured": True,
            "source_model_torque_not_a450mA_conversion": True, "actual_second_rotation_included": False}


def make_review(plan, empty, before):
    # Keep all established recovery/transport policies and the first route's
    # original evidence; the added nine cases certify only this diagnostic.
    result = original.make_review(plan, empty, original_hashes())
    new_models = models()
    after = hashes()
    if before != after:
        raise RuntimeError("Stage-2 sources changed during review")
    result["historical_first_forward_models"] = result.pop("forward_hold_models")
    result["historical_first_forward_goals"] = result.pop("loaded_goals")
    result["historical_first_fixed_hold_seconds"] = result.pop("fixed_hold_seconds")
    result["historical_first_fixed_hold_samples"] = result.pop("fixed_hold_samples")
    result.update(status="supervised_work_grip_transition_diagnostic_review", code_hashes=after,
                  work_grip_diagnostic_models=new_models, loaded_goals=len(validate_candidate()["goals"]),
                  second_rotation_included=False, no_live_policy=True,
                  scope="Exact actually executed first turn/10s hold -> bounded work-grip transition/10s hold -> user supports cube/SPACE OFF",
                  loaded_motion_authorized=True, full_task_success_certified=False,
                  new_joint_goal_vectors=True, diagnostic_hold_seconds=10.05)
    return result


def validate_review(plan, path):
    result = json.loads(Path(path).read_bytes())
    expected = make_review(plan, result["empty_recovery"], hashes())
    if any(result.get(k) != v for k, v in expected.items()):
        raise RuntimeError("Stage-2 proof/source/goal mismatch; no writes")
    return result


def run_screen(request, output, **kwargs):
    plan = WorkGripStage2Plan(request)
    reference = WorkGripStage2Plan(REQUEST)
    old = validate_review(reference, PROOF)
    if np.array_equal(plan.source_position(), reference.source_position()):
        result = make_review(plan, old["empty_recovery"], hashes())
        Path(output).write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return validate_review(plan, output)
    process_screen(request, output, checker=ROOT / "screen_work_grip_stage2_current_20261112.py",
                   failure_details=lambda _: "fresh stage-2 EMPTY recovery rejected", **kwargs)
    return validate_review(plan, output)
