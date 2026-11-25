"""Assemble the concrete diagnostic proof after all independent checks finish."""
import json
from work_grip_stage2_plan_20261112 import OUT, REQUEST, PROOF, WorkGripStage2Plan, hashes
from work_grip_stage2_gate_20261112 import make_review, validate_review


def main():
    if PROOF.exists():
        raise RuntimeError("Refuse to replace the frozen proof")
    plan = WorkGripStage2Plan(REQUEST)
    empty = json.loads((OUT / "seed_empty_screen/review.json").read_bytes())
    result = make_review(plan, empty, hashes())
    PROOF.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    validate_review(plan, PROOF)
    print(json.dumps({"status": result["status"], "loaded_goals": result["loaded_goals"],
                      "finite_loaded_conditions": result["work_grip_diagnostic_models"]["finite_cases"],
                      "hand_only_conditions": result["work_grip_diagnostic_models"]["hand_only_cases"],
                      "static_OFF_conditions": len(empty["static_cases"]), "dynamic_OFF_conditions": len(empty["dynamic_cases"]),
                      "second_rotation_included": result["second_rotation_included"], "devices_opened": 0}))


if __name__ == "__main__":
    main()
