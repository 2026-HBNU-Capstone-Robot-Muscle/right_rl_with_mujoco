"""Publish a verifiable handoff for the prepared user-started diagnostic."""
import json
from work_grip_stage2_plan_20261112 import ROOT, OUT, REQUEST, PROOF, CANDIDATE, MANIFEST, WorkGripStage2Plan, sha
from work_grip_stage2_gate_20261112 import validate_review


def main():
    proof = validate_review(WorkGripStage2Plan(REQUEST), PROOF)
    software = json.loads((OUT / "software_verification.json").read_bytes())
    if not software["passed"] or software["failures"] or software["errors"]:
        raise RuntimeError("Software verification is incomplete")
    sources = [ROOT / "leap_cycle_stage2_work_grip.cmd", CANDIDATE, MANIFEST, PROOF,
               OUT / "operator_guide.md", OUT / "REVIEW.md", OUT / "software_verification.json"]
    result = {"status": "stage2_diagnostic_executable_prepared", "command": "leap_cycle_stage2_work_grip.cmd",
              "scope": "actually executed first turn/10s -> bounded work-grip transition/10s -> support/SPACE OFF",
              "first_real_route_unchanged": True, "new_second_rotation_included": False,
              "whole_two_turn_cycle_ready": False, "full_work_grip_task_certified": False,
              "finite_loaded_model_cases_passed": 9, "hand_only_cases_passed": 3,
              "seed_static_OFF_cases_passed": 201, "seed_dynamic_OFF_cases_passed": 9,
              "software_tests_passed": software["tests"], "operator_trial_review_bound": proof["accepted_for_operator_trial"],
              "runtime_must_remeasure_and_screen_current_OFF": True,
              "motor_writes_require_operator_execute_and_SPACE": True,
              "current_ma": 450, "control_seconds": .075, "maximum_goal_step_ticks": 2,
              "active_motion_budget_seconds": 300, "PID_and_health_stops_unchanged": True,
              "new_physical_run_executed": False, "assistant_motor_writes": 0, "remote_writes": 0,
              "latest_host_inventory": {"ports": ["COM3", "COM4"], "known_robot_adapter_found": False,
                                        "serial_ports_opened": 0},
              "source_files_sha256": {p.relative_to(ROOT).as_posix(): sha(p) for p in sources}}
    (OUT / "READY.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k not in ("source_files_sha256", "latest_host_inventory")}))


if __name__ == "__main__":
    main()
