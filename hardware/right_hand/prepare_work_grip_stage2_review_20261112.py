"""Create a new offline seed request from an immutable successful real run."""
import json
from work_grip_stage2_plan_20261112 import ROOT, OUT, REQUEST, WorkGripStage2Plan, sha


def main():
    source = ROOT / "forward_hold_observe_20261111_220908_830958/fresh_off_request.json"
    real = ROOT / "forward_hold_observe_20261111_220908_830958/result.json"
    result = json.loads(real.read_bytes())
    if result["status"] != "forward_hold_observation_complete" or not result["all_torque_off_verified"] or not result["settings_restored"]:
        raise RuntimeError("Successful physical reference unavailable")
    old = json.loads(source.read_bytes())
    request = {"plan_version": WorkGripStage2Plan.PLAN_VERSION,
               "motor_ids_in_sim_order": old["motor_ids_in_sim_order"], "off_state": old["off_state"],
               "source": source.relative_to(ROOT).as_posix(), "source_sha256": sha(source),
               "physical_reference_result_sha256": sha(real), "hardware_execution_allowed": False,
               "seed_is_saved_OFF_not_current_device_measurement": True,
               "runtime_must_remeasure_actual_OFF_before_any_write": True}
    if REQUEST.exists():
        raise RuntimeError("Refuse to replace the seed request")
    OUT.mkdir(parents=True, exist_ok=True)
    REQUEST.write_text(json.dumps(request, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    plan = WorkGripStage2Plan(REQUEST)
    rows = plan.plan(plan.source_position())
    print(json.dumps({"request": REQUEST.relative_to(ROOT).as_posix(), "whole_goals": len(rows),
                      "motion_seconds_from_saved_seed": len(rows) * .075, "device_accesses": 0}))


if __name__ == "__main__":
    main()
