"""Verify the saved actual-start proof and the first trial's no-execute mode."""
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
from unittest.mock import patch

from forward_hold_plan_20261104 import ForwardHoldPlan
from forward_hold_gate_20261104 import validate_review

ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / "right_hand_rotation_20261109/prehardware_policy_cycle/physical_test_preparation_20261111/connected_20261111_214517_052288"


def main():
    plan = ForwardHoldPlan(FOLDER / "forward_hold_request.json")
    review = validate_review(plan, FOLDER / "current_review.json")
    empty = review["empty_recovery"]
    assert len(empty["static_cases"]) == 201 and len(empty["dynamic_cases"]) == 9
    assert empty["all_passed"] and all(c["passed"] for c in empty["static_cases"] + empty["dynamic_cases"])
    assert math.isclose(review["fixed_hold_seconds"], 10.05, rel_tol=0, abs_tol=1e-12)
    assert not review["automatic_open"] and not review["automatic_inverse"]
    assert review["loaded_goals"] == 1417 and plan.SAMPLE_PERIOD == .075 and plan.MAX_GOAL_STEP == 2
    assert not review["physical90_verified"] and not review["physical_body_hold_verified"]
    import leap_forward_hold_observe_20261104 as entry
    with patch("sys.argv", ["leap_forward_hold_observe_20261104.py"]), \
         patch("dynamixel_sdk.PortHandler", side_effect=AssertionError("Motor construction prohibited in default mode")) as port, \
         patch("camera_stream.CameraStream", side_effect=AssertionError("Camera construction prohibited in default mode")) as camera, \
         contextlib.redirect_stdout(io.StringIO()) as log:
        code = entry.main()
    assert code == 0 and port.call_count == 0 and camera.call_count == 0 and "CHECK ONLY" in log.getvalue()
    framing_path = FOLDER / "camera_framing_review.json"
    framing_accepted = False
    if framing_path.exists():
        framing = json.loads(framing_path.read_bytes())
        photograph = (ROOT / framing["photograph"]).resolve()
        photograph.relative_to(ROOT)
        assert hashlib.sha256(photograph.read_bytes()).hexdigest() == framing["photograph_sha256"]
        framing_accepted = framing["status"] == "acceptable_for_stage1_visual_observation"
    result = dict(status="stage1_actual_start_model_review_passed", static_conditions=201,
                  dynamic_conditions=9, loaded_hand_conditions=review["forward_hold_models"]["hand_cases"],
                  loaded_cube_conditions=review["forward_hold_models"]["cube_cases"],
                  actual_start_ticks=plan.source_position().tolist(),
                  command="leap_cycle_stage1_forward_hold.cmd", new_two_turn_cycle_ready=False,
                  camera_framing_confirmation_pending=not framing_accepted, operator_starts_motion_with_SPACE=True,
                  camera_and_motor_constructions_in_default_mode=0, physical_motion_executed=False,
                  actual_read_only_source="cycle_test_readonly_20261111_214517_052288/result.json",
                  actual_read_only_instructions={"PING": 16, "READ": 48}, assistant_motor_writes=0,
                  current_review_sha256=hashlib.sha256((FOLDER / "current_review.json").read_bytes()).hexdigest(),
                  launcher_sha256=hashlib.sha256((ROOT / "leap_cycle_stage1_forward_hold.cmd").read_bytes()).hexdigest())
    (FOLDER / "stage1_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k not in ("actual_start_ticks", "current_review_sha256", "launcher_sha256")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
