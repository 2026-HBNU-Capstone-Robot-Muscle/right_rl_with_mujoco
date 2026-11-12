"""Only the operator's --execute and terminal SPACE permit motor output."""
import numpy as np
from leap_forward_hold_observe_20261104 import ForwardHoldBus
from leap_reference_speed_yaw_20261029 import runtime, confirmed_run
from leap_natural_cradle_grasp_20261030 import NaturalCamera
from work_grip_stage2_plan_20261112 import WorkGripStage2Plan, REQUEST, PROOF, MOVE, HOLD
from work_grip_stage2_gate_20261112 import validate_review, run_screen
from work_grip_stage2_capture_20261112 import WorkGripStage2Capture

BASE_SAMPLE = ForwardHoldBus.checked_sample


class WorkGripStage2Bus(ForwardHoldBus):
    def approve(self, plan, proof):
        self.proof = validate_review(plan, proof)
        self.plan_module = plan
        self.planner = plan.validate_start
        self.screen_approved = True
        self.support_checked = False
        self.root_release_threshold = np.max(np.asarray([r["root_clear_goal_ticks"] for r in self.proof["empty_recovery"]["static_cases"]]), axis=0) + 4
        self.initial_pair_checked = not self.proof["initial_self_separation"]["enabled"]

    @staticmethod
    def checked_sample(bus, command, report, phase, **kwargs):
        state, errors = BASE_SAMPLE(bus, command, report, phase, **kwargs)
        if phase in (MOVE, HOLD) and hasattr(bus, "grasp_protocol"):
            bus.grasp_protocol.work_frame(bus, command, report, phase)
        return state, errors


WorkGripStage2Plan.CAMERA_TYPE = NaturalCamera
WorkGripStage2Plan.CAPTURE_PROTOCOL_TYPE = WorkGripStage2Capture


def main():
    try:
        validate_review(WorkGripStage2Plan(REQUEST), PROOF)
    except (OSError, ValueError, RuntimeError, KeyError, TypeError, AssertionError) as exc:
        print("NOT READY:", exc, ". No devices opened.", flush=True)
        return 2
    print("빈손 첫SPACE -> PLACE 초록위/주황손목 -> 두 번째SPACE: 첫 회전/10초 -> 파지 전환/10초.", flush=True)
    print("완료 안내 뒤 큐브 아래를 받치고 세 번째SPACE=OFF. 자동OPEN/역회전/두 번째 회전은 없습니다.", flush=True)
    print("450mA/기존PID/최대2틱/75ms와300초 구동 예산, 기존 전류·전압·추종·통신 중단 기준을 유지합니다.", flush=True)
    print("이 파일은 파지 전환 진단입니다. 실제 방향·공간·지지는 저장 영상으로 확인합니다. Q/Esc/Ctrl+C 중단.", flush=True)
    previous = runtime.run
    try:
        runtime.run = confirmed_run
        return runtime.main(WorkGripStage2Plan, bus_type=WorkGripStage2Bus, screener=run_screen)
    finally:
        runtime.run = previous


if __name__ == "__main__":
    raise SystemExit(main())
