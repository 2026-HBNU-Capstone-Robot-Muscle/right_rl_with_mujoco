"""Record before/after the new diagnostic, then support/SPACE OFF."""
import numpy as np
from forward_hold_capture_20261104 import ForwardHoldCapture
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from work_grip_stage2_plan_20261112 import HOLD, MOVE, validate_candidate


class WorkGripStage2Capture(ForwardHoldCapture):
    def require_open(self, command):
        if not np.array_equal(command, validate_candidate()["start"]):
            raise RuntimeError("Reviewed placement pose not reached")

    def place(self, bus, command, report, guard):
        print("PLACE에서만 배치합니다. 초록 위·주황 손목ID41 쪽. 손을 뗀 뒤 두 번째 SPACE.", flush=True)
        print("기존 첫 회전과10초 유지 뒤 파지를 바꾸고10초 관찰합니다. 두 번째 회전은 없습니다.", flush=True)
        NewPairGraspCapture.place(self, bus, command, report, guard)
        report.update(placement_orientation="GREEN up; ORANGE wristward", second_rotation_included=False,
                      physical_work_grip_verified=False, physical_upper_clearance_verified=False,
                      automatic_open=False, automatic_inverse=False, no_live_SAC=True)

    def hold_frame(self, bus, command, report, state):
        if not getattr(self, "grasp_notice", False):
            self.grasp_notice = True
            print("파지 확인 중. 이어서 기존 첫 회전과10초 유지를 진행합니다.", flush=True)
        return super().hold_frame(bus, command, report, state)

    def rotation_frame(self, bus, command, report, state):
        if not getattr(self, "rotation_notice", False):
            self.rotation_notice = True
            report["camera_instruction"] = "첫 회전 뒤10초 유지. 이어서 파지 전환. 손을 떼고 관찰. Q 중단"
            print("첫 회전 뒤10초 유지 중. 다음은 엄지·중지·약지·검지 순서의 파지 전환 관찰입니다.", flush=True)

    def work_frame(self, bus, command, report, phase):
        if phase == MOVE and not getattr(self, "transition_started", False):
            self.transition_started = True
            observe = self._observer(bus, command, report, self.guard)
            _, photo = self._photo("cube_before_work_grip.jpg", "work_grip_start_checkpoint", observe)
            report.update(work_grip_before_photo=photo, work_grip_transition_attempted=True,
                          phase=phase,
                          camera_instruction="파지 전환 중. 큐브를 만지지 말고 관찰. 이상하면 Q")
            print("파지 전환 시작. 큐브 지지와 윗단 공간을 관찰합니다. 이상하면 Q.", flush=True)
        if phase == HOLD and not getattr(self, "work_hold_started", False):
            self.work_hold_started = self.now()
            report["camera_instruction"] = "파지 전환 뒤10초 관찰. 계속 손을 떼고 관찰. Q 중단"
            print("파지 전환 뒤10초 유지 관찰 중입니다.", flush=True)

    def finish(self, bus, command, report):
        z = validate_candidate()
        if (not report.get("cube_placement_confirmed") or not report.get("work_grip_transition_attempted")
                or not np.array_equal(command, z["goals"][-1]) or not hasattr(bus, "pending") or bus.pending):
            raise RuntimeError("Stage-2 queue/checkpoints incomplete")
        observe = self._observer(bus, command, report, self.guard)
        _, photo = self._photo("cube_after_work_grip_hold.jpg", "work_grip_final_observation", observe)
        report.update(work_grip_after_photo=photo, diagnostic_hold_seconds=10.05,
                      physical_work_grip_verified=False, physical_upper_clearance_verified=False,
                      camera_instruction="관찰 종료. 큐브 아래를 받치고 세 번째 SPACE=OFF. 종료 뒤 큐브 제거")
        print("관찰이 끝났습니다. 큐브 아래를 받치고 세 번째 SPACE로 OFF. 종료 확인 뒤 큐브를 빼세요.", flush=True)
        state = self._space("support_cube_for_off", observe)
        report.update(final_on_state={k: v.tolist() for k, v in state.items()},
                      cube_support_confirmed_before_off=True, cube_removal_confirmed=False, cube_present=True)
