"""Keep the final target latched, with fresh health reads, until explicit exit."""
import time
import numpy as np
from held_joint_guard import HeldJointGuard, HeldJointFault
from final_grip_pose_plan_20261116 import candidate

class FinalGripPoseCapture:
    def __init__(self, camera, keys, *, cancel, sleep=time.sleep, now=time.monotonic):
        self.camera, self.keys, self.cancel = camera, keys, cancel
        self.sleep, self.now = sleep, now

    def release(self, *args):
        raise RuntimeError('Unexpected cube/contact release in empty pose mode')

    def finish(self, bus, command, report):
        if (not np.array_equal(command, candidate()['goals'][-1])
                or not hasattr(bus, 'pending') or bus.pending):
            raise RuntimeError('Final empty pose queue incomplete; OFF')
        deadline = report['enabled_at'] + bus.plan_module.ON_TIMEOUT
        self.cancel()
        state, _ = bus.checked_sample(bus, command, report, 'final_grip_user_hold')
        guard = HeldJointGuard(state['position'], command)
        self.keys.arm()
        began = self.now()
        checks = 0
        report.update(cube_present=False, empty_only=True, final_pose_reached=True,
                      automatic_open=False, physical_cube_grasp_verified=False,
                      final_pose_target_ticks=np.asarray(command).tolist(),
                      final_pose_hold_goal_writes=0)
        print('FINAL POSE HOLD: 빈손 최종 모양에 도착했습니다. 큐브를 넣지 마세요.', flush=True)
        print('이 자세를 유지합니다. 종료하려면 터미널 SPACE. Q/Esc/Ctrl+C도 중단합니다.', flush=True)
        print('유지 중 새 목표 쓰기는 없습니다. 기존 전체 구동 최대500초에 도달하면 OFF합니다.', flush=True)
        def check_snapshot(s):
            try:
                guard.check(s['position'], command, np.arange(16), 'final_grip_user_hold')
            except HeldJointFault as exc:
                report['fixed_hold_fault'] = exc.details
                raise
        while True:
            cycle = self.now()
            if cycle >= deadline:
                report['hold_end_reason'] = 'existing_500s_total_on_limit'
                break
            self.cancel()
            report['phase'] = 'final_grip_user_hold'
            state, _ = bus.checked_sample(bus, command, report, report['phase'], snapshot_check=check_snapshot)
            checks += 1
            if not report.get('final_pose_photo'):
                photo = self.camera.capture_named('final_grip_empty.jpg', after=began+.2)
                if photo is not None:
                    report['final_pose_photo'] = photo
            if self.keys.consume_space():
                report['hold_end_reason'] = 'operator_SPACE'
                break
            self.sleep(min(max(0., bus.plan_module.SAMPLE_PERIOD-(self.now()-cycle)),
                           max(0., deadline-self.now())))
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},
                      final_pose_hold_seconds=self.now()-began, final_pose_hold_checks=checks)
