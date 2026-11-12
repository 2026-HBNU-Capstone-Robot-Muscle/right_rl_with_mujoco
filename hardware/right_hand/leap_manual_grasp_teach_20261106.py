"""User starts the executed grasp; joint teaching receives only READ access."""
import json,sys,time
import numpy as np
from manual_grasp_plan_20261106 import R,REQUEST,PROOF,ManualGraspPlan
from manual_grasp_gate_20261106 import validate_review,run_screen
from manual_grasp_capture_20261106 import ManualGraspCapture
from leap_forward_hold_observe_20261104 import ForwardHoldBus
from leap_reference_speed_yaw_20261029 import runtime,confirmed_run
from leap_natural_cradle_grasp_20261030 import NaturalCamera
from manual_grasp_off_teach_20261106 import require_completed_off_motion,run as teach_readonly
class ManualGraspBus(ForwardHoldBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False;self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4;self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
ManualGraspPlan.CAMERA_TYPE=NaturalCamera;ManualGraspPlan.CAPTURE_PROTOCOL_TYPE=ManualGraspCapture
def completed_source(pointer,began):
    if not pointer.is_file() or pointer.stat().st_mtime<began:raise RuntimeError('No result from this motion invocation')
    report=json.loads(pointer.read_bytes());require_completed_off_motion(report);source=__import__('pathlib').Path(report['request_file']).parent/'result.json';source.resolve().relative_to(R)
    if source.read_bytes()!=pointer.read_bytes():raise RuntimeError('Motion result pointer/source mismatch')
    return source
def main():
    execute='--execute' in sys.argv
    try:validate_review(ManualGraspPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.',flush=True);return 2
    print('첫SPACE 빈손 -> PLACE/두 번째SPACE -> 같은 파지/고정 -> 큐브 받치고 세 번째SPACE OFF.',flush=True)
    print('자동 열기/회전 없음. 토크OFF 확인 뒤 손가락 각도 보정, 네 번째SPACE로 읽기/사진 저장. 자동 재구동 없음.',flush=True)
    saved=runtime.run;began=time.time()
    try:runtime.run=confirmed_run;code=runtime.main(ManualGraspPlan,bus_type=ManualGraspBus,screener=run_screen)
    finally:runtime.run=saved
    if not execute or code:return code
    try:source=completed_source(R/'leap_manual_grasp_teach_motion_result.json',began)
    except (OSError,ValueError,RuntimeError,KeyError) as exc:print('OFF teaching not started:',exc,flush=True);return 1
    return teach_readonly(source)
if __name__=='__main__':raise SystemExit(main())
