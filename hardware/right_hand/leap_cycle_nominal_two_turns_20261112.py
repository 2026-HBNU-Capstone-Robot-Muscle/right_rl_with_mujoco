"""Operator-only nominal cycle diagnostic; default mode opens no devices."""
import sys
import json
import numpy as np
from leap_forward_hold_observe_20261104 import ForwardHoldBus
from leap_reference_speed_yaw_20261029 import runtime,confirmed_run
from leap_natural_cradle_grasp_20261030 import NaturalCamera
from nominal_cycle_plan_20261112 import NominalCyclePlan,REQUEST,PROOF,APPROVAL,SOURCE,FIRST_HOLD,SECOND,FINAL_HOLD,scope_approved,sha,hashes
from nominal_cycle_gate_20261112 import validate_review,run_screen,make_review
from nominal_cycle_capture_20261112 import NominalCycleCapture
BASE_SAMPLE=ForwardHoldBus.checked_sample

class NominalCycleBus(ForwardHoldBus):
    def approve(self,plan,proof):
        if not scope_approved(): raise RuntimeError('Cycle scope not approved; no writes')
        self.proof=validate_review(plan,proof); self.plan_module=plan; self.planner=plan.validate_start
        self.screen_approved=True; self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([r['root_clear_goal_ticks'] for r in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
    @staticmethod
    def checked_sample(bus,command,report,phase,**kwargs):
        state,errors=BASE_SAMPLE(bus,command,report,phase,**kwargs)
        if phase in (FIRST_HOLD,SECOND,FINAL_HOLD) and hasattr(bus,'grasp_protocol'): bus.grasp_protocol.cycle_frame(bus,command,report,phase)
        return state,errors

NominalCyclePlan.CAMERA_TYPE=NaturalCamera; NominalCyclePlan.CAPTURE_PROTOCOL_TYPE=NominalCycleCapture

def main():
    plan=NominalCyclePlan(REQUEST)
    try: proof=validate_review(plan,PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:
        print('NOT READY:',exc,'. No devices opened.',flush=True); return 2
    if '--execute' in sys.argv and not scope_approved():
        print('실행 전 범위 확인: 기존300초 대신 최대500초를 쓰는 새 전체 경로 진단입니다.',flush=True)
        print('명목 모델의 두 회전/두 유지는 통과했습니다. 마찰·배치 오차 내성과 실물 성공은 미검증입니다.',flush=True)
        print('큐브 없는 손 검사에는 미세 접촉이 남아 있어 모든 조건의 안전 통과로 표시하지 않습니다.',flush=True)
        print('450mA/기존PID/2틱/75ms 및 기존 건강·추종·통신 중단 기준은 유지합니다.',flush=True)
        answer=input('이 한정 관찰 시험에 동의하면 500 입력 후 Enter. 취소는 Enter: ').strip()
        if answer!='500':
            print('취소했습니다. 장치를 열지 않았습니다.',flush=True); return 1
        APPROVAL.write_text(json.dumps({'user_explicitly_approved':True,'maximum_on_seconds':500,
            'source_goals_sha256':sha(SOURCE),'acknowledges_nominal_only_and_hand_only_failures':True,
            'approval_method':'operator typed500 in user-started terminal before device access'},indent=2)+'\n',encoding='utf-8')
        PROOF.write_text(json.dumps(make_review(plan,proof['empty_recovery'],hashes()),indent=2)+'\n',encoding='utf-8')
        validate_review(plan,PROOF)
    if '--execute' not in sys.argv and not scope_approved():
        print('CHECK ONLY: 구동시간500초와 명목조건 한정 진단의 사용자 승인 대기. 장치를 열지 않았습니다.',flush=True); return 0
    print(NominalCyclePlan.TITLE,flush=True)
    print('v3은 낙하 구간의 검지 지지를 보정한 시험 후보입니다. 추가 구동 지연 조건은 미달했고 실물 낙하 해결은 아직 확인 전입니다.',flush=True)
    print('빈손 첫SPACE -> PLACE 초록위/주황손목 -> 두 번째SPACE -> 두 회전/두 유지 -> 받치고 마지막SPACE OFF.',flush=True)
    print('450mA/기존PID/2틱/75ms와 건강·추종·통신 기준 유지. 전체 경로의 실물 성공은 미검증. Q/Esc/Ctrl+C 중단.',flush=True)
    previous=runtime.run
    try: runtime.run=confirmed_run; return runtime.main(NominalCyclePlan,bus_type=NominalCycleBus,screener=run_screen)
    finally: runtime.run=previous

if __name__=='__main__': raise SystemExit(main())
