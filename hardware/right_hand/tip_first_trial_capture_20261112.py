"""Record staged transfer observations; retain support until user SPACE OFF."""
import numpy as np
from forward_hold_capture_20261104 import ForwardHoldCapture
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from tip_first_trial_plan_20261112 import candidate,RECEIVE,RECEIVE_HOLD,PUSH,REAR,TIPS,NEXT,HOLD

NOTICES={RECEIVE:'중지 윗마디를 먼저 펴서 엄지 이송 통로를 만듭니다.',RECEIVE_HOLD:'중지를 뺀 받침 자세를 유지합니다.',
    PUSH:'엄지로 큐브를 중지 받침 쪽으로 옮깁니다.',REAR:'엄지 접촉 목표를 주황 옆면의 뒤쪽으로 옮깁니다.',
    TIPS:'엄지 ID1·중지 ID9 끝마디를 먼저 굽힙니다. 눌리는 다른 마디의 목표는 조금 풀어 줍니다.',
    NEXT:'그다음 엄지 ID2·중지 ID11을 굽힙니다. 기존 추종 기준은 그대로 유지합니다.',
    HOLD:'옮긴 자세를4.05초 관찰합니다. 약지 닫힘과 다음 회전은 없습니다.'}

class TipFirstTrialCapture(ForwardHoldCapture):
    def require_open(self,command):
        if not np.array_equal(command,candidate()['start']):raise RuntimeError('Support-first placement pose mismatch')
    def place(self,bus,command,report,guard):
        print('PLACE: 초록 위/주황 손목ID41 방향. 손을 뗀 뒤 두 번째 SPACE.',flush=True)
        print('첫 회전 -> 중지 빼기 -> 엄지 이송/뒤쪽 접촉 -> 끝마디 먼저 -> 다음 마디 -> 유지 관찰입니다.',flush=True)
        NewPairGraspCapture.place(self,bus,command,report,guard)
        report.update(placement_orientation='GREEN up; ORANGE wristward',second_rotation_included=False,
            physical_transfer_success=False,physical_upper_clearance_verified=False,final_left_alignment_verified=False,
            automatic_open=False,automatic_inverse=False,no_live_SAC=True,support_first_phases_seen=[])
    def rotation_frame(self,bus,command,report,state):
        if not getattr(self,'first_turn_notice',False):
            self.first_turn_notice=True;print('첫 회전 뒤 기존10초 유지. 이어서 받침 우선 파지 전환입니다.',flush=True)
    def transfer_frame(self,bus,command,report,phase):
        seen=report.setdefault('support_first_phases_seen',[])
        if phase not in seen:
            seen.append(phase);report['camera_instruction']=NOTICES[phase]+' 큐브를 만지지 말고 관찰. 이상하면 Q.'
            print(NOTICES[phase]+' 이상하면 Q.',flush=True)
        # Video is continuously recorded by the existing camera worker. No
        # extra blocking photo or servo write is added to moving phases.
    def finish(self,bus,command,report):
        phases=report.get('support_first_phases_seen',[])
        if (not report.get('cube_placement_confirmed') or not all(p in phases for p in NOTICES)
                or not np.array_equal(command,candidate()['goals'][-1]) or bus.pending):
            raise RuntimeError('Support-first queue/checkpoints incomplete')
        observe=self._observer(bus,command,report,self.guard)
        _,photo=self._photo('cube_tip_first_hold.jpg','tip_first_final_observation',observe)
        report.update(support_first_final_photo=photo,physical_transfer_success=False,physical_upper_clearance_verified=False,
            final_left_alignment_verified=False,camera_instruction='관찰 종료. 큐브 아래를 받치고 마지막 SPACE=OFF. 종료 뒤 제거.')
        print('관찰 종료. 큐브 아래를 받치고 마지막 SPACE=OFF. 종료 뒤 큐브를 제거하세요.',flush=True)
        state=self._space('support_cube_for_off',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_support_confirmed_before_off=True,
            cube_removal_confirmed=False,cube_present=True)
