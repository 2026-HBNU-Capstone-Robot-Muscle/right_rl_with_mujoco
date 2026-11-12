"""Observe both hold stages, retain torque until operator supports cube/SPACE OFF."""
import numpy as np
from forward_hold_capture_20261104 import ForwardHoldCapture
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from nominal_cycle_plan_20261112 import candidate,FIRST_HOLD,SECOND,FINAL_HOLD

class NominalCycleCapture(ForwardHoldCapture):
    def require_open(self,command):
        if not np.array_equal(command,candidate()['start']): raise RuntimeError('Cycle placement pose mismatch')
    def place(self,bus,command,report,guard):
        print('PLACE: 초록 위/주황 손목ID41 쪽. 손을 떼고 두 번째 SPACE. 전체 두 회전 진단입니다.',flush=True)
        NewPairGraspCapture.place(self,bus,command,report,guard)
        report.update(placement_orientation='GREEN up; ORANGE wristward',physical_two_turn_success=False,no_live_SAC=True,automatic_open=False,automatic_inverse=False)
    def cycle_frame(self,bus,command,report,phase):
        if phase==FIRST_HOLD and not getattr(self,'first_hold_seen',False):
            self.first_hold_seen=True
            _,photo=self._photo('cube_first_cycle_hold.jpg','first_cycle_hold_checkpoint',self._observer(bus,command,report,self.guard))
            report.update(first_cycle_hold_photo=photo,phase=phase,camera_instruction='첫 유지 뒤 다음 회전으로 이어집니다. 이상하면 Q.')
            print('첫 회전 뒤 유지. 이어서 다음 회전입니다. 이상하면 Q.',flush=True)
        if phase==SECOND: report['second_rotation_attempted']=True
    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed') or not report.get('second_rotation_attempted') or not np.array_equal(command,candidate()['goals'][-1]) or bus.pending:
            raise RuntimeError('Cycle checkpoints or final queue incomplete')
        observe=self._observer(bus,command,report,self.guard)
        _,photo=self._photo('cube_second_cycle_hold.jpg','second_cycle_hold_checkpoint',observe)
        report.update(second_cycle_hold_photo=photo,physical_two_turn_success=False,
                      camera_instruction='관찰 종료. 큐브 아래를 받치고 마지막 SPACE=OFF. 종료 뒤 제거.')
        print('큐브 아래를 받치고 마지막 SPACE=OFF. 종료 뒤 제거하세요.',flush=True)
        state=self._space('support_cube_for_off',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_support_confirmed_before_off=True,cube_removal_confirmed=False,cube_present=True)
