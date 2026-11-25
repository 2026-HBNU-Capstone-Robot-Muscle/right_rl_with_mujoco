"""Use the successful recent placement reference and ordinary automatic OPEN."""
from leap_reference_speed_yaw_20261029 import ReferenceCapture
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
class SingleForwardCapture(ReferenceCapture):
 def place(self,bus,command,report,guard):
  print('배치 기준: 초록 위, 주황 면 손목ID41 쪽. 두 번째 성공 배치 사진을 안내 페이지에서 참고하세요.',flush=True)
  print('이번에는 정회전 후 자동 OPEN과 REMOVE로 끝납니다.',flush=True)
  NewPairGraspCapture.place(self,bus,command,report,guard)
  report.update(placement_orientation='GREEN up; ORANGE wristward',placement_reference='yaw_roundtrip_trial_20261030_011416_269699',programmed_inverse=False)
