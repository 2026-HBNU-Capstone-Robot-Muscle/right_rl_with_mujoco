"""Fixed-goal placement/photo/removal. Never sends a position target."""
import time
import numpy as np
from leap_reference_speed_yaw_20261029 import ReferenceCapture
from held_joint_guard import HeldJointGuard
from horizontal_placement_settle_20261027 import confirm as confirm_placement
from placement_alignment_plan_20261030 import REFERENCE_IMAGE

def capture(bus,command,report,camera,keys,*,cancel,sleep=time.sleep,now=time.monotonic):
 protocol=ReferenceCapture(camera,keys,cancel=cancel,sleep=sleep,now=now);protocol.require_open(command)
 if getattr(bus,'pending',None):raise RuntimeError('EMPTY queue has not completed; no placement capture')
 report['placement_goals_locked']=True;before=sum(e.get('address')==116 for e in report.get('write_events',[]));state,_=bus.checked_sample(bus,command,report,'prepared_empty_reference');guard=HeldJointGuard(state['position'],command);observe=protocol._observer(bus,command,report,guard)
 readings=[]
 for _ in range(10):
  s=observe('placement_checkpoint');readings.append(dict(position=s['position'].copy(),goal=s['goal'].copy(),sample_index=len(report.get('samples',[]))-1))
 confirm_placement(readings,command,report)
 state,photo=protocol._photo('prepared_empty.jpg','prepared_empty_reference',observe);report.update(prepared_empty_photo=photo,prepared_empty_on_state={k:v.tolist() for k,v in state.items()},prepared_empty_ticks=state['position'].tolist(),reference_placement_image=str(REFERENCE_IMAGE))
 report.update(phase='place_cube',camera_instruction='PLACE: match YELLOW cube edges; GREEN up; ORANGE toward WRIST. SPACE=photo only')
 print('\nPLACE CUBE: 초록 위, 주황 면은 손목 ID41 쪽입니다.',flush=True)
 print('왼쪽 화면의 노란 큐브 윤곽에 실제 큐브의 모서리를 맞춰보세요. 오른쪽은 이전 배치 사진입니다.',flush=True)
 print('TARGET GAP은 예시와의 대략적인 위치 차이입니다. 작아지도록 조정하되, 누르거나 공중에서 손으로 받치지 마세요.',flush=True)
 print('ID40이 보여야 노란 윤곽이 표시됩니다. 안 보이면 오른쪽 예시를 참고해 사진만 저장해도 됩니다.',flush=True)
 print('손을 뺀 뒤 이 터미널에서 두 번째 SPACE: 사진만 저장합니다. 잡기와 회전은 실행되지 않습니다.',flush=True)
 print('맞춰 넣으려고 누르지 말고, 놓아둘 수 없으면 Q로 종료하세요. ID40은 가려져도 됩니다.',flush=True)
 protocol.PLACEMENT_TIMEOUT=120;protocol._space('place_cube',observe)
 state,photo=protocol._photo('cube_placed.jpg','place_cube',observe)
 report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),placement_goal_ticks=np.asarray(command).tolist(),placement_on_state={k:v.tolist() for k,v in state.items()},cube_placement_confirmed=True,cube_present=True,registration_verified=False,physical_grasp_verified=False,physical_rotation_verified=False)
 report.update(phase='remove_cube',camera_instruction='PHOTO SAVED. REMOVE CUBE; then SPACE in TERMINAL=OFF. No grasp/turn.')
 print('사진 저장 완료. 큐브를 빼고 세 번째 SPACE를 누르면 토크 OFF합니다.',flush=True)
 state=protocol._space('remove_cube',observe);report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_removal_confirmed=True,cube_present=False)
 after=sum(e.get('address')==116 for e in report.get('write_events',[]));report['post_placement_goal_writes']=after-before
 if after!=before:raise RuntimeError('A position target was written during fixed-goal placement')
