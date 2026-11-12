"""Record measured completion without treating it as cube success."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'thumb_prefix_empty_20261021_190731_591017'
r=json.loads((folder/'review.json').read_text(encoding='utf-8'))
assert r['status']=='empty_thumb_prefix_complete' and r['torque_off'] and r['restored']
r['visual_review']=dict(images=['camera/frame_00152.jpg','camera/frame_00235.jpg'],
    observation='Thumb configuration changes visibly between the start of the thumb stage and the first fixed-hold frame. Most non-thumb links appear stationary in this view. Two frames alone do not establish clearance throughout the trajectory.',
    complete_motion_contact_verified=False)
(folder/'review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-21 19:07 시험: 작은 엄지 준비 동작 완료

- 사용자 실행 `thumb_prefix_empty_20261021_190731_591017` 완주, 토크 OFF 및 설정 복원 확인. 250개 카메라 프레임 저장.
- 엄지 실제 이동 ID1 +6.416°, ID2 +6.855°, ID3 -7.471°, ID4 +3.867°. 최종 목표 오차 절댓값 0.879°/0.967°/0.967°/2.021°. 뿌리 실제 이동 ID8 +4.219°, ID12 +3.604°, ID16 +3.340°.
- 전체 최저 전압 4.2V, 최대 전류 65mA, 최대 추종 오차 0.04295rad. 엄지 단계 시작과 고정 유지 시작 프레임을 직접 확인해 엄지 자세 변화 확인. 모든 순간의 접촉 없음이나 모델 정합까지 확정하지 않음.
- 이번 완료는 빈손 뿌리 상승 + 엄지 첫 구간 시험 완료. 전체 준비 자세/큐브 회전/live SAC 성공이 아님.
- 다음 검토: 기록된 영상·각도로 엄지 모델의 이동 방향 및 전체 준비 경로 비교, 다음 범위 결정. 현재 같은 시험 재실행이나 새 촬영은 요청하지 않음.
- 근거: `thumb_prefix_empty_20261021_190731_591017/review.json` 및 `camera/frames.json`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    f=ROOT/name;t=f.read_text(encoding='utf-8')
    if note.splitlines()[0] not in t:f.write_text(note+t,encoding='utf-8')
f=ROOT/'right_hand_start_20261019/objective_and_schedule.json'
x=json.loads(f.read_text(encoding='utf-8'))
x['current_user_task']='None: first bounded thumb preparation trial complete; do not repeat it without a new revision.'
x['latest_user_hardware_trial']=dict(timestamp=r['timestamp'],status=r['status'],
    evidence='thumb_prefix_empty_20261021_190731_591017/review.json',cube_rotation_verified=False)
x['current_blocker']='Small thumb preparation tracks measured commands. Full preparation and physical cube reorientation remain unvalidated; native thumb model range mismatch remains documented.'
f.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Measured thumb trial completion recorded; no hardware access.')
