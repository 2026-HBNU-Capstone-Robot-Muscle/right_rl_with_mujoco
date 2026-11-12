"""Current handoff stays non-executable: placement and release are still unresolved."""
import json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/four_forward_review';report=json.loads((O/'rotation_hold_research.json').read_bytes())
section='''<!-- model-results-summary-20261030 --><article><h2>추가 모델 검사 결과</h2><p><b>배치 민감도가 확인됨.</b> 기준 배치에서는 회전 후 하부 재파지와 8초 고정이 마찰과 강성 및 받침 위치 27조건에서 통과했다. 하지만 초기 위치를 한쪽으로 5mm 옮긴 조건에서는 약 62도였던 회전이 약 10도로 줄었고 두 손끝의 접촉이 없어 고정에 실패했다. 이 결과는 실물의 정확한 위치 오차를 측정한 것은 아니며, 사용자의 배치만 원인이라고 확정할 수 없다.</p><p>새 후보는 엄지의 설계 접촉점을 16mm, 중지는 4mm 낮춰 큐브 아래쪽을 잡는다. 명목 모델의 고정 시험에서 ±0.01N·m 설계 토크를 가했을 때 최대 각도 변화는 약 0.31도였다. 이 토크는 실제 상단 레이어의 회전 부하를 측정한 값이 아니다. 실제 파지력과 90도 반복 회전은 미검증이다.</p><p><b>놓기 단계는 아직 완료되지 않음.</b> 일반 OPEN은 명목 모델에서 약 13도 되비틀림을 만들었다. 접촉을 먼저 푸는 새 후보는 약 1.7도로 줄었지만 잔여 접촉이 남았다. 따라서 새 실물 실행은 아직 제공하지 않는다. 상단 작업 중에는 오른손의 고정 상태를 유지하고, 작업이 끝난 뒤 놓기를 별도 단계로 처리할 예정이다.</p><div class="photos"><figure><img src="model_views/after_turn_before_regrasp_front.png"><figcaption>회전 직후 명목 모델. 빨강은 엄지, 초록은 중지.</figcaption></figure><figure><img src="model_views/lower_body_held_front.png"><figcaption>엄지와 중지를 낮춰 다시 잡은 실제 시뮬레이션 상태.</figcaption></figure></div><p><small>회색 큐브는 형상만 보여주는 접촉 모델이다. 이 그림의 색으로 실물 배치 방향을 정하지 않는다. 실물 기준은 여전히 초록 위, 주황 손목 ID41 쪽이며 아래의 성공한 실물 사진을 참고한다. 렌더링 색만 변경했고 물리 형상과 힘 및 PID는 유지했다.</small></p><p>현재 단계: 연결 확인 후 현재 관절값 읽기 → 성공 배치 기준으로 위치 안내 보완 → 회전 전 접촉 확인 → 회전과 하부 고정 및 명시적 놓기의 실물 시험 준비.</p></article>'''
p=O/'index.html';s=p.read_text(encoding='utf-8');marker='<!-- model-results-summary-20261030 -->'
if marker not in s:s=s.replace('<h2>이번 네 번의 실제 기록</h2>',section+'<h2>이번 네 번의 실제 기록</h2>')
s=s.replace('자유물체 동역학과 고정 검사는 진행 중이다.','기준 배치에서 자유물체 동역학과 고정을 확인했지만, 배치 오차와 놓기 단계에는 보완이 필요하다.').replace('사진은 저장 원본이다.','실물 사진은 저장 원본이다.');p.write_text(s,encoding='utf-8')
for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261030/single_forward/handoff.json'):
 p=R/rel;data=json.loads(p.read_bytes());data.update(status='placement_sensitive_rotation_and_lower_body_hold_research',research_report='right_hand_rotation_20261030/four_forward_review/rotation_hold_research.json',operator_review='right_hand_rotation_20261030/four_forward_review/index.html',next_command=None,user_command=None,ready=False,new_motion_ready=False,user_action_required=True,pending_user_action='confirm existing5V motor power and connection; latest PING ID1 did not respond',latest_readonly_attempt='fast_off_readonly_20261030_062953_077410/result.json',physical_rotation_repeatability_verified=False,physical_body_hold_verified=False,assistant_motor_writes=0);p.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
notes='''

2026-10-30 회전과 몸통 고정 후속 검토

- 최신 네 실행의 실제 사진과 지령을 비교했다. 05:26은 회전 후 주황 앞면이 흰 면으로 바뀌었고, 05:31은 지령을 완주했지만 큐브 이동 및 기울어짐과 약 6.7도 카메라 격자 회전만 확인됐다. 나머지 두 실행은 회전 중 취소 및 실제 동작 전 모델 검사 중단이었다.
- 하부 재파지 후보는 기준 배치의 모델 27조건에서 고정 과제를 통과했다. 모델 회전은 약 61~64도로, 90도 성공 인증을 뜻하지 않는다. 초기 위치 +5mm 조건은 약 10도와 접촉 상실로 실패했다. 고정된 지령의 배치 민감도 보완이 필요하다.
- 오른손 방향 변경 → 하부 재파지와 몸통 고정 → 왼손 상단 레이어 작업 → 다음 방향 변경과 고정으로 단계를 분리한다. 상단 작업 중 자동 OPEN을 넣지 않는다.
- 일반 OPEN은 명목 모델에서 약 13도 되비틀림을 만들었다. 접촉 해제 후보는 약 1.7도로 줄였지만 잔여 접촉이 남아 미완료다. 새 실행 명령은 없다. 실물의 고정과 90도 반복 재현 및 상단 작업 연계는 미검증이다.
- 현재 자세 읽기는 fast_off_readonly_20261030_062953_077410에서 ID1 PING 응답 없음으로 중단됐다. 실제 목표와 토크 쓰기는 0회다. 사용자에게 기존5V 전원과 연결 확인을 요청했고 현재 각도 읽기가 다음 단계다.
- 결과와 그림: right_hand_rotation_20261030/four_forward_review/index.html
'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
 p=R/name;s=p.read_text(encoding='utf-8');tag='2026-10-30 회전과 몸통 고정 후속 검토'
 if tag not in s:p.write_text(s+notes,encoding='utf-8')
(O/'handoff.json').write_text(json.dumps(dict(status='awaiting_motor_power_for_current_readonly_pose',research_report='right_hand_rotation_20261030/four_forward_review/rotation_hold_research.json',latest_readonly_attempt='fast_off_readonly_20261030_062953_077410/result.json',next_motion_command=None,ready_for_motion=False,assistant_motor_writes=0,remaining=['placement alignment and verified contact','physical repeatable reorientation','physical lower-body hold','release without cube drag','upper-layer tool handoff']),indent=2)+'\n',encoding='utf-8');print('Published findings; old motion pointers remain retired.')
