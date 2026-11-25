import json,re
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/placement_alignment_capture';h=json.loads((O/'handoff.json').read_bytes());assert h['status']=='placement_alignment_photo_trial_ready' and not h['grasp_or_rotation_trial_ready']
page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>큐브 배치 비교 촬영</title><style>body{max-width:1100px;margin:24px auto;padding:0 18px;font:18px/1.7 system-ui;color:#202d39;background:#f5f7fa}article{background:white;padding:22px;border-radius:12px;margin:18px 0}h1{font-size:29px}img{max-width:100%;height:auto}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#e8eef5;padding:14px}li{margin:10px 0}a{color:#195ea0}.note{border-left:6px solid #c89b21}</style><h1>큐브를 놓는 위치부터 맞추기</h1><article class="note"><p>모델에서 5mm의 배치 차이로 회전과 접촉이 크게 달라졌다. 이번 실행은 이전에 실제로 돌았던 배치와 현재 배치를 비교할 사진을 확보한다. 잡기와 회전은 실행하지 않는다.</p><p>준비 자세가 자동으로 만들어진 뒤 비교한다. 손가락을 미리 사진처럼 맞출 필요는 없다.</p></article><article><h2>한 번 실행</h2><pre>'''+h['operator_command']+'''</pre><ol><li>큐브를 손과 카메라 화면 밖에 두고 터미널에서 첫 SPACE. 약 40초 동안 준비 자세로 움직인다.</li><li><b>PLACE CUBE</b>가 나오면 화면 오른쪽 예시처럼 배치한다. <b>초록 위, 주황 면은 손목 ID41 쪽.</b> 손바닥 옆 엄지와 중지 사이에 두고, 사람이 받치지 않아도 놓여 있어야 한다. 억지로 누르지 않는다.</li><li>손을 뺀 뒤 터미널에서 두 번째 SPACE. 사진만 저장하며 손가락 목표는 바뀌지 않는다.</li><li>큐브를 빼고 터미널에서 세 번째 SPACE. 토크 OFF와 설정 복구를 확인하고 끝난다. Q 또는 Esc로 중단할 수 있다.</li></ol><p>카메라 각도가 다르면 본체를 기준으로 큐브 위치를 비교한다. ID40이 가려져도 촬영할 수 있다. 화면의 좌우 사진은 비교용이며 3D 정렬 완료 표시가 아니다.</p></article><article><h2>오른쪽에 나오는 실제 배치 예시</h2><a href="../../single_forward_trial_20261030_052636_068402/cube_before_grasp.jpg"><img src="../../single_forward_trial_20261030_052636_068402/cube_before_grasp.jpg" alt="초록 위, 주황 면이 손목 쪽인 이전 배치 사진"></a><p>이 실행에서는 이후 주황 앞면이 흰 면으로 바뀌는 회전이 관찰됐다. 정확한 90도 성공 인증을 뜻하지 않는다.</p><h2>비교 화면 확인</h2><img src="camera_probe/comparison_preview.jpg" alt="왼쪽 현재 손, 오른쪽 이전 배치 사진의 실제 카메라 전용 점검 화면"><p>위 점검 화면의 현재 손은 준비 전 OFF 자세다. 첫 SPACE 뒤 준비가 끝났을 때 예시와 비교한다.</p></article><article><p>검증: 현재 OFF 자세의 형상 201조건과 동역학 9조건, 소프트웨어 시험 17개 통과. 기존 전류와 PID 및 상태 중단 기준을 유지한다. 큐브 배치 이후의 위치 지령 쓰기는 차단한다.</p><p>끝나면 <b>“했어”</b>만 알려주면 된다. 사진과 관절값은 자동 저장되며 로그를 길게 붙여 넣을 필요가 없다.</p><a href="../four_forward_review/index.html">앞선 네 실행과 회전·고정 모델 검토</a></article></html>'''
(O/'operator_guide.html').write_text(page,encoding='utf-8');missing=[]
for link in re.findall(r'(?:src|href)="([^"]+)"',page):
 if not (O/link).resolve().exists():missing.append(link)
assert not missing,missing
status=dict(status='placement_alignment_capture_ready',scope='placement_photo_only_no_grasp_or_rotation',next_command=h['operator_command'],user_command=h['operator_command'],ready=True,rotation_trial_ready=False,loaded_motion_ready=False,user_action_required=True,pending_user_action='run alignment capture; place cube only at PLACE; photo; remove; OFF',latest_readonly='fast_off_readonly_20261030_090421_857740/result.json',current_motor_connection_verified=True,read_only_torque_OFF_verified=True,alignment_guide='right_hand_rotation_20261030/placement_alignment_capture/operator_guide.html',assistant_motor_writes=0)
for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
 p=R/rel;data=json.loads(p.read_bytes());data.update(status);p.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
p=R/'right_hand_rotation_20261030/four_forward_review/handoff.json';data=json.loads(p.read_bytes());data.update(status);data['ready_for_motion']=True;data['motion_scope']='empty_preparation_then_fixed_goal_photo';data['next_motion_command']=h['operator_command'];p.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
banner='''<!-- alignment-capture-ready-20261030 --><aside style="padding:16px;margin:16px;background:#e7f3ed;border:2px solid #4f8768"><b>현재 다음 단계: 배치 비교 촬영.</b> 회전 실행은 대기 중이며 새 명령은 빈손 준비와 배치 사진만 수행합니다. <a href="../placement_alignment_capture/operator_guide.html">새 배치 비교 촬영 안내</a></aside>'''
for rel in ('right_hand_rotation_20261030/yaw_roundtrip/operator_guide.html','right_hand_rotation_20261030/single_forward/operator_guide.html','right_hand_rotation_20261030/four_forward_review/index.html'):
 p=R/rel;s=p.read_text(encoding='utf-8')
 if '<!-- alignment-capture-ready-20261030 -->' not in s:
  at=s.lower().find('<main');at=s.find('>',at)+1 if at>=0 else 0;s=s[:at]+banner+s[at:];p.write_text(s,encoding='utf-8')
note='''

2026-10-30 09시 연결 복구와 배치 비교 촬영 준비

- fast_off_readonly_20261030_090421_857740: 16개 PING과 설정 확인 및 OFF 상태 3회 읽기 통과, 최대 자세 차이 1틱. 위치 및 토크 쓰기 0회.
- 현재 자세에서 기존 OPEN 준비 자세까지 형상 201조건, 동역학 9조건 통과. 배치 이후에는 위치 지령을 쓰지 않는 별도 촬영 명령을 만들었다. 잡기와 회전 목표는 포함하지 않는다.
- 소프트웨어 시험 17개와 카메라 전용 비교 화면 점검 통과. 카메라 원본은1280x720이며 예시 사진은 표시 화면에만 합성된다. 모터 포트 생성은 카메라 점검에서 0회였다.
- 사용자 다음 행동: leap_placement_alignment.cmd 실행, 빈손 첫SPACE, PLACE에서 초록 위/주황 손목ID41 쪽으로 배치, 두번째SPACE 사진, 큐브를 빼고 세번째SPACE OFF. 결과는 자동 저장된다.
- right_hand_rotation_20261030/placement_alignment_capture/operator_guide.html 참조. 회전·고정 실물 시험은 아직 준비 완료 상태가 아니다.
'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
 p=R/name;s=p.read_text(encoding='utf-8')
 if '2026-10-30 09시 연결 복구와 배치 비교 촬영 준비' not in s:p.write_text(s+note,encoding='utf-8')
from placement_alignment_plan_20261030 import PlacementAlignmentPlan,REQUEST,PROOF
from placement_alignment_gate_20261030 import validate_review
validate_review(PlacementAlignmentPlan(REQUEST),PROOF)
print('Published placement-only command and guide; proof remains valid and rotation commands remain retired.')
