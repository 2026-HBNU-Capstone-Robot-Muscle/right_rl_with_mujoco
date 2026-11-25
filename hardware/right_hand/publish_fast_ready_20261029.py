"""Publish reviewed current empty recovery and the tested faster read operator."""
import hashlib,json
from retained_slow_yaw_plan_20261029 import ROOT,OUT,PROOF,REQUEST,hashes
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
proof=read(PROOF);ready=read(OUT/'readiness.json');latest=read(OUT/'latest_fast_OFF_comparison.json')
assert proof['code_hashes']==hashes() and ready['review_sha256']==sha(PROOF)
assert ready['reference_only'] is False and latest['status']=='fresh_OFF_matches_screened_start'
assert latest['maximum_difference_ticks']<=4 and latest['cube_removed_confirmed_by_user'] is True
verification=read(OUT/'verification.json')
verification.update(status='fast_read_and_current_empty_recovery_ready',reference_only=False,
    fresh_current_empty_hand_review_pending=False,cube_removed_confirmed_by_user=True,current_empty_recovery_cases_passed=210,
    current_empty_recovery='right_hand_rotation_20261029/retained_slow_yaw_v1/recovery_fast_023256/review.json',
    latest_OFF_comparison=latest,default_entry_check='CHECK ONLY; no camera, motor port or simulator process opened',
    operator_test_cases_current_reference_passed=12,operator_UI_placement_text_clarified_after_logic_checks=True,
    review_sha256=sha(PROOF),assistant_motor_writes=0)
(OUT/'verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
command='& "'+str(ROOT/'leap_retained_slow_yaw.cmd')+'"'
guide='''고속 읽기를 반영한 수평 회전 실험

현재 빈손 자세를 다시 맞출 필요는 없습니다. 로봇 USB와 기존5V 전원, Streaming Webcam을 유지합니다.

1. leap_retained_slow_yaw.cmd 실행 후 터미널에서 첫 SPACE를 빈손으로 누릅니다.
2. 준비가 끝나 PLACE CUBE가 뜨면 빨강 중앙 ID2가 위, 초록 중앙 ID5가 손목 ID41 쪽으로 놓습니다.
   손바닥 ID40 옆의 엄지 쪽 공간에 예시 사진처럼 놓습니다. 비교 그림의 왼쪽 사진을 참고하세요.
   비교 그림: placement_comparison.svg
   실제 예시 사진: ../../pitch_slow_yaw_20261029_004532_520638/cube_transfer_hold.jpg
3. 큐브가 스스로 놓이면 사람 손을 뺀 뒤 두 번째 SPACE를 누릅니다.
4. 잡기와 느린 수평 회전, 자동 OPEN을 기다립니다. REMOVE가 뜨면 큐브를 빼고 세 번째 SPACE로 OFF합니다.

명목 동작240.45초, 약4~5분입니다. Q/Esc/Ctrl+C는 중단합니다.
이번에는 뒤집기를 생략합니다. 수평 회전량은 저장 영상으로 확인합니다.

Fast Sync Read로8개씩 두 회신을 읽고,16개 상태가 모두 있어야 다음 단계로 진행합니다.
실제 읽기 전용 비교 중앙값34.74ms에서19.35ms, 오류 뒤 두 번의 전체 재확인은 약61~63ms였습니다.
통신속도1Mbps와 모터 설정, 원래 관절 목표 및 간격, 전류와 PID, 전압/온도/추종 기준을 유지합니다.
전체200ms와 정상 상태2회 확인도 유지합니다. 목표나 설정 명령은 다시 보내지 않습니다.
공식 설명: https://emanual.robotis.com/docs/en/dxl/protocol2/#fast-sync-read-0x8a

현재 빈손 복귀210개 모델 조건을 통과했고 최근 읽기와 검사 자세의 차이는 최대1틱이었습니다.
실제 수평90도와 연속2축 완주는 아직 확인 전입니다. 동작 중 통신 안정성은 이번 영상과 기록으로 확인합니다.
끝나면 했어라고 알려주면 저장 파일을 확인합니다.
'''
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    p=ROOT/name;d=read(p);d.update(status='ready_current_empty_fast_read_yaw_20261029',current_status='ready_current_empty_fast_read_yaw_20261029',
        next_physical_command_ready=True,next_command=command,
        next_step='User starts same command: EMPTY first SPACE; PLACE RED center up/GREEN center wristward in pictured thumb-side space; second SPACE; REMOVE then third SPACE',
        next_ready=ready,next_verification=verification,cube_removal_confirmation_pending=False,
        cube_removed_confirmed_by_user=True,current_empty_recovery_review_pending=False,
        latest_OFF=latest,model_recovery_cases_passed=210,assistant_motor_writes=0)
    p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
for path,digest in {'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}.items():assert sha(ROOT/path)==digest,path
notice=('2026-10-29 고속 읽기 반영 후 현재 빈손 출발점까지 준비 완료. 사용자가 큐브 제거를 확인했다. '
    '기존 일반 읽기는 중간에 누락됐으나 검증된 고속 읽기로3회 같은 관절값과 토크 OFF를 확보했다. '
    '현재 복귀210개 모델 조건이 통과했고 재확인한 세 상태는 검사 출발점과 최대1틱 차이였다. '
    '실행 파일의 현재 출발점 검사와 기본 무장치 실행을 통과했다. '
    '앞서 사용량 한도로 미실행된 시뮬레이션은 안내된 시각 이후 승인 재검토를 통과해 정상 완료했다. '
    '다음 명령은 같은 leap_retained_slow_yaw.cmd다. 빈손 첫 SPACE, PLACE 뒤 빨강 중심 ID2가 위와 초록 중심 ID5가 손목 ID41 쪽, '
    '예시 사진의 엄지 쪽 공간에 놓고 두 번째 SPACE, REMOVE 뒤 제거하고 세 번째 SPACE다. '
    '명목240.45초이며 실제 수평 회전은 다음 실물 확인 단계다. 임의 모터 쓰기는 실행하지 않았다.\n\n')
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if notice not in old:p.write_text(notice+old,encoding='utf-8')
print(json.dumps(dict(status='current_fast_yaw_ready_published',command=command,nominal_seconds=ready['nominal_motion_seconds'],
    current_reference_maximum_difference_ticks=latest['maximum_difference_ticks'],model_cases_passed=210,
    guide=str(OUT/'실행안내.md'),assistant_motor_writes=0),ensure_ascii=False))
