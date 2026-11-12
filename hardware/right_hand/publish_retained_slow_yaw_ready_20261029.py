"""Publish the reviewed next operator step; no devices and no poster edits."""
import json,hashlib
from pathlib import Path
from retained_slow_yaw_plan_20261029 import ROOT,OUT,REQUEST,PROOF,hashes
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
review=read(PROOF);ready=read(OUT/'readiness.json')
assert review['code_hashes']==hashes() and ready['review_sha256']==sha(PROOF)
protected={
 'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
 'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
 'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
 'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}
for path,digest in protected.items():assert sha(ROOT/path)==digest,path
verification=dict(status='offline_checks_passed',transport_regression_tests_passed=12,operator_tests_passed=9,
    test_source_hashes={n:sha(ROOT/n) for n in ('test_combined_quiet_reserve_20261029.py','test_retained_slow_yaw_operator_20261029.py')},
    default_entry_check='CHECK ONLY: no camera, motor port, or simulator process opened',protected_hashes=protected,
    empty_cases=210,isolated_hand_cases=9,physical_retained_red_yaw_verified=False,assistant_motor_writes=0)
(OUT/'verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
guide='''이번 실행은 수평 구간만 확인합니다.

1. 큐브를 손 밖에 둔 채 손가락과 받침은 그대로 둡니다. USB, 기존5V 전원, Streaming Webcam을 연결합니다.
2. leap_retained_slow_yaw.cmd를 실행하고 터미널에서 첫 SPACE를 누릅니다. 빈손으로 준비 자세까지 이동합니다.
3. PLACE CUBE가 뜨면 빨강이 위, 초록이 손목 ID41 쪽으로 놓습니다.
   위치 사진: ../../pitch_slow_yaw_20261029_004532_520638/cube_transfer_hold.jpg
   사진의 손바닥 ID40 옆 손가락 뿌리 쪽 받침 위치를 사용합니다. 큐브가 손으로 받치지 않아도 놓여야 합니다.
4. 사람 손을 뺀 뒤 터미널에서 두 번째 SPACE를 누릅니다. 잡기, 느린 수평 회전, 자동 손 펴기가 이어집니다.
5. REMOVE CUBE가 뜨면 큐브를 빼고 세 번째 SPACE를 눌러 OFF합니다. 약4~5분이며 Q/Esc/Ctrl+C는 중단합니다.

뒤집기는 이전 실행에서 초록 위에서 빨강 위로 바뀌는 모습이 관찰됐습니다. 이번에는 빨강 위에서 시작합니다.
새 수평 파일에는 실제 조용해지는 시간을 반영하는 통신 재확인 수정이 적용됐습니다.
200ms 전체 한도, 정상 전체 상태2회 확인,450mA, 기존PID와 전압 및 추종 기준은 유지합니다.
이전7~8분 통합 파일에는 이 수정이 아직 적용되지 않았으므로 이번에는 새 수평 파일을 사용합니다.

이 실행은 감독하에 진행하는 실험입니다. 실제90도와 연속2축 완주는 아직 확인되지 않았습니다.
넓은 마찰과 구동 조건에서 모델 실패 사례가 있어 안정적인 제어 완성을 뜻하지 않습니다.
끝난 뒤에는 로그 전체 대신 했어라고 알려주면 저장 파일을 확인합니다.
'''
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
stop=read(ROOT/'pitch_slow_yaw_20261029_004532_520638/offline_review/physical_observation_review.json')
stop['cube_removed_confirmed_by_user']=True
stop['subsequent_readonly_result']='prefix638_off_readonly_20261029_005950_947910/result.json'
stop['next_ready_command']='leap_retained_slow_yaw.cmd'
(ROOT/'pitch_slow_yaw_20261029_004532_520638/offline_review/physical_observation_review.json').write_text(json.dumps(stop,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
command='& "'+str(ROOT/'leap_retained_slow_yaw.cmd')+'"'
for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json',
    'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    p=ROOT/name;r=read(p)
    for k in list(r):
        if 'next' in k.lower() and 'command' in k.lower():r[k]=command
    r.update(status='ready_for_supervised_retained_red_slow_yaw_20261029',current_status='ready_for_supervised_retained_red_slow_yaw_20261029',
        latest_combined_run=stop,next_physical_command_ready=True,next_command=command,
        next_step='Only slow horizontal yaw; EMPTY first SPACE, PLACE RED up/GREEN toward wrist ID41, second SPACE, REMOVE/third SPACE',
        next_review=str(PROOF.relative_to(ROOT)),next_guide=str((OUT/'실행안내.md').relative_to(ROOT)),
        next_ready=ready,cube_removed_confirmed_by_user=True,combined_physical_completion_verified=False,
        scoped_retry_fix_in_new_command_only=True)
    p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 다음 실물 확인 준비 완료: leap_retained_slow_yaw.cmd로 수평 구간만 확인한다. '
    '사용자가 큐브 제거를 확인한 뒤3회 읽기에서 토크 OFF와 최대1틱 안정 상태를 확인했다. '
    '현재 빈손 복귀201개 정적 조건과9개 동적 조건, 분리된 수평 경로의 손 동작9개 조건을 통과했다. '
    '새 명령에 통신 복구의 선예약 계산 수정이 반영됐고 회귀 테스트12개와 실행 절차 테스트9개가 통과했다. '
    '첫 SPACE는 빈손, PLACE 이후 빨강 위와 초록 손목 ID41 쪽으로 놓고 두 번째 SPACE, '
    '자동 OPEN 뒤 REMOVE에서 큐브를 빼고 세 번째 SPACE다. 명목251.1초와 기존300초 구동 한도를 사용한다. '
    '실제 수평 회전량과 연속2축 완주는 확인 전이다. 이전 통합 명령의 복구 코드는 그대로이므로 새 수평 명령을 사용한다.\n\n')
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if notice not in old:p.write_text(notice+old,encoding='utf-8')
print(json.dumps(dict(status='next_step_published',command=command,guide=str(OUT/'실행안내.md'),tests_passed=21,
    model_cases_passed=219,poster_and_existing_limits_unchanged=True,assistant_motor_writes=0),ensure_ascii=False))
