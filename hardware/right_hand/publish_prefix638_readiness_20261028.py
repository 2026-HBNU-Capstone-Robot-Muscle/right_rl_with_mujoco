"""Publish the tested next action, preserving raw reports and poster."""
import datetime as dt,hashlib,json
from pathlib import Path
import calibrated_quarter_plan_20261028 as p
from calibrated_quarter_gate_20261028 import validate_review
from horizontal_quarter_plan_20261027 import OUT as BASE_OUT
proof=validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
checks=json.loads((p.OUT/'calibration_tests.json').read_bytes())
baseline=json.loads((BASE_OUT/'verification.json').read_bytes())
assert checks['status']=='passed' and checks['tests']==14 and not checks['errors'] and not checks['failures']
assert baseline['tests_passed']==150 and baseline['protected_hashes']
for name,wanted in baseline['protected_hashes'].items():
    path=Path(name) if Path(name).is_absolute() else p.ROOT/name
    assert hashlib.sha256(path.read_bytes()).hexdigest()==wanted,name
command='& "'+str(p.ROOT/'leap_calibrated_quarter_trial.cmd')+'"'
ready=dict(status='calibrated_prefix638_ready_for_one_user_trial',timestamp=dt.datetime.now().astimezone().isoformat(),
    previous_run='horizontal_quarter_trial_20261028_045326_636152/result.json',
    previous_result='full_command_complete_but_physical90_target_overshot',previous_final_angle_approx_deg=128,
    previous_communications_errors=0,previous_black_frames=0,
    command=command,proof=p.PROOF.relative_to(p.ROOT).as_posix(),plan_version=proof['plan_version'],
    kept_original_yaw_goals=638,original_yaw_goals=1302,unchanged_reached_grasp=True,
    automatic_open_after_shorter_turn=True,current_empty_model_cases=210,loaded_model_cases=27,loaded_static_cases=201,
    all_model_cases=438,tests_passed=164,assistant_motor_writes=0,
    physical_shorter_route90_verified=False,nominal_model_yaw_deg=proof['nominal_model_yaw_deg'],
    nominal_model_yaw_reference='after grasp hold',observed_camera90_reference='placed cube before loaded grasp',
    actual_output_requires_saved_video_review=True,robust_repetition_not_verified=True,
    user_steps=['Leave fingers, base, camera and USB connections as they are; cube OUT.',
        'Run new command once. FIRST SPACE in TERMINAL with EMPTY hand.',
        'Only at PLACE: GREEN up, ORANGE toward WRIST/ID41. Hands clear, SECOND SPACE.',
        'Automatic OPEN follows turn/hold. REMOVE cube before THIRD SPACE = OFF.',
        'Send done; saved files will be reviewed. No full console paste needed.'],
    health_limits_unchanged=True,original_path_preserved=True,
    video_preview='right_hand_rotation_20261028/completed_quarter_045326/cube_rotation_preview_4x.mp4',
    physical_review='right_hand_rotation_20261028/completed_quarter_045326/physical_review.json')
(p.OUT/'readiness.json').write_bytes((json.dumps(ready,indent=2)+'\n').encode())
archive=p.DATA/'prior_current_pointers';archive.mkdir(exist_ok=True)
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=p.ROOT/name;old=archive/path.name
    if path.exists() and not old.exists():old.write_bytes(path.read_bytes())
    path.write_bytes((json.dumps(ready,indent=2)+'\n').encode())
old_ready=BASE_OUT/'readiness.json'
if old_ready.exists():
    old=json.loads(old_ready.read_bytes());old.update(recommended_next_operator_trial=False,
        superseded_by_readiness=(p.OUT/'readiness.json').relative_to(p.ROOT).as_posix(),
        superseded_reason='Completed full path overshot~90degrees; use separately reviewed shorter calibration once.')
    old_ready.write_bytes((json.dumps(old,indent=2)+'\n').encode())
text='2026-10-28 다음 실행 준비 완료: 04:53 전체 경로는 오류 없이 완주했으나 약128도로 지나쳤다. 저장 영상에서 배치 직후를 기준으로 약90도를 통과한638번째 목표까지 회전 구간을 줄인 별도 보정판을 준비했다. 파지 중의 회전도 이 전체 각도에 포함된다. 새 모델의 회전 구간만의 값은 파지 고정 후 기준 약62도이므로, 배치 전후 카메라 각도90도와 같은 기준으로 혼동하지 않는다. 단축판의 실물 각도와 반복성은 아직 확인 전이다.\n\n현재 빈손 복귀210조건, 단축 회전과 자동OPEN의 동적27조건 및 정적201조건을 모두 통과했고, 기존150개와 추가14개 제어 검증도 통과했다. 모터 구동이나 설정 쓰기는 하지 않았다. 기존 원본 경로와 발표용 족자 파일은 보존했다. 다음 사용자 작업은 손과 연결을 그대로 두고 새 `leap_calibrated_quarter_trial.cmd`를 한 번 실행하는 것이다. 빈손 첫SPACE, PLACE에서 초록위/주황손목ID41쪽 배치 및 손을 뺀 뒤 두 번째SPACE, 자동OPEN 후 큐브제거/세 번째SPACE. 다음에는 ‘했어’만 보내면 기록을 확인한다. 이전 전체회전 명령을 반복하지 않는다. 현재 근거 `right_hand_rotation_20261028/observed90_prefix638/operator_review/readiness.json`.\n\n<!-- END PREFIX638 CALIBRATION READY 20261028 -->\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=p.ROOT/name;old=path.read_text(encoding='utf-8-sig')
    if '<!-- END PREFIX638 CALIBRATION READY 20261028 -->' not in old:path.write_bytes((text+old).encode())
validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
print(json.dumps(dict(status=ready['status'],model_cases=438,tests=164,protected_assets_unchanged=True,command=command,physical_shorter_route90_verified=False)))
