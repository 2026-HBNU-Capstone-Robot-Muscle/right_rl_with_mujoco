"""Publish a screened current-start cache; unchanged loaded goals and code."""
import datetime as dt,json,subprocess,sys
from pathlib import Path
import calibrated_quarter_plan_20261028 as p
from calibrated_quarter_gate_20261028 import validate_review
source=p.ROOT/'right_hand_rotation_20261028/repeatability_after_052456'
new_plan=p.HorizontalQuarterPlan(source/'request.json');new_review=validate_review(new_plan,source/'review.json')
request_bytes=(source/'request.json').read_bytes();proof_bytes=(source/'review.json').read_bytes()
history=p.DATA/'operator_before_repeatability_052456';history.mkdir(exist_ok=True)
old_request=p.REQUEST.read_bytes();old_proof=p.PROOF.read_bytes()
for current in (p.REQUEST,p.PROOF,p.OUT/'readiness.json'):
    saved=history/current.name
    if current.exists() and not saved.exists():saved.write_bytes(current.read_bytes())
try:
    for path,data in ((p.REQUEST,request_bytes),(p.PROOF,proof_bytes)):
        pending=path.with_suffix(path.suffix+'.pending');pending.write_bytes(data);pending.replace(path)
    validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
    cli=subprocess.run([sys.executable,'-X','utf8','-B',str(p.ROOT/'leap_calibrated_quarter_20261028.py')],cwd=p.ROOT,capture_output=True,text=True,encoding='utf-8',timeout=30)
    if cli.returncode or 'CHECK ONLY: no camera, motor port, or simulator process opened.' not in cli.stdout:raise RuntimeError('Updated default CLI check failed: '+cli.stdout+cli.stderr)
except BaseException:
    p.REQUEST.write_bytes(old_request);p.PROOF.write_bytes(old_proof)
    raise
observed=json.loads((p.ROOT/'right_hand_rotation_20261028/completed_calibrated_052456/physical_review.json').read_bytes())
ready=dict(status='same_prefix638_ready_for_one_repeatability_trial',timestamp=dt.datetime.now().astimezone().isoformat(),
    plan_version=new_review['plan_version'],command='& "'+str(p.ROOT/'leap_calibrated_quarter_trial.cmd')+'"',
    proof=p.PROOF.relative_to(p.ROOT).as_posix(),fresh_empty_recovery=source.relative_to(p.ROOT).as_posix(),
    previous_source_run=observed['source_run'],previous_physical_review='right_hand_rotation_20261028/completed_calibrated_052456/physical_review.json',
    held_camera_relative_yaw_estimate_deg=observed['held_camera_relative_yaw_estimate_deg'],
    after_open_camera_relative_yaw_estimate_deg=observed['after_open_camera_relative_yaw_estimate_deg'],
    near90_while_grasped_observed=True,exact_robot_frame90_certified=False,repeated_same_route_near90_verified=False,
    goal_of_next_trial='one same-route near90 repeatability observation before changing goals',
    yaw_goals=638,loaded_trajectory_unchanged=True,health_limits_unchanged=True,host_baud=1000000,
    new_current_empty_model_cases=210,reused_loaded_physics_cases=27,reused_loaded_geometry_cases=201,
    unchanged_control_code_previously_passed_tests=164,updated_default_cli_opens_no_devices=True,assistant_motor_writes=0,
    user_steps=['Leave fingers, base, camera and USB as they are; cube OUT.',
      'Same calibrated command once; FIRST SPACE in TERMINAL with EMPTY hand.',
      'At PLACE: GREEN up, ORANGE toward WRIST/ID41; hands clear, SECOND SPACE.',
      'After automatic OPEN remove cube before THIRD SPACE = OFF; send done.'],
    video_preview=observed['video_preview_4x'],video_realtime=observed['video_realtime'])
(p.OUT/'readiness.json').write_bytes((json.dumps(ready,indent=2)+'\n').encode())
(source/'readiness.json').write_bytes((json.dumps(ready,indent=2)+'\n').encode())
archive=source/'prior_pointers';archive.mkdir(exist_ok=True)
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=p.ROOT/name;old=archive/path.name
    if path.exists() and not old.exists():old.write_bytes(path.read_bytes())
    path.write_bytes((json.dumps(ready,indent=2)+'\n').encode())
text='2026-10-28 다음 사용자 작업: 같은 `leap_calibrated_quarter_trial.cmd`를 한 번 더 실행해638개 회전 목표의 재현성을 확인한다. 직전05:24 동작에서 파지 중 약88도와 OPEN 후 약82도를 관찰했으므로 회전량이나 힘을 더 바꾸지 않았다. 현재 OFF 자세는 읽기만 해 다시 확인했고, 이 자세의 빈손 복귀210조건을 통과한 근거로 기본 출발 캐시를 교체했다. 이전 요청과 심사 파일은 별도 폴더에 보존했다. 실제 구동과 설정 쓰기는 하지 않았고, 기본 CLI도 장치를 열지 않음을 확인했다. 손과 받침, 카메라와 USB 연결은 그대로 두고 빈손 첫SPACE, PLACE에서 초록위/주황손목ID41쪽 배치 후 손을 빼고 두 번째SPACE, 자동OPEN 후 큐브제거/세 번째SPACE. 완료 후 ‘했어’만 보내면 된다. 현재 근거 `right_hand_rotation_20261028/repeatability_after_052456/readiness.json`.\n\n<!-- END SAME PREFIX638 REPEATABILITY READY 20261028 -->\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=p.ROOT/name;old=path.read_text(encoding='utf-8-sig')
    if '<!-- END SAME PREFIX638 REPEATABILITY READY 20261028 -->' not in old:path.write_bytes((text+old).encode())
validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
print(json.dumps(dict(status=ready['status'],model_cases=210,loaded_goals_unchanged=True,default_cli_no_devices=True,motor_writes=0,command=ready['command'])))
