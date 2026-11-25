"""Record the concrete next operator action without touching poster/source data."""
import datetime as dt,hashlib,json
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,ROOT,OUT,REQUEST,PROOF,LOADED_REVIEW,STATIC_REVIEW
from horizontal_quarter_gate_20261027 import validate_review

def main():
    plan=HorizontalQuarterPlan(REQUEST);review=validate_review(plan,PROOF);loaded=json.loads(LOADED_REVIEW.read_bytes());stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat();video='right_hand_rotation_20261027/pair_free_yaw_optimization_v6/candidate_video/model_yaw_diagnostic_3x.mp4'
    # The video is the nominal optimizer, not the final conservative operator
    # path. Keep the two descriptions distinct in every handoff.
    state=dict(status='horizontal_quarter_operator_experiment_ready',timestamp=stamp,legacy_reference_usage='right_hand_rotation_20261027/legacy_reference_usage.md',legacy_source_pose_count=7,legacy_actual_runs_compared=2,operator_model_review=PROOF.relative_to(ROOT).as_posix(),operator_model_nominal_yaw_before_open_deg=review['nominal_model_yaw_deg'],operator_model_response_yaw_range_deg=review['model_yaw_range_deg'],optimizer_nominal_yaw_deg=90.77004440647585,optimizer_video=video,optimizer_video_is_not_final_operator_path=True,final_operator_model_video='right_hand_rotation_20261027/horizontal_quarter_trial_v1/model_video/model_yaw_diagnostic_3x.mp4',device_free_tests_passed=24,static_loaded_cases=201,current_recovery_cases=210,loaded_physics_cases=27,model_responses_vary=True,robust_90_degree_rotation_verified=False,physical_horizontal_90_verified=False,horizontal_quarter_turn_verified=False,arbitrary_direction_rotation_verified=False,physical_forward_flip_successes=2,next_user_command='leap_horizontal_quarter_trial.cmd',next_user_task='EMPTY first SPACE; PLACE GREEN up ORANGE toward WRIST/ID41, hands clear second SPACE; automatic grasp/large turn/open; REMOVE before third SPACE. Send done only.',next_user_command_ready=True,next_motion_command_ready=True,rotation_execution_ready=True,hardware_execution_allowed=False,hardware_execution_requires_user_started_command_and_spaces=True,assistant_motor_writes=0,assistant_torque_or_setting_writes=0,assistant_serial_ping_read_only_performed=True,latest_readonly='repeat_off_readonly_20261027_133354_321429/result.json',latest_readonly_stable_drift_ticks=1,pending_user_input=None,active_motion_limit_s=300,nominal_active_seconds=len(plan.plan(plan.source_position()))*.075)
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        path=ROOT/name;old=json.loads(path.read_bytes());old.update(state);path.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    guide=f'''# 수평 90도 목표 실물 확인

예전 `sequence_flip.py`의 7개 자세와 10월 11일 실행 두 건을 참고해 새 회전 경로를 만들었다. 같은 파지 확인만 반복하는 실행이 아니다. 준비 자세에서 큐브를 잡은 뒤 큰 수평 회전 목표를 보내고 3초 관찰한 다음 자동으로 연다.

현재 관절값은 3회 읽기에서 최대 1틱 차이, 최소 4.6V, 최고 19도였다. 손가락을 예전 모양으로 맞출 필요 없다. 실제 시작 때 다시 읽은 각도로 빈손 복귀를 확인하며, 회전 경로는 고정된 검토 자료를 사용한다.

1. 큐브를 손과 카메라 밖에 둔 상태로 아래 명령을 실행한다. 카메라는 자동으로 열린다. 터미널 첫 SPACE를 누른다.
2. `PLACE CUBE`가 나온 뒤 엄지와 중지 사이 손바닥과 손가락 뿌리에 큐브를 자연스럽게 받친다. 초록은 위, 주황은 손목 ID41 쪽이다. 사람 손을 뺀 뒤 터미널 두 번째 SPACE를 누른다.
3. 파지, 큰 수평 회전, 3초 유지와 자동 열기까지 기다린다. `OPEN complete / REMOVE CUBE` 안내 뒤 큐브를 빼고 터미널 세 번째 SPACE를 눌러 OFF한다. Q, Esc, Ctrl+C는 중단이다.
4. 완료나 중단 어느 쪽이든 `했어`만 보내면 저장 결과와 영상을 확인한다. 로그 전체를 복사할 필요 없다.

```powershell
& "C:\\Users\\김준서\\Documents\\Codex\\2026-09-23\\files-mentioned-by-the-user-leap\\outputs\\leap_horizontal_quarter_trial.cmd"
```

이동 자체는 약 4분 10초이며 배치와 제거 대기는 별도다. 더 긴 경로의 동작 예산만 300초로 정했다. 450mA와 원래 PID, 목표 2틱, 전압과 전류 및 추종 중단 기준은 유지했다. 처음 SPACE 뒤 현재 빈손 복귀만 다시 모델 검사할 수 있으며 이때 모터는 OFF다.

빈손 복귀 210개, 회전 구간의 관절 오차 기하 201개, 파지부터 회전과 열기까지 자유 큐브 물리 27조건을 확인했다. 최종 실행 후보의 명목 회전은 열기 전 약 {review['nominal_model_yaw_deg']:.1f}도다. 손 응답과 마찰 가정에 따라 약 {review['model_yaw_range_deg'][0]:.1f}~{review['model_yaw_range_deg'][1]:.1f}도로 달라져 매번 90도 회전하는 신뢰성이 입증된 상태는 아니다. 실물 각도는 저장 영상으로 확인해야 한다. 자동 열기 때 일부 방향이 되돌아오는 모델 결과도 기록했다.

90.8도 영상은 자유 큐브 최적화의 한 조건을 3배속으로 재생한 비교 영상이다. 뒤의 여유 조정을 포함한 최종 실행 경로와 동일하지 않으며 실물 성공 영상도 아니다. 실시간 카메라 피드백이나 SAC 온라인 학습을 실행하는 프로그램은 아니다.
'''
    (OUT/'operator_guide.md').write_text(guide,encoding='utf-8');(OUT/'readiness.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block=f'''## CURRENT 2026-10-27: 예전 회전 기록을 반영한 큰 수평 회전 실험 준비

- 원본 `sequence_flip.py` 7자세의 해시와 10월 11일 실제 완주 2건을 다시 확인했다. 현재 열린 자세의 엄지와 중지 값은 원본 P4에서 가져왔고, 당시 P4 사진과 실제 각도에서 만든 큐브 배치 가정을 사용했다. `right_hand_rotation_20261027/legacy_reference_usage.md`와 `legacy_reference_traceability.json`에 직접 연결을 기록했다. 원본 경험을 실패로 무효화하지 않았다.
- 새 파지에서 자유 큐브와 유한 강성 액추에이터를 사용한 오프라인 최적화로 90.77도 및 3초 유지 사례가 나왔다. 영상은 `{video}`다. 이는 최적화 명목 사례이며 이후 보수적으로 조정한 최종 실행 경로와 동일하지 않다.
- 최종 후보는 관절 끝에서 여유를 두고, 엄지 압박 목표를 낮추며, 중지가 손바닥에 가까워지기 전에 뿌리를 분리한다. 원래 관절 범위와 450mA/PID 및 전압/전류/추종 기준은 유지했다. 목표 간 2틱 및 75ms를 유지하고 약 247초의 긴 경로에 맞춰 활성 동작 예산만 300초로 정했다.
- 현재 13:33 PING/READ 3회의 편차는 최대 1틱, 전부 OFF/오류0/워치독0, 최소4.6V, 최고19도다. 현재 빈손 복귀 210개와 loaded 기하 201개 및 파지/회전/자동열기 물리27조건을 통과했다. 최종 실행 후보의 명목 열기 전 회전은 {review['nominal_model_yaw_deg']:.2f}도, 전체 응답 범위는 {review['model_yaw_range_deg'][0]:.2f}~{review['model_yaw_range_deg'][1]:.2f}도다. 자동 열기에서 일부 방향이 되돌아오므로 90도 유지/재파지 연결은 별도 후속 검증이다. 강성/마찰/배치는 식별된 실측 모델이 아니다.
- 새 사용자 시작 명령은 `leap_horizontal_quarter_trial.cmd`다. 빈손 첫 SPACE, PLACE 뒤 초록 위/주황 손목ID41 쪽으로 배치하고 손 뺀 뒤 두 번째 SPACE, 자동 열기 후 큐브 제거와 세 번째 SPACE다. 카메라는 자동으로 열리며 기본 Python 실행은 무장치다. 현재각이 바뀌면 동일한 회전/열기 검토만 재사용하고 빈손 복귀는 다시 검사한다. 완료/중단 뒤 `했어`만 받는다.
- 실물 수평90도 및 임의 방향 회전은 미검증, 기존 수직 성공 총2회는 유지한다. 실제 새 모터 목표/토크/설정 쓰기는 0회다. 원본 구동/자세/관절범위와 전시 족자 파일은 그대로다.

<!-- END HORIZONTAL QUARTER READY 20261027 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig');marker='<!-- END HORIZONTAL QUARTER READY 20261027 -->'
        if marker in text:raise RuntimeError('Already published')
        path.write_text(block+text.replace('## CURRENT 2026-10-27:','## PREVIOUS 2026-10-27:',1),encoding='utf-8')
    protected={'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b','right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716','exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0'}
    for name,wanted in protected.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==wanted,name
    print(json.dumps(dict(status=state['status'],model_nominal_yaw_deg=review['nominal_model_yaw_deg'],next_user_command=state['next_user_command'],protected_hashes_unchanged=True,assistant_motor_writes=0),ensure_ascii=False))
if __name__=='__main__':main()
