"""Record one observed quarter turn/static hold, preserving its limited scope."""
import hashlib,json
from pathlib import Path
import numpy as np
from forward_hold_plan_20261104 import ForwardHoldPlan,validate_candidate

ROOT=Path(__file__).resolve().parent
F=ROOT/'forward_hold_observe_20261105_030930_516359'
OUT=ROOT/'right_hand_rotation_20261104/forward_hold/actual_review_20261105'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    r=json.loads((F/'result.json').read_bytes());a=json.loads((OUT/'orientation_and_hold_review.json').read_bytes());clips=json.loads((OUT/'video_exports.json').read_bytes())
    if a['source_result_sha256']!=sha(F/'result.json') or not a['hold_all_goals_identical'] or a['hold_actual_sample_span_seconds']<10 or not r['all_torque_off_verified'] or not r['settings_restored']:raise RuntimeError('Actual evidence failed')
    if any(sha(ROOT/c['path'])!=c['sha256'] for c in clips):raise RuntimeError('Video changed')
    baseline=json.loads((ROOT/'right_hand_rotation_20261104/resume_review.json').read_bytes())['protected_sources_unchanged']
    if not all(sha(ROOT/p)==h for p,h in baseline.items()):raise RuntimeError('Protected source changed')
    p=ForwardHoldPlan(F/'fresh_off_request.json');rows=p.plan(np.asarray(r['accepted_enable_goal'],int));expected=[q.tolist() for _,q in rows]
    writes=[e['requested'] for e in r['write_events'] if e['address']==116]
    matches=[i for i in range(len(writes)-len(expected)+1) if writes[i:i+len(expected)]==expected]
    # The original trial logs are immutable; review findings live separately.
    summary=dict(status='one_actual_forward_quarter_turn_and_fixed_hold_observed',source=F.name,source_result_sha256=sha(F/'result.json'),
        visual_observation=dict(start_top='green',start_wrist_facing_side='orange',finish_top='green',finish_wrist_facing_side='white',approximately_quarter_turn_observed=True,exact_robot_axis_90_tolerance_certified=False,static_retention_over10s_observed=True,human_hand_visible_at_robot_base=True,no_cube_contact_visible_in_reviewed_turn_and_hold_frames=True,requires_repetition_to_establish_reliability=True),
        all_goal_writes_match_generated_queue=bool(matches),queue_goals=len(expected),matching_write_offsets=matches,
        hold_seconds_observed=a['hold_actual_sample_span_seconds'],hold_goal_samples=a['hold_goal_samples'],all_hold_goals_identical=True,
        min_voltage_v=a['minimum_voltage_v'],peak_absolute_current_ma=a['peak_absolute_current_ma'],max_tracking_rad=a['maximum_tracking_rad'],
        camera_blank_frames=r['camera_recording']['image_signal_blank_frames'],all_torque_off_verified=True,settings_restored=True,
        final_on_goal_by_motor_ID_1_to16=r['final_on_state']['goal'],final_on_measured_position_by_motor_ID_1_to16=r['final_on_state']['position'],
        final_off_reference_valid=r['final_off_reference_valid'],gravity_settling_after_off_is_not_a_motion_failure=True,final_off_angles_must_not_be_reused_as_current_empty_reference=True,
        after_exit_cube_removal_not_observed=True,cube_removal_confirmed=False,
        live_sac=False,all_directions_90_verified=False,left_layer_turn_hold_verified=False,physical_grip_force_measured=False,
        assistant_motor_writes=0,protected_sources_unchanged=True,source_images_and_logs_unchanged=True,
        orientation_review=(OUT/'orientation_and_hold_review.json').relative_to(ROOT).as_posix(),videos=clips,
        next_work='Integration: preserve placement/support geometry, align left-hand contact height to actual cube; separately verify holding under upper-layer turning load. Repeatability and other directions remain unverified.',
        next_user_motion_command=None,next_user_motion_command_ready=False)
    if not matches:raise RuntimeError('Issued goal queue differs from screened plan')
    (OUT/'actual_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>실제 회전과 10초 유지 확인</title><style>body{max-width:1000px;margin:24px auto;padding:0 20px;font:18px/1.8 system-ui;background:#f5f7fa;color:#203040}article{padding:20px;background:white;margin:18px 0}video,img{width:100%;height:auto}a{color:#057}</style><h1>오른손 실제 회전과 10초 유지</h1><article><p>2026년 11월 5일 저장 실행에서 초록 윗면을 유지한 채 손목 쪽 면이 주황에서 흰색으로 바뀌었습니다. 약 4분의 1바퀴 회전한 뒤 10초 이상 같은 목표각으로 유지되는 모습을 확인했습니다. 자동으로 손을 펴거나 역회전하지 않았습니다.</p><p>토크 OFF와 설정 복구를 확인했습니다. 최소 전압 4.7V, 최대 측정 전류 107mA, 최대 추종 오차 0.0813rad입니다. 검정 프레임은 0장이었습니다.</p><p>이번 방향의 한 차례 관찰입니다. 정밀한 90도 오차 범위, 반복 성공률, 다른 방향의 회전, 왼손이 윗층을 돌릴 때의 고정력은 아직 확인하지 않았습니다.</p></article><article><h2>실제 회전과 유지 영상</h2><p>약 63.5초, 원래 시간 흐름입니다. 가속하거나 동작을 이어 붙이지 않았습니다.</p><video controls preload="metadata" src="turn_and_hold.mp4"></video><p><a href="turn_and_hold.mp4">영상 파일 열기</a> / <a href="full_run.mp4">준비와 배치를 포함한 전체 영상 3분 30초</a></p></article><article><h2>시간순 장면</h2><img src="saved_frames_contact_sheet.jpg" alt="회전 전부터 유지 종료까지 저장된 장면"><p>128.9초: 파지 유지, 140~180초: 정회전, 185~190초: 끝 자세 유지. 왼쪽 사람 손은 로봇 본체 쪽에 보입니다.</p></article><article><h2>통합 시 전달할 내용</h2><p>오른손의 받침과 손가락 뿌리 쪽 자연스러운 큐브 배치를 유지하고, 이때의 실제 큐브 높이를 왼손의 회전 접촉 높이와 맞춥니다. 최종 관절 목표는 <a href="actual_summary.json">결과 JSON</a>에 저장했습니다. 종료 후 중력으로 내려온 관절값은 다음 실행의 출발점으로 그대로 재사용하지 않습니다.</p><p>왼손의 윗층 회전 힘을 주었을 때 큐브 본체가 따라 돌지 않는지, 접촉 중 손 전체가 간섭하지 않는지는 통합 후 별도로 확인합니다. 현재 큐브 제거 여부는 종료 후 영상으로 확인되지 않아 기록에서 제거 완료로 처리하지 않았습니다.</p></article></html>'''
    (OUT/'index.html').write_text(page,encoding='utf-8')
    note='''## CURRENT 2026-11-05: 실제 정회전 후 10초 유지 관찰

- `forward_hold_observe_20261105_030930_516359` 실행이 중단 없이 완료되었고 토크 OFF와 설정 복구를 확인했다. 기록된 전체 명령을 생성된 승인 경로와 대조해 일치함을 확인했다.
- 초록 윗면을 유지한 채 손목 쪽 면이 주황에서 흰색으로 바뀌는 약 4분의 1바퀴 회전을 저장 장면에서 확인했다. 이후 134개 동일 목표 표본의 실제 시간 범위 10.156초 동안 큐브가 떨어지거나 눈에 띄게 비틀리지 않고 남아 있었다. 이번 방향의 1회 관찰이며 정확한 로봇 좌표 90도 허용오차나 반복 성공률 인증은 아니다.
- 최소 전압4.7V, 최대 절대전류107mA, 최대 추종오차0.081301rad, 최대 온도23도. 카메라1622프레임 모두 유효, 검정 프레임0장.
- 정확한 각도 비교를 위한 초기 초록 격자 적합은 부족하여 정밀 회전각 수치를 주장하지 않는다. 마지막 유지 구간의 격자 적합은 확보했으며 프레임 사이 분포를 절대 오차로 해석하지 않는다. 실제 파지력, 다른 방향 회전, 왼손 윗층 회전 하중 아래의 본체 고정은 미검증이다.
- 종료 후 큐브 제거는 아직 별도로 확인하지 않았다. OFF 직후 중력에 의한 자세 변동이 있어 최종 OFF 참조는 무효이며 다음 빈손 복귀에 재사용하지 않는다. 사람이 손가락을 다시 맞출 필요는 없고 다음 동작이 필요할 때 새 상태를 읽고 검사한다.
- 영상과 결과: `right_hand_rotation_20261104/forward_hold/actual_review_20261105/index.html` 및 `actual_summary.json`. 원본 사진과 로그, 기존 전류/PID/전압/통신/각도 제한은 변경하지 않았다. 보조자는 모터 명령을 보내지 않았다.

'''
    progress=ROOT/'professor_progress_20261015.md';old=progress.read_text(encoding='utf-8')
    if not old.startswith('## CURRENT 2026-11-05: 실제 정회전 후 10초 유지 관찰'):
        if old.startswith('## CURRENT'):old=old.replace('## CURRENT','## PREVIOUS',1)
        progress.write_text(note+old,encoding='utf-8')
    for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=ROOT/rel;d=json.loads(path.read_bytes());d.update(status=summary['status'],latest_actual_summary=(OUT/'actual_summary.json').relative_to(ROOT).as_posix(),physical_single_forward_quarter_turn_observed=True,physical_static_hold_over10s_observed=True,precise_robot_axis90_verified=False,left_layer_turn_hold_verified=False,next_user_action_required=False,next_user_command=None,next_user_command_ready=False,next_user_action='Review saved actual rotation/hold; next integrated load test needs left mechanism.')
        path.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('videos','final_on_goal_by_motor_ID_1_to16','final_on_measured_position_by_motor_ID_1_to16')},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
