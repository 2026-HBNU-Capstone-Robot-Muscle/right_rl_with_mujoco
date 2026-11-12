"""Publish the second visual reproduction and the separate cleanup finding."""
import json,hashlib,os
from pathlib import Path
import numpy as np
import review_forward_hold_actual_20261105 as visual
from forward_hold_plan_20261104 import ForwardHoldPlan
from restore_known_off_settings_20261105 import baseline
R=Path(__file__).resolve().parent;F=R/'forward_hold_observe_20261105_040030_646453';O=R/'right_hand_rotation_20261105/preintegration/repeat/actual_review_040030'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    d=json.loads((F/'result.json').read_bytes());a=json.loads((O/'orientation_and_hold_review.json').read_bytes());clips=json.loads((O/'video_exports.json').read_bytes())
    if a['source_result_sha256']!=sha(F/'result.json') or not d['all_torque_off_verified'] or not d['settings_restored'] or not a['hold_all_goals_identical'] or a['hold_actual_sample_span_seconds']<10:raise RuntimeError('Repeat evidence failed')
    if any(sha(R/c['path'])!=c['sha256'] for c in clips):raise RuntimeError('Video source changed')
    p=ForwardHoldPlan(F/'fresh_off_request.json');expected=[q.tolist() for phase,q in p.plan(np.asarray(d['accepted_enable_goal'],int))];writes=[e['requested'] for e in d['write_events'] if e['address']==116];matches=[i for i in range(len(writes)-len(expected)+1) if writes[i:i+len(expected)]==expected]
    if not matches:raise RuntimeError('Actual command queue mismatch')
    visual.F=F;visual.contact_sheet(json.loads((F/'camera/frames.json').read_bytes())['frames'],d['enabled_at'],a['phase_ranges'],O/'saved_frames_contact_sheet.jpg')
    original=baseline();final=d['final_off_snapshots'][-1];settings={k:dict(expected=original[k],actual=final[k]) for k in ('d_gain','i_gain','p_gain','current_cap','acceleration','velocity','watchdog') if original[k]!=final[k]}
    assert set(settings)=={'d_gain','p_gain'}
    recovered=d['read_recovery_confirmation_events'];assert len(recovered)==1 and recovered[0]['status']=='confirmed' and recovered[0]['clean_reads']==2 and recovered[0]['elapsed_ms']<200
    protected=json.loads((R/'right_hand_rotation_20261104/resume_review.json').read_bytes())['protected_sources_unchanged'];assert all(sha(R/n)==h for n,h in protected.items())
    fix=json.loads((R/'right_hand_rotation_20261105/preintegration/post_off_cleanup_fix/review.json').read_bytes());assert fix['software']['passed']
    summary=dict(status='second_actual_forward_quarter_turn_and10s_hold_observed_with_prior_PID_cleanup_pending',source=F.name,source_sha256=sha(F/'result.json'),
        physical_observed_forward_turn_and_static_hold_runs=['forward_hold_observe_20261105_030930_516359',F.name],physical_observation_count=2,consecutive_uninterrupted_trial_successes=False,physical_angle_tolerance_certified=False,repeatability_under_all_conditions_verified=False,
        observed_start='GREEN up; ORANGE toward wrist',observed_finish='GREEN up; WHITE toward wrist',approximately_quarter_turn_observed=True,static_retention_over10s_observed=True,
        actual_queue_matches_reviewed_path=True,queue_goals=len(expected),matching_write_offsets=matches,hold_seconds=a['hold_actual_sample_span_seconds'],hold_goals_identical=True,
        min_voltage_v=a['minimum_voltage_v'],peak_absolute_current_ma=a['peak_absolute_current_ma'],max_tracking_rad=a['maximum_tracking_rad'],camera_frames=d['camera_recording']['frame_count'],blank_frames=0,
        read_failure_events=1,recovery_confirmed_ms=recovered[0]['elapsed_ms'],goal_advance_during_read_recovery=0,
        all_torque_off_verified=True,settings_restored_to_THIS_run_entry=True,restored_to_original_pre_camera_failure_settings=False,original_settings_differences=settings,
        pending_user_command='& "'+str(R/'leap_restore_off_settings.cmd')+'"',command_scope='Original RAM settings only, torque remains OFF; no goal/mode/baud/EEPROM/torque-enable writes.',cleanup_fix=fix,
        cause_of_prior_incomplete_settings_restore='Fatal camera callback was still active in OFF settings readback; next run captured the leftover PID as its entry state.',
        physical_postoff_camera_failure_cleanup_fix_verified=False,underlying_camera_or_USB_fault_fixed=False,
        physical_body_hold_under_layer_turn_verified=False,all_directions_verified=False,cube_removal_after_exit_confirmed=False,original_logs_and_frames_unchanged=True,assistant_motor_writes=0,videos=clips)
    (O/'actual_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    first=R/'right_hand_rotation_20261104/forward_hold/actual_review_20261105/turn_and_hold.mp4';first_url=os.path.relpath(first,O).replace('\\','/')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>정회전과 10초 유지 재현</title><style>body{max-width:1000px;margin:24px auto;padding:0 18px;font:18px/1.8 system-ui;background:#f6f8fb;color:#203040}article{background:white;padding:20px;margin:18px 0}video,img{width:100%;height:auto}pre{white-space:pre-wrap;overflow-wrap:anywhere}</style><h1>정회전과 10초 유지 재현: 관찰 2회</h1><article><p>2026년 11월 5일 03:09 실행과 04:00 실행에서 초록 윗면을 유지한 채 손목 쪽 면이 주황에서 흰색으로 바뀌는 약 4분의 1바퀴 회전과 10초 이상 유지가 관찰되었습니다. 정확한 로봇 좌표 90도 오차 범위나 모든 배치의 반복 성공률 인증은 아닙니다. 두 실행 사이 통신 중단과 웹캠 중단도 있었습니다.</p><p>마지막 실행은 최소4.7V, 최대97mA, 최대추종0.0783rad, 검정프레임0장, 토크OFF 확인. 통신 오류1건은 약89.2ms 안에 정상 읽기2회로 복구했으며 목표 재전송은 없었습니다.</p></article><article><h2>두 번째 실제 회전과 유지</h2><video controls preload="metadata" src="turn_and_hold.mp4"></video><p><a href="full_run.mp4">전체 실행 영상</a> / <a href="'''+first_url+'''">첫 번째 성공 관찰 영상</a></p><img src="saved_frames_contact_sheet.jpg" alt="이 실행의 실제 단계 시간에서 선택한 회전과 유지 장면"></article><article><h2>별도로 남은 설정 정리</h2><p>직전 웹캠 중단 때 토크OFF 뒤의 PID 복구가 중단됐습니다. 마지막 실행은 그때 남은 값을 시작값으로 저장한 뒤 되돌렸으므로, 원래 P2100/D1900으로 복원됐다는 뜻은 아닙니다. 현재 토크OFF는 확인됐고, 원래 설정만 복원하는 별도 명령을 준비했습니다. 종료 뒤 카메라 오류가 설정 복원을 끊지 않도록 코드도 수정하고20개 소프트웨어 검사를 통과했습니다. 수정한 종료 처리를 실물의 카메라 오류로 다시 검증한 것은 아닙니다.</p><pre>'''+summary['pending_user_command']+'''</pre><p>이 명령은 토크를 켜거나 목표각을 보내지 않습니다. 모든 모터가OFF인지 확인한 뒤 원래RAM설정과 대조해 다른 값만 복원합니다.</p></article><article><h2>다음 기능</h2><p>윗층 회전 공간을 피하는 몸통 파지, 다른 방향의 회전 연결, 왼손 회전 하중 아래의 본체 고정은 아직 남아 있습니다.</p><p><a href="actual_summary.json">상세 결과</a></p></article></html>'''
    (O/'index.html').write_text(page,encoding='utf-8')
    note='''## CURRENT 2026-11-05: 정회전과 10초 유지 관찰 2회 / 원래 PID 복원 필요

- `forward_hold_observe_20261105_040030_646453`가 완료됐다. 첫 성공03:09과 마찬가지로 초록 위를 유지한 채 손목 쪽 면이 주황에서 흰색으로 바뀌고 마지막10.078초 동안134개의 동일 목표로 큐브가 남아 있는 것을 확인했다. 실제 전체 목표 쓰기 순서는 생성된 경로와 일치한다. 서로 독립적인 성공 관찰2회이며 중간에 통신/카메라 중단이 있어 연속 무중단 성공률 또는 정밀90도 인증으로 보지 않는다.
- 최소4.7V, 최대97mA, 최대추종0.078233rad, 영상1914프레임/검정0. 파지 중 통신 오류1건은89.185ms 안에 정상 상태2회로 확인되어 목표 재전송 없이 진행했다. 토크OFF 확인.
- 직전03:57실행은PLACE대기 때 웹캠이 검게 되어 중단됐다. 토크OFF는 확인됐으나 카메라의 오류 콜백 때문에 이후 원래 PID 복구가 끊겼다. 마지막04:00실행은 남은P600/D200(뿌리P450/D150)을 시작값으로 캡처하고 그 값으로 되돌렸으므로, 원본 `settings_restored=true`를 최초P2100/D1900 복원으로 해석하면 안 된다. 다른 복구 대상인전류2352/프로파일0/watchdog0는 원래와 일치한다.
- 새 종료 처리에서는 이미 검증된OFF 뒤 정확한시작RAM설정 복원에 한해 카메라의 실패 콜백을 분리했다. 구동 중 카메라 감시와 전압/전류/추종/통신 중단 기준, 관절 경로는 그대로다. 원래 소스와검증파일을보존하고 새 코드로 검증을 다시 결합했으며 총20개 소프트웨어검사를 통과했다. 실물의카메라오류 상황에서 새종료처리검증은아직없다.
- 현재 원래 설정 복원을 위한 사용자 명령은 `leap_restore_off_settings.cmd`다. 토크를켜거나목표각을보내지않고모든모터OFF확인뒤 원본으로검증된RAM설정만복원한다. 보조자는 실제모터쓰기명령을 보내지 않았다.
- 결과와실제영상: `right_hand_rotation_20261105/preintegration/repeat/actual_review_040030/index.html`. 새 몸통파지와윗층회전하중검증은여전히남아있다. 기존성공동작의추가반복은요청하지않는다.

'''
    progress=R/'professor_progress_20261015.md';oldtext=progress.read_text(encoding='utf-8')
    if not oldtext.startswith('## CURRENT 2026-11-05: 정회전과 10초 유지 관찰 2회'):
        if oldtext.startswith('## CURRENT'):oldtext=oldtext.replace('## CURRENT','## PREVIOUS',1)
        progress.write_text(note+oldtext,encoding='utf-8')
    for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=R/rel;v=json.loads(path.read_bytes());v.update(status=summary['status'],latest_actual_run=F.name,latest_actual_summary=(O/'actual_summary.json').relative_to(R).as_posix(),physical_observed_forward_hold_count=2,next_user_action_required=True,next_user_action='Restore original OFF RAM settings once; no motion.',next_user_command=summary['pending_user_command'],next_user_command_ready=True,original_PID_restore_pending=True,new_body_hold_operator_ready=False);path.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('videos','original_settings_differences','cleanup_fix')},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
