"""Record connection success, real/model mismatch and current READ-only silence."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    out=ROOT/'right_hand_rotation_20261027/power_branch_20261027'
    exact=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v4_rewired/exact_mesh_diagnostic/review.json'
    result=json.loads(exact.read_bytes());scan=ROOT/'baud_scan_readonly_20261027_182755_967964/result.json';baud=json.loads(scan.read_bytes())
    assert all(not row['responding_ids'] for row in baud['rates'])
    report=dict(status='post_rewire_idle_verified_model_calibration_unresolved_current_motor_power_check_needed',timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),
        connection_evidence='after_rewire_readonly_review.json',all16_previously_responded_at_3mbps=True,previous_idle_voltage_v=[5.0,5.1],
        second_fresh_readonly_source='repeat_off_readonly_20261027_163718_560361/result.json',second_read_pose_identical_to_first=True,
        actual_photo_source='rewired_contact_readonly_20261027_164011_332429/result.json',photo='rewired_contact_readonly_20261027_164011_332429/frame_02.jpg',
        photo_encoder_endpoint_drift_ticks=1,user_reports_no_actual_parts_touching=True,visible_gap_in_photo=True,metric_gap_calibrated=False,
        raw_model_initial_overlap_mm=1.3049056525334004,model_pair='cadtip_mf_rf_mf_ds_0|cadtip_mf_rf_rf_ds_0',model_mesh='distal',
        collision_visual_transforms_equal=True,exact_triangular_cad_check=exact.relative_to(ROOT).as_posix(),
        exact_triangle_collision_at_initial_pose=True,exact_cad_sampled_poses=result['sampled_poses'],
        conclusion='Disagreement is not explained by convexification alone. Do not hide it by widening collision tolerance. Inspect CAD/real kinematic or shape correspondence.',
        root_order_diagnostic='right_hand_rotation_20261027/horizontal_quarter_trial_v4_rewired/root_order_diagnostic/review.json',
        middle_first_diagnostic='right_hand_rotation_20261027/horizontal_quarter_trial_v4_rewired/middle_first_diagnostic/review.json',
        middle_first_not_an_accepted_fix=True,mid_first_step_overlap_increase_mm=.035729059174907096,mid_first_recontact_mm=1.1832723593169971,
        dependency_install_initial_review_failed_due_to_usage_limit=True,initial_review_failure_was_not_unsafe_action_judgment=True,
        retry_after_kst='2026-10-27T18:16:00+09:00',retry_after_clock_verified_kst='2026-10-27T18:27:22+09:00',
        dependency_install_retry_approved_and_completed=True,dependency='python-fcl0.7.0.11',dependency_install_report='right_hand_rotation_20261027/horizontal_quarter_trial_v4_rewired/fcl_install_report.json',
        current_host_baud_scan=scan.relative_to(ROOT).as_posix(),current_all_supported_rates_silent=True,motor_baud_writes=0,
        supported_host_scan_bps=[row['host_bps'] for row in baud['rates']],baud_question_answer='bps controls PC/motor data transfer. Keep3Mbps; GUI connection baud and EEPROM Baud Rate are different.',
        official_baud_reference='https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#baud-rate8',
        current_operator_command_ready=False,current_rewired_recovery_model_review_accepted=False,
        pending_user_input='Confirm existing5V motor supply and hub connection ON; keep cube out, do not change baud or finger posture.',
        next_offline_work='Compare the saved real hand and CAD link/zero-angle geometry. No active self-collision tolerance or model-force changes have been made.',
        physical_loaded_voltage_verified=False,physical_horizontal90_verified=False,assistant_motor_writes=0,assistant_torque_or_setting_writes=0)
    (out/'model_and_baud_handoff.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        path=ROOT/name;state=json.loads(path.read_bytes());state.update(status=report['status'],timestamp=report['timestamp'],
            latest_model_handoff=(out/'model_and_baud_handoff.json').relative_to(ROOT).as_posix(),
            latest_readonly='repeat_off_readonly_20261027_163718_560361/result.json',latest_contact_photo=report['photo'],
            next_user_action=report['pending_user_input'],next_user_task=report['pending_user_input'],pending_user_input=report['pending_user_input'],
            next_user_command_ready=False,next_motion_command_ready=False,rotation_execution_ready=False,approval_usage_block_currently_resolved=True)
        path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block='''## CURRENT 2026-10-27: 배선 뒤 정지 전압 정상, 모델 실물 차이 및 현재 전원 응답 확인 필요

-16:26 및16:37 PING/READ는16개 모두 응답했고 전부 토크OFF/오류0/워치독0이었다. 자세는 두 기록이 같고 각3회 편차1틱 이내다. 정지 전압은 이전4.4~4.9V에서5.0~5.1V로 높아졌다. 사용자는 위자드에서도 전체 연결을 확인했다고 밝혔다. 실제 배선이3+13인지8+8인지는 보고되지 않았으며 움직일 때 전압 개선은 미검증이다.
- 사용자 요구대로16:40 웹캠을 직접 촬영했다.5장 원본과 전후 READ가 `rewired_contact_readonly_20261027_164011_332429`에 있고 관절 끝점 차이는1틱이다. 사용자는 붙은 부품이 없다고 했고 사진에서도 사이 틈이 보인다. 실제 틈의mm는 보정/측정하지 않았다. 모터는 움직이지 않았다.
- 새 현재각 복귀 검사는 모델 초기부터 `cadtip_mf_rf_mf_ds_0|cadtip_mf_rf_rf_ds_0` 사이1.3049mm 겹침으로 실패했다. 두 모델 CAD는 `distal` 메쉬이며 visual과 collision의 변환은 같다. 뿌리 순서6개 모두 초기 겹침이 남는다. 중지 우선67개 진단에서도 최대 단계 증가0.0357mm와 재접촉1.1833mm가 있어 단순 순서 변경을 승인하지 않았다. 전역 허용치나 형상/관절/힘을 늘리거나 제거하지 않았다.
- python-fcl 설치는 처음 사용량 한도로 자동 승인 검토가 실패했으나 안내된18:16을 지난18:27에 재검토가 승인돼 작업 폴더 `.analysis_packages`에0.7.0.11이 설치됐다. 정밀 삼각형 검사도 초기 CAD 표면 교차를 확인했다. 따라서 이 사례를 볼록 껍질 오검출만으로 설명하거나 허용치 확대를 적용하지 않는다. 실물과 CAD 형상/각도 대응을 다음으로 확인한다. 현재 대기 원인은 설치 승인이 아니다.
- 사용자 bps 질문: bps는 데이터 통신 속도다. 기존 프로그램은3,000,000bps이고16개가 이 속도에서 응답한 기록이 있다. 위자드 연결 속도 선택과 제어표 EEPROM Baud Rate(주소8)는 다르며 후자의값5가3Mbps다. 설정은 변경하지 않는다.18:26 재조회는 ID1 응답 없음,18:27 지원7속도/ID1~16 PING에서도 모두 응답 없음이었다. COM7은 열렸다. 다음 사용자 작업은 기존5V 모터 전원과 허브 연결을 확인/켜는 것이다. 손 모양이나bps를 바꿀 필요는 없다.
- 실행 코드는 현재 `horizontal_quarter_trial_v4_rewired`의 새 원점에 바인딩되어 모델 검토 미완료이면 장치 열기 전에 거부한다. 새 동작 명령은 아직 안내하지 않는다. 고전압/전류/추종/범위/끼임 기준과 loaded 경로를 변경하지 않았다. 실물 수평90도 미검증 및 기존 수직 성공 총2회는 유지한다. 실제 목표/토크/EEPROM/설정 쓰기0회다.
- 근거: `right_hand_rotation_20261027/power_branch_20261027/model_and_baud_handoff.json`, `after_rewire_readonly_review.json`, 새 원점 아래 `exact_mesh_diagnostic`, `proxy_identity`, `root_order_diagnostic`, `middle_first_diagnostic`. 공식 통신 표는 `https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#baud-rate8`이다.

<!-- END REWIRED MODEL AND BAUD HANDOFF 20261027 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if '<!-- END REWIRED MODEL AND BAUD HANDOFF 20261027 -->' in text:raise ValueError('Already recorded')
        path.write_text(block+text.replace('## CURRENT 2026-10-27:','## PREVIOUS 2026-10-27:',1),encoding='utf-8')
    print(json.dumps(dict(status=report['status'],install_block_resolved=True,current_motor_responders=0,new_motor_command_ready=False,motor_writes=0)))
if __name__=='__main__':main()
