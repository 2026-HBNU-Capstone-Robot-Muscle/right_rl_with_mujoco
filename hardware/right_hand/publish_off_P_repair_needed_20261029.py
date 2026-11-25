"""Persist the actual stop and the narrowly scoped user-executed OFF repair."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
run=ROOT/'retained_slow_yaw_20261029_032447_137236';out=run/'offline_review'
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
r=read(run/'result.json');restore=read(out/'restoration_readonly_check.json')
camera=read(ROOT/'camera_fast_readonly_20261029_033422_242127/result.json')
source=ROOT/'fast_off_readonly_20261029_034117_379771/result.json';current=read(source)
assert current['status']=='off_snapshots_saved' and current['servo_writes']==0 and current['torque_changes']==0
assert r['all_torque_off_verified'] is True and restore['all_torque_OFF'] is True
assert restore['all_settings_match'] is False and restore['comparison']['p_gain']['matches'] is False
assert all(v['matches'] for k,v in restore['comparison'].items() if k!='p_gain')
action=dict(status='user_OFF_P_restoration_ready',source_sha256=sha(run/'result.json'),source=run.relative_to(ROOT).as_posix(),
    saved_original_P=restore['comparison']['p_gain']['expected'],current_P=restore['comparison']['p_gain']['actual'],
    only_write_address=84,only_write_size=2,write_packets_maximum=1,register_memory='RAM',
    torque_enable_writes=0,goal_writes=0,mode_writes=0,baud_writes=0,EEPROM_writes=0,
    requires_two_healthy_OFF_reads_before_write=True,requires_two_verified_OFF_reads_after_write=True,
    setting_write_retries=0,operator_SPACE_required=True,unit_tests_passed=8,
    files={n:sha(ROOT/n) for n in ('restore_off_P_20261029.py','leap_restore_off_P.cmd','test_restore_off_P_20261029.py')},
    assistant_motor_writes=0)
(out/'OFF_P_repair_review.json').write_text(json.dumps(action,indent=2)+'\n',encoding='utf-8')
summary=dict(status='stopped_before_grasp_settings_P_pending',actual_run=run.relative_to(ROOT).as_posix(),
    actual_run_sha256=sha(run/'result.json'),failed_phase=r['failed_phase'],error=r['error'],
    all_torque_off_verified=True,settings_restored=False,cube_removed_confirmed_by_user=True,
    original_setting_still_missing='p_gain',other_recorded_settings_readback_match=True,
    motion_or_cube_rotation_started_after_placement=False,voltage_min_v=4.9,current_peak_ma=123,
    tracking_max_rad=.10124273200045232,camera_usable_frames=673,camera_blank_frames=0,
    readonly_camera_comparison=dict(source='camera_fast_readonly_20261029_033422_242127/result.json',
        source_sha256=sha(ROOT/'camera_fast_readonly_20261029_033422_242127/result.json'),
        camera_OFF_complete=camera['blocks'][0]['complete'],camera_ON_complete=camera['blocks'][1]['complete'],
        camera_OFF_errors=len(camera['blocks'][0]['errors']),camera_ON_errors=len(camera['blocks'][1]['errors']),
        camera_frames=camera['blocks'][1]['camera_frames'],torque_OFF_throughout=True),
    root_cause_not_established=True,placement_orientation_correct_in_photo=True,
    next_action=action,latest_empty_OFF_source=source.relative_to(ROOT).as_posix(),
    latest_empty_OFF_source_sha256=sha(source),assistant_motor_writes=0)
(out/'current_stop_and_next_action.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
command='& "'+str(ROOT/'leap_restore_off_P.cmd')+'"'
for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    p=ROOT/name;d=read(p);d.update(status='OFF_P_repair_needed_before_next_motion',current_status='OFF_P_repair_needed_before_next_motion',
        next_physical_command_ready=True,next_physical_command_kind='RAM_P_restore_only_NO_MOTION',next_command=command,
        yaw_motion_ready=False,latest_retained_run=summary,cube_removal_confirmation_pending=False,cube_removed_confirmed_by_user=True,
        next_step='User runs OFF-only original P restoration and presses SPACE once; review saved result before next motion')
    p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 03:24 실행은 PLACE 뒤 카운트다운에서 중단되어 잡기와 수평 회전은 시작하지 않았다. '
    '고속 읽기 복구가 여러 차례 성공했지만 최종 정상 전체 상태2회 확인은200ms를 소진했다. '
    '토크 OFF는 확인했고 마지막 원래 설정 복구 중 일반 읽기80주소에서 통신이 실패했다. '
    '현재 읽기 결과 D1900/I0/전류상한2352/프로파일0은 기존값과 같지만 P는 시험600/450으로 남아 있고 원래2100 복구가 필요하다. '
    '사용자가 큐브 제거를 확인했다. 현재 빈손 세 상태는 안정적이며 설정 쓰기는 실행하지 않았다. '
    '토크 OFF 상태의 카메라 OFF100회와 ON500회 읽기는 오류0회였다. 모터 구동 시 오류 원인은 확정하지 못했다. '
    '다음 사용자 명령 leap_restore_off_P.cmd는 SPACE 한 번 뒤 OFF를 확인하고 RAM84주소의 P만2100으로 한 번 복구하며 '
    '토크/목표/모드/baud/EEPROM 쓰기는 패킷 수준에서 거부한다. 완료 후 다시2회 OFF 읽기로 확인한다. '
    '복구 검증8개 테스트와 기본 무장치 실행을 통과했다. 회전 명령의 재실행을 아직 요청하지 않는다.\n\n')
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if notice not in old:p.write_text(notice+old,encoding='utf-8')
for path,digest in {'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}.items():assert sha(ROOT/path)==digest,path
print(json.dumps(dict(status='OFF_repair_ready',command=command,tests_passed=8,assistant_motor_writes=0),ensure_ascii=False))
