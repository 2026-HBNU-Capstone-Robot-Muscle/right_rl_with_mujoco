"""Preserve actual zero-write camera failure and reviewed photo-mode recovery."""
import datetime,hashlib,json
from pathlib import Path
from leap_quarter_support_photo_20261025 import require_mode_review,require_offline_review
from support_photo_path_plan_20261025 import FIXTURE
from support_photo_contact_resolution_20261025 import metadata
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/rigid_contact_review_20261025'
def load(p):return json.loads(Path(p).read_bytes())
def save(p,r):Path(p).write_text(json.dumps(r,indent=2,ensure_ascii=False),encoding='utf-8')
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    require_mode_review();proof=require_offline_review();mode=load(OUT/'photo_mode_review.json')
    assert mode['total_tests_passed']==19
    source=ROOT/'quarter_support_photo_20261025_214901_644144/result.json';actual=load(source)
    assert actual['write_events']==[] and actual['all_torque_off_verified'] and 'Camera image stayed black/blank' in actual['error']
    baseline=load(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json');poster='exhibition_20261023/족자_붙여넣기.txt'
    assert digest(ROOT/poster)==baseline['production_hashes_recorded_without_reissuing_certificates'][poster]
    assert digest(ROOT/'right_hand_rotation_20261023/native_motor_limits.json')=='c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716'
    assert digest(Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'))=='008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'
    now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat()
    r=dict(timestamp=now,actual_failure_source=source.relative_to(ROOT).as_posix(),actual_failure_sha256=digest(source),
        actual_error=actual['error'],actual_failure_phase=actual['phase'],actual_motor_writes=0,actual_torque_off_verified=True,
        actual_settings_not_modified=True,reviewed_last_nonblank_image='quarter_support_photo_20261025_214901_644144/camera/frame_00191.jpg',
        visual_review='Hand crosses/occludes the camera lens before the final blank frame; not a confirmed mechanical fault.',
        off_camera_adjustment_wait_seconds=10,on_camera_blank_stop_seconds_unchanged=.5,
        configure_or_enabled_phase_cannot_use_adjustment_pause=True,fresh_empty_evidence_frames_after_recovery=10,
        current_pose_model_fixture=FIXTURE.relative_to(ROOT).as_posix(),static_cases=len(proof['static_screens']),native_cases=len(proof['dynamic_sensitivity']),
        initial_contact_resolution=metadata(),maximum_initial_thumb_resolution_discarded_mm=max((max(v.values(),default=0.) for v in proof['initial_resolution_discarded_maximums_by_front']),default=0.),
        extra_thumb_MCP_movement_proposal_adopted=False,photo_goals_identical_to_original_parent=True,
        tests_passed=19,photo_mode_review=mode,next_user_command='leap_quarter_support_photo.cmd',next_user_command_ready=True,
        next_user_action='Set SIDE camera angle and clear lens BEFORE FIRST SPACE; FIRST SPACE EMPTY preparation; PLACE Cube then SECOND SPACE photo; REMOVE Cube before THIRD SPACE/OFF.',
        loaded_motion_in_next_command=False,physical_ninety_degree_rotation_verified=False,assistant_hardware_commands_sent=0,
        poster_unchanged=True,original_reference_and_original_range_file_unchanged=True)
    save(OUT/'camera_failure_recovery_review.json',r)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json'):
        p=ROOT/name;d=load(p);d.update(timestamp=now,next_user_command=r['next_user_command'],next_user_command_ready=True,next_user_command_photo_only=True,
            rotation_in_next_command=False,next_action=r['next_user_action'],support_photo_camera_recovery_review=r,physical_ninety_degree_rotation_verified=False);save(p,d)
    p=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json';d=load(p)
    d.update(timestamp=now,supervised_experimental_loaded_trial_ready=False,next_user_command=r['next_user_command'],next_user_command_photo_only=True,
        quarter_support_photo_mode_review=mode,support_photo_camera_recovery_review=r);save(p,d)
    note='''## CURRENT 2026-10-25: 측면 촬영 중 카메라 가림 확인 / OFF 화면 복구 대기 수정 완료

- `quarter_support_photo_20261025_214901_644144`는 준비 경로를 검사하던 중 카메라 검은 화면이 0.5초 감지돼 중단됐다. 저장된 마지막 정상 프레임 191에서는 렌즈 앞에 손이 지나가 화면 대부분을 가린 모습이 보인다. 전체 저장 193장 중 192장은 화면이 있었고 마지막 한 장은 검정이었다. 실제 모터 쓰기는 0회이며 OFF를 확인했다. 설정을 변경하기 전이므로 `settings_restored=false`는 복원 동작 실패를 뜻하지 않는다.
- 촬영 전용 카메라에 한해, 아직 모터 구성을 시작하지 않은 준비 단계에서 화면이 가려지거나 잠깐 오래되면 최대 10초 기다리도록 했다. Q와 카메라 종료는 계속 즉시 처리한다. 재개 전 신선하고 유효한 화면과 새 빈손 마커 검사 10프레임을 요구한다. 구성 시작 뒤 또는 토크 ON 이후에는 이 대기를 적용하지 않으며 기존 검은 화면 0.5초 중단과 오래된 화면 중단을 유지한다. 실제 목표 전송과 전류 및 전압 기준은 그대로다.
- 현재 저장 자세의 모델 검사에서는 준비 초기에 엄지 패드와 손바닥 근사 도형 두 이음새 사이에 최대 약 0.00621mm 접촉이 계산됐다. 촬영 전용 검사에서 이 두 쌍에만 시작 자세와 초기 분리 구간의 수치 해상도 0.01mm를 명시했다. 접촉 힘은 제거하지 않으며 그보다 깊은 접촉, 다른 쌍의 접촉과 분리 후 접촉은 그대로 거부한다. 관절 범위 및 실제 추종과 전류 중단 기준은 변경하지 않았다. 추가로 엄지를 움직이는 개발안은 채택하지 않았으며 원래 준비 목표와 같다.
- 새 촬영 전용 검사 `current_pose_v3`는 현재 저장 자세에서 정적 201개 조건과 동역학 9개 조건을 통과했다. 촬영 경계와 OFF 및 설정 복원, OFF 화면 복구와 ON 중단 유지, 수치 해상도 범위 위조 거부를 포함한 프로그램 검사 19개도 통과했다. 실제 실행 때의 신선한 관절값으로 다시 검사한다. 원래 전체 회전 프로그램과 과거 기록, 전시 문구는 보존했다. 이번 수정 과정에서 실제 장치는 열지 않았다.
- 다음 사용자 명령은 같은 **`leap_quarter_support_photo.cmd`**다. **카메라를 낮은 측면 각도로 먼저 고정하고 렌즈에서 손을 뗀 뒤 첫 SPACE**를 누른다. 첫 SPACE는 빈손 준비다. PLACE에서 큐브를 놓고 둘째 SPACE로 촬영한다. 큐브를 먼저 뺀 뒤 셋째 SPACE로 OFF한다. 적재 회전은 실행하지 않는다. 끝나면 했어만 알려주면 된다.

<!-- END SUPPORT PHOTO CAMERA RECOVERY 20261025 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8')
        if old.startswith('## CURRENT 2026-10-25: 측면 촬영 중 카메라 가림'):old=old.split('<!-- END SUPPORT PHOTO CAMERA RECOVERY 20261025 -->',1)[1].lstrip()
        p.write_text(note+old.replace('## CURRENT','## PREVIOUS'),encoding='utf-8')
    (OUT/'camera_recovery_README.md').write_text(note,encoding='utf-8')
    print(json.dumps({k:r[k] for k in ('actual_motor_writes','actual_torque_off_verified','static_cases','native_cases','tests_passed','next_user_command','assistant_hardware_commands_sent')},ensure_ascii=False))
if __name__=='__main__':main()
