"""Review and install only the camera/startup wrapper for the same EMPTY path."""
import hashlib,json
from datetime import datetime
from pathlib import Path
from quarter_finish_empty_plan_20261025 import ROOT,FIXTURE,QuarterFinishEmptyPlan
from quarter_finish_empty_gate_20261025 import validate_proof
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_bytes())
def write(path,value):Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    folder=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/finish_preview_fix_2328';folder.mkdir(exist_ok=True)
    proof=validate_proof(QuarterFinishEmptyPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
    source=ROOT/'finish_empty_camera_only_20261025_233351_910767/result.json';camera=read(source)
    if (camera['status']!='camera_only_preview_verified' or camera['motor_port_opened'] or camera['motor_goal_writes'] or camera['torque_changes']
            or camera['first_packet_present_before_require'] or not camera['pending_first_space_accepted'] or camera['camera_recording']['status']!='complete'
            or not camera['camera_first_space_waits'][0]['resumed']):raise ValueError('Live camera race replay incomplete')
    attempts=read(ROOT/'right_hand_rotation_20261024/support_photo_review_221703/finish_empty_2328_attempts_review.json')
    if len(attempts)!=4 or any(s['phase']!='preview' or s['write_count'] or s['sample_count'] or s['error']!='Wait for a fresh preview; no motor writes' for s in attempts):raise ValueError('Failed attempt evidence mismatch')
    stamp=datetime.now().astimezone().isoformat();wrapper=ROOT/'leap_quarter_finish_empty_preview_fixed_20261025.py';module=ROOT/'quarter_finish_empty_camera_20261025.py'
    review=dict(timestamp=stamp,root_cause='A SPACE already in the terminal buffer could reach require_fresh before the asynchronous first camera packet. RuntimeError was treated as fatal rather than pending preview.',
        failed_attempts=[dict(folder=s['folder'],source_sha256=s['source_sha256']) for s in attempts],all_failed_before_hardware_authorization=True,
        failed_attempts_motor_port_opened=False,failed_attempts_motor_goal_writes=0,
        off_verification_and_settings_restore_false_reason='Motor bus was never opened and no settings were changed; these defaults do not show failed torque shutdown/restoration.',
        wrapper_sha256=sha(wrapper),camera_module_sha256=sha(module),native_motion_proof_sha256=sha(FIXTURE/'screen.json'),
        program_tests_passed=11,test_source_sha256=sha(ROOT/'test_finish_empty_preview_wait_20261025.py'),
        motor_path_changed=False,configuration_and_on_camera_stops_unchanged=True,first_space_pending_maximum_seconds=5.,
        parent_motor_path_proof_independently_revalidated=True,native_static_cases=len(proof['static_screens']),native_dynamic_cases=len(proof['dynamic_sensitivity']),
        motion_proof_reissued=False,live_camera_only_probe=dict(source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),
            first_packet_present_before_require=False,resumed_after_seconds=camera['camera_first_space_waits'][0]['elapsed_seconds'],
            usable_saved_frames=camera['camera_recording']['image_signal_usable_frames'],blank_saved_frames=camera['camera_recording']['image_signal_blank_frames'],
            motor_port_opened=False,motor_goal_writes=0,torque_changes=0),
        new_empty_camera_only_marker_screen=True,absence_of_decoded_cube_marker_is_not_proof_of_empty_hand=True,
        actual_ninety_degree_rotation_verified=False,next_user_command='leap_quarter_finish_empty.cmd',next_action='Same command, EMPTY for the entire run. FIRST SPACE is retained while initial camera frames arrive; then the unchanged fresh OFF checks and motion run. Q/Esc/Ctrl+C stops.')
    write(folder/'review.json',review)
    command='@echo off\ncd /d "%~dp0"\necho EMPTY HAND FOR ENTIRE RUN. Cube outside hand and camera. No PLACE prompt.\necho One SPACE in terminal. Initial camera wait is automatic; same reviewed motor path.\n".runtime\\python\\python.exe" -X utf8 -B "leap_quarter_finish_empty_preview_fixed_20261025.py" --execute\npause\n'
    (ROOT/'leap_quarter_finish_empty.cmd').write_text(command,encoding='ascii')
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        p=ROOT/name;r=read(p);r.update(timestamp=stamp,finish_empty_preview_startup_fix=review,next_user_command=review['next_user_command'],next_user_command_ready=True,next_user_command_empty_only=True,next_user_command_has_loaded_motion=False,next_action=review['next_action']);write(p,r)
    text='''## CURRENT 2026-10-25: 빈손 실행 4회는 첫 화면 대기 오류 / 카메라 초기 대기 수정 및 실기 확인

- 23:26부터 23:28까지 `quarter_finish_empty` 실행 4회가 모두 `preview`에서 `Wait for a fresh preview; no motor writes`로 종료됐다. 카메라 시작 알림과 실제 첫 프레임 수신은 별개인데, 화면이 오기 전에 들어온 SPACE를 처리하면서 재시도 가능한 대기 상황을 일반 오류로 종료한 문제다. 모터 포트는 열리지 않았고 상태 읽기 및 목표 전송은 0회다. 이때 OFF 확인과 설정 복원 항목의 기본값 false는 종료나 복원이 실패한 뜻이 아니다.
- 새 카메라 전용 실행 래퍼에서 첫 SPACE를 유지하고 최초 프레임과 빈손 화면 검사가 준비될 때까지 최대 5초 기다린다. 이 대기는 하드웨어 승인 및 포트 열기 전 preview에만 적용한다. 카메라 장치 오류와 사용자 중단은 계속 즉시 처리하며 모터 구성 및 ON 중의 검은 화면과 오래된 화면 중단은 유지한다. 공용 카메라와 기존 모터 실행 코드 및 목표는 변경하지 않았다. 노출값도 바꾸지 않는다.
- 첫 프레임 지연, 화면 안정과 빈손 검사 지연, 제한 시간과 사용자 중단, 장치 오류 전파, 승인 이후 대기 금지와 포트 열기 경계를 포함한 새 검사 11개를 통과했다. 실제 Streaming Webcam에서도 첫 프레임이 없는 직후 곧바로 시작 확인을 호출해 약 0.53초 자동 대기 뒤 준비 완료를 확인했다. 저장 8장 모두 유효하며 검정 화면은 없었다. 이 실제 검사는 카메라 전용이며 모터 포트는 열지 않았다.
- 모터 경로가 바뀌지 않아 기존 정적 201개 및 모터 응답 9개 조건의 증거를 그대로 두고 독립 검증했다. 물리 검사 증거를 새 코드 해시로 덮어쓰지 않았다. 새 래퍼와 카메라 모듈은 별도 해시 및 검사 검토와 기존 경로 증거가 모두 일치해야 장치를 연다.
- 다음 명령은 같은 **`leap_quarter_finish_empty.cmd`**다. **큐브는 처음부터 끝까지 손과 카메라 밖에 둔다. 터미널 SPACE 한 번**으로 시작한다. 첫 화면이 로딩 중이면 같은 SPACE 입력으로 자동 대기한다. 이후 신선한 OFF 검사와 기존 경로 및 새 마무리를 수행하고 자동 OFF한다. 손가락과 받침을 다시 맞추거나 이전 실패 명령을 여러 번 반복할 필요는 없다. 끝나면 했어만 알리면 저장 파일을 검토한다.
- 세부 근거는 `right_hand_rotation_20261024/support_photo_review_221703/finish_preview_fix_2328/review.json`이다. 실제 큐브 90도 회전은 여전히 미검증이며 이번 명령은 빈손 확인이다.

<!-- END EMPTY PREVIEW STARTUP FIX 2328 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig');marker='<!-- END EMPTY PREVIEW STARTUP FIX 2328 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT 2026-10-25: 새 마무리 후보','## PREVIOUS 2026-10-25: 새 마무리 후보',1);p.write_text(text+old,encoding='utf-8')
    from leap_quarter_finish_empty_preview_fixed_20261025 import require_camera_mode_review
    require_camera_mode_review()
    print(json.dumps(dict(preview_fix_ready=True,failed_attempts=4,focused_tests=11,live_camera_probe_passed=True,motor_path_changed=False,next_user_command=review['next_user_command']),ensure_ascii=False))
if __name__=='__main__':main()
