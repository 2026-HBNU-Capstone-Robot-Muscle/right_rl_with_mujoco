"""Persist current offline validation before requesting one read-only check."""
import datetime as dt
import hashlib
import json
from pathlib import Path
from grasp_place_plan import GraspPlacePlan
from supported_cube_rotation_plan import SupportedCubeRotationPlan

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261023'


def main():
    tests=json.loads((OUT/'regression_tests.json').read_bytes())
    native=json.loads((ROOT/'right_hand_rotation_20261022/integration_validation.json').read_bytes())
    assert tests['passed'] and tests['tests_run']==173
    assert native['passed'] and len(native['fixtures'])==21
    assert all(x['accepted'] and x['static_cases']==67 and x['dynamic_cases']==3 and
               not x['unchanged_validated_proof_reused'] for x in native['fixtures'])
    paths=[]
    for kind,relative in (
        (GraspPlacePlan,'cube_grasp_trial_20261023_041546_404551'),
        (SupportedCubeRotationPlan,'cube_supported_rotation_trial_20261023_034640_750784')):
        folder=ROOT/relative;plan=kind(folder/'fresh_off_request.json')
        historical=json.loads((folder/'current_start_screen.json').read_bytes())
        digest=plan.plan_digest(plan.source_position())
        assert digest==historical['plan_sha256']
        paths.append(dict(source=relative,plan_sha256=digest,exact_goals_unchanged=True))
    files=('camera_stream.py','empty_trial_camera.py','leap_camera_readonly_check.py',
           'leap_camera_readonly_check.cmd','test_camera_preview_isolation.py','test_camera_readonly_check.py')
    result=dict(timestamp=dt.datetime.now().astimezone().isoformat(),
        status='preview_isolated_readonly_physical_check_ready',
        initiating_trial='cube_grasp_trial_20261023_041546_404551',
        observed_controller_gap_ms=718,observed_fault='all sixteen watchdogs 255 during remove_cube',
        exact_original_blocking_call_verified=False,
        change='imshow/waitKey/window management moved from control thread to the disposable camera worker process.',
        control_preview_updates='nonblocking bounded metadata queue; no frame drawing, GUI waits or frame copies in camera.check',
        original_frame_pixels_preserved=True,watchdog_ms_unchanged=500,
        camera_stale_seconds_unchanged=2.,blank_stop_seconds_unchanged=.5,
        servo_goal_or_limit_changes=False,paths=paths,
        regression_tests_passed=tests['tests_run'],fresh_native_start_proofs=len(native['fixtures']),
        fresh_static_cases=21*67,fresh_dynamic_cases=21*3,
        hardware_commands_sent_by_agent=0,real_camera_or_serial_opened_by_agent=False,
        real_camera_integration_verified=False,physical_cube_rotation_verified=False,
        new_readonly_program='leap_camera_readonly_check.cmd',new_program_servo_writes=0,
        next_user_action='Cube out; USB + motor power connected, torque already OFF. Run new cmd, terminal SPACE once; briefly drag camera window 2-3 times during the automatic 12s read-only check.',
        file_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files},
        limitation='Removing a GUI blocking path does not eliminate USB, OS scheduling, storage or other controller stalls. Existing stop checks stay active.')
    (OUT/'camera_preview_isolation_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    text='''## CURRENT 2026-10-23: 카메라 표시 분리 완료 / 다음은 12초 읽기 전용 확인

- 마지막 접촉 촬영의718ms 제어 공백을 계기로 촬영창 처리를 분리했다. 정확히 어느 GUI 호출이 마지막 실행에서 멈췄는지는 계측되지 않았으므로 원인 확정으로 표현하지 않는다. 다만 제어 스레드의 `imshow/waitKey` 경로를 제거했다.
- 현재 짧은 시험의 `EmptyTrialCamera`는 카메라 전용 프로세스가 창 생성/표시/키를 담당한다. 제어 쪽 `check()`는 기존 영상신호/신선도와 비차단 안내문 큐·Q 신호만 확인한다. 큐가 차면 안내 갱신을 다음으로 넘기며 제어를 기다리게 하지 않는다. 원본 프레임에 안내를 그려 저장하지 않는다. Q/창닫기는 제어 쪽 중단으로 전달된다.
-850ms GUI 정체 모사 중500ms 워치독 읽기 간격 유지, 가득 찬 큐, Q, 창 닫기, 멈춘 프로세스 정리, 원본 픽셀 보존 및 읽기 전용 기능 검사를 포함해 전체173개 회귀검사 통과. 기존21개 시작을 각각67정적/3응답으로 현재 코드에서 모두 다시 재생했다. 증명 해시만 바꾼 재사용은 없다. 기존 두 실제 실행의 관절 경로SHA도 그대로다.
- 모터 워치독500ms, 카메라2초 신선도/0.5초 빈 화면, 기존 전류·전압·추종·목표간격·OFF 복원 기준은 그대로다. 카메라 창이 오래 멈추면 카메라 자체의 신선도 중단은 여전히 발생할 수 있다. 실제 장치 통합 검증이나 큐브 회전/SAC 성공은 아직 아니다. 조수는 실제 장치를 열지 않았다.
- 새 `leap_camera_readonly_check.cmd`는12초 동안 PING/Sync READ만 한다. 목표/토크/게인/모드/EEPROM/워치독 설정 쓰기를 하지 않는다. 이미OFF인 상태만 받으며ON·하드웨어오류·남은워치독오류·불완전읽기는 설정을 고치지 않고 중단한다. 루프는200ms 확인 예산을 검사한다.
- 다음 사용자 작업: 큐브를 빼고 손가락을 그대로 둔다. 다른 모터 프로그램을 닫고 USB/모터 전원을 연결한다. 새cmd실행→터미널SPACE한번→12초동안카메라창제목줄을잡아짧게2~3번움직인다. 웹캠본체나손가락은움직일필요없다. 자동종료후했어만알리면 `leap_camera_readonly_check_result.json`을 직접읽는다. 이 단계에서 큐브를 올리거나 예전 회전 명령을 반복하지 않는다.
- 근거: `right_hand_rotation_20261023/camera_preview_isolation_review.json`, `regression_tests.json`, `right_hand_rotation_20261022/integration_validation.json`. 시각 SAC 정책/배치 오차 학습은 이 변경에 포함되지 않는다.

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8')
        if not old.startswith('## CURRENT 2026-10-23: 카메라 표시 분리 완료'):p.write_text(text+old,encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('status','regression_tests_passed','fresh_native_start_proofs',
        'servo_goal_or_limit_changes','real_camera_integration_verified','new_readonly_program')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
