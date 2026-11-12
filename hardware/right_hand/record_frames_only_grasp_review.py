"""Record actual camera fix verification and the missing motor connection."""
import hashlib
import json
from book_grasp_trial_plan import ROOT
from leap_book_cube_grasp_trial import require_offline_review

proof=require_offline_review()
fixture=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v5'
tests=json.loads((fixture/'device_free_validation.json').read_bytes())
camera_folder=ROOT/'book_camera_confirmation_20261024_210907_685466'
camera=json.loads((camera_folder/'result.json').read_bytes())
read_folder=ROOT/'book_settled_off_readonly_20261024_210924_925860'
read=json.loads((read_folder/'result.json').read_bytes())
if not tests['successful'] or camera['camera_recording']['status']!='complete':
    raise RuntimeError('Camera implementation checks not complete')
record=dict(camera_control='preserve existing exposure; no exposure GET/SET session',
    camera_recording=camera['camera_recording'],camera_only_seconds=camera['duration_seconds'],
    camera_only_motor_port_opened=False,
    actual_motor_trial='book_cube_grasp_trial_20261024_181129_687478',
    actual_first_stop='camera exposure readback -1 four times; not a motor collision stop',
    actual_first_off_verified=True,actual_first_settings_restored=True,
    subsequent_failed_empty_route='book_cube_grasp_trial_20261024_181338_435214/current_start_screen.json',
    subsequent_route_still_not_approved=True,
    subsequent_route_reason='Conservative -5mm book-front case: initial support overlap up to4.739mm, model tracking0.151858rad. Other book-front cases pass. No support depth or tracking stop relaxed.',
    dev_replay=dict(source='grasp_route_fixture_v5/request.json',static_cases=len(proof['static_screens']),
        dynamic_cases=len(proof['dynamic_sensitivity']),
        proof_sha256=hashlib.sha256((fixture/'screen_frames_only.json').read_bytes()).hexdigest(),
        substitutes_for_current_pose=False),
    device_free_validation=tests,
    latest_motor_read=dict(folder=str(read_folder.relative_to(ROOT)),status=read['status'],error=read['error'],
        write_commands_sent=0),
    serial_inventory='Only Bluetooth COM3/COM4, no USB serial adapter',
    current_physical_motion_approved=False,
    assistant_motor_writes_sent=0,
    next_user_action='Reconnect USB motor adapter and5V motor power, leave hand/base/book unchanged. No command run required. Then acquire fresh READ-only angles.',
    physical_rotation_verified=False)
(fixture/'frames_only_readiness_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
entry='''## CURRENT 2026-10-24 21:09: 노출 조회로 멈춘 원인 확인 / 영상 기반 카메라 수정 / USB 재연결 대기

- 사용자3회 실행을 원본 로그와 결과에서 직접 확인했다. `book_cube_grasp_trial_20261024_181129_687478`은 약지 분리 및 엄지 분리와 준비824개 목표를 완료하고 PLACE를 기다리다가 카메라 노출 조회4회 모두-1로 중단됐다. 모터 충돌 중단이 아니며 약지에 손이 닿은 시점과 겹쳤을 가능성만 있다. 실제 최저4.1V, 최대111mA, 최대 추종0.095107rad, 토크 OFF와 설정 복원 확인이다. 큐브 배치 확인과 파지 접근은 시작하지 않았다.
- `181338_435214`는 현재 OFF 자세의 모델 검사에서 중단했고 모터 쓰기0회다. 엄지 외형이나 과거 자세와의 단순 비교로 막힌 것이 아니다. 책 앞끝-15/-10mm 가정은 통과하지만-5mm 가정에서 기존 구조물 초기 겹침이 최대4.739mm이고 모델 추종0.151858rad로 기준을 벗어난다. 분리 후 모델 추종은 정상이고 실패의 위치별 민감도를 추가 기록했다. 받침 침투와 추종 제한을 올려 통과시키지 않았다. `181424_938986`은 다시 노출 조회 오류로 미리보기에서 중단돼 모터 포트를 열지 않았다.
- 이 관찰용 책 파지 실행의 Camera만 `PRESERVE_EXPOSURE=True`로 바꿨다. 원래 카메라 노출을 바꾸거나 노출 속성을 GET하지 않는 기본 영상 워커를 사용한다. 실제 프레임의 유효성과 최신성, 검은 화면과 단절 및 Q 중단, 시작 전 큐브 마커 증거는 그대로 확인한다. 다른 고정 노출 실행의 기본 동작은 유지했다. 복원이 필요 없었다는 상태와 복원을 확인했다는 상태를 구분해 기록한다.
- 실제 카메라만65초 촬영한 `book_camera_confirmation_20261024_210907_685466`에서502프레임 모두 유효하며 노출 변경0회와 카메라 종료를 확인했다. 마지막 웹캠에는 큐브가 손 밖에 있다. 카메라 변경을 반영한 과거 실제 빈손 입력의 개발용201개 정적 및9개 동역학 재생과 관련 장치 없는 검사86개를 통과했다. 이 과거 입력이 현재 자세를 대신하지 않는다.
- 새 현재 각도를 READ로 확인하려 했지만 `book_settled_off_readonly_20261024_210924_925860`에서 COM7이 없었다. 포트 목록은 Bluetooth COM3/COM4뿐이며 USB 모터 어댑터가 없다. 모터 쓰기와 토크 변경0회다. **지금 필요한 사용자 작업은 USB 모터 어댑터와5V 모터 전원 연결뿐이다.** 손가락과 본체 및 책은 그대로 두고 프로그램은 실행하지 않는다. 연결 응답 뒤 새 안정 OFF 읽기와 복귀 경로를 확인하며 현재 모터 실행 명령은 요청하지 않는다.
- 사용자가 기존2~5일 예상을 지적했다. 그 예상은 실제 회전이 확인되기 전의 과도한 낙관이었다고 인정했다. 검증되지 않은 완료 날짜나 진행률을 새로 확답하지 않는다. 물리 목표 회전은 여전히 미확인이고 전시 문구는 변경하지 않았다.
- 근거: `right_hand_rotation_20261024/book20_current_review/grasp_1811_review.json`, `support_1813_diagnostic/result.json`, `grasp_route_fixture_v5/screen_frames_only.json`, `device_free_validation.json`, `frames_only_readiness_review.json`. 과거 증명과 실제 결과는 그대로 보존했다.

'''
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    path=ROOT/name;prior=path.read_text(encoding='utf-8')
    if not prior.startswith(entry.splitlines()[0]):
        if prior.startswith('## CURRENT'):prior=prior.replace('## CURRENT','## PREVIOUS',1)
        path.write_text(entry+prior,encoding='utf-8')
print(json.dumps(dict(camera_frames=camera['camera_recording']['frame_count'],tests=tests['tests_run'],
    current_physical_motion_approved=False,next_user_action=record['next_user_action']),ensure_ascii=False))
