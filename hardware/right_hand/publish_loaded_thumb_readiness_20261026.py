"""Publish completed EMPTY evidence and independently checked supervised loaded mode."""
import datetime as dt,hashlib,json
from pathlib import Path
import numpy as np
from loaded_thumb_rotation_plan_20261026 import ROOT,FIXTURE,ACTUAL,COMPLETION,REPLAY,LoadedThumbRotationPlan,TURN_PHASE,extra_hashes
from leap_loaded_thumb_rotation_20261026 import require_review
from loaded_thumb_rotation_camera_20261026 import metadata as camera_metadata
from loaded_thumb_return_clearance_20261026 import metadata as return_metadata
from loaded_thumb_return_model_resolution_20261026 import metadata as resolution_metadata

def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    print('Checking full saved review chain; no devices opened.',flush=True)
    proof=require_review();completion=read(COMPLETION);replay=read(REPLAY)
    assert completion['accepted'] and completion['source_sha256']==sha(ACTUAL)
    process=read(FIXTURE/'off_process_review.json')
    assert process['accepted'] and process['hardware_commands_sent']==0 and not process['devices_opened']
    assert (len(proof['static_screens']),len(proof['dynamic_sensitivity']),len(proof['camera_gap_hold_sensitivity']))==(201,9,9)
    protected={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt'
    previous=read(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json')['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    protected[poster]=previous['sha256'] if isinstance(previous,dict) else previous
    assert all(sha(path)==expected for path,expected in protected.items())
    p=LoadedThumbRotationPlan(FIXTURE/'request.json');raw=p.source_position();rows=p.plan(raw)
    start=next(i for i,(phase,q) in enumerate(rows) if phase==TURN_PHASE)
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    command='leap_loaded_thumb_rotation.cmd'
    action='Run leap_loaded_thumb_rotation.cmd once. FIRST terminal SPACE EMPTY, cube outside hand/camera. At PLACE use flat index/middle link tops, GREEN up/ORANGE toward WRIST ID41. Hands clear; SECOND SPACE starts about96s full reference plus thumb retreat/root lift. At REMOVE, remove cube BEFORE THIRD SPACE; then OFF. Send done; no pasted log is needed. No automatic retry.'
    ready=dict(timestamp=stamp,ready=True,command=command,plan_version=p.PLAN_VERSION,
        experimental_supervised_loaded_only=True,physical_ninety_degree_rotation_verified=False,
        actual_empty_completion=completion,actual_empty_result_sha256=sha(ACTUAL),
        completed_empty_review_sha256=sha(COMPLETION),measured_cube_replay_sha256=sha(REPLAY),
        conditional_measured_cube_cases=9,conditional_measured_cube_cases_passed=9,
        conditional_cube_replay_assumptions=replay.get('assumptions',{}),
        native_static_cases=201,native_response_cases=9,native_empty_camera_hold_cases=9,
        native_proof_sha256=sha(FIXTURE/'screen.json'),off_process_review=process,
        code_hashes=extra_hashes(),planned_goals=len(rows),empty_preparation_goals=start,
        loaded_goals=len(rows)-start,nominal_goal_motion_seconds=len(rows)*p.SAMPLE_PERIOD,
        nominal_loaded_seconds=(len(rows)-start+20)*p.SAMPLE_PERIOD,
        motion_limit_seconds=p.ON_TIMEOUT,off_screen_limit_seconds=240,
        camera_policy=camera_metadata(),empty_return_clearance=return_metadata(),
        empty_return_model_resolution=resolution_metadata(),
        hardware_current_PID_voltage_tracking_limits_unchanged=True,
        loaded_goal_queue_equal_physically_completed_empty_sequence=True,
        placement_and_removal_confirmation_required=True,loaded_camera_failure_stops=True,
        registered_cube_placement_verified=False,loaded_actuator_response_identified=False,
        tests_passed=22,test_file_sha256=sha(ROOT/'test_loaded_thumb_rotation_20261026.py'),
        parent_empty_production_review_chain_preserved=True,
        protected_files_verified={str(path):value for path,value in protected.items()},
        assistant_motor_goal_writes=0,assistant_torque_changes=0,assistant_motor_setting_changes=0,
        next_action=action)
    write(FIXTURE/'readiness_review.json',ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json',
                 'right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json',
                 'right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        path=ROOT/name;r=read(path);r.update(timestamp=stamp,loaded_thumb_rotation_readiness=ready,
            actual_thumb_root_empty_completed=True,next_user_command=command,next_user_command_ready=True,
            next_user_command_empty_only=False,next_user_command_photo_only=False,next_user_command_has_loaded_motion=True,
            rotation_in_next_command=True,next_action=action,physical_ninety_degree_rotation_verified=False)
        write(path,r)
    block='''## CURRENT 2026-10-26: 빈손 완주 확인 및 큐브를 올린 회전 실험 준비

- 최신 `thumb_clear_root_empty_20261026_125659_859144`는 빈손 동작을 완료했다. 목표 1,720개의 순서와 목표 읽기값, 이후 고정 관측 20개를 대조했다. 최저 3.7V, 최대 전류 116mA, 최대 추종 오차 약 0.101rad다. 영상 2,568장을 확보했으며 토크 OFF와 원래 설정 복원을 확인했다. 직후 OFF 관측 중 검지와 중지가 내려왔으므로 그 끝값은 안정된 출발점으로 재사용하지 않았다. 이후 별도 읽기 전용 관측 `finish_empty_readonly_20261026_130555_407292`의 안정된 값을 기준으로 새 준비 경로를 검사했다.
- 실측 빈손 관절 궤적을 그대로 처방한 큐브 모델에서 놓는 위치 세 가지와 마찰 세 가지, 총 9개 조건이 목표 방향에 도달했다. 큐브 55mm, 질량 100g, 마찰 0.2/0.3/0.5와 손-큐브 초기 위치 및 받침 정합은 모델 가정이다. 부하를 받은 모터 응답이나 실물 회전 성공의 검증이 아니다. 재생 영상과 놓을 위치 그림은 `measured_thumb_125659_cube_replay/preview`에 저장했다.
- 새 `leap_loaded_thumb_rotation.cmd`는 빈손 복귀 후 PLACE에서 큐브를 놓고, 완료했던 기준 궤적과 엄지 후퇴 및 검지/중지 뿌리 올리기를 실행한다. 큐브 전 빈손 복귀에서만 엄지 CMC를 최대 2틱(약 0.18도) 더 들어 손바닥 스침을 피하고, PLACE 자세와 이후 목표는 그대로 유지한다. RETURN의 약지 끝/손바닥4 모델 모서리 관측 약 4.7nm에는 해당 쌍/구간에만 10nm 수치 해상도를 명시했다. 형상과 접촉 힘은 보존했고 실제 전압, 전류, PID와 추종 중단 기준은 바꾸지 않았다. 초기 접촉의 이탈 및 이후 다른 접촉은 계속 검사한다.
- 새 경로의 정적 조건 201개, 정상 응답 9개, 빈손 구간 고정 목표 대기 응답 9개와 저장 검증 연결이 통과했다. 프로그램 검사 22개는 PLACE 전 취소 시 회전 목표 미전송, 모든 회전 목표의 순서 및 1회 전송, 상태 이상 및 제거 취소 시 OFF/설정 복원, 기존 저전압 중단과 읽기 시한, PLACE 안내부터 영상 상실 시 중단, 기본 실행의 장치 미접근 및 Windows/WSL 파일 키 정합과 변조된 검증 결과의 거부를 포함한다. 명목 빈손 준비 약 69초와 큐브 동작 약 96초이며, 새 모드만 ON 제한 210초다. 카메라 복구 대기는 PLACE 안내 전 빈손 준비에만 허용한다. PLACE 안내 이후에는 큐브가 놓였을 수 있으므로 기존 엄격한 영상 중단을 적용한다.
- 다음 사용자 작업: `leap_loaded_thumb_rotation.cmd` 한 번. 첫 터미널 SPACE는 빈손이며 큐브를 카메라 밖에 둔다. PLACE 안내 후 검지와 중지의 평평한 링크 윗부분에 초록 위/주황 손목 ID41 방향으로 놓고 손을 치운 뒤 두 번째 SPACE다. REMOVE 안내가 나오면 큐브를 뺀 뒤 세 번째 SPACE로 OFF한다. 실물 90도 회전은 아직 미검증이다. 로그를 붙일 필요 없이 ‘했어’라고 알려주면 저장 결과를 확인한다.
- 근거: 최신 실행의 `completion_review.json`, `loaded_thumb_root_fixture_v4_portable_proof/screen.json`, `off_process_review.json`, `readiness_review.json`, `test_loaded_thumb_rotation_20261026.py`. 전시 문구와 원본 시퀀스 및 관절 범위는 보존했다. 이번 분석과 개발 중 실제 모터 쓰기/토크/설정 변경은 0회다.

<!-- END LOADED THUMB CURRENT 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END LOADED THUMB CURRENT 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);path.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(ready=True,command=command,goals=len(rows),tests=22,static=201,dynamic=9,camera_hold=9,physical_rotation_verified=False)),flush=True)
if __name__=='__main__':main()
