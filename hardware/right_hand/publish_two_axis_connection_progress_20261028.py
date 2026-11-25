"""Update current progress from existing, independently preserved evidence."""
import datetime as dt
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/two_axis_connection'
MARKER='<!-- END TWO AXIS CONNECTION PROGRESS 20261028 -->'
def read(path):return json.loads(path.read_bytes())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    pitch=read(OUT/'saved_pitch/source_review.json')
    yawpath=ROOT/'right_hand_rotation_20261028/completed_calibrated_174723/physical_review.json'
    yaw=read(yawpath)
    assert pitch['physical_successes']==2
    assert yaw['preparation_grasp_yaw_open_and_removal_complete'] is True
    source=ROOT/yaw['source_run']/'result.json'
    assert sha(source)==yaw['source_result_sha256']
    state=dict(status='two_separate_axes_observed_loaded_connection_under_model_review',
        timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),
        latest_run=yaw['source_run'],latest_run_complete=True,
        pitch_physical_successes=2,pitch_success_scope='qualitative RED up / GREEN toward wrist; exact angle not certified',
        pitch_evidence='right_hand_rotation_20261028/two_axis_connection/saved_pitch/source_review.json',
        identical_pitch_loaded_goals=1268,
        yaw_evidence=yawpath.relative_to(ROOT).as_posix(),
        yaw_held_camera_angle_deg=yaw['held_camera_relative_yaw_estimate_deg'],
        yaw_after_open_camera_angle_deg=yaw['after_open_camera_relative_yaw_estimate_deg'],
        communications_failures_recovered=2,
        exact_robot_frame90_certified=False,loaded_pitch_yaw_connection_verified=False,
        arbitrary_direction_control_verified=False,cube_solving_verified=False,
        repeat_current_command_required=False,next_user_command=None,
        next_user_action='None yet; keep the existing demonstrated motions and review loaded regrasp/connection offline.',
        current_work='Connect the demonstrated pitch ending to the yaw grasp while retaining and seating the cube.',
        naive_loaded_connection_rejected_in_model=True,
        motor_writes_by_assistant=0,real_devices_opened_for_this_review=False)
    (OUT/'current_progress.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    archive=OUT/'prior_pointers';archive.mkdir(exist_ok=True)
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
                 'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=ROOT/name;saved=archive/path.name
        if not saved.exists():saved.write_bytes(path.read_bytes())
        path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block='''2026-10-28 이어서 진행: 기존 성공 자료를 재검증해 수직 뒤집기와 수평 회전을 구분해 정리했다. 수직은 10월26일 `loaded_thumb_rotation_20261026_135418_442474`와 `loaded_repeat_20261026_170932_163649`에서 초록 위/주황 손목 쪽에서 빨강 위/초록 손목 쪽으로 바뀐 실물 관찰 2회가 있다. 두 실행의 PLACE 이후 목표 1,268개와75ms 주기가 원본 전송 기록과 완전히 같음을 다시 확인했다. 정밀90도 각도나 통계적 반복성을 인증한 것은 아니다. 이 경로는 `two_axis_connection/saved_pitch`에 원본과 분리해 보존했다.

수평은 오늘17:47 보완판의 전체 완주와 고정 중 약88도, 자동OPEN 후 약82도의 카메라 상대 추정을 그대로 유지한다. 앞선 기록의 ‘다른 축 미확인’은 이번 수평 실행만의 검증 범위로 읽어야 한다. 과거 수직 뒤집기 관찰 2회까지 없었다는 뜻이 아니다. 같은 수직/수평 동작의 단순 반복은 요청하지 않는다.

현재 과제는 큐브를 내려놓지 않고 뒤집기 끝 자세에서 수평 파지로 옮겨 잡는 연결이다. 손 자체의 정적 충돌 검사는25가지 순서가 통과했지만, 단순 동시 연결에서는 큐브가 다음 파지 기준보다 약24mm 손목 쪽에 남았고 이후 회전 모델에서 이탈했다. 따라서 이를 실물 실행 명령으로 만들지 않고 접촉 물리와 재파지 순서를 계속 검토한다. 초기 큐브 위치는 과거 모델의 가설이므로 새 계산도 실물 좌표 인증이 아니다. 사용자 추가 동작은 아직 필요 없고, 이번 작업에서 실물 모터 명령/토크/설정 쓰기는0회다.

진행 근거: `right_hand_rotation_20261028/two_axis_connection/current_progress.json`, `saved_pitch/source_review.json`, `hand_join/review.json`, `loaded_join/together_with_yaw/review.json`. 기존 성공 코드와 원본 결과, 시퀀스, 관절 범위 및 전시 문구는 보존했다.

'''+MARKER+'\n\n'
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
        if MARKER in old:old=old.split(MARKER,1)[1].lstrip('\n')
        path.write_text(block+old,encoding='utf-8')
    print(json.dumps(state,ensure_ascii=False))

if __name__=='__main__':main()
