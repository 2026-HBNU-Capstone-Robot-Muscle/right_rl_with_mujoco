"""Record the second observed forward flip without overstating reliability."""
import datetime as dt,hashlib,json
from pathlib import Path
import numpy as np
from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan,TURN_PHASE
from loaded_repeat_plan_20261026 import LoadedRepeatPlan
ROOT=Path(__file__).resolve().parent
FIRST=ROOT/'loaded_thumb_rotation_20261026_135418_442474'
SECOND=ROOT/'loaded_repeat_20261026_170932_163649'
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    source=read(SECOND/'result.json');commands=read(SECOND/'command_completion_review.json');out=SECOND/'visual_review'
    index=read(out/'video_source_index.json');video=read(out/'video_export_review.json');first_review=read(FIRST/'visual_review/physical_flip_review.json')
    assert commands['all_goals_exactly_once'] and commands['source_sha256']==sha(SECOND/'result.json')==index['result_sha256']
    assert video['source_index_sha256']==sha(out/'video_source_index.json') and video['encoded_sha256']==sha(out/video['encoded_file'])
    assert source['status']=='loaded_repeat_complete' and source['cube_removal_confirmed'] and source['all_torque_off_verified'] and source['settings_restored']
    assert first_review['result_sha256']==sha(FIRST/'result.json')
    tails=[]
    for folder,plan_type in ((FIRST,LoadedThumbRotationPlan),(SECOND,LoadedRepeatPlan)):
        r=read(folder/'result.json');p=plan_type(folder/'fresh_off_request.json');rows=p.plan(np.asarray(r['accepted_enable_goal']))
        start=next(i for i,(phase,q) in enumerate(rows) if phase==TURN_PHASE);tails.append(rows[start:])
    assert [phase for phase,q in tails[0]]==[phase for phase,q in tails[1]]
    np.testing.assert_array_equal([q for phase,q in tails[0]],[q for phase,q in tails[1]])
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    review=dict(timestamp=stamp,status='physical_forward_flip_reproduced_once',source=SECOND.name,result_sha256=sha(SECOND/'result.json'),
        command_completion_sha256=sha(SECOND/'command_completion_review.json'),
        previous_success=FIRST.name,previous_success_review_sha256=sha(FIRST/'visual_review/physical_flip_review.json'),
        observed_start='GREEN up / ORANGE toward wrist ID41',observed_end='RED up / GREEN toward wrist ID41',
        qualitative_target_reached=True,final_target_pose_maintained=True,exact_3D_angle_measured=False,
        successful_physical_forward_flips=2,loaded_attempts_observed=3,interrupted_loaded_attempts=1,
        one_reproduction_observed=True,consecutive_success_claim=False,robust_repeatability_verified=False,
        arbitrary_direction_rotation_verified=False,horizontal_quarter_turn_verified=False,cube_solving_verified=False,
        loaded_goal_positions_order_and_pacing_equal_between_successes=True,loaded_goal_count=len(tails[1]),
        reviewed_selected_frames=index['selected_frames'],human_cube_contact_observed_in_reviewed_frames=False,
        base_support_by_person_visible=True,continuous_human_contact_exclusion_certified=False,
        camera_image_count=source['camera_recording']['frame_count'],blank_image_count=source['camera_recording']['image_signal_blank_frames'],
        recovered_voltage_confirmations_ms=[e['elapsed_ms'] for e in source.get('voltage_events',[])],
        minimum_voltage_v=commands['minimum_voltage_v'],maximum_current_ma=commands['maximum_current_ma'],maximum_tracking_rad=commands['maximum_tracking_rad'],
        all_torque_off_verified=True,settings_restored=True,raw_result_physical_flip_flag_preserved=source['physical_flip_verified'],
        video=video['encoded_file'],video_sha256=video['encoded_sha256'],recorded_span_seconds=index['recorded_span_seconds'],
        source_frames=index['frame_count'],maximum_recorded_frame_gap_seconds=index['maximum_recorded_frame_gap_seconds'],
        raw_result_not_modified=True,assistant_motor_goal_writes=0,assistant_setting_writes=0)
    write(out/'physical_flip_review.json',review)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json',
                 'right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json',
                 'right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        path=ROOT/name;r=read(path);r.update(timestamp=stamp,physical_forward_flip_reproduction=review,
            latest_loaded_repeat_completed=True,latest_loaded_repeat_error=None,
            physical_forward_flip_successful_trials=2,physical_forward_flip_loaded_attempts=3,
            physical_quarter_turn_demonstrated=True,physical_ninety_degree_rotation_verified=True,
            physical_rotation_verification_basis='Qualitative face transition reproduced; exact angle not calibrated.',
            repeatability_verified=True,repeatability_scope='One reproduction observed; 2 successes and 1 voltage-interrupted loaded attempt.',
            robust_repeatability_verified=False,arbitrary_direction_rotation_verified=False,
            next_user_command=None,next_user_command_ready=False,next_action='Continue offline horizontal-quarter-turn work; no further repetition requested now.')
        write(path,r)
    block='''## CURRENT 2026-10-26: 실물 수직 뒤집기 재현 확인, 성공 총 2회

- `loaded_repeat_20261026_170932_163649`가 새 시간 측정/읽기 간격 방식으로 끝까지 완주했다. 목표 1,913개를 순서대로 한 번씩 보냈고 목표 읽기값은 모두 일치했다. 이전 성공과 출발 빈손 자세는 다르지만 PLACE 이후 목표 1,268개와 75ms 주기는 정확히 같다. 큐브 제거, 토크 OFF 및 설정 복원을 확인했다. 최저 전압 3.6V, 최대 전류 128mA, 최대 추종 오차 약 0.107rad다. 전압 확인 세 번은 각각 약 54.17/80.78/54.31ms 안에 전체 ID 3.8V 이상 두 번을 확보했고 그동안 목표를 전송하지 않았다. 상태 읽기 누락 한 번도 새 읽기로 62ms 안에 회복했다.
- 전후 사진과 추출 장면에서 초록 위/주황 손목 방향에서 빨강 위/초록 손목 방향으로 바뀐 뒤 마지막 고정 구간에도 유지되는 것을 확인했다. 검토한 장면의 사람 손가락은 본체를 지지하며 큐브 직접 조작은 보이지 않는다. 정밀한 3D 각도 측정이나 모든 프레임의 사람 접촉 부재 인증은 아니다. 수직 방향 목표 면 전환은 총 2회 실물에서 확인됐고, 그 사이 전압으로 중단된 큐브 실행 1회가 있으므로 연속 성공이나 통계적 신뢰성으로 표현하지 않는다.
- 두 번째 성공 영상은 `loaded_repeat_20261026_170932_163649/visual_review/actual_forward_flip_realtime.mp4`다. 실제 촬영 구간 약 99.891초의 저장 프레임 774장을 원래 시각에 맞춰 H.264로 묶었고 원본 촬영 내용은 바꾸지 않았다. 전체 카메라 기록 3,128장은 모두 정상이며 구간의 최대 저장 프레임 간격은 약 0.172초다. 첫 성공 영상과 새 성공 영상 및 각 원본 결과는 모두 보존했다.
- 같은 수직 동작 반복 요청은 여기서 마친다. 다음은 수평 방향 90도 경로와 동작 간 연결을 오프라인에서 검토한다. 현재까지 수평 90도 실물 회전, 임의 방향 제어, 연속 재파지 및 큐브 풀이 전체는 확인하지 않았다. 새 경로가 준비되기 전에는 추가 사용자 실행을 요청하지 않는다. 전시 문구와 기존 성공 코드 및 원본 시퀀스, 관절 범위는 그대로다.
- 근거: 두 실행의 `command_completion_review.json`, 새 `visual_review/physical_flip_review.json`, `video_export_review.json`, `actual_rotation_contact_sheet.jpg`. 원본 결과의 자동 `physical_flip_verified=false` 값은 그대로 두고 별도 시각 검토에 성공을 기록했다. 이번 분석은 모터 목표/토크/설정 쓰기 0회다.

<!-- END FLIP REPRODUCTION CURRENT 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END FLIP REPRODUCTION CURRENT 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);path.write_text(block+old,encoding='utf-8')
    assert sha(SECOND/'result.json')==review['result_sha256']
    print(json.dumps(dict(status=review['status'],successes=2,loaded_attempts=3,loaded_goal_count=len(tails[1]),video=str(out/video['encoded_file']))),flush=True)
if __name__=='__main__':main()
