"""Record the completed user capture; preserve prior evidence and poster."""
import hashlib,json
from datetime import datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def read(p):return json.loads((ROOT/p).read_bytes())
def write(p,r):(ROOT/p).write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    reviewdir='right_hand_rotation_20261024/support_photo_review_221703'
    capture=read(reviewdir+'/initial_review.json')
    source=ROOT/capture['source']/'result.json'
    if hashlib.sha256(source.read_bytes()).hexdigest()!=capture['source_sha256']:raise ValueError('Capture source changed')
    if not(capture['status']=='quarter_support_photo_complete' and capture['loaded_goal_count']==0 and capture['all_torque_off_verified'] and capture['settings_restored'] and capture['cube_removal_confirmed']):raise ValueError('Incomplete photo sequence')
    seating=read(reviewdir+'/level_book_seating/seating_grid.json');valid=[r for r in seating['cases'] if r['valid_two_link_upright_start']]
    finish=read(reviewdir+'/level_book_seating/finish_root_sweep.json')
    stamp=datetime.now().astimezone().isoformat()
    report=dict(timestamp=stamp,capture=capture,
        qualitative_support_observation='Cube lower edge rests near flat index/middle motor-link surfaces; fingertip pads are not its principal visible support.',
        cable_contact_under_cube_unresolved=True,camera_hand_registration_verified=False,
        edge_alignment_rejected_as_pose_calibration=True,edge_alignment_rejection='Low edge scores put the cube too low/sideways and do not align thumb and ring simultaneously.',
        book_level_gravity_is_setup_hypothesis=True,native_book_normal_tilt_from_default_gravity_deg=19.8524910133,
        model_gravity_changed_in_hardware_execution_programs=False,
        upright_initial_seating_grid_cases=len(seating['cases']),valid_two_link_upright_seatings=len(valid),
        unchanged_recorded_path_orientation_retention_passes=sum(r.get('orientation_retention_pass',False) for r in valid),
        stable_initial_seating_subset_summary=[{k:r[k] for k in ('book_frame_position_mm','red_up_error_deg','max_cube_overlap_mm','max_no_hand_contact_s')} for r in valid if r['book_frame_position_mm'][0] in (36,44)],
        root_finish_exploration_cases=len(finish['cases']),root_finish_accepted_cases=sum(r.get('candidate_accepted',False) for r in finish['cases']),
        root_finish_rejection='Angle alone can look successful while 3.7mm or more cube/body overlap remains; larger extensions can drop the cube.',
        new_physical_cube_rotation_verified=False,hardware_execution_allowed=False,assistant_motor_goal_writes=0,assistant_torque_changes=0,
        next_action='Continue offline contact/stiffness/time-step checks; clarify whether a cable was under the cube. Do not repeat the same photo or loaded path.')
    write(reviewdir+'/review.json',report)
    for p in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        r=read(p);r.update(timestamp=stamp,quarter_support_photo_completed=True,additional_quarter_support_photo_needed=False,
            support_photo_completed_review=report,next_user_command=None,next_user_command_ready=False,next_user_command_photo_only=False,
            next_user_command_has_loaded_motion=False,rotation_in_next_command=False,supervised_experimental_loaded_trial_ready=False,
            robust_loaded_execution_ready=False,physical_ninety_degree_rotation_verified=False,unchanged_loaded_trial_repeat_requested=False,
            previous_nominal_gravity_and_cube_pose_unverified=True,next_action=report['next_action'])
        if 'new_device_free_photo_tests_passed' in r:r['new_device_free_photo_tests_passed']=19
        write(p,r)
    section='''## CURRENT 2026-10-25: 측면 받침 촬영 완료 / 수평 받침 조건으로 회전 미달 재현

- `quarter_support_photo_20261025_221703_805528`는 정상 완료됐다. 카메라 960장이 모두 유효하며 큐브 제거 확인, 토크 OFF와 설정 복원을 확인했다. 최대 추종 0.0874rad, 최대 전류 113mA, 최저 전압 3.9V다. 빈손 준비 쓰기는 있었지만 적재 회전 목표는 0개다. 같은 사진을 다시 요청하지 않는다.
- 새 측면 사진에서 큐브 바닥은 검지와 중지 쪽 평평한 모터 및 링크 부근에 놓이며 손끝 패드가 주된 받침은 아니다. 케이블이 밑모서리 가까이 지나가지만 사진만으로 밑에 깔렸는지나 실패 원인인지는 확정하지 않는다. 이 점만 이전 촬영에 대한 기억으로 질문했으며 재실행은 필요 없다. 영상 모서리 맞춤은 엄지와 약지를 동시에 잘 맞추지 못해 좌표 보정으로 채택하지 않았다.
- 기존 모델의 책과 손바닥 밑면은 기본 중력의 수직 방향에서 약 19.85도 기울어져 있다. 실물 책상이 수평이라는 가정 아래 받침 기준으로 중력을 맞춘 별도 오프라인 계산을 추가했다. 실행 프로그램의 중력이나 장치 설정을 변경한 것은 아니며 실물 설치 각도를 새로 계측한 결과도 아니다.
- 이 조건에서 15개 초기 배치를 만들고 9개가 검지와 중지 쪽에 안정적으로 놓이는 조건을 만족했다. 기록된 실제 적재 관절 경로를 재생했지만 9개 모두 목표 방향 및 유지 기준을 만족하지 못했다. 중앙 쪽 6개는 큐브와 손의 최대 모델 겹침이 약 0.13~0.32mm였으며 약 30~35도 기울어진 상태에 머물렀다. 이 각도는 시뮬레이션 결과이며 실물 계측값이 아니다.
- 추가 뿌리 회전 후보에서는 각도만 보면 90도에 가까워지는 안도 있었지만, 큐브와 손바닥 모델이 3.7mm 이상 겹치거나 큐브가 떨어지는 결과가 있어 실행안에 넣지 않았다. 시간 간격 및 접촉 강도를 명시적으로 바꾼 추가 비교를 진행한다. 현재는 새 실물 실행 명령이 없으며 이전 전체 회전도 그대로 반복하지 않는다. 실제 90도 성공은 아직 미검증이다.
- 근거: `right_hand_rotation_20261024/support_photo_review_221703/review.json`과 `level_book_seating/seating_grid.json`. 현재 검토에서 실제 장치는 열지 않았으며 원본 동작, 실제 로그와 전시 문구는 보존했다.

<!-- END SUPPORT PHOTO COMPLETED 221703 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;text=p.read_text(encoding='utf-8-sig')
        marker='<!-- END SUPPORT PHOTO COMPLETED 221703 -->'
        if marker in text:text=text.split(marker,1)[1].lstrip('\n')
        text=text.replace('## CURRENT 2026-10-25: 측면 촬영 중 카메라 가림 확인','## PREVIOUS 2026-10-25: 측면 촬영 중 카메라 가림 확인',1)
        p.write_text(section+text,encoding='utf-8')
    print(json.dumps(dict(capture_completed=True,frames=capture['camera']['frame_count'],valid_seatings=len(valid),same_photo_repeat_requested=False,next_user_command=None),ensure_ascii=False))
if __name__=='__main__':main()
