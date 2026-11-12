"""Publish completed saved-camera review and a distinct camera-only next step."""
import ast,datetime as dt,hashlib,json
import xml.etree.ElementTree as ET
from pathlib import Path
from publish_yaw_camera_next_step_20261026 import marker_svg
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026';REG=OUT/'saved_marker_registration';VIS=ROOT/'cube_visibility_20261026_192140_408522'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    protected={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',ROOT/'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',ROOT/'loaded_repeat_20261026_170932_163649/result.json':'1722388e82f512b9f7fa24d60a75ef8320c59435c61d5f6117c6d7fec70a794d'}
    assert all(sha(p)==s for p,s in protected.items())
    capture=json.loads((VIS/'saved_review.json').read_bytes());pose=json.loads((VIS/'spin_invariant_pose_review.json').read_bytes());replay=json.loads((REG/'raw_registered_physical_flip_replay.json').read_bytes());end=json.loads((REG/'saved_after_flip_metric_position.json').read_bytes());stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    assert capture['saved_frames']==24 and capture['motor_commands_sent']==0 and all(c['conditional_forward_flip'] for c in replay['cases'])
    assert all(v['passed'] for v in pose['centre_spin_invariance_checks'])
    script=ROOT/'cube_red_green_capture_20261026.py';tree=ast.parse(script.read_text(encoding='utf-8'));imports=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Import):imports.extend(a.name for a in node.names)
        elif isinstance(node,ast.ImportFrom):imports.append(node.module or '')
    assert not any(s.startswith(('serial','dynamixel_sdk','leap_loaded','book_grasp','loaded_thumb')) for s in imports)
    layout=(OUT/'cube_camera_visibility_layout.svg').read_text(encoding='utf-8')
    layout=layout.replace('카메라 전용 촬영 배치','빨강 / 초록 기준 촬영').replace('초록 면은 위로, 카메라는 위쪽에서','빨강 면은 위로, 초록 면은 카메라 쪽으로').replace('중앙 마커 ID5가 크게','위 ID2와 앞 ID5가 함께').replace('ID5 크기 40px 이상을 목표로, 위쪽에서 가까이 비춰줘. Q는 종료.','손과 받침은 그대로. 큐브만 책상 위에서 돌려 놓아줘. Q는 종료.')
    layout=layout.replace('#f28b3d','#28b87c').replace('fill="#28b87c"/','fill="#df4044"/')
    old=marker_svg(5,86,86,56);assert old in layout;layout=layout.replace(old,marker_svg(2,86,86,56),1)
    layout=layout.replace('</svg>','<g transform="matrix(0.75 -0.18 0 0.575 648 330)">'+marker_svg(5,86,86,56)+'</g></svg>');ET.fromstring(layout);(OUT/'red_green_marker_capture_layout.svg').write_text(layout,encoding='utf-8')
    action=dict(command='leap_cube_red_green_check.cmd',camera_only=True,motor_power='OFF',robot_hand_base_and_book='unchanged',cube='ON TABLE outside hand: RED UP, GREEN toward camera; show ID2, ID5 and wrist ID41 together',marker40_required=False,space_is_not_marker_gated=True,keys='One SPACE in camera or terminal',result_pointer='leap_cube_red_green_result.json',reason='Calibrate the red-face marker using the known 55mm cube and the measured green marker. End-frame top-marker depth still varies about9.4mm; no extra motor replay is requested.',do_not_repeat_without_saved_file_review=True)
    new=dict(timestamp=stamp,status='green_reference_complete_raw_video_registration_improved_red_reference_needed',physical_forward_flip_successes=2,horizontal_quarter_turn_verified=False,horizontal_loaded_execution_ready=False,
        completed_camera_capture=capture,first_green_capture_complete=True,repeat_green_capture_requested=False,two_face_pose_profiles=pose['profiles'],centre_spin_regression_checks=pose['centre_spin_invariance_checks'],single_face_full_orientation_not_certified=True,
        raw_frame_metric_initial_model_overlap_mm=.41804493232902684,representative_snapshot_metric_model_overlap_mm=10.516066183353734,raw_frame_does_not_certify_absolute_hand_calibration=True,
        actual_joint_replay_cases=replay['cases'],model_replay_does_not_create_another_physical_success=True,end_pose_review='right_hand_rotation_20261026/saved_marker_registration/saved_after_flip_metric_position.json',end_pose_center_peak_to_peak_mm=end['center_peak_to_peak_mm'],
        end_pose_exact_code_bits=True,end_pose_border_glare_still_present=True,end_pose_single_face_and_approximate_hand_registration=True,
        next_user_action=action,next_user_command=action['command'],next_user_command_ready=True,next_motion_command_ready=False,camera_program_sha256=sha(script),camera_program_direct_imports=sorted(set(imports)),camera_default_mode_verified_without_devices=True,
        motor_motion_programs_changed=False,assistant_motor_commands_sent=0,real_devices_opened_during_this_review=False,protected_hashes={p.relative_to(ROOT).as_posix():sha(p) for p in protected},hardware_execution_allowed=False)
    write(OUT/'current_camera_and_yaw_review_20261026.json',new)
    oldpath=OUT/'horizontal_next_step_20261026.json';old=json.loads(oldpath.read_bytes());old.update(timestamp=stamp,status=new['status'],operator_camera_capture_not_yet_performed=False,first_green_capture_complete=True,completed_green_capture_source='cube_visibility_20261026_192140_408522',next_user_action=action,latest_review='right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json',next_user_command=action['command'],next_user_command_ready=True);write(oldpath,old)
    block='''## CURRENT 2026-10-26: 초록 기준 촬영 완료, 기존 영상으로 실제 뒤집기 경로 재생

- 사용자 촬영 `cube_visibility_20261026_192140_408522`의 24장을 확인했다. ID5와 ID41은 22장에서 함께 읽혔고 노출 -5가 선명했다. 권장 40px에는 못 미쳤지만 약 30px 자료도 사용했으며 같은 초록 촬영의 재실행은 요청하지 않는다. 카메라 설정의 원래 읽기값 복원을 확인했고 모터 접근 및 쓰기는 0회다.
- 두 면을 쓰는 새 오프라인 자세 계산에서 원래 노출 6장 모두 후보가 나왔고 정지 큐브의 축별 위치 범위는 최대 약 1.2mm였다. 이는 정지 반복 정밀도이며 절대 손 좌표 정확도는 아니다. 각 면의 중앙 마커가 90도씩 따로 회전하는 합성 대응점 검사 6개를 통과했다. 실제 레이어 회전 촬영은 아니며 한 면만으로 전체 방향을 확정하지 않는다. 실제 제어 프로그램과 라이브 검출기는 변경하지 않았다.
- 성공 실행의 대표 사진만 썼을 때는 손 좌표 계산에서 큐브와 모델이 약 10.5mm 겹쳤다. 같은 시각의 영상 원본 `frame_02304.jpg`를 찾아 손목 마커의 맞춤 오차가 2.75px에서 0.35px로 줄었고 모델 겹침도 약 0.42mm로 줄었다. 원본 프레임을 쓰는 것이 중요하지만 이 결과 자체가 손의 3D 등록을 인증하지는 않는다.
- 원본 사진의 시작 위치와 두 번째 성공 실행의 실측 관절 1,275개를 95.11초 동안 처방해 자유 큐브를 재생했다. 손바닥 법선과 새 책상 참고 방향의 두 중력 가정에서 모두 빨강 위/초록 손목 방향으로 끝났다. 최종 빨강 위 오차는 약 3.24도와 5.37도, 최대 큐브 겹침은 약 0.43mm와 0.42mm, 손 자체 및 받침 겹침은 0이었다. 마찰 0.3과 질량 100g, 초기 단일 면 자세 및 손 등록이 가정된 진단이다. 실물 성공은 기존 총 2회로 유지한다.
- 기존 종료 영상 16장을 재검사해 빨강 ID2와 손목 ID41을 13장에서 찾았다. 중앙 코드 16비트가 일치하고 테두리 20칸 중 반사 영향이 3칸 이하인 6장을 따로 계산했지만, 단일 면 깊이 추정의 축별 범위가 최대 약 9.4mm이고 손 좌표 등록도 근사다. 이 값을 바로 수평 회전 목표로 쓰지 않는다. 이미 모터 기록과 영상이 있으므로 수직 동작 반복 대신, 빨강 ID2와 초록 ID5가 함께 보이는 책상 촬영 한 번으로 빨강 마커의 크기와 반사 영향을 확인한다.
- 재파지는 인접한 검지/중지와 반대면 엄지/검지 또는 엄지/중지 방식, 접촉 위치 갱신 및 접촉점 높이 조절을 모델에서 비교했다. 일부 손 자체 간섭 없는 경로와 순간 두 손끝 접촉은 찾았지만 큐브 유지와 수평 90도까지 통과한 실행 후보는 없다. 모델의 깊은 접촉이나 기울어진 작은 회전을 성공으로 채택하지 않았다. 전압/전류/추종 중단, 원본 시퀀스와 관절 범위, 성공 프로그램 및 전시 문구는 그대로다.
- 다음 사용자 작업은 `leap_cube_red_green_check.cmd` 한 번이다. 모터 5V 전원을 계속 끄고 손, 본체, 책은 그대로 둔다. 큐브만 손 밖 책상에서 빨강 위/초록 카메라 방향으로 돌려 놓고 ID2, ID5, 손목 ID41이 같이 보이게 한다. 카메라 또는 터미널 SPACE 한 번이면 약 10초 동안 18장을 저장한다. ID40 요구나 마커 부족 촬영 거부는 없다. `red_green_marker_capture_layout.svg`에 그림을 저장했다. 기본 검사 모드는 장치를 열지 않고 통과했으며 실제 새 촬영은 사용자 작업으로 남아 있다.
- 상세 근거는 `current_camera_and_yaw_review_20261026.json`, `saved_marker_registration/raw_registered_physical_flip_replay.json`, `saved_after_flip_metric_position.json` 및 촬영 폴더의 `spin_invariant_pose_review.json`이다. 이번 저장 자료 분석에서 실제 장치를 열거나 모터를 쓴 적은 없다.

<!-- END CAMERA RAW FRAME REVIEW 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END CAMERA RAW FRAME REVIEW 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);path.write_text(block+old,encoding='utf-8')
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        path=ROOT/name;r=json.loads(path.read_bytes());r.update(timestamp=stamp,next_user_command=action['command'],next_user_command_ready=True,next_user_command_photo_only=True,next_user_command_camera_only=True,next_user_command_empty_only=False,rotation_in_next_command=False,next_motion_command_ready=False,horizontal_loaded_execution_ready=False,next_user_action=action,next_action='One camera-only RED UP / GREEN forward table reference; first green capture already complete.',latest_camera_and_yaw_review='right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json');write(path,r)
    assert all(sha(p)==s for p,s in protected.items());print(json.dumps(dict(next_command=action['command'],camera_only=True,green_capture_complete=True,protected_files_unchanged=True,horizontal_motion_ready=False)),flush=True)
if __name__=='__main__':main()
