"""Verified slow-demo completion, available media and pending grip observation."""
import hashlib
import json
from pathlib import Path
from slow_grasp_demo_plan import SlowGraspDemoPlan
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'slow_grasp_demo_20261023_182105_573825'


def main():
    blob=(FOLDER/'result.json').read_bytes();source=json.loads(blob)
    completion=json.loads((FOLDER/'completion_review.json').read_bytes())
    paired=json.loads((FOLDER/'on_paired_pose_review.json').read_bytes())
    native=json.loads((FOLDER/'on_native_geometry_review.json').read_bytes())
    closure=json.loads((FOLDER/'conditional_contact_closure_review.json').read_bytes())
    videos=json.loads((FOLDER/'video_export_review.json').read_bytes())
    proof=validate_proof(SlowGraspDemoPlan(FOLDER/'fresh_off_request.json'),FOLDER/'current_start_screen.json')
    assert source['status']=='slow_grasp_demo_complete' and source['all_torque_off_verified'] and source['settings_restored']
    assert completion['hold_same_goal_samples']==60 and proof['accepted']
    assert paired['directly_paired_ON_frames']==23 and native['old_OFF_angles_used'] is False
    assert not closure['hardware_execution_allowed'] and closure['selected'] is not None
    record=dict(status='physical_slow_grasp_command_demo_complete_grip_force_unverified',source=FOLDER.name,
        source_sha256=hashlib.sha256(blob).hexdigest(),same_code_native_proof_valid=True,
        all_torque_off_verified=True,settings_restored=True,camera_frames=925,
        fixed_same_goal_samples=60,hold_sample_span_s=completion['hold_sample_span_s'],
        minimum_voltage_v=completion['minimum_voltage_v'],peak_absolute_current_ma=completion['peak_absolute_current_ma'],
        maximum_tracking_rad=completion['maximum_tracking_rad'],direct_ON_pairs=23,
        solved_green_grid_diagnostic_pairs=11,old_OFF_angles_used=False,
        physical_grasp_strength_or_lift_verified=False,physical_cube_rotation_verified=False,
        real_demo_videos=videos['outputs'],
        next_contact_candidate_maximum_delta_ticks=max(abs(t) for t in closure['selected']['delta_ticks']),
        next_contact_candidate_is_conditional_and_not_executable=True,
        marker_to_model_registration_verified=False,next_contact_full_path_screened=False,
        pending_operator_observation='Did cube removal feel lightly gripped by fingers, or did it lift freely from the palm? No rerun needed.',
        assistant_hardware_commands_sent=0)
    out=ROOT/'right_hand_rotation_20261023/slow_demo_success_review.json'
    out.write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-23 18:21: 느린 파지 시연 완주 / ON 자료 확보'
    note='''

- `slow_grasp_demo_20261023_182105_573825`는 `slow_grasp_demo_complete`다. 빈손 약지 분리와 준비, 큐브 배치, 느린 파지 명령 322개, 같은 목표 60회 유지, 제거 확인, 16개 토크 OFF와 설정 복원을 모두 마쳤다. 같은 목표를 유지한 60개 표본의 첫끝 시간 차이는 3.125초였다. 현재 코드와 저장된 시작의 실제 경로 증명이 일치하는 것도 확인했다.
- 카메라 원본 925장은 모두 정상 신호였고 기존 노출 설정 복원도 확인했다. 최저 전압 4.1V, 최대 절대 전류 85mA, 최대 추종 오차 약 0.0752rad였으며 하드웨어 오류는 없었다. 최종 사진에서도 큐브 제거가 보인다. 다만 파지 명령의 완주와 손가락의 유지력을 같은 의미로 취급하지 않는다. 들어 올리기나 큐브 회전은 이번 실행에 포함되지 않았다.
- 실제 ON 유지 중 사진 23장에 같은 실행의 앞뒤 건강한 관절 표본을 짝지었다. 손바닥 ID40과 초록 중앙 ID5를 같은 프레임에서 직접 읽었으며, 관절 전후 차이와 시간 간격을 확인했다. 이 자료는 OFF 후 처진 각도가 아니다. 작은 마커 짧은 변은 약 20px로 기존 24px 촬영 안내보다 작다. 반복 촬영을 요구하는 실행 차단 조건은 바꾸지 않고, 이 자료는 불확실성을 적은 오프라인 진단에만 사용한다.
- 초록 면 8칸 격자 진단이 11장에서 성공했다. 작은 마커의 두 평면 분기와 손바닥의 두 분기를 모두 보존했다. 현재 장착 좌표 후보를 그대로 쓴 기본 격자 분기에서는 검지와 중지 끝 약 4~5mm, 엄지 끝 약 2~4mm의 모델 간격이 나온다. 이는 실제 간격을 측정한 결과가 아니며 장착 좌표는 여전히 미확인이다. 약지는 큐브에서 멀리 있다. 손가락 접촉을 맞추려고 장착 좌표나 관절 영점을 바꾸지 않았다.
- 이전 빈손 실행과 동일한 목표 322개를 비교했다. 말미의 전류 차이는 최대 약 14mA로 작았지만 별도 실행의 전압과 이력 및 마찰 차이가 있으므로 접촉이나 파지 부재를 단정하지 않는다. 사용자에게 이번 REMOVE 때 큐브가 손가락에 살짝 잡혀 걸렸는지, 손바닥에서 쉽게 들렸는지 한 가지 관찰을 요청했다. 같은 파지 시험이나 새 사진 촬영을 다시 요청하지 않는다.
- 기다리는 동안 이번 ON 추종 오차를 별도로 기록해 작은 닫힘 후보를 계산했다. 조건부 기본 장착 분기에서 최대 32틱인 후보가 세 가지 영상 위치 가정의 간격을 줄였다. 상자 기하로 끝점만 비교한 결과이며 전체 경로와 힘 및 유지 안정성은 아직 검증하지 않았다. 실행 파일을 만들지 않았고 기존 제어 목표, 전류와 PID도 바꾸지 않았다. native 거리 함수의 0을 접촉으로 간주하지 않고 독립적인 상자 거리와 겹침을 확인했다.
- 공유 가능한 실제 영상: 이번 실행 폴더의 `slow_grasp_close.mp4`는 파지와 유지 약 20.7초, `slow_grasp_full.mp4`는 준비부터 유지까지 약 66.8초다. 저장된 시간 흐름을 유지했고 원본 이미지를 변경하지 않았다. 큐브를 맞춘 영상이나 회전 성공 영상으로 표기하지 않는다.
- **현재 사용자 작업:** 이미 요청한 제거 느낌만 알려주면 된다. 추가 모터 명령 실행은 아직 필요 없다. 응답에 따라 이미 잡힌 큐브의 작은 회전과 추가 닫힘 중 필요한 다음 경로를 선택하고 검증한다.
- 근거: `right_hand_rotation_20261023/slow_demo_success_review.json`, 이번 폴더의 `completion_review.json`, `on_paired_pose_review.json`, `on_native_geometry_review.json`, `conditional_contact_closure_review.json`, `video_export_review.json`.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(title+note+old,encoding='utf-8')
    print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
