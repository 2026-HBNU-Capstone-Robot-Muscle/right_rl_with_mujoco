"""Publish derived review and next input; leave user results and controls intact."""
import ast
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/original_sequence_review'


def main():
    audit=json.loads((OUT/'source_and_path_audit.json').read_bytes())
    full=json.loads((OUT/'free_cube_replay.json').read_bytes())
    prefix=json.loads((OUT/'ninety_degree_prefix_review.json').read_bytes())
    matched=json.loads((OUT/'same_session_measured_trace_replay.json').read_bytes())
    original=Path(audit['source']['original_source_windows'])
    assert hashlib.sha256(original.read_bytes()).hexdigest()==audit['source']['original_sha256']
    assert audit['native_range_violation_count']==0
    assert len(full['placement_search'])==27 and len(matched['runs'])==30
    assert all(np.isfinite([s['change_angle_deg'] for s in r['stages']]).all()
               for r in full['placement_search'])
    production=[ROOT/n for n in ('hand_motor_mapping.py','book_support_model_20261024.py',
        'leap_book_dynamic_yaw_trial.py','book_grasp_trial_plan.py','sequence_flip_poses.json',
        'exhibition_20261023/족자_붙여넣기.txt') if (ROOT/n).exists()]
    assert all(not r['hardware_execution_allowed'] for r in [full,prefix,matched] if 'hardware_execution_allowed' in r)
    # Source artifacts and their current hashes are recorded without refreshing
    # any hardware execution certificate.
    summary=dict(original_source_sha256=audit['source']['original_sha256'],source_matches_archive=True,
        original_poses=7,poses_with_native_range_violation=0,
        original_source_not_imported_or_executed=True,
        maximum_original_step_ticks=max(t['max_source_step_ticks'] for t in audit['transitions']),
        full_path_near_ninety_in_some_conditions=True,full_path_retention_robust=False,
        prefix_hold_retention_robust=all(r['final_retained'] for r in prefix['sensitivity']),
        native_actuator_response_is_not_identified_450mA_hardware=True,
        same_session_start_cases_acceptable=sum(r['initial_overlap_mm']<=.5 for r in matched['runs']),
        same_session_rotation_comparison='not_evaluable_when_all_starts_overlap',
        historical_user_observation='green remained up; orange front changed to yellow with horizontal motion',
        new_hardware_candidate_ready=False,hardware_commands_sent=0,
        physical_ninety_degree_rotation_verified=False,
        production_hashes_recorded_without_reissuing_certificates={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in production},
        next_user_action=dict(command='leap_middle_ring_side_readonly.cmd',
            cube='removed',motors='existing torque OFF, USB and supply available for reads',
            fingers_base_book='unchanged',camera='side view showing middle and ring roots, housings, tips and palm edge',
            key='one SPACE',motor_goal_writes=0,torque_changes=0))
    for filename in ('review_original_sequence_20261024.py','replay_original_sequence_20261024.py',
                     'replay_original_measured_trace_20261024.py','register_original_sequence_images_20261024.py'):
        tree=ast.parse((ROOT/filename).read_text(encoding='utf-8'))
        imports=[]
        for node in ast.walk(tree):
            if isinstance(node,ast.Import):imports.extend(a.name for a in node.names)
            elif isinstance(node,ast.ImportFrom):imports.append(node.module or '')
        assert not any(n.split('.')[0] in ('serial','dynamixel_sdk','main','leap_hw') for n in imports)
    summary['analysis_sources_have_no_direct_motor_interface_imports']=True
    (OUT/'review_summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')
    # Static side by side with the actual archived images, never change sources.
    actual_folder=ROOT/'sequence_vision_20261011_051517_532925'
    strip=np.full((7*400,1280,3),235,np.uint8)
    files=['placement_at_space.png']+[f'stage_{i}_end.png' for i in range(1,7)]
    for i,name in enumerate(files):
        real=cv.imdecode(np.frombuffer((actual_folder/name).read_bytes(),np.uint8),cv.IMREAD_COLOR)
        real=cv.resize(real,(640,360))
        sim=cv.imdecode(np.frombuffer((OUT/f'pose_{i}.jpg').read_bytes(),np.uint8),cv.IMREAD_COLOR)
        sim=cv.resize(sim,(480,360));canvas=np.full((360,640,3),235,np.uint8);canvas[:,80:560]=sim
        top=i*400
        strip[top+40:top+400,:640]=real;strip[top+40:top+400,640:]=canvas
        cv.putText(strip,f'Archived real image - pose {i}',(10,top+28),cv.FONT_HERSHEY_SIMPLEX,.65,(20,20,20),2)
        cv.putText(strip,'Original goals, corrected model IDs (not same camera)',(650,top+28),cv.FONT_HERSHEY_SIMPLEX,.55,(20,20,20),1)
    cv.imencode('.jpg',strip,[cv.IMWRITE_JPEG_QUALITY,85])[1].tofile(str(OUT/'historical_real_and_model.jpg'))
    text='''# 원본 7개 자세 시뮬레이션 검토

사용자가 이전에 실행했다고 제공한 `sequence_flip.py`를 원본 해시와 대조했다. 파일을 실행하거나 가져오지 않고 AST에서 7개 배열만 추출했다. 보존된 `sequence_flip_poses.json`과 16개 모터의 값이 일치한다.

현재 확인된 모터 대응을 사용하면 7개 끝점 모두 모델의 원래 각도 범위 안에 있다. 원본 속도는 현재 실물 시험의 명령당 2틱보다 큰 변화가 있으므로 이 분석은 원본 실행 승인이 아니다. 원본과 현재 제어 코드 및 전류, PID, 중단 기준과 전시 문구는 바꾸지 않았다.

## 재생한 내용

- 55mm, 100g 자유 큐브와 책 20mm 및 유리와 나무 받침 가정을 사용했다. 초기 배치와 마찰은 측정으로 정합한 값이 아니다.
- 원본 전체 경로를 27개 배치에서 재생했다. 일부 조건에서는 앞부분에서 약 90도에 가까워지나 뒤쪽에서 큐브가 이탈했다.
- 원본의 앞부분에서 멈추고 4초 유지한 비교도 마찰과 손 응답 가정에 따라 유지 여부가 달라졌다. 전체 경로 후보는 아직 실물 실행 준비 상태가 아니다.
- 지정된 손 궤적으로 큐브만 동역학 계산한 경우와 네이티브 손 액추에이터를 함께 계산한 경우를 구분했다. 후자는 실제 450mA 모터 응답을 식별한 모델이 아니다.
- 10월 11일 실제 완주 2회의 관절 기록도 넣었다. 이전 배치 가정에서는 모델의 윗면 변경 예측이 당시 사용자의 초록 윗면 유지 관찰과 달랐다.
- 같은 실행의 사진을 현재 치수로 다시 계산한 배치는 모든 비교 시작점에서 모델 겹침이 생겼다. 따라서 그 재생에서 일치나 불일치 비율을 계산할 유효 시작 사례는 없다. 이는 실물의 침투 깊이를 측정한 결과가 아니다.
- 단일 큐브 마커에서 추정한 큰 각도도 사용자가 관찰한 초록 윗면 유지와 어긋났다. 이 수치를 실제 회전 각도나 90도 성공 근거로 사용하지 않는다.

## 파일

- `original_sequence_free_cube.gif`: 원본 경로의 자유 큐브 재생. 일부 회전 뒤 이탈하는 예시이며 실제 성공 영상이 아니다.
- `original_seven_poses.jpg`: 현재 번호 대응으로 복원한 원본 7개 손 자세.
- `historical_real_and_model.jpg`: 당시 저장 사진과 원본 목표 자세의 정성 비교. 같은 카메라 투영 정합은 아니다.
- `source_and_path_audit.json`: 원본 해시, 모터 대응, 각도 범위, 모델 접촉 진단.
- `free_cube_replay.json`, `ninety_degree_prefix_review.json`: 전체 경로와 정지 유지 비교.
- `historical_execution_evidence.json`: 실제 완주 기록과 당시 사용자 관찰.
- `measured_trace_replay.json`, `same_session_measured_trace_replay.json`: 실제 각도 재생과 배치 정합 한계.

## 이어갈 방향과 필요한 입력

원본의 엄지 밀기와 자세 재조정 구간을 90도 회전 및 재파지 설계의 기준으로 유지한다. 손가락의 작은 움직임만 반복하지 않는다. 원본 전체를 즉시 실물 재실행하거나 모델 충돌 허용치를 넓혀 통과시키지 않는다.

다음 실물 동작을 만들기 전에 현재 중지와 약지 및 손바닥 외형을 확인한다. 큐브를 빼고 손가락, 본체와 책을 그대로 둔 상태에서 카메라만 옆으로 옮긴다. `leap_middle_ring_side_readonly.cmd`를 실행하고 SPACE 한 번으로 사진과 전후 관절 읽기를 저장한다. 목표, 토크와 설정 쓰기는 없다. 이는 현재 형상 비교 입력이며 원본의 역사적 배치나 마찰까지 보정됐다는 뜻은 아니다.
'''
    (OUT/'README.md').write_text(text,encoding='utf-8')
    status='''## CURRENT 2026-10-24: 원본 7개 자세를 90도 계획 기준으로 복원하고 전체 재생 및 실물 기록 비교 완료

- 사용자 요청에 따라 이전 원본 `sequence_flip.py`를 AST로만 읽고 보존 해시 및 raw ID 배열을 대조했다. 현재 모터 대응으로 7개 끝점 모두 네이티브 각도 범위 안이다. 원본, 실행 명령과 전시 문구는 변경하지 않았다.
- `right_hand_rotation_20261024/original_sequence_review/`에 원본 자세 그림, 자유 큐브 전체 GIF, 실제 사진 비교와 수치 결과를 저장했다. 일부 시뮬레이션 조건에서 앞부분에 90도에 가까워지는 움직임은 있으나 전체 후반 이탈 및 정지 유지의 마찰 민감성이 남는다. 실물 성공이나 실행 준비로 표시하지 않는다.
- 실제 10월 11일 완주 2회 기록도 재생했다. 모델의 윗면 변경 예측과 사용자의 초록 윗면 유지 관찰이 어긋났다. 같은 실행 사진에서 현재 치수로 추정한 큐브 배치는 모두 초기 모델 겹침이 있어 유효 시작 사례가 없다. 단일 마커 각도도 사용자 관찰과 어긋나 성공 근거에서 제외했다. 기존 센서와 실제 접촉 및 마찰 정합 문제를 숨기지 않는다.
- 다음 기준은 원본의 밀기 및 재파지 순서를 사용한 전체 90도 계획이다. 3도 시험 반복으로 돌아가지 않는다. 새 실제 모터 쓰기는 0회이며 기존 전류, PID, 중단 기준과 관절 범위를 유지했다.
- 다음 사용자 작업은 기존 `leap_middle_ring_side_readonly.cmd`의 옆면 짝 촬영이다. 큐브 제거, 기존 토크 OFF, 손과 책 및 본체 그대로, 카메라만 옆으로 옮겨 중지와 약지의 뿌리부터 끝 및 손바닥 가장자리를 보인다. SPACE 한 번이며 모터 목표나 토크 쓰기는 없다. 이 입력 전 새 회전 실행을 요청하지 않는다.

'''
    p=ROOT/'sac_current_status.md';old_status=p.read_text(encoding='utf-8')
    if not old_status.startswith(status):p.write_text(status+old_status,encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('production_hashes_recorded_without_reissuing_certificates','next_user_action')},indent=2,ensure_ascii=False))


if __name__=='__main__':main()
