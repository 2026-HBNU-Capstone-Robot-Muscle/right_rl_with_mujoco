"""Summarize the three latest saved start-screen failures; no hardware APIs."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261112/tip_first_start_actual_review'
names=['tip_first_trial_20261112_173345_707995','tip_first_trial_20261112_173745_907847','tip_first_trial_20261112_173936_071900']
rows=[];bindings={}
for name in names:
    folder=ROOT/name
    result=json.loads((folder/'result.json').read_bytes())
    review=json.loads((folder/'empty_recovery_screen/review.json').read_bytes())
    nominal=[c for c in review['static_cases'] if not any(c['offset_ticks'])]
    pair_depths={}
    for case in review['static_cases']:
        for pair,depth in case.get('worst_self_pairs_mm',{}).items():pair_depths[pair]=max(pair_depths.get(pair,0),depth)
    state=result['preflight_state']
    rows.append(dict(run=name,status=result['status'],failed_phase=result['failed_phase'],
                     model_nominal_maximum_self_overlap_mm=max(c['max_self_mm'] for c in nominal),
                     model_perturbed_maximum_self_overlap_mm=max(pair_depths.values()),
                     model_pairs=pair_depths,static_passed=sum(c['passed'] for c in review['static_cases']),static_total=len(review['static_cases']),
                     dynamic_passed=sum(c['passed'] for c in review['dynamic_cases']),dynamic_total=len(review['dynamic_cases']),
                     motor_write_events=len(result.get('write_events',[])),samples_recorded=len(result.get('samples',[])),
                     torque_enabled_at_preflight=any(state['torque']),motor_error_flags=state['error'],
                     voltage_range_V=[min(state['voltage'])/10,max(state['voltage'])/10],
                     all_torque_off_verified=result['all_torque_off_verified'],settings_restored=result['settings_restored']))
    for relative in ['result.json','fresh_off_request.json','current_start_screen.log','empty_recovery_screen/review.json']:
        p=folder/relative;bindings[p.relative_to(ROOT).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
frames=[names[-1]+'/camera/frame_00148.jpg',names[-1]+'/camera/frame_00446.jpg']
for relative in frames:bindings[relative]=hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()
summary=dict(status='saved_start_screen_failures_reviewed',runs=rows,source_sha256=bindings,
             inspected_saved_frames=frames,full_video_frame_by_frame_review=False,
             physical_collision_verified=False,loaded_rotation_started=False,
             new_robot_run_performed=False,hardware_access_performed=False,
             next_step='Check and revise the empty-hand recovery path for thumb-tip/index-base clearance; retain existing collision and drive limits.',
             previous_loaded_trial='lower_grip_trial_20261112_162806_610535',
             previous_loaded_trial_review='right_hand_rotation_20261112/lower_grip_actual_review/')
OUT.mkdir(parents=True,exist_ok=True)
target=OUT/'review.json'
if target.exists():raise RuntimeError('Preserve prior review')
target.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'REVIEW.md').write_text('''# 최근 실행 검토

2026-11-12 17:33, 17:37, 17:39의 끝마디 우선 시험 세 번은 모두 `screen_current_pose`에서 종료됐다.
회전·엄지 밀기 단계에는 진입하지 않았으며, 세 결과의 모터 쓰기 기록은 모두 0개다.

명목 빈손 복귀 모델에서 `if_bs_collision_1|th_tip`(검지 뿌리 쪽 충돌 형상과 엄지 끝) 겹침이 약 0.936mm였다.
배치/관절 교란 조건을 포함한 최대값은 약 1.699mm다. 정적 201조건과 동적 9조건은 세 실행에서 각각 모두 거부됐다.
이는 모델 검사 결과이며 실제 손가락 충돌이나 부품 변형을 측정한 값이 아니다.

마지막 실행의 저장 화면 `frame_00148.jpg`, `frame_00446.jpg`는 큐브 없는 손의 비슷한 자세를 보여 준다.
단일 시점 화면으로 엄지와 검지 사이의 실제 간격을 mm로 판정하지 않는다. 모든 영상 프레임을 검토한 것은 아니다.

세 실행 모두 시작 시 모터 오류 플래그 0, 전압 5.0~5.1V, 토크 OFF였으며 종료 시 OFF 확인도 True다.
`settings_restored=False`만으로 설정 복구 고장이라고 판단하지 않는다. 이 실행들은 설정 변경 및 모터 구동 단계 이전에 종료됐고 쓰기 기록이 없다.
17:18 실행은 카메라 미리보기에서 사용자 중단한 별도 시도다.

다음 수정 대상은 빈손 복귀 중 엄지 끝과 검지 뿌리의 통로다. 기존 충돌 한계를 완화하거나 형상을 삭제해서 통과시키지 않는다.
실물 밀기 효과를 확인한 결과로 세지 않으며, 이전 16:28 약지 추종 중단과 구분한다.
장치 연결, 새 모터 구동, 실행 경로 변경, GitHub 업로드는 수행하지 않았다.
''',encoding='utf-8')
print(json.dumps(dict(review=target.relative_to(ROOT).as_posix(),runs=len(rows),motor_writes=[r['motor_write_events'] for r in rows],nominal_overlap_mm=rows[-1]['model_nominal_maximum_self_overlap_mm'],all_off=all(r['all_torque_off_verified'] for r in rows)),ensure_ascii=False))
