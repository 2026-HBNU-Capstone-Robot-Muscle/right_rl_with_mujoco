"""Review real native replay/test outputs and record the revised next action."""
import hashlib
import json
from book_grasp_trial_plan import ROOT,BookGraspPlan,RING_RELEASE,THUMB_RELEASE
from leap_book_cube_grasp_trial import require_offline_review

folder=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v5'
proof=require_offline_review();plan=BookGraspPlan(folder/'request.json')
validation=json.loads((folder/'device_free_validation.json').read_bytes())
if not validation['successful']:raise RuntimeError('Device-free checks failed')
read_folder=ROOT/'book_settled_off_readonly_20261024_180453_383542'
read=json.loads((read_folder/'result.json').read_bytes())
if read['status']!='stable_current_off_read' or read.get('motion_admission_error'):
    raise RuntimeError('Actual EMPTY source is not admitted')
rows=plan.plan(plan.source_position())
r=dict(implementation_reviewed=True,plan_version=plan.PLAN_VERSION,
    source_actual_read=str(read_folder.relative_to(ROOT)),actual_ticks=plan.source_position().tolist(),
    fresh_runtime_off_read_and_screen_required=True,
    historical_fixture_is_not_runtime_hardware_approval=True,
    native_proof_sha256=hashlib.sha256((folder/'screen_final.json').read_bytes()).hexdigest(),
    static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),
    model_policy=proof['observed_model_policy'],
    ring_release_goals=sum(p==RING_RELEASE for p,_ in rows),
    thumb_release_goals=sum(p==THUMB_RELEASE for p,_ in rows),
    command_count=len(rows),max_model_tracking_rad=max(s['max_tracking_rad'] for s in proof['dynamic_sensitivity']),
    max_raw_ring_model_overlap_mm=proof['max_ring_tip_palm_raw_overlap_mm'],
    physical_contact_depth_measured=False,
    measured_release_required_before_thumb_and_root_goals=True,
    original_actual_tracking_stop_rad=.18,original_actual_goal_current_ma=450,
    current_PID_voltage_temperature_no_progress_stops_unchanged=True,
    device_free_validation=validation,assistant_motor_write_commands_sent=0,
    physical_rotation_verified=False,
    next_user_action='Leave fingers/base/book unchanged, cube outside hand. Run leap_book_cube_grasp_trial.cmd. '
        'First SPACE starts EMPTY recovery. PLACE/second SPACE starts the observed grasp prefix. '
        'REMOVE/third SPACE confirms removal and OFF. No rotation.')
(folder/'readiness_review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
entry='''## CURRENT 2026-10-24 18:04: 실측 OFF 엄지 범위 반영 / 약지 자동 분리 후 복귀

- 사용자는 실물이 가능한 자세를 모델 제약으로 막는다면 모델을 고쳐야 한다고 지적했다. 원래 ID4 하한은 가져온 모델의 -0.349rad이며 이 손의 실제 스토퍼를 측정한 값이 아니었다. 이전의 엄지와 약지 수동 정리 요청은 철회했다. 책과 본체 및 손가락을 손으로 다시 맞추지 않는다.
- `observed_book_model_policy.py`에서 실제 안정 OFF ID4=1782틱과 기존 +/-4틱 검사 범위를 반영해 하한1778틱으로 수정했다. 원본 모델과 원본 제한 JSON은 보존하며 책 파지 모델의 ID4 하한만 메모리에서 바꾼다. 생성 경로는 이 관절을 실제 시작 각도보다 더 처지게 구동하지 않고 원래 범위로 복귀한다. 다른15개 관절과 모든 상한은 그대로다.
- 약지 끝과 근사 손바닥 상자의 `palm_collision_7|rf_tip` 한 쌍에만0.1mm 분류 오차를 적용했다. 접촉 힘과 메시를 제거하거나 모든 부품의 충돌 판정을 완화하지 않았다. MuJoCo의 부드러운 관절 제한에서 생기는 ID4 한계의1틱 이내 수치 잔차만 동역학에서 구분하며 목표와 입장 범위에는 적용하지 않는다. 물리 기하 보정 완료를 주장하지 않는다.
- 새 직접 읽기 `book_settled_off_readonly_20261024_180453_383542`는 안정 OFF, 목표 및 설정 쓰기0회다. 엄지 ID4는1782틱 그대로이고 ID16은3101틱으로 더 내려왔다. 이 자세의 모델 약지 겹침 약1.25mm는0.1mm 오차로 덮지 않았다. 약지 중간관절 ID15를80틱(약7도) 먼저 펴는40목표 자동 분리를 추가했다. 기존에 있던 세 약지 끝/손바닥 접촉만 깊어지지 않으며 분리해야 하고40목표 뒤에는 없어야 한다. 실제 새 읽기에서 약지 분리를 확인하지 못하면 엄지 목표를 보내지 않는다. 이어 엄지 끝64틱 분리 후 실측 확인이 있어야 뿌리 목표를 보낸다.
- v5는 새 실제 빈손 각도를 바꾸지 않은 입력으로201개 정적 및9개 동역학 조건을 통과했다. 총1103개 목표이며 PLACE 뒤279개 접근 목표는 이전 실물에서 도달한 그대로다. 최대 모델 추종 약0.116704rad다. 관련 장치 없는 검사84개도 통과했고 정책 변조와 다른 관절 수치 잔차 및 분리 실패가 차단됨을 확인했다. 전류450mA, PID, 실제 추종0.18rad 및 무진행과 전압 및 온도 중단은 유지했다.
- 다음 사용자 작업은 빈손 상태에서 `leap_book_cube_grasp_trial.cmd` 실행이다. 손가락과 책 및 본체는 그대로 둔다. 첫 SPACE는 빈손 자동 복귀, PLACE에서만 큐브를 놓고 두 번째 SPACE로 느린 파지, REMOVE에서 큐브를 뺀 뒤 세 번째 SPACE로 OFF다. 이번 실행은 자동 복귀와 파지 확인이며 회전은 아직 포함하지 않는다. 실행 시 새 안정 OFF 읽기와 실제 시작 경로 검사를 다시 수행한다. 원본 실물 결과와 과거 증명 및 전시 문구는 수정하지 않았다.
- 근거: `right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v5/screen_final.json`, `device_free_validation.json`, `readiness_review.json`, `right_hand_rotation_20261024/observed_model_envelope_audit/result.json`. 참고 원본 모델: https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/leap_hand/right_hand.xml .

'''
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    path=ROOT/name;prior=path.read_text(encoding='utf-8')
    if not prior.startswith(entry.splitlines()[0]):
        if prior.startswith('## CURRENT'):prior=prior.replace('## CURRENT','## PREVIOUS',1)
        path.write_text(entry+prior,encoding='utf-8')
print(json.dumps({k:v for k,v in r.items() if k not in ('device_free_validation','actual_ticks')},ensure_ascii=False))
