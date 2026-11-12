"""Record the user's placement clarification and corrected offline assumptions."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'prepared_cube_review_20261022'
observation=dict(source_trial='prepared_placement_capture_20261022_010440_125130',
    source_sha256='c7b713d509e16623b7be3bd593aeee55fb8cb7f2676905b561a1a25a0098d029',
    user_verbatim='손바닥에 놓으려했는데 id 40바로 옆에 붙여도 손가락 뿌리 쪽 파인부분으로 기울어지더라',
    interpretation='User attempted a palm placement beside ID40; cube tilted into the recessed root region.',
    contact_surface_quantitatively_measured=False,flat_palm_reset_not_assumed=True,
    cube_side_mm=55,cube_mass_kg=0.100,repeat_capture_requested=False,
    motor_commands_sent_by_assistant=0)
(OUT/'reported_support.json').write_text(json.dumps(observation,ensure_ascii=False,indent=2),encoding='utf-8')
p=OUT/'current_status.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(status='reported_root_tilt_modeled_offline_grasp_search_in_progress',
    cube_support_surface=observation['interpretation'],
    historical_bridge_rejected_for_deployment=True,
    historical_bridge_reason='Still unverified mounting/registration. Previous world-Z tilt rejection alone was incorrect: measured gravity is tilted; branch 000 is 11.1deg from actual gravity and qualitatively plausible for the reported tipping.',
    next_user_task=None,next_motion_launcher=None)
p.write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')
p=ROOT/'right_hand_start_20261019/objective_and_schedule.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(current_user_task=None,current_blocker='User reports cube tipping into root recess. Settled tilted support candidates built; collision-aware grasp/rotation adaptation is being computed offline.',
    latest_offline_review='prepared_cube_review_20261022/support_settling_scan.json',next_trial_review=None)
p.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
note='''## CURRENT 2026-10-22: 손바닥 배치에서 뿌리 홈으로 기울어짐 보고 반영

- 사용자: “손바닥에 놓으려했는데 id 40바로 옆에 붙여도 손가락 뿌리 쪽 파인부분으로 기울어지더라.” 평평한 손바닥에 고정된 reset을 실제 배치로 가정하지 않는다. 최종 접촉면/정확한 3D 좌표를 측정한 것은 아니지만, 홈 방향 기울어짐을 시작 조건 범위에 반영. 같은 촬영/구동을 다시 요청하지 않음.
- 앞선 해석 정정: 모델 world +Z가 측정 중력의 반대 방향과 같지 않다. 저장 중력은 `[-3.9700,-0.2684,-8.9676]m/s²`. 과거 marker bridge 000의 초록 법선은 worldZ와34.6°이지만 측정 중력 반대 방향과11.1°이다. 앞서 모든 해를 ‘윗면 불일치’로 제외한 근거는 불충분했다. 000은 사진의 손 윤곽과 조건부 투영에서 정성적으로 맞는 후보로 다시 분석하되, 과거 부착 관계/3D 좌표가 교정됐다고 보거나 컨트롤러에 설치하지 않음.
- 현재 관절과 55mm/100g 큐브로 주위35개 합성 낙하/settling 검사: 초기 관통 없고 정지해 손에 남은 후보25개. nominal은 손바닥 앞쪽 단차와 뒤쪽 면을 함께 받치며 초록 법선이 중력 반대 방향에서 약9.43° 기울어짐. 고정 관절 probe와 독립적인 동역학 1초 유지7개: 큐브 drift 약0.06~0.39mm, 최대 추종0.0331rad. 이는 실제 숨은 접촉면 측정이 아니라 사용자 관찰과 양립하는 모델 후보다.
- 새 기울어진 시작7개에서 예전 회전 의도/자기 접촉 회피 경로를 세 속도로 검사했지만, nominal 회전은 대부분 약10~20°에 그침. 단순히 속도를 늦추거나 엄지를 고정하는 것으로90° 회전이 해결되지 않음. 초기 접촉을 만드는 손가락 접근/파지부터 계산 중. 모든 새 결과는 offline only, 실제 회전 및 live SAC 성공은 미검증.
- 근거: `prepared_cube_review_20261022/reported_support.json`, `conditional_hand_overlay_000.jpg`, `support_settling_scan.json`, `tilted_start_path_comparison.json`, `finger_only_probe.json`. 모터 런타임/보호 조건 변경 없음, 조수 하드웨어 접근 없음.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;content=p.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:p.write_text(note+content,encoding='utf-8')
print('Recorded user-reported tipping and corrected gravity interpretation; no repeat user action requested.')
