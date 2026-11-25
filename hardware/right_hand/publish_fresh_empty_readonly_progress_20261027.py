"""Record that the requested power/read step is complete, without motor writes."""
import datetime,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026';latest=json.loads((OUT/'fresh_empty_start_latest.json').read_bytes());folder=ROOT/latest['folder'];r=json.loads((ROOT/latest['result']).read_bytes());route=json.loads((folder/'recovery_plan_v2_ring_first/review.json').read_bytes())
state=dict(status='fresh_read_complete_new_empty_route_sensitivity_running',updated_local=datetime.datetime.now().isoformat(timespec='seconds'),power_confirmation_received=True,cube_removed_user_confirmed=True,pending_user_input=None,next_user_action=None,next_user_command=None,readonly_source=latest,readonly_snapshots=3,readonly_max_drift_ticks=r['max_drift_ticks'],readonly_min_voltage_v=r['min_voltage_v'],readonly_max_temperature_c=r['max_temperature_c'],all_motors_torque_off=True,motor_writes_sent_this_turn=0,serial_ports_opened_readonly=True,camera_opened_this_turn=False,empty_route_goals=route['goals'],empty_route_nominal_seconds=route['nominal_seconds'],root_support_separation_nominal_seconds=route['root_escape_first_clear_sample']*.075,early_ring_release_ticks=80,geometry_model_only=True,full_sensitivity_complete=False,ready_for_operator_empty_trial=False,ready_for_real_yaw=False,physical_vertical_successes_total=2,physical_horizontal_90_verified=False,additional_marker_measurement_required=False,additional_capture_required=False)
for name in ('horizontal_next_step_20261026.json','current_camera_and_yaw_review_20261026.json'):
    p=OUT/name;old=json.loads(p.read_bytes());old.update(state);p.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(folder/'progress_20261027.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
block='''## CURRENT 2026-10-27: 전원 확인 완료, 현재 각도 3회 읽기 및 새 빈손 경로 검사

- 사용자가 5V 전원 ON 및 큐브 제거를 확인했다. COM7에서 16개 XC330-M288을 PING/READ로만 세 번 읽었고 관절값 차이는 0틱이었다. 모두 토크 OFF, 하드웨어 오류 0, watchdog 0이다. 읽은 전압은 4.3~4.8V, 최대 온도는 22도였다. 목표/토크/설정 쓰기는 0회다. 기존 전원 확인 질문은 해결됐고 현재 사용자 다음 작업은 없다.
- 원본 읽기와 반복 읽기는 `right_hand_rotation_20261026/fresh_empty_start_20261026_235258/result.json`에 보존했다. 현재 각도는 승인된 ID4 하한 -2도 예외를 포함한 설정 범위 안이다. 모델상 초기 검지/중지-유리 겹침을 기존 범위 안에서 줄이는 경로를 찾았다. 현재 각도가 그대로 움직인다는 명령각 재생에서는 약 1.35초에 초기 겹침이 해소됐지만 실물 접촉이나 실제 해소 시간을 확정한 것은 아니다.
- 약지를 접은 채 기다리는 추종 모델에서 초기 약지 끝-손바닥 접촉이 나타나 기존 80틱 약지 PIP 분리를 맨 먼저 하는 순서로 수정했다. 초기 3개 약지/손바닥 쌍은 이미 쓰던 4mm 모델 분리 범위와 실제 OFF +/-4틱 관측 범위로 평가하고, 분리 뒤에는 기존 무겹침 기준을 유지한다. 새 접촉/더 깊은 접촉 또는 모터 건강 기준을 허용하도록 넓히지 않았다.
- 새 빈손 제안은 총 1,575개 목표, 75ms 주기 기준 약 118초다. 현재각에서 뿌리 분리, 약지 열기, 엄지/중지 파지 자세까지 이동하고 자동 OFF하며 큐브와 SAC 동작은 없다. 전체 시작각/받침/추종/카메라 지연 검사는 진행 중이며 실행 준비 완료로 표시하지 않는다. `leap_fresh_empty_grasp.cmd`는 검토 자료가 완성되지 않으면 카메라/모터를 열기 전에 거부하도록 준비 중이다. Python 기본 실행은 장치를 열지 않는 검사 모드로 통과했다.
- 실물 수직 뒤집기 성공 총 2회 및 수평 90도 미검증 상태를 유지한다. 빨강 ID2 약 10.5~10.7mm 답변은 반영했고 추가 치수 측정/촬영은 요청하지 않는다. 전시 족자 파일은 수정하지 않는다.

<!-- END FRESH READONLY PROGRESS 20261027 -->

'''
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8-sig');marker='<!-- END FRESH READONLY PROGRESS 20261027 -->'
    if marker in old:old=old.split(marker,1)[1].lstrip()
    if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
    p.write_text(block+old,encoding='utf-8')
expected='cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0';assert hashlib.sha256((ROOT/'exhibition_20261023/족자_붙여넣기.txt').read_bytes()).hexdigest()==expected
print(json.dumps({k:v for k,v in state.items() if k!='readonly_source'},ensure_ascii=False,indent=2))
