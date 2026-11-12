"""Save actual stop, tested fast transport, and the outstanding physical task."""
import json,hashlib,base64
from pathlib import Path
from retained_slow_yaw_plan_20261029 import ROOT,OUT,PROOF,hashes
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
proof=read(PROOF);assert proof['code_hashes']==hashes()
run=ROOT/'retained_slow_yaw_20261029_014356_533789';r=read(run/'result.json');saved=read(run/'offline_review/saved_summary.json')
review=dict(status='stopped_during_loaded_grasp_before_yaw',run=run.relative_to(ROOT).as_posix(),result_sha256=sha(run/'result.json'),
    all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],cube_removal_confirmed=False,
    physical_horizontal_rotation_executed=False,error=r['error'],phase=r['failed_phase'],measured=saved['measured'],
    camera_usable_frames=825,camera_blank_frames=0,read_failures=r['read_failures'],
    confirmation_events=r['read_recovery_confirmation_events'],
    interpretation='First corrupt read recovered; the second complete snapshot did not fit the shared200ms. Its last reply window was0.629ms, so its incomplete packet is not evidence of a separate hardware defect.',
    placement_review='Cube appears in a different position relative to palm ID40 than the retained transfer reference; no metric offset certified.',
    current_angles_not_yet_admitted_as_empty=True,next_physical_task='Remove cube only and confirm; keep fingers and support unchanged')
(run/'offline_review/physical_and_transport_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
checks=dict(status='fast_read_offline_reference_passed_waiting_for_cube_removal',transport_regression_tests_passed=16,
    fast_decoder_tests_passed=10,operator_integration_tests_passed=12,original_health_confirmation_tests_passed=12,
    test_total=50,fast_read_evidence=proof['fast_health_read_evidence'],
    firmware_evidence='firmware_transport_readonly_20261029_014828_559357/result.json',
    reference_only=True,fresh_current_empty_hand_review_pending=True,assistant_motor_writes=0,
    files={n:sha(ROOT/n) for n in ('test_combined_quiet_reserve_20261029.py','test_fast_health_decode_20261029.py',
        'test_retained_slow_yaw_operator_20261029.py','test_precise_two_full_20261028.py')})
(OUT/'verification.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
# Native SVG with embedded unmodified camera images for the next placement.
def data(relative):return 'data:image/jpeg;base64,'+base64.b64encode((ROOT/relative).read_bytes()).decode('ascii')
a=data('pitch_slow_yaw_20261029_004532_520638/cube_transfer_hold.jpg');b=data('retained_slow_yaw_20261029_014356_533789/cube_before_grasp.jpg')
svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="1320" height="570" viewBox="0 0 1320 570">
<rect width="1320" height="570" fill="#f5f7fa"/><g font-family="Malgun Gothic, sans-serif" fill="#182230">
<text x="20" y="34" font-size="23">큐브 배치 비교 — 다음 PLACE 대기 때 참고</text>
<text x="20" y="66" font-size="17">이전 자세 이동 뒤 큐브가 유지된 위치</text><text x="680" y="66" font-size="17">이번에 놓인 위치</text>
<image href="{a}" x="20" y="85" width="620" height="348.75"/>
<image href="{b}" x="680" y="85" width="620" height="348.75"/>
<rect x="233" y="143" width="114" height="159" fill="none" stroke="#00e5ff" stroke-width="4"/>
<rect x="821" y="122" width="133" height="164" fill="none" stroke="#ffb000" stroke-width="4"/>
<text x="20" y="474" font-size="21">빨강 중앙 ID2가 위, 초록 중앙 ID5가 손목 ID41 쪽</text>
<text x="20" y="510" font-size="18">다음에는 왼쪽 예시의 손바닥 마커와 엄지 사이 배치를 참고해 주세요.</text>
<text x="20" y="543" font-size="17">지금은 큐브만 빼고, 프로그램의 PLACE 안내가 나온 뒤 놓습니다.</text></g></svg>'''
(OUT/'placement_comparison.svg').write_text(svg,encoding='utf-8')
import xml.etree.ElementTree as ET
ET.fromstring(svg)
for path,digest in {'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}.items():assert sha(ROOT/path)==digest,path
for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    p=ROOT/name;d=read(p);d.update(status='fast_read_validated_awaiting_cube_removal',current_status='fast_read_validated_awaiting_cube_removal',
        next_physical_command_ready=False,next_command=None,next_step='User removes cube only; then read current OFF angles and screen fresh empty recovery',
        latest_retained_run=review,next_verification=checks,fast_health_read_evidence=proof['fast_health_read_evidence'],
        cube_removal_confirmation_pending=True,current_empty_recovery_review_pending=True,
        placement_reference=(OUT/'placement_comparison.svg').relative_to(ROOT).as_posix())
    p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 Fast Sync Read 검증 완료, 큐브 제거 확인 대기. 01:43 실행은 큐브를 놓고 잡는 구간에서 멈췄고 수평 회전은 시작하지 않았다. '
    '첫 읽기 오류는157ms에 복구했지만 두 번째 전체 상태 확인을 마치기 전에200ms 한도가 소진됐다. '
    '두 번째 미완성 패킷의 수신 창은0.629ms였으므로 별도의 모터 불량으로 판정하지 않는다. '
    '토크 OFF와 설정 복구 확인, 전압 최저4.9V, 전류 최고123mA, 추종 최대0.100rad, 카메라825장 정상이다. '
    'ROBOTIS 공식 Fast Sync Read는 X330 펌웨어46 이상에서 지원하며 실제16개 모터는 모두53이었다. '
    '8개씩 묶는 두 회신과 CRC 및 각 모터의 상태 오류 확인을 구현했다. 기존SDK1024바이트 한도를 유지한다. '
    '읽기 전용 실제 비교 중앙값은 기존34.74ms에서19.35ms로 감소했고160회 읽기와3회 호스트 데이터 오류 검사에 성공했다. '
    '두 번의 전체 상태 재확인은 오류 검사에서 약61~63ms였다. 움직임 중 영구 안정성을 인증한 결과는 아니다. '
    '관절 목표와 구동 간격, 전류 및 PID, 모터 건강 기준, 전체200ms 한도와 정상 상태2회 확인은 유지한다. '
    '검증된 과거 빈손 출발점 기준의 오프라인 검토와50개 테스트는 통과했다. 현재 큐브 제거 확인은 아직 없으므로 새 빈손 경로는 미확정이다. '
    '다음 할 일은 큐브만 빼고 확인받은 뒤 각도만 읽고 실제 현재 출발점의 복귀를 검사하는 것이다. '
    '실행 소스 v2와 당시 검토는 retained_slow_yaw_20261029_014356_533789/offline_review/execution_source_v2 에 보존했다.\n\n')
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if notice not in old:p.write_text(notice+old,encoding='utf-8')
print(json.dumps(dict(status='fast_read_ready_offline_pending_cube_removal',tests_passed=50,
    reference_source_hashes_valid=True,placement_svg=str(OUT/'placement_comparison.svg'),assistant_motor_writes=0),ensure_ascii=False))
