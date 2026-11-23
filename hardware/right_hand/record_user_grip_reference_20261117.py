"""Record a user-confirmed physical grip reference, without device access."""
from pathlib import Path
import hashlib, json
from loaded_quarter_joint_limits_20261025 import configured_limits

root = Path(__file__).resolve().parent
source = root/'ring_reference_readonly_20261117_153805_503148/result.json'
raw = json.loads(source.read_bytes())
samples = [{r['id']: r for r in s['motors']} for s in raw['snapshots']]
if raw['errors'] or not raw['all_16_saved_twice'] or not raw['serial_port_closed']:
    raise RuntimeError('Complete read-only source required')
rows = [samples[-1][i] for i in range(1, 17)]
lo, hi = configured_limits()
exceptions = [dict(id=r['id'], actual_ticks=r['position_ticks'], goal_ticks=r['goal_ticks_readonly'],
    existing_lower_ticks=float(lo[r['id']-1]), existing_upper_ticks=float(hi[r['id']-1]))
    for r in rows if not lo[r['id']-1] <= r['position_ticks'] <= hi[r['id']-1]
    or not lo[r['id']-1] <= r['goal_ticks_readonly'] <= hi[r['id']-1]]
record = dict(status='user_confirmed_physical_cube_hold_reference',
    timestamp_kst=raw['timestamp_kst'], source_path=source.relative_to(root).as_posix(),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    user_observation='큐브를 손으로 받쳐주지 않아도 현재 자세로 잡고 유지 중이라고 사용자가 확인함',
    preferred_reference_over_independent_model_grip=True, motor_ids=list(range(1, 17)),
    measured_ticks=[r['position_ticks'] for r in rows],
    holding_goal_ticks_readonly=[r['goal_ticks_readonly'] for r in rows],
    measured_angles_are_not_interchangeable_with_loaded_holding_goals=True,
    max_two_read_difference_ticks=max(abs(samples[1][i]['position_ticks']-samples[0][i]['position_ticks']) for i in range(1, 17)),
    existing_software_range_exceptions=exceptions, existing_limits_changed=False,
    original_motor_observations=rows, cube_pose_measured=False, contact_locations_measured=False,
    upper_layer_clearance_verified=False, reaching_path_verified=False,
    next_turn_and_regrasp_verified=False, executable_motion_candidate=False,
    new_SAC_training=False, this_recording_opened_devices=False)
out = root/'right_hand_rotation_20261117/user_measured_grip'
out.mkdir(exist_ok=False)
(out/'reference.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
(out/'README.md').write_text('''# 사용자 실물 파지 기준 · 2026-11-17

사용자가 현재 자세로 큐브를 손으로 받쳐주지 않고 잡아 유지 중이라고 확인했다.
앞으로 파지 작업은 이 실물 자세를 우선 기준으로 삼는다. 독립 초기화한 모델 후보의 유지 시간만 늘리는 작업을 우선하지 않는다.

16개 모터를 두 번 읽었으며 차이는0틱이다. 이 두 번의 읽기는 장시간 유지나 회전 연결 검증이 아니다. 읽을 당시 모든 토크ON, 장치 오류0, 읽은 뒤 포트 닫힘. 모터 명령 쓰기는 없었다.

reference.json에는 측정 각도와 당시 읽힌 유지 목표를 따로 보존했다. 특히 ID12는 현재2098틱, 목표2124틱이므로 측정 각도를 목표로 그대로 복사하면 누르는 힘이 달라질 수 있다. 현재 목표가 다른 시작 자세에서도 바로 실행 가능한 경로라는 뜻은 아니다.

큐브 자세·실제 접촉 위치·윗단 공간은 아직 측정하지 않았다. 이 관절값에 큐브 위치를 임의로 맞춘 모델 결과를 실물 파지 검증으로 설명하지 않는다.

기존 소프트웨어 범위와 비교하면 ID11 현재3279틱/목표3284틱이 상한3276.83틱보다 각각 약2.17틱/7.17틱 높다. 이 차이를 기록했으며 제한을 바꾸거나 값을 자동으로 잘라 새 목표를 만들지 않았다. 진입 경로를 준비할 때 이 관절의 실제 동작 범위를 따로 반영해야 한다.

다음 작업은 이 실물 자세의 접촉·윗단 공간을 확인하고, 실제 회전 끝에서 이 파지로 들어오는 전환을 준비하는 것이다. 자동 구동 파일이나 새 정책은 이번 기록에서 생성하지 않았다.
''', encoding='utf-8')
block = '''<!-- user_measured_grip_20261117 -->
## 최신 우선 기준: 사용자가 실제로 큐브를 유지 중인 파지

사용자는 모델 후보 그림을 본 뒤 현재 실물 자세 읽기를 요청했고, 2026-11-17 15:38 KST에16개 관절을 PING/READ만으로 두 번 저장했다. 이어 “큐브를 잡고 유지 중”이라고 확인했다. 앞으로 이 자세를 우선 파지 기준으로 삼는다. 원본은 `ring_reference_readonly_20261117_153805_503148/result.json`, 정리본은 `right_hand_rotation_20261117/user_measured_grip/reference.json`과 `README.md`다. 측정 각도와 당시 유지 목표를 구분한다(ID12 현재2098/목표2124틱). ID11의 기존 소프트웨어 상한과의 작은 차이를 기록했으며 제한·모터 설정을 변경하지 않았다. 사용자 유지 관찰과 모델 검증을 구분하며 실제 윗단 공간·접촉 위치·진입/다음 회전 경로는 아직 미검증이다. 독립 모델 후보의20초 유지 성공을 이 실물 파지의 성공으로 섞지 않는다.
<!-- /user_measured_grip_20261117 -->

'''
for name in ('cloud_handoff_20261106/CLOUD_START_PROMPT.md', 'professor_progress_20261015.md'):
    path = root/name
    previous = path.read_text(encoding='utf-8')
    if '<!-- user_measured_grip_20261117 -->' in previous:
        raise RuntimeError('Reference handoff already recorded')
    path.write_text(block+previous, encoding='utf-8')
print(json.dumps(dict(saved=str(out/'reference.json'), measured=record['measured_ticks'],
    goals=record['holding_goal_ticks_readonly'], range_exceptions=exceptions), ensure_ascii=False))
