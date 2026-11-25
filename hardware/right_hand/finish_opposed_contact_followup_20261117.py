"""Collect independently verified progress, with unsuccessful trials last."""
from pathlib import Path
import ast, hashlib, html, json
ROOT = Path(__file__).resolve().parent
BASE = ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
CHAIN = BASE/'core_support_chain_10s'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


chosen = CHAIN/'predictive_verified20s/review.json'
verified = read(chosen)
a = verified['audit']
if not verified['passed'] or a['certificate']['queries'] != a['expected_hold_steps'] or a['stop']:
    raise RuntimeError('Independent complete working hold required')
model = ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
if digest(model) != verified['model_sha256']:
    raise RuntimeError('Verified model changed')
frozen = read(ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json')['code_hashes']
changed = [name for name, expected in frozen.items() if not (ROOT/name).is_file() or digest(ROOT/name) != expected]
if changed:
    raise RuntimeError('Frozen original source changed: '+str(changed))
scripts = ['predictive_opposed_hold_20261117.py', 'verify_predictive_opposed_prefix_20261117.py',
    'gait_middle_contact_20261117.py', 'opposed_force_slide_control_20261117.py',
    'opposed_grip_force_balance_20261116.py', 'screen_opposed_cube_placement_20261117.py',
    'active_opposed_chain_hold_20261117.py', 'plan_opposed_thumb_transfer_20261117.py',
    'check_opposed_support_transfer_20261117.py', 'render_opposed_grip_review_20261117.py',
    Path(__file__).name]
for name in scripts:
    ast.parse((ROOT/name).read_text(encoding='utf-8'))
paths = [BASE/'support_transfer_feasibility/review.json', BASE/'thumb_transfer_feasibility/review.json',
    BASE/'middle_gait_v1/review.json', BASE/'middle_gait_v1_eq20/review.json',
    BASE/'placement_x3_roll0_hold30/review.json', BASE/'placement_x3_roll-3_hold30/review.json',
    CHAIN/'active_hold30_normal0p7/review.json', CHAIN/'predictive_hold30_h3_edge0.5/review.json',
    CHAIN/'predictive_hold30_h3_edge0.5_planes/review.json', CHAIN/'predictive_hold30_h3_edge0.5_guarded/review.json',
    CHAIN/'predictive_next_turn-5_h3_edge0.5_guarded/review.json',
    CHAIN/'predictive_next_turn5_h3_edge0.5_guarded/review.json']
for p in paths:
    if not p.is_file():
        raise RuntimeError('Trial still pending: '+str(p))
trials = [{ 'path': p.relative_to(ROOT).as_posix(), 'sha256': digest(p), 'result': read(p)} for p in paths]
next_turns = [t for t in trials if 'predictive_next_turn' in t['path']]
next_lines = []
for t in next_turns:
    r = t['result']
    audit = r.get('audit')
    if r['passed']:
        next_lines.append(f"다음 {r['next_turn_target_deg']:g}도 연구 회전과 마지막 {audit['evaluated_hold_seconds']:.3f}초 유지가 전체 연속 재생을 통과했다.")
    else:
        next_lines.append(f"다음 {r['next_turn_target_deg']:g}도 비교는 최종 파지 검사를 통과하지 못했다. 중단: `{r['stop']['reason']}`.")
result = dict(status='verified_postturn_hold20s_and_contact_reposition_research',
    working_hold_passed=True, selected_review_path=chosen.relative_to(ROOT).as_posix(), selected_review_sha256=digest(chosen),
    selected_audit=a, frozen_source_files=len(frozen), frozen_source_mismatches=changed,
    source_hashes={name:digest(ROOT/name) for name in scripts}, trials=trials,
    small_turn_only=True, two_small_turns_and_holds_passed=any(t['result']['passed'] for t in next_turns),
    full_quarter_turn_cycle_verified=False, middle_contact_reposition_verified=False,
    new_SAC_training=False, perfect_simulated_state_for_prediction=True, devices_opened=False,
    physical_grasp_verified=False, hardware_execution_allowed=False)
(BASE/'CONTACT_REPOSITION.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
seconds = a['evaluated_hold_seconds']
md = f'''# 회전 뒤 파지 보정 후속 결과 · 2026-11-17

기존 유지 → 작은5도 연구 회전 → 앞뒤 하부 재파지를 처음부터 이어 재생하고, 마지막 작업 파지 **{seconds:.3f}초 유지**를 독립 검사했다. 기존10.05초 결과에서 유지 시간이 늘었다. 손·큐브 상태를 구간 사이에 초기화하지 않았다.

| 마지막 작업 파지 검사 | 값 |
|---|---:|
| 검사 시간 | {seconds:.3f}초 |
| 최대 중심 오차 | {a['maximum_center_error_mm']:.3f}mm |
| 최대 자세 오차 | {a['maximum_angle_error_deg']:.3f}도 |
| 엄지·중지 하부 접촉 비율 | {a['required_lower_contact_fraction']*100:.3f}% |
| 앞뒤 지지 방향 비율 | {a['opposed_front_back_fraction']*100:.3f}% |
| 증명된 윗단 최소 여유 | {a['minimum_exact_support_plane_gap_mm']:.3f}mm |
| 실제 공간 조회 수 | {a['certificate']['queries']} |

852목표 전체를 원래 시작 모델 상태에서 재생했다. 마지막267목표가20.025초 파지다. 공간 실패 무시나 건너뛴 검사는 없다. 이 시작 상태는 독립 초기화한 모델 파지를 안정시킨 저장 상태다. 실제 로봇의 초기 자세에서 진입하는 경로를 검증한 것은 아니다.

관절 목표를 조금 바꿨을 때의 자유 동역학을0.225초씩 예측해 다음 목표를 선택하는 별도 연구 제어기를 추가했다. 완전한 모델 상태를 쓰는 오프라인 계산이며 SAC 추가 학습·증류·실물 정책 배포는 하지 않았다. 실제 실행 장치,카메라,포트 접근은0이다. 예측용 상태 복사와, 채택된 연속 궤적에서 큐브를 강제로 옮기는 것은 구분한다. 채택 경로에는 큐브 위치 재설정이나 외력 주입이 없다.

원본170개 해시와 기존 실제 관절 범위·추종·충돌 기준을 유지했다. 관절 목표는 최대2틱/75ms다. 모델의0.15N·m 설정을 실제450mA 환산값으로 사용하지 않는다. 끝 자세는 중력 기준 약{a['final_gravity_tilt_deg']:.2f}도 기울어졌으므로 예전 수평5도 과제 실패는 보존한다.

- [20초 독립 검사](core_support_chain_10s/predictive_verified20s/review.json)
- [모든 후속 비교와 소스 해시](CONTACT_REPOSITION.json)
- [20초 마지막 모델 그림](predictive_hold_evidence/regrasp_verified20s_side.png)
- [이전10초 검사와 경로 출처](REGRASP_FOLLOWUP.md)

## 미완료 및 실패 기록

30초 생성은 성공하지 못했다. 보정 경로는 마지막 파지 시작부터 약23.941초에 윗단 공간 하한0.299927mm로0.3mm 기준에 미달했고, 그 시점 엄지 하부 접촉은 없었다. 이전 고정 목표 비교의14.643초 중단과 구분한다. 그 실패 이전133목표를 선택해 위20.025초 과제로 고정하고 독립 재검사했다.30초 성공이나 영구 유지로 확대하지 않는다.

중지 접촉을 모서리 안쪽에 다시 자리 잡게 하는 문제는 해결되지 않았다. 보정 중에도 접촉점이 가장자리에 가까워지는 추세가 남았다. 유지 시간 증가는 확인했지만 접촉 위치 개선을 완료했다고 말하지 않는다.

현재 접촉에서 검지·엄지만으로 중지 하중을0.1N 이하로 넘기는 정적 힘 균형은 성립하지 않았다. 엄지 접촉을 큐브 가로−5mm 쪽으로 옮기는 별도 정적 자세에서는 힘 균형이 가능한 해를 찾았으나, 실제 동역학으로 그 자세에 옮겨 가는 두 비교는 모두 실패했다. 정적 계산을 실제 재파지 성공으로 쓰지 않는다.

큐브를 처음부터3mm 옮겨 놓은 두 비교는 회전 중 모델 손가락 간섭 검사에 걸렸다. 이들은 초기 배치를 바꾼 민감도 검사이며 손 안에서3mm를 옮긴 연속 동작이 아니다. 초기 예측 제어 비교의 미세 모델 간섭/관절 경계 미달 기록도JSON에 보존했다. 실제 충돌 크기를 측정한 결과로 해석하지 않는다.

{chr(10).join(next_lines)}

반대 방향−5도 비교는 중간에 목표각 근처에 갔지만 엄지·중지 하부 접촉이 사라졌고, 끝에는 모델 큐브가 외부 받침에 닿아 중단됐다. 같은 방향+5도 비교는 실제 약2.73도에서 기존 관절/추종/자기접촉 예측 검사를 통과하는 후보를 찾지 못해 다음 목표를 보내지 않고 종료했다. 두 비교 모두 마지막10초 작업 파지 검증에 들어가지 못했다.

원하는 모든 방향의90도 회전 → 파지 → 다음 회전 → 파지, 실제 시작 자세에서의 진입, 배치·마찰·구동 오차 및 실물 검증은 남아 있다. 새 실물 실행.cmd를 배포하거나 기존 실행파일을 바꾸지 않았다.
'''
(BASE/'CONTACT_REPOSITION.md').write_text(md, encoding='utf-8')
block = f'''<!-- contact_reposition_20261117 -->
## 최신: 회전 뒤 작업 파지20.025초 독립 재생 통과

기존 유지→작은5도 회전→하부 앞뒤 재파지를 처음부터852목표 연속 재생했다. 마지막20.025초 작업 파지는 통과, 최대 중심0.925mm·자세0.958도·엄지중지 접촉99.985%·윗단 여유 하한0.605mm·공간20025표본 전부 통과다. 원본170개 해시·실제 관절 범위·기존 동적 기준·목표2틱/75ms 유지. 새 오프라인 예측 보정 제어이며 새 SAC 학습이나 증류가 아니다. 모델 완전 관측만 사용했고 장치 접근0.

30초 생성은총23.941초에 공간 기준과 엄지 하부 접촉을 잃어 실패했다. 그전 구간을 별도20.025초 과제로 정해 독립 검증한 결과이며30초 성공이 아니다. 중지 모서리 접촉의 안쪽 이동은 아직 미해결, 정적 엄지 이동 해와 실제 이송 실패를 구분한다. 실제 로봇 시작 자세 진입,90도 반복/다음 회전 연결과 실물 준비 완료로 안내하지 않는다. 최신 전체 근거: `right_hand_rotation_20261117/opposed_lower_grip/CONTACT_REPOSITION.md`, `CONTACT_REPOSITION.json`.
<!-- /contact_reposition_20261117 -->

'''
for filename in ('cloud_handoff_20261106/CLOUD_START_PROMPT.md', 'professor_progress_20261015.md'):
    path = ROOT/filename
    old = path.read_text(encoding='utf-8')
    if '<!-- contact_reposition_20261117 -->' in old:
        raise RuntimeError('Do not duplicate latest handoff')
    path.write_text(block+old, encoding='utf-8')
path = BASE/'index.html'
page = path.read_text(encoding='utf-8')
anchor = '<p><a href="REGRASP_FOLLOWUP.md">'
pos = page.index(anchor)
current = f'''<section style="padding:18px;border:1px solid #63bc9a;border-radius:10px;margin:20px 0"><h1>最新：小回転後の把持20.025秒を独立検証</h1><p><strong>작은5도 회전 → 재파지 →20.025초 유지: 모델 검사 통과.</strong> 최대 위치0.925mm·자세0.958도·윗단 여유 하한0.605mm. 모든20,025개 공간 표본을 확인했다.</p><p>완전한 모델 관측을 쓰는 오프라인 예측 보정이다.90도 반복 동작이나 실물 준비가 끝난 것은 아니다.</p><p><a href="CONTACT_REPOSITION.md">최신 결과와 남은 문제</a> · <a href="CONTACT_REPOSITION.json">검사 원본과 해시</a> · <a href="core_support_chain_10s/predictive_verified20s/review.json">20초 독립 검사</a></p><img src="predictive_hold_evidence/regrasp_verified20s_side.png" alt="작은 회전 뒤20초 작업 파지를 통과한 마지막 모델 상태"></section>'''
current = current.replace('最新：小回転後の把持20.025秒を独立検証', '최신: 작은 회전 뒤 파지20.025초 독립 검증')
page = page[:pos]+current+page[pos:]
page = page.replace('<h1>앞뒤 하부 파지 · 2026-11-17</h1>', '<h2>기존 회전 전 초기 파지의 별도30초 검사</h2>')
page = page.replace('<b>회전 뒤 작업 파지 복구가 남아 있다.</b>', '위 옛 분기의 실패는 보존하며, 새 경로의 회전 뒤20.025초 파지는 상단 최신 검사를 참고한다.')
path.write_text(page, encoding='utf-8')
print(json.dumps(dict(selected_passed=verified['passed'], seconds=seconds, original_hashes=len(frozen),
    next_rotation_passed=[t['result']['passed'] for t in next_turns], report=str(BASE/'CONTACT_REPOSITION.md')), ensure_ascii=False))
