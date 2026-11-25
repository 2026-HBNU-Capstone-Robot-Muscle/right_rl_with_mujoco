"""Record the partial outcome, create a direction guide and an OFF-only native-screen request."""
import hashlib
import html
import json
from pathlib import Path
from forward_cube_placement_20261026 import ROOT, SOURCE, SOURCE_SHA, REVIEW, FILES, sha, ForwardPlacementPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS

OUT = REVIEW.parent
READONLY = ROOT/'finish_empty_readonly_20261026_012502_138255/result.json'
CURRENT_FIXTURE = OUT/'forward_placement_current_off_fixture'
MODEL_ROOT = ROOT/'right_hand_rotation_20261024/support_photo_review_221703/level_book_seating'

def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

def main():
    if sha(SOURCE) != SOURCE_SHA:
        raise ValueError('Actual loaded result changed')
    source = json.loads(SOURCE.read_bytes())
    previous = json.loads((OUT/'initial_review.json').read_bytes())
    diagnostic = []
    for name in ('measured_loaded_005744_root_seating', 'measured_loaded_005744_mu03'):
        p = MODEL_ROOT/name/'seating_grid.json'
        r = json.loads(p.read_bytes())
        diagnostic.append(dict(path=p.relative_to(ROOT).as_posix(), sha256=sha(p),
            cases=len(r['cases']), accepted=sum(bool(v.get('candidate_accepted')) for v in r['cases']),
            hardware_execution_allowed=False, assumptions_not_physical_calibration=True))
    searches = []
    for sub in ('', 'larger_root_clearance/'):
        p = OUT/'thumb_clearance_search'/sub/'search.json'
        r = json.loads(p.read_bytes())
        cases = r['cases']
        searches.append(dict(path=p.relative_to(ROOT).as_posix(), sha256=sha(p),
            cases=len(cases), accepted=sum(bool(v.get('candidate_accepted')) for v in cases),
            hardware_execution_allowed=False))
    write(OUT/'physical_partial_review.json', dict(
        source=SOURCE.relative_to(ROOT).as_posix(), source_sha256=SOURCE_SHA,
        command_sequence_completed=source['reference_and_finish_commanded'],
        completed_loaded_goals=824, physical_ninety_degree_rotation_verified=False,
        operator_report='기울어진 채 멈춤. 엄지에 걸려 넘어가지 못한 것 같다고 보고함.',
        thumb_obstruction_is_operator_hypothesis=True, exact_contact_part_identified=False,
        automatic_image_angle_measurement_valid=False, cube_hand_registration_verified=False,
        max_tracking_rad=previous['max_tracking_rad'], peak_current_ma=previous['peak_current_ma'],
        min_voltage_v=previous['min_voltage_v'], all_torque_off_verified=source['all_torque_off_verified'],
        settings_restored=source['settings_restored'], conditional_initial_seating_diagnostics=diagnostic,
        additional_thumb_root_searches=searches,
        next_change='Approximate5mm initial cube placement shift toward fingertips, away from thumb/wrist. Same motor goals.',
        cause_confirmed=False, unchanged_placement_repeat_requested=False))
    write(REVIEW, dict(review_ready=False, source_sha256=SOURCE_SHA,
        prior_physical_flip_completed=False, placement_only_comparison=True,
        requested_forward_shift_mm=5., cube_hand_registration_verified=False,
        physical_ninety_degree_rotation_verified=False,
        mode_code_hashes={n:sha(ROOT/n) for n in FILES},
        actual_partial_review='physical_partial_review.json',
        previous_native_route_proof_reused_without_relabeling=True,
        hand_targets_or_control_settings_changed=False, program_tests_passed=None))
    off = json.loads(READONLY.read_bytes())
    if off['status'] != 'off_snapshots_saved' or off['maximum_drift_ticks'] > 2 or off['servo_write_commands_sent']:
        raise ValueError('Stable read-only source required')
    CURRENT_FIXTURE.mkdir(exist_ok=True)
    request = dict(plan_version=ForwardPlacementPlan.PLAN_VERSION,
        motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS), origin='assistant_readonly_stable_off_reads',
        off_state=off['snapshots'][-1], readonly_source=READONLY.relative_to(ROOT).as_posix(),
        readonly_source_sha256=sha(READONLY))
    write(CURRENT_FIXTURE/'request.json', request)
    # Original image is referenced, not edited. SVG marks direction only; it
    # does not assert calibrated pixels/mm or a registered target footprint.
    image_path = '../../'+SOURCE.parent.name+'/cube_before_turn.jpg'
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>다음 큐브 배치 방향</title><style>
body{margin:0;background:#14202b;color:#f5f8fb;font:18px/1.65 system-ui,"Malgun Gothic",sans-serif}main{max-width:1160px;margin:auto;padding:22px}h1{font-size:27px;margin:0 0 12px}.note{background:#233444;padding:14px 20px;border-radius:10px;margin:16px 0}b{color:#8bffcf}.view{position:relative;background:black;overflow:hidden;border-radius:12px}.view img{display:block;width:100%}.view svg{position:absolute;inset:0;width:100%;height:100%}button{font:inherit;padding:7px 15px;cursor:pointer;border:1px solid #7f9baa;border-radius:7px;background:#233444;color:white}svg text{font-family:system-ui,"Malgun Gothic",sans-serif;font-weight:700;paint-order:stroke;stroke:#10202c;stroke-width:7px;stroke-linejoin:round}.off .marks{display:none}.legend{font-size:16px;color:#d3dce2}
</style><main><h1>이전 위치에서 손끝 쪽으로 약 5mm</h1>
<p>손을 따로 펴거나 옮길 필요는 없어. 프로그램이 빈손 준비를 마친 뒤 <b>PLACE CUBE</b>가 나올 때 큐브를 놓아.</p>
<div class="view" id="view"><img src="IMAGE_PATH" alt="직전 실행에서 회전 시작 전 실제 큐브 위치"><svg viewBox="0 0 1280 720" aria-label="사진 위쪽 손끝 방향으로 조금 이동">
<defs><marker id="arrow" markerWidth="10" markerHeight="10" refX="8" refY="5" orient="auto"><path d="M0,0 L10,5 L0,10 Z" fill="#7bffd2"/></marker></defs>
<g class="marks"><path d="M930 350 L946 190" fill="none" stroke="#7bffd2" stroke-width="8" marker-end="url(#arrow)"/>
<text x="958" y="248" fill="#7bffd2" font-size="29">손끝 쪽</text><text x="958" y="290" fill="#7bffd2" font-size="27">약 5mm</text>
<path d="M572 132 L844 175 L805 355 L563 290 Z" fill="#70ffca" fill-opacity=".07" stroke="#7bffd2" stroke-width="4" stroke-dasharray="9 7"/>
<text x="722" y="443" fill="white" font-size="25">엄지와 손목에서 멀어지는 쪽</text>
<text x="432" y="700" fill="white" font-size="25">손목 / ID41</text></g></svg></div>
<p><button onclick="document.getElementById('view').classList.toggle('off')">방향 표시 켜기 / 끄기</button></p>
<p class="legend">원본 사진은 직전 배치야. 화살표와 점선은 이동 방향 안내이며, 정합된 목표 위치나 실측 길이는 아니야.</p>
<div class="note"><b>초록 위, 주황은 손목 ID41 방향</b>을 유지해. 이전보다 손끝 쪽으로 약 5mm 옮겨서 검지와 중지의 평평한 모터 커버 위에 자연히 받쳐줘. 손바닥 홈에 밀어 넣지 마. 혼자 받쳐지지 않으면 Q로 멈춰.</div>
<p>첫 SPACE는 빈손 시작. PLACE에서 배치하고 사람 손을 뺀 뒤 둘째 SPACE. REMOVE에서 큐브를 먼저 빼고 셋째 SPACE로 모터를 꺼.</p>
<p class="legend">이번에는 배치만 비교해. 회전 목표 824개와 전류, 속도, 중단 기준은 직전 실행과 같아. 90도 성공 여부는 저장 영상과 실제 관찰로 확인해.</p></main></html>'''
    (OUT/'forward_placement_guide.html').write_text(page.replace('IMAGE_PATH',html.escape(image_path,quote=True)), encoding='utf-8')
    print(json.dumps(dict(partial_review_recorded=True, native_request=str(CURRENT_FIXTURE/'request.json'),
        guide=str(OUT/'forward_placement_guide.html'), thumb_root_cases_rejected=sum(v['cases'] for v in searches)), ensure_ascii=False))

if __name__ == '__main__':main()
