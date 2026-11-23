"""Summarize only saved offline evidence; never publish a hardware-ready file."""
from pathlib import Path
from datetime import datetime
import ast, hashlib, html, json

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
AUDIT=BASE/'level_triple_normal1p2_settle1_settled_frame_active_hold400_heightlock0p0/complete_hold_audit/review.json'


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    audit=json.loads(AUDIT.read_bytes())
    if not audit['passed']: raise RuntimeError('Working hold is not verified')
    protected=json.loads((ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']
    bad=[p for p,h in protected.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    if bad: raise RuntimeError('Protected source changed: '+repr(bad))
    scripts=['level_supported_opposed_grip_20261117.py','audit_opposed_working_grip_20261117.py',
             'probe_opposed_grip_turn_20261117.py','inspect_opposed_turn_stop_20261117.py',
             'verify_opposed_postturn_hold_20261117.py','render_opposed_grip_review_20261117.py',Path(__file__).name]
    for name in scripts: ast.parse((ROOT/name).read_text(encoding='utf-8-sig'))
    turns=[]
    for path in sorted(BASE.glob('next_turn*/review.json')):
        d=json.loads(path.read_bytes())
        turns.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path),
            target_deg=d['target_turn_deg'],actual_deg=d['actual_turn_deg'],
            target_error_deg=d['target_error_deg'],center_error_mm=d['center_error_mm'],
            stop=d['stop'],passed=d['small_turn_and_hold_passed']))
    summary=dict(status='working_grip_hold_verified_cycle_unfinished',date='2026-11-17',
        recorded_at=datetime.now().astimezone().isoformat(),working_grip=dict(
            thumb='lower front',back='index and middle',underside='actual palm/hand support',ring_required=False),
        complete_hold_audit_path=AUDIT.relative_to(ROOT).as_posix(),complete_hold_audit_sha256=sha(AUDIT),
        hold_seconds=audit['evaluated_hold_seconds'],max_angle_deg=audit['maximum_angle_error_deg'],
        max_shift_mm=audit['maximum_center_error_mm'],opposed_contact_fraction=audit['opposed_front_back_fraction'],
        three_lower_contact_fraction=audit['lower_three_contact_fraction'],
        minimum_certified_upper_space_mm=audit['minimum_exact_support_plane_gap_mm'],
        original_gravity_upright_task_still_failed=True,source_gravity_tilt_deg=audit['source_gravity_tilt_deg'],
        source_is_independently_initialized_model_grip=True,perfect_simulated_feedback=True,
        reaching_path_verified=False,full_quarter_turn_cycle_verified=False,
        placement_friction_uncertainty_verified=False,left_hand_alignment_verified=False,
        new_SAC_training=False,physical_grasp_verified=False,hardware_execution_allowed=False,
        devices_opened=False,frozen_sources=dict(count=len(protected),unchanged=True),
        scripts={name:sha(ROOT/name) for name in scripts},turn_diagnostics=turns)
    def explain(t):
        if t['passed']: return '통과'
        stop=t['stop'] or {}
        if stop.get('reason')=='bounded_turn_not_reached':
            return f"각도 근접, 위치 변화 {t['center_error_mm']:.3f}mm로 고정 위치 복귀·유지 진입 미달"
        return '기존 기준 중단: '+', '.join(stop.get('failed',[stop.get('reason','미완료')]))
    failure_rows='\n'.join(f"| `{Path(t['path']).parent.name}` | {t['target_deg']:+g}° | {t['actual_deg']:+.3f}° | {t['target_error_deg']:.3f}° | {explain(t)} |" for t in turns)
    postturn=BASE/'next_turn5p0_palmclear_margin8_fingerclear_allpalms/reached_pose_fixed_hold30/review.json'
    post=json.loads(postturn.read_bytes()) if postturn.exists() else None
    summary['postturn_hold_diagnostic']=None if post is None else dict(path=postturn.relative_to(ROOT).as_posix(),
        sha256=sha(postturn),passed=post['passed'],stop=post['stop'],
        upper_gap_lower_bound_mm=post['minimum_exact_support_plane_gap_mm'],
        opposed_front_back_fraction=post['opposed_front_back_fraction'])
    (BASE/'STATUS.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    text=f'''# 앞뒤 하부 파지 — 2026-11-17

엄지 앞면 아래 + 검지·중지 뒷면 아래 + 실제 손바닥 받침의 **모델상 30초 유지**를 독립 재생으로 확인했다. 약지는 필수가 아니다.

| 30초 전체 1ms 검사 | 결과 |
|---|---:|
| 위치 변화 최대 | {summary['max_shift_mm']:.3f} mm |
| 자세 변화 최대 | {summary['max_angle_deg']:.3f}° |
| 앞뒤 하부 접촉 비율 | {100*summary['opposed_contact_fraction']:.2f}% |
| 세 손가락 하부 접촉 비율 | {100*summary['three_lower_contact_fraction']:.2f}% |
| 윗단 회전 공간의 증명된 최소 여유 | {summary['minimum_certified_upper_space_mm']:.6f} mm |
| 윗단 검사 | 30,000/30,000, 침범 0 |

손끝 접촉 높이를 큐브 작업 좌표계에서 유지하도록 보완했다. 기존과 동일한 모델·관절·목표 2틱/75ms·추종·충돌 기준으로 저장 목표를 다시 재생했다. 원본 결합 소스 {len(protected)}개 SHA256은 유지됐다.

- [전체 유지 검사]({AUDIT.relative_to(BASE).as_posix()})
- [기계 판독 상태](STATUS.json)
- [모델 그림](validated_hold_evidence/verified_hold30_side.png)
- [원본 생성 기록](level_triple_normal1p2_settle1_settled_frame_active_hold400_heightlock0p0/review.json)

## 판정 범위

이는 독립 초기화한 모델 파지가 안정된 상태에서 이어받은 결과다. 실물 시작 자세에서 이 파지에 도달하는 경로가 아니다. 현재 파지는 중력 수평 기준 약 {summary['source_gravity_tilt_deg']:.2f}° 기울어져 있으며, 기존 수평 5° 과제의 실패 판정은 그대로 보존했다. 손 기준 유지 판정은 별도로 기록했다. 왼손 정렬은 검증하지 않았다.

윗단 여유는 기존 형상 전체와 원통의 정확한 지지 평면으로 확인했다. 슬랩/반지름 화면 검사값은 원통 끝부분의 실제 유클리드 거리 하한이 아니므로 둘을 같은 mm 여유로 해석하지 않는다. 관련 이전 감사: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/upper_gap_metric_audit.json`. 0.300mm 모델 여유는 실물 오차 내성의 증거가 아니다.

완전한 모델 자세·접촉력을 이용한 보정 연구이며 새 SAC 학습이나 실물 정책 배포가 아니다. `.cmd` 실행파일을 새 파지에 교체하지 않았고 장치 접근·원격 업로드는 하지 않았다.

## 미완료 및 제외한 비교

1. 실제 회전 끝 자세에서 이 공통 파지로 옮기는 접근·재파지 경로.
2. 공통 파지 끝 상태에서 다음 90° 회전과 유지까지 이어지는 경로.
3. 연결 경로의 배치·마찰·구동 오차 검사 및 실물 확인.

다음 회전 진단은 30초 유지 검사의 실제 마지막 상태를 복원해 시작했다. 양 방향 작은 회전을 비교했으며, 초기화해서 회전한 성공으로 세지 않는다. 다음 단계는 접촉 전환과 회피를 포함한 회전 경로다.

가장 진행된 수정안은 **5° 목표에 실제 4.906°(자세 오차0.241°)**까지 도달했고 기존 구동·간섭 기준으로 400목표를 완료했다. 이 분기의 `bounded_turn_not_reached`는 **회전각이 안 나왔다는 뜻이 아니라, 중심이4.591mm 이동해 요청한 고정 위치에서 유지 단계로 넘어가지 못했다는 뜻**이다. 이후 도달 자세 그대로 고정 유지하는 별도 검사도 첫1ms에 윗단 공간 검사로 실패했다(분리 평면 하한 -0.944mm이며 실제 침투 깊이와 같지 않다). 해당 시점 앞뒤 손끝 하부 지지도 각0.2N 초과 기준에 미달했다. 손바닥 지지와 작업용 앞뒤 파지를 구분한다. 다음 수정의 핵심은 회전 뒤 옆면 접촉을 다시 확보하는 재파지이며, 이 분기를 회전→파지 유지 성공으로 세지 않는다.

| 분기 | 목표 | 실제 | 목표 자세 오차 | 중단/판정 |
|---|---:|---:|---:|---|
{failure_rows}

접촉 높이를 2mm 낮춘 비교는 약 18초 유지 후 손가락 형상 간섭으로 제외했다. 소스 접촉력 그대로 유지하는 비교도 윗단 여유 기준에 걸려 제외했다. 중단한 미세 모델 겹침을 큰 실물 충돌이나 큐브 낙하로 단정하지 않는다.
'''
    (BASE/'README.md').write_text(text,encoding='utf-8')
    table=''.join(f'<tr><td>{html.escape(Path(t["path"]).parent.name)}</td><td>{t["actual_deg"]:.3f}°</td><td>{html.escape(explain(t))}</td></tr>' for t in turns)
    page=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>앞뒤 하부 파지 검토 · 2026-11-17</title><style>body{{font:17px/1.7 system-ui,sans-serif;background:#101722;color:#edf2f7;max-width:1040px;margin:35px auto;padding:0 24px}}a{{color:#8ccaff}}strong{{color:#9de5bf}}img{{max-width:100%;border-radius:10px}}table{{border-collapse:collapse;width:100%;font-size:14px}}td,th{{padding:10px;text-align:left;border-bottom:1px solid #344154}}.metrics{{display:flex;gap:16px;flex-wrap:wrap}}.metrics p{{background:#1b2839;padding:15px 25px;border-radius:10px}}small{{color:#b8c6d8}}</style>
<h1>앞뒤 하부 파지 · 2026-11-17</h1><p><strong>모델상30초 유지 확인.</strong> 엄지 앞면 아래, 검지·중지 뒷면 아래, 손바닥 받침. 약지는 필수가 아니다.</p>
<div class="metrics"><p>최대 위치 변화<br><strong>{summary['max_shift_mm']:.3f}mm</strong></p><p>최대 자세 변화<br><strong>{summary['max_angle_deg']:.3f}°</strong></p><p>앞뒤 접촉 비율<br><strong>{100*summary['opposed_contact_fraction']:.2f}%</strong></p></div>
<p>30,000개1ms 검사에서 윗단 침범0. 증명된 최소 모델 여유 {summary['minimum_certified_upper_space_mm']:.6f}mm. 실물 오차 내성을 확인한 수치는 아니다.</p>
<figure><img src="validated_hold_evidence/verified_hold30_side.png" alt="30초 파지 검사 마지막 모델 자세"><figcaption>모델 그림. 빨강=엄지, 파랑=검지, 초록=중지. 반투명 주황 원통=윗단 회전 공간.</figcaption></figure>
<p>새 SAC 학습·실물 구동은 없었다. 독립 초기화한 모델 파지가 안정된 상태에서 시작했으며, 실제 시작 자세에서의 진입 경로는 아직 없다. 중력 기준 약9.47° 기울어진 작업 자세여서 기존 수평5° 과제 실패는 보존했다.</p>
<p><a href="README.md">상세 인계</a> · <a href="STATUS.json">검사 출처와 해시</a> · <a href="{AUDIT.relative_to(BASE).as_posix()}">30초 전체 검사</a></p>
<h2>남은 일과 실패한 비교</h2><p>실제 회전 끝에서 새 파지로 진입하기, 다음90° 회전 뒤 재파지, 연결 경로의 오차 검사와 실물 확인.</p>
<p>수정 경로는5° 목표에 실제4.906°까지 움직였다. 중심은4.591mm 이동했고, 그 도달 자세를 고정하는 검사에서도 윗단 공간과 앞뒤 손끝 접촉이 확보되지 않았다. <b>회전 뒤 작업 파지 복구가 남아 있다.</b> 90° 연속 동작이나 실물 시험 준비 완료가 아니다.</p>
<figure><img src="validated_hold_evidence/next_turn_probe_side.png" alt="다음 회전 뒤 재파지 미완료 모델 자세"><figcaption>회전 진단 끝 상태. 손 위 지지와 작업 파지 완료를 구분한다.</figcaption></figure>
<table><tr><th>모델 분기</th><th>실제 회전</th><th>판정</th></tr>{table}</table><p><small>원본170개 결합 해시와 기존 중단 기준 보존. 장치 접근0, 실물 실행파일 교체0, 원격 업로드0.</small></p></html>'''
    (BASE/'index.html').write_text(page,encoding='utf-8')
    block=f'''<!-- opposed_lower_grip_verified_20261117 -->
## 최신: 엄지 앞·검지/중지 뒤 하부 파지의 모델상 30초 유지 확인

사용자 목표는 새 앞뒤 하부 파지를 만들고 회전→유지→다음 회전→유지를 연결하는 것이다. 미들웨어·카메라보다 이 파지를 우선한다. 독립 초기화 모델의 안정된 마지막 상태에서 접촉 높이 보정 후 30초 목표를 독립 재생했다. 30,000개 1ms 검사에서 최대 이동 {summary['max_shift_mm']:.3f}mm·자세 변화 {summary['max_angle_deg']:.3f}도, 앞뒤 하부 접촉 {100*summary['opposed_contact_fraction']:.2f}%, 증명된 윗단 최소 여유 {summary['minimum_certified_upper_space_mm']:.6f}mm였다. 손 위/손바닥 받침은 지지에 포함하고 약지는 필수가 아니다.

현재 작업 파지는 중력 기준 약 {summary['source_gravity_tilt_deg']:.2f}도 기울어져 있어 예전 수평5도 과제 실패는 보존했다. 정확한 원통 지지 평면의 여유와 기존 슬랩 반지름 검사값을 혼동하지 않는다. 완전한 모델 관측을 쓰는 오프라인 보정이며 새 SAC 학습·실물 정책 제어가 아니다. 원본170해시·구동 제한·기존 중단 기준 보존, 장치 접근0.

미완료는 실제 시작/회전 끝에서 새 파지로의 진입, 다음90도 회전과 유지의 연결, 배치·마찰·구동 오차 및 실물 확인이다. 작은 다음 회전은 유지 마지막 상태에서 양 방향 진단했지만 아직 통과한 연속 경로가 없다. 미세 모델 겹침을 큰 실물 충돌이나 낙하로 단정하지 않는다. 새 실물.cmd 준비 완료로 안내하지 않는다. 최신 결과와 분기별 중단은 `right_hand_rotation_20261117/opposed_lower_grip/README.md`, `STATUS.json`을 읽고, 옛 Nov16 실패를 현재 유일 결과로 되돌리지 않는다.

마지막 회피 수정은5도 목표에4.906도(자세오차0.241도)까지 도달하고 기존 동적 기준400목표를 완료했다. 고정 위치 유지 진입 실패 이유는 중심4.591mm 변화다. 도달 자세 고정 검사에서는 첫1ms에 윗단 공간 검사 실패(분리 평면 하한-0.944mm이며 실제 침투 깊이는 아님)·앞뒤 손끝 하부 지지 각0.2N 초과 기준 미달이었다. 따라서 이제 핵심은 작은 회전 자체보다 회전 뒤 작업 파지 복구다. 손바닥 지지만으로 앞뒤 작업 파지 완료라고 하지 않는다.
<!-- /opposed_lower_grip_verified_20261117 -->

'''
    for target in [ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',ROOT/'professor_progress_20261015.md']:
        old=target.read_text(encoding='utf-8-sig')
        start='<!-- opposed_lower_grip_verified_20261117 -->'
        end='<!-- /opposed_lower_grip_verified_20261117 -->'
        if start in old:
            left=old.index(start);right=old.index(end,left)+len(end)
            old=old[:left]+old[right:].lstrip('\r\n')
        target.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(saved=str(BASE/'README.md'),frozen=len(protected),hold_passed=True,
                         turn_branches=len(turns),cycle_passed=False),ensure_ascii=False))


if __name__=='__main__': main()
