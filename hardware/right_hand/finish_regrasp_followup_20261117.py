"""Preserve the latest regrasp evidence without claiming hardware readiness."""
from pathlib import Path
import ast,hashlib,json
ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    path=BASE/'core_support_chain_10s/review.json';report=json.loads(path.read_bytes());audit=report['audit']
    selected_path=path
    extensions={}
    for name in ('extended_hold30','active_hold30'):
        p=path.parent/name/'review.json'
        if p.exists():
            d=json.loads(p.read_bytes());extensions[name]=d
            if d['passed']:selected_path=p;audit=d['audit']
    support_passed=audit['core_support_passed'];working_passed=audit['passed']
    inspection=json.loads((BASE/'regrasp_state_inspection_extra.json').read_bytes())['core_support_chain_10s']
    mf=max((c for c in inspection['contacts'] if c['name']=='mf_tip'),key=lambda c:c['force_N'])
    edge=27.5-abs(mf['cube_local_mm'][0])
    receipt={}
    for p in ROOT.glob('*20261117.py'):
        if any(s in p.name for s in ('opposed','regrasp','convex_surface')):
            ast.parse(p.read_text(encoding='utf-8-sig'));receipt[p.name]=sha(p)
    protected=json.loads((ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']
    if any(sha(ROOT/p)!=h for p,h in protected.items()):raise RuntimeError('Protected file changed')
    cases=[]
    for glob in ('feedback_regrasp_*/review.json','regrasp_*together*/review.json','regrasp_*backmiddle/review.json','regrasp_supportfirst_*/review.json','latched_preload_*/review.json'):
        for p in BASE.glob(glob):
            d=json.loads(p.read_bytes());cases.append(dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p),
                passed=d.get('regrasp_and_hold_passed',False),stop=d.get('stop') or (d.get('audit') or {}).get('stop'),
                phase=d.get('phase'),geometry_passed=d.get('geometry_passed')))
    latest=dict(status='regrasp_followup_support_and_upper_space_separated',support_chain=report,
        selected_proof_path=selected_path.relative_to(ROOT).as_posix(),selected_proof_sha256=sha(selected_path),
        selected_audit=audit,extensions=extensions,
        final_middle_contact_edge_margin_mm=edge,
        support_chain_review_sha256=sha(path),runtime_sources_unchanged=len(protected),
        script_hashes=receipt,branch_results=cases,geometry_feature_tests=dict(analytic=5,random_box=100,passed=True),
        hardware_execution_allowed=False,devices_opened=False)
    (BASE/'REGRASP_FOLLOWUP.json').write_text(json.dumps(latest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    result='통과' if support_passed else '미통과'
    text=f'''# 회전 뒤 재파지 후속 검토 · 2026-11-17

작은5도 연구 회전 뒤 앞뒤 지지 독립 검사: **{result}**. 윗단 공간을 포함한 손 기준 작업 파지 판정: **{'통과' if working_passed else '미통과'}**.

기존 유지 → 작은5도 회전 → 앞뒤 재파지 명령을 한 번에 이어 재생했다. 손·큐브 상태를 중간에 초기화하지 않았다. 이 시작 상태는 독립 초기화한 모델 파지에서 얻은 저장 상태이며 실제 로봇 시작 자세에서의 진입은 포함하지 않는다.

| 마지막 지지 검사 | 값 |
|---|---:|
| 지지 검사 시간 | {audit['evaluated_hold_seconds']:.3f}초 |
| 최대 위치 오차 | {audit['maximum_center_error_mm']:.3f}mm |
| 최대 자세 오차 | {audit['maximum_angle_error_deg']:.3f}도 |
| 엄지·중지 하부 접촉 비율 | {100*audit['required_lower_contact_fraction']:.2f}% |
| 앞뒤 접촉 방향 확인 비율 | {100*audit['opposed_front_back_fraction']:.2f}% |
| 증명된 윗단 최소 여유 | {audit['minimum_exact_support_plane_gap_mm']:.3f}mm |
| 윗단 공간 검사 수 | {audit['certificate']['queries']} |

검사 마지막 중지 접촉점은 큐브 모서리에서 약 **{edge:.2f}mm** 떨어져 있었다. 이는 마지막 모델 자세의 접촉 위치이며 실제 사진으로 측정한 값은 아니다.

회전에서 재파지로 넘어가는 시점을 앞당겼고, 엄지·중지가 실제로 닿은 뒤 검지 비우기 단계로 넘어가도록 했다. 실제 접촉점의 누르는 힘과 손끝 이동을 분리했다. 관절 상한은 유지하며 계산용 목표 여유는2틱인 별도 연구 분기를 사용했다. 검지 가로 위치는 고정하지 않고 높이와 후퇴를 제어하는 분기도 비교했다. 새 SAC 학습이나 실물 제어가 아니다.

기존 원본170개 결합 해시와 모델·건강·관절·목표2틱/75ms·추종·충돌 기준을 유지했다. 기하 표면 계산은 상자의 면·모서리·꼭짓점5사례와 무작위100사례로 확인했다. 몸체는 중력 기준 약10도 기울어진 작업 자세이며 기존 수평5도 과제의 실패는 보존했다.

- [연속 재생 원본](core_support_chain_10s/review.json)
- [전체 분기와 소스 해시](REGRASP_FOLLOWUP.json)
- [기존30초 작업 파지 검사](level_triple_normal1p2_settle1_settled_frame_active_hold400_heightlock0p0/complete_hold_audit/review.json)
- [이번 회전 뒤 파지 그림](followup_evidence/regrasp_verified_side.png)

## 미완료 및 실패 기록

이번에 선택한 경로의 윗단 공간 판정은 **{'통과' if working_passed else '미통과'}**다. 10.05초 검사에서는 진단 모드로 시작했지만 실제로 모든 공간 표본도 조회·통과했다. 공간 실패를 무시해서 통과한 결과가 아니다. 90도 연속 회전·파지 반복, 실제 시작 자세의 진입, 배치/마찰 오차 및 실물 시험 준비는 미완료다.

마지막 목표를 고정해30초까지 늘린 비교는 총14.643초에서 공간 여유 하한0.295mm로0.3mm 기준을 밑돌았다. 이 시점 큐브가 떨어졌다는 뜻은 아니며 앞뒤 접촉 비율은99% 이상이었다. 고정 유지 비교 실패는 보존하고, 잡힌 접촉점과 자세를 보정하는 별도 유지 분기를 구분한다.

접촉점과 자세를 보정하는 유지 분기도 추가 약4초 후 공간 기준에 미달했다. 그 시점 엄지·중지 하부 지지가 각0.2N 초과 기준에 미달한 관측을 보존한다. 따라서10초 성공을30초 유지 성공으로 확대하지 않는다.

제외한 초기 분기에서는 검지를 먼저 빼거나 다른 손가락을 먼저 닫는 순서만 바꿔도 지지가 무너졌다. 후속 검지 후퇴 분기도 접촉을 잃었고, 중지를 안쪽에 먼저 자리 잡게 하는 전체 생성 분기는 해당 전환 조건에 도달하지 못했다. 다만 그 생성 경로 중 작업 파지가 이미 성립한 구간을 고정된10.05초 과제로 분리해 전체 앞 구간부터 재생했으며, 위 통과 수치는 그719목표 경로의 독립 검증이다. 작은 형상 겹침을 큰 실물 충돌로 단정하지 않는다.

다음은 모서리 가까운 중지 접촉을 더 안쪽에 자리 잡게 하는 큐브 이송·재파지를 검토하고, 유지 시간을 늘린 뒤90도와 다음 회전까지 연결하는 것이다. 새 실물 실행파일을 배포하거나 기존.cmd를 교체하지 않았다.
'''
    (BASE/'REGRASP_FOLLOWUP.md').write_text(text,encoding='utf-8')
    marker='<!-- regrasp_followup_20261117 -->'
    end='<!-- /regrasp_followup_20261117 -->'
    block=f'''{marker}
## 최신 재파지 후속 검사

기존 유지→작은5도 연구 회전→새 앞뒤 파지를 초기화 없이 연속 재생했다. 마지막 지지 {audit['evaluated_hold_seconds']:.3f}초는 {result}, 윗단 공간 포함 손 기준 작업 파지는 {'통과' if working_passed else '미통과'}다. 최대 이동오차 {audit['maximum_center_error_mm']:.3f}mm·자세오차 {audit['maximum_angle_error_deg']:.3f}도·엄지중지 하부 접촉 {100*audit['required_lower_contact_fraction']:.2f}%, 공간 하한 {audit['minimum_exact_support_plane_gap_mm']:.3f}mm다. 모든 {audit['certificate']['queries']}개 공간 표본을 조회했으며 실패 무시로 통과한 것이 아니다. 고정 목표30초 확장은14.643초에 여유0.295mm로 미달한 기록을 보존한다. 중력 기준 약10도 기울어진 작업 자세여서 예전 수평5도 과제 실패는 유지한다. 새로운90도 연속동작, 실제 시작 자세 진입, 오차 내성이나 실물 준비 완료로 안내하지 않는다. 원본170해시·실제 상한·원래 동적 기준 보존, 장치 접근0. 새 SAC 학습 없음. 최신 근거: `right_hand_rotation_20261117/opposed_lower_grip/REGRASP_FOLLOWUP.md`, `REGRASP_FOLLOWUP.json`.

접촉·자세 보정 유지도 추가 약4초에서 공간 기준과 엄지중지 지지 미달이 관측됐다. 10초 성공을30초나 전체90도 연속동작 성공으로 확대하지 않는다. 중지 접촉이 모서리에 가까운 점을 기준으로 큐브를 조금 이송한 뒤 다시 잡는 경로를 우선 검토한다. 이미 통과한 작은 회전 뒤10초 파지를 미완료로 되돌리거나 원래 관절 상한을 바꿔서 완료로 만들지 않는다.

검사 마지막 중지 접촉점은55mm 큐브 모서리에서 약{edge:.2f}mm였다. 이는 저장 모델 접촉점의 한 시점 값이다. 권장 후속은 중지 접촉을 더 안쪽에 자리 잡게 하는 이송·재파지 검토다. 전체 생성 분기의미수렴 기록과, 그중719목표를 선택해 검증한10.05초 작업 파지 성공을 구분한다.
{end}

'''
    for target in [ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',ROOT/'professor_progress_20261015.md']:
        old=target.read_text(encoding='utf-8-sig')
        if marker in old:
            i=old.index(marker);j=old.index(end,i)+len(end);old=old[:i]+old[j:].lstrip('\r\n')
        target.write_text(block+old,encoding='utf-8')
    page=BASE/'index.html';old=page.read_text(encoding='utf-8')
    note='<p><a href="REGRASP_FOLLOWUP.md">최신 회전 뒤 재파지 검토와 연속 지지 검사</a></p>'
    if note not in old:page.write_text(old.replace('<h1>',note+'<h1>',1),encoding='utf-8')
    print(json.dumps(dict(support_passed=support_passed,working_grip_passed=working_passed,
        frozen_sources=len(protected),syntax_checked=len(receipt),report=str(BASE/'REGRASP_FOLLOWUP.md')),ensure_ascii=False))


if __name__=='__main__':main()
