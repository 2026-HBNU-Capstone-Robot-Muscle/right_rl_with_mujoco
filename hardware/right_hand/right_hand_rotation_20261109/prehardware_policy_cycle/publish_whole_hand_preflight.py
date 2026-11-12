"""Publish local whole-hand preparation evidence; never certify unrun physics."""
from pathlib import Path
import html
import json
import re
import hashlib

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
BASE=OUT/'next_cycle/handoff_refinement_20261110'
FOLDER=BASE/'whole_hand_preflight_v2'
BEGIN='<!-- whole_hand_preflight_20261110 -->'
END='<!-- whole_hand_preflight_20261110_end -->'


def read(path):return json.loads(path.read_text(encoding='utf-8'))


def insert(path,body,after=None):
    source=path.read_text(encoding='utf-8')
    if BEGIN in source:
        start=source.index(BEGIN);end=source.index(END,start)+len(END)
        source=source[:start]+source[end:].lstrip('\n')
    block=BEGIN+'\n'+body.rstrip()+'\n'+END+'\n\n'
    if after:
        position=source.index(after)+len(after)
        source=source[:position]+'\n\n'+block+source[position:].lstrip('\n')
    else:source=block+source
    path.write_text(source,encoding='utf-8')


def banner(path,relative):
    source=path.read_text(encoding='utf-8')
    if BEGIN in source:
        start=source.index(BEGIN);end=source.index(END,start)+len(END)
        source=source[:start]+source[end:]
    body=re.search(r'<body\b[^>]*>',source,re.I)
    content=re.search(r'<(?:h[1-6]|main|section)\b',source,re.I) if body is None else None
    if body is None and content is None:raise RuntimeError('Report lacks body content')
    position=body.end() if body is not None else content.start()
    panel=(BEGIN+'<aside style="margin:16px;padding:16px;border:2px solid #32766b;background:#edf8f5;color:#153b34">'
           '<strong>2026-11-10 후속: 손 전체의 지지를 인정하는 모델 평가를 준비했습니다.</strong> '
           '접촉 계산14검사와 저장 접촉력4상태를 확인했습니다. 새 MuJoCo 계산은 승인 검토의 사용량 제한으로 미실행이며 전체 실물 준비 완료가 아닙니다. '
           '<a href="'+html.escape(relative,quote=True)+'">최신 준비 상태와 남은 검사</a></aside>'+END)
    source=source[:position]+panel+source[position:]
    path.write_text(source,encoding='utf-8')


def main():
    report=read(FOLDER/'review.json');queue=read(FOLDER/'queue_check.json')
    if any(task['result_exists'] for task in queue['tasks']):
        raise RuntimeError('Publish new native evidence separately after execution; this is the preparation report')
    unit=read(OUT/'whole_hand_support_unit_verification.json')
    if unit['status']!='passed':raise RuntimeError('Do not publish failed unit checks as passed')
    readiness_path=OUT/'next_cycle/readiness.json';readiness=read(readiness_path)
    readiness['whole_hand_support_preparation']=dict(
        path=(FOLDER/'review.json').relative_to(OUT).as_posix(),
        sha256=hashlib.sha256((FOLDER/'review.json').read_bytes()).hexdigest(),
        analytical_tests=unit['tests_run'],cached_force_states=4,
        native_evaluation_pending=True,exact_recorded_ring_comparison_pending=True,
        whole_cycle_ready_for_hardware=False,hardware_execution_allowed=False)
    readiness_path.write_text(json.dumps(readiness,ensure_ascii=False,indent=2),encoding='utf-8')
    stopped=[row for row in report['archived_trial_review'] if 'stored_endpoint_kinematics' in row]
    table=[]
    names={4118:'기존 정책',4243:'짧은 창 최종 보정',4267:'짧은 창 중간 보정',
           4233:'초기 개입 보정',4313:'긴 창 보정'}
    for row in stopped:
        state=row['stored_endpoint_kinematics']
        table.append('<tr><td>'+html.escape(names.get(row['steps'],str(row['steps'])))+'</td><td>'+str(row['steps'])+
            '</td><td>'+f"{-state['upward_mm']:.2f}"+'</td><td>'+f"{state['lateral_mm']:.2f}"+
            '</td><td>'+f"{-state['upward_velocity_m_s']:.3f}"+'</td></tr>')
    text='''# 실물 전 준비 후속: 손 전체 지지 평가 — 2026-11-10

현재는 **평가 코드와 재시험 입력 준비 완료, 새 MuJoCo 비교 미실행** 상태다. 전체 회전→유지→다음 회전→유지의 실물 준비 완료가 아니다.

손바닥, 검지 뿌리, 약지 등 손에 속하는 모든 부위의 큐브 접촉력을 집계한다. 힘의 방향은 모델 중력 반대 방향으로 계산하고, 테이블 등 외부 물체에서 받는 하중은 손 지지로 세지 않는다. 접촉 이름이 아니라 손바닥 몸체와 그 자손의 body ID로 구분한다. 접촉점이나 형상을 여러 개로 나눈 탓에 같은 총 지지력이 탈락하지 않도록 했다.

새 `whole_hand_v1` 연구 평가에서는 손 전체의 위쪽 합력이0.2N을 넘는 표본을 지지 표본으로 센다. 이 수치는 기존0.2N 힘 척도를 사용한 연구 판정이며 실제 손의 허용 하중이나450mA의 토크 환산값이 아니다. 지지 표본은 최종4.05초의4050개 상태 중80%를 넘어야 하고 외부 하중이 없어야 한다. 기존 방향5도·위치2mm·유지 중 방향 변동3도/위치 변동2mm·윗단 공간0.3mm 초과와 원래 동적 중단 조건을 함께 요구한다. 단순히 손에 닿거나 순간적으로 받친 것만으로 성공하지 않는다. 다른 손의 실제 윗단 회전 하중에 대한 내력은 별도다.

원래 손끝 두 곳의 접촉 조건은 비교 지표로 보존했고 새 모드는 별도 결과 폴더에 저장한다. 원래 모델과170개 실행 결합 파일 및 실물450mA/PID/2틱/75ms 조건을 바꾸지 않았다.

확인 완료:

- 접촉 소유권, 바닥 제외, 손바닥 단독 지지, 약지/뿌리 지지, 분산 하중, 중력 축, 힘 프레임/부호와4050표본/80% 경계의 분석적 검사14개 통과. 새 자유 동역학 시험이 아니다.
- 저장된 모델 접촉력4상태를 새 집계로 다시 계산해 기존 전체 손의 힘 벡터 및 중력 반대 방향 합력과 일치했다. 기존10회 모델의 단일 상태이며 현재50회 모델의 연속 유지 성공으로 이전하지 않는다. 이 캐시에는 외부 접촉 전체가 없어 새 파지 성공 판정에 쓰지 않는다.
- 기존 모델 시험15개의 판정을 보존했다. 손끝 조건만 미달한 실패는0개라서 그 조건의 수정만으로 통과로 바뀌는 기존 실패가 없다.
- 최근50회 모델 실패5개의 저장 상태에서 초기 대비 중력 방향 하강은 약49.4~50.0mm, 옆 방향 이동은 약62.6~63.3mm, 아래 방향 속도는 약0.853~0.862m/s였다. 안정적으로 얹혀 있는 상태와 구분한다. 이 계산만으로 손 밖 완전 이탈이나 모든 손 접촉 소실을 확정하지는 않는다. 실물 낙하 기록이 아니다.

남은 실행은 다음 순서로 묶었다.

1. 사용자가 실제 만든 약지 네 값의 정확한 자세와 기존 범위 안으로 자른 자세를 저장 모델 자세3곳에서 비교한다. 정적 형상 검사이며 실물 지지력 인증이 아니다.
2. 저장된50회 모델 실패 상태의 전체 손·외부 접촉을 다시 계산하고, 힘의 좌표계/부호를 일반화 구속력과 대조한다.
3. 손/큐브 상태를 중간에 초기화하지 않고 첫 회전부터 새 전체 손 지지 평가로 실행한다. 중단 시 접촉력과 위치 이동 성분도 저장한다.
4. 위 결과를 바탕으로 필요한 보정을 하고, 파지 유지에서 다음90도와 재유지까지 연결 검증한 후 단계별 실물 시험 후보를 판정한다.

`run_whole_hand_native_preflight.py --check`의 입력 검사는 실행했고 위 세 native 작업은 아직 실행하지 않았다. 이 스크립트는 WSL을 자동 실행하거나 설치·권한 변경을 하지 않는다. 이미 승인된 MuJoCo 환경에서 실행할 오프라인 검사만 묶는다. 직접 장치 출력을 하는 코드가 아니다.

모델 실행 차단 사유: 앞선 WSL 비교 요청의 자동 승인 검토가 사용량 제한으로 완료되지 않아 실행되지 않았다. 위험 판정이 아니며 승인 절차를 우회하지 않았다. 기록한 약지 네 값을 다시 읽거나 사용자의 연결·촬영·반복 시연을 요구하지 않는다.

근거: [저장 자료 재검토](review.json), [검사 순서와 입력 해시](queue_check.json), [분석적 검사](../../../whole_hand_support_unit_verification.json).
'''
    (FOLDER/'STATUS.md').write_text(text,encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>손 전체 지지: 실물 전 준비</title>
<style>body{font:17px/1.7 system-ui,sans-serif;max-width:1040px;margin:40px auto;padding:0 20px;color:#16352f;background:#f6faf8}section{background:white;border:1px solid #c9dcd5;padding:20px;margin:20px 0;border-radius:8px}table{border-collapse:collapse;width:100%;font-size:15px}td,th{padding:8px;border-bottom:1px solid #d1dfda;text-align:right}td:first-child,th:first-child{text-align:left}a{color:#176e64}.state{font-size:20px;font-weight:650}</style><body>
<h1>손 전체 지지를 포함한 실물 전 준비</h1>
<p class="state">평가 코드·입력 준비 완료 / 새 모델 실행 미완료</p>
<p>전체 회전 → 유지 → 다음 회전 → 유지의 실물 준비 완료가 아닙니다.</p>
<section><h2>반영한 지지 방식</h2><p>손바닥과 손가락 위에서 받치는 지지를 인정합니다. 손 전체 접촉의 실제 위쪽 합력을 집계하며 미끄러짐·자세·윗단 공간과 유지 시간 조건을 함께 확인합니다. 외부 바닥 하중은 제외합니다.</p><p>분석적 접촉 검사14개와 저장 접촉력4상태의 집계 비교를 확인했습니다. 새 MuJoCo 자유 동역학 성공 횟수는0회입니다.</p></section>
<section><h2>기존 실패 상태에서 확인한 움직임</h2><p>시뮬레이션 큐브 중심의 초기 위치 대비 이동입니다. 손끝 조건만 바꿔 통과로 바뀌는 기존 실패는 없었습니다. 완전한 손 접촉 소실을 확인한 표는 아닙니다.</p><table><thead><tr><th>정책</th><th>목표 수</th><th>아래 이동 mm</th><th>옆 이동 mm</th><th>아래 속도 m/s</th></tr></thead><tbody>'''+''.join(table)+'''</tbody></table></section>
<section><h2>다음 모델 검사</h2><ol><li>기록 약지 네 값 그대로의 자세와 잘린 자세 비교</li><li>저장 실패 상태의 전체 손·외부 접촉 및 힘 방향 대조</li><li>첫 회전부터 초기화 없는 자유 동역학 재평가</li><li>보정 후 다음 회전과 재유지 연결 검증</li></ol><p>자동 승인 검토의 사용량 제한으로 native 실행은 미실행입니다. 준비된 검사 입력 확인만 통과했습니다. 실물 재연결은 지금 필요하지 않습니다.</p></section>
<p><a href="STATUS.md">상세 조건</a> · <a href="review.json">저장 자료 분석</a> · <a href="queue_check.json">검사 입력</a> · <a href="../index.html">기존 검토</a></p></body></html>'''
    (FOLDER/'index.html').write_text(page,encoding='utf-8')
    latest='''2026-11-10 최신 후속: **손바닥·검지 뿌리·약지 등 전체 손의 지지를 평가하는 별도 연구 모드를 구현했다.** 손 전체의 중력 반대 방향 합력과 외부 하중 여부를 기록하고 기존 자세·유지 변동·윗단 공간·동적 중단 조건을 함께 요구한다. 원래 손끝 두 곳 조건은 진단으로 보존하고 과거 결과를 재분류하지 않았다. 분석적 검사14개와 저장 접촉력4상태의 집계 비교를 통과했다. 기존 모델 시험15개 중 손끝 조건만 미달한 실패는0개다. 최근50회 모델 실패5개의 저장 끝 상태는 약50mm 하강과 아래 속도 약0.85m/s를 보여 안정적으로 얹힌 상태와 구분된다. 실물 낙하나 완전한 손 접촉 소실을 확정한 결과는 아니다.

정확한 약지 네 값 비교 → 저장 상태의 전체 접촉 검산 → 새 전체 손 평가로 초기 배치부터 첫 회전 재시험을 오프라인 실행기로 묶고 입력 확인까지 완료했다. **새 native 비교·첫 회전 성공·다음 회전 검증은 이번에 실행하지 않았다.** 앞선 WSL 요청의 자동 승인 검토 사용량 제한이 미해결이며 위험 판정은 아니다. 실물·원격·UI 출력은 없고 원래170개 결합 파일과450mA/PID/2틱/75ms 조건을 보존한다. 사용자 재연결·촬영·반복 실험은 지금 필요하지 않다.

최신 근거: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/whole_hand_preflight_v2/STATUS.md`, `index.html`, `review.json`, `queue_check.json`. 아래 기록 자세/손끝 검사 점검은 이 보완 전 기록으로 구분한다.'''
    insert(ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',latest,after='## 지금 먼저 할 일')
    insert(ROOT/'professor_progress_20261015.md',latest)
    insert(BASE/'STATUS.md','최신 후속: 전체 손 지지 평가 코드와14개 분석적 검사를 완료했다. 새 native 계산은 승인 검토 사용량 제한으로 미실행이다. [최신 준비 상태](whole_hand_preflight_v2/index.html)를 먼저 본다.',after='# 약지 하중 인계 후속: 2026-11-10')
    banner(BASE/'index.html','whole_hand_preflight_v2/index.html')
    banner(OUT/'review/index.html','../next_cycle/handoff_refinement_20261110/whole_hand_preflight_v2/index.html')
    banner(ROOT/'right_hand_rotation_20261109/policy_audit/explanation/index.html','../../prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/whole_hand_preflight_v2/index.html')
    print(json.dumps(dict(published=str(FOLDER.relative_to(ROOT)),analytical_tests=unit['tests_run'],
                         native_evaluations_executed=False,hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
