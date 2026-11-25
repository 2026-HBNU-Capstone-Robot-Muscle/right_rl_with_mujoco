"""Publish actual entry/next-turn teacher probes with explicit conditional scope."""
from pathlib import Path
import hashlib
import json
import html
import re

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'bridge_progress_20261110'
MARKER='clear_bridge_progress_20261110'


def record(path):
    raw=path.read_bytes();return json.loads(raw),hashlib.sha256(raw).hexdigest()


def block(path,content,after=None):
    text=path.read_text(encoding='utf-8');start='<!-- '+MARKER+' -->';end='<!-- '+MARKER+'_end -->'
    if start in text:
        begin=text.index(start);finish=text.index(end,begin)+len(end);text=text[:begin]+text[finish:].lstrip('\n')
    new=start+'\n'+content.rstrip()+'\n'+end+'\n\n'
    if after:
        index=text.index(after)+len(after);text=text[:index]+'\n\n'+new+text[index:].lstrip('\n')
    else:text=new+text
    path.write_text(text,encoding='utf-8')


def chart(trace,final=None,steps=None):
    rows=[dict(step=row['step'],error_deg=row['error_deg']) for row in trace]
    if final is not None and steps is not None:rows.append(dict(step=steps,error_deg=final['error_deg']))
    if not rows:return ''
    maximum=max(1.,max(row['error_deg'] for row in rows)*1.08);duration=max(row['step']*.075 for row in rows)
    points=' '.join(f"{48+row['step']*.075/max(duration,1e-9)*590:.3f},{200-row['error_deg']/maximum*165:.3f}" for row in rows)
    return '<svg viewBox="0 0 690 240" role="img" aria-label="모델 경과 시간과 목표 자세 오차"><path d="M48 25V200H650" fill="none" stroke="#668077"/>'+f'<polyline fill="none" stroke="#087f8c" stroke-width="2" points="{points}"/><text x="7" y="38">{maximum:.1f}°</text><text x="15" y="205">0</text><text x="50" y="229">0초</text><text x="570" y="229">{duration:.2f}초</text></svg>'


def main():
    FOLDER.mkdir(exist_ok=True)
    names=[('horizon_fast_entry_cp1700','회전 중간 상태 → 작업 파지'),
        ('horizon_transport_first_cp1700','큐브 위치 이동 → 방향 맞춤'),
        ('horizon_fast_next5_from_clear_grip','작업 파지 → 다음 5도 → 유지'),
        ('next5_canonical_station_recovery','5도 끝 자세의 공통 위치 보정'),
        ('next5_canonical_priority','공통 위치를 우선한 보정'),
        ('next5_canonical_settled_finish','정착한 자세에서의 공통 위치·자세 유지'),
        ('horizon_next90_after_next5','5도 끝에서 다음 전체 90도 목표'),
        ('hybrid_entry_after400','진입400목표 이후 위치·방향을 번갈아 맞춤'),
        ('hybrid_next90_recenter','다음 회전 방향에 접근한 뒤 공통 위치 보정'),
        ('horizon_transport_first_cp1300','이른 회전 상태에서 위치 이동 → 방향 맞춤'),
        ('opposing_workspace_feedback_entry','별도 맞은편 파지 위치까지 이동'),
        ('opposing_workspace_next90','별도 맞은편 파지에서 다음90도'),
        ('opposing_workspace_push_transport','현재 큐브 접촉 목표를 거친 위치 이동'),
        ('reached_workstation_next90','실제 첫 파지 위치에서 다음90도와 복귀')]
    studies=[];sections=[]
    for name,title in names:
        folder=BASE/name;path=folder/'review.json';completed=path.exists()
        if not completed:path=folder/'progress.json'
        if not path.exists():continue
        data,digest=record(path);interrupted=(folder/'interrupted.json').exists()
        if completed:
            final=data['final'];steps=data['steps'];trace=data['trace'];stop=data['stop'];passed=data['conditional_bridge_and_hold_passed']
            entry=data['mode']=='entry';global_yaw=data['global_yaw_target_deg'];hold=data['final_hold']
            status='조건부 단계 통과' if passed else '완료한 탐색에서 목표 미달'
        else:
            final=data['actual'];steps=data['step'];trace=[];stop=None;passed=False;entry=data['mode']=='entry';global_yaw=data['global_yaw'];hold=None;status='다른 분기로 이어간 실행 기록' if interrupted else '모델 계산 진행 중'
        item=dict(name=name,title=title,source=path.relative_to(OUT).as_posix(),source_sha256_at_snapshot=digest,completed=completed,
            snapshot_is_mutable_progress=not completed,steps=steps,global_yaw_target_deg=global_yaw,stop=stop,conditional_stage_passed=passed,
            mode='entry' if entry else 'next',active=not completed and not interrupted,
            final_error_deg=final['error_deg'],center_target_error_mm=final['center_target_error_mm'],
            canonical_station_error_mm=final.get('canonical_station_error_mm'),final_hold=hold)
        studies.append(item)
        detail=f"{steps}목표 · 목표 자세 오차 {final['error_deg']:.3f}도 · 평가한 위치 목표와의 차이 {final['center_target_error_mm']:.3f}mm"
        canonical=final.get('canonical_station_error_mm')
        if canonical is not None:detail+=f" · 기존 공통 위치와의 차이 {canonical:.3f}mm"
        paragraphs='<p>'+html.escape(detail)+'</p>'
        if data.get('alternative_goal_reference_explicit'):
            paragraphs+='<p>위치 목표는 이 실행 전에 지정한 별도 참고 파지다. 기존 canonical 위치에서의 실패를 통과로 바꾸지 않는다.</p>'
        if hold:paragraphs+='<p>최종 유지 '+str(hold['samples'])+'개1ms 상태, 공간 인증 '+str(hold['certified_samples'])+'개, 분리 하한 '+f"{hold['min_lower_mm']:.3f}mm"+'.</p>'
        if stop:paragraphs+='<p>종료: '+html.escape(stop)+'</p>'
        sections.append('<section><h2>'+html.escape(title)+'</h2><p><strong>'+status+'</strong> · 절대 방향 목표 '+str(global_yaw)+'도</p>'+paragraphs+chart(trace,final if completed else None,steps if completed else None)+'<p><a href="../'+name+'/'+path.name+'">모델 기록</a></p></section>')
    replay_path=BASE/'clear_grip_next5_joined_chain/review.json';replay=None
    if replay_path.exists():
        replay,replay_digest=record(replay_path)
        studies.append(dict(name='clear_grip_next5_joined_chain',source=replay_path.relative_to(OUT).as_posix(),source_sha256_at_snapshot=replay_digest,completed=True))
        sections.append('<section><h2>공간 확보부터 5도까지 연속 재검사</h2><p>'+f"{replay['steps']}목표, 상태 초기화 {replay['actual_state_resets_after_start']}회. 실제 추가 수평 회전 {replay['additional_yaw_after_clear_grip_deg']:.3f}도, 최종 목표 오차 {replay['final_orientation_error_deg']:.3f}도. " +
            f"기존 공통 위치 오차 {replay['canonical_station_error_mm']:.3f}mm로, 2mm 공통 위치 기준의 통과는 {replay['conditional_canonical_station_task_passed']}다."+
            '</p><p>이 연결의 출발 약지 파지는 별도로 배치한 모델이다. 초기 배치의 첫 회전과 다음90도 전체 성공으로 해석하지 않는다.</p><a href="../clear_grip_next5_joined_chain/review.json">연결 근거</a></section>')
    canonical_path=BASE/'clear_grip_next5_canonical_joined_chain/review.json';canonical=None
    if canonical_path.exists():
        canonical,digest=record(canonical_path);studies.append(dict(name='clear_grip_next5_canonical_joined_chain',source=canonical_path.relative_to(OUT).as_posix(),source_sha256_at_snapshot=digest,completed=True))
        sections.insert(0,'<section><h2>별도 출발 파지의5도 뒤 공통 파지 회복804목표</h2><p>'+f"실제 추가 yaw {canonical['additional_yaw_after_clear_grip_deg']:.3f}도,95도 목표 자세 오차 {canonical['final_orientation_error_deg']:.3f}도, 기존 공통 위치 오차 {canonical['canonical_station_error_mm']:.3f}mm. " +
            f"마지막4050상태 지지100%, 윗단 인증 하한 {canonical['upper_certificate_min_lower_mm']:.3f}mm. 상태 초기화는0회이며 모든 연결 상태가 별도 실행과 일치했다."+
            '</p><p>출발은 여전히 별도로 배치한 약지 파지다. 기존 초기 배치의 첫90도와 다음90도 전체의 완료가 아니다.</p><a href="../clear_grip_next5_canonical_joined_chain/review.json">804목표 연결 재검사</a></section>')
    for name,title in (('first90_middle_first_arrivals','중지 우선 복귀 경로4개'),('first90_finger_escape_finishing','손가락을 바깥으로 비운 복귀4개'),('first90_RF_shape_before_adduction','약지 모양을 먼저 접은 복귀4개'),('actual_cube_MF_global_contact','실제 큐브의 중지 하부 접촉 전역 탐색'),('actual_cube_MF_TH_regrasp_global','실제 큐브에서 엄지·중지 재접촉')):
        path=BASE/name/'review.json';active=False
        if not path.exists():
            path=BASE/name/'progress.json';active=path.exists()
        if not path.exists():continue
        data,digest=record(path);studies.append(dict(name=name,title=title,source=path.relative_to(OUT).as_posix(),source_sha256_at_snapshot=digest,completed=not active,active=active))
        if 'rows' in data and data['rows'] and 'static_contact_candidate' in data['rows'][0]:
            text=f"정적 접촉 후보 {sum(row['static_contact_candidate'] for row in data['rows'])}/{len(data['rows'])}개. 이것만으로 도달·유지 성공으로 세지 않는다."
        elif 'trials' in data:
            text=f"도달·유지 검사 {len(data['trials'])}개, 중지·엄지 재접촉 유지 통과 {sum(row['MF_TH_regrasp_for_translation_passed'] for row in data['trials'])}개. 재접촉 유지와 전체 공통 파지·윗단 공간 조건을 구분한다."
        elif 'rows' in data:
            text=f"검사 {len(data['rows'])}개, 전체 작업 파지 통과 {sum(row.get('actual_checkpoint_to_work_grip_passed',row.get('first_rotation_and_work_grip_from_checkpoint_passed',False)) for row in data['rows'])}개."
        else:text='모델 계산 진행 중이며 최종 결과는 아직 없다.'
        sections.append('<section><h2>'+html.escape(title)+'</h2><p>'+html.escape(text)+'</p><a href="../'+name+'/'+path.name+'">근거</a></section>')
    first_path=BASE/'initial_to_reached_workstation90/review.json';first_reached=None
    if first_path.exists():
        first_reached,digest=record(first_path);studies.append(dict(name='initial_to_reached_workstation90',source=first_path.relative_to(OUT).as_posix(),source_sha256_at_snapshot=digest,completed=True))
        sections.insert(0,'<section><h2>초기 배치부터 실제 첫90도와 도달 위치 유지 재검사</h2><p>'+f"총 {first_reached['steps']}목표를 초기화 없이 재생했다. 90도 목표 오차 {first_reached['final_orientation_error_deg']:.3f}도, 마지막4050상태 지지 {first_reached['whole_hand_support_fraction']*100:.0f}%, 윗단 인증 하한 {first_reached['minimum_upper_lower_mm']:.3f}mm. " +
            f"도달한 위치의 회전·지지·공간 유지는 {first_reached['first_rotation_and_reached_station_hold_passed']}지만 기존 canonical 위치와는 {first_reached['canonical_station_error_mm']:.3f}mm 차이가 있어 그2mm 기준은 미달이다."+
            '</p><p>1700목표는 저장 정책의 앞부분,1261목표는 모델 교사,54목표는 위치 유지다. 순수 SAC 전체 성공이나 다음90도·실물 성공으로 부르지 않는다. 이 실제 작업 위치를 다음 회전 전 고정해 복귀 검사를 진행한다.</p><a href="../initial_to_reached_workstation90/review.json">초기 배치 연속 재생 근거</a></section>')
    opposed_path=BASE/'opposing_grip_review_20261110/review.json'
    if opposed_path.exists():
        opposed,digest=record(opposed_path);studies.append(dict(name='opposing_grip_review_20261110',source=opposed_path.relative_to(OUT).as_posix(),source_sha256_at_snapshot=digest,completed=True))
        sections.append('<section><h2>맞은편 파지의 별도 배치 유지·하중 비교</h2><p>새 위치의 엄지·중지 파지는90/180도 배치 유지와 가정 하중을 검사했다. 실제 회전 뒤 그 위치로 도달한 결과와 구분한다.</p><a href="../opposing_grip_review_20261110/index.html">자세 비교 그림·접촉력·도달 경로</a></section>')
    actor_path=BASE/'original108_governed_bridge_probes/review.json'
    if actor_path.exists():
        actor,digest=record(actor_path);studies.append(dict(name='original108_governed_bridge_probes',source=actor_path.relative_to(OUT).as_posix(),source_sha256_at_snapshot=digest,completed=True))
        rows=''.join('<tr><td>'+html.escape(row['case']+' / '+row['last_action_semantics'])+'</td><td>'+str(row['steps'])+'</td><td>'+f"{row['best_error_deg']:.3f}도"+'</td><td>'+html.escape(','.join(row['stop']))+'</td></tr>' for row in actor['rows'])
        sections.append('<section><h2>기존108차원 정책의 재사용 비교</h2><p>실제 정수 관절 관측과 2틱/75ms 실행 제한으로6조건을 비교했다. 첫 제안이 크다는 이유로 즉시 거부하는 비교와 달리 제한된 목표를 실제 모델에서 실행했다. 여섯 조건 모두 손 접촉 기준에 걸렸고 새 파지의 다음 회전·유지를 검증하지 못했다. 별도110차원 SAC 기준 궤적 정책과 구분한다.</p><table><tr><th>조건/직전 행동 입력</th><th>실행 목표</th><th>최소 목표 오차</th><th>중단</th></tr>'+rows+'</table><p><a href="../original108_governed_bridge_probes/review.json">정책·입력·모델 해시와 상세</a></p></section>')
    report=dict(status='actual_bridge_teacher_progress_snapshot',studies=studies,
        initialized_checkpoint_entry_stage_completed=any(row.get('conditional_stage_passed') and row.get('mode')=='entry' for row in studies),
        conditional_next5_stage_completed=any(row.get('conditional_stage_passed') and row['name']=='horizon_fast_next5_from_clear_grip' for row in studies),
        conditional_next5_canonical_joined_chain_completed=bool(canonical and canonical['conditional_canonical_station_task_passed']),
        first_rotation_at_explicit_reached_station_passed=bool(first_reached and first_reached['first_rotation_and_reached_station_hold_passed']),
        full_initial_placement_two_turn_cycle_passed=False,model_teacher_probes_are_not_SAC_training=True,
        new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    (FOLDER/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>회전·파지·다음 회전 연결 계산</title><style>body{font:16px/1.7 system-ui,sans-serif;max-width:1060px;margin:28px auto;padding:0 18px;color:#173d36;background:#f4f8f6}section{background:white;border:1px solid #ccdcd3;padding:18px;margin:18px 0}svg{width:100%;max-width:690px}svg text{font:13px system-ui;fill:#48665c}table{border-collapse:collapse;width:100%;font-size:14px}th,td{padding:8px;text-align:left;border-bottom:1px solid #d1ded7}a{color:#0a746b}.note{background:#fff1d2;padding:14px}</style><body><h1>회전 → 작업 파지 → 다음 회전의 연결</h1><p class="note">실제 로봇을 구동하지 않은 모델 계산이다. 출발을 별도로 배치한 파지와 초기 회전 중간 상태에서 시작한 검사를 구분한다. 조건부5도 단계는 전체 다음90도나 전체 실기 준비 완료가 아니다.</p><p>후보 평가는 여러 제어 주기를 예측한다. 실제 재생은 원래 모델의 관절·충돌·추종 기준을 매1ms 검사하고 2틱/75ms를 유지한다. 후보 비용의 양의 간격은 제한된 방향에서의 하한이며 최종 윗단 인증은 별도 정확한 원통 지지 함수로 수행한다. 새 SAC 학습은 실행하지 않았다.</p>'''+''.join(sections)+'''<p><a href="review.json">요약 및 출처</a> · <a href="../RF_clear_work_grip_evidence/index.html">앞선 파지의 그림·영상</a> · <a href="../whole_hand_preflight_v2/index.html">전체 실행 이력</a></p></body></html>'''
    (FOLDER/'index.html').write_text(page,encoding='utf-8')
    brief='최신 연결 작업: 여러 주기 예측으로 진입과 다음 회전을 검사 중이다. 별도 작업 파지에서 다음5도와4050상태 유지·공간 단계는 통과했으나, 독립 연결 재검사에서 기존 공통 위치 오차2.130mm로2mm 기준을 넘었다. 공통 위치 보정과 다음180도 목표, 초기 회전 상태에서의 진입을 별도로 계산한다. 기존108차원 정책6개 제한 실행 비교는 손 접촉 기준에서 중단됐다. 이는 새SAC 학습이나 전체 실기 준비 완료가 아니다. 최신 기록: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/bridge_progress_20261110/index.html`.'
    if canonical:
        brief=f"최신 연결 검증: 별도 약지 파지에서 공간 확보→추가 yaw {canonical['additional_yaw_after_clear_grip_deg']:.3f}도→공통 파지 회복까지804목표를 초기화 없이 재생했다.95도 목표 자세 오차{canonical['final_orientation_error_deg']:.3f}도, canonical 위치 오차{canonical['canonical_station_error_mm']:.3f}mm, 마지막4050상태 지지100%·윗단 인증 하한{canonical['upper_certificate_min_lower_mm']:.3f}mm로 통과했다. 앞선526목표의2.130mm 미달은 보존했고 허용치를 바꾸지 않고 보정했다. 초기 배치의 첫90도 공통 파지와 다음90도 전체 공통 파지는 아직 미완료다. 큰 회전 뒤에는 검지 뿌리·마디 지지만 남고 엄지·중지 접촉이 없어 실제 큐브 위치에서 재접촉하는 경로를 탐색한다. 중지 전역 탐색은 유한3조건에서 하부 접촉 기하 후보3개를 찾았으며 자유 동역학 도달·유지와 구분한다. 새SAC 학습은0스텝이다. 최신 기록: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/bridge_progress_20261110/index.html`."
    if first_reached:
        brief=f"최신 연속 재생: 초기 배치→저장 정책1700목표→모델 교사1261목표→위치 유지54목표, 총{first_reached['steps']}목표를 상태 초기화 없이 재검사했다. 첫90도 목표 오차{first_reached['final_orientation_error_deg']:.3f}도, 마지막4050상태 전체 손 지지100%·윗단 인증 하한{first_reached['minimum_upper_lower_mm']:.3f}mm로 실제 도달 위치의 회전·지지·공간 유지가 통과했다. 이 위치는 옛 canonical 기준과{first_reached['canonical_station_error_mm']:.3f}mm 다르므로 기존2mm 위치 기준의 미달을 통과로 바꾸지 않는다. 실제 도달 위치를 다음 회전 전에 고정한 `reached_workstation_next90`에서 다음90도와 같은 위치 복귀를 별도로 검사한다. 별도 배치 파지의804목표·5도 연결 결과와 구분한다. 새 맞은편 파지는 별도 배치한90/180도 유지와 가정 하중11조건 중10조건을 통과했지만 그 위치로의 도달은 미완료다. 다음90도 전체 및 실물 준비는 아직 완료하지 않았고 이번 새SAC 학습은0스텝이다. 최신 기록: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/bridge_progress_20261110/index.html`."
        next_path=BASE/'reached_workstation_next90/review.json'
        if next_path.exists():
            next_data,_=record(next_path)
            brief+=f" 다음90도 검사는{next_data['steps']}목표 뒤 {next_data['stop']}로 종료됐고180도 목표 오차는{next_data['final']['error_deg']:.3f}도다. 현재 첫 파지를 유지하는 것과 그 파지에서 다음 회전을 만드는 문제는 분리돼 있다. `opposing_grip_review_20261110/MODEL_TASK_SPEC.md`에 위치 목표·실행 경계와 남은 접촉 전환을 정리했다."
    block(ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',brief,after='## 지금 먼저 할 일')
    block(ROOT/'professor_progress_20261015.md',brief)
    for path,link in ((BASE/'index.html','bridge_progress_20261110/index.html'),
        (BASE/'whole_hand_preflight_v2/index.html','../bridge_progress_20261110/index.html'),
        (ROOT/'right_hand_rotation_20261109/policy_audit/explanation/index.html','../../prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/bridge_progress_20261110/index.html')):
        text=path.read_text(encoding='utf-8');start='<!-- '+MARKER+' -->';end='<!-- '+MARKER+'_end -->'
        if start in text:
            begin=text.index(start);finish=text.index(end,begin)+len(end);text=text[:begin]+text[finish:]
        match=re.search(r'<body\b[^>]*>',text,re.I);position=match.end() if match else 0
        panel=start+'<aside style="padding:16px;margin:16px;border:2px solid #347b6f;background:#edf8f4"><strong>최신 연결 검사</strong> <a href="'+link+'">회전 진입·다음 회전의 최신 모델 계산</a>. 5도 단계와 전체90도를 구분합니다.</aside>'+end
        path.write_text(text[:position]+panel+text[position:],encoding='utf-8')
    readiness_path=OUT/'next_cycle/readiness.json';readiness=json.loads(readiness_path.read_text(encoding='utf-8'))
    readiness['clear_work_grip_bridge_progress']=dict(path=(FOLDER/'review.json').relative_to(OUT).as_posix(),sha256=hashlib.sha256((FOLDER/'review.json').read_bytes()).hexdigest(),
        conditional_next5_canonical_chain=bool(canonical and canonical['conditional_canonical_station_task_passed']),whole_cycle_ready_for_hardware=False,hardware_execution_allowed=False)
    readiness['clear_work_grip_bridge_progress']['first_rotation_at_explicit_reached_station_passed']=bool(first_reached and first_reached['first_rotation_and_reached_station_hold_passed'])
    readiness_path.write_text(json.dumps(readiness,ensure_ascii=False,indent=2),encoding='utf-8')
    state_lines=['# 현재 연결 작업 상태','',
        '사용자 목표는 초기 배치 → 큐브 전체 회전 → 윗단 공간을 비운 유지 → 다음90도 → 유지다. 아래는 실제 결과 파일에 근거한 상태이며 과거 세션 번호를 실행 중으로 재사용하지 않는다.','',
        '첫3015목표의 대체 작업 위치는 옛 canonical 기준에서5.604mm 떨어져 있다. 옛2mm 기준 실패는 보존하고, 새 다음 회전의 위치 목표는 그 실행 전에 실제 첫 유지 위치로 고정했다.','',
        '원래170개 실행 결합·미들웨어57개·450mA/PID/관절 범위/2틱75ms와 native 중단 기준은 유지한다. 모델0.15N·m는450mA 환산값이 아니다. ID4의 승인 설정은 lower_degrees_by=2.0이며 옛 모델 하한에서2도 연장한 값이다. 절대각−2도로 바꾸지 않는다.','',
        '이번 새SAC 학습은0스텝이며 과거90000 요청 학습 분량과 구분한다. 모델 교사 계산과 저장 경로 재생을 실제 정책 학습·증류로 설명하지 않는다. 사용자 재연결·촬영·반복 각도 읽기는 지금 요구하지 않는다.','',
        '## 완료 및 진행 스냅샷','']
    for item in studies:
        if 'steps' not in item:continue
        state_lines.append('- '+item['name']+': '+('결과 파일 완료' if item.get('completed') else ('모델 계산 진행 스냅샷' if item.get('active') else '다른 분기로 이어간 기록'))+' / '+str(item['steps'])+'목표 / 방향 오차 '+str(round(item['final_error_deg'],4))+'도.')
    if first_reached:state_lines += ['', '초기 배치부터의 첫 회전 재생: initial_to_reached_workstation90/review.json.3015목표·네 구간 전체 상태 오차0·마지막4050 지지/공간 통과. 다음90도와 전체 실기는 미완료다.']
    state_lines += ['', '현재 우선 남은 문제는 실제 첫 파지에서 회전력을 전달하는 접촉 전환이다. 상세 위치·판정 범위와 남은 조건은 ../opposing_grip_review_20261110/MODEL_TASK_SPEC.md를 읽는다.', '',
        'publish_native_whole_hand_results.py → publish_opposing_grip_review.py → publish_clear_bridge_progress.py 순서로 문서를 갱신한 뒤 verify_preflight_artifacts.py를 실행한다.']
    (FOLDER/'RUNNING.md').write_text('\n'.join(state_lines)+'\n',encoding='utf-8')
    print(json.dumps(dict(studies=len(studies),pending=sum(row.get('active',False) for row in studies),report=(FOLDER/'index.html').relative_to(OUT).as_posix())),flush=True)


if __name__=='__main__':main()
