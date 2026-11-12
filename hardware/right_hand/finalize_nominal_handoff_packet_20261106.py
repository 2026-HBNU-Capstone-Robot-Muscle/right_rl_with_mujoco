"""Promote only the fully replayed nominal task, retaining historical failures.

Called by the packet builder. Old robustness matrices remain bound to their
old candidate; none are transferred to the new6166-goal nominal trajectory.
"""
import hashlib,html,json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    file=O/'offline_review.json';packet=json.loads(file.read_bytes());follow=json.loads((O/'offline_followup.json').read_bytes());current=follow['current_nominal_task_candidate'];review=O/current['source'];raw=json.loads(review.read_bytes());assert raw['quarter_and_handoff_passed'] and raw['all_cube_dynamics_continuous_no_resets']
    candidate=O/current['candidate'];z=np.load(candidate);assert sha(candidate)==current['candidate_sha256'];assert len(z['goals'])==6166
    old_candidate=O/'tip_squeeze/id1/plus12/candidate.npz';oldhash=sha(old_candidate)
    packet['historical_63deg_reference']=dict(candidate=old_candidate.relative_to(R).as_posix(),candidate_sha256=oldhash,matrix_counts_are_not_new_path_robustness=True,full_model_matrix=packet['full_model_matrix'],hand_only=packet['hand_only'],placement=packet['initial_placement_sensitivity'],loads=packet['hypothetical_reaction'])
    for key in ('full_model_matrix','hand_only','initial_placement_sensitivity','hypothetical_reaction'):
        packet[key].update(candidate_sha256=oldhash,historical_candidate_only=True,does_not_validate_current6166_goal_candidate=True)
    packet.update(status='nominal_model_quarter_and_handoff_passed_integration_not_ready',current_offline_candidate=candidate.relative_to(R).as_posix(),candidate_sha256=sha(candidate),selection='one nominal90plusminus5 model path; neither exact90 nor physical completion certified',total_goals=6166,added_regrasp_and_hold_goals=6166-packet['original_forward_goals_unchanged'],nominal_seconds=462.45,new_valid_whole_cube90_path=True,current_nominal_task_candidate=current,nominal_common_center_not_a_repeatability_result=True,new_path_robustness_matrix_complete=False,next_rotation_from_new_grip_verified=False,actual_left_hand_load_verified=False,current_user_physical_action_required=False)
    packet['remaining']=[
        '가정한−0.01N·m 하중에서 몸통 흔들림을 줄인다. 고정 목표는4.38도로3도 기준을 넘었고 자세 보정도 아직 전 조건을 통과하지 못했다.',
        '새6166목표 경로의 배치 오차와 구동력, 마찰 변화 시험을 한다. 이전 후보의27조건 통과를 새 경로에 적용하지 않는다.',
        '현재 약7분40초인 경로의 불필요한 대기와 재파지를 줄이고 새 하단 파지에서 다음 방향 변경을 연결한다. 다른 손이 퇴각한 뒤에만 다음 오른손 동작을 시작한다.',
        '장치를 다시 연결할 때 현재 빈손 각도와 받침 위치를 읽고 복귀부터 검토한다. 실제 왼손 형상과 하중, 윗단90도 회전 및 실물 재현은 통합 확인이 필요하다.'
    ]
    packet['remaining'][1]='새 경로의 배치6조건 중 통과·미달과 마찰 및 구동 가설8조건의 실패 원인을 보완한다. 한 변수를 바꾼14조건은 완료했지만 결합 오차 행렬과 실물 반복성은 미확인이다.'
    packet['remaining'][2]='약7분40초 경로를 단축하고 새 하단 파지에서 다음 수평 회전 및 윗면을 바꾸는 회전을 연결한다. 중간 대기를 줄인6분38초 안은 공간 기준 미달로 채택하지 않았다. 다른 손 퇴각 후에만 다음 오른손 동작을 시작한다.'
    sources=packet['source_documents_sha256']
    for p in (review,candidate,O/'continuous_handoff/trajectory.npz',O/'continuous_handoff/end_state.npz',O/'continuous_handoff/evidence/manifest.json',O/'temporal_handoff_review.json'):
        sources[p.relative_to(R).as_posix()]=sha(p)
    for name in ('source_model','source_initial_state'):
        p=R/raw[name];assert sha(p)==raw[name+'_sha256'];sources[p.relative_to(R).as_posix()]=sha(p)
    for s in raw['segments']:
        for key in ('source_goals','source_saved_endpoint'):
            if s[key]:
                p=R/s[key];assert sha(p)==s[key+'_sha256'];sources[p.relative_to(R).as_posix()]=sha(p)
    for p in sorted(R.glob('*20261106.py')):packet['implementation_sha256'][p.name]=sha(p)
    packet['source_documents_sha256']=sources;packet['source_documents_sha256'][(O/'offline_followup.json').relative_to(R).as_posix()]=sha(O/'offline_followup.json')
    file.write_text(json.dumps(packet,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    loads=current['load_results'];loadrows=''.join(f"<tr><td>{c['torque_Nm']:+g}</td><td>{'통과' if c['passed'] else '미달'}</td><td>{c['yaw_drift_deg']:.2f}도</td><td>{c['gap_mm']:.2f}mm</td></tr>" for c in loads)
    remaining=''.join('<li>'+html.escape(s)+'</li>' for s in packet['remaining'])
    feedback=[]
    for p in sorted((O/'feedback_body_hold').glob('*/review.json')):
        d=json.loads(p.read_bytes());feedback.append((p.parent.name,len(d['cases']),sum(c['passed'] for c in d['cases'])))
    feedbackrows=''.join('<tr><td>'+html.escape(n)+'</td><td>'+f'{ok}/{total}'+'</td></tr>' for n,total,ok in feedback)
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>오른손 회전과 하단 파지: 현재 결과</title><style>body{font:17px/1.8 system-ui;max-width:1080px;margin:30px auto;padding:0 20px;color:#24313c;background:#fafbfc}.notice{background:#fff0ce;border-radius:10px;padding:20px}.ok{background:#e3f3eb;padding:20px;border-radius:10px}.flow{display:flex;gap:8px;flex-wrap:wrap}.flow span{padding:12px;background:#e5edf7;border-radius:8px}video{width:100%;max-width:960px}img{width:100%;max-width:960px}table{border-collapse:collapse;width:100%;background:white}td,th{padding:10px;text-align:left;border-bottom:1px solid #dce1e5}details{padding:15px;background:white;margin:15px 0}a{color:#145c9e}.small{font-size:14px;color:#596775}</style><h1>오른손 회전과 하단 파지: 현재 결과</h1><p class="ok">새 기준 모델 한 조건에서 처음 배치 → 약85.94도 회전 → 하단 재파지 → 윗단 공간 확보를 연속 재생했다. 목표90±5도 범위를 통과했다. 큐브 자세를 중간에 초기화하지 않았다.</p><p class="notice">정확히90도 및 실물 성공을 확인한 결과는 아니다. 큰 하중 한 방향, 새 경로의 배치 오차, 다음 회전 연결과 실제 왼손 통합은 아직 남아 있다. 현재 경로는 약7분40초로 길며 새 실물 실행 명령은 만들지 않았다.</p><div class="flow"><span>처음 배치</span><span>전체 방향 변경</span><span>재파지와 공간 확보<br>기준 모델 통과</span><span>다른 손의 윗단 회전<br>실물 통합 미확인</span><span>다음 방향 변경<br>연결 미완성</span></div><h2>현재 연속 경로</h2><table><tr><th>최종 회전</th><td>85.94도, 목표90±5도</td></tr><tr><th>윗면 기울기</th><td>1.69도</td></tr><tr><th>목표 자세와의 전체 각도 오차</th><td>4.29도, 기준5도 이내</td></tr><tr><th>윗단 공간 최소 여유</th><td>0.89mm, 마지막4.05초의 모든4050개 상태 검사</td></tr><tr><th>하단 검지와 엄지 접촉</th><td>마지막 검사 구간의96.0%</td></tr><tr><th>목표 수와 시간</th><td>6166개, 최대2틱/75ms, 총462.45초</td></tr><tr><th>연속성</th><td>9개 구간의 목표 경계 일치, 저장 중간 상태 재현 오차0</td></tr><tr><th>동적 기준</th><td>모든1ms 상태의 원래 관절, 추종, 충돌 기준 통과</td></tr></table><h2>전체 경로 영상 12배속</h2><video controls preload="metadata" src="continuous_handoff/evidence/whole_path_12x.mp4"></video><p><a href="continuous_handoff/evidence/index.html">마지막30초 원래 속도와 최종 자세 그림</a></p><img src="continuous_handoff/evidence/final_side.png" alt="윗단 공간을 비운 새 모델 하단 파지"><p>파랑=검지, 초록=중지, 노랑=약지, 빨강=엄지. 주황 원통은 윗단 작업 공간이다. 색상과 원통은 표시용이다.</p><h2>다른 손이 들어오는 시점</h2><p>오른손이 큐브 전체를 돌리거나 재파지할 때는 윗단 공간을 사용해도 된다. 다른 손 접근 전에 비우고, 접근과 윗단 회전, 퇴각이 끝날 때까지 하단 고정과 공간 여유를 유지한다. 다른 손이 완전히 빠진 뒤 다음 오른손 회전을 시작한다.</p><p><a href="temporal_handoff_review.html">단계별 공간 사용과 기기 없는 연결 순서</a></p><h2>가정한 윗단 회전 하중</h2><p>새 파지에서 관절 목표를 고정하고 전체 강체 큐브에 회전 하중을 주었다. 실제 왼손과 분리된 윗단을 함께 실행한 결과는 아니다.</p><table><tr><th>가정 하중 N·m</th><th>결과</th><th>몸통 최대 회전</th><th>윗단 공간 최소 여유</th></tr>'''+loadrows+'''</table><p>−0.01N·m에서 몸통이4.38도 움직여3도 기준을 넘었다. 힘만 높이는 방법과 완전한 모델 자세를 이용하는 피드백도 비교했다. 후자는 실제 카메라나 SAC 제어가 아니다.</p><details><summary>자세 보정 비교 결과</summary><table><tr><th>모델 보정 조건</th><th>통과/3조건</th></tr>'''+feedbackrows+'''</table><p>각 조건은0와±0.01N·m의10.05초 시험이다. 실패는 숨기지 않았으며 회전, 기울기, 이동과 공간 여유를 모두 확인했다. 실제 구동 설정은 바꾸지 않았다.</p></details><details><summary>이전 후보의 오차 시험: 새 경로의 성공률로 사용하지 않음</summary><p>이전 약63도 경로는 전체 모델27/27, 손만9/9, 배치 오차25/27, 비영 반력14/18이었다. 이 횟수는 이전 후보에만 해당한다. 새6166목표 경로의 배치와 마찰 오차 시험은 아직 완료하지 않았다.</p><p><a href="evidence_thumb12/index.html">이전 약63도 후보 영상</a> / <a href="countergrip_progress/index.html">회전 뒤 재파지의 이전 미달 사례</a> / <a href="near_quarter_full_replay/evidence/index.html">이전 약89도 회전과 공간 미달 사례</a></p></details><details><summary>모델 가정과 별도 윗단 시험</summary><p>새 경로의 관절 토크 한도0.15N·m와 마찰0.3은 모델 가정이다. 실제450mA를 환산한 값이 아니다. 모터 전류와PID, 원래 관절 한계 및 충돌 형상을 바꾸지 않았다. 윗단을 분리한 이전 별도 모델6조건은 통과했으나, 새 경로의 끝 상태와 연결해 검증한 결과는 아니다.</p><p><a href="articulated_upper_layer/evidence/index.html">이전 별도 윗단 모델 영상</a> / <a href="offline_followup.json">정적 제안, 자유 큐브 탐색 및 실패 기록</a></p></details><h2>남은 작업</h2><ol>'''+remaining+'''</ol><p>현재 기기를 연결하거나 추가로 촬영할 필요는 없다. 실물 실행 준비가 끝나면 빈손 복귀부터 별도 안내한다.</p><p><a href="offline_review.json">현재 결과와 모든 출처 해시</a> / <a href="continuous_handoff/review.json">새 연속 경로 원시 결과</a> / <a href="two_hand_interface_draft.md">양손 연결 초안</a> / <a href="software_checks.log">소프트웨어 시험 기록</a></p></html>'''
    sensitivity=current['one_factor_sensitivity'];srows=''.join('<tr><td>'+{'placement':'배치 ±1mm 또는 ±2도','friction':'마찰0.2 /0.25 /0.4 /0.5','gain':'구동 배율0.8 /0.9 /1.1 /1.2'}[g['group']]+'</td><td>'+f"{g['passed']} / {g['cases']}"+'</td><td>'+f"{g['original_dynamic_safety_passed']} / {g['cases']}"+'</td></tr>' for g in sensitivity['groups'])
    section='<h2>새 경로의 배치와 모델 오차 시험</h2><p>기준 조건을 중복 계산하지 않고14조건을 추가했다. 매번 처음 배치부터 전체 경로를 다시 재생했으며 목표90도와 최종 기준 위치를 바꾸지 않았다.</p><table><tr><th>바꾼 가설</th><th>전체 목표 통과</th><th>동적 기준 통과</th></tr>'+srows+'</table><p>한 번에 한 변수만 바꾼 시험이다. 마찰과 구동 가설의 변화에서 실패가 남아 실물 반복성을 인증하지 않는다.</p><p><a href="continuous_sensitivity/review.json">14조건의 실패 원인과 원시 결과</a></p>'
    page=page.replace('<h2>가정한 윗단 회전 하중</h2>',section+'<h2>가정한 윗단 회전 하중</h2>')
    page=page.replace('처음 배치 → 약85.94도 회전','처음 배치 → 수평 약85.94도 회전')
    page=page.replace('새6166목표 경로의 배치와 마찰 오차 시험은 아직 완료하지 않았다.','새6166목표 경로는 별도로14개 단일 변수 오차 조건을 검사했으며 위 표에 미달을 포함해 표시했다. 결합 오차 행렬과 실물 반복 시험은 아직 미확인이다.')
    page=page.replace('<h2>남은 작업</h2>','<p>현재 성공 범위는 선택한 초록 윗면을 유지하는 수평 방향 변경이다. 윗면을 다른 색 면으로 바꾸는 회전 경로는 별도 검증이 필요하다.</p><h2>남은 작업</h2>')
    (O/'offline_review.html').write_text(page,encoding='utf-8')
    note='# 현재 오프라인 작업\n\n새 기준 모델 한 조건에서 초기 배치부터6166목표를 연속 재생했다. 최종 회전85.94도, 기울기1.69도, 윗단 여유0.89mm로90±5도 목표와 하단 재파지를 통과했다. 실제 로봇 또는 정확한90도 성공은 미확인이다.\n\n고정 목표의 강체 하중 시험은4/5다. −0.01N·m에서 몸통4.38도 움직임이 남는다. 모델 자세 피드백도 연구했으나 실제 카메라나SAC 제어가 아니다.\n\n이전27/27 등 오차 시험 횟수는 이전 약63도 후보에만 해당한다. 새 경로는 모델 한 조건이며 배치와 마찰 오차, 다음 회전 연결, 실물 복귀와 왼손 통합은 남았다.\n\n현재 영상: `continuous_handoff/evidence/index.html`. 전체 결과: `offline_review.html` 및 `offline_review.json`.\n\n'+'\n'.join('- '+s for s in packet['remaining'])+'\n'
    (O/'next_stage.md').write_text(note,encoding='utf-8')
    progress=R/'professor_progress_20261015.md';data=progress.read_bytes();begin=b'<!-- offline_packet_20261105_current -->';end=b'<!-- offline_packet_20261105_end -->'
    assert data.startswith(begin) and end in data;history=data.split(end,1)[1].lstrip(b'\r\n')
    header='<!-- offline_packet_20261105_current -->\n## CURRENT 2026-11-06: 새 연속 모델 경로와 하단 파지\n\n- 실물 장치는 열지 않았다. 사용자 요청에 따라 손과 웹캠 없이 계산했다. 새 실물 동작 명령은 만들지 않았다.\n- `continuous_handoff/candidate.npz`: 처음 배치부터6166목표를 중간 초기화 없이 연속 재생, 약85.94도 회전과 하단 재파지 및 윗단 공간 확보를 기준 모델 한 조건에서 통과했다. 목표는90±5도 범위이며 정확히90도 또는 실물 성공 인증이 아니다.\n- 기울기1.69도, 전체 자세 오차4.29도, 마지막4050개1ms 상태의 공간 최소 여유0.89mm, 하단 접촉96.0%다. 목표 경계와9개 중간 상태가 일치했다.\n- 고정 목표의 가정한 하중은4/5다. −0.01N·m에서 몸통이4.38도 움직여 미달했다. 실제 왼손 하중과 형상은 미확인이다. 자세 피드백은 완전한 모델 관측을 쓰는 연구이며 실물 카메라 또는SAC 실행이 아니다.\n- 현재 모델 경로는462.45초다. 단축, 새 경로의 배치와 구동 오차 검증, 다음 방향 변경 연결, 실물 복귀와 통합을 이어서 진행한다. 이전27/27 등의 횟수는 이전 약63도 후보에만 해당한다.\n- 소프트웨어 시험'+str(packet['software_tests']['run'])+'개 통과. 전류/PID, 원래 관절과 동적 기준 및 보호 대상6파일은 유지했다.0.15N·m 모델 한도는450mA의 환산값이 아니다.\n- 현재 영상과 보고서: `right_hand_rotation_20261105/canonical_body_grip/offline_review.html`. 상세 영상: `continuous_handoff/evidence/index.html`.\n\n<!-- offline_packet_20261105_end -->\n\n'
    header=header.replace('약85.94도 회전','수평 약85.94도 회전')
    header=header.replace('- 현재 모델 경로는462.45초다.','- 새 경로의 단일 변수 오차 시험14조건을 완료했다. 실패 조건을 별도로 기록했으며 실물 반복성 검증은 아니다. 현재 모델 경로는462.45초다.')
    progress.write_bytes(header.replace('\n','\r\n').encode('utf-8')+history)
    assert progress.read_bytes().split(end,1)[1].lstrip(b'\r\n')==history
    print(json.dumps(dict(current_candidate_sha256=sha(candidate),nominal_quarter_and_handoff=True,physical_success=False,historical_counts_not_transferred=True,load_passed=current['load_passed'],load_cases=current['load_cases'],history_sha256=hashlib.sha256(history).hexdigest(),devices_opened=False,hardware_execution_allowed=False)),flush=True)
if __name__=='__main__':main()
