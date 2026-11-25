"""Publish evidence for repositioning before reusing an existing second turn."""
from pathlib import Path
import hashlib
import html
import json
import os

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'reposition_then_second_rotation_20261111'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def link(p,label):return '<a href="'+os.path.relpath(p,FOLDER).replace('\\','/')+'">'+html.escape(label)+'</a>'
def block(path,body,after=None):
    begin='<!-- rotation_entry_reposition_20261111 -->';end='<!-- rotation_entry_reposition_20261111_end -->';s=path.read_text(encoding='utf-8')
    if begin in s:a=s.index(begin);b=s.index(end,a)+len(end);s=s[:a]+s[b:].lstrip('\n')
    at=s.index(after)+len(after) if after else 0;new=begin+'\n'+body+'\n'+end+'\n\n';path.write_text(s[:at]+('\n\n' if after else '')+new+s[at:].lstrip('\n'),encoding='utf-8')


def main():
    sources={};sections=[];trials=[];planners=[]
    def get(p):sources[p.relative_to(OUT).as_posix()]=sha(p);return read(p)
    audit=get(FOLDER/'entry_state_comparison.json');get(FOLDER/'visual_manifest.json')
    sections.append('<section><h2>먼저 맞추는 대상</h2><p>현재 낮은 마찰 모델의 첫 유지와 기존 두 번째 회전 성공 경로의 시작 상태를 비교했다. 큐브 중심 차이는 '+f"{audit['cube_center_difference_mm']:.3f} mm, 방향 차이는 {audit['cube_orientation_difference_deg']:.3f}도, 손가락 지령 차이는 최대 {audit['maximum_motor_command_difference_ticks']:.0f}틱"+'이다. 기존 성공 상태는 이동 목표이며 실제 모델 상태를 그곳으로 순간이동시키지 않는다.</p><img style="width:100%" src="entry_comparison.png">'+link(FOLDER/'entry_state_comparison.json','시작 상태 원본 비교')+'</section>')
    for name in ('robust_mu025_reposition_to_nominal_entry_v1','robust_mu025_regrasp_nominal_entry_v2','robust_mu025_entry_orientation_repair_v3'):
        path=BASE/name/'review.json'
        if not path.exists():continue
        r=get(path);final=r['final'];row=dict(name=name,steps=r['steps'],stop=r['stop'],start_center_error_mm=r['start']['center_target_error_mm'],final_center_error_mm=final['center_target_error_mm'],final90_error_deg=final['error_deg'],reference_command_error_ticks=final.get('reference_command_error_ticks'),entry_command_tolerance_ticks=r.get('entry_command_tolerance_ticks'),passed=r['conditional_bridge_and_hold_passed']);planners.append(row)
        detail=f"{r['steps']}목표. 시작 위치 오차 {row['start_center_error_mm']:.3f} → {row['final_center_error_mm']:.3f} mm, 첫90도 방향 오차 {row['final90_error_deg']:.3f}도. 종료 사유 {r['stop']}, 해당 경로의 자체 진입 유지 검사={row['passed']}."
        sections.append('<section><h2>'+html.escape(name)+'</h2><p>'+html.escape(detail)+'</p>'+link(path,'계획·실패를 포함한 전체 기록')+'</section>')
    finger_path=FOLDER/'finger_alignment_orders_v1/review.json'
    if finger_path.exists():
        fingers=get(finger_path);complete=[r for r in fingers['rows'] if len(r['order'])==4 and not r['stop'] and r['final']['command_error_ticks']==0]
        sections.append('<section><h2>손가락 순서에 따른 재파지</h2><p>중지를 먼저 단순 정렬한 경우 모델 자기 접촉에서 멈췄다. 검지를 먼저 옮긴 두 순서는 네 손가락 목표까지 충돌 없이 도달했지만 큐브 방향이 기존 시작 상태에서 약9.7~9.8도 틀어졌다. 순차 정렬 완료 '+str(len(complete))+'개를 회전 성공으로 세지 않는다.</p>'+link(finger_path,'4개 개별 동작과6개 순서 비교')+'</section>')
    for path in sorted((FOLDER/'suffix_trials').glob('*/review.json')):
        r=get(path);row=dict(name=path.parent.name,steps=r['steps'],stop=r['stop'],before=r['before'],after_alignment=r['after_alignment'],final=r['final'],initialized_reference_control=r.get('initialized_reference_control_not_low_friction_arrival',False),passed=r['second_rotation_and_clear_hold_passed']);trials.append(row)
    divergence_path=FOLDER/'friction_divergence/review.json'
    if divergence_path.exists():
        divergence=get(divergence_path);first=divergence['first_divergence'];finals={r['case']:r['trace'][-1]['error180_deg'] for r in divergence['rows']}
        sections.append('<section><h2>같은 시작 상태에서 마찰만 바꾼 대조</h2><p>'+f"첫 차이는 {first['after_goal']}목표에서 나타났고 방향 차이가 {first['orientation_difference_deg']:.3f}도였다. 마지막 교사 구간 뒤 목표 오차는 명목 {finals['nominal']:.3f}도, 마찰0.25 {finals['mu025']:.3f}도였다."+' 정확한 시작 상태만 맞추면 기존 지령을 그대로 재사용할 수 있다는 가정은 이 조건에서 성립하지 않았다. 낮은 마찰 대조의 초기화는 실제 도달 연결로 세지 않는다.</p>'+link(divergence_path,'단계별 실제 모델 상태·접촉 비교')+'</section>')
    contact_studies=[]
    for name in ('robust_mu025_reposition_MF_contact_geometry_v1','robust_mu025_reposition_TH_counter_geometry_v1','robust_mu025_reposition_MF_contact_arrival_v1','robust_mu025_reposition_MF_pressure_v1','robust_mu025_reposition_MF_reset_gait_v1','robust_mu025_reposition_MF_gait2_geometry','robust_mu025_reposition_MF_gait2_arrival','robust_mu025_directed_standoff_v1','robust_mu025_directed_standoff_plan_v2','robust_mu025_free_MF_standoff_route_v1'):
        path=BASE/name/'review.json'
        if not path.exists():continue
        study=get(path);contact_studies.append(dict(name=name,status=study['status'],path=path.relative_to(OUT).as_posix()))
        sections.append('<details><summary>'+html.escape(name)+'</summary>'+link(path,'원본 모델 결과와 실패 기록')+'</details>')
    direction_path=FOLDER/'actual_contact_direction_audit/review.json'
    if direction_path.exists():
        direction=get(direction_path);actual=direction['rows'][0]
        sections.append('<section><h2>접근 중 회전과 끝의 접촉 방향</h2><p>중지 접근 중 약14.8도 회전했지만 실제 도달 뒤 중지 끝 접촉의 회전 모멘트 팔은 '+f"{actual['positive_yaw_lever_mm']:.3f} mm"+'로 반대 방향이었다. 이 상태에서 압력만 더한 기존 연구 제어는225목표에서80mm 이탈 조건에 걸렸다. 이를 올바른 접촉 도달이나 전체 회전 성공으로 세지 않는다.</p>'+link(direction_path,'실제 접촉 방향 재검사')+'</section>')
    guard_path=BASE/'robust_mu025_reposition_pressure_direction_guard_v2/review.json'
    if guard_path.exists():
        guard=get(guard_path);get(BASE/'directed_pressure_guard_nominal_regression/review.json')
        assert all(r['steps']==0 and r['stop']==['actual_contact_would_push_reverse_yaw'] for r in guard['rows'])
        sections.append('<section><h2>현재 접촉 방향을 확인한 뒤 지령</h2><p>압력 제어에 실제 접촉 방향 검사를 추가했다. 위 반대 방향 상태는 세 압력 조건 모두 지령0개에서 거부했다. 명목 모델의 기존 정상 압력 제어는 앞10목표를 재현했다. 반대 방향 차단은 회전 경로 완성을 뜻하지 않는다. 큐브를 스치지 않는 접근 경로를 별도로 계획한다.</p>'+link(guard_path,'지령0개 차단 결과')+'</section>')
    for name in ('robust_mu025_standoff_MF_pressure_v3','robust_mu025_declared_face_feedback_v4','robust_mu025_declared_face_feedback_v5','robust_mu025_feedback_reset_v6','robust_mu025_feedback_reset_v7','robust_mu025_feedback_gait_geometry_v8','robust_mu025_feedback_standoff_plan_v8','robust_mu025_feedback_free_route_v8','robust_mu025_feedback_free_route_diagnostic_v9','robust_mu025_feedback_free_route_v10','robust_mu025_feedback_gait_push_v11'):
        path=BASE/name/'review.json'
        if not path.exists():continue
        study=get(path);contact_studies.append(dict(name=name,status=study['status'],path=path.relative_to(OUT).as_posix()))
        sections.append('<details><summary>'+html.escape(name)+'</summary>'+link(path,'원본 모델 결과와 실패 기록')+'</details>')
    endpoint=get(FOLDER/'standoff_endpoint_audit/review.json');route=get(FOLDER/'free_standoff_replay_verification/review.json');continuation=get(FOLDER/'continuation.json');latest=continuation['completed_model_studies'][-1]
    sections.insert(0,'<section><h2>현재 결과와 남은 일</h2><p>마찰0.25 가정에서 중지의 접촉 위치를 큐브에 맞춰 갱신하는 모델 제어를 만들었다. 두 번째 회전의 목표 오차는 약80.5 →51.1도로 감소했다. 압력0.8N 가정으로 이어 민 별도 분기는42.7도에서 중지 벌림 관절의 상한 접근 때문에 멈췄다. 상한에 닿기 전 상태에서 중지를 다시 배치하고 우회 접근한 새 분기는 원래 관절·충돌·추종 제한을 넘지 않고37.578도까지 줄었다. 최종 유지 단계에는 아직 들어가지 않았다.</p><p>중지 접근 경로240목표를 별도로 재생해18000개의1ms 상태에서 지지와 저장 끝 상태 일치를 확인했다. 도달 끝의 실제 간격7.462mm는 독립 형상 계산과도 일치했다. 이후 재배치 경로는 원래 모델에서 큐브와 충돌하도록 지정된 형상만 확인하며, 자기 접촉용 형상을 큐브 장애물로 잘못 추가하던 계획 검사를 바로잡았다. 모델 충돌 설정과 원래 중단 기준은 바꾸지 않았다.</p><p>전체6736목표 구간의 저장 시각을 검사하니 중간에 중단된 연구 유지의 마지막 지령이38ms였다. 이 구간을 포함한 정확한 상태 연결 재생과 모든 지령75ms 비교가 필요하다. 구간별 성공을 전체 실행 준비 완료로 세지 않는다.</p><p>추가 압력 비교와 처음 배치부터의 독립 재생은 자동 승인 검토의 사용량 제한으로 실행되지 않았다. 위험하다는 판정이 아니라 승인 검토가 진행되지 못한 것이며, 다른 실행 방식으로 우회하지 않았다.</p>'+link(FOLDER/'continuation.json','완료 결과·시각 검사·미실행 계산 목록')+' · '+link(FOLDER/'free_standoff_replay_verification/review.json','240목표 독립 재생')+'</section>')
    replay_text=' '.join(f"{r['steps']}목표, 지령75ms={r['original75ms_schedule_used']}, 최종 오차{r['final180_error_deg']:.3f}도, 중단={r['stop']}." for r in continuation.get('independent_replays',[]))
    selected=continuation.get('selected_stable_partial_model_case',latest)
    sections.insert(0,'<section><h2>계산 재개 후 현재 결과</h2><p>이전 사용량 제한 이후 후속 계산을 실제 실행했다. 보존된 부분 경로의 현재 오차는 '+f"{selected['final180_error_deg']:.3f}도"+'이며 최종 회전·유지 성공은 아니다.</p><p>'+html.escape(replay_text)+'</p><p>정확한 저장 시각 재생의11개 상태 연결은 모두 일치했다. 전체75ms 재생도 원래 관절·충돌·추종 제한을 넘지 않았고 첫4050샘플 유지에서 지지와 윗단 공간을 다시 확인했다.</p><p>중지 일부의 반대 방향 접촉만으로 차단하던 보조 검사 대신, 손끝과 중지 전체의 합산 방향을 검사하는 비교를 수행했다. 실제 예에서 반대 지지 모멘트-0.00158N·m보다 손끝의+0.02799N·m가 컸고 원래 모델 제한도 통과했다. 이 보조 검사 수정은 전류·관절·충돌 한계 변경이 아니다. 그래도 회전 정체는 남았다.</p><p>엄지 하중 완화는 중간 오차21.6도 뒤 큐브가 기울고 이탈하는 실패로 끝났다. 약지 접촉4면 탐색에서는 새 후보를 찾지 못했고, 엄지는 정적 후보가 있었지만 단순 접근3경로는 큐브 자세를 흐트러뜨렸다. 모두 실패로 보존한다. 이어지는 전체 손 예측 제어의 결과는 아래 원본 기록에 포함한다.</p>'+link(FOLDER/'continuation.json','실행 결과와 다음 계산 상태')+'</section>')
    for name in ('robust_mu025_feedback_pressure_v12','robust_mu025_feedback_bounded_fit_v13','robust_mu025_feedback_net_moment_v14','robust_mu025_late_RF_geometry_v15','robust_mu025_late_TH_geometry_v15','robust_mu025_late_TH_standoff_v16','robust_mu025_TH_relief_v18','robust_mu025_late_whole_hand_v19','robust_mu025_feedback_chain_exact_replay_v1','robust_mu025_feedback_chain75ms_replay_v1'):
        path=BASE/name/'review.json'
        if path.exists():
            r=get(path);contact_studies.append(dict(name=name,status=r['status'],path=path.relative_to(OUT).as_posix()));sections.append('<details><summary>'+html.escape(name)+'</summary>'+link(path,'재개 후 실제 계산 결과')+'</details>')
    get(FOLDER/'finger_moment_admission_audit/review.json')
    table=''
    for row in trials:
        after=row['after_alignment'] or row['before'];label=row['name']+('（기준 상태 초기화 대조）' if row['initialized_reference_control'] else '');table+='<tr><td>'+html.escape(label)+'</td><td>'+f"{row['before']['cube_center_error_to_entry_mm']:.3f}"+'</td><td>'+f"{after['cube_center_error_to_entry_mm']:.3f}"+'</td><td>'+f"{after['cube_orientation_error_to_entry_deg']:.3f}"+'</td><td>'+f"{row['final']['final180_error_deg']:.3f}"+'</td><td>'+html.escape(str(row['stop']))+'</td><td>'+('통과' if row['passed'] else '미달')+'</td></tr>'
    sections.append('<section><h2>같은 기존 회전 경로를 이어 본 결과</h2><table><tr><th>진입 방식</th><th>재배치 전 중심차 mm</th><th>재배치 후 중심차 mm</th><th>재배치 후 방향차 도</th><th>최종180도 오차</th><th>중단</th><th>두 번째 유지</th></tr>'+table+'</table><p>손가락 목표만 같아도 큐브 위치와 방향이 다르면 기존 회전 결과가 달라졌다. 중간의 윗단 공간 미달은 회전 도중의 진단으로 기록하고, 최종 유지에서 전체 공간 검사를 요구한다. 첫 유지의 기존 작은 기준 미달을 통과로 바꾸지 않는다.</p></section>')
    summary='2026-11-11 사용자 요청: 먼저 큐브를 기존 회전의 시작 위치·파지로 옮긴 후 기존 회전을 재사용하는 연결을 검증한다. 현재 첫 유지와 기존 성공 시작의 큐브 중심 차이9.638mm, 방향차2.465도, 손가락 지령 최대591틱 차이를 확인했다. 위치 먼저 이동한 경로는 중심차0.970mm·첫90도 오차3.563도에 도달했다. 이어 손가락을 단순 정렬하자 중심차1.274mm·기존 시작과 방향차12.497도가 되어 회전 전 자세가 다시 틀어졌고, 같은 두 번째 경로의 최종180도 오차88.212도로 실패했다. 손가락만 먼저 맞춘 비교도87.457도로 실패했다. 다음 연결은 큐브 위치·방향을 관측하면서 손가락 시작 파지까지 함께 맞추는 진입을 사용한다. 기존 명목 모델의5089목표 두 회전·두 유지 성공과1틱 여유 중간 파지4050샘플 성공은 유지한다. 새SAC 학습0, 물리 장치 및 원격 쓰기0, 전체 실물 실행 허용은false다. 최신 근거: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/reposition_then_second_rotation_20261111/index.html`.'
    for row in planners:
        if not row['name'].endswith('_v1'):summary+=f" {row['name']}: {row['steps']}목표, 중심차{row['final_center_error_mm']:.3f}mm,90도 오차{row['final90_error_deg']:.3f}도, 손가락 목표 차이{row['reference_command_error_ticks']}틱, 종료 사유{row['stop']}."
    if finger_path.exists():summary+=' 약지→검지→중지→엄지 순서는540목표에서 네 손가락 지령을 맞췄으나 큐브 방향이 기존 시작에서9.731도 차이 났다. 순차 정렬 완료와 첫90도 자세 복원은 별도다.'
    for row in trials:
        if row['initialized_reference_control']:summary+=f" 기존 성공 상태를 그대로 초기화하고 마찰만0.25로 바꾼 대조의 최종180도 오차는{row['final']['final180_error_deg']:.3f}도, 두 번째 유지 통과={row['passed']}다. 이 초기화는 낮은 마찰에서 실제 도달한 연결로 세지 않는다."
    if direction_path.exists():summary+=' 실제 접촉 방향을 재검사하니14.8도 접근 후 중지 끝의 회전 팔은-26.591mm로 반대 방향이었다. 압력만 추가한225목표 이탈은 실패로 보존했다. 새 실제 방향 검사로 이 상태를 지령0개에서 거부했고, 기존 명목 압력 제어 앞10목표도 재현했다. 정적 접촉 제안과 실제 접촉 도달을 구분하며, 큐브를 스치지 않는 중지 우회 경로를 계획한다.'
    summary='2026-11-11 최신: 사용자 요청으로 회전 전 큐브 이동과 재파지 연결을 연구했다. 현재 낮은 마찰0.25 모델의 두 번째 회전은 중지 접촉점을 관측·갱신하는 제어와 중지 벌림을 줄이는 재배치를 거쳐 최종180도 목표 오차37.578도까지 줄었다. 0.8N 단순 연속 분기는42.698도에서 관절 상한 접근 때문에 중단됐으며, 상한을 넓히지 않고 그 이전 상태에서 재배치한 분기는1600목표 동안 기존 한계 초과 없이 진행됐다. 37.578도도 목표 미달이며 마지막 유지·전체 연결 성공은 아니다. 반대 방향 중지 접촉은 압력 지령0개에서 차단하며, 이전225목표 이탈 실패는 보존했다. 새 우회 접근240목표는 독립 재생18000샘플에서 지지·끝 상태 일치와 실제 간격7.462mm를 확인했다. 추가 재배치 우회110목표도 기존 모델 충돌 설정을 유지해 도달했다. 구간별 저장 시각 검사에서 총6736목표 중 중단된 연구 유지의 마지막 지령이38ms인 것을 확인했다. 정확한 상태 연결과 전체75ms 재생을 따로 검사해야 하며 아직 실행하지 않았다. 후속 압력1.5N 가정 비교와 초기 배치부터의 재생 호출은 자동 승인 검토의 사용량 제한 때문에 실행되지 않았다. 위험 판정이 아니고 승인 검토 미완료다. 실행할 스크립트·인수·출처 해시는 `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/reposition_then_second_rotation_20261111/continuation.json`에 있다. 기존 명목 모델5089목표 두 회전·두 유지 성공, 낮은 마찰 및 배치 오차 실패는 보존했다. 새SAC 학습0, 장치·원격 쓰기0. 남은 일은 낮은 마찰의 나머지 회전과 최종 유지, 처음 배치부터75ms 연결, 배치 오차 재검사다. 물리 관측 좌표 정합과 실물 전체 동작은 미검증이며 지금 재연결·촬영을 요구하지 않는다.'
    summary='2026-11-11 계산 재개: 이전 자동 승인 검토 사용량 제한 이후 압력 비교와 독립 재생3개를 실제 실행했다. 낮은 마찰0.25의 저장 지령6736목표를 원래 배치부터 초기화 없이 재생했으며11개 상태 경계가 모두0 차이였다. 38ms로 중단됐던 연구 지령을75ms 끝까지 유지한 비교도505200개의1ms 검사에서 원래 관절·충돌·추종 기준을 통과했다. 첫 유지4050샘플은 전부 지지·윗단 공간을 통과했고 최종180도 오차는37.591도다. 후속 압력1.5N 가정 비교는26.369도까지 줄었다. 중지 합산 모멘트 검사를 추가한 보존된 부분 경로는'+f"{selected['final180_error_deg']:.3f}도"+'이며 전체 회전·최종 유지 성공은 아니다. 중지 일부가 반대로 받쳐도 합산 회전 방향은 양수인 실제 접촉을 확인했고, 이 보조 방향 검사를 수정하는 비교에서도 원래 제한은 유지했다. 엄지 하중을 줄이는 비교는 한때21.6도 오차 뒤 기울기·이탈로 끝나 실패로 보존했다. 약지4면 탐색은 후보0개, 엄지 정적 후보의 단순 접근3개도 실패했다. 전체 손의 위치·방향 예측 제어 후속은 원본 결과와 함께 확인한다. 최신 실행 목록과 근거는 `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/reposition_then_second_rotation_20261111/continuation.json`이다. 새SAC 학습0, 물리 장치·원격 쓰기0, 실물 전체 동작 허용false다. 물리 관측 좌표 정합과 배치 오차 검증도 남아 있으며 현재 연결·촬영을 요구하지 않는다.'
    report=dict(status='rotation_entry_reposition_and_regrasp_in_progress',source_reports_sha256=sources,entry_comparison=audit['status'],planners=planners,suffix_trials=trials,contact_studies=contact_studies,latest_low_friction_final180_error_deg=selected['final180_error_deg'],latest_low_friction_final_hold_samples=selected['hold_samples'],all_segment_durations_75ms=continuation['all_segment_durations_75ms'],complete75ms_partial_chain_replay_completed=True,pending_model_calculations_due_to_approval_usage_limit=False,full_initial_placement_chain_replay_completed=True,full_two_turn_two_hold_task_passed=False,old_nominal_two_turn_success_preserved=True,original_native_guards_and_joint_limits_preserved=True,original2mm_same_station_miss_not_added_as_prerequisite=True,new_SAC_training_steps=0,devices_opened=0,remote_writes=0,hardware_execution_allowed=False)
    final_path=BASE/'robust_mu025_final_pose_space_v38/review.json';whole_path=BASE/'robust_mu025_complete_cycle75ms_replay_v39/review.json'
    if final_path.exists():
        final=get(final_path);whole=get(whole_path) if whole_path.exists() else None;window=final['final_hold'];full_pass=bool(whole and whole['full_two_turn_two_hold_success'])
        for path in (BASE/'robust_mu025_late_TH_normal_release_v35/review.json',BASE/'robust_mu025_final_clear_hold_v37/review.json',FOLDER/'final_hold_space_audit_named/review.json'):
            get(path)
        plot=FOLDER/'late_rotation_plot.json'
        if plot.exists():get(plot)
        full_text=(f"원래 배치부터 {whole['steps']}목표, {whole['all1ms_samples']}개의1ms 검사를 초기화 없이 재생했다. 전체 지령75ms={whole['original75ms_schedule_used']}, 두 회전·두 유지 통과={full_pass}. 최종 오차{whole['final180_error_deg']:.3f}도." if whole else '최종 구간까지 포함한 처음 배치부터의 독립 재생은 진행 중이다.')
        sections=[s.replace('계산 재개 후 현재 결과','중간 계산 재개 기록') for s in sections]
        visual='<img style="width:100%" src="late_rotation_progress.png"><p>막대 높이는 공간 증명에서 확인한 하한이다. 음수이면 해당 공간 확보 검사가 미달했다는 의미다.</p>' if plot.exists() else ''
        sections.insert(0,'<section><h2>최신: 회전 뒤 공간과 방향을 함께 보정</h2><p>엄지의 실제 접촉면 바깥으로 빠지는 제어에서는 최종 약2.964도 오차로4050샘플 지지를 유지했다. 남은 공간 부족은 중지 뿌리 외형3개였다. 중지를 비우면 오차6.952도·공간 하한-0.600mm, 전체 기존 파지로 바꾸면 공간0.752mm·오차17.293도로 서로 다른 조건이 미달했다.</p><p>'+f"마지막 위치·방향 예측 보정은 {final['steps']}목표에서 최종 오차{window['final_error_deg']:.3f}도, 공간 하한{window['min_lower_mm']:.3f}mm, 지지4050/4050 및 공간4050/4050을 통과했다. 하한은 기존0.3mm 기준에 가까우며 일반 오차 내성을 보장하지 않는다."+'</p><p>'+html.escape(full_text)+'</p><p>이는 특정 마찰0.25 가정의 모델 결과다. 기존 배치1mm 실패, 실제 관측 좌표 정합, 왼손 도달·하중, 실물 전체 실행은 별도이며 새SAC 학습·증류·실물 전송은 수행하지 않았다. 실제450mA와 모델 접촉 힘은 환산하지 않는다.</p>'+visual+link(final_path,'마지막 보정·4050샘플 유지')+(' · '+link(whole_path,'원래 배치부터 전체 독립 재생') if whole else '')+'</section>')
        summary='2026-11-11 최신: 엄지 새 접촉과 실제 접촉 법선 바깥 이탈을 연결해 모델 전체180도 목표 오차2.964도, 기울기1.982도,4050샘플 전부 지지·작은 흔들림을 확인했다. 이때 공간은 중지 뿌리 외형3개에서 미달했다. 이후 기존 파지들을 실제 연결해 비교했고, 마지막 위치·방향·공간 예측 보정86목표에서 방향 오차4.032도, 공간 하한0.329mm, 지지·공간 각각4050/4050을 통과했다. '+full_text+' 공간 기준0.3mm와의 여유가 작으며 특정마찰0.25 모델 조건의 결과다. 배치1mm 조건의 중지±1틱4개 비교는 모두 자기 접촉에서 실패했고 원래 실패를 유지했다. 물리 관측 좌표 정합, 새 경로의 오차 내성, 왼손 도달·하중과 실물 전체 동작은 미검증이다. 새SAC 학습0, 증류·실물 전송0, 장치·원격 쓰기0. 기존170개 실물 결합과57개 미들웨어 원본은 유지한다. 최신 근거: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/reposition_then_second_rotation_20261111/index.html`.'
        report.update(full_two_turn_two_hold_task_passed=full_pass,full_two_turn_pass_scope='Specified low-friction model, reached-location holds; not physical success or general robustness',conditional_final_clear_hold_passed=final['conditional_bridge_and_hold_passed'],conditional_final_hold=window,complete_final_cycle_replay_completed=whole is not None,latest_low_friction_final180_error_deg=whole['final180_error_deg'] if whole else window['final_error_deg'],latest_low_friction_final_hold_samples=whole['final_hold']['samples'] if whole else window['samples'])
        handoff_path=FOLDER/'complete_cycle_handoff.json'
        if handoff_path.exists():
            handoff=get(handoff_path);table=''
            for case in handoff['finite_variations']:
                if not case['report_available']:continue
                table+='<tr><td>'+case['case']+'</td><td>'+str(case['steps'])+'</td><td>'+html.escape(str(case['stop']))+'</td><td>'+f"{case['final180_error_deg']:.3f}"+'</td><td>'+('통과' if case['full_two_turn_two_hold_success'] else '미달')+'</td></tr>'
            sections.insert(1,'<section><h2>실물 준비에 남는 한계</h2><p>'+f"새 경로는 모델에서 {handoff['simulated_seconds']:.2f}초, 첫 유지 대비 마지막 위치 변화 {handoff['center_displacement_mm']:.3f}mm다. 왼손이 이 이동을 흡수할 수 있다고 검증한 것은 아니다. 첫 방향 기준과 여유는 {handoff['first_hold_direction_margin_to5deg']:.4f}도, 마지막 공간 기준과 여유는 {handoff['final_upper_margin_above0p3mm']:.4f}mm다."+'</p><table><tr><th>변화 조건</th><th>실행 목표</th><th>중단 사유</th><th>최종180도 오차</th><th>전체 결과</th></tr>'+table+'</table><p>마찰0.24·0.26 또는 초기 배치+1mm 조건에서 같은 저장 지령을 재생하는 비교는 모두 실패했다. 특정 조건의 성공과 작은 오차에 대한 내성은 별도다. 다음은 현재 관측에 맞춰 접촉 전 재배치와 회전 후 파지를 보정하는 제어의 검증이다. 지금 새 물리 실행 명령을 허용하지 않는다.</p><p>전체 재생 중 원래 학습 정책에서 유래한 저장 앞부분은1700목표이고, 나머지는 모델 교사 지령이다. 이 전체 지령 재생을 새SAC 학습·증류·실물 정책 배포로 설명하지 않는다.</p>'+link(handoff_path,'전체 경로·좌표 변화·오차 비교 원본')+'</section>')
            for case in handoff['finite_variations']:
                path=BASE/('complete_cycle_variation_'+case['case'])/'review.json'
                if path.exists():get(path)
            report.update(finite_new_cycle_variations=handoff['finite_variations'],new_cycle_position_displacement_mm=handoff['center_displacement_mm'],new_cycle_small_error_robustness_passed=False,left_hand_reach_load_and_physical_registration_unverified=True)
            summary+=' 새 저장 경로의 마찰0.24·0.26 및 초기 배치+1mm 유한 비교는 모두7744목표에서 기존 관절 범위 검사로 중단돼 실패했다. 따라서 새 경로를 작은 오차에도 바로 쓰는 실물 실행 후보로 승인하지 않는다. 새 모델 경로 시간721.05초, 첫 유지 대비 최종 위치 이동20.876mm이며 왼손 도달 가능성은 미검증이다. 현재 상태 관측에 맞춘 접촉 진입·유지 보정과 오차 내성 검증이 다음 과제다. 전체 경로는 기존 학습 정책에서 유래한 저장1700목표와 모델 교사7914목표의 연결이며 새SAC 학습이나 증류가 아니다.'
    save=FOLDER/'review.json';save.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    sections=[s.replace('현재 결과와 남은 일','이전37.578도 단계의 기록') for s in sections]
    title='회전 전 큐브 이동과 시작 파지 연결'
    page='<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+title+'</title><style>body{max-width:1200px;margin:28px auto;padding:0 18px;background:#f5f8fa;color:#203d46;font:16px/1.7 system-ui}section{padding:18px;margin:18px 0;background:white;border:1px solid #d4dfe2}table{border-collapse:collapse;width:100%;font-size:14px}td,th{border-bottom:1px solid #ddd;padding:8px}a{color:#087b73}.flow{padding:16px;background:#fff0ce}</style><body><h1>'+title+'</h1><p class="flow">실제 첫 유지 → 큐브 위치 이동 → 방향을 유지하며 시작 파지 맞추기 → 기존 회전 → 최종 유지</p>'+''.join(sections)+'<p>모델 연구 기록. 실제 모터·카메라는 사용하지 않았다. 실제 관측 좌표 정합과 실물 전체 동작은 미검증이다.</p>'+link(save,'검증 범위와 출처')+' · '+link(FOLDER/'STATUS.md','인계 상태')+'</body></html>'
    (FOLDER/'index.html').write_text(page,encoding='utf-8');(FOLDER/'STATUS.md').write_text('# '+title+'\n\n'+summary+'\n',encoding='utf-8')
    block(ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',summary,after='## 지금 먼저 할 일');block(ROOT/'professor_progress_20261015.md',summary)
    for target in (BASE/'index.html',BASE/'robust_cycle_preparation_20261111/index.html',OUT.parent/'policy_audit/explanation/index.html'):
        s=target.read_text(encoding='utf-8');begin='<!-- rotation_entry_latest_link -->';end='<!-- rotation_entry_latest_link_end -->'
        if begin in s:a=s.index(begin);b=s.index(end,a)+len(end);s=s[:a]+s[b:]
        href=os.path.relpath(FOLDER/'index.html',target.parent).replace('\\','/');banner=begin+'<aside style="padding:16px;background:#fff0ce"><a href="'+href+'">최신: 회전 전 큐브 이동과 시작 파지 연결</a></aside>'+end;at=s.find('>',s.lower().find('<body'))+1;target.write_text(s[:at]+banner+s[at:],encoding='utf-8')
    readiness=OUT/'next_cycle/readiness.json';r=read(readiness);r['rotation_entry_reposition']=dict(path=save.relative_to(OUT).as_posix(),sha256=sha(save),hardware_execution_allowed=False);readiness.write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(planners=len(planners),suffix_trials=len(trials),hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
