"""Publish actual robustness and no-output integration work, retaining failures."""
from pathlib import Path
import hashlib
import html
import json
import os

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'robust_cycle_preparation_20261111'
BATCHES=('live_SAC_prefix','encoder_pose_feedback_v1_complete_bundle','identified_pose_feedback_v2','encoder_clearance_fixed2_x1','pose_feedback_margin15_mu025','tip_trajectory_feedback_v3','low_friction_teacher_cycle_connection_v4')
TEACHERS=('robust_mu025_native_teacher_v1','robust_mu025_transport_teacher_v2','robust_mu025_finish_teacher_v3','robust_mu025_finish_teacher_v4')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def link(p,label):return '<a href="'+os.path.relpath(p,FOLDER).replace('\\','/')+'">'+html.escape(label)+'</a>'


def block(path,body,after=None):
    begin='<!-- robust_cycle_preparation_20261111 -->';end='<!-- robust_cycle_preparation_20261111_end -->';s=path.read_text(encoding='utf-8')
    if begin in s:
        a=s.index(begin);b=s.index(end,a)+len(end);s=s[:a]+s[b:].lstrip('\n')
    new=begin+'\n'+body+'\n'+end+'\n\n';at=s.index(after)+len(after) if after else 0
    path.write_text(s[:at]+('\n\n' if after else '')+new+s[at:].lstrip('\n'),encoding='utf-8')


def main():
    sources={};sections=[];rows=[];teachers=[]
    def get(path):sources[path.relative_to(OUT).as_posix()]=sha(path);return read(path)
    failure=get(FOLDER/'failure_checkpoints/review.json');contact=get(FOLDER/'contact_diagnosis/review.json');geometry=get(FOLDER/'encoder_clearance/review.json');adapter=get(FOLDER/'observation_adapter_verification.json');gates=get(FOLDER/'runtime_gate_verification.json');parity=get(FOLDER/'encoder_pose_feedback_v1_complete_bundle/shadow_fixture_verification.json')
    sections.append('<section><h2>이번 범위와 이미 된 동작</h2><p>초기 배치부터5089목표의 두 회전·두 유지는 기존 명목 모델에서 통과했다. 이 성공을 미완료로 되돌리지 않는다. 사용자와 정리한 남은 범위는 회전 중 미끄러짐, 시작 배치 차이, 실물 보정 입력·실행기 연결이다. 첫 유지 위치에서7.4mm 옮겨진 문제는 왼손 대응 후보로 분리한다. 원래 같은 위치2mm 검사 미달은 보존하되 오른손 단독 시험의 필수 복귀 조건으로 다시 올리지 않는다. 왼손 도달·간섭·접촉 하중은 통합 단계에서 확인한다.</p></section>')
    sections.append('<section><h2>실패 원인을 정확히 재현</h2><p>1mm 배치 변경은1835목표에서 중지 베이스와 끝의 모델 겹침으로 중단됐다. 깊이 약0.000183mm이며 실제 손의 측정값은 아니다. CCD50/100/200과 독립 볼록 형상 계산이 일치하므로 반복 횟수만 늘려 해결하지 않는다. 가정 마찰0.25는2272목표에서 큐브80mm 이동으로 중단됐다. 두 실패 직전의 전체 동역학 상태와 지령을 보존했다.</p>'+link(FOLDER/'failure_checkpoints/review.json','재현과 실제 중간 상태')+' · '+link(FOLDER/'contact_diagnosis/review.json','접촉 수치 진단')+'</section>')
    sections.append('<section><h2>기존 SAC를 관측과 함께 다시 사용</h2><p>이번 앞1700목표는 기존110차원 기준 궤적+SAC 보정 묶음을 새 관측으로 실행했다. 명목 조건의 지령은 기존 기록과 차이0이었다. 배치·마찰 변경에서 앞부분 지령은 달라졌지만, 뒤쪽 교사 경로를 그대로 붙인 두 조건은 계속 실패했다. 원본108차원 정책 재사용과 구분한다. 이번 새 SAC 학습은0스텝이며, 기존 누적90000스텝 예산 기록도 변경하지 않았다.</p></section>')
    for name in BATCHES:
        path=FOLDER/name/'review.json'
        if not path.exists():continue
        batch=get(path)
        for r in batch.get('rows',[]):
            holds=r.get('holds',[])
            row=dict(controller=name,case=r['case'],path=r['path'],steps=r['steps'],stop=r['stop'],passed=bool(r.get('two_turn_reached_location_holds_passed',False)),final180_error_deg=r.get('final180_error_deg',holds[1]['orientation_error_deg'] if len(holds)>1 else None),first90_error_deg=r.get('first90_error_deg',holds[0]['orientation_error_deg'] if holds else None),first_hold_passed=r.get('first_hold_passed'),second_hold_passed=r.get('second_hold_passed'),hold_samples=r.get('hold_samples'),hold_upper_certified_samples=r.get('hold_upper_certified_samples'))
            rows.append(row)
        table=''.join('<tr><td>'+html.escape(r['case'])+'</td><td>'+str(r['steps'])+'</td><td>'+str(r['first90_error_deg'])+'</td><td>'+str(r['final180_error_deg'])+'</td><td>'+('통과' if r['passed'] else html.escape(str(r['stop']) if r['stop'] else '회전 자세·유지 기준 미달'))+'</td></tr>' for r in rows if r['controller']==name)
        sections.append('<details><summary>'+html.escape(name)+'</summary><table><tr><th>조건</th><th>실행 목표</th><th>첫90도 오차</th><th>최종180도 오차</th><th>도달 위치 두 유지 결과</th></tr>'+table+'</table>'+link(path,'전체 원본')+'</details>')
    sections.append('<section><h2>모터 각도 기반 여유 보정과 큐브 자세 보정</h2><p>중지 형상 계산을 NumPy로 분리했고140자세의 위치·회전·여유가 MuJoCo와 일치했다. 후보 지령을 고를 때 중지 여유를 만들거나 손끝 경로 변화를 줄이는 방법을 비교했다. 일부 조건은 최초 접촉 중단을 피했지만 최종 회전이 미달하거나 이후 이탈해 전체 해결로 세지 않는다. 위치·자세 오차에 대한37개 국소 반응 계수도 계산했으나, 해당 선형 피드백의 배치·마찰 조건은 실제 모델 검사에서 실패했다. 국소 반응 식별과 정적 여유 계산을 SAC 학습, 동적 충돌 인증 또는 실물 힘 센서로 설명하지 않는다.</p>'+link(FOLDER/'encoder_clearance/review.json','140자세 계산 비교')+' · '+link(FOLDER/'local_pose_response/review.json','37개 국소 반응 식별')+'</section>')
    connection=get(FOLDER/'low_friction_teacher_cycle_connection_v4/review.json');kept=next(r for r in connection['rows'] if r['case']=='keep_teacher_pose')
    assert kept['first_hold_passed'] and not kept['second_hold_passed'] and kept['holds'][0]['samples']==4050 and kept['holds'][0]['upper_certified_samples']==4050 and kept['holds'][0]['supported_samples']==4050
    extracted=get(FOLDER/'low_friction_first_hold_state_v4/review.json')
    assert extracted['replayed_goal_count']==2478 and extracted['trajectory_qpos_difference']==0
    sections.append('<section><h2>낮은 마찰의 첫 유지와 다음 회전</h2><p>마찰0.25 가정에서 새 모델 교사를 연결한 첫90도 유지가4050/4050 샘플의 손 지지와 윗단 여유를 통과했다. 첫 방향 오차는'+f"{kept['holds'][0]['orientation_error_deg']:.3f}"+'도이고 실제 동역학 상태를 다시 놓지 않았다. 첫 유지 직후2478목표의 전체 배치 재생 상태를 저장 궤적과 정확히 일치하게 추출했다. 기존 두 번째 회전 지령을 그대로 붙인 경우 최종180도 방향 오차는'+f"{kept['holds'][1]['orientation_error_deg']:.3f}"+'도였다. 충돌 중단은 없었지만 두 번째 회전은 통과하지 않았다. 이는 기존 교사 지령의 연결 결과이며 새로운 SAC 정책의 성능으로 세지 않는다.</p>'+link(FOLDER/'low_friction_teacher_cycle_connection_v4/review.json','세 가지 연결 비교')+' · '+link(FOLDER/'low_friction_first_hold_state_v4/review.json','다음 계획의 정확한 시작 상태')+'</section>')
    loaded=get(FOLDER/'low_friction_next_turn_contact_diagnosis_v4/review.json')
    assert all(set(r['normal_load_by_prefix_N'])=={'if'} for r in loaded['rows'])
    sections.append('<section><h2>지지와 회전력의 차이</h2><p>저장된 동역학 상태를 모델에서 복원해 접촉을 다시 조회하자, 첫 유지와 실패한 두 번째 유지 모두 검지 베이스 부근의 하중만 나왔다. 엄지·중지·약지의 하중 접촉은 이 두 상태에서 확인되지 않았다. 손이 큐브 무게를 지탱한 기준 통과와 큐브를 다음 방향으로 돌릴 파지력 확보는 별개로 기록한다. 복원 상태의 모델 조회이며 실물 힘 측정은 아니다.</p>'+link(FOLDER/'low_friction_next_turn_contact_diagnosis_v4/review.json','두 상태의 접촉 비교')+'</section>')
    for name in TEACHERS:
        path=BASE/name/'review.json';done=path.exists()
        if not done:
            live=BASE/name/'progress.json'
            if not live.exists():continue
            path=FOLDER/(name+'_snapshot.json');path.write_bytes(live.read_bytes())
        r=get(path);end=r['final'] if done else r['actual'];t=dict(name=name,completed=done,steps=r['steps'] if done else r['step'],orientation_error_deg=end['error_deg'],position_target_error_mm=end['center_target_error_mm'],passed=bool(done and r['conditional_bridge_and_hold_passed']),path=path.relative_to(OUT).as_posix());teachers.append(t)
        sections.append('<section><h2>'+html.escape(name)+'</h2><p>'+('완료한 유한 모델 검사' if done else '진행 중인 계산의 고정 스냅샷')+f": {t['steps']}목표, 첫90도 오차 {t['orientation_error_deg']:.3f}도, 해당 위치 목표와 {t['position_target_error_mm']:.3f}mm 차이, 원래 위치·공간·유지 조건 통과={t['passed']}. "+'마찰0.25를 명시한 실제 도달 상태부터 재계획했다. 정확한 모델 정보를 쓰는 교사이며75ms 실물 제어로 배포된 것은 아니다.</p>'+link(path,'교사 원본')+'</section>')
    waypoint=get(FOLDER/'low_friction_loaded_waypoint_hold_v1/review.json')
    diagnosis=get(FOLDER/'low_friction_waypoint_failure_diagnosis_v1/review.json')
    reserve=get(FOLDER/'low_friction_waypoint_joint_reserve_v1/review.json')
    chosen=next(r for r in reserve['rows'] if r['ID6_reserve_ticks']==1)
    assert chosen['loaded_MF_TH_hold_passed'] and chosen['hold_samples']==4050 and chosen['supported_samples']==4050
    excess=diagnosis['rows'][1]
    waypoint_summary=f" 낮은 마찰의 다음 회전 중368목표의 중간 파지는 ID6 실제 각도의 상한 초과{excess['largest_signed_range_excess_rad']*180/3.141592653589793:.8f}도로1394샘플에서 중단됐다. ID6 지령을1틱 안쪽으로 두자4050샘플 지지와 중지·엄지 하중 유지가 통과했고, 최대 이동{chosen['maximum_shift_mm']:.3f}mm·흔들림{chosen['maximum_drift_deg']:.3f}도였다. 윗단 공간은 중간 회전에서 필수로 요구하지 않으며 최종 유지에서 검사한다."
    sections.append('<section><h2>관절 여유를 둔 안정 중간 파지</h2><p>'+html.escape(waypoint_summary)+' 손끝 윗단 여유의 음수 하한은 간섭 깊이 측정값이 아니다. 두 번째90도 완료를 뜻하지 않는다.</p>'+link(FOLDER/'low_friction_waypoint_joint_reserve_v1/review.json','0·1·2틱 여유 비교')+' · '+link(FOLDER/'low_friction_waypoint_failure_diagnosis_v1/review.json','관절·형상 진단')+'</section>')
    next_summary='';next_turns=[]
    for name in ('robust_mu025_next_turn_teacher_v1','robust_mu025_next_turn_teacher_v2','robust_mu025_next_turn_teacher_v3'):
        next_path=BASE/name/'review.json'
        if not next_path.exists():continue
        n=get(next_path)
        assert n['mode']=='next' and n['global_yaw_target_deg']==180 and n['sliding_friction_hypothesis']==.25
        trace=n['trace'];both=sum(r['contact_finger_normals_N'].get('mf',0)>.1 and r['contact_finger_normals_N'].get('th',0)>.1 for r in trace)
        entry=dict(name=name,steps=n['steps'],error_deg=n['final']['error_deg'],stop=n['stop'],both_middle_thumb_loaded_boundaries=both,control_boundaries=len(trace),passed=n['conditional_bridge_and_hold_passed']);next_turns.append(entry)
        next_summary+=f" {name}: {n['steps']}목표, 최종180도 오차{n['final']['error_deg']:.3f}도, 중지·엄지 동시 하중 접촉은 제어 경계{both}/{len(trace)}개, 유지 통과={n['conditional_bridge_and_hold_passed']}, 종료 사유{n['stop']}."
        detail=f"실제 첫 유지 이후의 별도 모델 교사. {n['steps']}목표, 최종180도 오차 {n['final']['error_deg']:.3f}도, 중지·엄지 동시 하중 접촉 {both}/{len(trace)}개 제어 경계, 유지 통과={n['conditional_bridge_and_hold_passed']}. 이어 붙인 전체 두 회전 검증은 별도로 필요하다."
        sections.append('<section><h2>'+html.escape(name)+'</h2><p>'+html.escape(detail)+'</p>'+link(next_path,'다음 회전 교사 원본')+'</section>')
    sections.append('<section><h2>실물 연결부의 무출력 검증</h2><p>'+f"Windows에서 모델 관측 지령 {parity['goal_comparisons']}개를 무출력 묶음으로 재현했고 차이는0이었다. "+'기존 실행기의 전체 건강 상태 검사를 받은 위치·목표·추종값을 관측 어댑터에 연결하는 인터페이스를 만들었다. 합성 카메라/마커 좌표128조건, 오래된 관측·좌표계·목표 불일치·중복 입력·응답 확인을 검사했다. 마지막 목표는 유지하며 자동 OPEN/OFF를 넣지 않았다. 기존450mA·PID·최대2틱·75ms와 전압/온도/전류/추종/통신 중단 절차는 변경하지 않았다.</p>'+link(FOLDER/'encoder_pose_feedback_v1_complete_bundle/shadow_fixture_verification.json','15534지령 Windows 재현')+' · '+link(FOLDER/'observation_adapter_verification.json','좌표·관측 검사')+' · '+link(FOLDER/'runtime_gate_verification.json','실행 경계 검사')+'</section>')
    sections.append('<section><h2>아직 안 된 것</h2><p>배치1mm와 마찰0.25에서 같은 실행기가 두 회전·유지를 통과한 결과는 아직 없다. 실제 카메라와 정책 좌표의 정합도 확인되지 않았다. 저장된 카메라 내부 보정 후보와 대략적인 마커 장착 치수만으로 정합을 완료했다고 하지 않는다. 현재 묶음은 등록 정보가 없으면 실물 관측 입력을 거부한다. 새 모터 실행 명령은 열지 않았으며 기존 물리 명령170개 결합 해시는 유지한다.</p></section>')
    report=dict(status='robust_cycle_no_output_preparation_in_progress',source_reports_sha256=sources,batches=rows,teachers=teachers,next_turn_teachers=next_turns,intermediate_loaded_hold=dict(ID6_command_reserve_ticks=1,samples=chosen['hold_samples'],maximum_shift_mm=chosen['maximum_shift_mm'],maximum_drift_deg=chosen['maximum_drift_deg'],passed=True,not_second_turn_completion=True),software_verification=dict(FK_states=geometry['fixture_states'],observer_transform_fixtures=128,runtime_gate_tests=len(gates['tests']),Windows_goal_comparisons=parity['goal_comparisons'],Windows_goal_mismatches=0),user_deferred_7p4mm_position_shift_to_left_hand_integration=True,original2mm_miss_preserved_but_not_added_as_standalone_prerequisite=True,remaining_work=dict(slip_robustness='not_passed',placement_robustness='not_passed',no_output_runtime='implemented_and_fixture_verified',physical_camera_registration='unverified'),new_SAC_training_steps=0,original_execution_limits_and_guards_preserved=True,devices_opened=0,remote_writes=0,hardware_execution_allowed=False)
    (FOLDER/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    title='오차 대응과 실물 무출력 준비 — 2026-11-11';page='<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+title+'</title><style>body{max-width:1100px;margin:30px auto;padding:0 18px;background:#f5f8fa;color:#203d46;font:16px/1.7 system-ui}section,details{padding:18px;margin:18px 0;background:white;border:1px solid #d4dfe2}.note{background:#fff0ce;padding:18px}table{border-collapse:collapse;width:100%;font-size:14px}td,th{border-bottom:1px solid #ddd;padding:8px}a{color:#087b73}</style><body><h1>'+title+'</h1><p class="note">명목 모델의 두 회전·유지는 완료했다. 오차 대응은 검사 중이며 실물 준비는 아직 미완료다. 관측·실행 연결부는 무출력으로 검증했고 실제 장치는 사용하지 않았다.</p>'+''.join(sections)+'<p>'+link(FOLDER/'review.json','출처와 범위')+' · '+link(FOLDER/'STATUS.md','인계 상태')+'</p></body></html>'
    (FOLDER/'index.html').write_text(page,encoding='utf-8')
    summary='2026-11-11 사용자 최신 승인: 통합 전 미끄러짐 대응·배치 차이 대응·실물 관측/실행기 무출력 연결을 진행한다. 7.4mm 위치 이동은 왼손 대응 후보로 분리했고 같은 위치2mm 복귀를 오른손 단독 시험의 선행 필수 문제로 다시 올리지 않는다. 기존5089목표 명목 두 회전·두 유지는 통과 상태를 유지한다. 기존110D SAC를 앞1700목표에서 실제 새 관측으로 재사용했으며 명목 지령 차이0, 오차 조건에서는 보정됐지만 전체 실패는 남았다. 마찰0.25 가정에서 모델 교사 경로의 첫90도 회전 후4050샘플 손 지지·윗단 여유는 통과했다. 이 상태는 검지 베이스의 지지 하중만 확인되고 엄지·중지·약지의 하중 접촉은 없으므로 다음 회전용 파지로 검증된 것은 아니다. 같은 상태에 기존 두 번째 회전 지령을 붙인 결과 최종180도 오차87.496도로 실패했다. 배치1mm와 낮은 마찰의 두 회전·두 유지 전체 성공은 아직 없다. 중지의0.000183mm 모델 겹침은 CCD50/100/200과 독립 볼록 형상에서 일치했다. 모터각 여유 보정·손끝 경로 비용·국소 자세 피드백을 모델에서 검사했으나 전체 성공은 없다. 새SAC 학습0이며 국소 반응 식별37조건은 SAC/증류로 세지 않는다. 무출력 실행기의15534개 지령이 Windows에서 차이0, 정적 FK140자세와 합성 좌표128조건 및 실행 경계13검사를 통과했다. 실제 카메라 좌표 정합은 미검증이라 실물 관측 입력은 현재 차단된다. 기존450mA/PID/2틱/75ms·전류/전압/추종/통신 중단 기준과 원본170결합 파일은 변경하지 않았고 실제 장치·원격 쓰기0이다. 모델 교사를 실물 제어 또는 완료로 세지 않는다. 최신 근거: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111/index.html`.'
    summary+=waypoint_summary+next_summary
    (FOLDER/'STATUS.md').write_text('# '+title+'\n\n'+summary+'\n\n'+ '\n'.join(f"- {t['name']}: 완료={t['completed']}, {t['steps']}목표,90도 오차{t['orientation_error_deg']:.3f}도, 위치 차이{t['position_target_error_mm']:.3f}mm, 전체 유지 기준 통과={t['passed']}." for t in teachers)+'\n\n실물 전체 경로는 아직 실행하지 않는다. 기존 실행 종료의 토크OFF는 정상 처리이며 별도 고장으로 취급하지 않는다.\n',encoding='utf-8')
    block(ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',summary,after='## 지금 먼저 할 일');block(ROOT/'professor_progress_20261015.md',summary)
    for target in (BASE/'index.html',BASE/'first_to_next_contact_bridge_20261111/index.html',OUT.parent/'policy_audit/explanation/index.html'):
        s=target.read_text(encoding='utf-8');begin='<!-- robust_cycle_latest_link -->';end='<!-- robust_cycle_latest_link_end -->'
        if begin in s:a=s.index(begin);b=s.index(end,a)+len(end);s=s[:a]+s[b:]
        href=os.path.relpath(FOLDER/'index.html',target.parent).replace('\\','/');banner=begin+'<aside style="padding:16px;background:#fff0ce"><a href="'+href+'">최신: 오차 대응과 실물 무출력 실행 준비</a></aside>'+end;at=s.find('>',s.lower().find('<body'))+1;target.write_text(s[:at]+banner+s[at:],encoding='utf-8')
    readiness=OUT/'next_cycle/readiness.json';r=read(readiness);r['robust_cycle_preparation']=dict(path=(FOLDER/'review.json').relative_to(OUT).as_posix(),sha256=sha(FOLDER/'review.json'),user_deferred_position_shift=True,software_fixture_verified=True,slip_and_placement_robustness_passed=False,camera_registration_verified=False,hardware_execution_allowed=False);readiness.write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(status=report['status'],native_comparisons=len(rows),teachers=len(teachers),Windows_goal_comparisons=parity['goal_comparisons'],hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
