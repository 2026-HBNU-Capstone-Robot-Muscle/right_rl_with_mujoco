"""Publish whole-hand support and continuous first-to-next-turn preparation."""
from pathlib import Path
import json
import hashlib
import html
import os
import numpy as np

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'first_to_next_contact_bridge_20261111'
CASES=('first90_loaded_MFTH_entry1000','first90_loaded_MFTH_entry1700','first90_loaded_cp152_retained_hybrid','actual_first_A_reconfigured_next90','actual_first_A_support_transfer_next90')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def relative(p):return p.relative_to(OUT).as_posix()
def link(p,label):return '<a href="../'+p.relative_to(BASE).as_posix()+'">'+html.escape(label)+'</a>'
def block(path,body,after=None):
    a='<!-- whole_hand_cycle_progress_20261111 -->';b='<!-- whole_hand_cycle_progress_20261111_end -->';s=path.read_text(encoding='utf-8')
    if a in s:
        start=s.index(a);end=s.index(b,start)+len(b);s=s[:start]+s[end:].lstrip('\n')
    new=a+'\n'+body+'\n'+b+'\n\n'
    if after:
        index=s.index(after)+len(after);s=s[:index]+'\n\n'+new+s[index:].lstrip('\n')
    else:s=new+s
    path.write_text(s,encoding='utf-8')


def latest_link(path):
    start='<!-- whole_hand_latest_link -->';end='<!-- whole_hand_latest_link_end -->';s=path.read_text(encoding='utf-8')
    if start in s:
        a=s.index(start);b=s.index(end,a)+len(end);s=s[:a]+s[b:]
    target=os.path.relpath(FOLDER/'index.html',path.parent).replace('\\','/')
    banner=start+'<aside style="padding:16px;background:#fff0cd;color:#173d40;margin:16px"><a href="'+target+'">2026-11-11 최신 검토: 손 전체 지지와 초기 배치부터 다음 회전 연결</a> — 특정 손가락 접촉은 바뀔 수 있으며 전체 실물 준비는 미완료입니다.</aside>'+end
    body=s.lower().find('<body');at=s.find('>',body)+1 if body>=0 else 0
    path.write_text(s[:at]+banner+s[at:],encoding='utf-8')


def main():
    FOLDER.mkdir(exist_ok=True);sources={};parts=[]
    def get(p):sources[relative(p)]=sha(p);return read(p)
    audit=get(FOLDER/'audit.json');first=get(BASE/'initial_to_reconfigured_first90/review.json');root=get(BASE/'whole_hand_support_transfer_20261111/review.json');relay=get(BASE/'whole_hand_contact_relay_20261111/review.json');reverse=get(BASE/'reached_first90_reverse_quarter_probe/review.json');actor=get(BASE/'actual_first_A_original108_reuse/review.json')
    A=next(r for r in audit['rows'] if r['label']=='reached_first90');B=next(r for r in audit['rows'] if r['label']=='successful_next90_start_initialized')
    parts.append('<section><h2>접촉은 바뀌어도 된다</h2><p>사용자 확인: 같은 중지·엄지 접촉을 계속 유지할 필요는 없다. 큐브의 지지를 손바닥·뿌리·마디·다른 손가락으로 넘길 수 있다. 기존 손 전체 지지와 관절·추종·충돌 검사는 유지한다. 특정 손가락 하중을 계속 요구한 검사는 한 비교 조건이며 전체 작업의 필수조건이 아니다.</p></section>')
    parts.append('<section><h2>서로 다른 두 시작 상태를 확인</h2><p>'+f"실제로 도달한 첫 회전 끝에서는 검지 계열 약 {A['finger_normals_N']['if']:.3f}N이 지지했고 중지·엄지 하중은0이었다. 이것도 손의 지지로 센다. 별도로 배치한 다음 회전 시작 파지는 중지·엄지 하중이 각각 {B['finger_normals_N']['mf']:.3f}/{B['finger_normals_N']['th']:.3f}N이고 두 위치는 {A['distance_to_next_start_mm']:.3f}mm 다르다. "+'후자의 하중 구성을 모든 경로에서 유지해야 한다는 뜻은 아니다.</p>'+link(FOLDER/'audit.json','실제 중간·끝 상태 비교')+'</section>')
    parts.append('<section><h2>초기 배치부터 새 첫 회전 출발 자세까지 연속 재생</h2><p>'+f"총 {first['steps']}목표: 기존 첫 회전3015 → 같은 위치에서 손 자세 변경336 → 유지54. 세 이음의 모든 저장 동역학 상태 차이가0이고 상태 재설정은 없다. 마지막4050개1ms 상태에서 손 전체 지지100%, 윗단 인증 하한 {first['minimum_upper_lower_mm']:.3f}mm,90도 오차 {first['final90_error_deg']:.3f}도, 기존 도달 위치와 {first['reference_A_center_error_mm']:.3f}mm 차이다. "+'옛 canonical 위치 미달 기록과 실물 미검증 상태는 유지한다.</p>'+link(BASE/'initial_to_reconfigured_first90/review.json','3405목표 독립 재생')+'</section>')
    root_pass=sum(r['intermediate_whole_hand_supported'] for r in root['rows']);relay_pass=sum(r['intermediate_contact_relay_passed'] for r in relay['rows'])
    table=''
    for r in root['rows']:
        if r.get('skipped'):continue
        table+=f"<tr><td>{r['requested_root_change_deg']:+g}도</td><td>{r['steps']}</td><td>{r['whole_hand_support_fraction']*100:.1f}%</td><td>{r['global90_orientation_error_deg']:.3f}도</td><td>{'중간 지지 통과' if r['intermediate_whole_hand_supported'] else html.escape(str(r['stop']))}</td></tr>"
    parts.append('<section><h2>검지·손바닥 지지로 넘기는 중간 동작</h2><p>'+f"검지 뿌리 ID8만 바꾸고 나머지15개 지령을 그대로 유지한 {len(root['rows'])}조건 중 {root_pass}조건이2250상태 중간 지지 검사를 통과했다. "+'일부 조건에서는 엄지가 새로 닿았다. 중간 기울기는 최종 유지 오차 통과로 세지 않는다.−18도 조건의80mm 이동 중단은 보존했다.</p><table><tr><th>ID8 변화</th><th>목표 수</th><th>손 전체 지지</th><th>90도 오차</th><th>중간 결과</th></tr>'+table+'</table>'+link(BASE/'whole_hand_support_transfer_20261111/review.json','여섯 조건 원본')+'</section>')
    parts.append('<section><h2>접촉을 얻은 뒤 검지를 되돌리는 지지 전환</h2><p>'+f"{len(relay['rows'])}조건 중 {relay_pass}조건에서 다른 접촉을 얻은 후 검지 뿌리를 되돌리고4050상태의 손 전체 지지100%를 확인했다. "+'도중 접촉 획득은 단계 전환 신호다. 끝에서 엄지 하중이0이 되어도 손 전체가 받치면 지지로 센다. 이 결과는 중간 지지 전환이며 최종90도 자세·윗단 공간·다음90도 통과는 아니다. 정확한 모델 거리·힘을 쓴 교사이며 실제 센서 제어가 아니다.</p>'+link(BASE/'whole_hand_contact_relay_20261111/review.json','접촉 전환 원본')+'</section>')
    studies=[]
    for name in CASES:
        p=BASE/name/'review.json';done=p.exists()
        if not done:p=BASE/name/'progress.json'
        if not p.exists():continue
        r=get(p);e=r['final'] if done else r['actual'];steps=r['steps'] if done else r['step'];is_next=name.startswith('actual_first_A_')
        row=dict(name=name,path=relative(p),completed=done,steps=steps,global_target_deg=180 if is_next else 90,orientation_error_deg=e['error_deg'],reference_station_error_mm=e['center_target_error_mm'],contact_finger_normals_N=e['contact_finger_normals_N'],passed=bool(done and r['conditional_bridge_and_hold_passed']),specific_finger_retention_is_comparison_only=name=='first90_loaded_cp152_retained_hybrid')
        if done:row.update(stop=r['stop'],whole_hand_support_at_end=e['whole_hand_support']['supported_sample'],contact_invariant_requested=r.get('retain_loaded_contacts_at_control_boundaries',False))
        studies.append(row)
        parts.append('<section><h2>'+html.escape(name)+'</h2><p>'+('완료한 비교' if done else '진행 중인 스냅샷')+f": {steps}목표, {row['global_target_deg']}도 목표 오차 {e['error_deg']:.3f}도, 미리 정한 위치와 {e['center_target_error_mm']:.3f}mm 차이. "+('회전·유지 통과.' if row['passed'] else '완료된 회전·유지로 세지 않는다.')+(' 특정 중지·엄지 접촉을 유지한 제한적 비교이며 전체 작업 필수조건이 아니다.' if row['specific_finger_retention_is_comparison_only'] else ' 손 전체 지지로 판정하며 특정 손가락의 지속 접촉은 강제하지 않았다.')+'</p>'+link(p,'모델 원본')+'</section>')
    chain=get(BASE/'initial_through_next_MF_contact_gait/review.json');reset=get(BASE/'actual_A_next_MF_reset376/review.json');pressure=get(BASE/'actual_A_next_MF_yaw_pressure/review.json');extended=get(BASE/'actual_A_next_MF_yaw_pressure_continued/review.json');thumb=get(BASE/'actual_A_next_counter_thumb_arrival/review.json');ring=get(BASE/'actual_A_next_counter_ring_geometry/review.json')
    extension=extended['rows'][0];safe=pressure['rows'][0]
    parts.insert(1,'<section><h2>초기 배치에서 다음 회전 일부까지4673목표 연결</h2><p>'+f"첫 회전3405 → 다음 회전 중간376 → 중지 재배치201 → 새 접촉점 이동191 → 중지 보정500을 초기화 없이 재현했다. 다섯 이음에서 qpos/qvel/ctrl/act/qacc_warmstart 차이0, 전체1ms 상태의 손 전체 지지 비율 {chain['whole_hand_support_fraction']*100:.4f}%, 두 번째 회전의 추가 수평 회전 {chain['additional_next_yaw_deg']:.3f}도다. 최종180도 자세 오차 {chain['final180_error_deg']:.3f}도, 첫 유지 위치와 {chain['reference_first_station_error_mm']:.3f}mm 차이로 아직 미완료다. "+'첫 유지4050상태의 공간 인증은 통과했지만 마지막 자세의 윗단 공간은 미인증이다. 음수 분리 하한을 실제 침투 깊이라고 해석하지 않는다.</p>'+link(BASE/'initial_through_next_MF_contact_gait/review.json','전체4673목표 독립 재생 원본')+'</section>')
    parts.append('<section><h2>중지는 접촉을 풀고 다시 배치할 수 있다</h2><p>다음 회전 중간의 중지 여섯 재배치가 모두 손 전체 지지를 유지했다. 다른12개 지령은 그대로였고2250개 상태의 지지100%, 큐브 변화는0.22mm·0.22도 미만이었다. 중지 PIP 여유는 약20~51도로 늘었다. 손가락 접촉 고정 때문에 막힌 것으로 처리하지 않는다.</p>'+link(BASE/'actual_A_next_MF_reset376/review.json','중지 재배치 여섯 조건')+'</section>')
    parts.append('<section><h2>중지만 계속 미는 방식의 한계를 실제 동역학에서 확인</h2><p>'+f"새 접촉점에서500목표를 보정해 {safe['yaw_advance_deg']:.3f}도 더 진행했다. 압박 설정 세 개는 실행 속도 제한 때문에 동일한 지령 경로를 만들었으며 독립된 세 성공으로 세지 않는다. 이를753목표로 연장한 비교는 앞500목표와 모든 저장 상태가 정확히 같았다. 오차가 한때 {extension['best180_error_deg']:.3f}도까지 줄었지만 이후 큐브가 옆으로 밀려 원래80mm 이동 중단 기준에 걸렸다. "+'이19.8도는 안정된 최종 결과가 아니다. 물체 이동 실패이며 실제 로봇 낙하 기록이나 모터 고장은 아니다.</p>'+link(BASE/'actual_A_next_MF_yaw_pressure/review.json','500목표 보정')+' · '+link(BASE/'actual_A_next_MF_yaw_pressure_continued/review.json','연장 실패와500목표 일치 근거')+'</section>')
    thumb_supported=sum(not r.get('stop') for r in thumb['rows'])
    parts.append('<section><h2>엄지와 약지의 추가 받침 후보</h2><p>'+f"엄지 후보까지 실제 자유 물체를 움직이는 여섯 조건 중 {thumb_supported}조건은4050상태의 손 전체 지지100%를 유지했다. 다른 세 경로는80mm 이동에서 중단됐으며 실패를 보존했다. 약지 끝으로 네 면에 양의 회전 지렛팔 접촉을 찾은 제한적 탐색에서는 후보0개였다. "+'손가락 마디 접촉, 다른 큐브 위치, 다른 중간 경로까지 불가능하다는 뜻은 아니다. 정적 IK에서만 큐브를 고정했고 동적 경로의 큐브는 자유 물체다.</p>'+link(BASE/'actual_A_next_counter_thumb_arrival/review.json','엄지 동적 이동')+' · '+link(BASE/'actual_A_next_counter_ring_geometry/review.json','약지의 제한된 정적 탐색')+'</section>')
    teacher_folder=BASE/'actual_A_next_two_contact_centered_teacher';teacher_path=teacher_folder/'review.json';teacher_done=teacher_path.exists();teacher_result=None
    if not teacher_done:teacher_path=teacher_folder/'progress.json'
    if teacher_path.exists():
        if not teacher_done:
            # A live progress file changes while native computation runs. Bind
            # publication to a frozen snapshot, never a mutable file hash.
            snapshot=FOLDER/'teacher_progress_snapshot.json';snapshot.write_bytes(teacher_path.read_bytes());teacher_path=snapshot
        teacher=get(teacher_path);e=teacher['final'] if teacher_done else teacher['actual'];teacher_result=dict(completed=teacher_done,steps=teacher['steps'] if teacher_done else teacher['step'],orientation_error_deg=e['error_deg'],first_station_error_mm=e['center_target_error_mm'],task_passed=bool(teacher_done and teacher['conditional_bridge_and_hold_passed']),path=relative(teacher_path))
        parts.append('<section><h2>엄지 재배치 후 회전과 위치를 함께 보정</h2><p>'+('완료한 모델 검사: ' if teacher_done else '진행 중인 계산의 스냅샷: ')+f"{teacher_result['steps']}목표,180도 자세 오차 {e['error_deg']:.3f}도, 첫 유지 위치와 {e['center_target_error_mm']:.3f}mm 차이. "+('이 조건의 자세·위치·4050상태 유지 및 공간 검사를 통과했다. 전체 초기 연결과 배치·구동 오차는 별도로 검증해야 한다.' if teacher_result['task_passed'] else '작은 자세 오차만으로 최종 공간·지지·위치·다음 회전 완료를 판정하지 않는다.')+'</p>'+link(teacher_path,'위치와 회전 보정 원본')+'</section>')
    clear_hold=None;replay=None;perturb=None
    hold_path=BASE/'actual_A_second_turn_clear_hold/review.json'
    if hold_path.exists():
        clear_hold=get(hold_path);h=next(r for r in clear_hold['rows'] if r['label']=='unchanged')
        parts.insert(1,'<section><h2>두 번째 회전 뒤 도달한 자리의 유지와 공간</h2><p>'+f"추가 손가락 이동 없이4050개1ms 상태에서 손 전체 지지 {h['whole_hand_hold_support_fraction']*100:.1f}%, 윗단 인증 {h['upper_certified_samples']}/4050, 최소 분리 하한 {h['upper_minimum_lower_mm']:.3f}mm를 확인했다. 최종 자세 오차 {h['final180_error_deg']:.3f}도, 기울기 {h['tilt_deg']:.3f}도, 유지 중 위치 변화 {h['hold_shift_mm']:.3f}mm·자세 변화 {h['hold_drift_deg']:.3f}도다. "+f"도달한 위치에서 회전·공간·유지 통과={h['rotation_and_clear_hold_at_reached_location_passed']}. 첫 유지 위치와 {h['first_station_error_mm']:.3f}mm 달라 기존2mm 위치 기준은 미달이다. "+'다른 손 자세로 되돌린 세 비교의 모델 접촉/물체 이동 실패도 보존했다. 같은 손가락 자세로 복귀하는 것은 필수조건이 아니다.</p>'+link(hold_path,'도달 위치 유지와 위치 복귀 비교')+'</section>')
    replay_path=BASE/'initial_second_turn_hold_replay/review.json'
    if replay_path.exists():
        replay=get(replay_path)
        parts.insert(1,'<section><h2>최신 독립 재생: 처음 배치 → 첫 회전·유지 → 다음 회전·유지</h2><p>'+f"총 {replay['steps']}목표, 명목 모델 시간 {replay['steps']*.075:.3f}초를 처음 배치부터 초기화 없이 재현했다. 네 이음의 전체 저장 동역학 상태 차이0이다. 두 유지 구간의1ms 상태를 각각4050개 다시 검사했다. 도달한 각 위치에서 두 회전·윗단 공간·유지 통과={replay['two_rotations_and_clear_holds_at_reached_locations_passed']}. 두 번째 실제 추가 수평 회전 {replay['additional_yaw_from_first_hold_deg']:.3f}도, 최종180도 오차 {replay['final180_error_deg']:.3f}도다. "+f"첫 위치와 {replay['first_station_error_mm']:.3f}mm 차이여서 같은 위치2mm 조건 통과={replay['same_first_station_two_turn_cycle_passed']}. "+'정확한 모델 피드백으로 만든 경로의 명목 재생이며 SAC 학습/증류 완료, 실제 센서 제어 또는 실물 성공은 아니다.</p>'+link(replay_path,'전체 두 회전·두 유지 독립 재생')+'</section>')
    perturb_path=BASE/'initial_two_turn_finite_perturbations/review.json'
    if perturb_path.exists():
        perturb=get(perturb_path);n=sum(r['two_rotation_clear_holds_at_reached_locations_passed'] for r in perturb['rows'])
        details=''.join('<li>'+html.escape(r['case'])+f": {r['executed_goals']}/{r['total_goals']}목표, 도달 위치 두 유지 통과={r['two_rotation_clear_holds_at_reached_locations_passed']}, 중단="+html.escape(str(r['stop']))+'</li>' for r in perturb['rows'])
        parts.insert(2,'<section><h2>작은 배치·구동·마찰 변화</h2><p>'+f"미리 저장한 같은 경로를 세 변화 조건에서 다시 실행해 {n}/3조건이 두 유지 검사를 통과했다. 각 조건은 처음에 한 번만 바꾸고 이후 큐브를 초기화하지 않았다. "+'마찰은 측정값이 아닌 가정이며 세 조건만으로 일반적인 강건성이나 실물 준비를 인증하지 않는다.</p><ul>'+details+'</ul>'+link(perturb_path,'유한 오차 조건 원본')+'</section>')
    recenter_path=BASE/'second_hold_IF_support_recentering/review.json'
    if recenter_path.exists():
        recenter=get(recenter_path)
        parts.insert(3,'<section><h2>첫 위치로 되돌리는 추가 검사</h2><p>검지의 지지면을 작은 범위에서 함께 옮긴 네 경로는 모두 손 전체 지지와 윗단 공간을4050상태 유지했다. 그러나 위치 차이는6.81~7.17mm로 남았고, 회전 자세까지 통과한 조건은 한 개뿐이었다. 단순히 받침을 이동하는 것만으로 기존2mm 복귀를 해결하지 못했다. 다른12개 지령은 그대로였고 기존 한계는 넓히지 않았다.</p>'+link(recenter_path,'지지면 이동 네 비교')+'</section>')
    parts.append('<section><h2>기존 학습 정책 재사용 비교</h2><p>초기 배치부터 연결 검증한 같은 출발 상태에서 원본108차원 actor를 두 이력 방식으로 비교했다. '+', '.join(str(r['steps'])+'목표' for r in actor['rows'])+'에서 모델 접촉 기준으로 중단됐고 다음90도·유지는 통과하지 않았다. 정책 가중치는 바꾸지 않았고 새 SAC 학습은0스텝이다. 현재 모델의 완전한 큐브 관측을 가정한 결과이며 정책 전체가 쓸모없다는 증명은 아니다.</p>'+link(BASE/'actual_first_A_original108_reuse/review.json','정책 재사용 원본')+'</section>')
    parts.append('<section><h2>반대 방향 다음 회전은 별도 비교</h2><p>같은 위치의 손 자세 변경과 첫90도 유지는 통과했다. 그 뒤 저장 경로를 거꾸로 재생하는 비교는 다음 방향을90→0도로 명시했으며 모델 접촉 기준에서 중단됐다. 같은 방향의 두90도나 전체 성공으로 세지 않는다.</p>'+link(BASE/'reached_first90_reverse_quarter_probe/review.json','방향이 명시된 실패 원본')+'</section>')
    visual=FOLDER/'visual_manifest.json'
    if visual.exists():
        v=get(visual);parts.insert(1,'<section><h2>저장 모델 상태 비교</h2><p>과학적 모델 렌더이며 실물 사진에 정밀 등록한 그림은 아니다.</p><div class="grid">'+''.join('<figure><img src="'+r['file']+'"><figcaption>'+html.escape(r['label'])+'</figcaption></figure>' for r in v['images'])+'</div></section>')
    report=dict(status='whole_hand_first_to_next_cycle_preparation',date='2026-11-11',source_reports_sha256=sources,first_reconfigured_replay_passed=first['first90_reconfigured_whole_hand_hold_passed'],first_reconfigured_replay_steps=first['steps'],support_transfer_conditions=len(root['rows']),support_transfer_passed=root_pass,contact_relay_conditions=len(relay['rows']),contact_relay_intermediates_passed=relay_pass,studies=studies,
        initial_through_partial_next_replay_steps=chain['steps'],initial_through_partial_next_yaw_deg=chain['additional_next_yaw_deg'],initial_through_partial_next_error_deg=chain['final180_error_deg'],pressure_settings_with_identical_commands=3,continued_MF_push_failed_with_object_translation=True,thumb_supported_arrival_conditions=thumb_supported,centered_teacher=teacher_result,
        nominal_two_turn_reached_locations_passed=bool(replay and replay['two_rotations_and_clear_holds_at_reached_locations_passed']),nominal_two_turn_replay_steps=replay['steps'] if replay else None,nominal_same_first_station_cycle_passed=bool(replay and replay['same_first_station_two_turn_cycle_passed']),finite_perturbation_conditions=len(perturb['rows']) if perturb else 0,finite_perturbation_passes=sum(r['two_rotation_clear_holds_at_reached_locations_passed'] for r in perturb['rows']) if perturb else 0,
        specific_finger_contact_not_a_global_invariant=True,palm_and_all_hand_parts_count_as_support=True,original_guards_and_joint_limits_preserved=True,full_initial_two_positive_quarter_cycle_passed=False,full_cycle_false_scope='Existing common-station2mm task and required robustness; nominal reached-location result is separate',old_canonical_failures_not_reclassified=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    (FOLDER/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    page='<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>손 전체 지지로 첫 회전과 다음 회전 연결</title><style>body{max-width:1180px;margin:28px auto;padding:0 18px;background:#f4f8f8;color:#173d40;font:16px/1.7 system-ui}section{background:white;padding:18px;margin:18px 0;border:1px solid #ccdcd9}.note{background:#fff0cd;padding:18px}.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:16px}figure{margin:0}img{width:100%}table{border-collapse:collapse;width:100%}td,th{padding:7px;border-bottom:1px solid #dce5e3}a{color:#08746b}@media(max-width:700px){.grid{grid-template-columns:1fr}}</style><body><h1>손 전체 지지 → 재파지 → 다음 회전</h1><p class="note">특정 손가락 접촉을 계속 유지하는 것은 필수조건이 아니다. 초기 배치부터 첫 회전·유지와 두 번째 회전 일부까지4673목표를 초기화 없이 재현했다. 다음90도·최종 위치·공간과 전체 동작의 실물 준비는 아직 완료하지 않았다.</p>'+''.join(parts)+'<p><a href="review.json">범위·출처 해시</a> · <a href="STATUS.md">인계 상태</a> · <a href="../MF_TH_interference_diagram_20261111/index.html">미세 모델 겹침 그림</a></p></body></html>'
    (FOLDER/'index.html').write_text(page,encoding='utf-8')
    if replay:
        old_note='초기 배치부터 첫 회전·유지와 두 번째 회전 일부까지4673목표를 초기화 없이 재현했다.'
        new_note=f"초기 배치부터 두 회전과 각 도달 위치의 유지까지 {replay['steps']}목표를 초기화 없이 재현했다. 도달 위치 유지 통과={replay['two_rotations_and_clear_holds_at_reached_locations_passed']}; 첫 위치2mm 복귀는 미달이다."
        (FOLDER/'index.html').write_text(page.replace(old_note,new_note),encoding='utf-8')
    for p in (BASE/'index.html',OUT.parent/'policy_audit/explanation/index.html',BASE/'ring_front_thumb_progress_20261111/index.html'):
        latest_link(p)
    summary='2026-11-11 최신 사용자 정정: 특정 중지·엄지 접촉을 계속 유지할 필요는 없다. 손바닥·뿌리·마디·다른 손가락이 지탱해도 손의 지지로 센다. 특정 두 손가락 하중 유지 검사는 제한적 비교이며 전체 동작 필수조건이 아니다. 초기 배치→기존 첫 회전3015목표→같은 위치에서 손 자세 변경336→유지54, 총3405목표를 상태 초기화 없이 독립 재생했다. 세 이음 전체 상태 차이0, 마지막4050상태 손 전체 지지100%·윗단 인증 하한0.874mm·90도 오차2.516도·기존 도달 위치와0.549mm 차이로 새 첫 회전 출발 자세를 확인했다. 기존 canonical 실패는 유지한다. 검지 뿌리를 움직여 지지를 넘기는6조건 중5조건과 추가 접촉 뒤 검지를 되돌리는2조건이 중간 지지를 통과했다. 중간 기울기와 접촉 획득을 최종90도·윗단 공간·다음90도 성공으로 세지 않는다. 같은 실제 출발 자세에서 다음180도 목표로 모델 교사를 검사하며, 원본108정책 두 비교도91/93목표에서 접촉 기준으로 중단됐다. 이번 새SAC 학습·장치 접근·원래 한계 변경은 없다. 전체 두 회전은 아직 미완료다. 최신 근거: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/first_to_next_contact_bridge_20261111/index.html`.'
    summary+=' 후속 검사: 중지를 풀어 재배치한 뒤 초기 배치부터4673목표를 독립 재생했다. 다섯 이음 동역학 상태 차이0, 두 번째 회전의 추가 수평 회전61.960도·최종180도 자세 오차30.582도·첫 유지 위치 차이7.184mm다. 마지막 윗단 공간과 위치 기준은 아직 미달이다. 중지 보정 세 압박 설정은 동일한500목표 경로였다. 이를753목표까지 밀면 오차가 한때19.847도까지 줄지만 큐브80mm 이동에서 중단된다. 모델 물체 이탈 실패이며 실제 로봇 낙하가 아니다. 엄지 동적 이동 여섯 조건 중 세 조건의4050상태 지지를 확인했고 약지 끝의 네 면 양의 회전 지렛팔 탐색에는 후보가 없었다. 제한된 탐색을 약지 일반 도달 불가능으로 확대 해석하지 않는다.'
    if teacher_result:summary+=f" 엄지 재배치 후 회전·위치 보정 교사: 완료={teacher_done}, {teacher_result['steps']}목표,180도 자세 오차{teacher_result['orientation_error_deg']:.3f}도·첫 유지 위치 차이{teacher_result['first_station_error_mm']:.3f}mm, 해당 유지 조건 통과={teacher_result['task_passed']}. 전체 초기 연결·오차 검증 및 실물 준비 완료는 아니다."
    if replay:summary+=f" 이후 독립 재생 최신 결과: 처음 배치부터 {replay['steps']}목표, 네 동역학 이음 차이0·초기화0, 첫/두 번째 유지 각4050상태를 다시 검사했다. 도달한 각 위치에서 두 회전·윗단 공간·유지 통과={replay['two_rotations_and_clear_holds_at_reached_locations_passed']}; 두 번째 추가 수평 회전{replay['additional_yaw_from_first_hold_deg']:.3f}도·최종180도 자세 오차{replay['final180_error_deg']:.3f}도다. 첫 유지 위치와{replay['first_station_error_mm']:.3f}mm 달라 같은 위치2mm 기준은 미달이다. 이 명목 모델 경로를 SAC 정책 완성이나 실물 준비 완료로 세지 않는다."
    if perturb:summary+=f" 같은 경로의 유한 오차 조건 {len(perturb['rows'])}개 중 도달 위치 두 유지 통과 {sum(r['two_rotation_clear_holds_at_reached_locations_passed'] for r in perturb['rows'])}개다. 실패는 보존하고 일반 강건성으로 확대하지 않는다."
    if replay:
        summary=('2026-11-11 최신: 사용자가 원한 오른손 전체 회전→윗단 공간을 비운 유지→다음 회전→유지를 명목 모델에서 처음 배치부터5089목표로 독립 재생했다. 상태 초기화0, 네 이음 qpos/qvel/ctrl/act/qacc_warmstart 차이0이며 두 유지 각4050개1ms 상태에서 손 전체 지지·윗단 공간을 통과했다. 첫90도 오차2.516도, 두 번째 추가 수평 회전91.454도·최종180도 자세 오차3.778도·기울기3.626도다. 마지막 유지의 윗단 인증 하한1.347mm, 위치 변화0.144mm·자세 변화0.099도다. 손바닥·뿌리·마디도 지지로 세며 특정 중지/엄지 접촉이나 동일한 손가락 자세로 복귀할 필요는 없다. '
                 '첫 유지 위치보다7.402mm 옮겨져 기존 같은 위치2mm 기준은 미달이다. 명목 경로의 두 회전·도달 위치 유지 성공과 원래 위치 복귀/강건성/실물 준비 완료를 구분한다. 검지 지지면 이동 네 조건도 위치 차이가6.81~7.17mm 남아2mm 복귀를 해결하지 못했다. '
                 '같은 저장 경로의 작은 오차 세 조건 중 중지/엄지 초기1틱 변화는 두 유지를 통과했다. 초기 큐브 worldX+1mm는1835목표에서 기존 모델 접촉 기준, 가정 마찰0.25는2272목표에서 큐브80mm 이동 기준에 걸려 첫 유지 전에 중단됐다. 이 결과는 저장 경로의 유한 조건 검사이며 새 폐루프 SAC 제어 시험이 아니다. 다음 우선순위는 작은 초기 배치·마찰 변화에 대응하는 제어와 위치 재현성이다. 이번 새SAC학습0, 실제 장치 접근0, 원본 한계 변경0이며 기존 학습 예산 기록은 유지한다. 사용자의 새 연결·촬영·실험 작업은 현재 없다. '
                 '최신 근거: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/first_to_next_contact_bridge_20261111/index.html`, `initial_second_turn_hold_replay/review.json`, `initial_two_turn_finite_perturbations/review.json`. 과거4673목표의30.582도 미달과 중지 단독 연장의 물체 이탈은 이전 비교 기록으로 보존하고 최신5089명목 결과를 미완료로 되돌리지 않는다. 실물 성공·SAC/증류 완료로도 세지 않는다.')
    block(ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',summary,after='## 지금 먼저 할 일');block(ROOT/'professor_progress_20261015.md',summary)
    text=['# 손 전체 지지로 첫 회전과 다음 회전 연결 — 2026-11-11','',summary,'','## 모델 검사 상태','']
    for r in studies:text.append(f"- {r['name']}: {r['steps']}목표, 목표 오차{r['orientation_error_deg']:.3f}도, 위치 차이{r['reference_station_error_mm']:.3f}mm, 완료={r['completed']}, 회전·유지 통과={r['passed']}.")
    text+=['','현재 사용자 연결·촬영·실물 실행 작업 없음. 다음 회전과 최종 공간·구동/배치 오차 검증이 남아 있다.','', '[그림과 원본](index.html)']
    (FOLDER/'STATUS.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
    ready_path=OUT/'next_cycle/readiness.json';ready=read(ready_path);ready['whole_hand_first_to_next_progress']=dict(path=relative(FOLDER/'review.json'),sha256=sha(FOLDER/'review.json'),finger_contact_identity_is_not_required=True,first3405_replay_passed=True,initial_through_partial_next_replay_steps=4673,full_two_turn_cycle_passed=False,hardware_execution_allowed=False);ready_path.write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
    ready['whole_hand_first_to_next_progress'].update(nominal_two_rotations_at_reached_locations_passed=report['nominal_two_turn_reached_locations_passed'],nominal_two_turn_replay_steps=report['nominal_two_turn_replay_steps'],same_first_station_cycle_passed=report['nominal_same_first_station_cycle_passed'],finite_perturbation_conditions=report['finite_perturbation_conditions'],finite_perturbation_passes=report['finite_perturbation_passes']);ready_path.write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(first_replay=first['steps'],support_transfer_passed=root_pass,relay_passed=relay_pass,pending=sum(not r['completed'] for r in studies),report=relative(FOLDER/'index.html'))),flush=True)


if __name__=='__main__':main()
