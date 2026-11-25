"""Publish separately scoped ring-front/thumb grip and next-turn evidence."""
from pathlib import Path
import json
import hashlib
import html
import os

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'ring_front_thumb_progress_20261111'


def read(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def link(p,label):return '<a href="../'+p.relative_to(BASE).as_posix()+'">'+html.escape(label)+'</a>'


def replace_block(path,body,after=None):
    start='<!-- ring_front_thumb_progress_20261111 -->';end='<!-- ring_front_thumb_progress_20261111_end -->';s=path.read_text(encoding='utf-8')
    if start in s:
        a=s.index(start);b=s.index(end,a)+len(end);s=s[:a]+s[b:].lstrip('\n')
    new=start+'\n'+body+'\n'+end+'\n\n'
    if after:
        i=s.index(after)+len(after);s=s[:i]+'\n\n'+new+s[i:].lstrip('\n')
    else:s=new+s
    path.write_text(s,encoding='utf-8')


def main():
    FOLDER.mkdir(exist_ok=True);sources={};sections=[]
    def get(p):
        sources[p.relative_to(OUT).as_posix()]=sha(p);return read(p)
    photo=get(BASE/'user_ring_photo_reference_20261110/review.json');assert photo['user_contact_clarification']['ring_contact_face']=='white_front_face'
    geom=get(BASE/'ring_thumb_photo_prior_seed431/review.json');exact=get(BASE/'ring_thumb_photo_prior_seed432_exact_static/review.json')
    parts=['<section><h2>사진에서 확인한 접촉과 모델 가설</h2><p>사용자가 약지는 흰색 앞면을 받친다고 확인했다. 모델 그림에서는 앞면을 로컬+X로 표시했다. 사진–카메라–손 좌표를 정밀 등록한 결과는 아니며 중지·엄지의 실제 각도도 아직 확정하지 않았다.</p>',
        '<p>기록된 약지 네 값은 그대로 보존했다. 정확한 네 값의 정적 기하 비교와 원래 구동 범위 안의 동역학 비교를 분리했다. 동역학 비교의 ID14 참고각은 기록보다10.877도 작다. 원래 상한을 바꾸지 않고 닫기 여유를 남긴 별도 모델 가설이며, 사진 자세와 같은 목표라고 부르지 않는다.</p>',
        link(BASE/'user_ring_photo_reference_20261110/index.html','사용자 사진·각도 출처')+'</section>']
    visual=get(FOLDER/'visual_manifest.json') if (FOLDER/'visual_manifest.json').exists() else None
    if visual:
        parts.append('<section><h2>약지 앞면–엄지 맞은편 가설의 모델 그림</h2><p>노랑 약지 · 빨강 엄지 · 초록 중지 · 파랑 검지. 큐브의 흰 앞면·빨간 윗면은 사용자의 면 설명에 맞춘 표시다. 카메라 시점을 사진에 정합한 것은 아니다. 분홍 원통은 윗단 회전 영역 표시다.</p><div class="grid">'+''.join('<figure><img src="'+r['file']+'"><figcaption>'+html.escape(r['label'])+'</figcaption></figure>' for r in visual['images'])+'</div></section>')
    cold=get(BASE/'ring_thumb_photo_prior_seed431/axis0_ring+1/free_hold_tests/review.json')
    cold08=next(r for r in cold['rows'] if r['preload_hypothesis_N']==.8)
    parts.append('<section><h2>처음 배치에서 이동한 값과 정착 후 유지를 구분</h2><p>'+f"기하 후보를 별도로 배치한 뒤 자유 동역학에서 큐브가 손바닥 쪽으로 {cold08['maximum_station_drift_mm']:.3f}mm 이동했다. 처음 배치 위치2mm 기준 미달은 보존했다. 정착한 실제 모델 끝 상태를 다음 검사의 출발 위치로 고정했다. "+'초기 배치에서 이 파지까지 들어온 경로가 검증된 것은 아니다.</p></section>')
    closing=get(BASE/'ring_thumb_only_closure/review.json');rows=''
    for r in closing['rows']:
        rows+=f"<tr><td>{r['extra_closing_hypothesis_N']:g}</td><td>{r['final_RF_normal_N']:.3f} / {r['final_TH_normal_N']:.3f}</td><td>{r['cube_shift_from_source_mm']:.3f}mm / {r['orientation_change_from_source_deg']:.3f}도</td><td>{r['upper_certified_samples']}</td><td>{'통과' if r['thumb_loaded_pose_and_clearance_hold_passed'] else '미달'}</td></tr>"
    parts.append('<section><h2>받침을 유지하고 엄지만 더 닫기</h2><p>다른12관절의 지령을 bit 단위로 유지하고 엄지만 닫았다. 모든 관절과 큐브의 동역학은 자유롭게 계산했다. 힘은 모델 가정과 계산값이며450mA의 환산값이 아니다.</p><table><tr><th>추가 닫기 가정N</th><th>약지/엄지 법선력N</th><th>큐브 변화</th><th>윗단 인증1ms 표본</th><th>하중·공간 유지</th></tr>'+rows+'</table>'+link(BASE/'ring_thumb_only_closure/review.json','엄지 보정 원본')+'</section>')
    replay=get(BASE/'ring_thumb_closure_replay/review.json') if (BASE/'ring_thumb_closure_replay/review.json').exists() else None
    if replay:parts.append('<section><h2>독립 재생 확인</h2><p>별도 모델 배치의 정착과 엄지 추가 닫기100목표를 이어 재생했고 두 구간의 전체 상태 차이가0이었다. 마지막4050상태의 약지·엄지 하중과 전체 손 지지, 윗단 인증이 모두 통과했다. 별도 배치에서의 첫 지령 설정은 명시한 초기화이며 실물 도달 경로가 아니다.</p>'+link(BASE/'ring_thumb_closure_replay/review.json','재생 근거')+'</section>')
    loads=get(BASE/'ring_thumb_settled_load_hypotheses/review.json');loadpassed=sum(r['conditional_hold_passed'] for r in loads['cases'])
    parts.append('<section><h2>정착 파지의 마찰·하중 가정</h2><p>'+f"{len(loads['cases'])}조건 중 {loadpassed}조건을 통과했다. 가정 마찰0.25/0.30/0.35의 무하중 및 양방향0.0005/0.001/0.003/0.01N·m 일정 모멘트다. 매 조건에서 약지·엄지0.1N 초과와4050상태 지지·공간을 검사했다. "+'실제 왼손 하중이나 일반적인 실물 허용력을 측정한 것이 아니다.</p>'+link(BASE/'ring_thumb_settled_load_hypotheses/review.json','가정 하중 원본')+'</section>')
    actor=get(BASE/'ring_thumb_original_actor_reuse/review.json')
    parts.append('<section><h2>저장 정책 재사용</h2><p>원본108차원 actor를 새 정착 파지에서 두 입력 이력 방식으로 시험했다. '+', '.join(str(r['steps'])+'목표' for r in actor['rows'])+'에서 손 접촉 기준으로 중단됐고 다음90도를 검증하지 못했다. 별도110차원 SAC 보정 정책과 구분한다. 이번에 새 SAC 학습이나 증류는 실행하지 않았다.</p>'+link(BASE/'ring_thumb_original_actor_reuse/review.json','정책 재사용 원본')+'</section>')
    turns=[]
    for name,title in (('ring_thumb_next90','약지·엄지 정착 파지의 다음90도'),('ring_thumb_relief_resume','중간 상태에서 여유·압박 보정'),('ring_thumb_tightened_relief_next90','엄지 추가 닫기 뒤 다음90도'),('ring_thumb_after_release_next90','손바닥 여유와 압박 해제 뒤 다음90도'),('ring_thumb_after_pair_retreat_next90','양쪽 손바닥 여유·관절 후퇴 뒤 접촉 유지 목표로 다음90도'),('ring_thumb_after_regrasp8_next90','추가 재파지 후 다음90도: 재파지 위치를 검사 전에 목표로 고정')):
        p=BASE/name/'review.json';done=p.exists()
        if not done:p=BASE/name/'progress.json'
        if not p.exists():continue
        r=get(p);state=r['final'] if done else r['actual'];steps=r['steps'] if done else r['step']
        turns.append(dict(name=name,source=p.relative_to(OUT).as_posix(),completed=done,steps=steps,error_deg=state['error_deg'],station_error_mm=state['center_target_error_mm'],passed=bool(done and r['conditional_bridge_and_hold_passed']),goal_reference=r.get('goal_reference')))
        parts.append('<section><h2>'+title+'</h2><p>'+('완료된 검사' if done else '진행 중인 계산 스냅샷')+f": {steps}목표,180도 목표 오차 {state['error_deg']:.3f}도, 출발 파지 위치와 {state['center_target_error_mm']:.3f}mm 차이. "+('다음 회전·유지 통과' if done and r['conditional_bridge_and_hold_passed'] else '완료한 회전·유지로 인증하지 않는다.')+'</p>'+link(p,'모델 기록')+'</section>')
    gait={}
    clear_path=BASE/'ring_tip_palm_clearance_waypoints/review.json';release_path=BASE/'ring_thumb_release_after_tip_clear/review.json';retreat_path=BASE/'ring_thumb_pair_clear_retreat_v2/review.json';chain_path=BASE/'ring_thumb_release_chain_replay/review.json'
    if all(p.exists() for p in (clear_path,release_path,retreat_path,chain_path)):
        clear=get(clear_path);release=get(release_path);retreat=get(retreat_path);chain=get(chain_path)
        strongest=retreat['trials'][2]
        gait=dict(RF_clear_passed=sum(r['RF_clear_supported_waypoint_passed'] for r in clear['trials']),pressure_release_passed=sum(r['intermediate_supported'] for r in release['rows']),paired_clear_retreat_passed=sum(r['RF_clear_supported_waypoint_passed'] for r in retreat['trials']),chain_replay_steps=chain['steps'],chain_replay_status=chain['status'],RF14_upper_room_deg=chain['RF14_upper_room_deg'],next90_not_certified=True)
        parts.insert(0,'')
        parts[0]='<section><h2>이번에 연결한 중간 동작</h2><p>다음 회전 중간에서 약지–손바닥 여유 확보 → 약지·엄지 압박 줄이기 → 양쪽 손바닥 여유 확보와 ID14 지령3도 후퇴를 연결했다. 관절·충돌 기준을 바꾸지 않았다. 마지막 탐색의 초기값 계산 오류는 같은 경계 안쪽의 초기값을 쓰도록 고쳤다.</p>'+f"<p>{chain['steps']}목표를 독립 재생했고 네 구간의 전체 상태 차이가0, 전체 {chain['samples']}개1ms 상태에서 손 지지가 유지됐다. ID14의 실제 모델 상한 여유는 {chain['RF14_upper_room_deg']:.3f}도다. 양쪽 여유 확보3조건이 모두 중간 지지 검사를 통과했으며 선택한 조건의 큐브 변화는 {strongest['cube_shift_mm']:.3f}mm·{strongest['orientation_change_deg']:.3f}도였다.</p>"+'<p>마지막3.37도 변화는 회전 도중의 변화다. 최종 파지 유지의3도 기준 통과로 세지 않았으며4050상태의 최종 윗단 공간도 이 중간 경로에서 인증하지 않았다. 이 경로 역시 별도로 배치한 약지·엄지 파지에서 출발하므로 첫 회전과의 전체 연결을 뜻하지 않는다.</p>'+link(chain_path,'426목표 독립 재생')+' · '+link(retreat_path,'양쪽 여유·후퇴3조건')+' · '+link(release_path,'압박 해제 성공·실패 보존')+'</section>'
        further_path=BASE/'ring_thumb_further_regrasp8/review.json';long_path=BASE/'ring_thumb_regrasp8_chain_replay/review.json'
        if further_path.exists() and long_path.exists():
            further=get(further_path);long=get(long_path);gait['further_regrasp8']=dict(conditions=len(further['trials']),supported_intermediates=sum(r['RF_clear_supported_waypoint_passed'] for r in further['trials']),replay_steps=long['steps'],RF14_upper_room_deg=long['RF14_upper_room_deg'])
            parts.insert(1,'<section><h2>추가 재파지로 관절 여유 확보</h2><p>'+f"같은 관절 제한 안에서 ID14 지령을8도 더 후퇴시키는 세 중간 경로가 손 지지를 유지했다. 선택한77목표까지 합친 {long['steps']}목표를 독립 재생했고 다섯 구간의 전체 상태 차이가0, 전체 손 지지100%, 실제 ID14 상한 여유 {long['RF14_upper_room_deg']:.3f}도를 확인했다. "+'실물 부하·최종 작업 파지·완료한 다음90도 인증은 아니다. 후속 회전은 이 재파지 위치를 검사 전에 고정한 별도 모델 조건이다. 이전 위치 복귀 실패를 통과로 바꾸지 않는다.</p>'+link(long_path,'503목표 재생')+' · '+link(further_path,'추가 재파지3조건')+'</section>')
    third_support=[]
    for geometry_name,arrival_name,title in (('ring_thumb_MF_third_support_geometry','ring_thumb_MF_third_support_arrival','회전 중간 상태의 중지 임시 지지'),('ring_thumb_regrasp8_MF_geometry','ring_thumb_regrasp8_MF_arrival','여유를 확보한 재파지 뒤 중지 임시 지지')):
        gp=BASE/geometry_name/'review.json';ap=BASE/arrival_name/'review.json'
        if not gp.exists():continue
        geometry=get(gp);arrival=get(ap) if ap.exists() else None
        item=dict(geometry=gp.relative_to(OUT).as_posix(),static_candidates=sum(r['static_contact_candidate'] for r in geometry['rows']),static_tests=len(geometry['rows']),arrival_completed=arrival is not None,
            arrival_trials=len(arrival['rows']) if arrival else 0,temporary_three_finger_hold_passed=sum(r['temporary_three_finger_hold_passed'] for r in arrival['rows']) if arrival else 0)
        third_support.append(item)
        text=f"정적 기하 후보 {item['static_candidates']}/{item['static_tests']}개. "
        text+=f"자유 큐브 도달 {item['arrival_trials']}조건 중 임시 중지·약지·엄지4050상태 유지 {item['temporary_three_finger_hold_passed']}조건 통과." if arrival else '자유 큐브 도달 검사는 진행 중이다.'
        parts.append('<section><h2>'+title+'</h2><p>'+text+' 약지·엄지 지령은 고정하고 중지만 움직였다. 정적 접촉 후보와 실제 동역학 도달은 구분하며, 임시 지지 검사는 최종 윗단 공간·다음90도 인증이 아니다. 회전 중간 소스의 중단은 약지 끝–손바닥의 미세 모델 겹침 기준이며 실제 위험한 충돌을 측정했다는 뜻이 아니다.</p>'+link(gp,'정적 접촉 원본')+(' · '+link(ap,'자유 큐브 도달 원본') if arrival else '')+'</section>')
    feedback_studies=[]
    for name in ('ring_thumb_MF_support_feedback','ring_thumb_MF_support_feedback_v2'):
        fp=BASE/name/'review.json'
        if not fp.exists():continue
        feedback=get(fp);passed=sum(r['temporary_three_finger_hold_passed'] for r in feedback['rows'])
        feedback_studies.append(dict(path=fp.relative_to(OUT).as_posix(),trials=len(feedback['rows']),temporary_three_finger_hold_passed=passed))
        parts.append('<section><h2>중지 접촉의 모델 피드백: '+name+'</h2><p>'+f"{len(feedback['rows'])}조건 중 임시 세 손가락 유지 {passed}조건 통과. "+'정확한 모델 거리·접촉 법선·힘을 가정하고 중지 지령만 보정했다. 새 SAC 학습이나 실제 센서로 구현한 제어가 아니다. 첫 보정의 외부 받침 접촉 중단과 중지 하중 유지 미달은 그대로 보존했다.</p>'+link(fp,'피드백 결과와 입력 가정')+'</section>')
    simultaneous=[]
    for name,key in (('ring_thumb_three_contact_retreat8','RF_clear_supported_waypoint_passed'),('ring_thumb_retreat_with_MF_feedback','coordinated_three_finger_retreat_passed')):
        rp=BASE/name/'review.json'
        if not rp.exists():continue
        r=get(rp);trial_rows=r.get('trials',r.get('rows',[]));passed=sum(row[key] for row in trial_rows);simultaneous.append(dict(path=rp.relative_to(OUT).as_posix(),conditions=len(trial_rows),passed=passed))
        if 'feedback_helper_sha256' in r:
            helper=OUT/'servo_MF_temporary_support.py';snapshot=rp.parent/'source_servo_MF_temporary_support.py'
            if not snapshot.exists():
                if sha(helper)!=r['feedback_helper_sha256']:raise ValueError('Feedback helper changed before archiving')
                snapshot.write_bytes(helper.read_bytes())
        parts.append('<section><h2>중지 지지 상태에서 약지 재배치: '+name+'</h2><p>'+f"{len(trial_rows)}조건 중 지지 전환 검사 {passed}조건 통과. "+('중지 지령을 고정한 비교에서는 약지 이동 뒤 중지 하중이 사라져 세 조건이 모두 미달했다. 손바닥을 포함한 손 지지가 유지되는 것과 중지 추가 지지가 유지되는 것을 구분했다.' if key=='RF_clear_supported_waypoint_passed' else '약지·엄지 경로에 중지 거리·하중 피드백을 동시에 적용했다. 정확한 모델 신호를 사용하는 별도 동역학 검사이며 실물 제어·다음90도 인증이 아니다.')+'</p>'+link(rp,'동시 제어 비교 원본')+'</section>')
    if simultaneous:parts.insert(0,'<section><h2>현재 남은 구체적인 접촉 전환</h2><p>중지의 임시 하중은 모델 피드백 한 조건에서 확보했다. 중지를 고정하면 약지 재배치 뒤 중지 하중을 잃었고, 동시 피드백 두 조건은 중지·엄지 끝 주변 충돌 프록시(cadtip_mf_th)의 미세 겹침 기준으로 중단됐다. 다음90도와 처음 배치부터의 전체 연결은 아직 통과하지 않았다. 이 미세 모델 겹침은 실물 위험 충돌의 측정값이 아니다.</p></section>')
    conditional_path=BASE/'resumed_next90_joined_branch/review.json';conditional=get(conditional_path) if conditional_path.exists() else None
    if conditional:parts.append('<section><h2>별도 중지·엄지 파지의 조건부 다음90도</h2><p>'+f"{conditional['steps']}목표를 초기화 없이 재생해 실제 추가 yaw {conditional['additional_yaw_deg']:.3f}도,180도 자세 오차 {conditional['final180_error_deg']:.3f}도, 마지막4050상태 지지·윗단 공간을 확인했다. "+f"출발은 별도로 배치한 중지·엄지 파지이며 출발–목표 작업 위치가 {conditional['start_to_target_station_distance_mm']:.3f}mm 다르다. "+'사진의 약지 파지 결과와 합쳐 전체 두 회전 성공으로 세지 않는다. 첫3015목표의 끝에서 이 출발 파지로 들어가는 연결도 아직 없다.</p>'+link(conditional_path,'조건부637목표 독립 재생')+'</section>')
    report=dict(status='ring_front_thumb_preparation_progress',date='2026-11-11',source_reports_sha256=sources,user_confirmed_ring_on_white_front_face=True,
        white_front_to_model_axis_is_display_and_geometry_hypothesis=True,photo_not_camera_registered=True,original_recorded_ring_values_preserved=True,
        dynamic_reference_ring_difference_from_record_deg=geom['ring_difference_from_record_deg'],exact_reference_static_geometry_separate=True,
        thumb_only_conditions_passed=sum(r['thumb_loaded_pose_and_clearance_hold_passed'] for r in closing['rows']),thumb_only_conditions=len(closing['rows']),
        initialized_settle_shift_mm=cold08['maximum_station_drift_mm'],old_shift_failure_not_reclassified=True,settled_load_hypotheses_passed=loadpassed,settled_load_hypotheses=len(loads['cases']),
        thumb_closure_replay_passed=bool(replay and replay['status']=='passed'),turn_studies=turns,intermediate_release_and_retreat=gait,temporary_MF_support_studies=third_support,MF_model_feedback_studies=feedback_studies,MF_support_during_RF_retreat=simultaneous,
        other_MF_TH_next90_between_two_stations_passed=bool(conditional and conditional['conditional_next90_between_two_stations_passed']),
        first_to_next_same_station_cycle_passed=False,source_workstation_not_reached_from_first3015_path=True,hardware_execution_allowed=False,new_SAC_training_steps=0,devices_opened=0)
    (FOLDER/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>약지 앞면 받침·엄지 보정</title><style>body{font:16px/1.7 system-ui;max-width:1150px;margin:28px auto;padding:0 18px;background:#f5f9f7;color:#173d36}section{background:white;border:1px solid #c6d8ce;margin:18px 0;padding:18px}.note{padding:16px;background:#fff0ce}.grid{display:grid;grid-template-columns:1fr 1fr;gap:15px}figure{margin:0}img{width:100%}table{border-collapse:collapse;width:100%}th,td{padding:9px;border-bottom:1px solid #d0dfd6;text-align:left;font-size:14px}a{color:#087768}@media(max-width:700px){.grid{grid-template-columns:1fr}table{display:block;overflow:auto}}</style><body><h1>약지 앞면 받침 → 엄지 보정 → 다음 회전</h1><p class="note">사진을 참고한 모델 가설과 실물 사진의 정확한 재현을 구분한다. 약지 받침·엄지 보정의 모델 유지가 통과했으며, 전체 초기 배치→회전→유지→다음 회전→유지의 실물 준비는 아직 완료하지 않았다.</p>'''+''.join(parts)+'''<p><a href="review.json">범위와 출처</a> · <a href="../bridge_progress_20261110/index.html">기존 연결 검증</a></p></body></html>'''
    (FOLDER/'index.html').write_text(page,encoding='utf-8')
    status=['# 2026-11-11 오프라인 준비 상태','',
        '전체 초기 배치 → 첫 회전 → 윗단 공간을 비운 유지 → 다음 회전 → 유지는 아직 완료되지 않았다. 현재 사용자 실물 작업은 없다.','',
        '- 사용자가 확인한 약지 흰 앞면 접촉을 모델 가설에 반영했다. 원기록·원래 관절 제한을 보존했다.',
        '- 엄지 추가 닫기3조건과 별도 모델 하중11조건, 독립 재생을 확인했다.',
        '- 약지·엄지 손바닥 여유와 재파지를503목표로 연결해 독립 재생했다. 다섯 상태 이음 차이0, 손바닥을 포함한 지지100%, ID14 모델 상한 여유11.991도다.',
        '- 이503목표는 별도로 배치한 약지·엄지 파지에서 출발한 중간 동작이다. 첫 회전 도달 경로와 다음90도를 대신하지 않는다.',
        '- 마지막 중간 자세의 변화는 최종 유지 오차 허용으로 세지 않는다.4050상태 최종 윗단 공간 인증은 별도다.','',
        '## 완료한 다음 회전 검사','']
    for r in turns:status.append(f"- {r['name']}: {r['steps']}목표,180도 오차{r['error_deg']:.3f}도, 위치 목표 오차{r['station_error_mm']:.3f}mm, 완료 결과 있음={r['completed']}, 회전·유지 통과={r['passed']}.")
    status+=['','## 다음에 해결할 항목','',
        '1. 재파지 중 중지의 실제 접촉 하중을 유지하고 약지·엄지의 다음 회전 접촉 전환을 연결한다.',
        '2. 초기 배치에서 실제 도달한 첫90도 끝 상태와 동일한 상태에서 다음 회전을 시작하도록 전체 연결을 재생한다.',
        '3. 같은 전체 경로의 구동·배치 오차와 최종 윗단 공간을 통과한 뒤 실물 시험 경로를 준비한다.','',
        '이번 후속은 모델 교사·기하·접촉 피드백 검사다. 새 SAC 학습0스텝, 장치 접근0회, 실물 구동 없음.','',
        '[그림과 원본 근거](index.html) · [범위와 출처 해시](review.json)']
    (FOLDER/'STATUS.md').write_text('\n'.join(status)+'\n',encoding='utf-8')
    for existing in (BASE/'index.html',BASE/'bridge_progress_20261110/index.html',BASE/'opposing_grip_review_20261110/index.html',ROOT/'right_hand_rotation_20261109/policy_audit/explanation/index.html'):
        s=existing.read_text(encoding='utf-8');start='<!-- ring_front_latest -->';end='<!-- ring_front_latest_end -->'
        if start in s:
            a=s.index(start);b=s.index(end,a)+len(end);s=s[:a]+s[b:]
        href=Path(os.path.relpath(FOLDER/'index.html',existing.parent)).as_posix()
        banner=start+'<p style="padding:14px;background:#fff0ce"><a href="'+href+'">2026-11-11 최신: 약지 앞면 받침·엄지 보정·중간 동작 연결 검토</a></p>'+end
        i=s.find('>',s.find('<body'))+1
        if i<=0:
            # The older explanation uses HTML's implicit body element.
            i=s.find('<h1')
            if i<0:raise ValueError('Report content missing: '+str(existing))
        existing.write_text(s[:i]+banner+s[i:],encoding='utf-8')
    summary='2026-11-11 최신: 사용자가 약지는 사진의 흰색 앞면을 받친다고 확인했다. 사진을 정밀 등록한 것으로 가정하지 않는다. 기록4값 그대로의 정적 접촉 후보와 원래 범위 안의 별도 동적 후보를 분리했다. 동적 참고 자세의 ID14는 원기록보다10.877도 작고 원래 기록·관절 제한은 보존했다. 정착 모델에서 약지 받침을 유지한 엄지 추가 닫기3조건 모두4050상태의 하중·지지·윗단 공간을 통과했고, 가장 큰 조건은 큐브 이동0.595mm·자세 변화0.573도·양쪽 법선력 약1.65N이다. 정착과 엄지100목표의 독립 재생은 두 구간 전체 상태 차이0이었다. 가정 마찰·모멘트11조건도 통과했으나 실물 힘 측정은 아니다. 새 약지 파지에서 원본108정책2회와 다음90도 모델 보정을 별도로 검사했다. 별도 중지·엄지 출발의637목표는 실제90.075도와 유지까지 통과했지만 출발–목표 위치가19.505mm 다르고 첫3015끝에서 출발까지 연결되지 않아 전체 두 회전 성공으로 세지 않는다. 이번 새SAC 학습·장치 접근·원래 제한 변경은 없다. 최신 근거: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/ring_front_thumb_progress_20261111/index.html`.'
    if gait:summary+=' 이번 후속: 약지 손바닥 여유 확보→압박 해제→약지·엄지 여유와ID14후퇴를426목표로 연결해 독립 재생했다. 네 구간 전체 상태 차이0, 전체31950개1ms 표본에서 손 지지 유지, ID14 실제 모델 상한 여유3.789도다. 마지막 중간 구간의3.37도 변화는 최종 유지3도 통과로 세지 않는다. 다음90도와 초기 첫 회전부터의 도달 경로는 아직 완료하지 않았다.'
    if gait.get('further_regrasp8'):summary+=f" 추가 재파지를 포함한503목표 재생도 다섯 구간 전체 상태 차이0, 손 지지100%, ID14 실제 상한 여유{gait['further_regrasp8']['RF14_upper_room_deg']:.3f}도다. 이번에는 새SAC 학습 대신 모델 교사와 접촉 전환을 검사했다."
    if feedback_studies:summary+=' 중지 추가 지지는 정확한 모델 거리·힘 피드백1조건에서4050표본 조건을 통과했다(중지0.1N초과81.4%, 손 지지·약지·엄지100%, 큐브 변화최대0.843mm·0.177도). 실물 센서 계약이 아니다. 중지 고정 뒤 약지 재배치3조건은 중지 하중을 잃었고, 중지 피드백 동시 적용2조건은 중지·엄지 끝 주변 프록시의 미세 모델 겹침으로 중단됐다. 이 접촉 전환을 해결하고 다음90도·초기 첫 회전부터의 전체 연결을 검증해야 한다.'
    replace_block(ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',summary,after='## 지금 먼저 할 일');replace_block(ROOT/'professor_progress_20261015.md',summary)
    readiness_path=OUT/'next_cycle/readiness.json';ready=read(readiness_path);ready['ring_front_thumb_progress']=dict(path=(FOLDER/'review.json').relative_to(OUT).as_posix(),sha256=sha(FOLDER/'review.json'),thumb_only_model_passed=True,full_cycle_passed=False,hardware_execution_allowed=False)
    readiness_path.write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(thumb_conditions_passed=report['thumb_only_conditions_passed'],loads_passed=loadpassed,pending=sum(not r['completed'] for r in turns),report=(FOLDER/'index.html').relative_to(OUT).as_posix())),flush=True)


if __name__=='__main__':main()
