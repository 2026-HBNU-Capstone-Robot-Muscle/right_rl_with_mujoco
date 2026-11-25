"""Publish completed research results without creating a robot command."""
import hashlib,json,shutil
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261106/taught_upper_clearance'
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    baseline=read(O/'baseline_review.json');cases=[]
    for name in ('proposal_review.json','deep_proposal_review.json'):
        for c in read(O/name)['cases']:cases.append(dict(family='fixed_contact_point',name=c['name'],dynamic=c['dynamic']))
    for c in read(O/'sliding_contact/review.json')['cases']:
        for v in c['free_replays']:cases.append(dict(family='sliding_contact',name=c['name']+'/'+v['order'],dynamic=v['dynamic']))
    searches=[]
    for name in ('dynamic_refinement','dynamic_refinement_strict'):
        r=read(O/name/'review.json');searches.append(dict(name=name,evaluations=r['search_evaluations']));cases.append(dict(family='compiled_free_search',name=name,dynamic=r['dynamic']))
    for c in read(O/'contact_following/review.json')['cases']:cases.append(dict(family='perfect_model_contact_following',name=str(c['target_normal_N']),dynamic=c['dynamic']))
    transfer=O/'index_contact_transfer/review.json'
    if transfer.exists():
        for c in read(transfer)['cases']:
            for v in c['cases']:cases.append(dict(family='index_contact_transfer',name=str(c['contact_height_mm'])+'/'+v['order'],dynamic=v['dynamic']))
    safe=sum(c['dynamic']['safety_passed'] for c in cases);task=sum(c['dynamic']['whole_grip_task_passed'] for c in cases)
    report=dict(status='completed_taught_curvature_research_no_motion_adopted',meaning='User bent the middle finger enough for upper-layer rotation; all four middle joints remain approximate curvature evidence.',baseline_grasp_safety_passed=baseline['dynamic']['safety_passed'],suffix_cases=len(cases),suffix_original_dynamic_guard_passes=safe,suffix_full_grip_and_upper_sweep_passes=task,compiled_proposals=sum(s['evaluations'] for s in searches),searches=searches,model_safety_is_not_physical_success=True,model_assumptions=dict(friction=.3,friction_not_measured=True,actuator_cap_Nm=.15,cap_not_measured_450mA_conversion=True),source_teaching_sha256=read(R/'right_hand_rotation_20261106/manual_grasp/middle_teach_review.json')['source_sha256'],cube_upper_layer_sweep_required_only_during_other_hand_turn=True,new_physical_command_created=False,devices_opened_by_simulation=False,physical_success=False,hardware_execution_allowed=False,cases=[dict(family=c['family'],name=c['name'],safety=c['dynamic']['safety_passed'],task=c['dynamic']['whole_grip_task_passed'],completed=c['dynamic']['completed_goals'],total=c['dynamic']['total_goals'],worst=c['dynamic']['worst'],final_upper_tilt_deg=c['dynamic']['final_up_tilt_deg'],tail=c['dynamic']['settled_tail']) for c in cases])
    current=O/'current_recovery_215211/context.json'
    readpath=R/(read(current)['source'] if current.exists() else 'resume_off_readonly_20261106_214425_132553/result.json');rr=read(readpath);report['latest_readonly']=dict(source=readpath.relative_to(R).as_posix(),source_sha256=sha(readpath),status=rr['status'],error=rr.get('error'),servo_writes=rr['servo_writes'],torque_changes=rr['torque_changes'])
    next_read='현재 관절값 읽기는 USB 어댑터가 연결 목록에 없어 수행되지 않았다. 큐브를 제거하고 USB와 기존 5V 전원을 연결한 뒤 알려주면, 먼저 각도만 읽어서 다음 복귀 경로를 준비한다. 손가락과 받침을 수동으로 맞출 필요는 없다.'
    latest_line='USB 어댑터가 없어 현재 각도를 읽지 못했다. 큐브 제거 후 USB와 기존 5V 연결이 다음 사용자 작업이다.'
    if current.exists():
        ctx=read(current);screen=read(current.parent/'screen/review.json');assert screen['all_passed'];report['fresh_empty_recovery']=dict(source=current.relative_to(R).as_posix(),source_sha256=sha(current),minimum_voltage_V=ctx['minimum_voltage_V'],maximum_temperature_C=ctx['maximum_temperature_C'],stationary_drift_ticks=ctx['position_drift_ticks'],goals=screen['goals'],static_passes=sum(c['passed'] for c in screen['static_cases']),dynamic_passes=sum(c['passed'] for c in screen['dynamic_cases']),all_passed=screen['all_passed'],review_sha256=sha(current.parent/'screen/review.json'),hardware_execution_allowed=False)
        camera=max(O.glob('current_camera_*/result.json'),key=lambda p:p.stat().st_mtime);cr=read(camera);report['latest_camera']=dict(source=camera.relative_to(R).as_posix(),status=cr['status'],error=cr.get('error'))
        next_read=f'USB와 5V 전원 연결을 확인했고 16개 모두 응답했다. 최저 전압은 {ctx["minimum_voltage_V"]:.1f}V, 최고 온도는 {ctx["maximum_temperature_C"]}도, 세 번 읽은 각도 차이는 0틱이다. 현재 자세의 빈손 복귀 경로는 정적 201조건과 동적 9조건을 통과했다. 실제 모터 목표나 토크 변경은 보내지 않았다. 현재 Streaming Webcam은 연결 목록에 없어 사진을 얻지 못했다. 웹캠 연결 후 현재 손 모양을 모델과 비교하고 다음 구동 후보를 검토한다. 새 구동 명령은 아직 발행하지 않았다.'
        latest_line='USB와 5V 연결 후 16개 정상 응답, 최저 5.0V, 최고 18도, 0틱 드리프트 및 토크 OFF를 확인했다. 현재 복귀 201정적/9동적 조건을 통과했다. Streaming Webcam 연결이 다음 사용자 작업이다.'
        if cr['status']=='camera_only_photo_saved':
            next_read='USB와 5V 전원 및 Streaming Webcam 연결을 확인했다. 큐브를 제거한 현재 손 사진과 사진 뒤의 OFF 관절값을 확보했다. 모터 목표와 토크 변경은 보내지 않았다. 다음 작업은 기존 파지에서 엄지를 고정하고 중지만 접었다 돌아오는 진단 동작이다. [중지 진단 실행 순서](../middle_clearance_probe/operator_guide.md)를 따른다. 완성된 하부 파지나 윗단 회전은 아직 검증하지 않았다.'
            latest_line='로봇과 웹캠 연결 후 빈손 사진 및 관절값을 확인했다. 다음 작업은 기존 파지에서 엄지를 고정하고 중지만 접었다 돌아오는 진단 동작이다.'
        (O/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    (O/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    note=f'''# 중지 굽힘과 윗단 공간 검토\n\n사용자가 중지를 굽힌 목적은 큐브 맨 윗단의 회전 공간을 확보하는 것이다. 방향만 참고하는 것으로 해석하지 않는다. 손끝부터 ID9, 11, 10, 12의 저장 각도를 모두 대략적인 굽힘 기준으로 유지한다. 토크 OFF 뒤 손 전체가 처졌으므로 다른 손가락까지 저장 자세로 되돌리지 않는다. 추가 OFF 보정 촬영은 요청하지 않는다.\n\n## 이번 계산 결과\n\n- 기존 파지의 준비된 배치부터 860개 파지 목표와 고정 10.05초를 다시 계산했다. 기준 모델에서 기존 동적 기준을 통과했다. 기존 파지에는 중지와 엄지가 윗단 회전 공간에 들어오는 부분이 있다.\n- 후속 경로 {len(cases)}개를 원래 1ms 동적 기준으로 검사했고 {safe}개가 완료됐다. 윗단 공간과 하부의 양쪽 손끝 접촉을 동시에 만족한 경로는 {task}개다.\n- 별도의 빠른 자유 큐브 탐색 {report['compiled_proposals']}개는 후보 생성용이다. 이 횟수를 완전 검증된 경로 수에 포함하지 않는다.\n- 중지 기록을 절반 반영한 정적 비교에서는 중지 쪽 공간이 확보됐지만 손끝 접촉이 사라졌다. 정적 그림은 잡는 힘을 증명하지 않는다.\n- 일부 동적 후보는 손바닥과 손가락 뿌리에 큐브를 얹은 채 유지됐다. 다른 손이 윗단을 돌릴 때 버틸 파지로 채택하지 않았다.\n- 실제 모터 전류, PID, 전압 중지 기준, 네이티브 관절 범위는 바꾸지 않았다.\n\n## 모델의 범위\n\n55mm 강체 큐브와 CAD 접촉 형상, 마찰 0.3, 모델 구동 한도 0.15 N·m 조건이다. 마찰은 실측값이 아니고 구동 한도는 실제 450mA를 환산한 값이 아니다. 큐브 레이어 자체나 다른 손은 이 실험에서 직접 움직이지 않았다. 실물 성공 및 새로운 구동 경로는 아직 없다.\n\n## 다음 단계\n\n다음 후보는 큐브 하부 접촉을 유지한 채 접촉점을 옮기는 방법과 손가락의 역할을 나누는 방법을 검토해야 한다. 현 단계 결과로 기존 물리 실행 파일을 새 자세용으로 바꾸지 않는다.\n\n현재 관절값 읽기는 USB 어댑터가 연결 목록에 없어 수행되지 않았다. 큐브를 제거하고 USB와 기존 5V 전원을 연결한 뒤 알려주면, 먼저 각도만 읽어서 다음 복귀 경로를 준비한다. 손가락과 받침을 수동으로 맞출 필요는 없다.\n\n[시뮬레이션 비교 그림](evidence/index.html) / [전체 결과](summary.json) / [출처와 경로 검사](validation.json)\n'''
    note=note.replace('현재 관절값 읽기는 USB 어댑터가 연결 목록에 없어 수행되지 않았다. 큐브를 제거하고 USB와 기존 5V 전원을 연결한 뒤 알려주면, 먼저 각도만 읽어서 다음 복귀 경로를 준비한다. 손가락과 받침을 수동으로 맞출 필요는 없다.',next_read)
    if transfer.exists():note+='\n검지로 접촉을 먼저 넘기는 경로 9개도 비교했다. 이 시도의 기하 최적화는 목표 접촉점에 도달하지 못했으며, 이 결과로 접촉 전환이 물리적으로 불가능하다고 판단하지 않는다. 검지와 중지의 배치를 추가 검토해야 한다.\n'
    (O/'progress.md').write_text(note,encoding='utf-8')
    # Keep a publication-time code snapshot; generation-time hashes are retained
    # in each historical result and are not silently rewritten.
    archive=O/'source_snapshot';archive.mkdir(exist_ok=True);files=['research_taught_grasp_20261106.py','research_taught_contact_slide_20261106.py','refine_taught_grasp_dynamic_20261106.py','follow_taught_lower_contacts_20261106.py','research_taught_contact_transfer_20261106.py','prepare_taught_current_review_20261106.py','capture_taught_current_camera_20261106.py','render_taught_grasp_20261106.py','validate_taught_grasp_artifacts_20261106.py','publish_taught_grasp_review_20261106.py'];bindings=[]
    for name in files:shutil.copyfile(R/name,archive/name);bindings.append(dict(source=name,sha256=sha(R/name),snapshot='source_snapshot/'+name))
    (O/'source_snapshot.json').write_text(json.dumps(dict(scope='publication_time_code_snapshot_not_a_replacement_for_generation_time_hashes',files=bindings),indent=2)+'\n',encoding='utf-8')
    progress=R/'professor_progress_20261015.md';old=progress.read_bytes();begin=b'<!-- taught_upper_clearance_20261106_current -->';end=b'<!-- taught_upper_clearance_20261106_end -->'
    if old.startswith(begin) and end in old:old=old.split(end,1)[1].lstrip(b'\r\n')
    header=f'''<!-- taught_upper_clearance_20261106_current -->\n# 현재 작업: 중지 굽힘을 반영한 윗단 공간 검토\n\n- 사용자 의도: 중지를 윗단이 돌아갈 정도로 굽힘. 네 관절의 저장 각도를 대략적인 자세 기준으로 사용하고, 전체 OFF 처짐을 새 목표로 채택하지 않는다.\n- 기존 자연 파지를 재현하고 후속 경로 {len(cases)}개를 검사했다. {safe}개는 원래 동적 기준을 완료했지만 윗단 공간과 하부 손끝 파지를 함께 만족한 후보는 {task}개다.\n- 빠른 후보 탐색 {report['compiled_proposals']}개는 완전 검증 경로 수와 구분한다. 이 연구 결과는 구동 후보로 채택하지 않았다.\n- {latest_line}\n- [계산 결과와 다음 단계](right_hand_rotation_20261106/taught_upper_clearance/progress.md) / [비교 그림](right_hand_rotation_20261106/taught_upper_clearance/evidence/index.html)\n<!-- taught_upper_clearance_20261106_end -->\n\n'''
    if not old.startswith(b'<!-- middle_probe_ready_20261106_current -->'):progress.write_bytes(header.encode('utf-8')+old)
    print(json.dumps({k:v for k,v in report.items() if k not in ('cases','searches')},ensure_ascii=False))
if __name__=='__main__':main()
