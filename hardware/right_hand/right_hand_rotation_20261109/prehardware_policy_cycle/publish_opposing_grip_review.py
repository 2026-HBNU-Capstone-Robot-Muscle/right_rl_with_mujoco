"""Report opposed-contact work, preserving cold starts and alternative targets."""
from pathlib import Path
import json
import html
import hashlib
import numpy as np

OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'opposing_grip_review_20261110'


def read(path):return json.loads(path.read_text(encoding='utf-8'))
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def link(path,label):return '<a href="../'+path.relative_to(BASE).as_posix()+'">'+html.escape(label)+'</a>'


def main():
    FOLDER.mkdir(exist_ok=True);sources={};sections=[]
    thumb_path=BASE/'thumb_only_closure_20261110/review.json'
    if thumb_path.exists():
        thumb=read(thumb_path);sources[thumb_path.relative_to(OUT).as_posix()]=digest(thumb_path)
        table=''
        for r in thumb['rows']:
            if r.get('skipped'):continue
            name='실제 첫 파지 도달 상태' if r['scenario']=='reached_first90' else '별도 맞은편 접촉 파지'
            table+=f"<tr><td>{name}</td><td>{r['extra_closing_hypothesis_N']:g}</td><td>{r['final_TH_normal_N']:.4f}</td><td>{r['cube_shift_from_source_mm']:.3f}mm / {r['orientation_change_from_source_deg']:.3f}도</td><td>{'통과' if r['thumb_loaded_pose_and_clearance_hold_passed'] else '엄지 하중 형성 미달'}</td></tr>"
        sections.append('<section><h2>사용자 제안: 다른 지령을 유지하고 엄지만 더 닫기</h2><p>맞은편 접촉이 있는 모델 파지에서는 엄지만 추가로 닫아 엄지 압박이 증가하면서 지지·윗단 공간 유지가 통과했다. 실제 첫 회전 끝 모델의 맞은편 접촉은 이와 달라 엄지 하중 형성은 미달이었다. 아래 힘은 모델 가정이며450mA 환산값이나 사진 자세의 측정값이 아니다. 엄지 하중 형성 미달을 전체 손 지지 실패로 부르지 않는다.</p><table><tr><th>출발</th><th>추가 닫기 가정N</th><th>최종 엄지 법선력N</th><th>큐브 변화</th><th>엄지 하중·공간 유지</th></tr>'+table+'</table>'+link(thumb_path,'원래 지령·자유 동역학 결과')+' · '+link(thumb_path.parent/'REVIEW.md','판정과 사용자 사진의 구분')+'</section>')
    first_path=BASE/'initial_to_reached_workstation90/review.json'
    if first_path.exists():
        first=read(first_path);sources[first_path.relative_to(OUT).as_posix()]=digest(first_path)
        sections.append('<section><h2>초기 배치부터 도달한 첫90도 파지</h2><p>'+f"{first['steps']}목표를 상태 초기화 없이 재생했고 네 구간의 전체 상태가 저장 결과와 일치했다. 90도 자세 오차 {first['final_orientation_error_deg']:.3f}도, 마지막4.05초 지지100%, 윗단 인증 하한 {first['minimum_upper_lower_mm']:.3f}mm다. "+f"기존 canonical 위치와 {first['canonical_station_error_mm']:.3f}mm 다르다. 실제 도달한 이 위치에서 다음90도와 복귀를 별도로 검사한다."+'</p><p>이 첫 파지의 연속 재생과 아래 맞은편 파지의 별도 배치를 구분한다.</p>'+link(first_path,'초기 배치부터의 실제 모델 재생')+'</section>')
    geometry=BASE/'opposing_workspace_Z_seed122/review.json';g=read(geometry);sources[geometry.relative_to(OUT).as_posix()]=digest(geometry)
    parent=OUT/g['source'];oldC=np.load(parent,allow_pickle=False)['qpos'][-7:-4];newC=np.asarray(g['center']);shift=float(np.linalg.norm(newC-oldC)*1000)
    sections.append('<section><h2>작업 위치를 함께 탐색한 이유</h2><p>큰 회전 뒤의 모델 큐브는 검지 뿌리·마디 위에 남아 있었다. 기존 관절 자세를 복사하면 큐브 위치가 달라 엄지·중지 접촉력이 남지 않았다. 현재 위치에서 맞은편 접촉점까지 탐색한 두 조건과, 위치도 함께 바꾼 두 조건을 구분했다.</p>'+
        f'<p>새 후보는 원래 상태에서 <strong>{shift:.3f}mm 떨어진 가정 위치</strong>다. 기존 공통 위치의2mm 기준과 실패 기록은 그대로다. 새 위치로의 도달과 두 회전 사이의 공통 위치 사용을 모두 검증해야 한다.</p>'+link(geometry,'기하 탐색 근거')+'</section>')
    holds=[]
    for name,label in (('free_hold_tests','약90도 근처 별도 배치'),('free_hold_tests_yaw180','정확한180도 별도 배치')):
        path=geometry.parent/name/'review.json'
        if not path.exists():continue
        data=read(path);sources[path.relative_to(OUT).as_posix()]=digest(path);holds.append(dict(label=label,source=path.relative_to(OUT).as_posix(),report=data))
        rows=''
        for r in data['rows']:
            if r.get('skipped'):rows+='<tr><td>'+str(r['preload_hypothesis_N'])+'</td><td colspan="5">원래 지령 범위 밖으로 제외</td></tr>';continue
            rows+=f"<tr><td>{r['preload_hypothesis_N']:g}N 가정</td><td>{'통과' if r['initialized_hold_passed'] else '미달'}</td><td>{r['hold_samples']}</td><td>{r['MF_load_fraction']*100:.0f}% / {r['TH_load_fraction']*100:.0f}%</td><td>{r['minimum_upper_lower_mm'] if r['minimum_upper_lower_mm'] is not None else '미평가'}</td><td>{r['maximum_station_drift_mm']:.3f}mm / {r['maximum_orientation_drift_deg']:.3f}도</td></tr>"
        sections.append('<section><h2>'+label+'</h2><p>큐브를 그 위치에 배치한 뒤 자유 동역학으로 유지했다. 첫 회전이나 다음 회전으로 도달한 결과가 아니다. 중력 보상과 접촉 예압은 모델 지령이며450mA의 환산값이 아니다.</p><table><tr><th>예압 가정</th><th>배치 유지</th><th>유지1ms 표본</th><th>중지/엄지0.1N 초과</th><th>윗단 인증 하한mm</th><th>처음 배치에서 변화</th></tr>'+rows+'</table>'+link(path,'유지·접촉력·한도 원본')+'</section>')
    loadpath=BASE/'opposing_grip_load_hypotheses/review.json';loads=None
    if loadpath.exists():
        loads=read(loadpath);sources[loadpath.relative_to(OUT).as_posix()]=digest(loadpath)
        rows=''.join(f"<tr><td>{r['assumed_sliding_friction']:g}</td><td>{r['assumed_constant_moment_about_up_Nm']:+g}</td><td>{'통과' if r['conditional_hold_passed'] else '미달'}</td><td>{r['angle_drift_deg']:.3f}도</td><td>{r['center_drift_mm']:.3f}mm</td></tr>" for r in loads['cases'])
        sections.append('<section><h2>하중·마찰 가정의 유지 비교</h2><p>새 파지에 배치해 유지한 끝 상태에서 각 조건을 따로 시작했다. 실제 왼손 하중, 실물 허용치, 도달 경로의 강건성을 측정한 결과가 아니다. 위치 기준은 명시한 새 파지이며 기존 공통 위치 판정을 바꾸지 않는다.</p><table><tr><th>마찰 가정</th><th>일정 외부 모멘트N·m</th><th>4.05초</th><th>방향 변화</th><th>위치 변화</th></tr>'+rows+'</table>'+link(loadpath,'가정 하중 검사')+'</section>')
    reached_loads=BASE/'reached_workstation_load_hypotheses/review.json'
    if reached_loads.exists():
        data=read(reached_loads);sources[reached_loads.relative_to(OUT).as_posix()]=digest(reached_loads)
        rows=''.join(f"<tr><td>{r['assumed_sliding_friction']:g}</td><td>{r['assumed_constant_moment_about_up_Nm']:+g}</td><td>{'통과' if r['conditional_hold_passed'] else '미달'}</td><td>{r['angle_drift_deg']:.3f}도</td></tr>" for r in data['cases'])
        sections.append('<section><h2>실제로 도달한 첫 파지의 가정 하중 비교</h2><p>이 검사는3015목표로 도달한 상태에서 시작했다. 위 맞은편 파지를 별도로 배치한 검사와 출발 상태가 다르다. 실제 왼손 하중은 여전히 측정하지 않았다.</p><table><tr><th>마찰</th><th>외부 모멘트N·m</th><th>유지</th><th>방향 변화</th></tr>'+rows+'</table>'+link(reached_loads,'첫 파지 유지 민감도')+'</section>')
    rf_path=BASE/'reached_workstation_RF_support/review.json'
    if rf_path.exists():
        rf=read(rf_path);sources[rf_path.relative_to(OUT).as_posix()]=digest(rf_path)
        sections.append('<section><h2>첫 파지에서 약지 지지 추가 비교</h2><p>'+f"약지 손끝·끝마디·중간마디의 유한 탐색{len(rf['proposals'])}개에서 하부 접촉과 윗단 여유를 함께 통과한 후보는{sum(r['static_clear_work_grip_candidate'] for r in rf['proposals'])}개였다. 기록한 실물 약지 각도는 원래 범위 안으로 잘라 탐색 시작점으로만 썼다. 제한을 넓히거나 이 유한 탐색으로 실물 약지가 닿을 수 없다고 결론내리지 않는다."+'</p>'+link(rf_path,'약지 추가 접촉 탐색')+'</section>')
    arrivals=BASE/'opposing_workspace_Z_arrivals/review.json';arrival_data=None
    if arrivals.exists():
        arrival_data=read(arrivals);sources[arrivals.relative_to(OUT).as_posix()]=digest(arrivals)
        rows=''.join(f"<tr><td>{html.escape(r['order'])}</td><td>{r['steps']}</td><td>{r['alternative_station_error_mm']:.3f}mm</td><td>{r['orientation_error_deg']:.3f}도</td><td>{'통과' if r['alternative_station_arrival_and_hold_passed'] else '미달'}</td></tr>" for r in arrival_data['rows'])
        sections.append('<section><h2>실제 모델 상태에서의 도달 경로</h2><p>회전 뒤 실제 모델 상태에서 시작해 큐브를 옮기지 않고 관절 목표를 재생했다. 네 경로는 큐브가 새 파지 위치로 이동하지 않아 미달했다. 이 때문에 배치 유지 통과만으로 실물 경로를 내보내지 않는다.</p><table><tr><th>관절 이동 순서</th><th>2틱/75ms 목표</th><th>새 위치와 차이</th><th>90도 자세 오차</th><th>새 작업 파지 도달</th></tr>'+rows+'</table>'+link(arrivals,'초기화 없는 도달 경로')+'</section>')
    bridges=[]
    for name,title in (('opposing_workspace_feedback_entry','새 파지 위치까지 모델 피드백 이동'),('opposing_workspace_next90','새 파지에서 다음90도 회전'),
        ('opposing_workspace_push_transport','실제 큐브 면을 거친 위치 이동'),('reached_workstation_next90','실제 첫 파지 위치에서 다음90도·복귀')):
        path=BASE/name/'review.json';done=path.exists()
        if not done:path=BASE/name/'progress.json'
        if not path.exists():continue
        data=read(path);sources[path.relative_to(OUT).as_posix()]=digest(path)
        final=data['final'] if done else data['actual'];steps=data['steps'] if done else data['step']
        status='조건부 경로 통과' if done and data['conditional_bridge_and_hold_passed'] else ('완료한 분량에서 미달' if done else '모델 계산 중인 스냅샷')
        bridges.append(dict(name=name,source=path.relative_to(OUT).as_posix(),completed=done,steps=steps,passed=bool(done and data['conditional_bridge_and_hold_passed']),
            final_orientation_error_deg=final['error_deg'],alternative_station_error_mm=final['center_target_error_mm']))
        sections.append('<section><h2>'+title+'</h2><p>'+status+f". 실행 {steps}목표, 최종 방향 오차 {final['error_deg']:.3f}도, 새 위치와의 차이 {final['center_target_error_mm']:.3f}mm. "+'미래 예측 복사본과 실제로 실행한 모델 상태를 구분하고 실제 구간에서는 큐브 상태를 초기화하지 않는다.</p>'+link(path,'피드백 기록')+'</section>')
    visualpath=FOLDER/'visual_manifest.json';figures=''
    if visualpath.exists():
        visual=read(visualpath);sources[visualpath.relative_to(OUT).as_posix()]=digest(visualpath)
        figures='<section><h2>저장 모델의 자세 비교</h2><p>파랑 검지 · 초록 중지 · 노랑 약지 · 빨강 엄지. 분홍 원통은 윗단 회전 영역 표시이며 수치 인증을 대체하지 않는다.</p><div class="grid">'+''.join('<figure><img src="'+html.escape(r['file'])+'"><figcaption>'+html.escape(r['label'])+'</figcaption></figure>' for r in visual['images'])+'</div></section>'
    result=dict(status='opposed_grip_preparation_snapshot',source_reports_sha256=sources,alternative_station_shift_from_actual_checkpoint_mm=shift,
        initialized_holds=holds,load_hypotheses_source=loadpath.relative_to(OUT).as_posix() if loads else None,load_hypotheses_passed=sum(r['conditional_hold_passed'] for r in loads['cases']) if loads else None,
        load_hypotheses_count=len(loads['cases']) if loads else 0,direct_arrival_paths_passed=sum(r['alternative_station_arrival_and_hold_passed'] for r in arrival_data['rows']) if arrival_data else None,
        bridge_studies=bridges,alternative_station_not_accepted_as_common_cycle_until_reached_both_times=True,old_canonical_failures_preserved=True,
        original_bounds_and_native_guards_preserved=True,physical_camera_registration_verified=False,force_hypotheses_not_calibrated_to_450mA=True,
        full_initial_placement_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    (FOLDER/'review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>맞은편 파지와 도달 경로</title><style>body{max-width:1100px;margin:30px auto;padding:0 18px;color:#173d36;background:#f4f8f6;font:16px/1.7 system-ui}section{background:white;border:1px solid #c4d8cc;padding:18px;margin:18px 0}.note{background:#fff1d2;padding:16px}table{border-collapse:collapse;width:100%;font-size:14px}th,td{padding:8px;border-bottom:1px solid #d3e0d8;text-align:left}.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}figure{margin:0}img{width:100%}a{color:#087463}@media(max-width:700px){.grid{grid-template-columns:1fr}table{display:block;overflow:auto}}</style><body><h1>맞은편 파지 후보와 회전 뒤 도달 경로</h1><p class="note">새 파지를 별도로 배치한 유지 검사와 실제 회전 뒤 도달 검사를 구분한다. 전체 회전→유지→다음90도→유지의 실물 준비는 아직 완료하지 않았다.</p>'''+figures+''.join(sections)+'''<p><a href="review.json">검사 범위·출처</a> · <a href="MODEL_TASK_SPEC.md">모델 작업 기준과 남은 연결</a> · <a href="../bridge_progress_20261110/index.html">앞선804목표와 연결 작업</a></p></body></html>'''
    (FOLDER/'index.html').write_text(page,encoding='utf-8')
    print(json.dumps(dict(report=(FOLDER/'index.html').relative_to(OUT).as_posix(),shift_mm=shift,loads_passed=result['load_hypotheses_passed'],load_count=result['load_hypotheses_count'],bridges=bridges)),flush=True)


if __name__=='__main__':main()
