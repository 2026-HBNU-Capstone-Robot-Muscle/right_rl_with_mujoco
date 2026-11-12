"""Build a concise, evidence-linked local review of the timed offline session."""
import json,hashlib,html
from pathlib import Path
from datetime import datetime
import cv2 as cv
import numpy as np

R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261104/rotation_hold_session'
def read(name):return json.loads((O/name).read_bytes())
def main():
    replay=read('full_replay_review.json');stress=read('torsion_stress_review.json');sweep=read('whole_hand_upper_sweep_review.json');tips=read('upper_layer_sweep_review.json');paths=read('third_contact_path_batch.json')
    root_fine=read('fine_support_sweep.json') if (O/'fine_support_sweep.json').exists() else []
    groups={}
    for row in stress['cases']:
        groups.setdefault(row['source'],[]).append(row)
    torque={name:dict(cases=len(rows),passed=sum(r['hold_probe_passed'] for r in rows),safety_passed=sum(r['safety_passed'] for r in rows),max_drift_deg=max(r['max_yaw_drift_deg'] for r in rows),max_shift_mm=max(r['max_center_shift_mm'] for r in rows)) for name,rows in groups.items()}
    dynamic=[c['dynamic'] for c in replay['cases'] if c.get('dynamic')]
    protected=json.loads((R/'right_hand_rotation_20261104/resume_review.json').read_bytes())['protected_sources_unchanged']
    protected_check={p:hashlib.sha256((R/p).read_bytes()).hexdigest()==digest for p,digest in protected.items()}
    candidate=np.load(O/'body_hold_research_candidate.npz');lohi=None
    steps=int(np.max(abs(np.diff(np.vstack([candidate['start'],candidate['goals']]),axis=0))))
    summary=dict(status='offline_body_hold_candidate_found_not_operator_ready',started_utc='2026-11-04 10:26:04 UTC',updated_at=datetime.now().astimezone().isoformat(),goal='Whole-cube approximately90deg reorientation followed by stable body grasp',whole_cube90_verified=False,physical_hold_verified=False,hardware_execution_allowed=False,assistant_motor_writes=0,candidate='body_hold_research_candidate.npz',candidate_sha256=hashlib.sha256((O/'body_hold_research_candidate.npz').read_bytes()).hexdigest(),candidate_goal_step_ticks=steps,continuous_replay_cases=len(replay['cases']),continuous_replay_passed=sum(c['initial_placement']['passed'] and c.get('dynamic',{}).get('safety_passed',False) for c in replay['cases']),initial_seating_failures=sum(not c['initial_placement']['passed'] for c in replay['cases']),whole_yaw_range_deg=[min(r['whole_yaw_deg'] for r in dynamic),max(r['whole_yaw_deg'] for r in dynamic)],final_hold_yaw_drift_range_deg=[min(r['hold_drift_deg'] for r in dynamic),max(r['hold_drift_deg'] for r in dynamic)],final_hold_center_shift_max_mm=max(r['hold_shift_mm'] for r in dynamic),assumed_torque_probe_Nm=.002,actual_left_hand_torque_measured=False,torque_probe=torque,both_fingertips_upper_sweep_clear=next(r['both_tips_clear_through_90deg'] for r in tips['cases'] if r['name']=='lower_pair_m20_t16'),whole_collision_hand_upper_sweep_clear=sweep['all_existing_hand_geoms_clear'],remaining_sweep_geometries=[r['geom'] for r in sweep['interfering']],third_contact_orders_tested=len(paths),third_contact_orders_safety_passed=sum(r['safety_passed'] for r in paths),fine_support_candidates=root_fine,protected_sources_verified=protected_check,repeat_static_capture_requested=False,next_user_motion_command=None,next_work='Reconfigure middle-finger body clearance while retaining body support; revisit missing90deg and initial low-gain seating before an operator trial.')
    assert all(protected_check.values()) and steps<=2
    (O/'session_summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    manifest=read('views/manifest.json');frames=manifest['frames'];video=O/'views/body_hold_suffix.mp4'
    tmp=R/'body_hold_review_video_temp.mp4';writer=cv.VideoWriter(str(tmp.relative_to(R)),cv.VideoWriter_fourcc(*'mp4v'),1/.15,(768,576))
    if writer.isOpened():
        for frame in frames:
            p=O/'views/frames'/frame['image'];im=cv.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv.IMREAD_COLOR);writer.write(im)
        writer.release()
        if tmp.exists() and tmp.stat().st_size>1000:tmp.replace(video)
    else:writer.release()
    fine_rows=''.join('<tr><td>'+str(r['raise_ticks'])+'</td><td>'+('통과' if r['initial_placement']['passed'] else '초기 배치 이탈')+'</td><td>'+('—' if not r.get('dynamic') else f"{r['dynamic']['whole_yaw_deg']:.1f}°")+'</td></tr>' for r in root_fine)
    torque_rows=''.join(f'<tr><td>{html.escape(name)}</td><td>{r["passed"]}/{r["cases"]}</td><td>{r["max_drift_deg"]:.2f}°</td><td>{r["max_shift_mm"]:.2f}mm</td></tr>' for name,r in torque.items())
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>회전 후 파지 검토</title><style>body{max-width:1080px;margin:24px auto;padding:0 18px;background:#f5f7fa;color:#203040;font:17px/1.7 system-ui}article{background:white;padding:20px;margin:16px 0;border-radius:10px}img,video{max-width:100%;height:auto}.pair{display:flex;gap:12px}.pair figure{margin:0;width:49%}table{border-collapse:collapse;width:100%}th,td{padding:8px;border-bottom:1px solid #ccd;text-align:left}.notice{border-left:5px solid #b66}input{width:90%}button{font:inherit;padding:5px 12px}</style>
<h1>회전 뒤 큐브 몸통을 잡는 경로 검토</h1><article class="notice"><p><strong>시뮬레이션 연구 결과입니다. 실물 90도 회전과 안정 파지는 아직 완료되지 않았습니다.</strong> 새 실물 실행 명령은 없습니다. 방금 마친 정지 배치 촬영은 반복하지 않아도 됩니다.</p></article>
<article><h2>확보한 후보</h2><p>기존 회전 중 약66.6도 상태에서 중지와 엄지 접촉을 아래로 옮기는 경로를 만들었습니다. 기본 모델의 마지막 방향은 약63.8도입니다. 방향을 약90도로 완성하는 과제는 남아 있습니다.</p><p>원래 배치부터 새 파지까지9조건을 연속 재생한 결과,8조건에서 지지를 유지했고1조건은 움직이기 전 배치가 이탈했습니다. 통과한 조건의 마지막 고정 구간에서 방향 변화는 최대1.23도, 중심 이동은 최대0.92mm였습니다. 이는 가설 물성의 모델 결과이며 실물 힘 측정이 아닙니다.</p><div class="pair"><figure><img src="views/before_front.png"><figcaption>기존 회전 끝 모델</figcaption></figure><figure><img src="views/body_hold_front.png"><figcaption>아래쪽 접촉으로 바꾼 모델</figcaption></figure></div><p>파랑=검지, 초록=중지, 노랑=약지, 빨강=엄지. 회색 큐브와 로봇은 저장된 자유물체 시뮬레이션입니다.</p></article>
<article><h2>새 파지로 옮기는 약12.5초 구간</h2><p>이 영상은 전체90도 회전 영상이 아닙니다. 약66도 상태부터 접촉점을 내리는 후반부만 보여줍니다.</p><video controls preload="metadata" src="views/body_hold_suffix.mp4"></video><details><summary>프레임으로 보기</summary><img id="frame" src="views/frames/frame_0000.png"><p><button id="play">재생</button> <output id="stamp"></output></p><input id="slider" type="range" min="0" max="''' + str(len(frames)-1) + '''" value="0"></details></article>
<article><h2>작은 비틀림 가설 비교</h2><p>임의의±0.002Nm를 강체 큐브에 가한 진단입니다. 실제 왼손 힘이나 독립적으로 움직이는 루빅 큐브 레이어를 재현한 시험은 아닙니다. 이득3종류와 마찰3종류를 조합해 각9조건을 확인했습니다.</p><table><tr><th>후보</th><th>유지 기준 통과</th><th>최대 방향 변화</th><th>최대 이동</th></tr>''' + torque_rows + '''</table></article>
<article><h2>통합 전에 남은 간섭</h2><p>아래쪽 파지 후보는 두 손끝이 윗층90도 회전 공간을 비웠습니다. 손 전체56개 충돌 형상까지 검사하면 중지의 중간 및 끝 모터 몸통 쪽에서 간섭이 남습니다. 손끝만 확인해서 왼손 회전이 가능하다고 판정하지 않았습니다.</p><p>검지를 세 번째 접촉으로 옮기는7순서 중5순서는 기존 충돌·추종 한도를 지켰고, 마디부터 굽히는2순서는 추종 한도에서 거부했습니다. 세 번째 접촉은 아직 주 파지를 대체할 만큼 검증되지 않았습니다.</p></article>
<article><h2>받침 각도 주변 추가 비교</h2><table><tr><th>뿌리 추가 틱</th><th>초기 배치</th><th>마지막 방향 변화</th></tr>''' + fine_rows + '''</table></article>
<article><h2>다음 작업</h2><p>큐브 몸통 지지를 유지하면서 중지 몸통도 윗층 회전 공간에서 빼는 재파지를 설계합니다. 부족한 전체 회전과 초기 배치 이탈도 해결한 뒤 실물 실행을 준비합니다. 기존 모터 구동력, 중단 기준, 관절 한계와 원본 시퀀스는 보존했습니다.</p><a href="session_summary.json">전체 수치 요약</a> · <a href="full_replay_review.json">배치부터 연속 재생</a> · <a href="whole_hand_upper_sweep_review.json">손 전체 간섭 근거</a></article>
<script>const frames=''' + json.dumps(frames) + ''';const s=document.getElementById('slider'),im=document.getElementById('frame'),stamp=document.getElementById('stamp');function show(){const f=frames[+s.value];im.src='views/frames/'+f.image;stamp.textContent=f.simulated_seconds.toFixed(2)+'초';}s.oninput=show;let timer=null;document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;return;}timer=setInterval(()=>{s.value=(+s.value+1)%frames.length;show();},150)};show();</script></html>'''
    (O/'index.html').write_text(page,encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('fine_support_candidates','protected_sources_verified')},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
