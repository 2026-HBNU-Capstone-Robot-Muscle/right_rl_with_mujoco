"""Traceable illustrations of the current natural grasp and taught curvature."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,html
from pathlib import Path
import numpy as np,mujoco
from research_taught_grasp_20261106 import O,R,TEACH,details,sha,save_json
from research_rotation_body_hold_20261104 import restore
from hand_motor_mapping import ticks_to_sim
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display,extra,png

def main():
    out=O/'evidence';out.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(O/'model.mjb'));display_physical_cube(m);m.vis.global_.offwidth=1000;m.vis.global_.offheight=800
    for g in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[g])).name
        color=next((v for k,v in {'if_':[.22,.45,.83,1],'mf_':[.25,.65,.4,1],'rf_':[.88,.69,.14,1],'th_':[.83,.3,.3,1]}.items() if name.startswith(k)),None)
        if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
        if name=='cube':m.geom_matid[g]=-1;m.geom_rgba[g]=[.9,.9,.88,1]
    z=np.load(O/'baseline_end.npz');base=restore(m,z);taught=ticks_to_sim(np.array(json.loads(TEACH.read_bytes())['corrected_position_ticks']));poses=[('baseline','기존 파지의 기준 모델',base.qpos.copy(),'free_simulation')]
    for w in (.5,1.):
        q=base.qpos.copy();q[4:8]+=w*(taught[4:8]-q[4:8]);poses.append((f'taught{w:g}',f'중지 기록 {int(w*100)}% 반영: 모양 비교만',q,'pinned_cube_shape_comparison_only'))
    sliding=O/'sliding_contact/review.json'
    if sliding.exists():
        candidates=[]
        for case in json.loads(sliding.read_bytes())['cases']:
            for v in case['free_replays']:
                if v['dynamic']['safety_passed']:candidates.append((min(g for _,g in v['endpoint']['upper_gaps_mm']),case,v))
        if candidates:
            _,c,v=max(candidates,key=lambda x:x[0]);folder=sliding.parent/c['name'];q=np.load(folder/(v['order']+'_end.npz'))['qpos'];poses.append(('best_free','윗단 여유가 가장 큰 동적 후보: 검증 결과 별도',q,'free_simulation_research'))
    strict=O/'dynamic_refinement_strict/review.json'
    if strict.exists() and json.loads(strict.read_bytes())['dynamic']['safety_passed']:
        poses.append(('refined_free','미세 보정 후보: 하부 파지 미달로 미채택',np.load(strict.parent/'end.npz')['qpos'],'free_simulation_research'))
    renderer=mujoco.Renderer(m,height=720,width=960);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.distance=.32;cam.lookat[:]=[.14,-.025,-.015];opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0];d=mujoco.MjData(m);cube=m.geom('cube').id;images=[]
    for tag,title,q,kind in poses:
      for view,az,el in (('front',135,-28),('side',35,-28),('top',90,-78)):
        cam.azimuth=az;cam.elevation=el;d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(m,d);C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);renderer.update_scene(d,camera=cam,scene_option=opt);cube_display(renderer.scene,C,B,True)
        # Face colors are display-only overlays and do not alter geometry.
        for axis,sign,color in ((0,1,[.85,.16,.13,1]),(0,-1,[1.,.45,.07,1]),(1,1,[.12,.3,.85,1]),(1,-1,[.13,.7,.3,1]),(2,1,[.95,.95,.95,1]),(2,-1,[.95,.82,.08,1])):
            others=[i for i in range(3) if i!=axis]
            for u in (-1,0,1):
              for v in (-1,0,1):
                size=np.full(3,.0083);size[axis]=.00015;p=np.zeros(3);p[axis]=sign*.0277;p[others]=np.array([u,v])*.055/3;extra(renderer.scene,mujoco.mjtGeom.mjGEOM_BOX,size,C+B@p,B,color)
        file=f'{tag}_{view}.png';png(out/file,renderer.render());images.append(dict(file=file,title=title,view=view,evidence_kind=kind,sha256=sha(out/file)))
    renderer.close();save_json(out/'manifest.json',dict(model_sha256=sha(O/'model.mjb'),baseline_sha256=sha(O/'baseline_end.npz'),teaching_sha256=sha(TEACH),images=images,render_source_sha256=sha(Path(__file__)),devices_opened=False,physical_success=False))
    cards=''.join('<figure><img src="'+v['file']+'"><figcaption>'+html.escape(v['title'])+' / '+v['view']+'</figcaption></figure>' for v in images)
    text='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>중지 굽힘과 윗단 공간 검토</title><style>body{font:16px/1.75 system-ui;max-width:1250px;margin:30px auto;padding:0 20px;color:#24313b;background:#f5f7f9}.note{padding:16px;background:#fff0cd}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}figure{margin:0;background:white}img{width:100%;display:block}figcaption{padding:10px}@media(max-width:800px){.grid{grid-template-columns:1fr}}a{color:#1461a3}</style><h1>중지를 굽혀 윗단 공간 만들기</h1><p>파랑=검지, 초록=중지, 노랑=약지, 빨강=엄지. 주황색 반투명 원통은 55mm 큐브의 윗단을 90도 회전할 때 필요한 공간이다.</p><p class="note">실물 성공을 뜻하는 그림이 아니다. 중지 50% 및 100% 그림은 다른 관절과 큐브를 고정한 모양 비교로, 파지력이나 움직일 수 있는 경로의 증거가 아니다. 사용자 기록을 참고하되 토크 OFF로 손이 처진 각도를 그대로 새 목표로 사용하지 않는다.</p><p>기존 파지는 기준 모델에서 10.05초 유지됐지만, 중지와 엄지 일부가 윗단 공간에 들어와 있었다. 목표는 공간을 비운 상태에서도 큐브 하부 접촉과 자세가 유지되는 것이다. 관절 범위와 충돌 기준을 넓히지 않았다.</p><div class="grid">'''+cards+'''</div><p><a href="../progress.md">검사 결과와 다음 작업</a> / <a href="manifest.json">그림의 출처</a></p></html>'''
    (out/'index.html').write_text(text,encoding='utf-8');print(json.dumps(dict(images=len(images),output=str(out)),ensure_ascii=False),flush=True)
if __name__=='__main__':main()
