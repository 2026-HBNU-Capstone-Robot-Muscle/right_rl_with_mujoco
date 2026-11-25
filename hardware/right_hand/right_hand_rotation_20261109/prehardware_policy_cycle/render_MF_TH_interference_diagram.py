"""Scientific illustration of the exact recorded MF/TH native-guard stop.

Replay only saved local model commands. Draw the actual CAD collision hulls;
the location marker is enlarged, never the reported penetration depth.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from native_cycle import OUT,assets,restore,Guard,BOUND,ticks_to_sim,sha,save_json
from whole_hand_support import sample_model
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display,extra,png


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';trial=base/'ring_thumb_retreat_with_MF_feedback/force0.5'
    folder=base/'MF_TH_interference_diagram_20261111';folder.mkdir(exist_ok=True)
    review=json.loads((trial/'review.json').read_text());parent=json.loads((trial.parent/'review.json').read_text())
    source=OUT/parent['source'];assert sha(source)==parent['source_sha256']
    model,_,_,_,provenance=assets('active_ccd50');d=restore(model,dict(np.load(source,allow_pickle=False)));guard=Guard(model)
    goals=np.load(trial/'research_goals.npz',allow_pickle=False)['goals'];stop=[];ms=0
    for goal_index,command in enumerate(goals):
        d.ctrl[4:16]=ticks_to_sim(command)[4:16]
        for _ in range(75):
            mujoco.mj_step(model,d);ms+=1;stop=guard.check(d)
            if stop:break
        if stop:break
    assert goal_index+1==review['steps'] and stop==review['stop']
    expected=np.load(trial/'end_state.npz',allow_pickle=False)
    errors={key:float(np.max(abs(getattr(d,key)-expected[key]))) if expected[key].size else 0. for key in BOUND}
    assert max(errors.values())==0.,errors
    names=('cadtip_mf_th_mf_ds_0','cadtip_mf_th_th_ds_0');ids=[model.geom(n).id for n in names]
    contacts=[c for c in d.contact if {int(c.geom1),int(c.geom2)}==set(ids)]
    contact=min(contacts,key=lambda c:c.dist);point=contact.pos.copy();depth=-float(contact.dist)*1000
    assert abs(depth-review['terminal_guard']['self_pairs_mm']['|'.join(names)])<1e-12
    support=sample_model(model,d,guard.hand,model.geom('cube').id)
    physics={key:getattr(model,key).copy() for key in ('geom_size','geom_contype','geom_conaffinity','body_mass','body_inertia','jnt_range')}
    display=display_physical_cube(model);cube=model.geom('cube').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    colors={'if_':[.25,.46,.84,.22],'mf_':[.12,.65,.35,1.],'rf_':[.92,.71,.12,.4],'th_':[.89,.23,.24,1.]}
    for i in range(model.ngeom):
        body=model.body(int(model.geom_bodyid[i])).name or ''
        for prefix,color in colors.items():
            if body.startswith(prefix):model.geom_matid[i]=-1;model.geom_rgba[i]=color
    model.vis.global_.offwidth=1000;model.vis.global_.offheight=800
    renderer=mujoco.Renderer(model,height=520,width=680);option=mujoco.MjvOption();option.geomgroup[:]=[1,1,1,0,0,0]
    camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE
    direction=point-C
    if np.linalg.norm(direction)<.001:direction=B[:,2]
    direction=direction/np.linalg.norm(direction)
    def view(target,distance,desired):
        camera.lookat[:]=target;camera.distance=distance;choices=[]
        for az in range(0,360,15):
            for el in (-60,-40,-20,0,20,40,60):
                camera.azimuth=az;camera.elevation=el;renderer.update_scene(d,camera=camera,scene_option=option)
                pos=(renderer.scene.camera[0].pos+renderer.scene.camera[1].pos)/2-target
                choices.append((float(pos@desired/np.linalg.norm(pos)),az,el))
        _,camera.azimuth,camera.elevation=max(choices)
    def project(p):
        gl0,gl1=renderer.scene.camera;pos=(gl0.pos+gl1.pos)/2
        f=(gl0.forward+gl1.forward)/2;f=f/np.linalg.norm(f);up=(gl0.up+gl1.up)/2;up=up/np.linalg.norm(up);right=np.cross(f,up);v=p-pos;z=float(v@f)
        focal=260/np.tan(np.radians(float(model.vis.global_.fovy))/2)
        return (340+float(v@right)*focal/z,260-float(v@up)*focal/z)
    points={};images={}
    try:
        view(C,.29,direction+.25*np.array([0.,0.,1.]))
        renderer.update_scene(d,camera=camera,scene_option=option);cube_display(renderer.scene,C,B,False)
        extra(renderer.scene,mujoco.mjtGeom.mjGEOM_SPHERE,[.0022,0,0],point,np.eye(3),[1.,.05,.72,1.])
        points['context']=project(point);images['context']=renderer.render().copy();png(folder/'context.png',images['context'])
        # Show only the two exact meshes named by the recorded guard. Convex
        # hull rendering exposes the native collision shape, not CAD cavities.
        model.geom_group[:]=5
        for gid,color in zip(ids,([.12,.65,.35,.9],[.89,.23,.24,.9])):
            model.geom_group[gid]=2;model.geom_matid[gid]=-1;model.geom_rgba[gid]=color
        option.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL]=1
        pair_center=(d.geom_xpos[ids[0]]+d.geom_xpos[ids[1]])/2
        pair_axis=d.geom_xpos[ids[1]]-d.geom_xpos[ids[0]];pair_axis/=np.linalg.norm(pair_axis)
        desired=direction-pair_axis*float(direction@pair_axis)
        if np.linalg.norm(desired)<.1:desired=np.cross(pair_axis,[0.,0.,1.])
        desired/=np.linalg.norm(desired)
        view(point*.65+pair_center*.35,.17,desired)
        renderer.update_scene(d,camera=camera,scene_option=option)
        extra(renderer.scene,mujoco.mjtGeom.mjGEOM_SPHERE,[.0009,0,0],point,np.eye(3),[1.,.05,.72,1.])
        points['pair']=project(point);images['pair']=renderer.render().copy();png(folder/'pair.png',images['pair'])
    finally:renderer.close()
    for key,value in physics.items():assert np.array_equal(getattr(model,key),value),key
    font_path='/mnt/c/Windows/Fonts/malgun.ttf';bold_path='/mnt/c/Windows/Fonts/malgunbd.ttf'
    def font(size,bold=False):return ImageFont.truetype(bold_path if bold else font_path,size)
    canvas=Image.new('RGB',(1440,940),'#f4f7fa');draw=ImageDraw.Draw(canvas)
    draw.text((36,24),'중지·엄지의 어느 부위가 걸렸나?',font=font(34,True),fill='#142c3f')
    draw.text((36,77),'저장된 중단 순간을 정확히 재생한 모델 그림 · 실물 사진이 아님',font=font(21),fill='#536575')
    for x,label,key in ((30,'① 손 전체: 검지·약지는 반투명 표시','context'),(730,'② 판정된 두 충돌 형상만 확대','pair')):
        draw.rounded_rectangle((x,125,x+680,720),radius=12,fill='white',outline='#d1dce5',width=2)
        draw.text((x+16,140),label,font=font(23,True),fill='#233b50');canvas.paste(Image.fromarray(images[key]),(x,190))
        px,py=points[key];cx=x+px;cy=190+py
        if x+10<cx<x+670 and 200<cy<700:
            draw.ellipse((cx-18,cy-18,cx+18,cy+18),outline='#ce168e',width=4)
    y=742
    for x,color,text in ((38,'#20a659','중지 끝마디'),(345,'#e33b3d','엄지 끝마디'),(660,'#ecb61f','약지'),(870,'#4075d6','검지'),(1090,'#ff0db8','판정 위치')):
        draw.ellipse((x,y+5,x+20,y+25),fill=color);draw.text((x+30,y-2),text,font=font(23,True),fill='#203749')
    draw.text((38,790),f'기록된 겹침 판정: {depth:.6f} mm  (약 {depth*1000:.3f} μm)',font=font(25,True),fill='#a51e67')
    draw.text((38,837),'분홍 표식은 위치를 보여 주려고 확대함. 미세한 겹침 두께는 이 해상도로 구분할 수 없음.',font=font(20),fill='#465a6b')
    draw.text((38,875),'손 지지는 유지된 순간이며, 이 모델 판정만으로 실물 충돌을 확정하지 않음.',font=font(20),fill='#465a6b')
    canvas.save(folder/'interference_diagram.png')
    meta=dict(status='exact_saved_native_stop_illustrated',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),goals=(trial/'research_goals.npz').relative_to(OUT).as_posix(),goals_sha256=sha(trial/'research_goals.npz'),model_provenance=provenance,
        replay_steps=goal_index+1,replay_simulation_ms=ms,bound_state_errors=errors,guard=guard.last,pair=list(names),penetration_mm=depth,contact_world_m=point.tolist(),supported_sample=support['supported_sample'],
        meshes=[dict(geom=names[i],body=model.body(int(model.geom_bodyid[g])).name,mesh=model.mesh(int(model.geom_dataid[g])).name) for i,g in enumerate(ids)],
        exact_native_contact_pose_used_without_forward_recomputation=True,isolated_panel_displays_native_convex_hulls=True,context_IF_RF_transparent_for_visibility=True,position_marker_enlarged_not_penetration_depth=True,physics_properties_unchanged=True,display=display,
        source_code_sha256=sha(Path(__file__)),files_sha256={n:sha(folder/n) for n in ('context.png','pair.png','interference_diagram.png')},devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'manifest.json',meta)
    (folder/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>중지·엄지 모델 간섭 위치</title><style>body{margin:20px auto;max-width:1440px;font:18px/1.6 system-ui;background:#f4f7fa}img{width:100%;height:auto}a{color:#1762a0}</style><h1>중지·엄지 모델 간섭 위치</h1><img src="interference_diagram.png" alt="손 전체와 중지 엄지 끝마디 충돌 형상 확대 비교"><p>모델의 실제 중단 상태를 재생해 표시했다. 오른쪽은 기록에 나타난 두 충돌 형상만 따로 표시한 그림이다. 분홍 점은 판정 위치를 확대 표시하며 겹침 두께를 과장한 그림은 아니다.</p><p><a href="manifest.json">재생 일치·출처·표시 설정</a> · <a href="../ring_front_thumb_progress_20261111/index.html">전체 준비 검토</a></p></html>',encoding='utf-8')
    print(json.dumps(dict(diagram=(folder/'interference_diagram.png').relative_to(OUT).as_posix(),replay_goals=goal_index+1,ms=ms,depth_mm=depth,bound_errors=errors,point=point.tolist(),projected=points,meshes=meta['meshes'])),flush=True)


if __name__=='__main__':main()
