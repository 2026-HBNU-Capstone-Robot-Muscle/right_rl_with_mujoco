"""Scientific saved-state comparison; display changes never affect dynamics."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from native_cycle import OUT,assets,restore,sha,save_json
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'first_to_next_contact_bridge_20261111';folder.mkdir(exist_ok=True)
    states=[('first90','첫 회전 · 윗단 공간과 유지 통과',base/'initial_to_reconfigured_first90/end_state.npz',90),
        ('next_partial','다음 회전 중간 · 최종 목표 미달',base/'actual_A_next_MF_yaw_pressure/force0.5/end_state.npz',180),
        ('thumb_arrived','엄지 재배치 후 · 손 전체 지지 유지',base/'actual_A_next_counter_thumb_arrival/axis0_sign+1_force0.6/end_state.npz',180),
        ('single_push_failed','중지만 계속 밀었던 실패 · 모델 지지 이탈',base/'actual_A_next_MF_yaw_pressure_continued/force0.5/end_state.npz',180)]
    teacher=base/'actual_A_next_two_contact_centered_teacher/end_state.npz'
    if teacher.exists() and (teacher.parent/'review.json').exists():states.append(('centered_teacher','엄지 재배치 뒤 위치·회전 함께 보정',teacher,180))
    holds=base/'actual_A_second_turn_clear_hold/review.json'
    if holds.exists():
        rows=json.loads(holds.read_text())['rows'];best=min(rows,key=lambda r:(not r['rotation_and_clear_hold_at_reached_location_passed'],-r['upper_certified_samples'],r['final180_error_deg']))
        states.append(('second_hold','두 번째 회전 뒤 자세·공간 유지 비교',OUT/best['path']/'end_state.npz',180))
    model,initial,_,_,provenance=assets('active_ccd50');display=display_physical_cube(model);cube=model.geom('cube').id
    colors={'if_':[.22,.45,.83,1.],'mf_':[.25,.65,.4,1.],'rf_':[.88,.69,.14,1.],'th_':[.83,.3,.3,1.]}
    for gid in range(model.ngeom):
        name=model.body(int(model.geom_bodyid[gid])).name or ''
        for prefix,color in colors.items():
            if name.startswith(prefix):model.geom_matid[gid]=-1;model.geom_rgba[gid]=color
    model.vis.global_.offwidth=1000;model.vis.global_.offheight=800
    renderer=mujoco.Renderer(model,height=600,width=800);option=mujoco.MjvOption();option.geomgroup[:]=[1,1,1,0,0,0]
    camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE;camera.distance=.34;camera.azimuth=130;camera.elevation=-25
    reference=restore(model,dict(np.load(states[0][2],allow_pickle=False)));station=reference.geom_xpos[cube].copy();camera.lookat[:]=station
    font=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',20);bold=ImageFont.truetype('/mnt/c/Windows/Fonts/malgunbd.ttf',24)
    images=[];panels=[]
    try:
        for name,label,path,target_deg in states:
            d=restore(model,dict(np.load(path,allow_pickle=False)));C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
            renderer.update_scene(d,camera=camera,scene_option=option);cube_display(renderer.scene,C,B,False)
            panel=Image.new('RGB',(800,740),'#f5f8f8');panel.paste(Image.fromarray(renderer.render()),(0,90))
            draw=ImageDraw.Draw(panel);error=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],target_deg));center=float(np.linalg.norm(C-station)*1000)
            draw.text((18,8),label,fill='#173d40',font=bold)
            draw.text((18,44),f'{target_deg}° 목표 자세 오차 {error:.2f}° · 첫 유지 위치와 {center:.2f} mm',fill='#173d40',font=font)
            draw.text((18,704),'검지 파랑 · 중지 초록 · 약지 노랑 · 엄지 빨강 | 모델 그림, 실물 사진 아님',fill='#173d40',font=font)
            file=name+'.png';panel.save(folder/file);panels.append(panel)
            images.append(dict(file=file,label=label,source=path.relative_to(OUT).as_posix(),source_sha256=sha(path),target_orientation_deg=target_deg,orientation_error_deg=error,station_error_mm=center))
    finally:renderer.close()
    combined=Image.new('RGB',(1600,740*((len(panels)+1)//2)),'white')
    for i,panel in enumerate(panels):combined.paste(panel,((i%2)*800,(i//2)*740))
    combined.save(folder/'cycle_state_comparison.png')
    save_json(folder/'visual_manifest.json',dict(status='saved_whole_hand_cycle_model_views',images=images,combined='cycle_state_comparison.png',model_provenance=provenance,display=display,source_code_sha256=sha(Path(__file__)),same_world_camera_and_first_station_reference=True,rendered_geometry_recomputed_from_saved_postintegration_qpos=True,render_metrics_may_differ_from_last_native_preintegration_contact_pose=True,scientific_model_renders_not_photos=True,failed_state_not_a_successful_hold=True,devices_opened=0))
    print('whole_hand_images='+str(len(images)),flush=True)


if __name__=='__main__':main()
