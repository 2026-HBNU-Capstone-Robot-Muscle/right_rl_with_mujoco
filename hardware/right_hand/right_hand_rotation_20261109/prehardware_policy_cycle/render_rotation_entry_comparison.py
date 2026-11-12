"""Render scientific model views of the actual and desired rotation entry."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from native_cycle import OUT,assets,restore,sha,save_json
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/reposition_then_second_rotation_20261111'
    report=json.loads((folder/'entry_state_comparison.json').read_text(encoding='utf-8'))
    model,_,_,_,provenance=assets('active_ccd50');display_physical_cube(model)
    for gid in range(model.ngeom):
        body=model.body(int(model.geom_bodyid[gid])).name or ''
        for prefix,color in {'if_':[.22,.45,.83,1.],'mf_':[.25,.65,.4,1.],'rf_':[.88,.69,.14,1.],'th_':[.83,.3,.3,1.]}.items():
            if body.startswith(prefix):model.geom_matid[gid]=-1;model.geom_rgba[gid]=color
    model.vis.global_.offwidth=1000;model.vis.global_.offheight=800
    renderer=mujoco.Renderer(model,height=600,width=800);option=mujoco.MjvOption();option.geomgroup[:]=[1,1,1,0,0,0]
    camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE;camera.distance=.34;camera.azimuth=130;camera.elevation=-25
    camera.lookat[:]=np.mean([r['cube_center_world_m'] for r in report['states']],axis=0)
    font=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',20);bold=ImageFont.truetype('/mnt/c/Windows/Fonts/malgunbd.ttf',24);panels=[]
    try:
        for i,row in enumerate(report['states']):
            d=restore(model,dict(np.load(OUT/row['path'],allow_pickle=False)));cube=model.geom('cube').id
            renderer.update_scene(d,camera=camera,scene_option=option);cube_display(renderer.scene,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),False)
            panel=Image.new('RGB',(800,720),'#f5f8f8');panel.paste(Image.fromarray(renderer.render()),(0,70));draw=ImageDraw.Draw(panel)
            draw.text((16,8),'현재 낮은 마찰의 첫 회전 후 파지' if i==0 else '기존 두 번째 회전의 성공 시작 상태',font=bold,fill='#173d40')
            draw.text((16,42),'동일 카메라 · 검지 파랑 / 중지 초록 / 약지 노랑 / 엄지 빨강',font=font,fill='#173d40')
            draw.text((16,682),'모델 상태 비교. 오른쪽 상태로 순간이동시키거나 실물 실행하지 않음.',font=font,fill='#173d40')
            panel.save(folder/('actual_entry.png' if i==0 else 'reference_entry.png'));panels.append(panel)
    finally:renderer.close()
    combined=Image.new('RGB',(1600,800),'white');combined.paste(panels[0],(0,0));combined.paste(panels[1],(800,0));draw=ImageDraw.Draw(combined)
    draw.text((20,740),f"큐브 중심 차이 {report['cube_center_difference_mm']:.2f} mm / 방향 차이 {report['cube_orientation_difference_deg']:.2f}° / 관절 목표 최대 차이 {report['maximum_motor_command_difference_ticks']:.0f}틱",font=bold,fill='#173d40')
    combined.save(folder/'entry_comparison.png')
    save_json(folder/'visual_manifest.json',dict(status='scientific_entry_comparison',source_report_sha256=sha(folder/'entry_state_comparison.json'),same_camera=True,model_provenance=provenance,render_is_not_a_motion_or_hardware_test=True,devices_opened=0))
    print('entry_comparison.png',flush=True)


if __name__=='__main__':main()
