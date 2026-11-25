"""Render archived/free model states as scientific figures without devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json,hashlib
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display,extra,png

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261109/middle_probe_actual_review/model_figures'
SOURCE=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    model=mujoco.MjModel.from_binary_path('original.mjb',assets={'original.mjb':SOURCE.read_bytes()})
    display_physical_cube(model);model.vis.global_.offwidth=1000;model.vis.global_.offheight=800
    for g in range(model.ngeom):
        name=model.body(int(model.geom_bodyid[g])).name
        color=next((value for key,value in {'if_':[.22,.45,.83,1],'mf_':[.25,.65,.4,1],'rf_':[.88,.69,.14,1],'th_':[.83,.3,.3,1]}.items() if name.startswith(key)),None)
        if color is not None:model.geom_matid[g]=-1;model.geom_rgba[g]=color
    renderer=mujoco.Renderer(model,height=720,width=960);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.distance=.32;cam.lookat[:]=[.14,-.025,-.015];cam.azimuth=135;cam.elevation=-28
    opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0];cube=model.geom('cube').id
    sources={
        'original_grip':'middle_probe_reconstruction_linux/checkpoint_927.npz',
        'middle_fold':'middle_probe_reconstruction_linux/checkpoint_1042.npz',
        'lower_body_candidate':'lower_index_refinement/height-6/end.npz',
    }
    rows={}
    for name,path in sources.items():
        source=ROOT/'right_hand_rotation_20261109'/path;data=restore(model,np.load(source,allow_pickle=False));C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3)
        renderer.update_scene(data,camera=cam,scene_option=opt);cube_display(renderer.scene,C,B,True)
        for axis,sign,color in ((0,1,[.85,.16,.13,1]),(0,-1,[1.,.45,.07,1]),(1,1,[.12,.3,.85,1]),(1,-1,[.13,.7,.3,1]),(2,1,[.95,.95,.95,1]),(2,-1,[.95,.82,.08,1])):
            others=[i for i in range(3) if i!=axis]
            for u in (-1,0,1):
              for v in (-1,0,1):
                size=np.full(3,.0083);size[axis]=.00015;p=np.zeros(3);p[axis]=sign*.0277;p[others]=np.array([u,v])*.055/3;extra(renderer.scene,mujoco.mjtGeom.mjGEOM_BOX,size,C+B@p,B,color)
        target=OUT/(name+'.png');png(target,renderer.render())
        rows[name]={'source':str(source.relative_to(ROOT)),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'image_sha256':hashlib.sha256(target.read_bytes()).hexdigest()}
    renderer.close();(OUT/'manifest.json').write_text(json.dumps({'scientific_model_render_not_physical_photo':True,'source_model_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'states':rows,'devices_opened':False},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'figures':list(rows),'devices_opened':False}))


if __name__=='__main__':main()
