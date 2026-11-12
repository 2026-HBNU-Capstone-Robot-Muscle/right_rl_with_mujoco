"""Scientific model renders of actual checkpoint and initialized opposed grip."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,save_json,sha
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display,extra,png


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'opposing_grip_review_20261110';folder.mkdir(exist_ok=True)
    model,_,_,_,provenance=assets('active_ccd50');display=display_physical_cube(model);cube=model.geom('cube').id
    model.vis.global_.offwidth=1000;model.vis.global_.offheight=800
    colors={'if_':[.22,.45,.83,1.],'mf_':[.25,.65,.4,1.],'rf_':[.88,.69,.14,1.],'th_':[.83,.3,.3,1.]}
    for geom in range(model.ngeom):
        body=model.body(int(model.geom_bodyid[geom])).name or ''
        for prefix,color in colors.items():
            if body.startswith(prefix):model.geom_matid[geom]=-1;model.geom_rgba[geom]=color
    states=[('checkpoint','회전 뒤 실제 모델 상태: 새 파지에 도달하기 전',base/'first90_RF_shape_before_adduction/RF_park_MF_then_RF_shape_then_adduct/phase0_end_state.npz'),
        ('initialized90','별도 배치한 맞은편 파지: 유지 검사 뒤',base/'opposing_workspace_Z_seed122/free_hold_tests/force0p8/end_state.npz'),
        ('initialized180','180도 방향으로 별도 배치한 같은 위치의 파지',base/'opposing_workspace_Z_seed122/free_hold_tests_yaw180/force0p8/end_state.npz')]
    if (base/'initial_to_reached_workstation90/end_state.npz').exists():
        states.insert(0,('firstreached','초기 모델 배치부터3015목표로 도달한 첫90도 파지',base/'initial_to_reached_workstation90/end_state.npz'))
    camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE;camera.lookat[:]=[.145,-.04,-.013];camera.distance=.27
    option=mujoco.MjvOption();option.geomgroup[:]=[1,1,1,0,0,0];renderer=mujoco.Renderer(model,height=600,width=800);images=[]
    try:
        for name,label,path in states:
            if not path.exists():continue
            data=restore(model,dict(np.load(path,allow_pickle=False)));C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3)
            for view,azimuth in (('front',130),('side',35)):
                camera.azimuth=azimuth;camera.elevation=-30;renderer.update_scene(data,camera=camera,scene_option=option);cube_display(renderer.scene,C,B,False)
                for axis,sign,color in ((0,1,[.85,.16,.13,1]),(0,-1,[1,.45,.07,1]),(1,1,[.12,.3,.85,1]),(1,-1,[.13,.7,.3,1]),(2,1,[.95,.95,.95,1]),(2,-1,[.95,.82,.08,1])):
                    other=[i for i in range(3) if i!=axis]
                    for u in (-1,0,1):
                        for v in (-1,0,1):
                            size=np.full(3,.0083);size[axis]=.00015;point=np.zeros(3);point[axis]=sign*.0277;point[other]=np.array([u,v])*.055/3
                            extra(renderer.scene,mujoco.mjtGeom.mjGEOM_BOX,size,C+B@point,B,color)
                extra(renderer.scene,mujoco.mjtGeom.mjGEOM_CYLINDER,[.0275*np.sqrt(2),.055/6,0],C+B@np.array([0.,-.055/3,0.]),np.column_stack([B[:,0],B[:,2],-B[:,1]]),[.85,.2,.65,.2])
                filename=name+'_'+view+'.png';png(folder/filename,renderer.render());images.append(dict(file=filename,label=label+' · '+('정면' if view=='front' else '측면'),source=path.relative_to(OUT).as_posix(),source_sha256=sha(path)))
    finally:renderer.close()
    save_json(folder/'visual_manifest.json',dict(status='scientific_native_model_render',images=images,model_provenance=provenance,display=display,source_sha256=sha(Path(__file__)),
        exact_geometry_render_not_photograph=True,sweep_is_visual_overlay_only=True,initialized_states_not_arrival_evidence=True,devices_opened=0))
    print('model_images='+str(len(images)),flush=True)


if __name__=='__main__':main()
