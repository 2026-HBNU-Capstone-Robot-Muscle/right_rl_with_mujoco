"""Render front-ring/back-thumb hypotheses; colors encode the user's face words.

White front is assigned to local+X for this drawing. It is a qualitative model
frame choice, not camera/color registration of the actual cube.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,sha,save_json
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display,extra,png


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'ring_front_thumb_progress_20261111';folder.mkdir(exist_ok=True)
    states=[('exact_static','기록 약지4값 그대로: 정적 배치 가설',base/'ring_thumb_photo_prior_seed432_exact_static/axis0_ring+1/proposal.npz'),
        ('native_closed','원래 범위 안의 비교 자세: 약지 받침·엄지 추가 닫기 후',base/'ring_thumb_only_closure/custom_settled_grip_extra0p8/end_state.npz'),
        ('turn_checkpoint','다음 회전 중간: 관절·손바닥 접촉 여유가 적은 상태',base/'ring_thumb_turn_checkpoint/state.npz'),
        ('pressure_released','손바닥 여유 확보 뒤 약지·엄지 압박을 줄인 상태',base/'ring_thumb_release_after_tip_clear/release1.6_retreat0/end_state.npz'),
        ('pair_retreat','약지·엄지 손바닥 여유 확보와 ID14 지령 3도 후퇴 후: 중간 동작',base/'ring_thumb_pair_clear_retreat_v2/trial2/end_state.npz'),
        ('further_regrasp8','추가 재파지 후: ID14 지령8도 더 후퇴한 중간 동작',base/'ring_thumb_further_regrasp8/trial2/end_state.npz'),
        ('MF_model_feedback','중지 임시 하중 확보: 정확한 모델 거리·힘 피드백 가정',base/'ring_thumb_MF_support_feedback_v2/force0.5/end_state.npz')]
    model,_,_,_,provenance=assets('active_ccd50');display=display_physical_cube(model);cube=model.geom('cube').id
    for i in range(model.ngeom):
        body=model.body(int(model.geom_bodyid[i])).name or ''
        for prefix,color in {'if_':[.22,.45,.83,1.],'mf_':[.25,.65,.4,1.],'rf_':[.88,.69,.14,1.],'th_':[.83,.3,.3,1.]}.items():
            if body.startswith(prefix):model.geom_matid[i]=-1;model.geom_rgba[i]=color
    model.vis.global_.offwidth=1000;model.vis.global_.offheight=800;renderer=mujoco.Renderer(model,height=600,width=800);option=mujoco.MjvOption();option.geomgroup[:]=[1,1,1,0,0,0]
    camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE;camera.distance=.27;images=[]
    try:
        for name,label,path in states:
            data=restore(model,dict(np.load(path,allow_pickle=False)));C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3);camera.lookat[:]=C
            for view,desired in (('front',B[:,0]-.65*B[:,1]),('oblique',B[:,0]-.65*B[:,1]+.7*B[:,2])):
                desired=desired/np.linalg.norm(desired);candidates=[]
                for azimuth in range(0,360,30):
                    for elevation in (-45,-20,20,45):
                        camera.azimuth=azimuth;camera.elevation=elevation;renderer.update_scene(data,camera=camera,scene_option=option)
                        position=renderer.scene.camera[0].pos-C;candidates.append((float(position@desired/np.linalg.norm(position)),azimuth,elevation))
                _,azimuth,elevation=max(candidates);camera.azimuth=azimuth;camera.elevation=elevation
                renderer.update_scene(data,camera=camera,scene_option=option);cube_display(renderer.scene,C,B,False)
                for axis,sign,color in ((0,1,[.96,.96,.94,1]),(1,-1,[.86,.10,.13,1]),(0,-1,[.60,.63,.62,1]),(1,1,[.60,.63,.62,1]),(2,1,[.60,.63,.62,1]),(2,-1,[.60,.63,.62,1])):
                    other=[i for i in range(3) if i!=axis]
                    for u in (-1,0,1):
                        for v in (-1,0,1):
                            size=np.full(3,.0083);size[axis]=.00015;p=np.zeros(3);p[axis]=sign*.0277;p[other]=np.array([u,v])*.055/3
                            extra(renderer.scene,mujoco.mjtGeom.mjGEOM_BOX,size,C+B@p,B,color)
                extra(renderer.scene,mujoco.mjtGeom.mjGEOM_CYLINDER,[.0275*np.sqrt(2),.055/6,0],C+B@np.array([0.,-.055/3,0.]),np.column_stack([B[:,0],B[:,2],-B[:,1]]),[.85,.2,.65,.16])
                filename=name+'_'+view+'.png';png(folder/filename,renderer.render());images.append(dict(file=filename,label=label,view=view,source=path.relative_to(OUT).as_posix(),source_sha256=sha(path)))
    finally:renderer.close()
    save_json(folder/'visual_manifest.json',dict(status='scientific_model_hypothesis_views',images=images,model_provenance=provenance,display=display,
        source_code_sha256=sha(Path(__file__)),white_front_assigned_to_local_positive_X_only_for_hypothesis_display=True,red_top_assigned_to_local_negative_Y=True,
        not_camera_registration_or_exact_photo_reconstruction=True,exact_static_pose_not_a_native_goal=True,devices_opened=0))
    print('hypothesis_images='+str(len(images)),flush=True)


if __name__=='__main__':main()
