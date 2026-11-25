"""Separate, uncertainty-labelled after-flip seed; no hardware interface."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from loaded_thumb_rotation_plan_20261026 import final_goal
ROOT=Path(__file__).resolve().parent;REG=ROOT/'right_hand_rotation_20261026/saved_marker_registration';OUT=ROOT/'right_hand_rotation_20261026/yaw_after_flip_camera_seed_v1'
def main():
    OUT.mkdir(exist_ok=True);source=ROOT/'loaded_repeat_20261026_170932_163649/result.json';r=json.loads(source.read_bytes());metric=json.loads((REG/'saved_after_flip_outline_calibrated_hypothesis.json').read_bytes());actual=np.asarray(r['rotation_position_ticks']);base=final_goal();cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'seed_scene.xml');set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.dof_armature[:16]=1e6
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;body=int(m.geom_bodyid[cube]);palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def observe(d):
        sp=sup=cp=0.;contacts=[];external=False
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,dep)
            if (a in support and hb) or (b in support and ha):sup=max(sup,dep)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cp=max(cp,dep)
                    if c.dist<.0002:contacts.append(dict(geom=m.geom(other).name,depth_mm=dep))
                elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):external=True
        return dict(max_self_overlap_mm=sp,max_support_overlap_mm=sup,max_cube_overlap_mm=cp,contacts=contacts,external_cube_contact=external)
    candidates=[];states=[]
    for row in metric['source_frames']:
        q=Rotation.from_matrix(row['R_cube_world']).as_quat();initial=np.r_[row['center_world_m'],q[[3,0,1,2]]];d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(actual);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=initial;mujoco.mj_forward(m,d);up=-d.xmat[palm].reshape(3,3)[:,2];m.opt.gravity[:]=-9.81*up;m.geom_friction[cube,0]=.3;obs=observe(d);record=dict(source_frame=row['image'],source_center_palm_mm=row['center_palm_mm'],source_anchor_error_px=row['anchor_error_px'],source_marker_error_px=row['red_error_px'],initial_contacts=obs,initial_qpos=initial.tolist())
        if obs['max_cube_overlap_mm']<=1 and obs['max_self_overlap_mm']<=.01 and obs['max_support_overlap_mm']<=.15:
            maxcp=obs['max_cube_overlap_mm']
            for i in range(500):
                d.qpos[:16]=ticks_to_sim(actual);d.qvel[:16]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_step(m,d)
                if i%5==0:mujoco.mj_forward(m,d);maxcp=max(maxcp,observe(d)['max_cube_overlap_mm'])
            mujoco.mj_forward(m,d);final=observe(d);shift=float(1000*np.linalg.norm(d.qpos[-7:-4]-initial[:3]));B=d.xmat[body].reshape(3,3);tilt=float(np.degrees(np.arccos(np.clip(B[:,0]@up,-1,1))));record.update(settle_translation_mm=shift,settled_red_up_error_deg=tilt,settled_contacts=final,max_cube_overlap_during_settle_mm=maxcp,settled_cube_qpos=d.qpos[-7:].tolist(),accepted_model_seed=bool(shift<=5 and tilt<=15 and maxcp<=1 and not final['external_cube_contact']))
            if record['accepted_model_seed']:states.append((row['anchor_error_px']+row['red_error_px'],record,d.qpos[-7:].copy(),up.copy()))
        else:record['accepted_model_seed']=False
        candidates.append(record)
    result=dict(status='camera_derived_after_flip_seed_screen',candidates=candidates,model_seed_count=len(states),source_marker_geometry_manually_annotated=True,initial_pose_metric_not_certified=True,source_marker41_size_approximate=True,selection_uses_model_contact_consistency=True,physical_vertical_successes=2,hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'seed_screen.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    if states:
        _,choice,pose,up=min(states,key=lambda v:v[0]);seed=dict(source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),initial_cube_pose_hypothesis='end marker ID2, independently annotated 55mm face geometry; provisional hand registration; model-consistency filtered',actual_hand_ticks=actual.tolist(),commanded_hand_ticks=base.tolist(),response_residual_ticks=(actual-base).tolist(),settled_cube_qpos=pose.tolist(),up_world=up.tolist(),source_frame=choice['source_frame'],source_registration='right_hand_rotation_20261026/saved_marker_registration/saved_after_flip_outline_calibrated_hypothesis.json',source_marker_geometry='cube_red_green_20261026_215059_362527/outer_face_marker_diagnostic.json',source_marker_geometry_is_provisional=True,source_coordinate_hand_registration_not_certified=True,cube_gravity_reference_assumed_palm_normal=True,friction_assumed=.3,mass_kg_assumed=.1,hardware_execution_allowed=False,real_devices_opened=False)
        (OUT/'seed.json').write_text(json.dumps(seed,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()
