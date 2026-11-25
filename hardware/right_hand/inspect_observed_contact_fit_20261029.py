"""Inspect the local contact points of the feasible observed54 hypothesis."""
import json
from pathlib import Path
import numpy as np,mujoco
from hand_motor_mapping import ticks_to_sim
ROOT=Path(__file__).resolve().parent;O=ROOT/'right_hand_rotation_20261029/observed_inverse_contact_fit'
m=mujoco.MjModel.from_binary_path(str(ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint/checkpoint_model.mjb'))
z=np.load(O/'gapped_inverse_trial_20261029_184252_272166_conditional_state_v2.npz');d=mujoco.MjData(m)
d.qpos[:16]=ticks_to_sim(z['actual']);d.ctrl[:]=ticks_to_sim(z['goal']);d.qpos[-7:]=z['cube_qpos'];mujoco.mj_forward(m,d)
cube=m.geom('cube').id;cube_body=m.body('cube').id;rows=[]
for name in ('mf_tip','th_tip','palm_collision_1','if_bs_collision_1'):
    g=m.geom(name).id;body=int(m.geom_bodyid[g]);points=np.empty(6)
    dist=mujoco.mj_geomDistance(m,d,cube,g,.2,points)
    actual_contacts=[c for c in d.contact if set((int(c.geom1),int(c.geom2)))==set((cube,g))]
    if actual_contacts:point=np.mean([c.pos for c in actual_contacts],axis=0)
    else:point=(points[:3]+points[3:])/2
    local_cube=d.xmat[cube_body].reshape(3,3).T@(point-d.xpos[cube_body]);axis=int(np.argmax(abs(local_cube)))
    normal_local=np.zeros(3);normal_local[axis]=np.sign(local_cube[axis]);normal_world=-d.xmat[cube_body].reshape(3,3)@normal_local
    row=dict(geom=name,body=m.body(body).name,geom_types=[int(m.geom_type[cube]),int(m.geom_type[g])],
        actual_contacts_mm=[float(c.dist)*1000 for c in actual_contacts],GJK_distance_not_contact_certificate_mm=float(dist)*1000,
        world_contact_point=point.tolist(),cube_local_contact=local_cube.tolist(),finger_body_local_contact=(d.xmat[body].reshape(3,3).T@(point-d.xpos[body])).tolist(),
        cube_inward_contact_normal_world=normal_world.tolist(),finger_inward_normal_local=(d.xmat[body].reshape(3,3).T@normal_world).tolist())
    rows.append(row)
report=dict(status='conditional_contact_local_points',contacts=rows,cube_qpos=d.qpos[-7:].tolist(),actual_ticks=z['actual'].tolist(),goal_ticks=z['goal'].tolist(),
    geometry_unchanged=True,physical_registration_not_verified=True,hardware_execution_allowed=False,devices_opened=False)
(O/'local_contacts_v2.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
