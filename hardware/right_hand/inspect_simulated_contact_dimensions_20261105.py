"""Read source contact dimensions and friction moments without stepping."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json,numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT
from research_rotation_body_hold_20261104 import restore
folder=ROOT/'torque_cap_sensitivity/cap0.15';m=mujoco.MjModel.from_binary_path(str(folder/'model.mjb'));d=restore(m,np.load(folder/'end_state.npz'));cube=m.geom('cube').id;rows=[]
for i,c in enumerate(d.contact):
    x,y=int(c.geom1),int(c.geom2)
    if cube not in (x,y):continue
    f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f);other=y if x==cube else x;body=int(m.geom_bodyid[other]);jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,c.pos,body);J=jp[:,np.r_[np.arange(8),np.arange(12,16)]];singular=np.linalg.svd(J,compute_uv=False);n=c.frame[:3]*(1 if x==cube else -1)
    rows.append(dict(other=m.geom(other).name,hand_body=m.body(body).name,dim=int(c.dim),friction=c.friction.tolist(),force_and_moment_local=f.tolist(),actuated_point_translation_rank=int(sum(singular>1e-8)),translation_singular_values_m_per_rad=singular.tolist(),normal_translation_control_norm=float(np.linalg.norm(n@J))))
print(json.dumps(dict(native_cone=int(m.opt.cone),contact_rows=rows)))
from native_pyramidal_point_forces_20261106 import point_rays
print(json.dumps(dict(native_point_rays=point_rays(.3,.1).tolist())))
report=dict(status='source_contact_dimensions_and_control_directions_read_only',source_state_sha256=hashlib.sha256((folder/'end_state.npz').read_bytes()).hexdigest(),source_model_sha256=hashlib.sha256((folder/'model.mjb').read_bytes()).hexdigest(),native_cone=int(m.opt.cone),contacts=rows,point_friction_rays=point_rays(.3,.1).tolist(),rank_is_instantaneous_translation_only_not_a_motion_proof=True,hardware_execution_allowed=False,devices_opened=False)
out=ROOT/'contact_wrench_audit';out.mkdir(exist_ok=True);(out/'control_directions.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
