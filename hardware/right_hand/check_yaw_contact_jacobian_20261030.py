from pathlib import Path
import json,numpy as np,mujoco
R=Path(__file__).resolve().parent;CP=R/'right_hand_rotation_20261030/natural_grasp_yaw/checkpoint';m=mujoco.MjModel.from_binary_path(str(CP/'grasp_checkpoint.mjb'));s=np.load(CP/'grasp_checkpoint.npz');d=mujoco.MjData(m);d.qpos[:]=s['qpos'];d.qpos[:16]=s['ctrl'];d.ctrl[:]=s['ctrl'];mujoco.mj_forward(m,d);base=d.qpos.copy();cube=m.geom('cube').id;rows=[]
for name,active in [('mf_tip',range(4,8)),('th_tip',range(12,16))]:
 g=m.geom(name).id;pair=np.zeros(6);dist=float(mujoco.mj_geomDistance(m,d,cube,g,.2,pair));c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);J=np.zeros((3,m.nv));mujoco.mj_jac(m,d,J,None,c.pos,int(m.geom_bodyid[g]));analytic=n@J[:,list(active)];finite=[]
 for j in active:
  v=[]
  for sign in (-1,1):
   d.qpos[:]=base;d.qpos[j]+=sign*2e-5;mujoco.mj_forward(m,d);v.append(float(mujoco.mj_geomDistance(m,d,cube,g,.2,None)))
  finite.append((v[1]-v[0])/4e-5)
 d.qpos[:]=base;mujoco.mj_forward(m,d);relative=float(np.linalg.norm(analytic-finite)/max(1e-9,np.linalg.norm(finite)));rows.append(dict(tip=name,command_distance_mm=dist*1000,analytic=analytic.tolist(),finite=finite,relative_error=relative,passed=relative<.05))
out=dict(all_passed=all(r['passed'] for r in rows),cases=rows,motor_writes=0);(CP/'contact_jacobian_check.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8');print(json.dumps(out));assert out['all_passed']
