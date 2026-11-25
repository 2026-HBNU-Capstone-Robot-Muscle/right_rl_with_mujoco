"""Inspect a conditional after-flip yaw seed, never a hardware command."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from loaded_thumb_rotation_plan_20261026 import final_goal
ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/measured_thumb_125659_cube_replay'
SOURCE=ROOT/'loaded_repeat_20261026_170932_163649/result.json'
OUT=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1'
def main():
    OUT.mkdir(parents=True,exist_ok=True);r=json.loads(SOURCE.read_bytes())
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,OUT/'scene.xml');set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.dof_armature[:16]=1e6
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    d=mujoco.MjData(m);d.qpos[:]=np.load(BASE/'trace_s40_mu0.3.npz')['qpos'][-1]
    actual=np.asarray(r['rotation_position_ticks']);d.qpos[:16]=ticks_to_sim(actual);d.ctrl[:]=d.qpos[:16]
    mujoco.mj_forward(m,d);palm=m.body('palm').id;R=d.xmat[palm].reshape(3,3);up=-R[:,2]
    if up[2]<0:up=-up
    m.opt.gravity[:]=-9.81*up;m.geom_friction[m.geom('cube').id,0]=.3
    initial=d.qpos[-7:].copy()
    for _ in range(1000):
        d.qpos[:16]=ticks_to_sim(actual);d.qvel[:16]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_step(m,d)
    mujoco.mj_forward(m,d);cube=m.geom('cube').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    contact=[];selfpairs={};support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};support_peaks={}
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand;depth=max(0.,-float(c.dist)*1000)
        if ha and hb and depth:
            key='|'.join(sorted((m.geom(a).name,m.geom(b).name)));selfpairs[key]=max(selfpairs.get(key,0.),depth)
        if ((a in support and hb) or (b in support and ha)) and depth:
            key='|'.join(sorted((m.geom(a).name,m.geom(b).name)));support_peaks[key]=max(support_peaks.get(key,0.),depth)
        if cube in (a,b) and c.dist<.0003:
            other=b if a==cube else a;body=int(m.geom_bodyid[other]);B=d.xmat[body].reshape(3,3)
            contact.append(dict(geom=m.geom(other).name,body=m.body(body).name,depth_mm=depth,
                point_world_m=c.pos.tolist(),point_local_m=(B.T@(c.pos-d.xpos[body])).tolist()))
    C=d.xmat[int(m.geom_bodyid[cube])].reshape(3,3);red_error=float(np.degrees(np.arccos(np.clip(C[:,0]@up,-1,1))))
    out=dict(source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        initial_cube_pose_hypothesis='nominal end of EMPTY-trace conditional replay; not measured registration',
        actual_hand_ticks=actual.tolist(),commanded_hand_ticks=final_goal().tolist(),response_residual_ticks=(actual-final_goal()).tolist(),
        hand_sim_angles=d.qpos[:16].tolist(),settled_cube_qpos=d.qpos[-7:].tolist(),up_world=up.tolist(),
        center_shift_during_settle_mm=float(1000*np.linalg.norm(d.qpos[-7:-4]-initial[:3])),red_up_error_deg=red_error,
        contacts=contact,self_contact_mm=selfpairs,support_contact_mm=support_peaks,
        placement_and_friction_assumed=True,hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'seed.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(out),flush=True)
if __name__=='__main__':main()
