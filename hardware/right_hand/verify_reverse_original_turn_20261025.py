"""Independent native-motor replay with matched stationary controls."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np
import mujoco
import mujoco.rollout
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
from hand_motor_mapping import ticks_to_sim
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/original_sequence_clearance'

def main():
    p=OUT/'CAD_adapted_forward_reference_path_v2.json';blob=p.read_bytes();plan=json.loads(blob)
    raw=np.array(plan['dense_goal_ticks'])[::-1];q=ticks_to_sim(raw)
    reference=json.loads((OUT/'reverse_original_turn_evaluation.json').read_bytes());center=np.array(reference['reference_start_center_m'])
    original=json.loads((OUT/'CAD_tip_contact_comparison_v2.json').read_bytes());initial_quat=np.array(original['free_cube_dynamics'][0]['placement']['quat_wxyz'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'reverse_verification_CAD.xml');set_cube_physics(m,.055,.1)
    cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    x=np.array([1.,0,0]);x-=up*(x@up);x/=np.linalg.norm(x);y=np.cross(up,x)
    commands=np.r_[q[1:],np.repeat(q[-1:],80,axis=0)];substeps=round(.05/m.opt.timestep)
    kind=mujoco.mjtState.mjSTATE_FULLPHYSICS;gain=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy()
    d=mujoco.MjData(m);inspect=mujoco.MjData(m);results=[]
    cases=[(1.,.8,0.,0.)]
    cases +=[(g,f,dx,dy) for g in (.5,1.,1.5) for f in (.4,.8,1.2) for dx,dy in ((0.,0.),(-2.,0.),(2.,0.),(0.,-2.),(0.,2.)) if (g,f,dx,dy)!=(1.,.8,0.,0.)]
    with mujoco.rollout.Rollout(nthread=2) as pool:
        datas=[mujoco.MjData(m),mujoco.MjData(m)]
        for g,f,dx,dy in cases:
            m.actuator_gainprm[:]=gain;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=g;m.actuator_biasprm[:,1:3]*=g
            m.geom_friction[cube,0]=f;mujoco.mj_resetData(m,d);d.qpos[:16]=q[0];d.ctrl[:]=q[0];d.qpos[-7:]=[10,10,10,1,0,0,0]
            for _ in range(round(1./m.opt.timestep)):mujoco.mj_step(m,d)
            d.qpos[-7:]=np.r_[center+((-6+dx)*x+(-6+dy)*y)/1000+up*.008,initial_quat];d.qvel[-6:]=0
            mujoco.mj_forward(m,d);overlap=max([-float(c.dist)*1000 for c in d.contact if cube in (int(c.geom1),int(c.geom2))]+[0.])
            for _ in range(round(1.5/m.opt.timestep)):mujoco.mj_step(m,d)
            initial=d.qpos.copy();R0=Rotation.from_quat(initial[-4:][[1,2,3,0]]).as_matrix()
            green=float(np.degrees(np.arccos(np.clip((R0@np.array([0,-1.,0]))@up,-1,1))))
            touching=any(cube in (int(c.geom1),int(c.geom2)) and c.dist<.0003 and int(m.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)]) in hand for c in d.contact)
            if overlap>.5 or green>20 or not touching:
                results.append(dict(gain_scale=g,friction=f,offset_from_nominal_mm=[dx,dy],valid_start=False,
                    initial_overlap_mm=overlap,initial_green_up_error_deg=green));continue
            state=np.empty(mujoco.mj_stateSize(m,kind));mujoco.mj_getState(m,d,state,kind);target=target_quaternion(initial[-4:])
            sets=[commands]
            if dx==0 and dy==0:sets.append(np.repeat(q[:1],len(commands),axis=0))
            ctrls=np.repeat(np.array(sets),substeps,axis=1)
            states,_=pool.rollout(m,datas,state[None],ctrls,initial_warmstart=d.qacc_warmstart[None])
            for mode,(ss,cc) in enumerate(zip(states,sets)):
                actual=ss[:,1:1+m.nq];norm=np.maximum(np.linalg.norm(actual[:,-4:],axis=1),1e-12)
                errors=2*np.arccos(np.clip(abs(actual[:,-4:]@target/norm),0,1));hold=round(4/m.opt.timestep)
                tracking=float(np.max(abs(actual[:,:16]-ctrls[mode])));selfmax={};supportmax={};external=False;maxgap=gap=0.;endcontacts=[]
                for i in range(substeps-1,len(ss),substeps):
                    mujoco.mj_setState(m,inspect,ss[i],kind);inspect.ctrl[:]=cc[i//substeps];mujoco.mj_forward(m,inspect);touch=[]
                    for c in inspect.contact:
                        a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                        pair='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
                        if c.dist<0 and ha and hb:selfmax[pair]=max(selfmax.get(pair,0.),-float(c.dist)*1000)
                        if c.dist<0 and (a in support and hb or b in support and ha):supportmax[pair]=max(supportmax.get(pair,0.),-float(c.dist)*1000)
                        if cube in (a,b) and c.dist<.0003:
                            other=b if a==cube else a
                            if int(m.geom_bodyid[other]) in hand:touch.append(m.body(int(m.geom_bodyid[other])).name)
                            if other in support or m.geom(other).name=='floor':external=True
                    gap=0. if touch else gap+.05;maxgap=max(maxgap,gap);endcontacts=sorted(set(touch))
                R=Rotation.from_quat(actual[-1,-4:][[1,2,3,0]]).as_matrix();front=R0@np.array([-1.,0,0]);front-=up*(front@up);front/=np.linalg.norm(front)
                row=dict(gain_scale=g,friction=f,offset_from_nominal_mm=[dx,dy],mode='moving_path' if mode==0 else 'matched_stationary_control',
                    valid_start=True,initial_overlap_mm=overlap,initial_green_up_error_deg=green,
                    initial_state=state.tolist(),initial_cube_qpos=initial[-7:].tolist(),
                    maximum_tracking_rad=tracking,maximum_self_model_overlap_mm=max(selfmax.values(),default=0.),
                    self_overlap_pairs_mm=selfmax,support_overlap_pairs_mm=supportmax,
                    maximum_no_hand_contact_seconds=maxgap,external_cube_contact_ever=external,end_hand_contacts=endcontacts,
                    whole_rotation_angle_deg=float(np.degrees(Rotation.from_matrix(R@R0.T).magnitude())),
                    final_target_error_deg=float(np.degrees(errors[-1])),hold_maximum_target_error_deg=float(np.degrees(errors[-hold:].max())),
                    red_up_error_deg=float(np.degrees(np.arccos(np.clip((R@np.array([1.,0,0]))@up,-1,1)))),
                    green_front_error_deg=float(np.degrees(np.arccos(np.clip((R@np.array([0,-1.,0]))@front,-1,1)))),
                    geometric_contact_sampling_seconds=.05,
                    simulated_orientation_and_retention_pass=bool(errors[-hold:].max()<np.deg2rad(15) and endcontacts and not external and maxgap<=.1),
                    frames=actual[::20].tolist() if g==1 and f==.8 and dx==0 and dy==0 else [])
                results.append(row)
                print('REVERSE_VERIFY',g,f,dx,dy,row['mode'],'hold_error',round(row['hold_maximum_target_error_deg'],2),
                    'track',round(tracking,3),'self',round(row['maximum_self_model_overlap_mm'],3),
                    'pass',row['simulated_orientation_and_retention_pass'],flush=True)
    m.actuator_gainprm[:]=gain;m.actuator_biasprm[:]=bias
    report=dict(source_path_sha256=hashlib.sha256(blob).hexdigest(),model=manifest,runs=results,
        native_motor_parameters_identified=False,static_goal_path_clear=True,
        hardware_execution_allowed=False,hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False)
    (OUT/'reverse_original_native_verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')

if __name__=='__main__':main()
