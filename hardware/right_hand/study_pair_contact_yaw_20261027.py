"""Mesh-contact IK: let fingertips roll without fixing their mesh centres."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim,TICK,SIM_TO_MOTOR_IDS
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2';OUT=ROOT/'right_hand_rotation_20261027/reached_pair_contact_yaw';OUT.mkdir(exist_ok=True)
def main():
    began=time.perf_counter();state=np.load(SOURCE/'nominal_grasp_state.npz');cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,OUT/'scene.xml');proof=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());correct_model(m,np.array(proof['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002;up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);m.opt.gravity[:]=-9.81*up;cube=m.geom('cube').id;d=mujoco.MjData(m);d.qpos[:]=state['qpos'];d.qvel[:]=0;d.ctrl[:]=state['ctrl'];mujoco.mj_forward(m,d);C=d.qpos[-7:-4].copy();B=d.xmat[m.body('cube').id].reshape(3,3).copy();tips=[m.geom(n+'_tip').id for n in ('th','mf')];contacts0=[]
    for g in tips:
        segment=np.zeros(6);mujoco.mj_geomDistance(m,d,cube,g,1.,segment);contacts0.append(B.T@(segment[:3]-C))
    contacts0=np.array(contacts0);simids=np.array(SIM_TO_MOTOR_IDS)-1;actual=np.empty(16);actual[simids]=d.qpos[:16]/TICK+2048;base=np.empty(16);base[simids]=d.ctrl[:]/TICK+2048;base=np.rint(base).astype(int);bias=actual-base;lo,hi=limits();ids=np.array([0,1,2,3,8,9,10,11]);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);reports=[]
    for lift_mm in (4,8):
      for press_mm in (.25,.75,1.25):
        previous=base.astype(float);goals=[];angles=[];log=[];failed=None;targets=[(0,0,press_mm*k/4) for k in range(1,5)]+[(0,float(h)*.001,press_mm) for h in range(1,lift_mm+1)]+[(a,lift_mm*.001,press_mm) for a in range(3,121,3)]
        for angle,lift,press in targets:
            D=Rotation.from_rotvec(up*np.radians(angle)).as_matrix();centre=C+up*lift;contact_targets=centre+contacts0@(D@B).T
            def put(x):
                command=previous.astype(float).copy();command[ids]=x;d.qpos[:16]=ticks_to_sim(command+bias);d.qvel[:]=0;d.qpos[-7:-4]=centre;d.qpos[-4:]=Rotation.from_matrix(D@B).as_quat()[[3,0,1,2]];mujoco.mj_forward(m,d);return command
            def measures():
                vals=[];errors=[]
                for j,g in enumerate(tips):
                    line=np.zeros(6);dist=mujoco.mj_geomDistance(m,d,cube,g,1.,line)*1000;error=(line[:3]-contact_targets[j])*1000;vals.extend(error);vals.append(4*(dist+press));errors.append((float(np.linalg.norm(error)),float(dist)))
                su,sp=inspect(d);other=0.
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if cube in (a,b) and (b if a==cube else a) not in tips:other=max(other,max(0.,-float(c.dist)*1000))
                return np.array(vals),errors,max(sp.values(),default=0.),max(su.values(),default=0.),other
            def objective(x):
                put(x);v,_,sp,su,other=measures();return np.r_[v,20*sp,20*max(0,su-.1),20*max(0,other-.8),.001*(x-previous[ids])]
            fit=least_squares(objective,np.clip(previous[ids],lo[ids]+.001,hi[ids]-.001),bounds=(lo[ids]+.001,hi[ids]-.001),diff_step=.0001,max_nfev=140);command=put(np.rint(fit.x));_,errors,sp,su,other=measures();ok=bool(max(v[0] for v in errors)<2 and all(-press-.5<=v[1]<=.7 for v in errors) and sp<=1e-6 and su<=.15 and other<=1);record=dict(angle_deg=angle,lift_mm=lift*1000,press_mm=press,contacts=errors,self_mm=sp,support_mm=su,other_cube_mm=other,passed=ok,goal=np.rint(command).astype(int).tolist());log.append(record)
            if not ok:failed=record;break
            count=max(1,int(np.ceil(max(abs(command-previous))/2)));goals.extend(np.rint(previous+(command-previous)*i/count).astype(int) for i in range(1,count+1));angles.extend([angle]*count);previous=command.astype(float)
        name=f'lift{lift_mm}_press{press_mm:g}';np.savez_compressed(OUT/(name+'.npz'),start=base,goals=np.asarray(goals),desired_angles=np.array(angles),bias_ticks=bias);row=dict(name=name,lift_mm=lift_mm,press_mm=press_mm,reached_angle_deg=log[-2]['angle_deg'] if failed and len(log)>1 else (0 if failed else log[-1]['angle_deg']),all_endpoints_passed=failed is None,first_failed=failed,goals=len(goals),nominal_seconds=len(goals)*.075,endpoint_log=log);reports.append(row);print(json.dumps({k:v for k,v in row.items() if k!='endpoint_log'}),flush=True)
    (OUT/'ik_review.json').write_text(json.dumps(dict(cases=reports,goal_angle_max_deg=120,contact_surface_roll_allowed=True,force_model_not_yet_checked=True,full_edge_screen_not_done=True,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
