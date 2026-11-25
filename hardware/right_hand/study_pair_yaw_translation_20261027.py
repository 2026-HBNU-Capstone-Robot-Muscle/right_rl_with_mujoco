"""Offline90deg contact IK allowing a bounded shift across the palm."""
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
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2';OUT=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_translation';OUT.mkdir(exist_ok=True)
def main():
    began=time.perf_counter();state=np.load(SOURCE/'nominal_grasp_state.npz');cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,OUT/'scene.xml');proof=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());correct_model(m,np.array(proof['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002;up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);m.opt.gravity[:]=-9.81*up;cube=m.geom('cube').id;d=mujoco.MjData(m);d.qpos[:]=state['qpos'];d.qvel[:]=0;d.ctrl[:]=state['ctrl'];mujoco.mj_forward(m,d);C=d.qpos[-7:-4].copy();B=d.xmat[m.body('cube').id].reshape(3,3).copy();tips=[m.geom(n+'_tip').id for n in ('th','mf')];centres=np.array([d.geom_xpos[g].copy() for g in tips]);axes=np.array([d.geom_xmat[g].reshape(3,3).copy() for g in tips]);simids=np.array(SIM_TO_MOTOR_IDS)-1;actual=np.empty(16);actual[simids]=d.qpos[:16]/TICK+2048;base=np.empty(16);base[simids]=d.ctrl[:]/TICK+2048;base=np.rint(base).astype(int);bias=actual-base;lo,hi=limits();ids=np.array([0,1,2,3,8,9,10,11]);h1=np.cross([0.,0,1.],up);h1/=np.linalg.norm(h1);h2=np.cross(up,h1);basis=np.column_stack([h1,h2]);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);reports=[]
    for lift_mm in (4,8):
      for shift_bound in (10,20):
        previous=base.astype(float);shift=np.zeros(2);goals=[];angles=[];shifts=[];log=[];failed=None;targets=[(0,float(h)*.001) for h in range(1,lift_mm+1)]+[(a,lift_mm*.001) for a in range(3,121,3)]
        for angle,lift in targets:
            D=Rotation.from_rotvec(up*np.radians(angle)).as_matrix()
            def put(x):
                command=previous.astype(float).copy();command[ids]=x[:8];centre=C+up*lift+basis@x[8:]*.001;d.qpos[:16]=ticks_to_sim(command+bias);d.qvel[:]=0;d.qpos[-7:-4]=centre;d.qpos[-4:]=Rotation.from_matrix(D@B).as_quat()[[3,0,1,2]];mujoco.mj_forward(m,d);return command,centre
            def objective(x):
                command,centre=put(x);positions=centre+(centres-C)@D.T;su,sp=inspect(d);cp=0.
                for c in d.contact:
                    if cube in (int(c.geom1),int(c.geom2)):cp=max(cp,max(0.,-float(c.dist)*1000))
                return np.r_[np.concatenate([(d.geom_xpos[g]-positions[j])*1000 for j,g in enumerate(tips)]),np.concatenate([(d.geom_xmat[g].reshape(3,3)-D@axes[j]).ravel()*1.5 for j,g in enumerate(tips)]),20*max(sp.values(),default=0.),20*max(0,max(su.values(),default=0.)-.1),20*max(0,cp-.8),.001*(x[:8]-previous[ids]),.03*x[8:],.03*(x[8:]-shift)]
            initial=np.r_[previous[ids],shift];fit=least_squares(objective,np.clip(initial,np.r_[lo[ids]+.001,[-shift_bound]*2],np.r_[hi[ids]-.001,[shift_bound]*2]),bounds=(np.r_[lo[ids]+.001,[-shift_bound]*2],np.r_[hi[ids]-.001,[shift_bound]*2]),diff_step=.00003,max_nfev=120);x=fit.x.copy();x[:8]=np.rint(x[:8]);command,centre=put(x);positions=centre+(centres-C)@D.T;su,sp=inspect(d);cp=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.);error=float(max(np.linalg.norm(d.geom_xpos[g]-positions[j])*1000 for j,g in enumerate(tips)));ok=bool(error<1.5 and max(sp.values(),default=0.)<=1e-6 and max(su.values(),default=0.)<=.15 and cp<=1);record=dict(angle_deg=angle,lift_mm=lift*1000,shift_mm=x[8:].tolist(),max_tip_error_mm=error,self_mm=max(sp.values(),default=0.),support_mm=max(su.values(),default=0.),cube_mm=cp,passed=ok,goal=np.rint(command).astype(int).tolist());log.append(record)
            if not ok:failed=record;break
            count=max(1,int(np.ceil(max(abs(command-previous))/2)));goals.extend(np.rint(previous+(command-previous)*i/count).astype(int) for i in range(1,count+1));angles.extend([angle]*count);shifts.extend([x[8:].copy()]*count);previous=command.astype(float);shift=x[8:].copy()
        name=f'lift{lift_mm}_shift{shift_bound}';np.savez_compressed(OUT/(name+'.npz'),start=base,goals=np.asarray(goals),desired_angles=np.array(angles),shift_mm=np.array(shifts),bias_ticks=bias);row=dict(name=name,lift_mm=lift_mm,shift_bound_mm=shift_bound,reached_angle_deg=log[-2]['angle_deg'] if failed and len(log)>1 else (0 if failed else log[-1]['angle_deg']),all_endpoints_passed=failed is None,first_failed=failed,goals=len(goals),nominal_seconds=len(goals)*.075,endpoint_log=log);reports.append(row);print(json.dumps({k:v for k,v in row.items() if k!='endpoint_log'}),flush=True)
    (OUT/'ik_review.json').write_text(json.dumps(dict(cases=reports,goal_was_90_plus_margin_deg=120,free_cube_dynamics_not_checked=True,full_edge_screen_not_done=True,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
