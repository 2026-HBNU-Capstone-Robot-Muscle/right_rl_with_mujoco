"""Continue the60deg contact solution, allowing bounded rolling relocalization."""
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
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2';OLD=ROOT/'right_hand_rotation_20261027/reached_pair_contact_yaw';OUT=ROOT/'right_hand_rotation_20261027/reached_pair_contact_yaw_extension';OUT.mkdir(exist_ok=True)
def main():
    began=time.perf_counter();state=np.load(SOURCE/'nominal_grasp_state.npz');cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,OUT/'scene.xml');proof=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());correct_model(m,np.array(proof['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002;up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);m.opt.gravity[:]=-9.81*up;cube=m.geom('cube').id;d=mujoco.MjData(m);d.qpos[:]=state['qpos'];d.qvel[:]=0;d.ctrl[:]=state['ctrl'];mujoco.mj_forward(m,d);C=d.qpos[-7:-4].copy();B=d.xmat[m.body('cube').id].reshape(3,3).copy();tips=[m.geom(n+'_tip').id for n in ('th','mf')];contacts0=[]
    for g in tips:
        segment=np.zeros(6);mujoco.mj_geomDistance(m,d,cube,g,1.,segment);contacts0.append(B.T@(segment[:3]-C))
    original_contacts=np.array(contacts0);contacts0=original_contacts.copy();data=np.load(OLD/'lift4_press0.75.npz');base=data['start'];bias=data['bias_ticks'];goals=list(data['goals']);angles=list(data['desired_angles']);previous=goals[-1].astype(float);lo,hi=limits();ids=np.array([0,1,2,3,8,9,10,11]);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);log=[];relocalized=[];rng=np.random.default_rng(20261027);failed=None
    for angle in range(63,121,3):
        D=Rotation.from_rotvec(up*np.radians(angle)).as_matrix();R=D@B;centre=C+up*.004
        def put(x):
            command=previous.astype(float).copy();command[ids]=x;d.qpos[:16]=ticks_to_sim(command+bias);d.qvel[:]=0;d.qpos[-7:-4]=centre;d.qpos[-4:]=Rotation.from_matrix(R).as_quat()[[3,0,1,2]];mujoco.mj_forward(m,d);return command
        def measures():
            vals=[];errors=[];points=[]
            for j,g in enumerate(tips):
                line=np.zeros(6);dist=mujoco.mj_geomDistance(m,d,cube,g,1.,line)*1000;point=R.T@(line[:3]-centre);error=(point-contacts0[j])*1000;vals.extend(error);vals.append(4*(dist+.75));errors.append((float(np.linalg.norm(error)),float(dist)));points.append(point)
            su,sp=inspect(d);other=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2)) and (int(c.geom2) if int(c.geom1)==cube else int(c.geom1)) not in tips),default=0.)
            return np.array(vals),errors,np.array(points),max(sp.values(),default=0.),max(su.values(),default=0.),other
        def objective(x):
            put(x);v,_,_,sp,su,other=measures();return np.r_[v,20*sp,20*max(0,su-.1),20*max(0,other-.8),.001*(x-previous[ids])]
        best=None
        for adjustment in range(3):
            for attempt in range(5):
                initial=previous[ids].copy() if attempt==0 else previous[ids]+rng.normal(0,20 if attempt<3 else 80,8);fit=least_squares(objective,np.clip(initial,lo[ids]+.001,hi[ids]-.001),bounds=(lo[ids]+.001,hi[ids]-.001),diff_step=.0001,max_nfev=160);command=put(np.rint(fit.x));_,errors,points,sp,su,other=measures();ok=bool(max(v[0] for v in errors)<2 and all(-1.25<=v[1]<=.7 for v in errors) and sp<=1e-6 and su<=.15 and other<=1);score=max(v[0] for v in errors)+20*sp+20*max(0,su-.15)+20*max(0,other-1)
                row=dict(angle_deg=angle,contacts=errors,self_mm=sp,support_mm=su,other_cube_mm=other,passed=ok,goal=np.rint(command).astype(int).tolist(),points=points.tolist(),score=score)
                if best is None or row['score']<best['score']:best=row
                if ok:break
            if ok:break
            points=np.array(best['points']);new=contacts0.copy();new[:,1:]+=np.clip(points[:,1:]-contacts0[:,1:],-.006,.006);new[:,1:]=np.clip(new[:,1:],original_contacts[:,1:]-.014,original_contacts[:,1:]+.014);new[:,1:]=np.clip(new[:,1:],-.025,.025)
            if np.max(abs(new-contacts0))<1e-7:break
            relocalized.append(dict(angle_deg=angle,from_local=contacts0.tolist(),to_local=new.tolist()));contacts0=new;best=None
        log.append(best);print(json.dumps(best),flush=True)
        if not best['passed']:failed=best;break
        command=np.array(best['goal']);count=max(1,int(np.ceil(max(abs(command-previous))/2)));goals.extend(np.rint(previous+(command-previous)*i/count).astype(int) for i in range(1,count+1));angles.extend([angle]*count);previous=command.astype(float)
    np.savez_compressed(OUT/'continued.npz',start=base,goals=np.asarray(goals),desired_angles=np.asarray(angles),bias_ticks=bias);report=dict(status='contact_roll_continuation_endpoint_diagnostic',reached_angle_deg=float(max(angles)),goals=len(goals),nominal_seconds=len(goals)*.075,first_failed=failed,steps=log,contact_relocalizations=relocalized,free_cube_not_checked=True,full_edge_screen_not_done=True,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began);(OUT/'ik_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
