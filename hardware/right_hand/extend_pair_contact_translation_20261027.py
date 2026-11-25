"""Continue opposed mesh contacts while translating the cube within the hand."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2';OLD=ROOT/'right_hand_rotation_20261027/reached_pair_contact_yaw_extension';OUT=ROOT/'right_hand_rotation_20261027/reached_pair_contact_translation';OUT.mkdir(exist_ok=True)
def main():
    began=time.perf_counter();state=np.load(SOURCE/'nominal_grasp_state.npz');cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,OUT/'scene.xml');proof=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());correct_model(m,np.array(proof['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002;up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);m.opt.gravity[:]=-9.81*up;cube=m.geom('cube').id;d=mujoco.MjData(m);d.qpos[:]=state['qpos'];d.qvel[:]=0;d.ctrl[:]=state['ctrl'];mujoco.mj_forward(m,d);C=d.qpos[-7:-4].copy();B=d.xmat[m.body('cube').id].reshape(3,3).copy();tips=[m.geom(n+'_tip').id for n in ('th','mf')];contacts0=[]
    for g in tips:
        segment=np.zeros(6);mujoco.mj_geomDistance(m,d,cube,g,1.,segment);contacts0.append(B.T@(segment[:3]-C))
    contacts0=np.array(contacts0);data=np.load(OLD/'continued.npz');base=data['start'];bias=data['bias_ticks'];goals=list(data['goals']);angles=list(data['desired_angles']);previous=goals[-1].astype(float);shift=np.zeros(2);shifts=[np.zeros(2) for _ in goals];lo,hi=limits();ids=np.array([0,1,2,3,8,9,10,11]);lower=np.maximum(lo[ids],lo[ids]-bias[ids])+.001;upper=np.minimum(hi[ids],hi[ids]-bias[ids])-.001;h1=np.cross([0.,0,1.],up);h1/=np.linalg.norm(h1);basis=np.column_stack([h1,np.cross(up,h1)]);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);log=[];rng=np.random.default_rng(427);failed=None
    for angle in range(int(max(angles))+3,121,3):
        D=Rotation.from_rotvec(up*np.radians(angle)).as_matrix();R=D@B
        def put(x):
            command=previous.astype(float).copy();command[ids]=x[:8];centre=C+up*.004+basis@x[8:]*.001;d.qpos[:16]=ticks_to_sim(command+bias);d.qvel[:]=0;d.qpos[-7:-4]=centre;d.qpos[-4:]=Rotation.from_matrix(R).as_quat()[[3,0,1,2]];mujoco.mj_forward(m,d);return command,centre
        def measures(centre):
            vals=[];errors=[]
            for j,g in enumerate(tips):
                line=np.zeros(6);dist=mujoco.mj_geomDistance(m,d,cube,g,1.,line)*1000;point=R.T@(line[:3]-centre);error=(point-contacts0[j])*1000;vals.extend(error);vals.append(4*(dist+.75));errors.append((float(np.linalg.norm(error)),float(dist)))
            su,sp=inspect(d);other=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2)) and (int(c.geom2) if int(c.geom1)==cube else int(c.geom1)) not in tips),default=0.)
            return np.array(vals),errors,max(sp.values(),default=0.),max(su.values(),default=0.),other
        def objective(x):
            command,centre=put(x);v,_,sp,su,other=measures(centre);return np.r_[v,25*sp,25*max(0,su-.1),25*max(0,other-.7),.001*(x[:8]-previous[ids]),.025*x[8:],.03*(x[8:]-shift)]
        best=None
        for attempt in range(6):
            initial=np.r_[previous[ids],shift]
            if attempt:initial+=np.r_[rng.normal(0,25 if attempt<3 else 70,8),rng.normal(0,2 if attempt<3 else 5,2)]
            fit=least_squares(objective,np.clip(initial,np.r_[lower,[-15]*2],np.r_[upper,[15]*2]),bounds=(np.r_[lower,[-15]*2],np.r_[upper,[15]*2]),diff_step=.0001,max_nfev=160);x=fit.x.copy();x[:8]=np.rint(x[:8]);command,centre=put(x);_,errors,sp,su,other=measures(centre);ok=bool(max(v[0] for v in errors)<2 and all(-1.25<=v[1]<=.7 for v in errors) and sp<=1e-6 and su<=.15 and other<=1);score=max(v[0] for v in errors)+25*sp+25*max(0,su-.15)+25*max(0,other-1);row=dict(angle_deg=angle,contacts=errors,self_mm=sp,support_mm=su,other_cube_mm=other,passed=ok,goal=np.rint(command).astype(int).tolist(),shift_mm=x[8:].tolist(),score=score)
            if best is None or score<best['score']:best=row
            if ok:best=row;break
        log.append(best);print(json.dumps(best),flush=True)
        if not best['passed']:failed=best;break
        command=np.array(best['goal']);count=max(1,int(np.ceil(max(abs(command-previous))/2)));goals.extend(np.rint(previous+(command-previous)*i/count).astype(int) for i in range(1,count+1));angles.extend([angle]*count);shift=np.array(best['shift_mm']);shifts.extend([shift.copy()]*count);previous=command.astype(float)
        np.savez_compressed(OUT/'continued.npz',start=base,goals=np.asarray(goals),desired_angles=np.asarray(angles),shift_mm=np.asarray(shifts),bias_ticks=bias)
    (OUT/'ik_review.json').write_text(json.dumps(dict(status='contact_roll_translation_diagnostic',reached_angle_deg=float(max(angles)),steps=log,first_failed=failed,goals=len(goals),nominal_seconds=len(goals)*.075,free_cube_not_checked=True,full_edge_screen_not_done=True,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
