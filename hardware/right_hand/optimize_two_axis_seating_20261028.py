"""Offline finite-physics control search for a loaded seating transition.

Only hand actuator targets are optimized. Restoring a simulator snapshot is
used for independent trial evaluations, never to force a cube along a rollout.
No hardware API is imported or used; every result is a diagnostic candidate.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from hand_motor_mapping import ticks_to_sim,sim_to_hw,TICK
from sac_cube_model import set_cube_physics
from screen_pitch_yaw_join_20261028 import contact_reader
from calibrated_quarter_plan_20261028 import LOADED_PATH

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/two_axis_connection/seating_control_search'
SOURCE=ROOT/'right_hand_rotation_20261028/two_axis_connection/loaded_join/together/trace_s1_mu0.3.npz'

def main():
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    m,manifest=build(registry.load('LeapCubeFaceUp',config=cfg),OUT/'scene.xml',book_projection_mm=-10.)
    start=np.load(LOADED_PATH)['start'];correct_model(m,start);set_cube_physics(m,.055,.1)
    up=np.asarray(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world'])
    m.opt.timestep=.002;m.opt.gravity[:]=-9.81*up
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    cube=m.geom('cube').id;body=m.body('cube').id;m.geom_friction[cube,0]=.3
    inspect=contact_reader(m);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    target_center=np.asarray(next(v for v in prior['loaded_physics'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3))['settled_cube_qpos'][:3])
    d=mujoco.MjData(m);d.qpos[:]=np.load(SOURCE)['end_qpos'];d.ctrl[:]=ticks_to_sim(start)
    # Source ends after a3s fixed hold. Let finite dynamics settle further.
    for _ in range(500):mujoco.mj_step(m,d)
    kind=mujoco.mjtState.mjSTATE_INTEGRATION;size=mujoco.mj_stateSize(m,kind)
    def save():
        state=np.empty(size);mujoco.mj_getState(m,d,state,kind);return state
    def restore(state):mujoco.mj_setState(m,d,state,kind);mujoco.mj_forward(m,d)
    def pose():
        mujoco.mj_forward(m,d);return d.qpos[-7:-4].copy(),d.xmat[body].reshape(3,3).copy()
    def tilt(R):return float(np.degrees(np.arccos(np.clip(R[:,0]@up,-1,1))))
    lo,hi=limits();current=start.astype(float);accepted=save();accepted_rows=[];logs=[];evaluations=0
    for iteration in range(10):
        restore(accepted);C,B=pose();distance=float(np.linalg.norm(target_center-C))
        if distance<.006 and tilt(B)<8:break
        nextC=C+(target_center-C)*min(1.,.004/max(distance,1e-12))
        axis=np.cross(B[:,0],up);length=np.linalg.norm(axis)
        level=(Rotation.from_rotvec(axis/length*min(np.radians(2),np.arctan2(length,B[:,0]@up))).as_matrix()@B if length>1e-10 else B)
        def rollout(x,full=False):
            nonlocal evaluations
            evaluations+=1;restore(accepted)
            end=np.rint(x).astype(int) if full else x.copy()
            path=np.asarray([current+(end-current)*i/60 for i in range(1,61)])
            if full:path=np.rint(path).astype(int)
            path=np.vstack([path,np.repeat(path[-1][None,:],12,axis=0)])
            sp=sup=cp=track=excess=gap=longest=0.;external=False;steps=0
            for sample,q in enumerate(path,1):
                d.ctrl[:]=ticks_to_sim(q);target=round(sample*.075/.002)
                for j in range(target-steps):
                    mujoco.mj_step(m,d)
                    if not full and j!=target-steps-1:continue
                    a,b,_,_=inspect(d);sp=max(sp,a);sup=max(sup,b)
                    track=max(track,float(max(abs(d.qpos[:16]-d.ctrl))))
                    raw=sim_to_hw(d.qpos[:16])/TICK
                    excess=max(excess,float(max(0.,max(lo-raw),max(raw-hi))))
                    touching=False
                    for c in d.contact:
                        a,b=int(c.geom1),int(c.geom2)
                        if cube not in (a,b):continue
                        other=b if a==cube else a
                        if int(m.geom_bodyid[other]) in hand:
                            cp=max(cp,max(0.,-float(c.dist)*1000));touching|=c.dist<.0004
                        elif other in support and c.dist<.0004:external=True
                    gap=0. if touching else gap+(.002 if full else .075);longest=max(longest,gap)
                steps=target
            Cend,Rend=pose()
            return path,dict(self_mm=sp,support_mm=sup,cube_mm=cp,tracking_rad=track,
                             range_excess_ticks=excess,no_hand_contact_seconds=longest,
                             external=external,center_error_mm=float(np.linalg.norm(target_center-Cend)*1000),
                             red_up_deg=tilt(Rend)),Cend,Rend
        def objective(x):
            _,v,Cend,Rend=rollout(x)
            return np.r_[(Cend-nextC)*2000,Rotation.from_matrix(Rend@level.T).as_rotvec()*20,
                         100*v['self_mm'],100*max(0.,v['support_mm']-.1),
                         100*max(0.,v['cube_mm']-.8),100*max(0.,v['tracking_rad']-.15),
                         20*max(0.,v['range_excess_ticks']),150*max(0.,v['no_hand_contact_seconds']-.1),
                         300*float(v['external']),.002*(x-current)]
        lower=np.maximum(np.ceil(lo)+2,current-100)+.001
        upper=np.minimum(np.floor(hi)-2,current+100)-.001
        fit=least_squares(objective,np.clip(current,lower,upper),bounds=(lower,upper),
                          diff_step=.0004,max_nfev=25,ftol=.003,xtol=.00005)
        path,v,Cend,Rend=rollout(fit.x,full=True)
        v.update(iteration=iteration,evaluations=evaluations,requested_center_mm=(nextC*1000).tolist(),goal=path[-1].tolist())
        v['accepted']=bool(v['self_mm']<=1e-6 and v['support_mm']<=.15 and v['cube_mm']<=1
                           and v['tracking_rad']<=.18 and v['range_excess_ticks']<=1e-6
                           and v['no_hand_contact_seconds']<=.25 and not v['external']
                           and v['center_error_mm']<distance*1000-.3
                           and v['red_up_deg']<=max(15.,tilt(B)+.5)
                           and np.max(abs(np.diff(np.vstack([current,path]),axis=0)))<=2)
        logs.append(v);print(json.dumps(v),flush=True)
        if not v['accepted']:break
        accepted=save();current=path[-1].astype(float);accepted_rows.extend(path)
        np.savez_compressed(OUT/'candidate.npz',start=start,goals=np.asarray(accepted_rows,dtype=int),
                            end_qpos=d.qpos.copy(),end_qvel=d.qvel.copy(),end_ctrl=d.ctrl.copy())
    restore(accepted);C,R=pose()
    result=dict(status='offline_finite_physics_cube_seating_control_search',steps=logs,
                accepted_goals=len(accepted_rows),center_error_mm=float(np.linalg.norm(C-target_center)*1000),
                red_up_deg=tilt(R),initial_cube_registration_uncertified=True,
                finite_actuator_model=True,cube_pose_prescribed_during_rollout=False,
                hand_pose_prescribed_during_rollout=False,model=manifest,
                hardware_execution_allowed=False,real_devices_opened=False,
                elapsed_seconds=time.perf_counter()-began)
    (OUT/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('SUMMARY',json.dumps({k:result[k] for k in ('accepted_goals','center_error_mm','red_up_deg','elapsed_seconds')}),flush=True)

if __name__=='__main__':main()
