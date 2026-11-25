"""Bounded device-free trajectory search scored on cube dynamics, not tip yaw.

This is exploratory planning with prescribed measured-baseline hand response.
It is not SAC, physical calibration, or motor execution approval.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/book_yaw_measured_response'
MOVING=np.array([0,1,2,4,5,6,7,8,9,10])


def main():
    source=OUT/'measured_baseline_source.json';data=source.read_bytes();src=json.loads(data)
    base=np.array(src['grasp_goal_ticks']);actual=np.array(src['actual_baseline_ticks']);residual=actual-base
    prior=json.loads((OUT/'relative_candidate_replay.json').read_bytes())
    center=np.array(prior['selected_placement_hypothesis']['center_m']);R=np.array(prior['cube_rotation_world_hypothesis'])
    quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
    previous=json.loads((OUT/'positive_kinematic_contact_search.json').read_bytes())
    closed=base+np.array(previous['attempts'][0]['closure_delta_ticks'])
    count=max(20,int(np.ceil(max(abs(closed-base))/2)))
    closure=np.array([np.rint(base+(closed-base)*k/count).astype(int) for k in range(1,count+1)])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build_book(env,OUT/'dynamic_search_scene.xml',book_projection_mm=-10.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
    set_cube_physics(m,side_m=.055,mass_kg=.1);d=mujoco.MjData(m)
    cube=m.geom('cube').id;body=int(m.geom_bodyid[cube]);up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
    lo=np.maximum(np.array(limits['lower_ticks'])[MOVING],base[MOVING]-64)
    hi=np.minimum(np.array(limits['upper_ticks'])[MOVING],base[MOVING]+64)
    turns=100
    def route(x):
        final=base.copy();final[MOVING]=np.rint(x).astype(int)
        turn=[]
        for k in range(1,turns+1):
            u=k/turns;s=u*u*(3-2*u);turn.append(np.rint(closed+(final-closed)*s).astype(int))
        return np.vstack([closure,turn])
    def collision_free(path):
        if np.max(abs(np.diff(np.vstack([base,path]),axis=0)))>2:return False
        for response in (np.zeros(16),residual):
            for q in path[::5]:
                d.qpos[:16]=ticks_to_sim(q+response);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];d.qvel[:]=0
                mujoco.mj_forward(m,d)
                for c in d.contact:
                    if c.dist>=-1e-9:continue
                    a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                    if ha and hb or a in supports and hb or b in supports and ha:return False
        return True
    def simulate(path,friction=.8,offset=None):
        mujoco.mj_resetData(m,d);m.geom_friction[cube,0]=friction
        d.qpos[:16]=ticks_to_sim(actual);d.ctrl[:]=d.qpos[:16]
        d.qpos[-7:]=np.r_[center+(np.zeros(3) if offset is None else offset),quat];mujoco.mj_forward(m,d)
        previous=base.astype(float)
        def step(q,vel=None):
            a=ticks_to_sim(q+residual);d.qpos[:16]=a;d.ctrl[:]=a
            d.qvel[:16]=0 if vel is None else vel;mujoco.mj_step(m,d)
        for _ in range(round(2/m.opt.timestep)):step(base)
        initial=d.qpos[-7:].copy();R0=d.xmat[body].reshape(3,3).copy()
        for goal in path:
            velocity=(ticks_to_sim(goal+residual)-ticks_to_sim(previous+residual))/.05
            cycles=round(.05/m.opt.timestep)
            for k in range(cycles):step(previous+(goal-previous)*(k+1)/cycles,velocity)
            previous=goal
        for _ in range(round(1/m.opt.timestep)):step(previous)
        rv=Rotation.from_matrix(d.xmat[body].reshape(3,3)@R0.T).as_rotvec();shift=(d.qpos[-7:-4]-initial[:3])*1000
        return dict(yaw_deg=float(np.degrees(rv@up)),tilt_deg=float(np.degrees(np.linalg.norm(rv-up*(rv@up)))),
            lateral_shift_mm=float(np.linalg.norm(shift-up*(shift@up))),finite=bool(np.isfinite(d.qpos).all()))
    control_path=np.vstack([closure,np.tile(closed,(turns,1))])
    controls={f:simulate(control_path,f) for f in (.4,.8,1.2)}
    def score(run,control):
        if not run['finite']:return -1000.
        value=-(run['yaw_deg']-control['yaw_deg'])
        value-=8*max(0.,run['tilt_deg']-3.)+5*max(0.,run['lateral_shift_mm']-4.)
        return float(value)
    rng=np.random.default_rng(2026102422);mean=closed[MOVING].astype(float);history=[];all_elites=[]
    for generation,spread in enumerate((40.,26.,17.,11.)):
        pop=[np.clip(mean,lo,hi)]
        pop.extend(np.clip(mean+rng.normal(0,spread,len(MOVING)),lo,hi) for _ in range(31))
        rows=[]
        for i,x in enumerate(pop):
            path=route(x)
            if not collision_free(path):continue
            run=simulate(path,.8);rows.append(dict(x=np.rint(x).astype(int).tolist(),
                score=score(run,controls[.8]),nominal=run))
        rows.sort(key=lambda s:s['score'],reverse=True)
        robust=[]
        for row in rows[:8]:
            path=route(row['x']);runs={.8:row['nominal'],.4:simulate(path,.4),1.2:simulate(path,1.2)}
            row['runs']={str(k):v for k,v in runs.items()}
            row['robust_score']=min(score(run,controls[f]) for f,run in runs.items());robust.append(row)
        if not robust:raise RuntimeError('No collision-free dynamic candidate')
        robust.sort(key=lambda s:s['robust_score'],reverse=True)
        mean=np.mean([row['x'] for row in robust[:5]],axis=0);all_elites.extend(robust)
        history.append(dict(generation=generation,spread_ticks=spread,valid=len(rows),elites=robust))
        print('GENERATION',generation,'best_worst_case_delta_yaw',robust[0]['robust_score'],
            'best_delta',(np.array(robust[0]['x'])-base[MOVING]).tolist(),flush=True)
    candidates=[]
    for row in sorted(all_elites,key=lambda s:s['robust_score'],reverse=True)[:5]:
        if row['robust_score']<1.:continue
        path=route(row['x']);runs=[]
        for friction,offset in ((.4,np.zeros(3)),(.8,np.zeros(3)),(1.2,np.zeros(3)),
            (.6,R[:,0]*.001),(1.,-R[:,0]*.001),(.6,R[:,1]*.001),(1.,-R[:,1]*.001)):
            action=simulate(path,friction,offset);control=simulate(control_path,friction,offset)
            runs.append(dict(friction=friction,offset_m=offset.tolist(),action=action,no_turn=control,
                delta_yaw_deg=action['yaw_deg']-control['yaw_deg']))
        accepted=all(s['action']['finite'] and s['action']['yaw_deg']<=-1.5 and s['delta_yaw_deg']<=-1.
            and s['action']['tilt_deg']<4 and s['action']['lateral_shift_mm']<5 for s in runs)
        print('FINAL_SENSITIVITY','accepted',accepted,'delta_yaw',[s['delta_yaw_deg'] for s in runs],flush=True)
        if accepted:
            candidates.append(dict(base_goal_ticks=base.tolist(),goal_ticks=path.tolist(),closure_samples=count,
                rotation_samples=turns,closure_delta_ticks=(closed-base).tolist(),moving_motor_ids=(MOVING+1).tolist(),
                runs=runs,max_goal_step_ticks=int(np.max(abs(np.diff(np.vstack([base,path]),axis=0)))),
                max_goal_delta_ticks=int(np.max(abs(path-base)))))
            break
    out=dict(source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(data).hexdigest(),
        measured_baseline_ticks=actual.tolist(),controls=controls,history=history,candidates=candidates,
        hardware_execution_allowed=False,hardware_commands_sent=0,physical_rotation_verified=False,
        marker_to_model_registration_verified=False,prescribed_hand_response_not_force_prediction=True,
        optimizer='bounded_cross_entropy_search_not_SAC',desired_physical_direction='clockwise_viewed_from_above')
    (OUT/'dynamic_contact_search.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print('CANDIDATES',len(candidates),flush=True)


if __name__=='__main__':main()
