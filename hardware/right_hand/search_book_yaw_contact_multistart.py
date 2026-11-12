"""Measured-baseline contact search with one bounded index-root contribution.

All goals remain within 64 ticks of the reached grasp. Middle/root/ring/thumb
base targets remain fixed. Unknown cube placement and friction are hypotheses.
The multi-start closure search avoids interpreting a flat distance derivative
as an actual physical joint stop. This script opens no devices.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/book_yaw_measured_response'
MOVING=np.array([0,1,2,4,5,6,7,8,9,10]);NAMES=('if_tip','mf_tip','th_tip')


def main():
    source=OUT/'measured_baseline_source.json';raw=source.read_bytes();src=json.loads(raw)
    base=np.array(src['grasp_goal_ticks']);actual=np.array(src['actual_baseline_ticks']);residual=actual-base
    prior=json.loads((OUT/'relative_candidate_replay.json').read_bytes())
    center=np.array(prior['selected_placement_hypothesis']['center_m']);R=np.array(prior['cube_rotation_world_hypothesis'])
    quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build_book(env,OUT/'multistart_scene.xml',book_projection_mm=-10.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
    set_cube_physics(m,side_m=.055,mass_kg=.1);d=mujoco.MjData(m)
    cube=m.geom('cube').id;body=int(m.geom_bodyid[cube]);tips=[m.geom(n).id for n in NAMES]
    bodies=[int(m.geom_bodyid[g]) for g in tips];up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
    lo=np.maximum(np.array(limits['lower_ticks'])[MOVING],base[MOVING]-64)
    hi=np.minimum(np.array(limits['upper_ticks'])[MOVING],base[MOVING]+64)
    def simulate(goals,friction=.8,offset=None):
        mujoco.mj_resetData(m,d);m.geom_friction[cube,0]=friction
        d.qpos[:16]=ticks_to_sim(actual);d.ctrl[:]=d.qpos[:16]
        d.qpos[-7:]=np.r_[center+(np.zeros(3) if offset is None else offset),quat];mujoco.mj_forward(m,d)
        previous=base.astype(float)
        def step(q,velocity=None):
            a=ticks_to_sim(q+residual);d.qpos[:16]=a;d.ctrl[:]=a
            d.qvel[:16]=np.zeros(16) if velocity is None else velocity;mujoco.mj_step(m,d)
        for _ in range(round(2/m.opt.timestep)):step(base)
        initial=d.qpos[-7:].copy();R0=d.xmat[body].reshape(3,3).copy()
        for goal in goals:
            velocity=(ticks_to_sim(goal+residual)-ticks_to_sim(previous+residual))/.05
            cycles=round(.05/m.opt.timestep)
            for k in range(cycles):step(previous+(goal-previous)*(k+1)/cycles,velocity)
            previous=goal
        for _ in range(round(1/m.opt.timestep)):step(previous)
        rv=Rotation.from_matrix(d.xmat[body].reshape(3,3)@R0.T).as_rotvec();shift=(d.qpos[-7:-4]-initial[:3])*1000
        return dict(yaw_deg=float(np.degrees(rv@up)),tilt_deg=float(np.degrees(np.linalg.norm(rv-up*(rv@up)))),
            lateral_shift_mm=float(np.linalg.norm(shift-up*(shift@up))),finite=bool(np.isfinite(d.qpos).all()))
    simulate([]);settled=d.qpos[-7:].copy()
    def pose(q,cube_pose):
        d.qpos[:16]=ticks_to_sim(q+residual);d.qpos[-7:]=cube_pose;d.qvel[:]=0;mujoco.mj_forward(m,d)
    def goal(x):
        q=base.astype(float).copy();q[MOVING]=x;return q
    def gaps(q):
        pose(q,settled)
        return np.array([mujoco.mj_geomDistance(m,d,cube,g,1.,None)*1000 for g in tips])
    def objective(x):return np.r_[gaps(goal(x))+.1,.003*(x-base[MOVING])]
    rng=np.random.default_rng(20261024);seeds=[base[MOVING].astype(float)]
    for k in range(12):
        seed=np.clip(base[MOVING]+rng.uniform(-56,56,len(MOVING)),lo,hi)
        if k<6:seed[np.flatnonzero(MOVING==7)[0]]=base[7]+(k+1)*8
        seeds.append(seed)
    closures=[]
    for i,seed in enumerate(seeds):
        fit=least_squares(objective,seed,bounds=(lo,hi),diff_step=.0007,max_nfev=100)
        q=np.rint(goal(fit.x)).astype(int);gap=gaps(q)
        closures.append((float(max(abs(gap))),float(np.linalg.norm(q-base)),q))
        print('CLOSURE_SEED',i,'max_gap_mm',float(max(abs(gap))),'delta',(q-base).tolist(),flush=True)
    attempts=[];candidates=[]
    for worst,_,closed in sorted(closures,key=lambda c:c[:2])[:3]:
        count=max(20,int(np.ceil(max(abs(closed-base))/2)))
        closure=np.array([np.rint(base+(closed-base)*i/count).astype(int) for i in range(1,count+1)])
        simulate(closure);held=d.qpos[-7:].copy();C=held[:3];locals=[];contact_names=[]
        for g,b in zip(tips,bodies):
            line=np.zeros(6);gap=float(mujoco.mj_geomDistance(m,d,cube,g,1.,line))
            contacts=[c.pos.copy() for c in d.contact if cube in (int(c.geom1),int(c.geom2))
                and g in (int(c.geom1),int(c.geom2)) and c.dist<.0003]
            point=np.mean(contacts,axis=0) if contacts else line[3:].copy() if gap<=.0005 else None
            if point is not None:locals.append(d.xmat[b].reshape(3,3).T@(point-d.xpos[b]));contact_names.append(m.geom(g).name)
        if len(locals)!=3:
            print('NO_THREE_CONTACTS_AFTER_SETTLING',contact_names,flush=True);continue
        def points(q):
            pose(q,held)
            return np.array([d.xpos[b]+d.xmat[b].reshape(3,3)@p for b,p in zip(bodies,locals)])
        initial_points=points(closed)
        # Contact slip can make object yaw oppose fingertip kinematic yaw.
        # The earlier negative kinematic sweep failed the object-direction test.
        # Evaluate the complementary sweep, still against negative OBJECT yaw.
        for angle in (3.,5.,8.):
            previous=closed[MOVING].astype(float);path=list(closure);errors=[]
            for i in range(1,101):
                u=i/100;s=u*u*(3-2*u);turn=Rotation.from_rotvec(up*np.radians(angle)*s).as_matrix()
                target=(initial_points-C)@turn.T+C
                def turn_error(x):return np.r_[(points(goal(x))-target).ravel()*1000,.008*(x-closed[MOVING])]
                fit=least_squares(turn_error,previous,bounds=(lo,hi),diff_step=.0007,max_nfev=60)
                previous=fit.x;q=np.rint(goal(previous)).astype(int);path.append(q)
                errors.append(float(max(np.linalg.norm(points(q)-target,axis=1))*1000))
            # Retiming is allowed; an occasional rounded IK jump is subdivided
            # rather than raising the unchanged two-tick motor step bound.
            bounded=[];last=base.copy()
            for q in path:
                count_step=max(1,int(np.ceil(max(abs(q-last))/2)))
                bounded.extend(np.rint(last+(q-last)*i/count_step).astype(int) for i in range(1,count_step+1))
                last=q.copy()
            path=np.array(bounded);self_mm=support_mm=0.
            for response in (np.zeros(16),residual):
                for q in path:
                    pose(q+response-residual,held)
                    for c in d.contact:
                        a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                        if ha and hb:self_mm=max(self_mm,-float(c.dist)*1000)
                        if (a in supports and hb) or (b in supports and ha):support_mm=max(support_mm,-float(c.dist)*1000)
            rotation_count=len(path)-count
            row=dict(kinematic_angle_deg=angle,closure_samples=count,rotation_samples=rotation_count,
                closure_delta_ticks=(closed-base).tolist(),max_goal_step_ticks=int(np.max(abs(np.diff(np.vstack([base,path]),axis=0)))),
                max_goal_delta_ticks=int(np.max(abs(path-base))),self_overlap_mm=self_mm,
                support_overlap_mm=support_mm,max_contact_target_error_mm=max(errors))
            if row['max_goal_step_ticks']<=2 and self_mm<=1e-6 and support_mm<=1e-6:
                runs=[]
                for friction,offset in ((.4,np.zeros(3)),(.8,np.zeros(3)),(1.2,np.zeros(3)),
                    (.6,R[:,0]*.001),(1.,-R[:,0]*.001),(.6,R[:,1]*.001),(1.,-R[:,1]*.001)):
                    action=simulate(path,friction,offset)
                    control=simulate(np.vstack([closure,np.tile(closed,(rotation_count,1))]),friction,offset)
                    runs.append(dict(friction=friction,offset_m=offset.tolist(),action=action,no_turn=control,
                        delta_yaw_deg=action['yaw_deg']-control['yaw_deg']))
                row['runs']=runs
                row['direction_consistent']=all(s['action']['finite'] and s['action']['tilt_deg']<5
                    and s['action']['lateral_shift_mm']<6 and s['delta_yaw_deg']<=-1. for s in runs)
                if row['direction_consistent']:
                    candidates.append(dict(summary=row,base_goal_ticks=base.tolist(),goal_ticks=path.tolist(),
                        closure_samples=count,rotation_samples=rotation_count,closure_delta_ticks=(closed-base).tolist()))
            attempts.append(row);print('TURN',json.dumps(row),flush=True)
    result=dict(source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(raw).hexdigest(),
        measured_baseline_ticks=actual.tolist(),attempts=attempts,candidates=candidates,
        moving_motor_ids=(MOVING+1).tolist(),physical_rotation_verified=False,
        prescribed_hand_response_not_force_prediction=True,marker_registration_verified=False,
        hardware_execution_allowed=False,hardware_commands_sent=0)
    (OUT/'positive_kinematic_contact_search.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('CANDIDATES',len(candidates),flush=True)


if __name__=='__main__':main()
