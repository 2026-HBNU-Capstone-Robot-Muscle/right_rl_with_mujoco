"""New contact-point candidates at the actually completed shortened grasp.

No device access. Placement/friction are explicit hypotheses, and prescribed
hand response does not establish actuator or grip-force transfer to hardware.
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
OUT=ROOT/'right_hand_rotation_20261024/book_grasp_yaw_diagnostic'
SOURCE=ROOT/'book_cube_grasp_trial_20261024_212855_824047/result.json'
MOVING=np.array([0,1,2,4,5,6,8,9,10]);NAMES=('if_tip','mf_tip','th_tip')
DESIRED_SIGN=-1.  # clockwise viewed from above; specified before evaluation


def main():
    trial_blob=SOURCE.read_bytes();trial=json.loads(trial_blob)
    prior=json.loads((OUT/'relative_candidate_replay.json').read_bytes())
    base=np.array(trial['grasp_goal_ticks']);actual=np.array(prior['measured_grasp_ticks']);residual=actual-base
    center=np.array(prior['selected_placement_hypothesis']['center_m'])
    cubeR=np.array(prior['cube_rotation_world_hypothesis']);cubequat=Rotation.from_matrix(cubeR).as_quat()[[3,0,1,2]]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build_book(env,OUT/'search_scene.xml',book_projection_mm=-10.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
    set_cube_physics(m,side_m=.055,mass_kg=.1);d=mujoco.MjData(m)
    cube=m.geom('cube').id;cube_body=int(m.geom_bodyid[cube]);geoms=[m.geom(n).id for n in NAMES]
    bodies=[int(m.geom_bodyid[g]) for g in geoms];up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
    lower=np.array(limits['lower_ticks']);upper=np.array(limits['upper_ticks'])

    def initialize(friction=.8,offset=None):
        mujoco.mj_resetData(m,d);m.geom_friction[cube,0]=friction
        d.qpos[:16]=ticks_to_sim(actual);d.ctrl[:]=d.qpos[:16]
        d.qpos[-7:]=np.r_[center+(np.zeros(3) if offset is None else offset),cubequat]
        mujoco.mj_forward(m,d)

    def simulate(goals,friction=.8,offset=None):
        initialize(friction,offset);previous=base.astype(float)
        def step(goal,velocity=None):
            q=ticks_to_sim(goal+residual);d.qpos[:16]=q;d.qvel[:16]=0 if velocity is None else velocity;d.ctrl[:]=q
            mujoco.mj_step(m,d)
        for _ in range(round(2./m.opt.timestep)):step(base)
        start=d.qpos[-7:].copy();R0=d.xmat[cube_body].reshape(3,3).copy()
        for goal in goals:
            dq=(ticks_to_sim(goal+residual)-ticks_to_sim(previous+residual))/.05
            cycles=round(.05/m.opt.timestep)
            for k in range(cycles):step(previous+(goal-previous)*(k+1)/cycles,dq)
            previous=goal
        for _ in range(round(1./m.opt.timestep)):step(previous)
        rot=Rotation.from_matrix(d.xmat[cube_body].reshape(3,3)@R0.T).as_rotvec();shift=(d.qpos[-7:-4]-start[:3])*1000
        contacts=sorted({m.geom(int(c.geom2) if int(c.geom1)==cube else int(c.geom1)).name for c in d.contact
            if cube in (int(c.geom1),int(c.geom2)) and c.dist<.0003})
        return dict(yaw_deg=float(np.degrees(rot@up)),tilt_deg=float(np.degrees(np.linalg.norm(rot-up*(rot@up)))),
            lateral_shift_mm=float(np.linalg.norm(shift-up*(shift@up))),translation_mm=shift.tolist(),
            cube_contact_geoms=contacts,finite=bool(np.isfinite(d.qpos).all()))

    def fixed_pose(goal,pose):
        d.qpos[:16]=ticks_to_sim(np.asarray(goal)+residual);d.qpos[-7:]=pose;d.qvel[:]=0;mujoco.mj_forward(m,d)

    simulate([]);fixed=d.qpos[-7:].copy()
    initial_gaps=[float(mujoco.mj_geomDistance(m,d,cube,g,1.,None))*1000 for g in geoms]
    print('INITIAL_GAPS',initial_gaps,flush=True)
    attempts=[];candidates=[]
    for closure_limit in (18,32,64):
        lo=np.maximum(lower[MOVING],base[MOVING]-closure_limit)
        hi=np.minimum(upper[MOVING],base[MOVING]+closure_limit)
        def closure_objective(x):
            q=base.astype(float).copy();q[MOVING]=x;fixed_pose(q,fixed)
            gaps=np.array([mujoco.mj_geomDistance(m,d,cube,g,1.,None)*1000 for g in geoms])
            return np.r_[gaps+.1,.003*(x-base[MOVING])]
        fit=least_squares(closure_objective,base[MOVING].astype(float),bounds=(lo,hi),diff_step=.0007,max_nfev=100,
            ftol=1e-9,xtol=1e-9,gtol=1e-9)
        closed=base.copy();closed[MOVING]=np.rint(fit.x).astype(int)
        count=max(20,int(np.ceil(max(abs(closed-base))/2)))
        closure=np.array([np.rint(base+(closed-base)*i/count).astype(int) for i in range(1,count+1)])
        closure_result=simulate(closure);settled=d.qpos[-7:].copy();C=settled[:3]
        contacts=[];locals=[];gaps=[]
        for g,b in zip(geoms,bodies):
            line=np.zeros(6);gap=float(mujoco.mj_geomDistance(m,d,cube,g,1.,line));gaps.append(gap*1000)
            points=[c.pos.copy() for c in d.contact if cube in (int(c.geom1),int(c.geom2))
                and g in (int(c.geom1),int(c.geom2)) and c.dist<.0003]
            if points:point=np.mean(points,axis=0)
            elif gap<=.0005:point=line[3:].copy()
            else:point=None
            if point is not None:
                locals.append(d.xmat[b].reshape(3,3).T@(point-d.xpos[b]));contacts.append(m.geom(g).name)
        record=dict(closure_limit_ticks=closure_limit,closure_delta_ticks=(closed-base).tolist(),
            post_closure_gaps_mm=gaps,closure_motion=closure_result,contact_geoms=contacts)
        print('CLOSURE',json.dumps(record),flush=True)
        if len(locals)!=3:
            attempts.append(record);continue
        def points(goal):
            fixed_pose(goal,settled)
            return np.array([d.xpos[b]+d.xmat[b].reshape(3,3)@p for b,p in zip(bodies,locals)])
        initial_points=points(closed)
        lo=np.maximum(lower[MOVING],base[MOVING]-64);hi=np.minimum(upper[MOVING],base[MOVING]+64)
        for angle in (3.,-3.,5.,-5.,8.,-8.):
            previous=closed[MOVING].astype(float);path=list(closure);errors=[]
            for i in range(1,71):
                u=i/70;smooth=u*u*(3-2*u);R=Rotation.from_rotvec(up*np.radians(angle)*smooth).as_matrix()
                target=(initial_points-C)@R.T+C
                def objective(x):
                    q=base.astype(float).copy();q[MOVING]=x
                    return np.r_[(points(q)-target).ravel()*1000,.008*(x-closed[MOVING])]
                fit=least_squares(objective,previous,bounds=(lo,hi),diff_step=.0007,max_nfev=60,
                    ftol=1e-8,xtol=1e-8,gtol=1e-8)
                previous=fit.x;q=base.copy();q[MOVING]=np.rint(previous).astype(int);path.append(q)
                errors.append(float(max(np.linalg.norm(points(q)-target,axis=1))*1000))
            path=np.array(path);self_depth=support_depth=0.
            for response in (np.zeros(16),residual):
                for q in path:
                    fixed_pose(q+response-residual,settled)
                    for c in d.contact:
                        a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                        if ha and hb:self_depth=max(self_depth,-float(c.dist)*1000)
                        if (a in supports and hb) or (b in supports and ha):support_depth=max(support_depth,-float(c.dist)*1000)
            row=dict(closure_limit_ticks=closure_limit,kinematic_angle_deg=angle,
                max_goal_step_ticks=int(np.max(abs(np.diff(np.vstack([base,path]),axis=0)))),
                max_goal_delta_ticks=int(np.max(abs(path-base))),max_contact_target_error_mm=max(errors),
                self_overlap_mm=self_depth,support_overlap_mm=support_depth)
            if row['max_goal_step_ticks']<=2 and self_depth<=1e-6 and support_depth<=1e-6:
                runs=[]
                for friction in (.4,.8,1.2):
                    action=simulate(path,friction);no_turn=simulate(np.vstack([closure,np.tile(closed,(70,1))]),friction)
                    runs.append(dict(friction=friction,action=action,no_turn=no_turn,
                        delta_yaw_deg=action['yaw_deg']-no_turn['yaw_deg']))
                row['runs']=runs
                row['promising']=all(x['action']['finite'] and x['action']['tilt_deg']<=5
                    and x['action']['lateral_shift_mm']<=6 and DESIRED_SIGN*x['delta_yaw_deg']>=.5 for x in runs)
                if row['promising']:
                    held=[]
                    for friction,offset in ((.6,cubeR[:,0]*.001),(1.,-cubeR[:,0]*.001),
                                           (.6,cubeR[:,1]*.001),(1.,-cubeR[:,1]*.001)):
                        action=simulate(path,friction,offset);no_turn=simulate(np.vstack([closure,np.tile(closed,(70,1))]),friction,offset)
                        held.append(dict(friction=friction,offset_m=offset.tolist(),action=action,no_turn=no_turn,
                            delta_yaw_deg=action['yaw_deg']-no_turn['yaw_deg']))
                    row['held_out']=held
                    row['evaluated_direction_consistent']=all(x['action']['finite'] and x['action']['tilt_deg']<=5
                        and x['action']['lateral_shift_mm']<=6 and DESIRED_SIGN*x['delta_yaw_deg']>=.5 for x in held)
                    if row['evaluated_direction_consistent']:
                        candidates.append(dict(summary=row,base_goal_ticks=base.tolist(),goal_ticks=path.tolist(),
                            closure_samples=len(closure),rotation_samples=70,closure_delta_ticks=(closed-base).tolist()))
            else:row['promising']=False
            attempts.append(row);print('TURN',json.dumps(row),flush=True)
        if candidates:break
    result=dict(source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(trial_blob).hexdigest(),
        base_goal_ticks=base.tolist(),actual_grasp_ticks=actual.tolist(),initial_gaps_mm=initial_gaps,
        attempts=attempts,candidates=candidates,hardware_execution_allowed=False,physical_rotation_verified=False,
        marker_to_model_registration_verified=False,placement_and_friction_are_hypotheses=True,
        prescribed_hand_response_not_force_prediction=True,hardware_commands_sent=0)
    result['desired_physical_yaw_degrees']=-3.
    (OUT/'contact_search_clockwise.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('CANDIDATES',len(candidates),flush=True)


if __name__=='__main__':main()
