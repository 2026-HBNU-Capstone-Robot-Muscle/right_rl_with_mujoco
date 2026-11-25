"""Offline grasp/contact-point refinement and active-cube yaw comparison.

Cube pose and friction are explicit hypotheses. This file never authorizes
hardware, changes joint zeros, opens devices, or replaces existing candidates.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from grasp_place_plan import endpoints
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024'
SOURCE=ROOT/'grasped_yaw_trial_20261023_234707_886202/result.json'
MOVING=np.array([0,1,2,4,5,6,8,9,10]);NAMES=('if_tip','mf_tip','th_tip')


def main():
    OUT.mkdir(exist_ok=True)
    blob=SOURCE.read_bytes();trial=json.loads(blob)
    baseline=[s for s in trial['samples'] if s['phase']=='stage_4_fixed_grasp_observation'][-20:]
    base=endpoints()[1];actual=np.median([s['actual'] for s in baseline],axis=0);residual=actual-base
    scenario=json.loads((OUT/'completed_yaw_active_cube_replay.json').read_bytes())
    assert scenario['status']=='prescribed_measured_hand_active_cube_contact_diagnostic'
    center=np.array(scenario['placement_center_m'])
    paired=json.loads((ROOT/'slow_grasp_demo_20261023_182105_573825/on_paired_pose_review.json').read_bytes())
    rotations=np.array([h['cube_rotation_world'] for h in paired['hypotheses']
                       if h['source'].endswith('grid_diagnostic') and h['palm_branch']==0])
    cubeR=Rotation.from_matrix(rotations).mean().as_matrix();cubequat=Rotation.from_matrix(cubeR).as_quat()[[3,0,1,2]]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build_with_middle_thumb_hulls(env)
    set_cube_physics(m,side_m=.055,mass_kg=.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id;cube_body=int(m.geom_bodyid[cube])
    geoms=[m.geom(n).id for n in NAMES];bodies=[int(m.geom_bodyid[g]) for g in geoms]
    support={m.geom(n).id for n in ('support_glass','support_wood')};hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
    lower=np.array(limits['lower_ticks']);upper=np.array(limits['upper_ticks'])
    axis=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    def initialize(friction=.8,position_offset=None):
        mujoco.mj_resetData(m,d);m.geom_friction[cube,0]=friction
        offset=np.zeros(3) if position_offset is None else np.asarray(position_offset)
        d.qpos[:16]=ticks_to_sim(actual);d.qpos[-7:]=np.r_[center+offset,cubequat]
        d.ctrl[:]=ticks_to_sim(actual);mujoco.mj_forward(m,d)
    def fixed_cube_pose(goal,pose):
        d.qpos[:16]=ticks_to_sim(np.asarray(goal)+residual);d.qpos[-7:]=pose;d.qvel[:]=0
        mujoco.mj_forward(m,d)
    def simulate(goals,friction=.8,save=False,position_offset=None,response_offset_ticks=None,renderer=None,camera=None):
        initialize(friction,position_offset)
        response=np.zeros(16) if response_offset_ticks is None else np.asarray(response_offset_ticks)
        response=residual+response
        def step(goal,velocity=None):
            q=ticks_to_sim(goal+response);d.qpos[:16]=q;d.qvel[:16]=0 if velocity is None else velocity;d.ctrl[:]=q
            mujoco.mj_step(m,d)
        for _ in range(round(2./m.opt.timestep)):step(base)
        initial=d.qpos[-7:].copy();initialR=d.xmat[cube_body].reshape(3,3).copy()
        history=[];previous=base.copy();images=[]
        if renderer is not None:
            renderer.update_scene(d,camera=camera);images.extend([renderer.render().copy()]*4)
        for index,goal in enumerate(goals):
            dq=(ticks_to_sim(goal+response)-ticks_to_sim(previous+response))/.05
            cycles=round(.05/m.opt.timestep)
            for k in range(cycles):step(previous+(goal-previous)*(k+1)/cycles,dq)
            if save:history.append(dict(sample=index,center=d.qpos[-7:-4].tolist(),quat=d.qpos[-4:].tolist()))
            if renderer is not None and (index+1)%5==0:
                renderer.update_scene(d,camera=camera);images.append(renderer.render().copy())
            previous=goal
        for _ in range(round(1./m.opt.timestep)):step(previous)
        if renderer is not None:
            renderer.update_scene(d,camera=camera);images.extend([renderer.render().copy()]*4)
            np.savez_compressed(OUT/'contact_point_yaw_simulation_frames.npz',frames=np.asarray(images),fps=np.array(4.))
        rotation=Rotation.from_matrix(d.xmat[cube_body].reshape(3,3)@initialR.T).as_rotvec()
        translation=(d.qpos[-7:-4]-initial[:3])*1000
        yaw=float(np.degrees(rotation@axis));tilt=float(np.degrees(np.linalg.norm(rotation-axis*(rotation@axis))))
        return dict(friction=friction,yaw_about_gravity_up_deg=yaw,tilt_deg=tilt,
            lateral_translation_mm=float(np.linalg.norm(translation-axis*(translation@axis))),
            translation_mm=translation.tolist(),finite=bool(np.isfinite(d.qpos).all()),history=history,
            final_cube_contact_geoms=sorted({m.geom(int(c.geom2) if int(c.geom1)==cube else int(c.geom1)).name
                for c in d.contact if cube in (int(c.geom1),int(c.geom2)) and c.dist<.0003})),initial
    _,fixed=simulate([])
    lo=np.maximum(lower[MOVING],base[MOVING]-32);hi=np.minimum(upper[MOVING],base[MOVING]+32)
    def gap_objective(x):
        q=base.astype(float).copy();q[MOVING]=x;fixed_cube_pose(q,fixed)
        gaps=np.array([mujoco.mj_geomDistance(m,d,cube,g,1.,None) for g in geoms])*1000
        return np.r_[gaps+.15,.003*(x-base[MOVING])]
    closure=least_squares(gap_objective,base[MOVING].astype(float),bounds=(lo,hi),
        diff_step=.0007,max_nfev=120,ftol=1e-9,xtol=1e-9,gtol=1e-9)
    closed=base.copy();closed[MOVING]=np.rint(closure.x).astype(int);fixed_cube_pose(closed,fixed)
    gaps=[float(mujoco.mj_geomDistance(m,d,cube,g,1.,None))*1000 for g in geoms]
    print('CLOSURE',json.dumps(dict(delta_ticks=(closed-base).tolist(),gaps_mm=gaps)),flush=True)
    closure_rows=[np.rint(base+(closed-base)*i/20).astype(int) for i in range(1,21)]
    closure_motion,_=simulate(closure_rows)
    settled_cube=d.qpos[-7:].copy();C=settled_cube[:3].copy()
    contacts=[];local_points=[]
    for g,b in zip(geoms,bodies):
        points=[c.pos.copy() for c in d.contact if cube in (int(c.geom1),int(c.geom2))
                and g in (int(c.geom1),int(c.geom2)) and c.dist<.0003]
        if not points:
            line=np.zeros(6);gap=mujoco.mj_geomDistance(m,d,cube,g,1.,line)
            if gap>.0005:
                out=dict(status='contact_closure_not_established',closure_delta_ticks=(closed-base).tolist(),
                    static_gap_mm=gaps,closure_motion=closure_motion,missing_contact=m.geom(g).name,
                    post_closure_gap_mm=float(gap)*1000,hardware_execution_allowed=False,real_devices_opened=False)
                (OUT/'contact_point_yaw_search.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
                print('REJECTED',json.dumps(out),flush=True);return
            point=line[3:]
        else:point=np.mean(points,axis=0)
        local=d.xmat[b].reshape(3,3).T@(point-d.xpos[b])
        local_points.append(local);contacts.append(dict(geom=m.geom(g).name,body=m.body(b).name,point_world_m=point.tolist(),point_body_local_m=local.tolist()))
    initial_points=np.array([d.xpos[b]+d.xmat[b].reshape(3,3)@local for b,local in zip(bodies,local_points)])
    def contact_points(goal):
        fixed_cube_pose(goal,settled_cube)
        return np.array([d.xpos[b]+d.xmat[b].reshape(3,3)@local for b,local in zip(bodies,local_points)])
    lo=np.maximum(lower[MOVING],base[MOVING]-64);hi=np.minimum(upper[MOVING],base[MOVING]+64)
    attempts=[];candidates=[]
    for angle in (-5.,5.,-4.,4.,-3.,3.):
        previous=closed[MOVING].astype(float);goals=list(closure_rows);errors=[]
        for i in range(1,71):
            u=i/70;smooth=u*u*(3-2*u);rot=Rotation.from_rotvec(axis*np.radians(angle)*smooth).as_matrix()
            target=(initial_points-C)@rot.T+C
            def objective(x):
                q=base.astype(float).copy();q[MOVING]=x
                return np.r_[(contact_points(q)-target).ravel()*1000,.008*(x-closed[MOVING])]
            fit=least_squares(objective,previous,bounds=(lo,hi),diff_step=.0007,max_nfev=60,
                ftol=1e-8,xtol=1e-8,gtol=1e-8)
            previous=fit.x;q=base.copy();q[MOVING]=np.rint(previous).astype(int)
            goals.append(q);errors.append(np.linalg.norm(contact_points(q)-target,axis=1)*1000)
        goals=np.array(goals);self_overlap=support_overlap=0.
        for offset in (np.zeros(16),residual):
            for q in goals:
                fixed_cube_pose(q+offset-residual,settled_cube)
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                    if ha and hb:self_overlap=max(self_overlap,-float(c.dist)*1000)
                    if (a in support and hb) or (b in support and ha):support_overlap=max(support_overlap,-float(c.dist)*1000)
        row=dict(kinematic_target_degrees=angle,desired_physical_yaw_degrees=3.,
            max_contact_target_error_mm=float(np.max(errors)),
            max_goal_step_ticks=int(np.max(abs(np.diff(np.vstack([base,goals]),axis=0)))),
            max_goal_delta_ticks=int(np.max(abs(goals-base))),self_overlap_mm=self_overlap,support_overlap_mm=support_overlap)
        if row['max_goal_step_ticks']<=2 and self_overlap<=1e-6 and support_overlap<=1e-6:
            replay=[]
            for friction in (.4,.8,1.2):
                result,_=simulate(goals,friction);replay.append(result)
            row['active_cube_replays']=replay
            row['promising']=all(x['finite'] and 2<=x['yaw_about_gravity_up_deg']<=5
                and x['tilt_deg']<=3 and x['lateral_translation_mm']<=5
                and any(name.startswith('palm_collision') for name in x['final_cube_contact_geoms'])
                for x in replay)
            if row['promising']:
                extra=[]
                cases=[(.6,cubeR[:,0]*.001,None),(1.,-cubeR[:,0]*.001,None),
                       (.6,cubeR[:,1]*.001,None),(1.,-cubeR[:,1]*.001,None),
                       (.8,np.zeros(3),np.full(16,4)),(.8,np.zeros(3),np.full(16,-4))]
                for friction,offset,response in cases:
                    result,_=simulate(goals,friction,position_offset=offset,response_offset_ticks=response)
                    result.update(position_offset_m=offset.tolist(),response_offset_ticks=None if response is None else response.tolist())
                    extra.append(result)
                row['held_out_cases']=extra
                row['held_out_pass']=all(x['finite'] and 1<=x['yaw_about_gravity_up_deg']<=6
                    and x['tilt_deg']<=4 and x['lateral_translation_mm']<=6
                    and any(name.startswith('palm_collision') for name in x['final_cube_contact_geoms']) for x in extra)
                # Retain the failed angle criterion explicitly. A directionally
                # consistent exploratory candidate is not a robust controller.
                row['direction_consistent_in_evaluated_scenarios']=all(
                    x['finite'] and x['yaw_about_gravity_up_deg']>0 for x in replay+extra)
                if row['direction_consistent_in_evaluated_scenarios']:
                    candidates.append(dict(summary=row,goal_ticks=goals.tolist()))
        else:row['promising']=False
        attempts.append(row);print('CONTACT CANDIDATE',json.dumps(row),flush=True)
    if candidates:
        camera=mujoco.MjvCamera();camera.lookat[:]=[.125,0.,.0];camera.distance=.36;camera.azimuth=135.;camera.elevation=-25.
        try:
            with mujoco.Renderer(m,height=480,width=640) as renderer:
                rendered,_=simulate(np.asarray(candidates[0]['goal_ticks']),.8,renderer=renderer,camera=camera)
                print('SIMULATION_FRAMES_SAVED',json.dumps({k:v for k,v in rendered.items() if k!='history'}),flush=True)
        except Exception as exc:print('SIMULATION_RENDER_ERROR',str(exc),flush=True)
    out=dict(status='contact_point_kinematic_and_active_cube_diagnostic',source=str(SOURCE.relative_to(ROOT)),
        source_sha256=hashlib.sha256(blob).hexdigest(),closure_delta_ticks=(closed-base).tolist(),
        base_goal_ticks=base.tolist(),recorded_following_residual_ticks=residual.tolist(),
        closure_static_gaps_mm=gaps,closure_motion=closure_motion,contact_points=contacts,
        axis_world=axis.tolist(),attempts=attempts,promising_candidates=candidates,
        hardware_execution_allowed=False,marker_registration_verified=False,
        prescribed_hand_response_hypothesis=True,cube_friction_measured=False,real_devices_opened=False)
    (OUT/'contact_point_yaw_search.json').write_text(json.dumps(out,indent=2),encoding='utf-8')


if __name__=='__main__':main()
