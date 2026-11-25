"""Perfect-state-feedback diagnostic in MuJoCo. Never a hardware controller."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time
from pathlib import Path
import mujoco,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1'
def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--pair',type=int,default=14);parser.add_argument('--direction',type=int,choices=(-1,1),default=1);parser.add_argument('--thumb-pair',action='store_true');parser.add_argument('--warm-contact',action='store_true');parser.add_argument('--warm-end',action='store_true');parser.add_argument('--patch-contact',action='store_true');parser.add_argument('--lift-mm',type=float,default=0.);parser.add_argument('--squeeze-mm',type=float,default=.12);parser.add_argument('--level-after-lift',action='store_true');parser.add_argument('--seed-folder');args=parser.parse_args()
    if not 0<=args.lift_mm<=20 or not .05<=args.squeeze_mm<=.6:raise ValueError('Offline lift/squeeze diagnostic bounds')
    if args.seed_folder:OUT=(ROOT/args.seed_folder).resolve();OUT.relative_to(ROOT)
    seed=json.loads((OUT/'seed.json').read_bytes());base=np.asarray(seed['commanded_hand_ticks']);residual=np.asarray(seed['response_residual_ticks']);cube_pose=np.asarray(seed['settled_cube_qpos']);up=np.asarray(seed['up_world']);up_local=np.asarray(seed.get('up_cube_normal',[1,0,0]));front_local=np.asarray(seed.get('front_cube_normal',[-1,0,0] if seed.get('up_face')=='green' else [0,-1,0]));lo,hi=limits()
    pairs=[v for v in json.loads((OUT/('thumb_opposed_pinch_endpoints.json' if args.thumb_pair else 'pinch_upper_corner_endpoints.json')).read_bytes())['pairs' if args.thumb_pair else 'adjacent_pairs'] if v['endpoint_promising']];pair=pairs[args.pair]
    fingers=('th',pair['finger']) if args.thumb_pair else ('if','mf')
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    stem=f'adaptive_{"thumb_" if args.thumb_pair else ""}pair{args.pair}_dir{args.direction}{"_contact_latched" if args.warm_contact else ""}{"_end_latched" if args.warm_end else ""}{"_patch_level" if args.patch_contact else ""}'
    if args.lift_mm:stem+=f'_lift{args.lift_mm:g}_squeeze{args.squeeze_mm:g}'+('_level' if args.level_after_lift else '')
    m,manifest=build(env,OUT/(stem+'_scene.xml'));set_cube_physics(m,.055,.1);dt=.002;m.opt.timestep=dt;m.dof_armature[:16]=1e6;m.opt.gravity[:]=-9.81*up
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube]);m.geom_friction[cube,0]=.3;palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};tips={f:m.geom(f+'_tip').id for f in fingers};indices={'if':np.array([7,5,6,4]),'mf':np.array([11,9,10,8]),'th':np.array([3,2,1,0])}
    d=mujoco.MjData(m);probe=mujoco.MjData(m)
    def put(data,goal,cube_state):
        data.qpos[:16]=ticks_to_sim(goal+residual);data.qpos[-7:]=cube_state;data.qvel[:]=0;data.ctrl[:]=data.qpos[:16];mujoco.mj_forward(m,data)
    def observe(data):
        contact=set();sp=sup=cp=0.;external=False
        for c in data.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,depth)
            if (a in support and hb) or (b in support and ha):sup=max(sup,depth)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cp=max(cp,depth)
                    if c.dist<.00015:contact.add(other)
                elif c.dist<.0004 and (other in support or m.geom(other).name=='floor'):external=True
        gaps={f:float(mujoco.mj_geomDistance(m,data,cube,g,1.,None)*1000) for f,g in tips.items()}
        return dict(tip_contact={f:g in contact for f,g in tips.items()},gaps_mm=gaps,self_overlap_mm=sp,support_overlap_mm=sup,cube_overlap_mm=cp,external_cube_contact=external)
    # Preserve each endpoint's intended corner height/tangent rather than chasing
    # its world point after the free cube changes pose during approach.
    B0=Rotation.from_quat(cube_pose[-4:][[1,2,3,0]]).as_matrix();put(probe,np.asarray(pair['goal_ticks']),cube_pose)
    local={f:B0.T@(probe.geom_xpos[g]-cube_pose[:3]) for f,g in tips.items()}
    side=({'th':[pair['thumb']['axis'],pair['thumb']['sign']],pair['finger']:[pair['finger_candidate']['side_axis'],pair['finger_candidate']['side_sign']]} if args.thumb_pair else {'if':pair['index_side'],'mf':pair['middle_side']})
    put(d,base,cube_pose);current=base.copy();goals=[];trace=[];events=[];start=time.perf_counter();stop=None;maxself=maxsup=maxcp=0.
    if args.warm_contact or args.warm_end:
        if args.warm_end:
            saved=np.load(OUT/f'replay_thumb_pair{args.pair}.npz');current=np.asarray(json.loads((OUT/f'replay_thumb_pair{args.pair}_review.json').read_bytes())['end_goal_ticks']);d.qpos[:]=saved['end_qpos']
        else:
            saved=np.load(OUT/f'rrt_thumb_pair{args.pair}_first_contact.npz');current=saved['goal_ticks'].copy();d.qpos[:]=saved['qpos']
        d.qpos[:16]=ticks_to_sim(current+residual);d.qvel[:]=0;mujoco.mj_forward(m,d);R=d.xmat[cb].reshape(3,3);local={f:R.T@(d.geom_xpos[g]-d.qpos[-7:-4]) for f,g in tips.items()}
    else:
        for _ in range(500):d.qpos[:16]=ticks_to_sim(current+residual);d.qvel[:16]=0;mujoco.mj_step(m,d)
    patches={f:np.zeros(3) for f in fingers}
    if args.patch_contact:
        mujoco.mj_forward(m,d);R=d.xmat[cb].reshape(3,3)
        for f,g in tips.items():
            matches=[c for c in d.contact if set((int(c.geom1),int(c.geom2)))==set((cube,g))]
            if not matches:raise RuntimeError('Requested contact patch is absent')
            point=min(matches,key=lambda c:c.dist).pos.copy();patches[f]=d.geom_xmat[g].reshape(3,3).T@(point-d.geom_xpos[g]);local[f]=R.T@(point-d.qpos[-7:-4]);local[f]-=up_local*(local[f]@up_local)
    initial=d.qpos[-7:].copy();mujoco.mj_forward(m,d);initial_B=d.xmat[cb].reshape(3,3).copy();reference_front=initial_B@front_local;reference_front-=up*(reference_front@up);reference_front/=np.linalg.norm(reference_front)
    def yaw():
        front=d.xmat[cb].reshape(3,3)@front_local;front-=up*(front@up);front/=max(1e-12,np.linalg.norm(front));return float(np.degrees(np.arctan2(up@np.cross(reference_front,front),reference_front@front)))
    def ik(finger,current,desired_B,state,desired_gap_mm=-.12,center_offset=None):
        ids=indices[finger];axis,sign=side[finger];normal=desired_B[:,axis]*sign;U=desired_B@up_local;tangent=np.cross(normal,U);tangent/=np.linalg.norm(tangent);target=state[:3]+desired_B@local[finger]+(np.zeros(3) if center_offset is None else center_offset);allowed=tips[finger]
        def objective(value):
            goal=current.astype(float).copy();goal[ids]=value;put(probe,goal,state);obs=observe(probe);p=probe.geom_xpos[allowed]+probe.geom_xmat[allowed].reshape(3,3)@patches[finger]
            tangential=np.array([(p-target)@tangent,(p-target)@U])*1000
            return np.r_[tangential,8*(obs['gaps_mm'][finger]-desired_gap_mm),15*obs['self_overlap_mm'],15*obs['support_overlap_mm'],15*max(0,obs['cube_overlap_mm']-.6),.001*(value-current[ids])]
        fit=least_squares(objective,np.clip(current[ids].astype(float),lo[ids]+.001,hi[ids]-.001),bounds=(lo[ids]+.001,hi[ids]-.001),diff_step=.00004,max_nfev=100)
        return np.rint(fit.x).astype(int)
    stable=0;stage='approach';stage_started=0.;last_yaw=0.;last_yaw_time=0.;target=current.copy();stepcount=0;blocked_step=None
    for gi in range(2400):
        mujoco.mj_forward(m,d);obs=observe(d);B=d.xmat[cb].reshape(3,3).copy();elapsed=gi*.075
        stable=stable+1 if all(obs['tip_contact'].values()) else 0
        if stage=='approach' and stable>=20:stage='squeeze' if args.lift_mm else 'yaw';stage_started=elapsed;events.append(dict(event='both_tips_sustained',time_s=elapsed,yaw_deg=yaw()))
        if stage=='approach' and elapsed>100:stop='no_sustained_two_tip_grasp';break
        if stage=='squeeze' and elapsed-stage_started>=2 and stable>=10:stage='lift';stage_started=elapsed;events.append(dict(event='lift_started',time_s=elapsed))
        if stage=='squeeze' and elapsed-stage_started>=12:stop='no_sustained_grasp_after_squeeze';break
        height_change=float((d.qpos[-7:-4]-initial[:3])@up)
        if stage=='lift' and height_change>=args.lift_mm*.001-.001 and stable>=10:stage='yaw';stage_started=elapsed;events.append(dict(event='lift_complete',time_s=elapsed,height_change_mm=height_change*1000))
        if stage=='lift' and elapsed-stage_started>=35:stop='lift_no_progress';break
        if stage=='yaw' and args.direction*yaw()>=85:stage='hold';stage_started=elapsed
        if stage=='hold' and elapsed-stage_started>=2:break
        if stage=='yaw' and elapsed-last_yaw_time>=10:
            if abs(yaw()-last_yaw)<1 and elapsed-stage_started>10:stop='yaw_no_progress';break
            last_yaw=yaw();last_yaw_time=elapsed
        near=min(obs['gaps_mm'].values())<3
        if stage=='approach' and all(obs['tip_contact'].values()):target=current.copy()
        elif (gi%20==0 or near) and stage!='hold':
            desired=Rotation.from_rotvec(up*np.radians(args.direction*1.5)).as_matrix()@B if stage=='yaw' else B
            if args.patch_contact:
                n=desired@up_local;axis=np.cross(n,up);length=np.linalg.norm(axis)
                if length>1e-10:desired=Rotation.from_rotvec(axis/length*np.arctan2(length,n@up)).as_matrix()@desired
            elif args.level_after_lift and stage=='yaw':
                n=desired@up_local;axis=np.cross(n,up);length=np.linalg.norm(axis)
                if length>1e-10:desired=Rotation.from_rotvec(axis/length*min(np.radians(.2),np.arctan2(length,n@up))).as_matrix()@desired
            offset=up*min(.001,max(0.,args.lift_mm*.001-height_change)) if stage=='lift' else np.zeros(3)
            target=current.copy()
            for finger in fingers:
                desired_gap=3. if stage=='approach' and args.thumb_pair and finger=='th' and obs['gaps_mm'][pair['finger']]>4 else (-args.squeeze_mm if stage in ('squeeze','lift','yaw') else -.12)
                target[indices[finger]]=ik(finger,target,desired,d.qpos[-7:].copy(),desired_gap,offset)
        step_limit=1 if near else 2
        goal=current+np.clip(target-current,-step_limit,step_limit);put(probe,goal,d.qpos[-7:].copy());screen=observe(probe)
        if screen['self_overlap_mm']>.01 or screen['support_overlap_mm']>.15:
            stop='screened_interpolation_self_or_support_contact';blocked_step=dict(current=current.tolist(),target=target.tolist(),tentative_goal=goal.tolist(),contacts=[dict(a=m.geom(int(c.geom1)).name,b=m.geom(int(c.geom2)).name,depth_mm=-float(c.dist)*1000) for c in probe.contact if c.dist<-.00001]);break
        a=ticks_to_sim(current+residual);b=ticks_to_sim(goal+residual);count=round((gi+1)*.075/dt)-stepcount;stepcount+=count
        for j in range(count):
            d.qpos[:16]=a+(b-a)*(j+1)/count;d.qvel[:16]=(b-a)/.075;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
            if j%5==0:
                o=observe(d);maxself=max(maxself,o['self_overlap_mm']);maxsup=max(maxsup,o['support_overlap_mm']);maxcp=max(maxcp,o['cube_overlap_mm'])
                if maxself>.01 or maxsup>.15 or maxcp>1 or o['external_cube_contact']:stop='free_cube_contact_or_clearance_rejection';break
            mujoco.mj_step(m,d)
        current=goal;goals.append(goal.copy())
        if gi%5==0:trace.append(d.qpos.copy())
        if gi%100==0:print(json.dumps(dict(step=gi,stage=stage,yaw_deg=yaw(),**observe(d))),flush=True)
        if stop:break
    mujoco.mj_forward(m,d);obs=observe(d);B=d.xmat[cb].reshape(3,3);tilt=float(np.degrees(np.arccos(np.clip((B@up_local)@up,-1,1))));result=dict(status='adaptive_after_flip_oracle_state_diagnostic',pair=args.pair,direction=args.direction,stop=stop,stage=stage,
        elapsed_wall_seconds=time.perf_counter()-start,path_nominal_seconds=len(goals)*.075,goals=len(goals),events=events,final_yaw_deg=yaw(),up_face=seed.get('up_face','red'),up_face_error_deg=tilt,final_contacts=obs,max_self_overlap_mm=maxself,max_support_overlap_mm=maxsup,max_cube_overlap_mm=maxcp,
        conditional_model_candidate=bool(stop is None and args.direction*yaw()>=85 and tilt<=15 and all(obs['tip_contact'].values())),
        initial_cube_not_measured=True,perfect_simulator_state_feedback_used=True,not_a_camera_feedback_controller=True,following_residual_assumed=True,hardware_execution_allowed=False,real_devices_opened=False)
    result['fingers']=fingers;result['thumb_opposed_mode']=args.thumb_pair;result['blocked_step']=blocked_step;result['requested_lift_mm']=args.lift_mm;result['squeeze_surface_target_mm']=args.squeeze_mm;result['height_change_mm']=float((d.qpos[-7:-4]-initial[:3])@up*1000)
    (OUT/(stem+'.json')).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');np.savez_compressed(OUT/(stem+'.npz'),goals=np.asarray(goals),qpos=np.asarray(trace),end_qpos=d.qpos.copy());print('SUMMARY',json.dumps(result),flush=True)
if __name__=='__main__':main()
