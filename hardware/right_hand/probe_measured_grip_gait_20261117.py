"""Offline replan/turn/reposition diagnostic; finish at the unchanged measured grip.

The cube is free throughout integration. IK uses separate data and cannot reset
the integrated cube. Its initial placement remains an unmeasured hypothesis.
"""
from pathlib import Path
import argparse,hashlib,json,time
import mujoco,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,sim_to_ticks,SIM_TO_MOTOR_IDS,TICK
from research_rotation_body_hold_20261104 import restore,native,inspect_for

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261117/user_measured_grip'
MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
INITIAL=BASE/'model_hold_probe/end.npz'
REFERENCE=BASE/'reference.json'


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--middle-retreat',type=int,choices=(0,120,200),default=200)
    ap.add_argument('--coarse-angle',type=float,default=80.)
    ap.add_argument('--coordinated-reset',action='store_true')
    ap.add_argument('--balanced-preload',action='store_true')
    ap.add_argument('--middle-support-reset',action='store_true')
    ap.add_argument('--middle-lateral',type=int,choices=(0,160),default=0)
    args=ap.parse_args()
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    d=restore(m,dict(np.load(INITIAL,allow_pickle=False)))
    kin=mujoco.MjData(m);cube=m.geom('cube').id;palm=m.body('palm').id
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
    axis=d.xmat[palm].reshape(3,3)[:,2].copy()
    grip=np.array(json.loads(REFERENCE.read_bytes())['holding_goal_ticks_readonly'],dtype=int)
    goal=sim_to_ticks(d.ctrl);start=goal.copy();lo,hi,nlo,nhi=native(m)
    hi=np.maximum(hi,grip);rows=[];phases=[];trace=[];events=[];reason=None
    began=time.perf_counter()
    def observe(phase):
        current=d.geom_xmat[cube].reshape(3,3)
        change=Rotation.from_matrix(current@B0.T).as_rotvec()
        force={}
        for ci,c in enumerate(d.contact):
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a;f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f)
            role=(m.body(int(m.geom_bodyid[other])).name or '').split('_')[0]
            force[role]=force.get(role,0.)+float(f[0])
        return dict(phase=phase,time=float(d.time),yaw_degrees=float(np.degrees(change@axis)),
            center_shift_mm=float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000),
            tracking_rad=float(max(abs(d.ctrl-d.qpos[:16]))),group_force_n=force)
    def advance(target,phase,dwell=.060):
        nonlocal goal,reason
        target=np.asarray(target,dtype=int)
        if np.any(target<np.ceil(lo)) or np.any(target>np.floor(hi)):
            reason='native/scoped command range';return False
        begin=goal.copy();count=max(1,int(np.ceil(max(abs(target-begin))/2)))
        for k in range(1,count+1):
            candidate=np.rint(begin+(target-begin)*k/count).astype(int)
            paused=0.
            while max(abs(ticks_to_sim(candidate)-d.qpos[:16]))>.12:
                until=d.time+.075
                while d.time<until-1e-10:mujoco.mj_step(m,d)
                paused+=.075
                if paused>=2:
                    reason='tracking did not recover within2s';return False
            goal=candidate;d.ctrl[:]=ticks_to_sim(goal)
            until=d.time+dwell
            while d.time<until-1e-10:mujoco.mj_step(m,d)
            row=observe(phase);trace.append(row);rows.append(goal.copy());phases.append(phase)
            if row['center_shift_mm']>20 or row['tracking_rad']>.18 or not np.isfinite(d.qpos).all():
                reason='model retention/tracking diagnostic';return False
        return True
    def hold(phase,seconds=1.):
        for _ in range(int(np.ceil(seconds/.075))):
            if not advance(goal.copy(),phase,.075):return False
        events.append(observe(phase));return True
    def recontact(cycle):
        # Model contact forces guide research only; no N-to-mA conversion.
        for role,index,direction in [('if',6,1),('th',1,-1)]:
            for step in range(49):
                if observe('check')['group_force_n'].get(role,0.)>.2:break
                if step==48:
                    return False,'recontact did not establish '+role
                target=goal.copy();target[index]+=direction*2
                if not advance(target,f'cycle{cycle}_recontact_{role}'):
                    return False,'recontact stopped'
        return True,None
    def adjust_preload(phase):
        nonlocal reason
        from measured_grip_model_balance_20261117 import balance
        hand,_,_=inspect_for(m)
        for _ in range(2):
            control,certificate=balance(m,d,hand,cube,nlo,nhi)
            if control is None:
                events.append(dict(phase=phase,allocation=certificate))
                return True  # Diagnostic keeps current controls; never claims an allocation.
            target=sim_to_ticks(control)
            if not advance(target,phase):return False
            events.append(dict(phase=phase,allocation=certificate))
        return True
    def middle_support(cycle):
        nonlocal reason
        while goal[10]<grip[10]:
            if observe('check')['group_force_n'].get('mf',0.)>.35:return True
            target=goal.copy();target[10]=min(int(grip[10]),int(goal[10]+2))
            if not advance(target,f'cycle{cycle}_middle_support'):return False
        while goal[9]<grip[9]+args.middle_lateral:
            if observe('check')['group_force_n'].get('mf',0.)>.35:return True
            target=goal.copy();target[9]+=2
            if not advance(target,f'cycle{cycle}_middle_lateral'):return False
        if observe('check')['group_force_n'].get('mf',0.)>.15:return True
        reason='middle finger did not establish temporary support';return False
    def plan_reset():
        # Reset the thumb base while both contact points guide a small cube
        # translation. The integrated data d remains untouched by this solve.
        for key in ('qpos','qvel','ctrl','act','qacc_warmstart'):getattr(kin,key)[:]=getattr(d,key)
        mujoco.mj_forward(m,kin)
        C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
        palm_R=d.xmat[palm].reshape(3,3).copy()
        qbase=d.qpos[:16].copy();preload=d.ctrl-qbase
        active=np.r_[np.arange(4),np.arange(13,16)]
        lower=np.r_[nlo[active]+4*TICK-preload[active],[-5,-5,-2]]
        upper=np.r_[nhi[active]-4*TICK-preload[active],[5,5,2]]
        contacts=[]
        for prefix,preferred in [('if_','if_tip'),('th_','th_px_collision_1')]:
            choices=[]
            for ci,c in enumerate(d.contact):
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                g=b if a==cube else a;name=m.geom(g).name or ''
                if not (m.body(int(m.geom_bodyid[g])).name or '').startswith(prefix):continue
                f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f)
                if f[0]>.02:choices.append((name==preferred,float(f[0]),c,g))
            if not choices:return [],'reset contact unavailable'
            _,_,c,g=max(choices,key=lambda item:item[:2]);body=int(m.geom_bodyid[g]);G=d.xmat[body].reshape(3,3)
            normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
            contacts.append((body,G.T@(c.pos-d.xpos[body]),B.T@(c.pos-C),G.T@normal,normal))
        _,_,check=inspect_for(m)
        previous=np.r_[qbase[active],[0,0,0]];waypoints=[]
        count=max(1,int(np.ceil(abs(int(goal[3])-int(grip[3]))/8)))
        for root_goal in np.linspace(goal[3],grip[3],count+1)[1:]:
            fixed_root=(root_goal-2048)*TICK-preload[12]
            def measure(x):
                kin.qpos[:16]=qbase;kin.qpos[active]=x[:-3];kin.qpos[12]=fixed_root
                center=C+palm_R@(x[-3:]/1000)
                kin.qpos[-7:-4]=center
                mujoco.mj_fwdPosition(m,kin);pos=[];norm=[]
                for body,bpoint,cpoint,bnormal,normal in contacts:
                    G=kin.xmat[body].reshape(3,3)
                    pos.extend((kin.xpos[body]+G@bpoint-center-B@cpoint)*1000)
                    norm.extend((G@bnormal-normal)*3)
                sd,sp=check(kin)
                pen=max((max(0.,-c.dist*1000) for c in kin.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                return np.array(pos),np.array(norm),max(sd.values(),default=0.),max(sp.values(),default=0.),pen
            def cost(x):
                pos,norm,sd,sp,pen=measure(x)
                return np.r_[pos*3,norm,(x[:-3]-previous[:-3])*.15,x[-3:]*.2,
                             max(0,sd-.5)*30,max(0,sp-.2)*30,max(0,pen-.5)*30]
            fit=least_squares(cost,np.clip(previous,lower+1e-8,upper-1e-8),bounds=(lower,upper),
                              max_nfev=100,diff_step=1e-4,ftol=1e-7,xtol=1e-7,gtol=1e-7)
            pos,norm,sd,sp,pen=measure(fit.x)
            if max(np.linalg.norm(pos.reshape(-1,3),axis=1))>1.5 or sd>.6 or sp>.3 or pen>.7:
                return waypoints,'coordinated reset IK diagnostic'
            q=qbase.copy();q[active]=fit.x[:-3];q[12]=fixed_root
            waypoints.append(sim_to_ticks(q+preload));previous=fit.x.copy()
        return waypoints,None
    def plan_turn(degrees):
        # Only this independent IK data is assigned a desired cube pose.
        for key in ('qpos','qvel','ctrl','act','qacc_warmstart'):getattr(kin,key)[:]=getattr(d,key)
        mujoco.mj_forward(m,kin)
        C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
        qbase=d.qpos[:16].copy();preload=d.ctrl-qbase
        active=np.r_[np.arange(4),np.arange(12,16)]
        lower=nlo[active]+4*TICK-preload[active];upper=nhi[active]-4*TICK-preload[active]
        contacts=[]
        for prefix,preferred in [('if_','if_tip'),('th_','th_px_collision_1')]:
            candidates=[]
            for ci,c in enumerate(d.contact):
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                g=b if a==cube else a;name=m.geom(g).name or ''
                if not (m.body(int(m.geom_bodyid[g])).name or '').startswith(prefix):continue
                f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f)
                if f[0]>.02:candidates.append((name==preferred,float(f[0]),c,g))
            if not candidates:return None,'required turn contact unavailable'
            _,_,c,g=max(candidates,key=lambda item:item[:2]);body=int(m.geom_bodyid[g])
            G=d.xmat[body].reshape(3,3);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
            contacts.append((body,G.T@(c.pos-d.xpos[body]),B.T@(c.pos-C),G.T@normal,B.T@normal))
        _,_,check=inspect_for(m);previous=qbase[active].copy();waypoints=[]
        for angle in np.arange(1.,degrees+1e-6):
            wanted=Rotation.from_rotvec(axis*np.radians(angle)).as_matrix()@B
            quat=Rotation.from_matrix(wanted).as_quat()
            def measure(x):
                kin.qpos[:16]=qbase;kin.qpos[active]=x
                kin.qpos[-7:-4]=C;kin.qpos[-4:]=np.r_[quat[3],quat[:3]]
                mujoco.mj_fwdPosition(m,kin);pos=[];normal=[]
                for body,bpoint,cpoint,bnormal,cnormal in contacts:
                    G=kin.xmat[body].reshape(3,3)
                    pos.extend((kin.xpos[body]+G@bpoint-C-wanted@cpoint)*1000)
                    normal.extend((G@bnormal-wanted@cnormal)*3)
                sd,sp=check(kin)
                pen=max((max(0.,-c.dist*1000) for c in kin.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                return np.array(pos),np.array(normal),max(sd.values(),default=0.),max(sp.values(),default=0.),pen
            def cost(x):
                pos,norm,sd,sp,pen=measure(x)
                return np.r_[pos*3,norm,(x-previous)*.15,max(0,sd-.5)*30,max(0,sp-.2)*30,max(0,pen-.5)*30]
            fit=least_squares(cost,np.clip(previous,lower+1e-8,upper-1e-8),bounds=(lower,upper),
                              max_nfev=80,diff_step=1e-4,ftol=1e-7,xtol=1e-7,gtol=1e-7)
            pos,norm,sd,sp,pen=measure(fit.x)
            error=float(max(np.linalg.norm(pos.reshape(-1,3),axis=1)))
            if error>1.5 or sd>.6 or sp>.3 or pen>.7:
                return waypoints,dict(reason='turn IK diagnostic',angle=float(angle),point_error_mm=error,self_mm=float(sd),cube_mm=float(pen))
            q=qbase.copy();q[active]=fit.x;waypoints.append(sim_to_ticks(q+preload));previous=fit.x.copy()
        return waypoints,None
    target=goal.copy();target[10]-=args.middle_retreat
    if advance(target,'middle_clearance') and hold('initial_support'):
        for cycle in range(15):
            if time.perf_counter()-began>100:reason='bounded100second research budget';break
            before=observe('before_turn')['yaw_degrees']
            if before>=args.coarse_angle:break
            if args.balanced_preload and not adjust_preload(f'cycle{cycle+1}_preload'):break
            waypoints,failure=plan_turn(min(8.,np.ceil(args.coarse_angle-before)))
            if not waypoints:reason=str(failure);break
            for target in waypoints:
                if not advance(target,f'cycle{cycle+1}_turn'):break
            if reason or not hold(f'cycle{cycle+1}_turned'):break
            if args.middle_support_reset and not middle_support(cycle+1):break
            if args.coordinated_reset:
                reset,reset_error=plan_reset()
                if reset_error:
                    reason=reset_error;break
                for target in reset:
                    if not advance(target,f'cycle{cycle+1}_coordinated_reset'):break
                if reason:break
            else:
                old=goal.copy();target=goal.copy();target[1]=min(int(np.floor(hi[1]))-4,int(old[1]+96))
                if not advance(target,f'cycle{cycle+1}_thumb_release'):break
                target=goal.copy();target[3]=int(grip[3])
                if not advance(target,f'cycle{cycle+1}_thumb_root_reset'):break
                if not args.balanced_preload:
                    target=goal.copy();target[1]=int(old[1])
                    if not advance(target,f'cycle{cycle+1}_thumb_recontact'):break
            okay,contact_error=recontact(cycle+1)
            if not okay:
                reason=reason or contact_error;break
            if args.middle_support_reset:
                target=goal.copy();target[10]=int(grip[10]-args.middle_retreat);target[9]=int(grip[9])
                if not advance(target,f'cycle{cycle+1}_middle_release'):break
            if args.balanced_preload and not adjust_preload(f'cycle{cycle+1}_regrasp_preload'):break
            if not hold(f'cycle{cycle+1}_held'):break
            after=observe('after_regrasp')['yaw_degrees']
            events.append(dict(cycle=cycle+1,net_yaw_progress_degrees=after-before,ik_partial=failure))
            if after-before<.5:reason='turn did not accumulate after recontact';break
    if not reason:
        # The user-fixed grasp is the exact final command, not a newly fitted grip.
        if advance(grip,'fixed_grip_restore'):hold('fixed_grip_hold',3.)
    final=observe('final');desired=Rotation.from_rotvec(axis*np.pi/2).as_matrix()@B0
    final_error=float(np.degrees(np.linalg.norm(Rotation.from_matrix(d.geom_xmat[cube].reshape(3,3)@desired.T).as_rotvec())))
    out=BASE/(f'gait_middle{args.middle_retreat}_coarse{args.coarse_angle:g}_recontact'+
              ('_coordinated' if args.coordinated_reset else '')+
              ('_balanced' if args.balanced_preload else '')+
              ('_middle_support' if args.middle_support_reset else '')+
              (f'_middle_lateral{args.middle_lateral}' if args.middle_lateral else ''));out.mkdir(exist_ok=True)
    np.savez_compressed(out/'research_goals.npz',goals=np.array(rows),phases=np.array(phases),start_goal=start)
    np.savez_compressed(out/'end.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=np.array(d.time))
    report=dict(kind='measured_grip_replanned_turn_gait_model_diagnostic',events=events,trace=trace,final=final,
        stop_reason=reason,goal_count=len(rows),final_cube_orientation_error_degrees=final_error,
        exact_fixed_grip_command_reached=bool(np.array_equal(goal,grip)),
        final_trial_passed=not reason and np.array_equal(goal,grip) and final_error<=5,
        source_sha256=hashlib.sha256(INITIAL.read_bytes()).hexdigest(),model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        integration_cube_state_reset=False,physical_cube_pose_measured=False,physical_motion_executed=False,
        executable_candidate=False,elapsed_wall_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('trace','events')}))
    print('events',json.dumps(events))


if __name__=='__main__':main()
