"""Supported middle-finger repositioning after the saved small turn.

Thumb motion redistributes pressure to the index before the middle releases.
The middle then moves inward/lower, recloses, and the thumb returns. Planning
uses perfect model observations; no robot transport, injected forces or live
cube-pose assignments. Full prefix+suffix replay independently checks hold.
"""
from pathlib import Path
import argparse,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from audit_opposed_working_grip_20261117 import MODEL,ROLES,replay,sha
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from canonical_grip_dynamics_20261105 import run
from research_quasistatic_wrench_path_20261105 import allocate_native
from opposed_grip_force_balance_20261116 import allocate
from opposed_force_slide_control_20261117 import increment
from probe_opposed_grip_turn_20261117 import FingerClearPlanner
from closest_convex_surface_20261117 import prepare,closest
from upper_layer_cylinder_clearance_20261105 import precompute
from upper_layer_separation_certificate_20261105 import SweepCertificate
from hand_motor_mapping import ticks_to_sim

BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'


def observe(m,d,cube):
    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
    loads=dict.fromkeys(ROLES,0.);best={};strength={}
    for ci,c in enumerate(d.contact):
        a,b=int(c.geom1),int(c.geom2)
        if cube not in (a,b):continue
        g=b if a==cube else a;name=m.geom(g).name
        if name not in loads or c.dist>=.0004:continue
        n=(c.frame[:3]*(1 if a==cube else -1))@B
        wanted=n[2]>.5 if name=='th_tip' else n[2]<-.5
        if not wanted or (c.pos-C)@(-B[:,1])>=55/6000:continue
        ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff);f=float(ff[0]);loads[name]+=f
        if f>strength.get(name,-1):best[name]=(g,c);strength[name]=f
    return loads,best


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--equilibrium-weight',type=float,choices=(0.,20.),default=0.)
    args=ap.parse_args()
    out=BASE/('middle_gait_v1'+('_eq20' if args.equilibrium_weight else ''));out.mkdir(exist_ok=False)
    (out/'source_script.py').write_bytes(Path(__file__).read_bytes())
    support_hashes={}
    for name in ('opposed_force_slide_control_20261117.py','opposed_grip_force_balance_20261116.py',
                 'audit_opposed_working_grip_20261117.py','closest_convex_surface_20261117.py'):
        support_hashes[name]=sha(ROOT/name);(out/('support_'+name)).write_bytes((ROOT/name).read_bytes())
    prior=json.loads((BASE/'core_support_chain_10s/review.json').read_bytes())
    original=ROOT/prior['source_path'];old=np.load(BASE/'core_support_chain_10s/research_goals.npz',allow_pickle=False)
    prefix=old['goals'][:prior['hold_start']]
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    if sha(MODEL)!=prior['model_sha256'] or sha(original)!=prior['source_sha256']:raise RuntimeError('Source changed')
    z0=dict(np.load(original,allow_pickle=False));began=time.perf_counter()
    prefix_result=run(m,z0,prefix,1,out/'start_state.npz',None,False,5,('mf_tip','th_tip'))
    if not prefix_result['safety_passed']:raise RuntimeError('Saved prefix failed')
    z=dict(np.load(out/'start_state.npz',allow_pickle=False));d=restore(m,z)
    hand,support,check=inspect_for(m);lo,hi,nlo,nhi=native(m);cube=m.geom('cube').id;body=m.body('cube').id
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
    loads,live=observe(m,d,cube);targets={};anchors={};normals={}
    for name in ROLES:
        g,c=live[name];G=d.geom_xmat[g].reshape(3,3)
        targets[name]=B0.T@(c.pos-C0);anchors[name]=G.T@(c.pos-d.geom_xpos[g])
        normals[name]=np.array([0.,0.,1. if name=='th_tip' else -1.])
    initial={name:value.copy() for name,value in targets.items()}
    middle_goal=initial['mf_tip'].copy();middle_goal[0]-=.003;middle_goal[1]+=.001
    thumb_start_x=float(initial['th_tip'][0]);surfaces={g:prepare(v,e) for g,v,e in precompute(m,hand)}
    start=motor(d.ctrl);planner=FingerClearPlanner(start,np.minimum(lo+2,start),np.maximum(hi-2,start),m,d)
    planner.pairs=[(m.geom(g).name,'th_tip') for g in range(m.ngeom) if (m.geom(g).name or '').startswith('palm_collision_')]
    planner.pairs.append(('if_md_collision_5','mf_px_collision'))
    cert=SweepCertificate(m,hand,.3);properties={k:getattr(m,k).copy() for k in
        ('actuator_gainprm','actuator_biasprm','actuator_forcerange','jnt_range','geom_size','geom_contype','geom_conaffinity')}
    phase='thumb_out';phase_start=0;stable=held=0;hold_start=None;rows=[];phases=[];trace=[];poses=[];stop=None;loss=0.
    last_lp=None;last_geometry={};gate_queries=0;budgets=dict(thumb_out=240,open_middle=80,move_middle=120,
        close_middle=100,thumb_return=240,hold=400)
    for index in range(1180):
        elapsed=index-phase_start;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
        loads,live=observe(m,d,cube)
        xif=float((B.T@(live['if_tip'][1].pos-C))[0]) if 'if_tip' in live else float(initial['if_tip'][0])
        xmf=float((B.T@(live['mf_tip'][1].pos-C))[0]) if 'mf_tip' in live else float(middle_goal[0] if phase=='thumb_return' else initial['mf_tip'][0])
        xth=float((B.T@(live['th_tip'][1].pos-C))[0]) if 'th_tip' in live else float(targets['th_tip'][0])
        fraction=float(np.clip(elapsed/200,0,1));blend=fraction*fraction*(3-2*fraction)
        if phase=='thumb_out':targets['th_tip'][0]=thumb_start_x+(-.005-thumb_start_x)*blend
        if phase=='open_middle':targets['mf_tip'][2]=-.0283
        if phase=='move_middle':
            f=float(np.clip(elapsed/80,0,1));targets['mf_tip'][:2]=initial['mf_tip'][:2]+f*(middle_goal[:2]-initial['mf_tip'][:2]);targets['mf_tip'][2]=-.0283
        if phase=='close_middle':targets['mf_tip']=middle_goal.copy();targets['mf_tip'][2]=-.0275
        if phase=='thumb_return':targets['th_tip'][0]=-.005+(thumb_start_x+.005)*blend
        if phase in ('thumb_out','thumb_return'):
            share=float(np.clip((xth-xif)/max(xmf-xif,.005),.05,.95))
            refs={'if_tip':1-share,'mf_tip':share,'th_tip':1.}
        elif phase in ('open_middle','move_middle'):
            refs={'if_tip':.6,'mf_tip':0.,'th_tip':.6}
        elif phase=='close_middle':refs={'if_tip':.6,'mf_tip':.3,'th_tip':.9}
        else:refs={'if_tip':.15,'mf_tip':.85,'th_tip':1.}
        velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,body,velocity,0)
        F=-m.body_mass[body]*m.opt.gravity+30*(C0-C)-1.5*velocity[3:]
        T=.04*Rotation.from_matrix(B0@B.T).as_rotvec()-.002*velocity[:3]
        control,allocation=allocate_native(m,d,cube,C,F,T,refs.copy(),hand,True,(nlo,nhi),ROLES,normal_reference_weight=.2)
        if control is None:stop=dict(reason='force_allocation',details=allocation,goal=index+1);break
        tasks=[]
        for name in ROLES:
            g=m.geom(name).id;G=d.geom_xmat[g].reshape(3,3);origin=d.geom_xpos[g];target=C+B@targets[name]
            moving=(name=='th_tip' and phase in ('thumb_out','thumb_return')) or (name=='mf_tip' and phase in ('open_middle','move_middle','close_middle'))
            point=origin+G@closest(G.T@(target-origin),surfaces[g]) if moving else origin+G@anchors[name]
            error=target-point;length=np.linalg.norm(error)
            if length>.002:error*=.002/length
            tasks.append(dict(geom_id=g,point=point.tolist(),outward_normal=(B@normals[name]).tolist(),recover_tip_displacement=error.tolist()))
        try:
            goal=increment(m,d,allocation,tasks,planner,C,C0,B,B0,refs,loads,skip_index_force=False,
                ignored_force_names=('mf_tip',) if phase in ('open_middle','move_middle') else (),
                equilibrium_control=control,equilibrium_weight=args.equilibrium_weight)
        except ValueError as exc:
            if 'projection_infeasible' not in str(exc):raise
            stop=dict(reason=str(exc),goal=index+1);break
        rows.append(goal.copy());phases.append(phase);d.ctrl[:]=ticks_to_sim(goal)
        for sub in range(75):
            mujoco.mj_step(m,d);su,sp=check(d)
            values=dict(self_mm=max(sp.values(),default=0.),support_mm=max(su.values(),default=0.),cube_mm=0.,
                tracking_rad=float(max(abs(d.qpos[:16]-d.ctrl))),range_rad=float(max(0.,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
            inhand=external=False
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    inhand|=c.dist<.0004;values['cube_mm']=max(values['cube_mm'],max(0.,-float(c.dist)*1000))
                else:external|=other in support and c.dist<.0004
            loss=0. if inhand else loss+.001
            failed=[k for k,v in [('self_mm',1e-6),('support_mm',.15),('cube_mm',1.),('tracking_rad',.18),('range_rad',1e-6)] if values[k]>v]
            if external:failed.append('external_cube_support')
            if loss>.25:failed.append('hand_contact_loss')
            if np.linalg.norm(d.qpos[-7:-4]-C0)>.08:failed.append('translation_bound')
            if not np.isfinite(np.r_[d.qpos,d.qvel]).all():failed.append('nonfinite_state')
            if phase=='hold':
                _,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3))
                if bad:failed.append('upper_sweep_clearance')
            if failed:stop=dict(reason='original_guard_or_hold_space',goal=index+1,substep=sub+1,failed=failed,values=values);break
        C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();loads,live=observe(m,d,cube)
        angle=float(np.degrees(np.linalg.norm(Rotation.from_matrix(B@B0.T).as_rotvec())));shift=float(np.linalg.norm(C-C0)*1000)
        next_phase=None
        if phase=='thumb_out' and index%10==0 and 'th_tip' in live and 'if_tip' in live:
            actual_x=float((B.T@(live['th_tip'][1].pos-C))[0])
            if actual_x<0 and loads['if_tip']>.3 and loads['th_tip']>.3:
                _,last_lp=allocate(m,d,cube,hand,nlo,nhi,lo,hi,tip_minimum=.3,
                    required_tips=('if_tip','th_tip'),normal_bounds={'mf_tip':(0.,.1)})
                gate_queries+=1
                if last_lp.get('normal_N') is not None:next_phase='open_middle'
        elif phase in ('open_middle','move_middle'):
            g=m.geom('mf_tip').id;p=np.zeros(6);distance=float(mujoco.mj_geomDistance(m,d,cube,g,.1,p));clocal=B.T@(p[3:]-C)
            last_geometry=dict(gap_mm=distance*1000,cube_local_mm=(clocal*1000).tolist())
            ready=loads['if_tip']>.2 and loads['th_tip']>.2 and loads['mf_tip']<.15 and distance>.0004
            if phase=='move_middle':ready &= elapsed>=80 and np.linalg.norm(clocal[:2]-middle_goal[:2])<.001
            stable=stable+1 if ready else 0
            if stable>=5:next_phase='move_middle' if phase=='open_middle' else 'close_middle'
        elif phase=='close_middle':
            ready=loads['mf_tip']>.2 and loads['th_tip']>.3 and 'mf_tip' in live
            if ready:ready &= abs(float((B.T@(live['mf_tip'][1].pos-C))[0]))<.0255
            stable=stable+1 if ready else 0
            if stable>=10:next_phase='thumb_return'
        elif phase=='thumb_return':
            gap,bad=cert.query(d,C,B)
            ready=elapsed>=200 and loads['mf_tip']>.2 and loads['th_tip']>.2 and angle<5 and shift<2 and not bad
            if ready and 'mf_tip' in live and 'th_tip' in live:
                ready &= abs(float((B.T@(live['mf_tip'][1].pos-C))[0]))<.0255 and abs(float((B.T@(live['th_tip'][1].pos-C))[0])-thumb_start_x)<.0015
            stable=stable+1 if ready else 0
            if stable>=10:next_phase='hold';hold_start=len(rows)
        elif phase=='hold':held+=1
        if index%40==0 or stop or next_phase:
            trace.append(dict(goal=index+1,phase=phase,next_phase=next_phase,lower_N=loads,
                angle_error_deg=angle,center_error_mm=shift,thumb_x_mm=float((B.T@(live['th_tip'][1].pos-C))[0]*1000) if 'th_tip' in live else None))
            print(json.dumps(trace[-1]),flush=True)
        if index%2==0 or stop:poses.append(d.qpos.copy())
        if stop or held>=400:break
        if angle>15 or shift>15:stop=dict(reason='bounded_task_pose_divergence',angle_deg=angle,shift_mm=shift);break
        if next_phase:
            phase=next_phase;phase_start=index+1;stable=0
            for name,(g,c) in live.items():
                if phase in ('thumb_return','hold'):
                    targets[name]=B.T@(c.pos-C);anchors[name]=d.geom_xmat[g].reshape(3,3).T@(c.pos-d.geom_xpos[g])
            if phase=='thumb_return':targets['th_tip'][0]=-.005
        elif elapsed>=budgets[phase]-1:stop=dict(reason='bounded_stage_not_reached',phase=phase);break
    for k,v in properties.items():
        if not np.array_equal(getattr(m,k),v):raise RuntimeError('Model property changed')
    np.savez_compressed(out/'suffix.npz',start=start,goals=rows,phases=phases)
    np.savez_compressed(out/'trajectory.npz',qpos=poses)
    np.savez_compressed(out/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,
        time=d.time,up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    audit=None
    if stop is None and held==400:
        full=np.vstack([prefix,rows]);audit=replay(m,z0,full,len(prefix)+hold_start,out,C0,B0,required_roles=('mf_tip','th_tip'))
        np.savez_compressed(out/'research_goals.npz',start=old['start'],goals=full,hold_start=len(prefix)+hold_start)
    report=dict(status='supported_middle_contact_gait',passed=bool(audit and audit['passed']),audit=audit,
        equilibrium_weight=args.equilibrium_weight,
        stop=stop,phase=phase,goals=len(rows),held_goals=held,prefix_goals=len(prefix),source_path=original.relative_to(ROOT).as_posix(),
        source_sha256=sha(original),model_sha256=sha(MODEL),script_sha256=sha(Path(__file__)),support_source_hashes=support_hashes,
        initial_contacts_cube_local_m={k:v.tolist() for k,v in initial.items()},planned_middle_contact_m=middle_goal.tolist(),
        live_transfer_feasibility=last_lp,gate_queries=gate_queries,middle_geometry_at_exit=last_geometry,trace=trace,
        cube_or_hand_reset_between_segments=False,model_properties_unchanged=True,perfect_simulated_feedback=True,
        new_SAC_training=False,full_quarter_turn_cycle_verified=False,physical_grasp_verified=False,
        devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('trace','audit','initial_contacts_cube_local_m')},ensure_ascii=True),flush=True)


if __name__=='__main__':main()
