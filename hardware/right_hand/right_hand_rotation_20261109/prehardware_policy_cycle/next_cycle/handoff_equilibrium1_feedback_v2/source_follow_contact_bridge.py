"""Privileged model-feedback teacher tracks a collision-planned contact handoff.

No pose pinning during execution. Each chosen goal is checked at every1ms.
The learned SAC actor is not used by this teacher, and no device API is present.
"""
import argparse
import time
import json
import numpy as np
import mujoco
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,SIM_TO_MOTOR_IDS,TICK,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg
from inspect_endpoints import inspect
from handoff_support_wrench import contact_wrench


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--steps',type=int,default=1400)
    parser.add_argument('--tag',default='rrt0_force_feedback');parser.add_argument('--posture-weight',type=float,default=.02)
    parser.add_argument('--resume-from',help='Completed teacher folder relative to the work directory; retain its physical state')
    parser.add_argument('--lookahead-cycles',type=int,choices=(1,2,3),default=1)
    parser.add_argument('--fixed-index-root',action='store_true',help='Retain the planned index-base support command')
    parser.add_argument('--force-mode',choices=('individual','wrench'),default='individual')
    parser.add_argument('--orientation-weight',type=float,default=5.)
    parser.add_argument('--center-weight',type=float,default=3.)
    parser.add_argument('--exploration-candidates',type=int,default=0)
    parser.add_argument('--source',default='connected_sac_and_teacher/first_quarter_end_state.npz')
    parser.add_argument('--reference-path',default='next_cycle/rrt_rf_seed0_retain_index/research_goals.npz')
    parser.add_argument('--jacobian-period',type=int,default=1,help='Reuse local search derivatives; every chosen command still gets full native1ms validation')
    args=parser.parse_args();folder=OUT/'next_cycle'/args.tag;folder.mkdir(parents=True,exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed teacher run')
    from pathlib import Path
    (folder/'source_follow_contact_bridge.py').write_bytes(Path(__file__).read_bytes())
    source=OUT/args.source
    path=OUT/args.reference_path
    reference=np.load(path,allow_pickle=False)['goals'][:-134]
    m,_,_,_,provenance=assets('active');z=dict(np.load(source,allow_pickle=False));d=restore(m,z)
    checker=Guard(m);cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
    contact_wrench(m,d,checker.hand,verify=True)
    sid=np.array(SIM_TO_MOTOR_IDS)-1;lag=z['ctrl']-z['qpos'][:16]
    progress=0;switched=False;ring_streak=0;held=good=0;rows=[];trace=[];stop=None
    best_progress=0;stale=0;hold_B=hold_C=None;hold_drift=hold_shift=0.;began=time.perf_counter()
    remainder=np.zeros(16);rejections={}
    cached_J=None
    random=np.random.default_rng(1209)
    hold_flags=[];hold_rotations=[];hold_centers=[];resume_state=None
    if args.resume_from:
        resume_folder=OUT/args.resume_from;previous_report=json.loads((resume_folder/'review.json').read_text())
        if previous_report['planned_path_sha256']!=sha(path):raise RuntimeError('Reference path changed')
        resume_state=resume_folder/'end_state.npz';d=restore(m,dict(np.load(resume_state,allow_pickle=False)))
        progress=int(previous_report['final']['progress']);best_progress=progress
    def forces(data):
        result={}
        for ci,c in enumerate(data.contact):
            x,y=int(c.geom1),int(c.geom2)
            if cube not in (x,y):continue
            name=m.geom(y if x==cube else x).name
            if name in ('if_tip','rf_tip','th_tip','if_bs_collision_1'):
                f=np.zeros(6);mujoco.mj_contactForce(m,data,ci,f);result[name]=result.get(name,0.)+float(f[0])
        return result
    def evidence(data,qref):
        B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube];force=forces(data)
        axis_error=Rotation.from_matrix(B0@B.T).as_rotvec()*180/np.pi
        holding=('rf_tip','th_tip') if switched else ('if_tip','th_tip')
        force_pen=np.array([15*max(0.,.35-force.get(n,0.)) for n in holding]+[8*max(0.,.5-force.get('if_bs_collision_1',0.))])
        # During approach, index remains loaded until the new support is measured.
        if progress>=len(reference)-45:force_pen=np.r_[force_pen,15*max(0.,.3-force.get('rf_tip',0.))]
        else:force_pen=np.r_[force_pen,0.]
        if args.force_mode=='wrench':
            wrench=contact_wrench(m,data,checker.hand)
            # These are search costs, not acceptance thresholds. During a
            # contact transition, all existing hand contacts may carry weight.
            # Final support, pose, and original dynamic guards stay unchanged.
            ow=5. if progress>=len(reference)-45 else args.orientation_weight
            cw=3. if progress>=len(reference)-45 else args.center_weight
            residual=np.r_[axis_error*ow,(C-C0)*1000*cw,
                           data.qvel[-6:-3]*1000*.15,data.qvel[-3:]*.5,
                           wrench['net_force_with_gravity_N']*10,wrench['torque_world_Nm']*200,
                           max(0.,.15-force.get('th_tip',0.))*10,
                           max(0.,.3-force.get('rf_tip',0.))*15 if progress>=len(reference)-45 else 0.,
                           (data.qpos[:16]-qref)/TICK*args.posture_weight]
        else:
            residual=np.r_[axis_error*args.orientation_weight,(C-C0)*1000*args.center_weight,force_pen,
                           (data.qpos[:16]-qref)/TICK*args.posture_weight]
        score=float(np.linalg.norm(residual))
        return residual,score,dict(orientation_error_deg=orientation_error_deg(B,B0),
            center_error_mm=float(np.linalg.norm(C-C0)*1000),forces_N_model_only=force,
            posture_error_max_ticks=float(np.max(np.abs(data.qpos[:16]-qref))/TICK))
    def predict(goal,qref):
        if args.fixed_index_root:goal[sid[0]]=motor(z['ctrl'])[sid[0]]
        if np.any(goal<checker.lo) or np.any(goal>checker.hi):return None
        state={key:getattr(d,key).copy() for key in BOUND};state['time']=d.time
        trial=restore(m,state);trial.ctrl[:]=ticks_to_sim(goal);guard=Guard(m)
        guard.gap=checker.gap;guard.longest_gap=checker.longest_gap
        for _ in range(75):
            mujoco.mj_step(m,trial);why=guard.check(trial)
            if np.linalg.norm(trial.qpos[-7:-4]-z['initial_cube_center'])>.08:why=list(why)+['cube_translation_over80mm']
            if why:
                for reason in why:rejections[reason]=rejections.get(reason,0)+1
                return None
        from types import SimpleNamespace
        first=SimpleNamespace(qpos=trial.qpos.copy());value=evidence(trial,qref)
        for _ in range(75*(args.lookahead_cycles-1)):
            mujoco.mj_step(m,trial);why=guard.check(trial)
            if np.linalg.norm(trial.qpos[-7:-4]-z['initial_cube_center'])>.08:why=list(why)+['cube_translation_over80mm']
            if why:
                for reason in why:rejections['future_hold_'+reason]=rejections.get('future_hold_'+reason,0)+1
                return None
        return first,value
    for step in range(args.steps):
        values=forces(d);ring_streak=ring_streak+1 if values.get('rf_tip',0.)>.25 else 0
        if ring_streak>=10:switched=True
        if not switched and values.get('rf_tip',0.)<.1:ring_streak=0
        stop_index=min(len(reference),progress+60)
        qrefs=np.stack([ticks_to_sim(g)-lag for g in reference[progress:stop_index]])
        near=np.flatnonzero(np.max(np.abs(qrefs-d.qpos[:16]),axis=1)/TICK<35)
        if len(near):progress+=int(near[-1])
        qref=ticks_to_sim(reference[progress])-lag;previous=motor(d.ctrl)
        baseline=predict(previous,qref)
        # A hold command can be unsafe even though a bounded corrective command
        # is safe. Inspect the alternatives before declaring the state blocked.
        residual0=baseline[1][0] if baseline is not None else evidence(d,qref)[0]
        candidates=[(previous,baseline)] if baseline is not None else []
        if cached_J is None or step%max(1,args.jacobian_period)==0 or baseline is None:
            J=np.zeros((len(residual0),16))
            for joint in range(16):
                ends=[]
                for sign in (-1,1):
                    goal=previous.copy();goal[sid[joint]]+=sign*2;predicted=predict(goal,qref)
                    if predicted is not None:candidates.append((goal,predicted));ends.append(predicted[1][0])
                    else:ends.append(None)
                if all(x is not None for x in ends):J[:,joint]=(ends[1]-ends[0])/4
                elif ends[0] is not None:J[:,joint]=(residual0-ends[0])/2
                elif ends[1] is not None:J[:,joint]=(ends[1]-residual0)/2
            cached_J=J.copy()
        else:J=cached_J.copy()
        matrix=np.vstack([J,np.eye(16)*.015]);rhs=np.r_[-.5*residual0,np.zeros(16)]
        fit=lsq_linear(matrix,rhs,bounds=(-2,2),max_iter=100)
        intent=fit.x+remainder;delta=np.clip(np.rint(intent).astype(int),-2,2);remainder=np.clip(intent-delta,-.49999,.49999)
        for scale in (1.,.5):
            goal=previous.copy();goal[sid]+=np.rint(delta*scale).astype(int);predicted=predict(goal,qref)
            if predicted is not None:candidates.append((goal,predicted))
        # Coupled bounded moves can be feasible when every single-joint probe
        # is rejected. Also consider braking, without weakening any guard.
        proposals=[motor(qref),motor(d.qpos[:16])]
        for desired in proposals:
            goal=previous+np.clip(desired-previous,-2,2);predicted=predict(goal,qref)
            if predicted is not None:candidates.append((goal,predicted))
        if args.exploration_candidates and stale>=20 and step%5==0:
            toward=np.clip(motor(qref)-previous,-2,2)
            for _ in range(args.exploration_candidates):
                noise=random.integers(-2,3,16);delta=np.where(random.random(16)<.65,toward,noise)
                goal=previous+delta;predicted=predict(goal,qref)
                if predicted is not None:candidates.append((goal,predicted))
        if not candidates:
            for _ in range(96):
                goal=previous+random.choice(np.array([-2,0,2]),size=16);predicted=predict(goal,qref)
                if predicted is not None:candidates.append((goal,predicted))
        if not candidates:stop='no_safe_two_tick_correction_found';break
        goal,predicted=min(candidates,key=lambda item:item[1][1][1]);d.ctrl[:]=ticks_to_sim(goal);rows.append(goal.copy())
        for _ in range(75):
            mujoco.mj_step(m,d);why=checker.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-z['initial_cube_center'])>.08:why=list(why)+['cube_translation_over80mm']
            if why:stop=why;break
            if progress>=len(reference)-1:
                B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube];f=forces(d)
                if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                held+=1;good+=all(f.get(n,0.)>.2 for n in ('rf_tip','th_tip','if_bs_collision_1'))
                hold_flags.append(all(f.get(n,0.)>.2 for n in ('rf_tip','th_tip','if_bs_collision_1')))
                hold_rotations.append(B.copy());hold_centers.append(C.copy())
                hold_drift=max(hold_drift,orientation_error_deg(B,hold_B));hold_shift=max(hold_shift,float(np.linalg.norm(C-hold_C)*1000))
        _,score,actual=evidence(d,qref);actual.update(step=step+1,progress=progress,planned_moving_goals=len(reference),
            ring_support_selected=switched,prediction_qpos_error=float(np.max(np.abs(d.qpos-predicted[0].qpos))))
        trace.append(actual)
        if actual['prediction_qpos_error']>1e-8:stop='planning_execution_state_mismatch'
        stale=0 if progress>best_progress else stale+1;best_progress=max(best_progress,progress)
        if (step+1)%20==0 or stop:print(json.dumps(actual),flush=True)
        if stop or held>=10050:break
        if (folder/'STOP_OFFLINE_SEARCH').exists():stop='offline_search_stop_requested';break
        if stale>=140:stop='reference_progress_stalled';break
    endpoint=inspect(m,d,z['up'],z['initial_cube_rotation']);final=trace[-1] if trace else None
    if held>=4050:
        matrices=np.asarray(hold_rotations[-4050:]);centers=np.asarray(hold_centers[-4050:])
        products=np.einsum('nij,ij->n',matrices,matrices[0]);hold_drift=float(np.degrees(np.arccos(np.clip((products-1)/2,-1,1))).max())
        hold_shift=float(np.linalg.norm(centers-centers[0],axis=1).max()*1000)
        good=sum(hold_flags[-4050:]);tail_n=4050
    else:tail_n=held
    passed=not stop and tail_n==4050 and good/4050>.8 and final['orientation_error_deg']<5 and final['center_error_mm']<2 and hold_drift<3 and hold_shift<2
    start_goal=motor(np.load(resume_state,allow_pickle=False)['ctrl']) if resume_state is not None else motor(z['ctrl'])
    np.savez_compressed(folder/'research_goals.npz',start=start_goal,goals=rows)
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,
                        up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    report=dict(status='force_feedback_teacher_for_cooperative_contact_bridge',source_sha256=sha(source),
        planned_path_sha256=sha(path),model_provenance=provenance,steps=len(rows),stop=stop,trace=trace,
        final=final,endpoint=endpoint,worst=checker.worst,prediction_rejections=rejections,
        final_hold_samples=tail_n,final_support_fraction=good/max(1,tail_n),hold_orientation_drift_deg=hold_drift,
        hold_center_drift_mm=hold_shift,temporary_third_support_passed=passed,second90_rotation_passed=False,
        native_physics_limits_and_max2ticks75ms_retained=True,cube_never_reset_during_teacher=True,
        posture_weight=args.posture_weight,resume_state=None if resume_state is None else str(resume_state.relative_to(OUT)),
        fixed_index_root=args.fixed_index_root,
        conservative_hold_lookahead_cycles=args.lookahead_cycles,
        force_mode=args.force_mode,orientation_search_weight=args.orientation_weight,
        center_search_weight=args.center_weight,exploration_candidates=args.exploration_candidates,
        source_state_path=str(source.relative_to(OUT)),reference_path=str(path.relative_to(OUT)),
        final_support_pose_and_dynamic_acceptance_unchanged=True,
        local_search_jacobian_period=args.jacobian_period,
        resume_state_sha256=None if resume_state is None else sha(resume_state),
        is_SAC=False,devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report);print(json.dumps({key:value for key,value in report.items() if key not in ('trace','endpoint','model_provenance')}),flush=True)


if __name__=='__main__':main()
