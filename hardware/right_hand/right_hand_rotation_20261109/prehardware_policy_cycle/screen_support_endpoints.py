"""Test proposed RF contact endpoints in free physics, not reached trajectories.

Each test is initialized once at a saved IK proposal. It cannot establish that
the robot can reach the pose from its first-rotation grip.
"""
import time
import json
from pathlib import Path
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,save_json,sha
from handoff_support_wrench import contact_wrench
from cube_task_frame_20261105 import orientation_error_deg


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/endpoint_screen';folder.mkdir(parents=True,exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed endpoint screen')
    (folder/'source_screen_support_endpoints.py').write_bytes(Path(__file__).read_bytes())
    m,_,_,_,provenance=assets('active');original=dict(np.load(OUT/'connected_sac_and_teacher/first_quarter_end_state.npz',allow_pickle=False))
    cube=m.geom('cube').id;guard=Guard(m);rows=[];began=time.perf_counter()
    preloads=(0.,.3,.7)
    for seed in (0,1,2):
        path=OUT/f'next_cycle/clearance_rf_keep_index_contact/proposal{seed}.npz';z=dict(np.load(path,allow_pickle=False))
        initial={key:original[key].copy() for key in BOUND};initial['qpos']=z['qpos'].copy();initial['ctrl']=z['ctrl'].copy()
        initial['qvel'][:]=0;initial['qacc_warmstart'][:]=0;initial['time']=0.
        d=restore(m,initial);C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();directions={}
        for prefix,indices in (('rf',np.arange(8,12)),('if',np.arange(4)),('th',np.arange(12,16))):
            tip=m.geom(prefix+'_tip').id
            contact=next(c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip})
            normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1)
            J=np.zeros((3,m.nv));mujoco.mj_jac(m,d,J,None,contact.pos,int(m.geom_bodyid[tip]))
            delta=-J[:,:16].T@normal/m.actuator_gainprm[:,0];mask=np.zeros(16);mask[indices]=1;directions[prefix]=delta*mask
        for rf in preloads:
            for index in preloads:
                for thumb in preloads:
                    goal=motor(z['ctrl']+rf*directions['rf']+index*directions['if']+thumb*directions['th'])
                    if np.any(goal<guard.lo) or np.any(goal>guard.hi):continue
                    data=restore(m,initial);checker=Guard(m);data.ctrl[:]=ticks_to_sim(goal);stop=None
                    for step in range(750):
                        mujoco.mj_step(m,data);stop=checker.check(data)
                        if np.linalg.norm(data.qpos[-7:-4]-initial['qpos'][-7:-4])>.08:stop=list(stop)+['cube_translation_over80mm']
                        if stop:break
                    w=contact_wrench(m,data,checker.hand);force={name:r['normal_N'] for name,r in w['by_geom'].items()}
                    angle=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),B0)
                    center=float(np.linalg.norm(data.geom_xpos[cube]-C0)*1000)
                    viable=not stop and angle<5 and center<2 and all(force.get(n,0.)>.2 for n in ('rf_tip','th_tip','if_bs_collision_1'))
                    rows.append(dict(seed=seed,preload_N_model_only=[rf,index,thumb],source_sha256=sha(path),
                        goal_ticks=goal.tolist(),steps_1ms=step+1,stop=stop,orientation_error_deg=angle,center_error_mm=center,
                        forces_N_model_only=force,upward_support_N=w['upward_support_N'],weight_N=w['weight_N'],
                        worst=checker.worst,viable_screen=bool(viable)))
        print(json.dumps(dict(seed=seed,trials=len(rows),screen_passes=sum(r['viable_screen'] for r in rows))),flush=True)
    qualified=[]
    for candidate in sorted((r for r in rows if r['viable_screen']),key=lambda r:r['orientation_error_deg']+r['center_error_mm'])[:4]:
        seed=candidate['seed'];path=OUT/f'next_cycle/clearance_rf_keep_index_contact/proposal{seed}.npz'
        z=dict(np.load(path,allow_pickle=False));initial={key:original[key].copy() for key in BOUND}
        initial['qpos']=z['qpos'];initial['ctrl']=z['ctrl'];initial['qvel'][:]=0;initial['qacc_warmstart'][:]=0;initial['time']=0.
        data=restore(m,initial);checker=Guard(m);C0=data.geom_xpos[cube].copy();B0=data.geom_xmat[cube].reshape(3,3).copy()
        data.ctrl[:]=ticks_to_sim(np.array(candidate['goal_ticks']));stop=None;tail=good=0;first_B=first_C=None;drift=shift=0.
        for step in range(10050):
            mujoco.mj_step(m,data);stop=checker.check(data)
            if np.linalg.norm(data.qpos[-7:-4]-initial['qpos'][-7:-4])>.08:stop=list(stop)+['cube_translation_over80mm']
            if step>=6000:
                tail+=1;w=contact_wrench(m,data,checker.hand)
                good+=all(w['by_geom'].get(n,{}).get('normal_N',0.)>.2 for n in ('rf_tip','th_tip','if_bs_collision_1'))
                B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube]
                if first_B is None:first_B=B.copy();first_C=C.copy()
                drift=max(drift,orientation_error_deg(B,first_B));shift=max(shift,float(np.linalg.norm(C-first_C)*1000))
            if stop:break
        angle=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),B0);center=float(np.linalg.norm(data.geom_xpos[cube]-C0)*1000)
        passed=not stop and tail==4050 and good/4050>.8 and angle<5 and center<2 and drift<3 and shift<2
        qfolder=folder/f'qualified{len(qualified)}';qfolder.mkdir(exist_ok=True)
        np.savez_compressed(qfolder/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
            up=original['up'],initial_cube_rotation=original['initial_cube_rotation'],initial_cube_center=original['initial_cube_center'])
        qualified.append(dict(candidate=candidate,passed=bool(passed),stop=stop,tail_1ms_samples=tail,
            support_fraction=good/max(1,tail),orientation_error_deg=angle,center_error_mm=center,
            hold_orientation_drift_deg=drift,hold_center_drift_mm=shift,end_state=str((qfolder/'end_state.npz').relative_to(OUT))))
    report=dict(status='free_model_RF_support_endpoint_tests',model_provenance=provenance,screen_trials=rows,
        qualified_endpoints=qualified,initial_IK_proposals_not_reached_by_robot_motion=True,
        source_first_rotation_to_proposal_motion_verified=False,maximum_goal_spacing_not_tested_between_different_initializations=True,
        source_cube_dynamics_free_after_each_proposal_initialization=True,
        native_joint_collision_tracking_acceptance_unchanged=True,second90_rotation_passed=False,
        hardware_execution_allowed=False,devices_opened=0,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report)
    print(json.dumps(dict(trials=len(rows),screen_passes=sum(r['viable_screen'] for r in rows),
        qualified_endpoints=qualified,elapsed_seconds=report['elapsed_seconds'])),flush=True)


if __name__=='__main__':main()
