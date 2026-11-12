"""Free closed-loop physics evaluation of a cloned correction component."""
import argparse
import time
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,TICK,BOUND,save_json,sha
from export_and_evaluate import Actor
from build_recovery_teacher_dataset import CASES
from probe_reference_policy_from_training_start_20261109 import observation_type
from saved_policy_observation_provider_20261109 import build_observation
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',default='recovery_bc_v1');p.add_argument('--case',choices=[r[0] for r in CASES],default='nominal')
    p.add_argument('--bounded-actions',action='store_true',help='Select only admissible incremental actions; the native guard remains unchanged')
    p.add_argument('--hold-after-acquired',action='store_true',help='After fine acquisition, hold the position command and monitor the original quarter-task envelope')
    p.add_argument('--acquire-samples',type=int,default=54)
    p.add_argument('--hold-no-reentry',action='store_true',help='Evaluate fixed position holding after acquisition; no object-feedback correction during hold')
    a=p.parse_args();suffix=('_bounded_actions' if a.bounded_actions else '')+('_latched_hold' if a.hold_after_acquired else '')
    if a.acquire_samples!=54:suffix+=f'_acq{a.acquire_samples}'
    if a.hold_no_reentry:suffix+='_fixed_position_hold'
    folder=OUT/'next_cycle'/a.run/('evaluation_'+a.case+suffix);folder.mkdir(parents=True,exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed evaluation')
    case,prefix,_,friction=next(r for r in CASES if r[0]==a.case)
    source=OUT/'sac_current_contract_v4_20k/export_000000010256'/prefix/'end_state.npz'
    z=dict(np.load(source,allow_pickle=False));m,_,_,_,provenance=assets('active');guard=Guard(m)
    if friction is not None:
        for gid in range(m.ngeom):
            if int(m.geom_bodyid[gid]) in guard.hand or gid==m.geom('cube').id:m.geom_friction[gid,0]=friction
    data=restore(m,z);cube=m.geom('cube').id;geometry=precompute(m,guard.hand)
    target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);quat=np.empty(4);mujoco.mju_mat2Quat(quat,target.ravel())
    station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    b=observation_type()()
    for _ in range(3):build_observation(b,m,data,data.ctrl,quat)
    actor_path=folder.parent/'actor.npz';policy=Actor(actor_path);held=0;goals=[];observations=[];actions=[];trace=[];stop=None
    certificate=SweepCertificate(m,guard.hand,.3);tail_n=lower_n=bad_n=0;min_gap=float('inf');hold_B=hold_C=None;drift=shift=0.;began=time.perf_counter()
    initial_B=data.geom_xmat[cube].reshape(3,3)
    error=orientation_error_deg(initial_B,target);tilt=face_up_error_deg(initial_B,5,z['up'])
    center=float(np.linalg.norm(data.geom_xpos[cube]-station)*1000)
    bounded_events=[]
    latched=False;acquisition_step=None;hold_reentries=0
    for i in range(256):
        obs=np.r_[build_observation(b,m,data,data.ctrl,quat),1.,min(held/54,1.)]
        action=np.zeros(16) if latched else policy.action(obs)
        previous=motor(data.ctrl);goal=motor(data.ctrl+2*TICK*action)
        if a.bounded_actions:
            desired=goal-previous;choices=np.arange(-2,3);candidates=previous[:,None]+choices[None,:]
            allowed=(candidates>=guard.lo[:,None])&(candidates<=guard.hi[:,None])
            if not np.all(allowed.any(1)):raise AssertionError('Source state has no legal incremental action')
            cost=np.where(allowed,abs(choices[None,:]-desired[:,None]),np.inf)
            chosen=choices[np.argmin(cost,axis=1)];goal=previous+chosen
            if np.any(chosen!=desired):bounded_events.append(dict(step=i+1,motor_ids=(np.flatnonzero(chosen!=desired)+1).tolist()))
        if np.max(np.abs(goal-previous))>2:raise AssertionError('Action contract exceeded')
        if np.any(goal<guard.lo) or np.any(goal>guard.hi):stop=['native_goal_range'];break
        old=data.ctrl.copy();data.ctrl[:]=ticks_to_sim(goal);goals.append(goal.copy());observations.append(obs);actions.append(action)
        for _ in range(75):
            mujoco.mj_step(m,data);stop=guard.check(data)
            if np.linalg.norm(data.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=202:
                B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube];force={}
                for ci,c in enumerate(data.contact):
                    x,y=int(c.geom1),int(c.geom2)
                    if cube not in (x,y):continue
                    name=m.geom(y if x==cube else x).name
                    if name in ('if_tip','th_tip') and -(c.pos-C)@B[:,1]*1000<55/6:
                        f=np.zeros(6);mujoco.mj_contactForce(m,data,ci,f);force[name]=force.get(name,0.)+float(f[0])
                tail_n+=1;lower_n+=all(force.get(n,0.)>.2 for n in ('if_tip','th_tip'))
                gap,bad=certificate.query(data,C,B);min_gap=min(min_gap,gap);bad_n+=bool(bad)
                if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                drift=max(drift,orientation_error_deg(B,hold_B));shift=max(shift,float(np.linalg.norm(C-hold_C)*1000))
            if stop:break
        b.set_last_act(np.clip((data.ctrl-old)/.2,-1,1));B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube]
        error=orientation_error_deg(B,target);center=float(np.linalg.norm(C-station)*1000)
        gap=min(v for _,v in gaps(m,data,C,B,geometry,True));tilt=face_up_error_deg(B,5,z['up'])
        qualifies=error<1.5 and tilt<5 and center<2 and gap>.3;held=held+1 if qualifies else 0
        if a.hold_after_acquired:
            if not latched and held>=a.acquire_samples:
                latched=True
                if acquisition_step is None:acquisition_step=i+1
            elif latched and not a.hold_no_reentry and (error>3 or center>1.8 or gap<.6):
                latched=False;held=0;hold_reentries+=1
        trace.append(dict(step=i+1,error_deg=error,center_error_mm=center,upper_gap_mm=gap,held=held,latched_hold=latched))
        if stop:break
    why=[]
    if stop:why.append('native_dynamic_guard')
    holding_tolerance=5. if a.hold_after_acquired else 1.5
    if error>=holding_tolerance:why.append('quarter_orientation')
    if a.hold_after_acquired and acquisition_step is None:why.append('fine_acquisition_not_reached')
    if tilt>=5:why.append('tilt')
    if center>=2:why.append('common_station')
    if tail_n!=4050:why.append('hold_incomplete')
    if lower_n/max(1,tail_n)<=.8:why.append('lower_contacts')
    if bad_n or min_gap<=.3:why.append('upper_space_certificate')
    if drift>=3 or shift>=2:why.append('hold_drift')
    np.savez_compressed(folder/'research_trace.npz',start=motor(z['ctrl']),goals=goals,observations=observations,actions=actions)
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
        up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    report=dict(status='cloned_correction_component_closed_loop_evaluation',case=case,actor_sha256=sha(actor_path),
        source_sha256=sha(source),hypothetical_friction_override=friction,model_provenance=provenance,
        steps=len(goals),stop=stop,worst=guard.worst,trace=trace,final=trace[-1] if trace else None,
        correction_and_hold_passed=not why,reasons=why,tail_1ms_samples=tail_n,
        lower_contact_fraction=lower_n/max(1,tail_n),minimum_certificate_gap_mm=min_gap if np.isfinite(min_gap) else None,
        certificate_bad_steps=bad_n,certificate=certificate.metadata(),hold_orientation_drift_deg=drift,
        hold_center_drift_mm=shift,no_joint_reference_path_in_student=True,
        full_initial_placement_replayed=False,second90_rotation_passed=False,
        is_SAC=False,is_whole_cycle_distillation=False,devices_opened=0,hardware_execution_allowed=False,
        admissible_incremental_action_selection=a.bounded_actions,bounded_action_events=bounded_events,
        fine_acquisition_tolerance_deg=1.5,final_quarter_task_tolerance_deg=holding_tolerance,
        position_hold_after_acquisition=a.hold_after_acquired,acquisition_step=acquisition_step,
        acquisition_qualifying_samples=a.acquire_samples,acquisition_qualifying_seconds=a.acquire_samples*.075,
        fixed_position_hold_without_object_feedback_reentry=a.hold_no_reentry,
        soft_monitor_reentry_count=hold_reentries,
        native_joint_limits_not_expanded=True,
        elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report);print(json.dumps({k:v for k,v in report.items() if k not in ('trace','model_provenance')}),flush=True)


if __name__=='__main__':main()
