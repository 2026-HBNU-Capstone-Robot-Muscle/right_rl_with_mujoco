"""Reconstruct sensor-contract observations for three validated correction suffixes."""
import json
import argparse
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,ticks_to_sim,TICK,save_json,sha
from probe_reference_policy_from_training_start_20261109 import observation_type
from saved_policy_observation_provider_20261109 import build_observation
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate

CASES=[('nominal','evaluation_active_certified','next_model_feedback_5_all_fingers',None),
       ('x_plus05','evaluation_active_x_plus05','next_model_feedback_5_recover_x_plus05',None),
       ('friction025','evaluation_active_friction025','next_model_feedback_5_recover_friction025',.25)]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--tag',default='recovery_teacher_data')
    parser.add_argument('--hold-steps',type=int,default=0)
    parser.add_argument('--long-feedback-teachers',action='store_true')
    parser.add_argument('--nominal-teacher',help='Explicit completed nominal teacher folder; never silently replace a failed teacher')
    args=parser.parse_args()
    folder=OUT/'next_cycle'/args.tag;folder.mkdir(parents=True,exist_ok=True)
    if (folder/'manifest.json').exists():raise RuntimeError('Preserve completed data generation')
    X=[];Y=[];C=[];S=[];sources=[];parent=OUT/'sac_current_contract_v4_20k/export_000000010256'
    station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    for index,(case,prefix,teacher,friction) in enumerate(CASES):
        if args.long_feedback_teachers:teacher='next_model_feedback_5_recover_long_'+case
        if case=='nominal' and args.nominal_teacher:teacher=args.nominal_teacher
        teacher_review=json.loads((OUT/teacher/'review.json').read_text())
        if not teacher_review['success']:raise RuntimeError('A failed teacher is not valid imitation data')
        m,_,_,_,provenance=assets('active');source=parent/prefix/'end_state.npz';z=dict(np.load(source,allow_pickle=False));guard=Guard(m)
        if friction is not None:
            for gid in range(m.ngeom):
                if int(m.geom_bodyid[gid]) in guard.hand or gid==m.geom('cube').id:m.geom_friction[gid,0]=friction
        d=restore(m,z);geometry=precompute(m,guard.hand);cube=m.geom('cube').id
        target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);quat=np.empty(4);mujoco.mju_mat2Quat(quat,target.ravel())
        b=observation_type()()
        for _ in range(3):build_observation(b,m,d,d.ctrl,quat)
        goals_path=OUT/teacher/'research_goals.npz';goals=np.load(goals_path,allow_pickle=False)['goals'];held=0
        for step,goal in enumerate(goals):
            obs=build_observation(b,m,d,d.ctrl,quat);action=(ticks_to_sim(goal)-d.ctrl)/(2*TICK)
            if np.max(np.abs(action))>1+1e-9:raise AssertionError('Teacher has a goal increment exceeding2ticks')
            X.append(np.r_[obs,1.,min(held/54,1.)]);Y.append(action);C.append(index);S.append(step)
            old=d.ctrl.copy();d.ctrl[:]=ticks_to_sim(goal)
            for _ in range(75):
                mujoco.mj_step(m,d);why=guard.check(d)
                if why:raise AssertionError((case,step,why))
            b.set_last_act(np.clip((d.ctrl-old)/.2,-1,1))
            B=d.geom_xmat[cube].reshape(3,3);center=float(np.linalg.norm(d.geom_xpos[cube]-station)*1000)
            gap=min(value for _,value in gaps(m,d,d.geom_xpos[cube],B,geometry,True))
            valid=orientation_error_deg(B,target)<1.5 and face_up_error_deg(B,5,z['up'])<5 and center<2 and gap>.3
            held=held+1 if valid else 0
        expected=np.load(OUT/teacher/'end_state.npz',allow_pickle=False)['qpos']
        error=float(np.max(np.abs(d.qpos-expected)))
        if error>1e-8:raise AssertionError((case,'Teacher replay mismatch',error))
        hold_report=None
        if args.hold_steps:
            if args.hold_steps<134:raise ValueError('A full settling and4050 sample tail is required')
            constant=d.ctrl.copy();certificate=SweepCertificate(m,guard.hand,.3)
            tail_n=lower_n=bad_n=0;minimum=float('inf');tail_B=tail_C=None;drift=shift=0.;stop=None
            for t in range(args.hold_steps):
                obs=build_observation(b,m,d,d.ctrl,quat);X.append(np.r_[obs,1.,min(held/54,1.)])
                Y.append(np.zeros(16));C.append(index);S.append(len(goals)+t)
                d.ctrl[:]=constant
                for _ in range(75):
                    mujoco.mj_step(m,d);stop=guard.check(d)
                    if t>=args.hold_steps-54:
                        B=d.geom_xmat[cube].reshape(3,3);center_now=d.geom_xpos[cube];force={}
                        for ci,c in enumerate(d.contact):
                            x,y=int(c.geom1),int(c.geom2)
                            if cube not in (x,y):continue
                            name=m.geom(y if x==cube else x).name
                            if name in ('if_tip','th_tip') and -(c.pos-center_now)@B[:,1]*1000<55/6:
                                f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);force[name]=force.get(name,0.)+float(f[0])
                        tail_n+=1;lower_n+=all(force.get(n,0.)>.2 for n in ('if_tip','th_tip'))
                        gap,bad=certificate.query(d,center_now,B);minimum=min(minimum,gap);bad_n+=bool(bad)
                        if tail_B is None:tail_B=B.copy();tail_C=center_now.copy()
                        drift=max(drift,orientation_error_deg(B,tail_B));shift=max(shift,float(np.linalg.norm(center_now-tail_C)*1000))
                    if stop:break
                b.set_last_act(np.zeros(16));B=d.geom_xmat[cube].reshape(3,3)
                center=float(np.linalg.norm(d.geom_xpos[cube]-station)*1000)
                gap=min(v for _,v in gaps(m,d,d.geom_xpos[cube],B,geometry,True))
                valid=orientation_error_deg(B,target)<1.5 and face_up_error_deg(B,5,z['up'])<5 and center<2 and gap>.3
                held=held+1 if valid else 0
                if stop:break
            passed=not stop and tail_n==4050 and lower_n/4050>.8 and not bad_n and minimum>.3 and valid and drift<3 and shift<2
            hold_report=dict(case=case,constant_joint_goal_steps=t+1,stop=stop,passed=passed,
                target_error_deg=orientation_error_deg(B,target),common_station_error_mm=center,
                tail_1ms_samples=tail_n,lower_contact_fraction=lower_n/max(1,tail_n),
                minimum_certificate_gap_mm=minimum if np.isfinite(minimum) else None,
                certificate_bad_steps=bad_n,hold_orientation_drift_deg=drift,hold_center_drift_mm=shift,
                actual_cube_never_reset_during_teacher_and_hold=True,devices_opened=0,hardware_execution_allowed=False)
            save_json(folder/(case+'_constant_hold.json'),hold_report);print(json.dumps(hold_report),flush=True)
            if not passed:
                save_json(folder/'generation_failure.json',dict(status='failed',reason='constant hold did not meet original fine correction criteria',case=case))
                raise RuntimeError('Do not label a failed constant hold as a good teacher')
        sources.append(dict(case=case,source_state=str(source.relative_to(OUT)),source_sha256=sha(source),
            teacher_goals=str(goals_path.relative_to(OUT)),teacher_sha256=sha(goals_path),
            hypothetical_friction_override=friction,examples=len(goals)+args.hold_steps,replay_qpos_error=error,
            constant_hold=hold_report))
    np.savez_compressed(folder/'examples.npz',observations=np.asarray(X,np.float32),actions=np.asarray(Y,np.float32),
                        cases=np.asarray(C,np.int32),steps=np.asarray(S,np.int32))
    save_json(folder/'manifest.json',dict(status='validated_model_teacher_correction_dataset',
        sources=sources,examples=len(X),observation_size=110,action_size=16,
        appended_validated_constant_hold_steps_per_case=args.hold_steps,
        longer_feedback_teachers=args.long_feedback_teachers,
         explicit_nominal_teacher_override=args.nominal_teacher,
        input_contract='original108 sensor-contract observations +1 constant +consecutive qualifying pose count/54',
        action_contract='incremental sim-order joint action; rounded2ticks per75ms; no reference joint path used by student',
        goal='absolute original90-degree orientation and unchanged common station',
        builder_history_warmup='Three repeated noiseless initial model observations; physical sampling not certified',
        no_contact_force_or_model_internal_state_in_student_observation=True,
        is_new_SAC_training=False,is_whole_cycle_distillation=False,devices_opened=0,hardware_execution_allowed=False))
    print(json.dumps(dict(examples=len(X),cases=len(CASES),all_teacher_replays_matched=True)),flush=True)


if __name__=='__main__':main()
