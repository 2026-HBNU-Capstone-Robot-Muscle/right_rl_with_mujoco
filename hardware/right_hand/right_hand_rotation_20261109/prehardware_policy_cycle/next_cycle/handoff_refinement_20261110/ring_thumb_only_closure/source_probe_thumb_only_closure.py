"""Compare thumb-only closing with and without an existing opposing fingertip.

The supplied photograph is not metrically registered. These are two explicitly
named saved model states, never a reconstruction or physical-force claim.
Non-thumb commands stay fixed; every joint and the cube remain dynamic.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def finger_normal(model,support,name):
    return sum(v['normal_N'] for g,v in support['by_geom'].items()
        if (model.body(int(model.geom_bodyid[model.geom(g).id])).name or '').startswith(name+'_'))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source-state');parser.add_argument('--tag',default='thumb_only_closure_20261110');parser.add_argument('--hypothetical-source',action='store_true');args=parser.parse_args()
    if not args.tag.replace('_','').isalnum():raise ValueError('Invalid local tag')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/args.tag
    if folder.exists():raise RuntimeError('Preserve prior thumb comparison')
    scenarios=[('reached_first90',base/'initial_to_reached_workstation90/end_state.npz',base/'opposing_actual_cube_contact_geometry/opposed_Z_th_proposal.npz'),
        ('initialized_opposed_grip',base/'opposing_workspace_Z_seed122/free_hold_tests/force0p8/end_state.npz',None)]
    if args.source_state:
        custom=(OUT/args.source_state).resolve()
        if not custom.is_relative_to(OUT.resolve()):raise ValueError('Source must be local research data')
        scenarios=[('custom_settled_grip',custom,None)]
    folder.mkdir();(folder/'source_probe_thumb_only_closure.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for scenario,source,proposal_path in scenarios:
        model,initial,_,_,provenance=assets('active_ccd50');z=dict(np.load(source,allow_pickle=False));data=restore(model,z);guard=Guard(model)
        cube=model.geom('cube').id;tip=model.geom('th_tip').id;thumb=np.arange(12,16);sid=np.asarray(SIM_TO_MOTOR_IDS)-1
        C0=data.geom_xpos[cube].copy();B0=data.geom_xmat[cube].reshape(3,3).copy();lag=data.ctrl-data.qpos[:16]
        ik=restore(model,z)
        if proposal_path:
            proposal=dict(np.load(proposal_path,allow_pickle=False));ik.qpos[thumb]=proposal['qpos'][thumb];ik.ctrl[thumb]=ik.qpos[thumb]+lag[thumb]
            mujoco.mj_forward(model,ik)
        options=[c for c in ik.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}]
        if not options:raise RuntimeError('Selected thumb contact is not present in current source geometry')
        contact=min(options,key=lambda c:c.dist);normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1)
        jac=np.zeros((3,model.nv));mujoco.mj_jac(model,ik,jac,None,contact.pos,int(model.geom_bodyid[tip]))
        closing=np.zeros(16);closing[thumb]=(-jac[:,:16].T@normal/model.actuator_gainprm[:,0])[thumb]
        base_control=ik.ctrl.copy()
        if proposal_path:
            ik.qvel[:]=0;mujoco.mj_forward(model,ik)
            base_control[thumb]=ik.qpos[thumb]+((ik.qfrc_bias-ik.qfrc_passive)[:16]/model.actuator_gainprm[:,0])[thumb]
        for force in (0.,.4,.8):
            # Hypothesized generalized closing effort, not a commanded real force.
            desired=z['ctrl'].copy();desired[thumb]=(base_control+closing*force)[thumb]
            target=motor(desired);tag=scenario+'_extra'+str(force).replace('.','p');sub=folder/tag;sub.mkdir()
            if np.any(target<guard.lo) or np.any(target>guard.hi):
                rows.append(dict(scenario=scenario,extra_closing_hypothesis_N=force,skipped='native_goal_bounds'));continue
            actual=restore(model,z);check=Guard(model);start=motor(actual.ctrl);previous=start.copy();commands=[];history=[];stop=[]
            # Round only the thumb; retain each other control bit-for-bit.
            n=max(1,int(np.ceil(np.max(abs(target[sid[thumb]]-start[sid[thumb]]))/2)))
            for index in range(n+94):
                command=previous.copy()
                command[sid[thumb]]=np.rint(start[sid[thumb]]+(target[sid[thumb]]-start[sid[thumb]])*min(index+1,n)/n)
                if np.max(abs(command-previous))>2+1e-9:raise AssertionError('Rate changed')
                if np.any(command<check.lo) or np.any(command>check.hi):raise AssertionError('Range changed')
                actual.ctrl[thumb]=ticks_to_sim(command)[thumb];previous=command.copy();commands.append(command.copy())
                if not np.array_equal(actual.ctrl[:12],z['ctrl'][:12]):raise AssertionError('A non-thumb control changed')
                for _ in range(75):
                    mujoco.mj_step(model,actual);stop=check.check(actual)
                    if np.linalg.norm(actual.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if index>=n+40:
                        support=sample_model(model,actual,check.hand,cube)
                        history.append(dict(qpos=actual.qpos.copy(),supported=support['supported_sample'],TH_N=finger_normal(model,support,'th'),MF_N=finger_normal(model,support,'mf'),RF_N=finger_normal(model,support,'rf')))
                    if stop:break
                if stop:break
            C=actual.geom_xpos[cube];B=actual.geom_xmat[cube].reshape(3,3);shift=float(np.linalg.norm(C-C0)*1000);angle=orientation_error_deg(B,B0)
            supported=sum(r['supported'] for r in history)/max(1,len(history));loaded=sum(r['TH_N']>.1 for r in history)/max(1,len(history))
            certificate=SweepCertificate(model,check.hand,.3);lower,bad=certificate.query(actual,C,B);minimum=lower;certified=0
            pose_pass=bool(not stop and len(history)==4050 and supported>.8 and loaded>.8 and shift<2 and angle<3)
            if pose_pass and not bad and lower>.3:
                geometry=restore(model,z)
                for sample in history:
                    geometry.qpos[:]=sample['qpos'];mujoco.mj_forward(model,geometry);value,failed=certificate.query(geometry,geometry.geom_xpos[cube],geometry.geom_xmat[cube].reshape(3,3));minimum=min(minimum,value)
                    if failed or value<=.3:break
                    certified+=1
            np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands)
            np.savez_compressed(sub/'end_state.npz',**{k:getattr(actual,k) for k in BOUND},time=actual.time,up=initial['up'],initial_cube_rotation=initial['initial_cube_rotation'],initial_cube_center=initial['initial_cube_center'])
            terminal=sample_model(model,actual,check.hand,cube)
            row=dict(scenario=scenario,source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),source_has_initialized_ancestor=scenario=='initialized_opposed_grip' or args.hypothetical_source,
                thumb_contact_reference=proposal_path.relative_to(OUT).as_posix() if proposal_path else None,extra_closing_hypothesis_N=force,steps=len(commands),stop=stop,hold_samples=len(history),
                whole_hand_support_fraction=supported,TH_over_point1N_fraction=loaded,final_TH_normal_N=finger_normal(model,terminal,'th'),final_MF_normal_N=finger_normal(model,terminal,'mf'),final_RF_normal_N=finger_normal(model,terminal,'rf'),
                cube_shift_from_source_mm=shift,orientation_change_from_source_deg=angle,upper_certified_samples=certified,minimum_upper_lower_mm=minimum,final_bad_upper_geoms=[model.geom(int(certificate.ids[i])).name for i in bad],
                thumb_loaded_pose_and_clearance_hold_passed=bool(pose_pass and certified==4050),non_thumb_controls_bitwise_unchanged=True,
                terminal_support=terminal,worst=check.worst,terminal_guard=check.last,model_provenance=provenance,path=sub.relative_to(OUT).as_posix())
            save_json(sub/'review.json',row);rows.append(row)
            print(json.dumps({k:row[k] for k in ('scenario','extra_closing_hypothesis_N','steps','stop','final_TH_normal_N','cube_shift_from_source_mm','orientation_change_from_source_deg','thumb_loaded_pose_and_clearance_hold_passed')}),flush=True)
    save_json(folder/'review.json',dict(status='thumb_only_closing_at_two_named_model_states',rows=rows,
        photo_pose_not_metrically_registered_and_not_reconstructed=True,other_finger_controls_held_but_finger_dynamics_not_frozen=True,cube_free_in_all_actual_steps=True,
        constant_force_labels_are_model_hypotheses_not450mA_calibration=True,all_actual_moves_max2ticks_per75ms=True,original_native_guards_and_joint_limits_preserved=True,
        actual_state_resets_within_each_trial=0,next_rotation_not_evaluated=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
