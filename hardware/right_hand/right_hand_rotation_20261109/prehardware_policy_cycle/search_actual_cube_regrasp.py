"""Reacquire MF/TH contacts at the actual cube, before trying to move its center.

The object is fixed only in IK copies. Selected joint commands run against a
free native object and keep all existing command/contact/tracking bounds.
"""
from pathlib import Path
import argparse
import json
import time
import numpy as np
import mujoco
from scipy.optimize import least_squares,minimize,LinearConstraint
from scipy.spatial import ConvexHull
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,TICK,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--global-proposals');parser.add_argument('--model-gravity-balance',action='store_true')
    parser.add_argument('--first-finger',choices=('mf','rf'),default='mf');parser.add_argument('--source-state');args=parser.parse_args()
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/('actual_cube_MF_TH_regrasp_opposed' if args.global_proposals and 'opposing_' in args.global_proposals else ('actual_cube_MF_TH_regrasp_global' if args.global_proposals else 'actual_cube_MF_TH_regrasp'))
    if args.model_gravity_balance:folder=folder.with_name(folder.name+'_gravity')
    if args.first_finger=='rf':folder=folder.with_name(folder.name.replace('MF_TH','RF_TH'))
    if folder.exists():raise RuntimeError('Preserve previous regrasp study')
    source=base/'first90_RF_shape_before_adduction/RF_park_MF_then_RF_shape_then_adduct/phase0_end_state.npz'
    if args.source_state:
        source=(OUT/args.source_state).resolve()
        if not source.is_relative_to(OUT.resolve()):raise ValueError('Source must be local research data')
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;guard=Guard(model)
    z=dict(np.load(source,allow_pickle=False));data=restore(model,z);C=data.geom_xpos[cube].copy();B=data.geom_xmat[cube].reshape(3,3).copy();lag=data.ctrl-data.qpos[:16]
    global_data=None;global_path=None
    if args.global_proposals:
        global_path=(OUT/args.global_proposals).resolve()
        if not global_path.is_relative_to(OUT.resolve()):raise ValueError('Global proposals must be local')
        global_data=json.loads(global_path.read_text())
        if global_data['source_sha256']!=sha(source):raise RuntimeError('Global source differs from actual regrasp source')
    sid=np.array(SIM_TO_MOTOR_IDS)-1;low=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag);high=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    geometry={g:(v,e) for g,v,e in precompute(model,guard.hand)};reference=dict(np.load(base/'RF_clear_work_grip_joined_suffix/end_state.npz',allow_pickle=False))
    folder.mkdir();(folder/'source_search_actual_cube_regrasp.py').write_bytes(Path(__file__).read_bytes());began=time.perf_counter();fits=[];selected={}
    first_finger=args.first_finger;first_active=np.arange(4,8) if first_finger=='mf' else np.arange(8,12)
    for finger,active in ((first_finger,first_active),('th',np.arange(12,16))):
        if global_data and any(row['finger']==finger for row in global_data['rows']):
            chosen=[]
            for index,row in enumerate(global_data['rows']):
                if row['finger']!=finger or not row['static_contact_candidate']:continue
                proposal_path=OUT/row['proposal']
                if sha(proposal_path)!=row['proposal_sha256']:raise RuntimeError('Global proposal changed')
                proposal=dict(np.load(proposal_path,allow_pickle=False))
                if all(np.max(abs(motor(proposal['ctrl'])-motor(old[2])))>20 for old in chosen):chosen.append((0.,proposal['qpos'],proposal['ctrl'],10000+index))
                if len(chosen)==2:break
            selected[finger]=chosen
            print(json.dumps(dict(finger=finger,global_candidates=len(chosen))),flush=True)
            continue
        geom=model.geom(finger+'_tip').id;vertices,_=geometry[geom];hull=ConvexHull(vertices);A=hull.equations[:,:3];bounds=-hull.equations[:,3]
        center=(data.geom_xpos[geom]-C)@B*1000;axis=max((0,2),key=lambda a:abs(center[a]));across=2 if axis==0 else 0;sign=np.sign(center[axis]);candidates=[]
        for height in (0.,10.,20.):
            for offset in (-12.,0.,12.):
                local_target=np.zeros(3);local_target[axis]=sign*27.45;local_target[1]=height;local_target[across]=offset;target=C+B@local_target/1000
                G=data.geom_xmat[geom].reshape(3,3);local_goal=G.T@(target-data.geom_xpos[geom])
                closest=minimize(lambda point:.5*float(np.sum((point-local_goal)**2)),vertices.mean(0),jac=lambda point:point-local_goal,
                    constraints=[LinearConstraint(A,-np.inf,bounds)],method='SLSQP',options=dict(ftol=1e-14,maxiter=100))
                if np.max(A@closest.x-bounds)>1e-7:continue
                contact_local=closest.x;normal_local=A[int(np.argmax(A@contact_local-bounds))];normal_world=sign*B[:,axis]
                for seed_index,seed in enumerate((data.qpos[active],reference['qpos'][active])):
                    proposal=restore(model,z)
                    def residual(x):
                        proposal.qpos[:]=z['qpos'];proposal.qpos[active]=x;proposal.ctrl[:]=proposal.qpos[:16]+lag;mujoco.mj_forward(model,proposal)
                        R=proposal.geom_xmat[geom].reshape(3,3);point=proposal.geom_xpos[geom]+R@contact_local;support,own=guard.contacts(proposal)
                        penetration=max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                        return np.r_[(point-target)*1000,(R@normal_local+normal_world)*2.,(x-seed)*.05,
                            max(own.values(),default=0.)*1500,max(support.values(),default=0.)*1000,max(0.,penetration-.8)*400]
                    fit=least_squares(residual,np.clip(seed,low[active],high[active]),bounds=(low[active],high[active]),max_nfev=100,ftol=1e-8,xtol=1e-8,gtol=1e-8)
                    error=float(np.linalg.norm(residual(fit.x)[:3]));support,own=guard.contacts(proposal);goal=motor(proposal.ctrl)
                    contacts=[c for c in proposal.contact if cube in (int(c.geom1),int(c.geom2)) and
                        (model.body(int(model.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)])).name or '').startswith(finger+'_') and c.dist<.0002]
                    valid=bool(contacts and error<1.5 and max(own.values(),default=0.)<=1e-6 and max(support.values(),default=0.)<=.15 and np.all(goal>=guard.lo) and np.all(goal<=guard.hi))
                    row=dict(finger=finger,target_cube_frame_mm=local_target.tolist(),seed=seed_index,point_error_mm=error,self_pairs_mm=own,static_contact_candidate=valid,goal_ticks=goal.tolist())
                    fits.append(row)
                    if valid:candidates.append((float(np.linalg.norm(goal-motor(z['ctrl']))),proposal.qpos.copy(),proposal.ctrl.copy(),len(fits)-1))
        chosen=[]
        for candidate in sorted(candidates,key=lambda item:item[0]):
            if all(np.max(abs(motor(candidate[2])-motor(old[2])))>20 for old in chosen):chosen.append(candidate)
            if len(chosen)==2:break
        selected[finger]=chosen
        print(json.dumps(dict(finger=finger,fits=len([row for row in fits if row['finger']==finger]),candidates=len(candidates),selected=len(chosen))),flush=True)
    target_rotation=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90);canonical=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4];trials=[];skipped_commands=[]
    for mf in selected[first_finger]:
        for th in selected['th']:
            if global_data and mf[3]>=10000 and th[3]>=10000:
                if global_data['rows'][mf[3]-10000].get('grasp_profile')!=global_data['rows'][th[3]-10000].get('grasp_profile'):continue
            proposal=restore(model,z);proposal.qpos[first_active]=mf[1][first_active];proposal.qpos[12:16]=th[1][12:16];proposal.ctrl[:]=proposal.qpos[:16]+lag;mujoco.mj_forward(model,proposal)
            _,own=guard.contacts(proposal)
            if max(own.values(),default=0.)>1e-6:continue
            closing=np.zeros(16);present=True
            for finger,active in ((first_finger,first_active),('th',np.arange(12,16))):
                contacts=[c for c in proposal.contact if cube in (int(c.geom1),int(c.geom2)) and
                    (model.body(int(model.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)])).name or '').startswith(finger+'_')]
                if global_data and (mf[3] if finger==first_finger else th[3])>=10000:
                    metadata=global_data['rows'][(mf[3] if finger==first_finger else th[3])-10000]
                    if 'grasp_profile' in metadata:
                        local_point=np.asarray(metadata['target_cube_frame_mm']);axis=max((0,2),key=lambda a:abs(local_point[a]));expected_normal=np.sign(local_point[axis])*B[:,axis]
                        contacts=[c for c in contacts if float((c.frame[:3]*(1 if int(c.geom1)==cube else -1))@expected_normal)>.7]
                if not contacts:present=False;break
                contact=min(contacts,key=lambda c:c.dist);other=int(contact.geom2) if int(contact.geom1)==cube else int(contact.geom1)
                normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1);jac=np.zeros((3,model.nv));mujoco.mj_jac(model,proposal,jac,None,contact.pos,int(model.geom_bodyid[other]))
                closing[active]=(-jac[:,:16].T@normal/model.actuator_gainprm[:,0])[active]
            if not present:continue
            base_control=proposal.ctrl.copy()
            if args.model_gravity_balance:
                if not np.allclose(model.actuator_gear[:,0],1.) or not np.allclose(model.actuator_biasprm[:,1],-model.actuator_gainprm[:,0]):raise RuntimeError('Unexpected actuator force contract')
                proposal.qvel[:]=0;mujoco.mj_forward(model,proposal);active=np.r_[first_active,12:16]
                base_control[active]=proposal.qpos[active]+((proposal.qfrc_bias-proposal.qfrc_passive)[:16]/model.actuator_gainprm[:,0])[active]
            for preload in (.4,.8):
                goal=motor(base_control+closing*preload)
                if np.any(goal<guard.lo) or np.any(goal>guard.hi):
                    skipped_commands.append(dict(MF_fit=mf[3],TH_fit=th[3],preload=preload,reason='model_gravity_or_preload_native_goal_bounds'));continue
                for order,groups in (('together',[np.r_[sid[first_active],sid[12:16]]]),(first_finger.upper()+'_first',[sid[first_active],sid[12:16]]),('TH_first',[sid[12:16],sid[first_active]])):
                    actual=restore(model,z);check=Guard(model);start=motor(actual.ctrl);previous=start.copy();commands=[];stop=[]
                    for group in groups:
                        before=previous.copy();after=before.copy();after[group]=goal[group];count=max(1,int(np.ceil(np.max(abs(after-before))/2)))
                        for step in range(count):
                            command=np.rint(before+(after-before)*(step+1)/count).astype(int)
                            if np.max(abs(command-previous))>2:raise RuntimeError('Rate changed')
                            previous=command;actual.ctrl[:]=ticks_to_sim(command);commands.append(command.copy())
                            for _ in range(75):
                                mujoco.mj_step(model,actual);stop=check.check(actual)
                                if np.linalg.norm(actual.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                                if stop:break
                            if stop:break
                        if stop:break
                    samples=supported=mf_loaded=th_loaded=0
                    if not stop:
                        for cycle in range(94):
                            commands.append(previous.copy())
                            for _ in range(75):
                                mujoco.mj_step(model,actual);stop=check.check(actual)
                                if np.linalg.norm(actual.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                                if cycle>=40:
                                    support=sample_model(model,actual,check.hand,cube);samples+=1;supported+=support['supported_sample']
                                    loads={finger:sum(value['normal_N'] for name,value in support['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith(finger+'_')) for finger in (first_finger,'th')}
                                    mf_loaded+=loads[first_finger]>.1;th_loaded+=loads['th']>.1
                                if stop:break
                            if stop:break
                    error=orientation_error_deg(actual.geom_xmat[cube].reshape(3,3),target_rotation);center_error=float(np.linalg.norm(actual.geom_xpos[cube]-canonical)*1000)
                    passed=bool(not stop and samples==4050 and supported/4050>.8 and mf_loaded/4050>.8 and th_loaded/4050>.8 and error<5)
                    certificate=SweepCertificate(model,check.hand,.3);lower,bad=certificate.query(actual,actual.geom_xpos[cube],actual.geom_xmat[cube].reshape(3,3))
                    label=f'mf{mf[3]}_th{th[3]}_force{preload:g}_{order}';trial=folder/label;trial.mkdir();np.savez_compressed(trial/'research_goals.npz',start=start,goals=commands)
                    np.savez_compressed(trial/'end_state.npz',**{key:getattr(actual,key) for key in BOUND},time=actual.time,
                        up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
                    row=dict(path=trial.relative_to(OUT).as_posix(),MF_fit=mf[3],TH_fit=th[3],preload_force_hypothesis_N=preload,order=order,steps=len(commands),stop=stop,
                        hold_samples=samples,whole_hand_support_fraction=supported/max(1,samples),MF_loaded_fraction=mf_loaded/max(1,samples) if first_finger=='mf' else 0.,RF_loaded_fraction=mf_loaded/max(1,samples) if first_finger=='rf' else 0.,TH_loaded_fraction=th_loaded/max(1,samples),
                        final_orientation_error_deg=error,canonical_station_error_mm=center_error,MF_TH_regrasp_for_translation_passed=bool(passed and first_finger=='mf'),RF_TH_regrasp_for_translation_passed=bool(passed and first_finger=='rf'),paired_regrasp_for_translation_passed=passed,
                        final_upper_lower_mm=lower,final_bad_upper_geoms=[model.geom(int(certificate.ids[i])).name for i in bad],
                        full_work_grip_not_certified_by_final_pose_query=True,worst=check.worst,terminal_guard=check.last,terminal_support=sample_model(model,actual,check.hand,cube))
                    save_json(trial/'review.json',row);trials.append(row);print(json.dumps({key:row[key] for key in ('path','steps','stop','MF_TH_regrasp_for_translation_passed','canonical_station_error_mm')}),flush=True)
    save_json(folder/'review.json',dict(status='actual_cube_MF_TH_contact_reacquisition',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,
        fits=fits,selected_finger_candidates={key:len(value) for key,value in selected.items()},trials=trials,
        global_proposal_study=global_path.relative_to(OUT).as_posix() if global_path else None,global_proposal_sha256=sha(global_path) if global_path else None,
        first_contact_finger=first_finger,model_gravity_balance_only_on_declared_contact_fingers=args.model_gravity_balance,skipped_commands=skipped_commands,
        regrasp_passed=any(row['paired_regrasp_for_translation_passed'] for row in trials),
        cube_fixed_only_in_ik_copies=True,cube_free_in_every_actual_trial=True,original_guards_and_joint_limits_preserved=True,
        force_hypotheses_not_calibrated_to_450mA=True,full_initial_placement_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,
        elapsed_seconds=time.perf_counter()-began))


if __name__=='__main__':main()
