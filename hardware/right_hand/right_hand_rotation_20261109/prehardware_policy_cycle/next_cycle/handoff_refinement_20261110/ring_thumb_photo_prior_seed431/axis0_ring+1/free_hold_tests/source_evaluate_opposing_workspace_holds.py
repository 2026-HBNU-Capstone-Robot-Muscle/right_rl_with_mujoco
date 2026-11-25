"""Free-object cold-start hold tests of an explicitly hypothetical workspace.

No arrival is claimed from these initialized states. Forces are model-only
hypotheses; actuator bounds remain unchanged and are not a 450 mA conversion.
"""
from pathlib import Path
import argparse
import json
import time
import numpy as np
import mujoco
from native_cycle import OUT, assets, restore, Guard, BOUND, motor, ticks_to_sim, TICK, save_json, sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--proposal',required=True);parser.add_argument('--yaw',type=float);args=parser.parse_args()
    path=(OUT/args.proposal).resolve()
    if not path.is_relative_to(OUT.resolve()):raise ValueError('Local proposal required')
    report=json.loads(path.read_text());folder=path.parent/'free_hold_tests'
    if args.yaw is not None:
        if args.yaw not in (90.,180.):raise ValueError('Only explicit90/180 initialized orientation checks')
        folder=folder.with_name(folder.name+'_yaw'+str(int(args.yaw)))
    if folder.exists():raise RuntimeError('Preserve prior hold tests')
    if not report['static_opposed_contact_candidate']:raise RuntimeError('No paired static candidate')
    proposed=OUT/report['proposal']
    if sha(proposed)!=report['proposal_sha256']:raise RuntimeError('Proposal changed')
    model,initial,_,_,provenance=assets('active_ccd50');guard=Guard(model);cube=model.geom('cube').id
    if not np.allclose(model.actuator_gear[:,0],1.) or not np.allclose(model.actuator_biasprm[:,1],-model.actuator_gainprm[:,0]):raise RuntimeError('Unsupported actuator contract')
    z=dict(np.load(proposed,allow_pickle=False));ik=restore(model,z);original_B=ik.geom_xmat[cube].reshape(3,3).copy()
    if args.yaw is not None:
        target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],args.yaw)
        quat=np.empty(4);mujoco.mju_mat2Quat(quat,target.ravel());z['qpos']=z['qpos'].copy();z['qpos'][-4:]=quat;ik=restore(model,z)
    ik.qvel[:]=0.;ik.qacc_warmstart[:]=0.;mujoco.mj_forward(model,ik)
    C=ik.geom_xpos[cube].copy();B=ik.geom_xmat[cube].reshape(3,3).copy()
    base=ik.qpos[:16]+((ik.qfrc_bias-ik.qfrc_passive)[:16]/model.actuator_gainprm[:,0])
    closing=np.zeros(16);contact_setup=[]
    first_contact=next(item['finger'] for item in report['contacts'] if item['finger']!='th')
    for item in report['contacts']:
        name=item['finger'];active={'mf':np.arange(4,8),'rf':np.arange(8,12),'th':np.arange(12,16)}[name];tip=model.geom(name+'_tip').id
        target=np.asarray(item['target_cube_frame_mm']);axis=max((0,2),key=lambda a:abs(target[a]));expected=np.sign(target[axis])*original_B[:,axis]
        options=[c for c in ik.contact if {int(c.geom1),int(c.geom2)}=={cube,tip} and float((c.frame[:3]*(1 if int(c.geom1)==cube else -1))@expected)>.8]
        if not options:raise RuntimeError('Native opposite normal not reproduced')
        contact=min(options,key=lambda c:c.dist);normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1)
        jac=np.zeros((3,model.nv));mujoco.mj_jac(model,ik,jac,None,contact.pos,int(model.geom_bodyid[tip]))
        closing[active]=(-jac[:,:16].T@normal/model.actuator_gainprm[:,0])[active]
        contact_setup.append(dict(finger=name,normal_world=normal.tolist(),point_world_m=contact.pos.tolist()))
    folder.mkdir();(folder/'source_evaluate_opposing_workspace_holds.py').write_bytes(Path(__file__).read_bytes());began=time.perf_counter();rows=[]
    for force in (.8,1.6,2.4,3.2):
        goal=np.rint(motor(base+force*closing)).astype(int)
        if np.any(goal<guard.lo) or np.any(goal>guard.hi):
            rows.append(dict(preload_hypothesis_N=force,skipped='native_command_bounds',initialized_hold_passed=False));continue
        actual=restore(model,z);actual.qvel[:]=0.;actual.qacc_warmstart[:]=0.;actual.ctrl[:]=ticks_to_sim(goal);mujoco.mj_forward(model,actual)
        check=Guard(model);stop=check.check(actual);supported=mf=th=certified=count=0;minimum=float('inf');shift=angle=torque=0.;trace=[];trajectory=[actual.qpos.copy()]
        # Command is explicitly initialized here, not a rate-limited arrival.
        certificate=SweepCertificate(model,check.hand,.3)
        cycle=-1
        for cycle in range(0 if stop else 94):
            for _ in range(75):
                mujoco.mj_step(model,actual);stop=check.check(actual)
                if np.linalg.norm(actual.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                torque=max(torque,float(np.max(abs(actual.actuator_force))))
                if cycle>=40:
                    load=sample_model(model,actual,check.hand,cube);count+=1;supported+=load['supported_sample']
                    by={name:sum(v['normal_N'] for g,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(g).id])).name or '').startswith(name+'_')) for name in (first_contact,'th')}
                    mf+=by[first_contact]>.1;th+=by['th']>.1
                    lower,bad=certificate.query(actual,actual.geom_xpos[cube],actual.geom_xmat[cube].reshape(3,3));minimum=min(minimum,lower);certified+=not bad and lower>.3
                    shift=max(shift,float(np.linalg.norm(actual.geom_xpos[cube]-C)*1000));angle=max(angle,orientation_error_deg(actual.geom_xmat[cube].reshape(3,3),B))
                if stop:break
            trajectory.append(actual.qpos.copy())
            if cycle%10==0:trace.append(dict(cycle=cycle,shift_mm=float(np.linalg.norm(actual.geom_xpos[cube]-C)*1000),error_deg=orientation_error_deg(actual.geom_xmat[cube].reshape(3,3),B),support=sample_model(model,actual,check.hand,cube)))
            if stop:break
        label='force'+str(force).replace('.','p');sub=folder/label;sub.mkdir()
        np.savez_compressed(sub/'end_state.npz',**{key:getattr(actual,key) for key in BOUND},time=actual.time,up=initial['up'],initial_cube_rotation=initial['initial_cube_rotation'],initial_cube_center=initial['initial_cube_center'])
        np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075)
        passed=bool(not stop and count==4050 and supported/4050>.8 and mf/4050>.8 and th/4050>.8 and certified==4050 and shift<2 and angle<3)
        row=dict(preload_hypothesis_N=force,steps=cycle+1,stop=stop,hold_samples=count,whole_hand_support_fraction=supported/max(1,count),MF_load_fraction=mf/max(1,count) if first_contact=='mf' else 0.,
            RF_load_fraction=mf/max(1,count) if first_contact=='rf' else 0.,first_contact_finger=first_contact,opposing_finger_load_fraction=mf/max(1,count),TH_load_fraction=th/max(1,count),
            certified_upper_samples=certified,minimum_upper_lower_mm=minimum if np.isfinite(minimum) else None,maximum_station_drift_mm=shift,maximum_orientation_drift_deg=angle,
            max_model_actuator_force=torque,terminal_center=actual.geom_xpos[cube].tolist(),initialized_hold_passed=passed,worst=check.worst,terminal_guard=check.last,
            terminal_support=sample_model(model,actual,check.hand,cube),path=sub.relative_to(OUT).as_posix(),trace=trace)
        rows.append(row);save_json(sub/'review.json',row)
        print(json.dumps({key:row[key] for key in ('preload_hypothesis_N','stop','hold_samples','MF_load_fraction','TH_load_fraction','maximum_station_drift_mm','initialized_hold_passed')}),flush=True)
    save_json(folder/'review.json',dict(status='initialized_free_cube_opposite_face_hold_tests',proposal_study=path.relative_to(OUT).as_posix(),proposal_study_sha256=sha(path),model_provenance=provenance,
        initialized_cube_station_world_m=C.tolist(),explicit_initialized_global_yaw=args.yaw,first_contact_finger=first_contact,contact_setup=contact_setup,rows=rows,initialized_hold_passed=any(r['initialized_hold_passed'] for r in rows),
        cube_is_free_in_every_timestep=True,source_explicitly_initialized_not_reached=True,initial_control_jump_not_an_arrival=True,arrival_path_passed=False,
        force_hypotheses_not_calibrated_to_450mA=True,original_native_guards_and_joint_limits_preserved=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,
        elapsed_seconds=time.perf_counter()-began))


if __name__=='__main__':main()
