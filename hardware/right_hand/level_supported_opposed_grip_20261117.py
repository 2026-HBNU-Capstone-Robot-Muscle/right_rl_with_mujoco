"""Find a supported opposed-grip equilibrium with bounded offline feedback.

Uses perfect simulated pose/contacts to search position goals, then freezes
the goal and independently replays the whole path with the cube free. No motor
transport, external cube force, pose reset or simulator-limit edits.
"""
from pathlib import Path
import argparse,hashlib,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from research_quasistatic_wrench_path_20261105 import allocate_native,hybrid_increment
from offline_fractional_goal_accumulator_20261105 import FractionalGoals
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run
from hand_motor_mapping import ticks_to_sim
from cube_task_frame_20261105 import orientation_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--normal',type=float,choices=(1.2,1.6),default=1.2)
    ap.add_argument('--settle-deg',type=float,choices=(1.,3.),default=3.)
    ap.add_argument('--source-kind',choices=('pair','triple','settled_triple'),default='pair')
    ap.add_argument('--keep-lower-contacts',action='store_true')
    ap.add_argument('--active-hold',action='store_true')
    ap.add_argument('--hold-goals',type=int,choices=(134,400),default=134)
    ap.add_argument('--lock-measured-height',action='store_true')
    ap.add_argument('--lower-mm',type=float,choices=(0.,2.),default=0.)
    ap.add_argument('--measured-loads',action='store_true')
    a=ap.parse_args()
    settled=a.source_kind=='settled_triple'
    triple=a.source_kind in ('triple','settled_triple')
    parent=(ROOT/'right_hand_rotation_20261117/opposed_lower_grip/shared_palm_station18') if triple else (ROOT/'right_hand_rotation_20261116/opposed_lower_grip/index_base_support/palm_offset')
    source=parent/('hold_end.npz' if settled else 'independent_initial.npz')
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    tag=('level_triple_normal' if triple else 'level_normal')+str(a.normal).replace('.','p')+('_settle1' if a.settle_deg==1 else '')
    if settled:tag+='_settled_frame'
    if a.keep_lower_contacts:tag+='_lower'
    if a.active_hold:tag+='_active'
    if a.hold_goals!=134:tag+='_hold400'
    if a.lock_measured_height:tag+='_heightlock'+str(a.lower_mm).replace('.','p')
    if a.measured_loads:tag+='_measuredloads'
    out=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/tag
    out.mkdir(parents=True,exist_ok=False)
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()})
    z=dict(np.load(source,allow_pickle=False));d=restore(m,z);hand,support,check=inspect_for(m)
    geometry=precompute(m,hand);cube=m.geom('cube').id;body=m.body('cube').id
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();lo,hi,nlo,nhi=native(m)
    goal=start=motor(d.ctrl);planner=FractionalGoals(start,np.minimum(lo+8,start),np.maximum(hi-8,start))
    references=({'if_tip':a.normal*2/3,'mf_tip':a.normal*2/3,'th_tip':a.normal*4/3} if triple else {'if_tip':a.normal,'th_tip':a.normal})
    roles=tuple(references);memory={};normals={};measured_heights={}
    if a.measured_loads:
        references=dict.fromkeys(roles,0.)
        for ci,c in enumerate(d.contact):
            aa,bb=int(c.geom1),int(c.geom2)
            if cube not in (aa,bb):continue
            name=m.geom(bb if aa==cube else aa).name
            if name in references:
                force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
                references[name]+=float(force[0])
        if min(references.values())<=.2:raise RuntimeError('Source lower contact is not loaded')
    for name in references:
        g=m.geom(name).id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist)
        G=d.geom_xmat[g].reshape(3,3);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        normals[name]=(G.T@normal,B0.T@normal)
        measured_heights[name]=float((c.pos-C0)@(-B0[:,1]))-a.lower_mm/1000
    properties={k:getattr(m,k).copy() for k in ('actuator_gainprm','actuator_biasprm','actuator_forcelimited','actuator_forcerange','jnt_range','geom_size','geom_contype','geom_conaffinity')}
    rows=[];phases=[];trace=[];poses=[];stable=frozen=0;phase='level';stop=None;gap=0.;began=time.perf_counter()
    worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.)
    hold_certificate=SweepCertificate(m,hand,.3) if a.lock_measured_height else None
    hold_certificate_min=float('inf');hold_certificate_bad=0;hold_certificate_queries=0
    for index in range(1,401+a.hold_goals):
        C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
        if phase=='level' or (a.active_hold and phase=='feedback_hold'):
            velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,body,velocity,0)
            F=-m.body_mass[body]*m.opt.gravity+30*(C0-C)-1.5*velocity[3:]
            T=.04*Rotation.from_matrix(B0@B.T).as_rotvec()-.002*velocity[:3]
            proposed,allocation=allocate_native(m,d,cube,C,F,T,references.copy(),hand,True,(nlo,nhi),roles,normal_reference_weight=.2)
            if proposed is None:stop='force_allocation:'+allocation['reason'];break
            goal=hybrid_increment(m,d,allocation,planner,C,C0,B,B0,tip_memory=memory,
                maximum_tracking_rad=.12,contact_normal_frames=normals,tip_memory_names=roles,recovery_normal_targets=references,
                contact_height_targets=measured_heights if a.lock_measured_height else ({role:-.014 for role in roles} if a.keep_lower_contacts else None),
                world_up=-B[:,1] if a.lock_measured_height else z['up'])
        rows.append(goal.copy());phases.append(phase);d.ctrl[:]=ticks_to_sim(goal)
        for sub in range(75):
            mujoco.mj_step(m,d);su,sp=check(d)
            worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.))
            worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))))
            worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
            inhand=external=False
            for contact in d.contact:
                aa,bb=int(contact.geom1),int(contact.geom2)
                if cube not in (aa,bb):continue
                other=bb if aa==cube else aa
                if int(m.geom_bodyid[other]) in hand:
                    inhand|=contact.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(contact.dist)*1000))
                elif other in support and contact.dist<.0004:external=True
            gap=0. if inhand else gap+.001
            if (not np.isfinite(np.r_[d.qpos,d.qvel]).all() or worst['self_mm']>1e-6 or worst['support_mm']>.15
                or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or gap>.25
                or np.linalg.norm(d.qpos[-7:-4]-C0)>.08):stop='original_dynamic_bound';break
            if hold_certificate is not None and phase!='level':
                certified,bad=hold_certificate.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3))
                hold_certificate_queries+=1;hold_certificate_min=min(hold_certificate_min,certified)
                if bad:hold_certificate_bad+=1;stop='working_upper_space_bound';break
        C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
        tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))))
        error=orientation_error_deg(B,B0);shift=float(np.linalg.norm(C-C0)*1000)
        clearance=min(v for _,v in gaps(m,d,C,B,geometry,True))
        if phase=='level':
            velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,body,velocity,0)
            ready=(settled or tilt<a.settle_deg) and error<a.settle_deg and shift<1.5 and clearance>.8 and np.linalg.norm(velocity[:3])<.02 and np.linalg.norm(velocity[3:])<.002
            stable=stable+1 if ready else 0
            if stable>=20:phase='feedback_hold' if a.active_hold else 'fixed_hold'
        else:frozen+=1
        if index%20==0 or stop or frozen>=a.hold_goals:
            row=dict(sample=index,phase=phase,tilt_deg=tilt,orientation_error_deg=error,center_error_mm=shift,upper_gap_mm=clearance)
            trace.append(row);print(json.dumps(row),flush=True)
        if index%2==0:poses.append(d.qpos.copy())
        if stop or frozen>=a.hold_goals:break
        if phase=='level' and index>=400:stop='bounded_leveling_not_converged';break
    for key,value in properties.items():
        if not np.array_equal(getattr(m,key),value):raise RuntimeError('Model property changed: '+key)
    np.savez_compressed(out/'research_goals.npz',start=start,goals=np.array(rows),phases=np.array(phases))
    np.savez_compressed(out/'trajectory.npz',qpos=np.array(poses))
    np.savez_compressed(out/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,
        time=d.time,up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    verification=None
    if stop is None and frozen>=a.hold_goals:
        verification=run(m,z,rows,1,out/'verified_end_state.npz',out/'verified_trace.npz',True,5,roles)
    assessment=None
    if verification is not None:
        samples=np.load(out/'verified_trace.npz',allow_pickle=False)['qpos'];measure_d=mujoco.MjData(m)
        max_angle=max_shift=0.
        for q in samples:
            measure_d.qpos[:]=q;mujoco.mj_forward(m,measure_d)
            rot=measure_d.geom_xmat[cube].reshape(3,3)
            max_angle=max(max_angle,float(np.degrees(np.linalg.norm(Rotation.from_matrix(rot@B0.T).as_rotvec()))))
            max_shift=max(max_shift,float(np.linalg.norm(measure_d.geom_xpos[cube]-C0)*1000))
        tail=verification['settled_tail']
        assessment=dict(reference='saved settled hand-relative grip' if settled else 'original gravity-upright grip',
            source_gravity_tilt_deg=float(np.degrees(np.arccos(np.clip((-B0[:,1])@z['up'],-1,1)))),
            maximum_sampled_angle_deg=max_angle,maximum_sampled_shift_mm=max_shift,
            evaluated_feedback_hold_seconds=frozen*.075,feedback_used_during_hold=a.active_hold)
        assessment['stable_supported_grip_passed']=bool(verification['safety_passed'] and verification['completed_goals']==len(rows)
            and max_angle<5 and max_shift<2 and tail['minimum_upper_gap_sampled_75ms_mm']>.3
            and tail['separation_certificate_bad_steps_1ms']==0 and tail['opposed_lower_body_contact_fraction']>.8)
        if hold_certificate is not None:
            assessment['stable_supported_grip_passed'] &= hold_certificate_bad==0 and hold_certificate_queries==frozen*75
    report=dict(status='bounded_supported_opposed_grip_leveling',source_sha256=sha(source),model_sha256=sha(model),
        source_script_sha256=sha(Path(__file__)),
        source_path=source.relative_to(ROOT).as_posix(),required_lower_contacts=list(roles),
        parameters=vars(a),stop=stop,phase=phase,samples=len(rows),fixed_hold_samples=frozen,worst=worst,trace=trace,
        recorded_goal_replay=verification,common_grip_passed=bool(verification and verification['whole_grip_task_passed']),
        hand_relative_grip_assessment=assessment,gravity_upright_verdict_preserved=True,left_hand_alignment_verified=False,
        measured_contact_height_targets_m=measured_heights if a.lock_measured_height else None,
        contact_normal_targets_N=references,
        full_hold_upper_space_certificate=dict(queries=hold_certificate_queries,bad_steps=hold_certificate_bad,
            minimum_gap_mm=hold_certificate_min if np.isfinite(hold_certificate_min) else None),
        perfect_simulated_pose_and_contacts=True,external_cube_forces_written=False,cube_reset_after_start=False,
        model_properties_unchanged=True,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(passed=report['common_grip_passed'],hand_relative_assessment=assessment,stop=stop,samples=len(rows),report=str(out/'review.json'))),flush=True)


if __name__=='__main__':main()
